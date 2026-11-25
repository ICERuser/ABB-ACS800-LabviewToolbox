/*
 * windemu.c – Wind turbine emulator DLL for LabVIEW (ABB ACS800 + RETA-01, Modbus/TCP)
 * Native C port of the Java emulator. Build: see build_windows.bat / build_mingw.sh
 */
#define WINDEMU_BUILD
#include "windemu.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* =============================================================================================
 *  Platform layer: sockets, threads, mutex, time
 * ===========================================================================================*/
#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <winsock2.h>
#  include <ws2tcpip.h>
#  include <windows.h>
#  include <mmsystem.h>
   typedef SOCKET sock_t;
#  define BAD_SOCK INVALID_SOCKET
#  define sock_close closesocket
   typedef CRITICAL_SECTION mutex_t;
#  define mutex_init(m)   InitializeCriticalSection(m)
#  define mutex_lock(m)   EnterCriticalSection(m)
#  define mutex_unlock(m) LeaveCriticalSection(m)
#  define mutex_free(m)   DeleteCriticalSection(m)
   typedef HANDLE thread_t;
static double now_s(void) {
    static LARGE_INTEGER f; LARGE_INTEGER c;
    if (!f.QuadPart) QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)f.QuadPart;
}
static void sleep_s(double s) { if (s > 0) Sleep((DWORD)(s * 1000.0 + 0.5)); }
static volatile LONG g_wsa_init = 0;
static void net_init(void) {
    if (InterlockedCompareExchange(&g_wsa_init, 1, 0) == 0) {
        WSADATA w; WSAStartup(MAKEWORD(2, 2), &w);
    }
}
#else
#  include <pthread.h>
#  include <time.h>
#  include <unistd.h>
#  include <errno.h>
#  include <fcntl.h>
#  include <netdb.h>
#  include <sys/socket.h>
#  include <sys/select.h>
#  include <netinet/in.h>
#  include <netinet/tcp.h>
#  include <arpa/inet.h>
   typedef int sock_t;
#  define BAD_SOCK (-1)
#  define sock_close close
   typedef pthread_mutex_t mutex_t;
#  define mutex_init(m)   pthread_mutex_init(m, NULL)
#  define mutex_lock(m)   pthread_mutex_lock(m)
#  define mutex_unlock(m) pthread_mutex_unlock(m)
#  define mutex_free(m)   pthread_mutex_destroy(m)
   typedef pthread_t thread_t;
static double now_s(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}
static void sleep_s(double s) {
    if (s <= 0) return;
    struct timespec t; t.tv_sec = (time_t)s; t.tv_nsec = (long)((s - (double)t.tv_sec) * 1e9);
    nanosleep(&t, NULL);
}
static void net_init(void) { }
#endif

static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static double sgn(double v) { return (v > 0) - (v < 0); }
static void copy_str(char *dst, int32_t len, const char *src) {
    if (!dst || len <= 0) return;
    strncpy(dst, src ? src : "", (size_t)len - 1);
    dst[len - 1] = 0;
}

/* =============================================================================================
 *  ABB Drives profile
 * ===========================================================================================*/
#define CW_OFF1      0x0001
#define CW_OFF2      0x0002
#define CW_OFF3      0x0004
#define CW_INH_OP    0x0008
#define CW_RAMP_OUT  0x0010
#define CW_RAMP_HOLD 0x0020
#define CW_RAMP_IN   0x0040
#define CW_RESET     0x0080
#define CW_REMOTE    0x0400
#define CW_EXT2      0x0800
#define CW_BASE      (CW_OFF2 | CW_OFF3 | CW_RAMP_OUT | CW_RAMP_HOLD | CW_RAMP_IN | CW_REMOTE)
#define CW_SHUTDOWN  (CW_BASE)                          /* 0x0476 */
#define CW_SWITCH_ON (CW_BASE | CW_OFF1)                /* 0x0477 */
#define CW_RUN       (CW_BASE | CW_OFF1 | CW_INH_OP)    /* 0x047F */
#define CW_RESET_CMD (CW_BASE | CW_RESET)               /* 0x04F6 */
#define CW_EMERGENCY (CW_BASE & ~CW_OFF3)               /* 0x0472 */

#define SW_RDY_ON     0x0001
#define SW_RDY_RUN    0x0002
#define SW_RDY_REF    0x0004
#define SW_TRIPPED    0x0008
#define SW_OFF2_STA   0x0010
#define SW_OFF3_STA   0x0020
#define SW_SWC_INHIB  0x0040
#define SW_ALARM      0x0080
#define SW_AT_SETP    0x0100
#define SW_REMOTE     0x0200
#define SW_ABOVE_LIM  0x0400
#define SW_EXT2       0x0800
#define SW_EXT_RUN_EN 0x1000

static const char *sw_state_name(int sw) {
    if (sw & SW_TRIPPED) return "FAULT";
    if (sw & SW_SWC_INHIB) return "SWITCH-ON INHIBITED";
    if (!(sw & SW_OFF2_STA)) return "OFF2 ACTIVE";
    if (!(sw & SW_OFF3_STA)) return "OFF3 ACTIVE";
    if (sw & SW_RDY_REF) return "OPERATING";
    if (sw & SW_RDY_RUN) return "READY TO OPERATE";
    if (sw & SW_RDY_ON) return "READY TO SWITCH ON";
    return "NOT READY TO SWITCH ON";
}

static int16_t to_s16(int v) { return (int16_t)(v & 0xFFFF); }
static int sat16(double v) { return (int)clampd(floor(v + 0.5), -32767, 32767); }

/* =============================================================================================
 *  Modbus/TCP master  (FC03, FC06, FC16, FC23 – the codes RETA-01 supports)
 * ===========================================================================================*/
typedef struct {
    sock_t s;
    int unit, timeoutMs;
    uint16_t tid;
    char err[160];
} Modbus;

static void mb_close(Modbus *m) { if (m->s != BAD_SOCK) { sock_close(m->s); m->s = BAD_SOCK; } }

static int mb_connect(Modbus *m, const char *host, int port) {
    char portStr[16];
    struct addrinfo hints, *res = NULL;
    mb_close(m);
    net_init();
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    snprintf(portStr, sizeof portStr, "%d", port);
    if (getaddrinfo(host, portStr, &hints, &res) != 0 || !res) {
        snprintf(m->err, sizeof m->err, "cannot resolve host '%s'", host);
        return -1;
    }
    sock_t s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == BAD_SOCK) { freeaddrinfo(res); snprintf(m->err, sizeof m->err, "socket() failed"); return -1; }

    /* non-blocking connect with timeout */
#ifdef _WIN32
    u_long nb = 1; ioctlsocket(s, FIONBIO, &nb);
#else
    int fl = fcntl(s, F_GETFL, 0); fcntl(s, F_SETFL, fl | O_NONBLOCK);
#endif
    connect(s, res->ai_addr, (int)res->ai_addrlen);
    freeaddrinfo(res);
    fd_set wf, ef; FD_ZERO(&wf); FD_ZERO(&ef); FD_SET(s, &wf); FD_SET(s, &ef);
    int ctoMs = m->timeoutMs < 1000 ? 1000 : m->timeoutMs;
    struct timeval tv = { ctoMs / 1000, (ctoMs % 1000) * 1000 };
    int r = select((int)s + 1, NULL, &wf, &ef, &tv);
    int soerr = 0; socklen_t sl = sizeof soerr;
    if (r > 0) getsockopt(s, SOL_SOCKET, SO_ERROR, (char *)&soerr, &sl);
    if (r <= 0 || soerr != 0 || !FD_ISSET(s, &wf)) {
        sock_close(s);
        snprintf(m->err, sizeof m->err, r == 0 ? "connect timed out (%s:%d)" : "connection refused (%s:%d)", host, port);
        return -1;
    }
#ifdef _WIN32
    nb = 0; ioctlsocket(s, FIONBIO, &nb);
    DWORD to = (DWORD)m->timeoutMs;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char *)&to, sizeof to);
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, (const char *)&to, sizeof to);
#else
    fcntl(s, F_SETFL, fl);
    struct timeval rto = { m->timeoutMs / 1000, (m->timeoutMs % 1000) * 1000 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &rto, sizeof rto);
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &rto, sizeof rto);
#endif
    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof one);
    m->s = s;
    return 0;
}

static int recv_all(sock_t s, uint8_t *b, int n) {
    int got = 0;
    while (got < n) {
        int r = recv(s, (char *)b + got, n - got, 0);
        if (r <= 0) return -1;
        got += r;
    }
    return 0;
}

/* Send PDU, receive response PDU into resp (max 256). Returns response length or -1. */
static int mb_transact(Modbus *m, const uint8_t *pdu, int pduLen, uint8_t *resp) {
    uint8_t adu[270], hdr[7];
    if (m->s == BAD_SOCK) { snprintf(m->err, sizeof m->err, "not connected"); return -1; }
    m->tid++;
    adu[0] = (uint8_t)(m->tid >> 8); adu[1] = (uint8_t)m->tid;
    adu[2] = 0; adu[3] = 0;
    adu[4] = (uint8_t)((pduLen + 1) >> 8); adu[5] = (uint8_t)(pduLen + 1);
    adu[6] = (uint8_t)m->unit;
    memcpy(adu + 7, pdu, (size_t)pduLen);
    /* the whole ADU in one send(): embedded stacks dislike fragmented frames */
    if (send(m->s, (const char *)adu, 7 + pduLen, 0) != 7 + pduLen) {
        snprintf(m->err, sizeof m->err, "send failed"); return -1;
    }
    for (;;) {
        if (recv_all(m->s, hdr, 7) != 0) { snprintf(m->err, sizeof m->err, "receive timeout"); return -1; }
        int len = (hdr[4] << 8) | hdr[5];
        if (len < 2 || len > 254) { snprintf(m->err, sizeof m->err, "bad MBAP length %d", len); return -1; }
        if (recv_all(m->s, resp, len - 1) != 0) { snprintf(m->err, sizeof m->err, "receive timeout"); return -1; }
        uint16_t rtid = (uint16_t)((hdr[0] << 8) | hdr[1]);
        if (rtid != m->tid) continue;                       /* stale answer – discard */
        if (resp[0] & 0x80) {
            int code = resp[1];
            snprintf(m->err, sizeof m->err, "Modbus exception %d%s", code,
                     code == 1 ? " (illegal function)" : code == 2 ? " (illegal data address)" :
                     code == 3 ? " (illegal data value)" : "");
            return -1;
        }
        if (resp[0] != pdu[0]) { snprintf(m->err, sizeof m->err, "function code mismatch"); return -1; }
        return len - 1;
    }
}

static int mb_parse_regs(Modbus *m, const uint8_t *r, int rlen, int count, int *out) {
    if (rlen < 2 + 2 * count || r[1] != 2 * count) { snprintf(m->err, sizeof m->err, "unexpected byte count"); return -1; }
    for (int i = 0; i < count; i++) out[i] = (r[2 + 2 * i] << 8) | r[3 + 2 * i];
    return 0;
}

static int mb_read(Modbus *m, int addr, int count, int *out) {
    uint8_t p[5] = { 0x03, (uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)(count >> 8), (uint8_t)count }, r[256];
    int n = mb_transact(m, p, 5, r);
    return n < 0 ? -1 : mb_parse_regs(m, r, n, count, out);
}

static int mb_write_multi(Modbus *m, int addr, const int *v, int count) {
    uint8_t p[256], r[256];
    p[0] = 0x10; p[1] = (uint8_t)(addr >> 8); p[2] = (uint8_t)addr;
    p[3] = (uint8_t)(count >> 8); p[4] = (uint8_t)count; p[5] = (uint8_t)(2 * count);
    for (int i = 0; i < count; i++) { p[6 + 2 * i] = (uint8_t)(v[i] >> 8); p[7 + 2 * i] = (uint8_t)v[i]; }
    return mb_transact(m, p, 6 + 2 * count, r) < 0 ? -1 : 0;
}

static int mb_read_write(Modbus *m, int raddr, int rcount, int waddr, const int *v, int wcount, int *out) {
    uint8_t p[256], r[256];
    p[0] = 0x17;
    p[1] = (uint8_t)(raddr >> 8); p[2] = (uint8_t)raddr; p[3] = (uint8_t)(rcount >> 8); p[4] = (uint8_t)rcount;
    p[5] = (uint8_t)(waddr >> 8); p[6] = (uint8_t)waddr; p[7] = (uint8_t)(wcount >> 8); p[8] = (uint8_t)wcount;
    p[9] = (uint8_t)(2 * wcount);
    for (int i = 0; i < wcount; i++) { p[10 + 2 * i] = (uint8_t)(v[i] >> 8); p[11 + 2 * i] = (uint8_t)v[i]; }
    int n = mb_transact(m, p, 10 + 2 * wcount, r);
    return n < 0 ? -1 : mb_parse_regs(m, r, n, rcount, out);
}

/* =============================================================================================
 *  Turbine models
 * ===========================================================================================*/
/* Heier / Slootweg Cp(λ,β) */
static double cp_model(double lambda, double beta) {
    double l = lambda < 1e-3 ? 1e-3 : lambda;
    double inv = 1.0 / (l + 0.08 * beta) - 0.035 / (beta * beta * beta + 1.0);
    double cp = 0.5176 * (116.0 * inv - 0.4 * beta - 5.0) * exp(-21.0 * inv) + 0.0068 * l;
    return clampd(cp, -0.2, 0.593);
}
/* Cq = Cp/λ, held at its λ = 1 value below λ = 1 (finite start-up torque) */
static double cq_model(double lambda, double beta) {
    double l = lambda < 1.0 ? 1.0 : lambda;
    return cp_model(l, beta) / l;
}

typedef struct {
    double R, rho, Prated, wRated, J, G, kp, Jemu, lambdaOpt, cpMax, kOpt, Trated, Tnom, shadow;
    int blades;
} Turbine;

static void turbine_init(Turbine *t, const WE_Config *c) {
    t->R = c->rotorRadiusM;
    t->rho = c->airDensity;
    t->Prated = c->turbineRatedPowerW;
    t->wRated = c->turbineRatedRotorRpm * M_PI / 30.0;
    t->J = c->rotorInertiaKgm2;
    t->G = c->motorNomRpm / c->turbineRatedRotorRpm;
    t->kp = c->powerUsage * c->motorNomPowerW / c->turbineRatedPowerW;
    t->Jemu = t->J * t->kp / (t->G * t->G);
    t->Tnom = c->motorNomPowerW / (c->motorNomRpm * M_PI / 30.0);
    t->shadow = c->towerShadowAmplitude;
    t->blades = c->blades > 0 ? c->blades : 3;
    t->lambdaOpt = 0; t->cpMax = -1;
    for (double l = 1; l <= 15; l += 0.01) {
        double v = cp_model(l, 0);
        if (v > t->cpMax) { t->cpMax = v; t->lambdaOpt = l; }
    }
    t->kOpt = 0.5 * t->rho * M_PI * pow(t->R, 5) * t->cpMax / pow(t->lambdaOpt, 3);
    t->Trated = t->Prated / t->wRated;
}
static double tsr(const Turbine *t, double v, double w) { return v < 0.1 ? 0 : w * t->R / v; }
static double aero_torque(const Turbine *t, double v, double w, double beta) {
    if (v < 0.1) return 0;
    double lambda = tsr(t, v, w > 0 ? w : 0);
    double T = 0.5 * t->rho * M_PI * pow(t->R, 3) * v * v * cq_model(lambda, beta);
    if (T < 0) T *= clampd((w > 0 ? w : 0) / (0.05 * t->wRated), 0, 1);
    return T;
}
static double shadow_factor(const Turbine *t, double angle) {
    if (t->shadow <= 0) return 1;
    double c = 0.5 + 0.5 * cos(t->blades * angle);
    return 1 - t->shadow * pow(c, 6);
}
static double r2m_T(const Turbine *t, double T) { return T * t->kp / t->G; }
static double m2r_T(const Turbine *t, double T) { return T * t->G / t->kp; }
static double gen_torque_motor(const Turbine *t, double wm) {
    double wr = wm / t->G;
    if (wr <= 0) return 0;
    double T = t->kOpt * wr * wr;
    if (T > t->Trated) T = t->Trated;
    return r2m_T(t, T);
}

/* random numbers (xorshift64* + Box-Muller) */
static double rnd_uniform(uint64_t *s) {
    *s ^= *s >> 12; *s ^= *s << 25; *s ^= *s >> 27;
    return ((*s * 2685821657736338717ULL) >> 11) * (1.0 / 9007199254740992.0);
}
static double rnd_gauss(uint64_t *s) {
    double u1 = rnd_uniform(s), u2 = rnd_uniform(s);
    if (u1 < 1e-300) u1 = 1e-300;
    return sqrt(-2.0 * log(u1)) * cos(2 * M_PI * u2);
}

/* =============================================================================================
 *  Simulated drive (offline ACS800 + bench)
 * ===========================================================================================*/
typedef struct {
    int tripped, inhibited, readyOn, operating, prevReset, remote, ext2, braking;
    double w, T, speedInt;
} SimDrive;

static void sim_cw(SimDrive *d, int cw) {
    d->remote = (cw & CW_REMOTE) != 0;
    d->ext2 = (cw & CW_EXT2) != 0;
    int reset = (cw & CW_RESET) != 0;
    if (reset && !d->prevReset && d->tripped) { d->tripped = 0; d->inhibited = 1; }
    d->prevReset = reset;
    if (d->tripped || !d->remote) { d->operating = 0; d->readyOn = 0; return; }
    if (!(cw & CW_OFF2)) { d->operating = 0; d->readyOn = 0; d->inhibited = 1; d->braking = 0; return; }
    if (!(cw & CW_OFF3)) { if (d->operating) d->braking = 1; d->operating = 0; d->readyOn = 0; d->inhibited = 1; return; }
    if (!(cw & CW_OFF1)) { if (d->operating) d->braking = 1; d->inhibited = 0; d->readyOn = 1; d->operating = 0; return; }
    if (d->inhibited) return;
    d->operating = d->readyOn && (cw & CW_INH_OP);
}

static void sim_step(SimDrive *d, const WE_Config *c, const Turbine *t, double h, int ref1, int ref2) {
    double tLim = t->Tnom * c->maxTorquePct / 100.0, tCmd = 0;
    if (d->operating) {
        int torqueMode = c->torqueOnExt2 ? d->ext2 : (c->controlMode == 1);
        if (torqueMode) {
            tCmd = to_s16(ref2) / 10000.0 * c->ref2FullScalePct / 100.0 * t->Tnom;
        } else {
            double wRef = to_s16(ref1) / 20000.0 * c->ref1FullScaleRpm * M_PI / 30;
            double e = wRef - d->w, J = c->rigInertiaKgm2;
            d->speedInt = clampd(d->speedInt + J * 600 * e * h, -tLim, tLim);
            tCmd = J * 60 * e + d->speedInt;
        }
        tCmd = clampd(tCmd, -tLim, tLim);
        d->T += (tCmd - d->T) * (h / 0.002 < 1 ? h / 0.002 : 1);
    } else if (d->braking && !d->tripped) {
        d->T = -sgn(d->w) * tLim * 0.5;
        if (fabs(d->w) < 0.5) { d->braking = 0; d->T = 0; d->w = 0; }
        d->speedInt = 0;
    } else { d->T = 0; d->speedInt = 0; }
    double tGen = gen_torque_motor(t, d->w);
    d->w += (d->T - tGen - c->rigViscousNmPerRadS * d->w) / c->rigInertiaKgm2 * h;
    if (!d->operating && fabs(d->w) < 0.05 && tGen < 1e-9) d->w = 0;
    if (fabs(d->w * 30 / M_PI) > c->maxMotorRpm * 1.15 && !d->tripped) { d->tripped = 1; d->operating = 0; }
}

static int sim_sw(const SimDrive *d) {
    int sw = SW_OFF2_STA | SW_OFF3_STA;
    if (d->tripped) sw |= SW_TRIPPED;
    if (d->inhibited) sw |= SW_SWC_INHIB;
    if (d->readyOn && !d->tripped) sw |= SW_RDY_ON;
    if (d->operating) sw |= SW_RDY_RUN | SW_RDY_REF | SW_AT_SETP;
    if (d->remote) sw |= SW_REMOTE | SW_EXT_RUN_EN;
    if (d->ext2) sw |= SW_EXT2;
    return sw;
}

/* =============================================================================================
 *  Emulator instance
 * ===========================================================================================*/
#define MAX_INST 8
#define CMDQ 32

typedef struct {
    int used;
    WE_Config cfg;
    char host[128];
    Turbine tb;
    mutex_t mtx;
    thread_t thread;
    volatile int shutdown;

    /* shared (under mtx) */
    int cmdq[CMDQ], cmdHead, cmdTail;
    WE_Inputs in;
    WE_Telemetry tel;
    char msg[256];
    double lastPoll;
    double *profT, *profV; int profN;

    /* real-time state (thread only) */
    int state;
    double stateTimer, time;
    Modbus mb;
    int connected;
    SimDrive sim;
    double simLast;
    int sw, act1, act2;
    int commErrors, resetCycles;
    double wPrev, accF, wVirt, angle, Tref;
    double turb, gustStart, gustDur, gustAmp, profTime;
    double beta, pitchInt, wSp;
    uint64_t rng;
} Inst;

static Inst g_inst[MAX_INST];
static mutex_t g_mtx;
static int g_mtx_init;

#ifdef _WIN32
static INIT_ONCE g_once = INIT_ONCE_STATIC_INIT;
static BOOL CALLBACK init_once_cb(PINIT_ONCE o, PVOID p, PVOID *c) { (void)o; (void)p; (void)c; mutex_init(&g_mtx); g_mtx_init = 1; return TRUE; }
static void global_init(void) { InitOnceExecuteOnce(&g_once, init_once_cb, NULL, NULL); }
#else
static pthread_once_t g_once = PTHREAD_ONCE_INIT;
static void init_once_cb(void) { mutex_init(&g_mtx); g_mtx_init = 1; }
static void global_init(void) { pthread_once(&g_once, init_once_cb); }
#endif

static Inst *get_inst(int32_t h) {
    if (h < 1 || h > MAX_INST) return NULL;
    Inst *e = &g_inst[h - 1];
    return e->used ? e : NULL;
}

static void set_msg(Inst *e, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    mutex_lock(&e->mtx);
    vsnprintf(e->msg, sizeof e->msg, fmt, ap);
    mutex_unlock(&e->mtx);
    va_end(ap);
}

static const char *state_names[] = { "DISCONNECTED", "IDLE", "STARTING", "RUNNING", "STOPPING", "ESTOP", "FAULT", "COMM_LOST" };

static void enter(Inst *e, int s, const char *msg) {
    e->state = s;
    e->stateTimer = 0;
    set_msg(e, "%s", msg);
}

/* ---- drive I/O ------------------------------------------------------------------------------*/
static int drive_exchange(Inst *e, int cw, int ref1, int ref2) {
    const WE_Config *c = &e->cfg;
    if (c->simulate) {
        double now = now_s();
        double dt = e->simLast == 0 ? 0 : now - e->simLast;
        if (dt > 0.2) dt = 0.2;
        e->simLast = now;
        sim_cw(&e->sim, cw);
        int steps = (int)ceil(dt / 0.001); if (steps < 1) steps = 1;
        for (int i = 0; i < steps && dt > 0; i++) sim_step(&e->sim, c, &e->tb, dt / steps, ref1, ref2);
        e->sw = sim_sw(&e->sim);
        e->act1 = sat16(e->sim.w * 30 / M_PI / c->act1FullScaleRpm * 20000);
        e->act2 = sat16(e->sim.T / e->tb.Tnom * 100 / c->act2FullScalePct * 10000);
        return 0;
    }
    int w[3] = { cw & 0xFFFF, ref1 & 0xFFFF, ref2 & 0xFFFF }, r[3];
    int cwAddr = c->registerBase + c->regControlWord - 1;
    int swAddr = c->registerBase + c->regStatusWord - 1;
    int rc = c->useFc23 ? mb_read_write(&e->mb, swAddr, 3, cwAddr, w, 3, r)
                        : (mb_write_multi(&e->mb, cwAddr, w, 3) == 0 ? mb_read(&e->mb, swAddr, 3, r) : -1);
    if (rc != 0) return -1;
    e->sw = r[0]; e->act1 = to_s16(r[1]); e->act2 = to_s16(r[2]);
    return 0;
}

static void do_connect(Inst *e) {
    const WE_Config *c = &e->cfg;
    e->commErrors = 0;
    if (c->simulate) {
        memset(&e->sim, 0, sizeof e->sim);
        e->sim.inhibited = 1;
        e->simLast = 0;
    } else if (mb_connect(&e->mb, e->host, c->port) != 0) {
        e->connected = 0;
        char b[256]; snprintf(b, sizeof b, "Connect failed: %s", e->mb.err);
        enter(e, WE_DISCONNECTED, b);
        return;
    }
    e->connected = 1;
    if (drive_exchange(e, CW_SHUTDOWN, 0, 0) != 0) {
        char b[256]; snprintf(b, sizeof b, "Connected but no answer: %s", e->mb.err);
        mb_close(&e->mb); e->connected = 0;
        enter(e, WE_DISCONNECTED, b);
        return;
    }
    char b[256];
    if (c->simulate) snprintf(b, sizeof b, "Connected to SIMULATED ACS800 (offline)");
    else snprintf(b, sizeof b, "Connected to RETA-01 Modbus/TCP %s:%d", e->host, c->port);
    enter(e, (e->sw & SW_TRIPPED) ? WE_FAULT : WE_IDLE, b);
}

static void do_disconnect(Inst *e) {
    if (e->connected) drive_exchange(e, CW_SHUTDOWN, 0, 0);
    mb_close(&e->mb);
    e->connected = 0;
    e->sw = e->act1 = e->act2 = 0;
    enter(e, WE_DISCONNECTED, "Disconnected");
}

/* ---- commands -------------------------------------------------------------------------------*/
static void handle_command(Inst *e, int cmd, double wMotor) {
    switch (cmd) {
    case WE_CMD_CONNECT: do_connect(e); break;
    case WE_CMD_DISCONNECT: do_disconnect(e); break;
    case WE_CMD_START:
        if (e->state == WE_IDLE) enter(e, WE_STARTING, "Starting: 0x0476 -> 0x047F");
        break;
    case WE_CMD_STOP:
        if (e->state == WE_RUNNING) enter(e, WE_STOPPING, "Stopping: feathering blades");
        else if (e->state == WE_STARTING) enter(e, WE_IDLE, "Start aborted");
        break;
    case WE_CMD_ESTOP:
        if (e->state != WE_DISCONNECTED && e->state != WE_COMM_LOST)
            enter(e, WE_ESTOP, "EMERGENCY STOP (OFF3) - reset to recover");
        break;
    case WE_CMD_RESET:
        if (e->state == WE_ESTOP) {
            if (fabs(wMotor) < 1) enter(e, WE_FAULT, "Resetting...");
            else { set_msg(e, "Wait for standstill before reset"); break; }
        }
        if (e->state == WE_FAULT) e->resetCycles = 8;
        if (e->state == WE_COMM_LOST) do_connect(e);
        break;
    case WE_CMD_GUST: e->gustAmp = 5; e->gustDur = 10.5; e->gustStart = e->time; break;
    case WE_CMD_LULL: e->gustAmp = -3; e->gustDur = 8; e->gustStart = e->time; break;
    }
}

/* ---- ABB Drives state machine -> control word --------------------------------------------------*/
static int state_machine(Inst *e, double wMotor) {
    int sw = e->sw;
    char b[200];
    switch (e->state) {
    case WE_IDLE:
        if (sw & SW_TRIPPED) { snprintf(b, sizeof b, "Drive fault (status 0x%04X)", sw); enter(e, WE_FAULT, b); }
        return CW_SHUTDOWN;
    case WE_STARTING:
        if (sw & SW_TRIPPED) { enter(e, WE_FAULT, "Drive tripped during start"); return CW_SHUTDOWN; }
        if (!(sw & SW_REMOTE) && e->stateTimer > 1.0) {
            enter(e, WE_FAULT, "Drive not in REMOTE - check 98.02 COMM. MODULE LINK and LOC/REM key");
            return CW_SHUTDOWN;
        }
        if (e->stateTimer > 8.0) { snprintf(b, sizeof b, "Start timeout, drive state %s", sw_state_name(sw)); enter(e, WE_FAULT, b); return CW_SHUTDOWN; }
        if (sw & SW_RDY_REF) { enter(e, WE_RUNNING, "Emulating turbine"); return CW_RUN; }
        if ((sw & SW_RDY_ON) && !(sw & SW_SWC_INHIB)) return CW_RUN;
        return CW_SHUTDOWN;
    case WE_RUNNING:
        if (sw & SW_TRIPPED) { snprintf(b, sizeof b, "Drive fault while running (0x%04X)", sw); enter(e, WE_FAULT, b); return CW_SHUTDOWN; }
        if (!(sw & SW_RDY_REF) && e->stateTimer > 0.5) { snprintf(b, sizeof b, "Drive left OPERATING: %s", sw_state_name(sw)); enter(e, WE_FAULT, b); return CW_SHUTDOWN; }
        return CW_RUN;
    case WE_STOPPING: {
        if (sw & SW_TRIPPED) { enter(e, WE_FAULT, "Drive fault while stopping"); return CW_SHUTDOWN; }
        double pu = fabs(wMotor) / (e->tb.wRated * e->tb.G);
        if (pu < 0.05 || e->stateTimer > 60) { enter(e, WE_IDLE, "Stopped (OFF1)"); return CW_SHUTDOWN; }
        return CW_RUN;
    }
    case WE_ESTOP:
        return CW_EMERGENCY;
    case WE_FAULT:
        if (e->resetCycles > 0) {
            e->resetCycles--;
            if (e->resetCycles == 0) {
                if (sw & SW_TRIPPED) set_msg(e, "Reset sent but drive still tripped - check drive panel");
                else enter(e, WE_IDLE, "Fault reset");
            }
            return e->resetCycles > 3 ? CW_SHUTDOWN : CW_RESET_CMD;
        }
        return CW_SHUTDOWN;
    default:
        return CW_SHUTDOWN;
    }
}

/* ---- wind -------------------------------------------------------------------------------------*/
static double profile_value(Inst *e, double t) {
    double end = e->profT[e->profN - 1];
    if (end > 0) t = fmod(t, end);
    for (int i = 1; i < e->profN; i++) {
        if (t <= e->profT[i]) {
            double span = e->profT[i] - e->profT[i - 1];
            double f = span > 1e-9 ? (t - e->profT[i - 1]) / span : 0;
            return e->profV[i - 1] + f * (e->profV[i] - e->profV[i - 1]);
        }
    }
    return e->profV[e->profN - 1];
}

static double wind_step(Inst *e, const WE_Inputs *in, double dt) {
    double mean = in->windMeanSpeed;
    mutex_lock(&e->mtx);
    if (in->useWindProfile && e->profN > 0) { e->profTime += dt; mean = profile_value(e, e->profTime); }
    else e->profTime = 0;
    mutex_unlock(&e->mtx);
    double sigma = in->turbulenceIntensity * (mean > 0.5 ? mean : 0.5);
    double tau = e->cfg.turbulenceLengthScaleM / (mean > 1 ? mean : 1);
    double a = exp(-dt / tau);
    e->turb = a * e->turb + sigma * sqrt(1 - a * a) * rnd_gauss(&e->rng);
    double gust = 0;
    if (e->gustStart >= 0) {
        double tg = e->time - e->gustStart;
        if (tg <= e->gustDur) gust = e->gustAmp * 0.5 * (1 - cos(2 * M_PI * tg / e->gustDur));
        else e->gustStart = -1;
    }
    double v = mean + e->turb + gust;
    return v > 0 ? v : 0;
}

#define PITCH_GS_DEG 10.0  /* pitch at which the PI gains are halved (gain scheduling) */
/* ---- pitch ------------------------------------------------------------------------------------*/
static double pitch_step(Inst *e, const WE_Inputs *in, double dt, double wRotor, int feather) {
    const WE_Config *c = &e->cfg;
    double target;
    if (feather) {
        target = c->pitchMaxDeg; e->pitchInt = 0;
        e->wSp = wRotor > 0 ? wRotor : 0;          /* start-up ramp begins at the actual speed */
    }
    else if (in->manualPitch) { target = clampd(in->manualPitchDeg, 0, c->pitchMaxDeg); e->pitchInt = target; }
    else {
        /* speed set-point ramps to rated (start-up) – prevents the overshoot of a step to rated */
        double rampRate = e->tb.wRated / (c->pitchStartRampS > 0.1 ? c->pitchStartRampS : 0.1);
        e->wSp = e->wSp + rampRate * dt;
        /* lead the actual speed by at most 5 %: limits rotor acceleration to the ramp rate */
        if (e->wSp > wRotor + 0.05 * e->tb.wRated) e->wSp = wRotor + 0.05 * e->tb.wRated;
        if (e->wSp > e->tb.wRated) e->wSp = e->tb.wRated;
        double err = (wRotor - e->wSp) / e->tb.wRated;
        /* gain scheduling: aerodynamic sensitivity dP/dβ grows with pitch -> reduce gains */
        double gs = 1.0 / (1.0 + e->beta / PITCH_GS_DEG);
        e->pitchInt = clampd(e->pitchInt + gs * c->pitchKiDegPerSPerPu * err * dt, 0, c->pitchMaxDeg);
        target = clampd(e->pitchInt + gs * c->pitchKpDegPerPu * err, 0, c->pitchMaxDeg);
    }
    double step = c->pitchRateLimitDegPerS * dt;
    e->beta += clampd(target - e->beta, -step, step);
    return e->beta;
}

/* ---- one real-time cycle ----------------------------------------------------------------------*/
static void cycle(Inst *e, double dt) {
    const WE_Config *c = &e->cfg;
    const Turbine *t = &e->tb;
    WE_Inputs in;
    int cmds[CMDQ], ncmd = 0;

    e->time += dt;
    e->stateTimer += dt;

    mutex_lock(&e->mtx);
    in = e->in;
    while (e->cmdTail != e->cmdHead && ncmd < CMDQ) { cmds[ncmd++] = e->cmdq[e->cmdTail]; e->cmdTail = (e->cmdTail + 1) % CMDQ; }
    double lastPoll = e->lastPoll;
    mutex_unlock(&e->mtx);

    double wMotor = e->act1 / 20000.0 * c->act1FullScaleRpm * M_PI / 30;
    for (int i = 0; i < ncmd; i++) handle_command(e, cmds[i], wMotor);

    /* host watchdog: LabVIEW stopped polling (VI aborted / hung) -> controlled stop */
    if (c->hostWatchdogMs > 0 && (e->state == WE_RUNNING || e->state == WE_STARTING) &&
        now_s() - lastPoll > c->hostWatchdogMs / 1000.0) {
        enter(e, e->state == WE_RUNNING ? WE_STOPPING : WE_IDLE, "Host watchdog: LabVIEW stopped polling - stopping");
    }

    double v = wind_step(e, &in, dt);

    double accRaw = (wMotor - e->wPrev) / dt;
    e->wPrev = wMotor;
    double k = dt / c->accelFilterTauS; if (k > 1) k = 1;
    e->accF += (accRaw - e->accF) * k;
    double tMeas = e->act2 / 10000.0 * c->act2FullScalePct / 100.0 * t->Tnom;

    int active = e->state == WE_RUNNING || e->state == WE_STOPPING;
    int torqueMode = c->controlMode == 1;
    if (!active) e->wVirt = wMotor / t->G;
    double wRotor = torqueMode ? wMotor / t->G : e->wVirt;

    int feather = e->state != WE_RUNNING || v > c->cutOutWind;
    double beta = pitch_step(e, &in, dt, wRotor, feather);
    e->angle = fmod(e->angle + wRotor * dt, 2 * M_PI);
    double tAero = aero_torque(t, v, wRotor, beta) * shadow_factor(t, e->angle);
    double tAeroM = r2m_T(t, tAero);
    double tGenEst = tMeas - c->rigInertiaKgm2 * e->accF - c->rigViscousNmPerRadS * wMotor;

    double tCmd = 0, wCmd = 0;
    if (active) {
        if (torqueMode) {
            double jRig = c->rigInertiaKgm2;
            double jEmu = jRig + c->inertiaCompGain * (t->Jemu - jRig > 0 ? t->Jemu - jRig : 0);
            double r = jRig / jEmu; if (r > 1) r = 1;
            tCmd = tGenEst + c->rigViscousNmPerRadS * wMotor + r * (tAeroM - tGenEst);
        } else {
            e->wVirt += (tAero - m2r_T(t, tGenEst)) / t->J * dt;
            if (e->wVirt < 0) e->wVirt = 0;
            wCmd = e->wVirt * t->G;
        }
    }
    double tMax = t->Tnom * c->maxTorquePct / 100.0;
    tCmd = clampd(tCmd, -tMax, tMax);
    double ramp = t->Tnom * c->torqueRampPctPerS / 100.0 * dt;
    e->Tref += clampd(tCmd - e->Tref, -ramp, ramp);
    if (!active) e->Tref = 0;

    int cw = state_machine(e, wMotor);
    if ((cw == CW_RUN || cw == CW_SWITCH_ON) && torqueMode && c->torqueOnExt2) cw |= CW_EXT2;

    int ref1 = 0, ref2 = 0;
    if (active) {
        if (torqueMode) ref2 = sat16(e->Tref / t->Tnom * 100 / c->ref2FullScalePct * 10000);
        else {
            double rpm = wCmd * 30 / M_PI;
            if (rpm > c->maxMotorRpm) rpm = c->maxMotorRpm;
            ref1 = sat16(rpm / c->ref1FullScaleRpm * 20000);
        }
    }

    double motorRpm = wMotor * 30 / M_PI;
    if (fabs(motorRpm) > c->maxMotorRpm && e->state != WE_ESTOP && e->state != WE_DISCONNECTED) {
        char b[160]; snprintf(b, sizeof b, "OVERSPEED %.0f rpm > %.0f rpm - OFF3 issued", motorRpm, c->maxMotorRpm);
        enter(e, WE_ESTOP, b);
        cw = CW_EMERGENCY; ref1 = ref2 = 0;
    }

    if (e->connected && e->state != WE_DISCONNECTED && e->state != WE_COMM_LOST) {
        if (drive_exchange(e, cw, ref1, ref2) == 0) e->commErrors = 0;
        else if (++e->commErrors >= c->maxCommErrors) {
            char b[256]; snprintf(b, sizeof b, "Communication lost: %s - drive stops on its fieldbus timeout", e->mb.err);
            mb_close(&e->mb); e->connected = 0;
            enter(e, WE_COMM_LOST, b);
        }
    }

    /* publish */
    WE_Telemetry tl;
    double lambda = tsr(t, v, wRotor);
    tl.timeS = e->time;
    tl.windMs = v;
    tl.pitchDeg = beta;
    tl.lambda = lambda;
    tl.cp = cp_model(lambda, beta);
    tl.rotorRpm = wRotor * 30 / M_PI;
    tl.aeroTorqueKNm = tAero / 1e3;
    tl.aeroPowerKW = tAero * wRotor / 1e3;
    tl.motorRpm = motorRpm;
    tl.torqueRefPct = (torqueMode ? e->Tref : (active ? tAeroM : 0)) / t->Tnom * 100;
    tl.torqueActPct = tMeas / t->Tnom * 100;
    tl.speedRefRpm = wCmd * 30 / M_PI;
    tl.genTorqueEstPct = tGenEst / t->Tnom * 100;
    tl.genTorqueRecPct = gen_torque_motor(t, wMotor) / t->Tnom * 100;
    tl.cycleMs = 0;
    tl.state = e->state;
    tl.controlWord = cw;
    tl.statusWord = e->sw;
    tl.ref1 = to_s16(ref1);
    tl.ref2 = to_s16(ref2);
    tl.act1 = e->act1;
    tl.act2 = e->act2;
    tl.commErrors = e->commErrors;
    tl.driveTripped = (e->sw & SW_TRIPPED) != 0;
    tl.driveRemote = (e->sw & SW_REMOTE) != 0;
    mutex_lock(&e->mtx);
    e->tel = tl;
    mutex_unlock(&e->mtx);
}

#ifdef _WIN32
static DWORD WINAPI rt_thread(LPVOID arg)
#else
static void *rt_thread(void *arg)
#endif
{
    Inst *e = (Inst *)arg;
#ifdef _WIN32
    timeBeginPeriod(1);
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
#endif
    double period = (e->cfg.periodMs > 0 ? e->cfg.periodMs : 20) / 1000.0;
    double next = now_s(), prev = next;
    while (!e->shutdown) {
        double t0 = now_s();
        double dt = clampd(t0 - prev, 1e-4, 0.25);
        prev = t0;
        cycle(e, dt);
        double ms = (now_s() - t0) * 1000.0;
        mutex_lock(&e->mtx); e->tel.cycleMs = ms; mutex_unlock(&e->mtx);
        next += period;
        double sl = next - now_s();
        if (sl > 0) sleep_s(sl); else next = now_s();
    }
    if (e->connected) drive_exchange(e, CW_SHUTDOWN, 0, 0);   /* leave the drive safe */
    mb_close(&e->mb);
    e->connected = 0;
#ifdef _WIN32
    timeEndPeriod(1);
    return 0;
#else
    return NULL;
#endif
}

/* =============================================================================================
 *  Properties table (key <-> cluster field)
 * ===========================================================================================*/
typedef struct { const char *key; int isInt; size_t off; } Prop;
#define PD(k, f) { k, 0, offsetof(WE_Config, f) }
#define PI_(k, f) { k, 1, offsetof(WE_Config, f) }
static const Prop g_props[] = {
    PD("motor.nominalPowerW", motorNomPowerW), PD("motor.nominalRpm", motorNomRpm),
    PD("motor.rigInertiaKgm2", rigInertiaKgm2), PD("motor.rigViscousNmPerRadS", rigViscousNmPerRadS),
    PD("drive.ref1FullScaleRpm", ref1FullScaleRpm), PD("drive.ref2FullScalePct", ref2FullScalePct),
    PD("drive.act1FullScaleRpm", act1FullScaleRpm), PD("drive.act2FullScalePct", act2FullScalePct),
    PD("turbine.rotorRadiusM", rotorRadiusM), PD("turbine.airDensity", airDensity),
    PD("turbine.ratedPowerW", turbineRatedPowerW), PD("turbine.ratedRotorRpm", turbineRatedRotorRpm),
    PD("turbine.rotorInertiaKgm2", rotorInertiaKgm2), PD("turbine.cutInWind", cutInWind),
    PD("turbine.cutOutWind", cutOutWind), PD("turbine.towerShadowAmplitude", towerShadowAmplitude),
    PD("scaling.powerUsage", powerUsage), PD("scaling.inertiaCompensationGain", inertiaCompGain),
    PD("scaling.accelFilterTauS", accelFilterTauS),
    PD("pitch.kpDegPerPu", pitchKpDegPerPu), PD("pitch.kiDegPerSPerPu", pitchKiDegPerSPerPu),
    PD("pitch.rateLimitDegPerS", pitchRateLimitDegPerS), PD("pitch.maxDeg", pitchMaxDeg),
    PD("wind.meanSpeed", windMeanSpeed), PD("wind.turbulenceIntensity", turbulenceIntensity),
    PD("wind.turbulenceLengthScaleM", turbulenceLengthScaleM),
    PD("safety.maxMotorRpm", maxMotorRpm), PD("safety.maxTorquePct", maxTorquePct),
    PD("safety.torqueRampPctPerS", torqueRampPctPerS), PD("pitch.startRampS", pitchStartRampS),
    PI_("drive.port", port), PI_("drive.unitId", unitId), PI_("drive.timeoutMs", timeoutMs),
    PI_("drive.registerBase", registerBase), PI_("drive.reg.controlWord", regControlWord),
    PI_("drive.reg.statusWord", regStatusWord), PI_("drive.useFc23", useFc23),
    PI_("drive.simulate", simulate), PI_("emulator.controlMode", controlMode),
    PI_("drive.torqueOnExt2", torqueOnExt2), PI_("turbine.blades", blades),
    PI_("loop.periodMs", periodMs), PI_("safety.maxConsecutiveCommErrors", maxCommErrors),
    PI_("host.watchdogMs", hostWatchdogMs),
};
#define NPROPS ((int)(sizeof g_props / sizeof g_props[0]))

static char *trim(char *s) {
    while (isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
    return s;
}
static int parse_int_value(const char *v) {
    if (!strcmp(v, "true") || !strcmp(v, "TRUE")) return 1;
    if (!strcmp(v, "false") || !strcmp(v, "FALSE")) return 0;
    if (!strcmp(v, "TORQUE") || !strcmp(v, "torque")) return 1;
    if (!strcmp(v, "SPEED") || !strcmp(v, "speed")) return 0;
    return (int)strtol(v, NULL, 0);
}

/* =============================================================================================
 *  Exported API
 * ===========================================================================================*/
WE_API int32_t WE_CALL WE_Version(void) { return 0x00010000; }

WE_API int32_t WE_CALL WE_GetStructSizes(int32_t *a, int32_t *b, int32_t *c, int32_t *d) {
    if (a) *a = (int32_t)sizeof(WE_Config);
    if (b) *b = (int32_t)sizeof(WE_Inputs);
    if (c) *c = (int32_t)sizeof(WE_Telemetry);
    if (d) *d = (int32_t)sizeof(WE_TurbineInfo);
    return WE_OK;
}

WE_API int32_t WE_CALL WE_DefaultConfig(WE_Config *c) {
    if (!c) return WE_ERR_NULL;
    memset(c, 0, sizeof *c);
    c->motorNomPowerW = 15000; c->motorNomRpm = 1475; c->rigInertiaKgm2 = 0.35; c->rigViscousNmPerRadS = 0.02;
    c->ref1FullScaleRpm = 1500; c->ref2FullScalePct = 100; c->act1FullScaleRpm = 1500; c->act2FullScalePct = 100;
    c->rotorRadiusM = 38.5; c->airDensity = 1.225; c->turbineRatedPowerW = 1.5e6; c->turbineRatedRotorRpm = 20;
    c->rotorInertiaKgm2 = 4e6; c->cutInWind = 3; c->cutOutWind = 25; c->towerShadowAmplitude = 0.03;
    c->powerUsage = 0.8; c->inertiaCompGain = 1.0; c->accelFilterTauS = 0.05;
    c->pitchKpDegPerPu = 120; c->pitchKiDegPerSPerPu = 50; c->pitchRateLimitDegPerS = 8; c->pitchMaxDeg = 90;
    c->windMeanSpeed = 9; c->turbulenceIntensity = 0.10; c->turbulenceLengthScaleM = 150;
    c->maxMotorRpm = 1700; c->maxTorquePct = 150; c->torqueRampPctPerS = 300; c->pitchStartRampS = 30;
    c->port = 502; c->unitId = 1; c->timeoutMs = 200; c->registerBase = 0;
    c->regControlWord = 1; c->regStatusWord = 4; c->useFc23 = 0; c->simulate = 1;
    c->controlMode = 0; c->torqueOnExt2 = 1; c->blades = 3; c->periodMs = 20; c->maxCommErrors = 5;
    c->hostWatchdogMs = 2000;
    return WE_OK;
}

WE_API int32_t WE_CALL WE_LoadProperties(const char *path, WE_Config *cfg, char *host, int32_t hostLen) {
    if (!path || !cfg) return WE_ERR_NULL;
    FILE *f = fopen(path, "r");
    if (!f) return WE_ERR_FILE;
    char line[512];
    while (fgets(line, sizeof line, f)) {
        char *s = trim(line);
        if (!*s || *s == '#' || *s == '!') continue;
        char *eq = strpbrk(s, "=:");
        if (!eq) continue;
        *eq = 0;
        char *k = trim(s), *v = trim(eq + 1);
        if (!*v) continue;
        if (!strcmp(k, "drive.host")) { copy_str(host, hostLen, v); continue; }
        for (int i = 0; i < NPROPS; i++) {
            if (strcmp(k, g_props[i].key)) continue;
            char *p = (char *)cfg + g_props[i].off;
            if (g_props[i].isInt) { int32_t x = parse_int_value(v); memcpy(p, &x, 4); }
            else { double x = strtod(v, NULL); memcpy(p, &x, 8); }
            break;
        }
    }
    fclose(f);
    return WE_OK;
}

WE_API int32_t WE_CALL WE_SaveProperties(const char *path, const WE_Config *cfg, const char *host) {
    if (!path || !cfg) return WE_ERR_NULL;
    FILE *f = fopen(path, "w");
    if (!f) return WE_ERR_FILE;
    fprintf(f, "# Wind turbine emulator - written by windemu.dll\n");
    fprintf(f, "drive.host=%s\n", host ? host : "");
    for (int i = 0; i < NPROPS; i++) {
        const char *p = (const char *)cfg + g_props[i].off;
        if (g_props[i].isInt) {
            int32_t x; memcpy(&x, p, 4);
            if (!strcmp(g_props[i].key, "emulator.controlMode")) fprintf(f, "%s=%s\n", g_props[i].key, x ? "TORQUE" : "SPEED");
            else if (!strcmp(g_props[i].key, "drive.useFc23") || !strcmp(g_props[i].key, "drive.simulate") ||
                     !strcmp(g_props[i].key, "drive.torqueOnExt2"))
                fprintf(f, "%s=%s\n", g_props[i].key, x ? "true" : "false");
            else fprintf(f, "%s=%d\n", g_props[i].key, (int)x);
        } else {
            double x; memcpy(&x, p, 8);
            fprintf(f, "%s=%.10g\n", g_props[i].key, x);
        }
    }
    fclose(f);
    return WE_OK;
}

static int validate(const WE_Config *c) {
    if (c->motorNomPowerW <= 0 || c->motorNomRpm <= 0 || c->rigInertiaKgm2 <= 0 || c->turbineRatedPowerW <= 0 ||
        c->turbineRatedRotorRpm <= 0 || c->rotorRadiusM <= 0 || c->rotorInertiaKgm2 <= 0 || c->accelFilterTauS <= 0 ||
        c->ref1FullScaleRpm <= 0 || c->ref2FullScalePct <= 0 || c->act1FullScaleRpm <= 0 || c->act2FullScalePct <= 0 ||
        c->port <= 0 || c->port > 65535 || c->timeoutMs <= 0 || c->maxCommErrors <= 0 ||
        c->periodMs < 1 || c->periodMs > 1000 || c->regControlWord < 1 || c->regStatusWord < 1)
        return WE_ERR_ARG;
    return WE_OK;
}

WE_API int32_t WE_CALL WE_Open(const WE_Config *cfg, const char *host, int32_t *handle) {
    if (!cfg || !handle) return WE_ERR_NULL;
    *handle = 0;
    if (validate(cfg) != WE_OK) return WE_ERR_ARG;
    global_init();
    mutex_lock(&g_mtx);
    int slot = -1;
    for (int i = 0; i < MAX_INST; i++) if (!g_inst[i].used) { slot = i; break; }
    if (slot < 0) { mutex_unlock(&g_mtx); return WE_ERR_NO_SLOT; }
    Inst *e = &g_inst[slot];
    memset(e, 0, sizeof *e);
    e->used = 1;
    mutex_unlock(&g_mtx);

    e->cfg = *cfg;
    copy_str(e->host, sizeof e->host, host ? host : "");
    turbine_init(&e->tb, &e->cfg);
    mutex_init(&e->mtx);
    e->mb.s = BAD_SOCK;
    e->mb.unit = cfg->unitId;
    e->mb.timeoutMs = cfg->timeoutMs;
    e->in.windMeanSpeed = cfg->windMeanSpeed;
    e->in.turbulenceIntensity = cfg->turbulenceIntensity;
    e->gustStart = -1;
    e->beta = cfg->pitchMaxDeg;   /* parked turbine: feathered */
    e->rng = 0x9E3779B97F4A7C15ULL ^ (uint64_t)(now_s() * 1e6) ^ ((uint64_t)(slot + 1) << 32);
    e->lastPoll = now_s();
    e->state = WE_DISCONNECTED;
    snprintf(e->msg, sizeof e->msg, "Not connected");

#ifdef _WIN32
    e->thread = CreateThread(NULL, 0, rt_thread, e, 0, NULL);
    int ok = e->thread != NULL;
#else
    int ok = pthread_create(&e->thread, NULL, rt_thread, e) == 0;
#endif
    if (!ok) { mutex_free(&e->mtx); e->used = 0; return WE_ERR_THREAD; }
    *handle = slot + 1;
    return WE_OK;
}

WE_API int32_t WE_CALL WE_Close(int32_t handle) {
    Inst *e = get_inst(handle);
    if (!e) return WE_ERR_HANDLE;
    e->shutdown = 1;
#ifdef _WIN32
    WaitForSingleObject(e->thread, 5000);
    CloseHandle(e->thread);
#else
    pthread_join(e->thread, NULL);
#endif
    mutex_free(&e->mtx);
    free(e->profT); free(e->profV);
    global_init();
    mutex_lock(&g_mtx);
    e->used = 0;
    mutex_unlock(&g_mtx);
    return WE_OK;
}

WE_API int32_t WE_CALL WE_Command(int32_t handle, int32_t cmd) {
    Inst *e = get_inst(handle);
    if (!e) return WE_ERR_HANDLE;
    if (cmd < WE_CMD_CONNECT || cmd > WE_CMD_LULL) return WE_ERR_ARG;
    mutex_lock(&e->mtx);
    int next = (e->cmdHead + 1) % CMDQ;
    if (next != e->cmdTail) { e->cmdq[e->cmdHead] = cmd; e->cmdHead = next; }
    e->lastPoll = now_s();
    mutex_unlock(&e->mtx);
    return WE_OK;
}

WE_API int32_t WE_CALL WE_SetInputs(int32_t handle, const WE_Inputs *in) {
    Inst *e = get_inst(handle);
    if (!e) return WE_ERR_HANDLE;
    if (!in) return WE_ERR_NULL;
    mutex_lock(&e->mtx);
    e->in = *in;
    e->in.windMeanSpeed = clampd(in->windMeanSpeed, 0, 60);
    e->in.turbulenceIntensity = clampd(in->turbulenceIntensity, 0, 1);
    mutex_unlock(&e->mtx);
    return WE_OK;
}

WE_API int32_t WE_CALL WE_GetTelemetry(int32_t handle, WE_Telemetry *out) {
    Inst *e = get_inst(handle);
    if (!e) return WE_ERR_HANDLE;
    if (!out) return WE_ERR_NULL;
    mutex_lock(&e->mtx);
    *out = e->tel;
    e->lastPoll = now_s();
    mutex_unlock(&e->mtx);
    return WE_OK;
}

WE_API int32_t WE_CALL WE_GetTurbineInfo(int32_t handle, WE_TurbineInfo *o) {
    Inst *e = get_inst(handle);
    if (!e) return WE_ERR_HANDLE;
    if (!o) return WE_ERR_NULL;
    const Turbine *t = &e->tb;
    o->gearRatio = t->G;
    o->powerScale = t->kp;
    o->emulatedInertiaMotor = t->Jemu;
    o->lambdaOpt = t->lambdaOpt;
    o->cpMax = t->cpMax;
    o->kOpt = t->kOpt;
    o->ratedRotorTorqueKNm = t->Trated / 1e3;
    o->ratedTorqueAtMotorNm = r2m_T(t, t->Trated);
    o->motorNomTorqueNm = t->Tnom;
    o->ratedWindAtLambdaOpt = t->wRated * t->R / t->lambdaOpt;
    return WE_OK;
}

WE_API int32_t WE_CALL WE_GetMessage(int32_t handle, char *buf, int32_t len) {
    Inst *e = get_inst(handle);
    if (!e) return WE_ERR_HANDLE;
    if (!buf) return WE_ERR_NULL;
    mutex_lock(&e->mtx);
    copy_str(buf, len, e->msg);
    mutex_unlock(&e->mtx);
    return WE_OK;
}

WE_API int32_t WE_CALL WE_LoadWindProfile(int32_t handle, const char *path) {
    Inst *e = get_inst(handle);
    if (!e) return WE_ERR_HANDLE;
    if (!path) return WE_ERR_NULL;
    FILE *f = fopen(path, "r");
    if (!f) return WE_ERR_FILE;
    int cap = 256, n = 0;
    double *T = malloc(cap * sizeof *T), *V = malloc(cap * sizeof *V);
    char line[256];
    while (T && V && fgets(line, sizeof line, f)) {
        char *s = trim(line), *end;
        if (!*s || *s == '#' || isalpha((unsigned char)*s)) continue;
        double a = strtod(s, &end);
        if (end == s) continue;
        while (*end == ',' || *end == ';' || isspace((unsigned char)*end)) end++;
        double b = strtod(end, NULL);
        if (n == cap) { cap *= 2; T = realloc(T, cap * sizeof *T); V = realloc(V, cap * sizeof *V); if (!T || !V) break; }
        T[n] = a; V[n] = b; n++;
    }
    fclose(f);
    if (!T || !V || n == 0) { free(T); free(V); return WE_ERR_FILE; }
    mutex_lock(&e->mtx);
    free(e->profT); free(e->profV);
    e->profT = T; e->profV = V; e->profN = n; e->profTime = 0;
    mutex_unlock(&e->mtx);
    return WE_OK;
}

WE_API int32_t WE_CALL WE_DescribeStatus(int32_t sw, char *buf, int32_t len) {
    char b[256];
    if (!buf) return WE_ERR_NULL;
    snprintf(b, sizeof b, "%s%s%s%s%s%s", sw_state_name(sw),
             (sw & SW_ALARM) ? " | ALARM" : "",
             (sw & SW_REMOTE) ? "" : " | LOCAL (not remote!)",
             (sw & SW_AT_SETP) ? " | AT SETPOINT" : "",
             (sw & SW_ABOVE_LIM) ? " | ABOVE LIMIT" : "",
             (sw & SW_EXT2) ? " | EXT2" : " | EXT1");
    copy_str(buf, len, b);
    return WE_OK;
}

WE_API int32_t WE_CALL WE_StateName(int32_t state, char *buf, int32_t len) {
    if (!buf) return WE_ERR_NULL;
    if (state < 0 || state > 7) return WE_ERR_ARG;
    copy_str(buf, len, state_names[state]);
    return WE_OK;
}
