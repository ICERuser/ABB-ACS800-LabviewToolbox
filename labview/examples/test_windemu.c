/* Console test of windemu (same calls LabVIEW makes). Linux: gcc -I../src test_windemu.c -L../bin -lwindemu
   Needs tools/fake_reta01.py on port 15030 for the Modbus part. */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "windemu.h"
static void show(const char *tag, int h) {
    WE_Telemetry t; char m[256], st[32];
    WE_GetTelemetry(h, &t); WE_GetMessage(h, m, sizeof m); WE_StateName(t.state, st, sizeof st);
    printf("%-6s t=%6.1f %-9s v=%5.2f b=%5.1f lam=%4.2f cp=%.3f rotor=%5.2f motor=%6.0f Tref=%6.1f Tact=%6.1f P=%7.1fkW CW=%04X SW=%04X | %s\n",
        tag, t.timeS, st, t.windMs, t.pitchDeg, t.lambda, t.cp, t.rotorRpm, t.motorRpm, t.torqueRefPct, t.torqueActPct,
        t.aeroPowerKW, t.controlWord, t.statusWord, m);
}
static int st(int h) { WE_Telemetry t; WE_GetTelemetry(h, &t); return t.state; }
int main(void) {
    int a,b,c,d; WE_GetStructSizes(&a,&b,&c,&d); printf("sizes %d %d %d %d  version %08X\n", a,b,c,d, WE_Version());
    WE_Config cfg; char host[64] = "";
    WE_DefaultConfig(&cfg);
    int rc = WE_LoadProperties("../../emulator.properties", &cfg, host, sizeof host);
    printf("load rc=%d host=%s port=%d mode=%d sim=%d R=%.1f wind=%.1f\n", rc, host, cfg.port, cfg.controlMode, cfg.simulate, cfg.rotorRadiusM, cfg.windMeanSpeed);
    WE_SaveProperties("saved.properties", &cfg, host);
    WE_Config c2; char h2[64]; WE_DefaultConfig(&c2); c2.port = 1; WE_LoadProperties("saved.properties", &c2, h2, 64);
    printf("roundtrip identical: %s\n", memcmp(&cfg, &c2, sizeof cfg) == 0 && !strcmp(host, h2) ? "yes" : "NO");

    cfg.rotorInertiaKgm2 = 8e5; cfg.windMeanSpeed = 13; cfg.simulate = 1;
    WE_Config ct = cfg; ct.controlMode = 1;
    int hs, ht;
    int r1 = WE_Open(&cfg, host, &hs), r2 = WE_Open(&ct, host, &ht);
    printf("open rc=%d,%d handles %d %d\n", r1, r2, hs, ht);
    WE_TurbineInfo ti; WE_GetTurbineInfo(hs, &ti);
    printf("info G=%.2f kp=%.4f Jemu=%.3f lamOpt=%.2f cpMax=%.3f Trot=%.0fkNm Tmot=%.1fNm Tnom=%.1f vRated=%.2f\n",
        ti.gearRatio, ti.powerScale, ti.emulatedInertiaMotor, ti.lambdaOpt, ti.cpMax, ti.ratedRotorTorqueKNm, ti.ratedTorqueAtMotorNm, ti.motorNomTorqueNm, ti.ratedWindAtLambdaOpt);
    WE_Command(hs, WE_CMD_CONNECT); WE_Command(ht, WE_CMD_CONNECT); usleep(300000);
    WE_Command(hs, WE_CMD_START); WE_Command(ht, WE_CMD_START);
    WE_Telemetry dummy;
    for (int i = 1; i <= 16; i++) {
        for (int k = 0; k < 25; k++) { usleep(200000); WE_GetTelemetry(hs, &dummy); WE_GetTelemetry(ht, &dummy); }
        show("SPEED", hs); show("TORQUE", ht);
    }
    WE_Inputs in = { 13, 0.1, 0, 0, 0 };
    WE_SetInputs(hs, &in); WE_Command(hs, WE_CMD_GUST);
    for (int k = 0; k < 30; k++) { usleep(200000); WE_GetTelemetry(hs, &dummy); WE_GetTelemetry(ht, &dummy); }
    show("GUST", hs);

    /* E-stop -> reset -> restart on TORQUE instance */
    WE_Command(ht, WE_CMD_ESTOP); usleep(400000); show("ESTOP", ht);
    for (int i = 0; i < 40 && st(ht) == WE_ESTOP; i++) { WE_Command(ht, WE_CMD_RESET); usleep(300000); }
    show("RESET", ht); WE_Command(ht, WE_CMD_START); usleep(1500000); show("RESTRT", ht);

    /* host watchdog: SPEED instance not polled for 3 s */
    WE_Close(ht);
    usleep(3000000);
    show("WDOG", hs);
    r1 = WE_Close(hs); r2 = WE_Close(hs);
    printf("close rc=%d, second close rc=%d (expect 0, -1)\n", r1, r2);

    /* real Modbus/TCP against fake RETA-01 */
    WE_Config cm = cfg; cm.simulate = 0; cm.port = 15030; cm.controlMode = 1; int hm;
    WE_Open(&cm, "127.0.0.1", &hm);
    WE_Command(hm, WE_CMD_CONNECT); usleep(400000); show("MB", hm);
    WE_Command(hm, WE_CMD_START); usleep(1500000); show("MB", hm);
    WE_Close(hm);
    /* bad port -> connection refused */
    cm.port = 15999; WE_Open(&cm, "127.0.0.1", &hm); WE_Command(hm, WE_CMD_CONNECT); usleep(1500000); show("BAD", hm); WE_Close(hm);
    return 0;
}
