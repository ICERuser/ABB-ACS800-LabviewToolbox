/*
 * windemu.h – Wind turbine emulator DLL for LabVIEW
 * ABB ACS800 (DTC) controlled through RETA-01, protocol Modbus/TCP.
 *
 * Native C port of the Java emulator (same models, state machine and safety logic).
 * Use it from LabVIEW with "Call Library Function Node":
 *   - Calling convention: C
 *   - Thread: "Run in any thread" (all functions are thread safe)
 *   - Clusters: "Adapt to Type", "Handles by Value" -> passed as pointer to the struct
 *
 * CLUSTER LAYOUT RULE
 *   Every cluster is: all DBL fields first, then an EVEN number of I32 fields.
 *   This gives the identical memory layout in 32-bit LabVIEW (1-byte packing) and
 *   64-bit LabVIEW (natural alignment), so one cluster definition works with both DLLs.
 *   The element ORDER inside the LabVIEW cluster must be exactly the order below.
 */
#ifndef WINDEMU_H
#define WINDEMU_H

#include <stdint.h>

#ifdef _WIN32
#  ifdef WINDEMU_BUILD
#    define WE_API __declspec(dllexport)
#  else
#    define WE_API __declspec(dllimport)
#  endif
#  define WE_CALL __cdecl
#else
#  define WE_API __attribute__((visibility("default")))
#  define WE_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

/* ------------------------------------------------------------------------------------------
 * Config cluster  (= emulator.properties)   30 x DBL + 14 x I32  = 296 bytes
 * ---------------------------------------------------------------------------------------- */
typedef struct {
    /* motor / test rig */
    double motorNomPowerW;        /*  0  motor.nominalPowerW            15000            */
    double motorNomRpm;           /*  1  motor.nominalRpm               1475             */
    double rigInertiaKgm2;        /*  2  motor.rigInertiaKgm2           0.35             */
    double rigViscousNmPerRadS;   /*  3  motor.rigViscousNmPerRadS      0.02             */
    /* drive fieldbus scaling */
    double ref1FullScaleRpm;      /*  4  REF1 20000 = 11.05             1500             */
    double ref2FullScalePct;      /*  5  REF2 10000 = 11.08             100              */
    double act1FullScaleRpm;      /*  6  ACT1 20000 =                   1500             */
    double act2FullScalePct;      /*  7  ACT2 10000 =                   100              */
    /* turbine */
    double rotorRadiusM;          /*  8  turbine.rotorRadiusM           38.5             */
    double airDensity;            /*  9  turbine.airDensity             1.225            */
    double turbineRatedPowerW;    /* 10  turbine.ratedPowerW            1.5e6            */
    double turbineRatedRotorRpm;  /* 11  turbine.ratedRotorRpm          20               */
    double rotorInertiaKgm2;      /* 12  turbine.rotorInertiaKgm2       4e6              */
    double cutInWind;             /* 13  turbine.cutInWind              3                */
    double cutOutWind;            /* 14  turbine.cutOutWind             25               */
    double towerShadowAmplitude;  /* 15  turbine.towerShadowAmplitude   0.03             */
    /* scaling */
    double powerUsage;            /* 16  scaling.powerUsage             0.8              */
    double inertiaCompGain;       /* 17  scaling.inertiaCompensationGain 1.0             */
    double accelFilterTauS;       /* 18  scaling.accelFilterTauS        0.05             */
    /* pitch */
    double pitchKpDegPerPu;       /* 19  pitch.kpDegPerPu               120              */
    double pitchKiDegPerSPerPu;   /* 20  pitch.kiDegPerSPerPu           50               */
    double pitchRateLimitDegPerS; /* 21  pitch.rateLimitDegPerS         8                */
    double pitchMaxDeg;           /* 22  pitch.maxDeg                   90               */
    /* wind (initial values) */
    double windMeanSpeed;         /* 23  wind.meanSpeed                 9                */
    double turbulenceIntensity;   /* 24  wind.turbulenceIntensity       0.10             */
    double turbulenceLengthScaleM;/* 25  wind.turbulenceLengthScaleM    150              */
    /* safety */
    double maxMotorRpm;           /* 26  safety.maxMotorRpm             1700             */
    double maxTorquePct;          /* 27  safety.maxTorquePct            150              */
    double torqueRampPctPerS;     /* 28  safety.torqueRampPctPerS       300              */
    double pitchStartRampS;       /* 29  pitch.startRampS: rotor-speed set-point ramp to  */
                                  /*     rated during start-up (limits overshoot)  30    */

    /* ---- I32 block (even count) ---- */
    int32_t port;                 /* 30  drive.port                     502              */
    int32_t unitId;               /* 31  drive.unitId                   1                */
    int32_t timeoutMs;            /* 32  drive.timeoutMs                200              */
    int32_t registerBase;         /* 33  drive.registerBase             0                */
    int32_t regControlWord;       /* 34  drive.reg.controlWord          1  (40001)       */
    int32_t regStatusWord;        /* 35  drive.reg.statusWord           4  (40004)       */
    int32_t useFc23;              /* 36  drive.useFc23                  0/1              */
    int32_t simulate;             /* 37  drive.simulate                 0/1              */
    int32_t controlMode;          /* 38  0 = SPEED, 1 = TORQUE                           */
    int32_t torqueOnExt2;         /* 39  drive.torqueOnExt2             0/1              */
    int32_t blades;               /* 40  turbine.blades                 3                */
    int32_t periodMs;             /* 41  loop.periodMs                  20               */
    int32_t maxCommErrors;        /* 42  safety.maxConsecutiveCommErrors 5               */
    int32_t hostWatchdogMs;       /* 43  stop the drive if LabVIEW does not call          */
                                  /*     WE_GetTelemetry within this time (0 = off) 2000 */
} WE_Config;

/* ------------------------------------------------------------------------------------------
 * Operator inputs cluster (may be changed at any time)   3 x DBL + 2 x I32 = 32 bytes
 * ---------------------------------------------------------------------------------------- */
typedef struct {
    double  windMeanSpeed;        /* m/s                                                  */
    double  turbulenceIntensity;  /* 0..0.3                                               */
    double  manualPitchDeg;       /* used when manualPitch = 1                            */
    int32_t manualPitch;          /* 0 = automatic pitch, 1 = manual                      */
    int32_t useWindProfile;       /* 1 = follow profile loaded with WE_LoadWindProfile    */
} WE_Inputs;

/* ------------------------------------------------------------------------------------------
 * Telemetry cluster (indicators)   15 x DBL + 10 x I32 = 160 bytes
 * ---------------------------------------------------------------------------------------- */
typedef struct {
    double  timeS;                /*  0  emulator time                                     */
    double  windMs;               /*  1  hub wind speed                                    */
    double  pitchDeg;             /*  2                                                    */
    double  lambda;               /*  3  tip-speed ratio                                   */
    double  cp;                   /*  4  power coefficient                                 */
    double  rotorRpm;             /*  5  turbine rotor speed                               */
    double  aeroTorqueKNm;        /*  6  aerodynamic torque on the rotor                   */
    double  aeroPowerKW;          /*  7  aerodynamic power (turbine scale)                 */
    double  motorRpm;             /*  8  measured motor speed (ACT1)                       */
    double  torqueRefPct;         /*  9  motor torque reference, % of T_nom                */
    double  torqueActPct;         /* 10  measured motor torque (ACT2), %                   */
    double  speedRefRpm;          /* 11  speed reference (SPEED mode)                      */
    double  genTorqueEstPct;      /* 12  estimated generator torque, %                     */
    double  genTorqueRecPct;      /* 13  recommended MPPT generator torque, %              */
    double  cycleMs;              /* 14  execution time of one cycle                       */
    int32_t state;                /* 15  WE_State                                          */
    int32_t controlWord;          /* 16  CW sent                                           */
    int32_t statusWord;           /* 17  SW received                                       */
    int32_t ref1;                 /* 18  raw REF1                                          */
    int32_t ref2;                 /* 19  raw REF2                                          */
    int32_t act1;                 /* 20  raw ACT1                                          */
    int32_t act2;                 /* 21  raw ACT2                                          */
    int32_t commErrors;           /* 22  consecutive communication errors                  */
    int32_t driveTripped;         /* 23  SW bit 3                                          */
    int32_t driveRemote;          /* 24  SW bit 9                                          */
} WE_Telemetry;

/* ------------------------------------------------------------------------------------------
 * Turbine / scaling info cluster (indicators)   10 x DBL = 80 bytes
 * ---------------------------------------------------------------------------------------- */
typedef struct {
    double gearRatio;             /* G = n_motor,nom / n_rotor,rated                      */
    double powerScale;            /* k_p                                                  */
    double emulatedInertiaMotor;  /* J_rotor * k_p / G^2  [kgm2]                          */
    double lambdaOpt;
    double cpMax;
    double kOpt;                  /* MPPT constant [Nm/(rad/s)^2]                          */
    double ratedRotorTorqueKNm;
    double ratedTorqueAtMotorNm;
    double motorNomTorqueNm;
    double ratedWindAtLambdaOpt;  /* wind speed where rated rotor speed is reached at λopt */
} WE_TurbineInfo;

#pragma pack(pop)

/* States (use as LabVIEW enum, U16/I32, same order) */
typedef enum {
    WE_DISCONNECTED = 0, WE_IDLE = 1, WE_STARTING = 2, WE_RUNNING = 3,
    WE_STOPPING = 4, WE_ESTOP = 5, WE_FAULT = 6, WE_COMM_LOST = 7
} WE_State;

/* Commands for WE_Command (LabVIEW enum, same order) */
typedef enum {
    WE_CMD_CONNECT = 0, WE_CMD_DISCONNECT = 1, WE_CMD_START = 2, WE_CMD_STOP = 3,
    WE_CMD_ESTOP = 4, WE_CMD_RESET = 5, WE_CMD_GUST = 6, WE_CMD_LULL = 7
} WE_Cmd;

/* Return codes */
#define WE_OK               0
#define WE_ERR_HANDLE      -1   /* invalid or closed handle                     */
#define WE_ERR_NULL        -2   /* NULL pointer argument                        */
#define WE_ERR_THREAD      -3   /* could not create the real-time thread        */
#define WE_ERR_FILE        -4   /* file not found / unreadable                  */
#define WE_ERR_NO_SLOT     -5   /* too many open instances (max 8)              */
#define WE_ERR_ARG         -6   /* argument out of range                        */
#define WE_ERR_SIZE        -7   /* cluster size mismatch (see WE_GetStructSizes)  */

/* ---- lifecycle ---------------------------------------------------------------------------*/
/* Fill a config cluster with the default values (same as emulator.properties). */
WE_API int32_t WE_CALL WE_DefaultConfig(WE_Config *cfg);

/* Read an emulator.properties file into the config cluster and the host string.
 * Keys that are missing keep the value already in *cfg (call WE_DefaultConfig first). */
WE_API int32_t WE_CALL WE_LoadProperties(const char *path, WE_Config *cfg, char *host, int32_t hostLen);

/* Write the config cluster + host back to a .properties file. */
WE_API int32_t WE_CALL WE_SaveProperties(const char *path, const WE_Config *cfg, const char *host);

/* Create an emulator instance and start its real-time thread (not yet connected).
 * host = RETA-01 IP address, e.g. "192.168.0.16". The port is cfg->port. */
WE_API int32_t WE_CALL WE_Open(const WE_Config *cfg, const char *host, int32_t *handle);

/* Stop the thread, put the drive in a safe state (0x0476) and free the instance. */
WE_API int32_t WE_CALL WE_Close(int32_t handle);

/* ---- operation ---------------------------------------------------------------------------*/
WE_API int32_t WE_CALL WE_Command(int32_t handle, int32_t cmd);
WE_API int32_t WE_CALL WE_SetInputs(int32_t handle, const WE_Inputs *in);
WE_API int32_t WE_CALL WE_GetTelemetry(int32_t handle, WE_Telemetry *out);
WE_API int32_t WE_CALL WE_GetTurbineInfo(int32_t handle, WE_TurbineInfo *out);
/* Latest status message, e.g. "Connected to RETA-01 192.168.0.16:502". */
WE_API int32_t WE_CALL WE_GetMessage(int32_t handle, char *buf, int32_t len);
/* CSV "time_s,wind_ms" (looped). Enable it with WE_Inputs.useWindProfile = 1. */
WE_API int32_t WE_CALL WE_LoadWindProfile(int32_t handle, const char *path);

/* ---- helpers -----------------------------------------------------------------------------*/
/* Decode an ABB Drives status word, e.g. "OPERATING | AT SETPOINT | EXT1". */
WE_API int32_t WE_CALL WE_DescribeStatus(int32_t statusWord, char *buf, int32_t len);
/* Name of a WE_State. */
WE_API int32_t WE_CALL WE_StateName(int32_t state, char *buf, int32_t len);
/* Struct sizes compiled into the DLL: 296 / 32 / 160 / 80 bytes. Use to verify your clusters
 * with LabVIEW "Flatten To String" length. */
WE_API int32_t WE_CALL WE_GetStructSizes(int32_t *config, int32_t *inputs, int32_t *telemetry, int32_t *info);
/* DLL version, e.g. 0x00010000 = 1.0.0 */
WE_API int32_t WE_CALL WE_Version(void);

#ifdef __cplusplus
}
#endif
#endif /* WINDEMU_H */
