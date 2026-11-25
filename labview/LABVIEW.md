# LabVIEW integration – `windemu` DLL

`windemu_32.dll` / `windemu_64.dll` contain the complete wind turbine emulator as native code: the
Modbus/TCP link to the RETA-01, the turbine physics, the ABB Drives state machine, the safety logic and the
offline drive simulator. LabVIEW calls them through **Call Library Function Nodes (CLFN)**.

The DLL runs its own 20 ms real-time thread. LabVIEW does not need to be deterministic: it only sets inputs,
sends commands and reads the telemetry, every 50–200 ms.

```
 LabVIEW front panel                              windemu DLL                       test bench
┌───────────────────────────┐   WE_Open / WE_Close   ┌─────────────────────────┐     ┌──────────┐
│ Host (string)             │ ─────────────────────► │ real-time thread 20 ms  │     │ RETA-01  │
│ Config cluster  (44 el.)  │   WE_Command (enum)    │  wind ▸ Cp(λ,β) ▸ pitch │ TCP │ ACS800   │
│ Inputs cluster  (5 el.)   │ ─────────────────────► │  scaling ▸ CW/REF1/REF2 │◄───►│ motor    │
│ Command buttons           │   WE_SetInputs         │  safety ▸ state machine │ 502 │          │
│                           │ ◄───────────────────── │                         │     └──────────┘
│ Telemetry cluster (25 el.)│   WE_GetTelemetry      │  host watchdog          │
│ Turbine info (10 el.)     │   WE_GetMessage        └─────────────────────────┘
└───────────────────────────┘
```

## 1. Files

| File | Use |
|---|---|
| `bin/windemu_64.dll` | 64-bit LabVIEW (Windows) |
| `bin/windemu_32.dll` | 32-bit LabVIEW (Windows) |
| `bin/libwindemu.so` | LabVIEW for Linux / tests |
| `src/windemu.h` | Exact definition of every cluster and function |
| `src/windemu.c` | Source code |
| `build_windows.bat`, `build_mingw.sh` | Rebuild scripts (MSVC / MinGW) |
| `tools/fake_reta01.py` | Fake RETA-01 Modbus server for testing without a drive |
| `examples/test_windemu.c` | Console program making the same calls as LabVIEW |

The DLLs only depend on Windows system DLLs (`KERNEL32`, `WS2_32`, `WINMM`, `msvcrt`). No runtime needs
to be installed.

**Tip:** in every CLFN, enter the library name as **`windemu_*.dll`**. LabVIEW replaces `*` with `32` or `64`
to match its own bitness, so the same VIs work in both versions. Keep both DLLs next to your VIs.

## 2. Clusters

**Rules**
- Element order and types must be **exactly** as in the tables (order is the cluster order, see
  *Reorder Controls in Cluster*). The labels are only for you.
- Only **DBL** and **I32** are used. **Don't use Booleans**. For on/off settings use an I32 (you can show it as
  a ring "Off=0 / On=1").
- For enums (control mode, state, command) use an **enum with representation U32**, or an I32 ring.
- These clusters have the same memory layout in 32-bit and 64-bit LabVIEW. Make them **type definitions**.
- To check a cluster: `Flatten To String` length must equal the size from `WE_GetStructSizes`
  (Config 296, Inputs 32, Telemetry 160, Info 80 bytes).

### 2.1 Config cluster (controls) = `emulator.properties` – 30 DBL + 14 I32 = 296 bytes

| # | Label | Type | Default | properties key | Meaning |
|---|---|---|---|---|---|
| 0 | motorNomPowerW | DBL | 15000 | motor.nominalPowerW | Motor nominal power [W] |
| 1 | motorNomRpm | DBL | 1475 | motor.nominalRpm | Motor nominal speed [rpm] |
| 2 | rigInertiaKgm2 | DBL | 0.35 | motor.rigInertiaKgm2 | Motor + coupling + generator inertia |
| 3 | rigViscousNmPerRadS | DBL | 0.02 | motor.rigViscousNmPerRadS | Bench friction |
| 4 | ref1FullScaleRpm | DBL | 1500 | drive.ref1FullScaleRpm | rpm at REF1 = 20000 (par. 11.05) |
| 5 | ref2FullScalePct | DBL | 100 | drive.ref2FullScalePct | % torque at REF2 = 10000 (par. 11.08) |
| 6 | act1FullScaleRpm | DBL | 1500 | drive.act1FullScaleRpm | rpm at ACT1 = 20000 |
| 7 | act2FullScalePct | DBL | 100 | drive.act2FullScalePct | % torque at ACT2 = 10000 |
| 8 | rotorRadiusM | DBL | 38.5 | turbine.rotorRadiusM | Rotor radius [m] |
| 9 | airDensity | DBL | 1.225 | turbine.airDensity | [kg/m³] |
| 10 | turbineRatedPowerW | DBL | 1.5e6 | turbine.ratedPowerW | [W] |
| 11 | turbineRatedRotorRpm | DBL | 20 | turbine.ratedRotorRpm | [rpm] |
| 12 | rotorInertiaKgm2 | DBL | 4e6 | turbine.rotorInertiaKgm2 | [kgm²] |
| 13 | cutInWind | DBL | 3 | turbine.cutInWind | [m/s] |
| 14 | cutOutWind | DBL | 25 | turbine.cutOutWind | Feathers above this [m/s] |
| 15 | towerShadowAmplitude | DBL | 0.03 | turbine.towerShadowAmplitude | 3P torque dip (0 = off) |
| 16 | powerUsage | DBL | 0.8 | scaling.powerUsage | Turbine rated power → fraction of motor power |
| 17 | inertiaCompGain | DBL | 1.0 | scaling.inertiaCompensationGain | TORQUE mode, 0…1 |
| 18 | accelFilterTauS | DBL | 0.05 | scaling.accelFilterTauS | Acceleration filter [s] |
| 19 | pitchKpDegPerPu | DBL | 120 | pitch.kpDegPerPu | Pitch PI gain [°/pu] |
| 20 | pitchKiDegPerSPerPu | DBL | 50 | pitch.kiDegPerSPerPu | Pitch PI gain [°/(s·pu)] |
| 21 | pitchRateLimitDegPerS | DBL | 8 | pitch.rateLimitDegPerS | [°/s] |
| 22 | pitchMaxDeg | DBL | 90 | pitch.maxDeg | Feather position [°] |
| 23 | windMeanSpeed | DBL | 9 | wind.meanSpeed | Initial wind [m/s] |
| 24 | turbulenceIntensity | DBL | 0.10 | wind.turbulenceIntensity | Initial turbulence |
| 25 | turbulenceLengthScaleM | DBL | 150 | wind.turbulenceLengthScaleM | [m] |
| 26 | maxMotorRpm | DBL | 1700 | safety.maxMotorRpm | Overspeed trip → OFF3 |
| 27 | maxTorquePct | DBL | 150 | safety.maxTorquePct | Torque reference limit |
| 28 | torqueRampPctPerS | DBL | 300 | safety.torqueRampPctPerS | Torque reference slew rate |
| 29 | pitchStartRampS | DBL | 30 | pitch.startRampS | Start-up speed ramp to rated [s] |
| 30 | **port** | I32 | 502 | drive.port | **Modbus/TCP port of the RETA-01** |
| 31 | unitId | I32 | 1 | drive.unitId | Modbus unit ID |
| 32 | timeoutMs | I32 | 200 | drive.timeoutMs | Reply timeout [ms] |
| 33 | registerBase | I32 | 0 | drive.registerBase | PDU address of 40001 (0 or 1) |
| 34 | regControlWord | I32 | 1 | drive.reg.controlWord | 1 = register 40001 |
| 35 | regStatusWord | I32 | 4 | drive.reg.statusWord | 4 = register 40004 |
| 36 | useFc23 | I32 | 0 | drive.useFc23 | 1 = FC23 read/write in one telegram |
| 37 | simulate | I32 | 1 | drive.simulate | **1 = built-in simulator, 0 = real drive** |
| 38 | controlMode | U32 enum | 0 | emulator.controlMode | 0 = SPEED, 1 = TORQUE |
| 39 | torqueOnExt2 | I32 | 1 | drive.torqueOnExt2 | TORQUE mode via EXT2 (CW bit 11) |
| 40 | blades | I32 | 3 | turbine.blades | |
| 41 | periodMs | I32 | 20 | loop.periodMs | Real-time cycle [ms] |
| 42 | maxCommErrors | I32 | 5 | safety.maxConsecutiveCommErrors | Errors in a row → COMM_LOST |
| 43 | hostWatchdogMs | I32 | 2000 | host.watchdogMs | Stops the drive if LabVIEW stops polling (0 = off) |

The **IP address** is a separate **string control** ("Host", e.g. `192.168.0.16`), passed to `WE_Open`. The
**port** is element 30 of the config cluster. `WE_LoadProperties` fills both from an `emulator.properties`
file, and `WE_SaveProperties` writes them back.

### 2.2 Inputs cluster (controls, can be changed while running) – 3 DBL + 2 I32 = 32 bytes

| # | Label | Type | Meaning |
|---|---|---|---|
| 0 | windMeanSpeed | DBL | Mean wind [m/s] (slider 0…30) |
| 1 | turbulenceIntensity | DBL | 0…0.3 |
| 2 | manualPitchDeg | DBL | Pitch set-point in manual mode [°] |
| 3 | manualPitch | I32 | 0 = automatic, 1 = manual pitch |
| 4 | useWindProfile | I32 | 1 = follow the CSV loaded with `WE_LoadWindProfile` |

### 2.3 Telemetry cluster (indicators) – 15 DBL + 10 I32 = 160 bytes

| # | Label | Type | Meaning |
|---|---|---|---|
| 0 | timeS | DBL | Emulator time [s] |
| 1 | windMs | DBL | Hub wind incl. turbulence/gust [m/s] |
| 2 | pitchDeg | DBL | Blade pitch [°] |
| 3 | lambda | DBL | Tip-speed ratio |
| 4 | cp | DBL | Power coefficient |
| 5 | rotorRpm | DBL | Turbine rotor speed [rpm] |
| 6 | aeroTorqueKNm | DBL | Aerodynamic torque on rotor [kNm] |
| 7 | aeroPowerKW | DBL | Aerodynamic power, turbine scale [kW] |
| 8 | motorRpm | DBL | Measured motor speed (ACT1) [rpm] |
| 9 | torqueRefPct | DBL | Motor torque reference [% T_nom] |
| 10 | torqueActPct | DBL | Measured motor torque (ACT2) [%] |
| 11 | speedRefRpm | DBL | Speed reference, SPEED mode [rpm] |
| 12 | genTorqueEstPct | DBL | Estimated generator torque [%] |
| 13 | genTorqueRecPct | DBL | Recommended MPPT generator torque [%] |
| 14 | cycleMs | DBL | Execution time of one DLL cycle [ms] |
| 15 | state | U32 enum | 0 DISCONNECTED, 1 IDLE, 2 STARTING, 3 RUNNING, 4 STOPPING, 5 ESTOP, 6 FAULT, 7 COMM_LOST |
| 16 | controlWord | I32 | Control word sent (display in hex) |
| 17 | statusWord | I32 | Status word received (hex) |
| 18 | ref1 | I32 | Raw REF1 |
| 19 | ref2 | I32 | Raw REF2 |
| 20 | act1 | I32 | Raw ACT1 |
| 21 | act2 | I32 | Raw ACT2 |
| 22 | commErrors | I32 | Consecutive Modbus errors |
| 23 | driveTripped | I32 | 1 = drive fault (SW bit 3), use for an LED |
| 24 | driveRemote | I32 | 1 = drive in REMOTE (SW bit 9), use for an LED |

### 2.4 Turbine info cluster (indicators, read once after `WE_Open`) – 10 DBL = 80 bytes

| # | Label | Meaning |
|---|---|---|
| 0 | gearRatio | G = n_motor,nom / n_rotor,rated |
| 1 | powerScale | k_p |
| 2 | emulatedInertiaMotor | Turbine inertia seen at the motor [kgm²] |
| 3 | lambdaOpt | Optimum tip-speed ratio |
| 4 | cpMax | Maximum Cp |
| 5 | kOpt | MPPT constant [Nm/(rad/s)²] |
| 6 | ratedRotorTorqueKNm | |
| 7 | ratedTorqueAtMotorNm | |
| 8 | motorNomTorqueNm | |
| 9 | ratedWindAtLambdaOpt | Wind at which rated rotor speed is reached at λopt [m/s] |

### 2.5 Command enum (U32) for `WE_Command`

`0 CONNECT, 1 DISCONNECT, 2 START, 3 STOP, 4 ESTOP, 5 RESET, 6 GUST (+5 m/s), 7 LULL (−3 m/s)`

## 3. Call Library Function Node settings

For **all** nodes: Library = `windemu_*.dll`, Calling convention = **C**, Thread = **Run in any thread**
(the DLL is thread safe), return type = **Numeric I32** (error code, see §6).

Parameter types:
- **Cluster** → *Adapt to Type* (LabVIEW passes a pointer to the cluster data).
- **Handle / numbers in** → *Numeric I32, Value*.
- **Handle out** → *Numeric I32, Pointer to Value*.
- **String in** → *String, C String Pointer*.
- **String out** → *String, C String Pointer*, with *Minimum size* = the `len` parameter (use 256).

| Function | Parameters (in order) | Notes |
|---|---|---|
| `WE_DefaultConfig` | cfg: Config cluster (Adapt) | Fills the defaults. Wire an empty Config constant in, read the cluster out. |
| `WE_LoadProperties` | path: C string · cfg: Config cluster (Adapt) · host: C string out (min. size = hostLen) · hostLen: I32 = 256 | Call `WE_DefaultConfig` first. Missing keys keep their value. |
| `WE_SaveProperties` | path: C string · cfg: Config cluster · host: C string | |
| `WE_Open` | cfg: Config cluster · host: C string · handle: I32 *pointer* | Starts the DLL thread (not yet connected). Up to 8 instances. |
| `WE_Command` | handle: I32 · cmd: U32/I32 (enum) | Non-blocking (queued). |
| `WE_SetInputs` | handle: I32 · in: Inputs cluster | Call every loop iteration or on value change. |
| `WE_GetTelemetry` | handle: I32 · out: Telemetry cluster (Adapt) | **Call at least every `hostWatchdogMs`.** |
| `WE_GetTurbineInfo` | handle: I32 · out: Info cluster (Adapt) | |
| `WE_GetMessage` | handle: I32 · buf: C string out (min. size = len) · len: I32 = 256 | Status text for the status bar |
| `WE_LoadWindProfile` | handle: I32 · path: C string | CSV `time_s,wind_ms`, looped |
| `WE_DescribeStatus` | statusWord: I32 · buf: C string out · len: I32 = 256 | e.g. "OPERATING \| AT SETPOINT \| EXT1" |
| `WE_StateName` | state: I32 · buf: C string out · len: I32 = 64 | |
| `WE_GetStructSizes` | 4 × I32 *pointer* | 296 / 32 / 160 / 80 |
| `WE_Close` | handle: I32 | Sends 0x0476 (safe state), stops the thread. |
| `WE_Version` | (none) | 0x00010000 |

LabVIEW's *Import Shared Library* wizard can also read `src/windemu.h` and generate the wrapper VIs. Check
that it creates the clusters exactly as in §2.

## 4. Recommended VI structure

Make one small sub-VI per function (error in/out, handle in/out), plus three typedef clusters
(`WE_Config.ctl`, `WE_Inputs.ctl`, `WE_Telemetry.ctl`), then build the main VI like this:

```
┌─ Init ───────────────────────────────────────────────────────────────────────────────────────┐
│  Config constant ─► WE_DefaultConfig ─► WE_LoadProperties(path) ─► Config (front panel)       │
│                                          └─► Host string                                      │
│  Config, Host ─► WE_Open ─► handle ─► WE_GetTurbineInfo ─► Info indicator                     │
└──────────────────────────────────────────────────────────────────────────────────────────────┘
┌─ While loop (100 ms) ─────────────────────────────────────────────────────────────────────────┐
│  Event structure (timeout 100 ms)                                                             │
│    "Connect" / "Start" / "Stop" / "E-STOP" / "Reset" / "Gust" Value Change ─► WE_Command(cmd)│
│    Timeout case:                                                                              │
│      Inputs cluster ─► WE_SetInputs                                                           │
│      WE_GetTelemetry ─► Telemetry cluster ─► charts (windMs, rotorRpm, motorRpm, torque…)    │
│      WE_GetMessage   ─► status string;  WE_DescribeStatus(statusWord) ─► string               │
│  Stop when the "Exit" button is pressed                                                       │
└──────────────────────────────────────────────────────────────────────────────────────────────┘
┌─ Exit ───────────────────────────────────────────────────────────────────────────────────────┐
│  WE_Command(STOP) … wait until state = IDLE (or timeout) ─► WE_Close(handle)                 │
└──────────────────────────────────────────────────────────────────────────────────────────────┘
```

- Put **E-STOP** in its own event case so it is sent immediately. Mark it as a latched button with a red colour.
- Use a waveform chart with several plots: wind/pitch, rotor and motor speed, torque ref/actual.
- Config changes need `WE_Close` → `WE_Open` (the config is read at open). Inputs can be changed at any time.
- Logging: write the telemetry cluster with *Write Delimited Spreadsheet* or TDMS in the timeout case.

## 5. Safety behaviour inside the DLL

| Event | Reaction |
|---|---|
| Motor speed > `maxMotorRpm` | OFF3 emergency stop, state ESTOP |
| Drive trips (SW bit 3) | State FAULT, CW 0x0476; RESET sends the bit-7 edge |
| `maxCommErrors` Modbus errors in a row | State COMM_LOST. The drive stops on its own fieldbus timeout (51.17 / 30.18). |
| **LabVIEW stops calling `WE_GetTelemetry`** for `hostWatchdogMs` (VI aborted, hung, crashed) | Controlled stop: feather → OFF1 |
| `WE_Close` | CW 0x0476, then the socket is closed |
| Drive not in REMOTE within 1 s of START | State FAULT with a hint (98.02, LOC/REM key) |

If you abort a VI with the toolbar's Abort button, `WE_Close` isn't called. The host watchdog then stops the
turbine, and the thread keeps running until LabVIEW unloads the DLL (close the VI). Always end with your
Exit button.

## 6. Error codes (return value of every function)

| Code | Meaning |
|---|---|
| 0 | OK |
| −1 | Invalid handle (not opened / already closed) |
| −2 | NULL pointer (unwired cluster/string) |
| −3 | Could not create the real-time thread |
| −4 | File not found / not readable |
| −5 | All 8 instances in use (missing `WE_Close`?) |
| −6 | Argument out of range (e.g. port 0, inertia ≤ 0, period > 1000 ms) |

## 7. Troubleshooting

| Problem | Cause / fix |
|---|---|
| LabVIEW error 1097 / crash in CLFN | Cluster doesn't match: wrong order, a Boolean or U16 instead of I32, or a missing element. Compare the Flatten To String length with `WE_GetStructSizes`. |
| "Library not found or failed to load" | 32/64-bit mismatch. Use `windemu_*.dll` and keep both DLLs next to the VI. |
| Message "Connect failed: connect timed out" | Wrong IP / subnet, cable, or firewall. Check `ping <IP>`. |
| "Modbus exception 2 (illegal data address)" | Set `registerBase` = 1 |
| State FAULT "Drive not in REMOTE" | 98.02 = FIELDBUS, LOC/REM key on the panel |
| Turbine stops by itself, message "Host watchdog" | Your loop doesn't call `WE_GetTelemetry` often enough |

## 8. Test without a drive

1. `simulate = 1` uses the built-in ACS800 + bench model. Everything works offline.
2. For a protocol test: run `python3 tools/fake_reta01.py 502` on the PC, then set `simulate = 0` and Host
   = `127.0.0.1`. The fake server answers the state machine and echoes REF1/REF2 as ACT1/ACT2.
