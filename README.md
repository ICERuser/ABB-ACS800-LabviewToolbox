<div align="center">

# 🌬️ Wind Turbine Emulator for ABB ACS800

**Turn an ACS800 DTC drive and an induction motor into a real-time wind turbine emulator over Modbus/TCP (RETA-01).**

![Java](https://img.shields.io/badge/Java-17%2B-orange?logo=openjdk)
![Dependencies](https://img.shields.io/badge/dependencies-none-brightgreen)
![Protocol](https://img.shields.io/badge/protocol-Modbus%2FTCP-blue)
![Drive](https://img.shields.io/badge/drive-ABB%20ACS800%20%2B%20RETA--01-red)
![Status](https://img.shields.io/badge/status-tested%20in%20simulation-yellow)

<img src="docs/screenshot.png" alt="Emulator GUI" width="900">

</div>

---

## About

This project is for test benches where an **ABB ACS800** inverter drives an induction motor coupled to a
generator. A PC program reads the shaft speed and torque from the drive through the **RETA-01 Ethernet
adapter**. Every 20 ms it calculates how a real wind turbine rotor would behave in the current wind and sends
the result back to the drive as a speed or torque reference. The generator then "sees" a wind turbine, so
you can test generator control, MPPT, converters and grid-side behaviour in the lab without a real turbine.

Everything is plain Java: the Modbus/TCP client, the physics, the Swing GUI and the charts. There are no
external libraries. A built-in drive simulator lets you try it without any hardware.

## ✨ Features

- **Realistic wind:** mean speed set with a slider or loaded from a CSV profile, plus turbulence, gusts and lulls
- **Turbine aerodynamics:** Cp(λ, β) power-coefficient model, start-up torque and 3P tower-shadow ripple
- **Pitch control:** gain-scheduled PI speed limiter, start-up speed ramp (limits overshoot), rate limit, feathering at cut-out and on stop, manual mode
- **Scaling:** maps a MW-class turbine (default 1.5 MW, 77 m rotor) onto a kW lab motor, including its inertia
- **Two emulation modes**
  - **SPEED** (recommended): a virtual rotor runs on the PC, stable for any inertia ratio
  - **TORQUE**: observer-based inertia emulation with a DTC torque reference
- **ABB Drives profile:** full start sequence (0x0476 → 0x047F), fault reset, OFF1 ramp stop and OFF3 emergency stop
- **Safety supervision:** overspeed trip, torque limit and ramp, comms-loss detection, start timeout, REMOTE check
- **Operator GUI:** live values, decoded status word and three trend charts
- **Headless mode:** run from the command line and get a printed table
- **CSV logging:** every running cycle is logged
- **Offline simulator:** a virtual ACS800 + bench, so you can develop and demo without hardware
- **LabVIEW DLL:** native 32/64-bit `windemu` DLL with the same engine, configured and read through clusters (see [`labview/`](labview/LABVIEW.md))

## 🏗️ How it works

```
 ┌──────────────── PC (this program, 20 ms cycle) ────────────────┐          ┌─────────── test bench ───────────┐
 │ Wind model ─► Cp(λ,β) aerodynamics ─► scaling ─► REF1 / REF2   │ Modbus   │ RETA-01 ─► ACS800 (DTC) ─► Motor │
 │  mean+turb.     pitch controller       G, k_p    Control Word  │ ───TCP──►│                              │   │
 │  gusts, CSV     virtual rotor inertia            ◄─ Status Word │ ◄────────│ Status/ACT1 speed/ACT2 torque│   │
 │                 safety & state machine           ◄─ ACT1, ACT2 │          │        Generator ◄───────────┘   │
 └────────────────────────────────────────────────────────────────┘          │        + its own load drive/grid │
                                                                              └──────────────────────────────────┘
```

Turbine quantities are scaled to the motor shaft so that per-unit power, speed and stored energy are
preserved:

| Quantity | Scaling |
|---|---|
| Speed | ω_motor = G · ω_rotor |
| Power | P_motor = k_p · P_turbine |
| Torque | T_motor = T_rotor · k_p / G |
| Inertia | J_motor = J_rotor · k_p / G² |

With the default settings (a 1.5 MW turbine at 20 rpm on a 15 kW / 1475 rpm motor), G = 73.75 and
k_p = 0.008. The rated rotor torque of 716 kNm becomes 77.7 Nm at the motor (80 % of nominal), and the rotor
inertia of 4·10⁶ kgm² becomes 5.9 kgm².

## 🚀 Quick start

**Requirements:** JDK 17 or newer. Maven is optional.

```bash
git clone https://github.com/<your-user>/wind-emulator.git
cd wind-emulator
./build.sh                       # Windows: build.bat   |   or: mvn package
```

```bash
# 1) Try it without hardware (built-in ACS800 simulator)
java -jar wind-emulator.jar emulator.properties --sim

# 2) Connect to the real drive
java -jar wind-emulator.jar emulator.properties --real --host 192.168.0.16

# 3) No GUI: connect, start, run for 120 s, print values, stop
java -jar wind-emulator.jar emulator.properties --sim --headless 120
```

| Option | Meaning |
|---|---|
| `--sim` / `--real` | Use the built-in simulator or the real RETA-01 |
| `--host IP` | RETA-01 IP address |
| `--mode SPEED\|TORQUE` | Emulation mode |
| `--headless N` | Run N seconds without the GUI |

In the GUI: **Connect → Start**. Move the wind slider and try **Gust +5 m/s**. **Stop** feathers the blades
and waits for the rotor to slow down. **EMERGENCY STOP** sends OFF3.

All settings (IP address, motor data, turbine, scaling, pitch gains, safety limits) are in
[`emulator.properties`](emulator.properties), and each one is commented.

## 🧩 LabVIEW integration

The [`labview/`](labview/) folder contains a native C port of the emulator as **`windemu_32.dll` /
`windemu_64.dll`** for LabVIEW's Call Library Function Node. The settings from `emulator.properties` are a
**config cluster** of controls, with the RETA-01 IP as a string and the port in the cluster. Live values
come back as a **telemetry cluster** of indicators. The DLL runs its own 20 ms real-time thread and has a
host watchdog that stops the drive if the VI is aborted.

```
WE_DefaultConfig → WE_LoadProperties → WE_Open → loop { WE_SetInputs, WE_GetTelemetry, WE_Command } → WE_Close
```

The cluster layouts and node settings for each function are in **[LABVIEW.md](labview/LABVIEW.md)**.

## 🔌 Drive setup

<details>
<summary><b>RETA-01 fieldbus parameters</b></summary>

| Par. | Setting |
|---|---|
| 98.02 COMM. MODULE LINK | FIELDBUS |
| 98.07 COMM PROFILE | ABB DRIVES (the only profile on Modbus/TCP) |
| 51.02 COMM RATE | 0 = auto-negotiate |
| 51.03 DHCP | 0 = off (fixed IP recommended) |
| 51.04…51.07 | IP address, e.g. 192.168.0.16 |
| 51.08…51.15 | Subnet mask / gateway |
| 51.16 PROTOCOL | 0 = Modbus/TCP |
| 51.17 MODBUS TIMEOUT | e.g. 10 (= 1.0 s); don't use 0 |
| 51.27 FBA PAR REFRESH | REFRESH, to apply the changes |

</details>

<details>
<summary><b>ACS800 control parameters (standard control program)</b></summary>

| Par. | Setting |
|---|---|
| 99.04 MOTOR CTRL MODE | DTC |
| 99.02 APPLICATION MACRO | FACTORY (SPEED mode) or T CTRL (TORQUE mode) |
| 10.01 / 10.02 EXT1/EXT2 STRT/STP/DIR | COMM.CW |
| 10.03 REF DIRECTION | FORWARD |
| 11.02 EXT1/EXT2 SELECT | COMM.CW (bit 11 selects EXT2 = torque) |
| 11.03 EXT REF1 SELECT | COMM.REF, 11.05 = 1500 rpm |
| 11.06 EXT REF2 SELECT | COMM.REF, 11.08 = 100 % |
| 16.04 FAULT RESET SEL | COMM.CW |
| 30.18 COMM FLT FUNC | FAULT |
| 30.19 MAIN REF DS T-OUT | 1.0 s |
| 22.02 / 22.03 | Short ramps (≈ 0.5 s) for SPEED mode |
| 20.01 / 20.02 / 20.04 | Speed and torque limits of your bench |

> Check these names against the firmware manual for your version.

</details>

<details>
<summary><b>Modbus register map</b></summary>

| Register | Content | Direction |
|---|---|---|
| 40001 | Control Word | PC → drive |
| 40002 | REF1, speed (20000 = par. 11.05) | PC → drive |
| 40003 | REF2, torque (10000 = par. 11.08) | PC → drive |
| 40004 | Status Word | drive → PC |
| 40005 | ACT1, speed | drive → PC |
| 40006 | ACT2, torque (10000 = 100 %) | drive → PC |
| 4GGII | Drive parameter GG.II (e.g. 40102 = 01.02 SPEED) | read / write |

Function codes: FC16 + FC03 per cycle, or FC23 in a single telegram (`drive.useFc23=true`).
If the status word reads back as your reference, set `drive.registerBase=1`.

</details>

## 🧪 Testing done

| Test | Result |
|---|---|
| SPEED and TORQUE mode at 9 and 13 m/s (simulator) | Rated speed reached, pitch limits power, no oscillation |
| Modbus/TCP frames against an independent fake RETA-01 server | FC16 + FC03 and FC23 verified |
| Emergency stop → standstill → reset → restart | ✅ |
| LabVIEW DLLs (32/64-bit Windows) run via LoadLibrary: simulator in both modes + Modbus/TCP | ✅ |
| Start-up / gust overspeed, 8 cases (light/heavy rotor, 12/15 m/s) | Peak ≤ 9.5 % over rated |
| Real ACS800 + RETA-01 | ⏳ Not yet. Please report results! |

## 🗂️ Project structure

```
src/main/java/windemu/
├── Main.java                       CLI / GUI launcher
├── config/EmulatorConfig.java      settings from emulator.properties
├── modbus/ModbusTcpClient.java     Modbus/TCP master (FC03, 06, 16, 23)
├── drive/
│   ├── AbbDrivesProfile.java       control/status word bits and states
│   ├── RetaDriveLink.java          RETA-01 data sets and parameter access
│   └── SimulatedDriveLink.java     offline ACS800 + bench model
├── turbine/
│   ├── WindModel.java              wind: mean, turbulence, gusts, CSV
│   ├── CpModel.java                Cp(λ, β)
│   ├── PitchController.java        pitch PI + feathering
│   └── WindTurbineModel.java       aerodynamics + scaling
├── control/EmulatorController.java real-time loop, state machine, safety, logging
└── ui/                             Swing GUI and strip charts
labview/
├── src/windemu.h, windemu.c        native DLL (same engine) for LabVIEW
├── bin/windemu_32.dll, _64.dll     prebuilt DLLs
├── LABVIEW.md                      clusters, node settings, VI structure
└── tools/fake_reta01.py            fake RETA-01 Modbus server for tests
```

## ⚠️ Safety

This software is **not a safety function**. Before running a real bench, make sure you have:

- a hardware emergency stop (Safe Torque Off / main contactor)
- guarding of all rotating parts
- speed and torque limits inside the drive that hold even if the PC misbehaves

Start with the motor uncoupled and reduced limits, and pull the Ethernet cable once to confirm that the drive
trips on communication loss.

## 🤝 Contributing

Issues and pull requests are welcome, especially test reports from real ACS800 / RETA-01 setups,
other ABB drives (ACS550, ACS880 with FENA), and further turbine models.

## 📄 License

Add a license of your choice (e.g. MIT) as `LICENSE` before publishing.

## 📚 References

- [ABB RETA-01 Ethernet Adapter Module – User's Manual](https://library.e.abb.com/public/b3b3c13dcead1c3ec12572aa002977cc/en_RETA_01_um_d.pdf)
- ACS800 Standard Control Program – Firmware Manual (ABB)
- Slootweg, Polinder, Kling – *Representing wind turbine electrical generating systems in fundamental frequency simulations*, IEEE Trans. Energy Conversion, 2003 (Cp model)
