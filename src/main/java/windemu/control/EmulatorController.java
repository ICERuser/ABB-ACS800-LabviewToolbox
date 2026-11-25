package windemu.control;

import windemu.config.EmulatorConfig;
import windemu.drive.DriveLink;
import windemu.drive.RetaDriveLink;
import windemu.drive.SimulatedDriveLink;
import windemu.turbine.PitchController;
import windemu.turbine.WindModel;
import windemu.turbine.WindTurbineModel;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.locks.LockSupport;
import java.util.function.Consumer;

import static windemu.drive.AbbDrivesProfile.*;

/**
 * Real-time loop: wind → aerodynamics → scaled motor reference → RETA-01 → ACS800,
 * measured speed/torque ← ACS800, with the ABB Drives state machine and safety supervision.
 *
 * Two emulation modes:
 *  TORQUE – the drive runs in torque control (EXT2, REF2). The emulator sends the scaled aerodynamic
 *           torque minus an inertia-compensation term  (J_emu − J_rig)·dω/dt  so the rig "feels" the
 *           turbine's inertia. Good when J_emu / J_rig is small (≲ 3).
 *  SPEED  – the drive runs in speed control (EXT1, REF1). The emulator integrates the virtual rotor
 *           J_emu·dω/dt = T_aero − T_gen, estimating the generator torque from the drive's measured
 *           torque. Robust for any inertia ratio – recommended for MW-turbine emulation on a small rig.
 */
public final class EmulatorController {

    public enum State { DISCONNECTED, IDLE, STARTING, RUNNING, STOPPING, ESTOP, FAULT, COMM_LOST }

    private enum Cmd { CONNECT, DISCONNECT, START, STOP, ESTOP, RESET }

    private final EmulatorConfig cfg;
    private final WindTurbineModel turbine;
    private final WindModel wind;
    private final PitchController pitch;
    private final boolean torqueMode;
    private final ConcurrentLinkedQueue<Cmd> commands = new ConcurrentLinkedQueue<>();

    private volatile Consumer<Telemetry> listener = t -> { };
    private volatile Telemetry last;
    private volatile boolean shutdown;
    private volatile boolean logging = true;

    private DriveLink drive;
    private State state = State.DISCONNECTED;
    private String message = "Not connected";
    private double stateTimer;
    private int resetCycles;
    private int commErrors;

    // signals
    private double time;
    private double omegaMotorMeas, omegaMotorPrev, accelFilt;
    private double omegaRotorVirtual;     // SPEED mode
    private double rotorAngle;
    private double torqueRefNm;           // ramped
    private Feedback fb = new Feedback(0, 0, 0);
    private BufferedWriter log;

    private record Feedback(int sw, int act1, int act2) { }

    public EmulatorController(EmulatorConfig cfg) {
        this.cfg = cfg;
        this.turbine = new WindTurbineModel(cfg);
        this.wind = new WindModel(cfg.windMean(), cfg.turbulence(), cfg.turbLengthScale());
        this.pitch = new PitchController(cfg.pitchKp(), cfg.pitchKi(), cfg.pitchRateDegS(), cfg.pitchMaxDeg());
        this.torqueMode = "TORQUE".equals(cfg.controlMode());
        if (!cfg.windProfileCsv().isBlank()) {
            try {
                wind.loadProfile(Path.of(cfg.windProfileCsv()));
            } catch (IOException e) {
                System.err.println("Wind profile not loaded: " + e.getMessage());
            }
        }
    }

    // ------------------------------------------------------------------ public API (thread safe)
    public void connect()    { commands.add(Cmd.CONNECT); }
    public void disconnect() { commands.add(Cmd.DISCONNECT); }
    public void start()      { commands.add(Cmd.START); }
    public void stop()       { commands.add(Cmd.STOP); }
    public void emergencyStop() { commands.add(Cmd.ESTOP); }
    public void reset()      { commands.add(Cmd.RESET); }
    public void setListener(Consumer<Telemetry> l) { listener = l; }
    public Telemetry last()  { return last; }
    public WindModel wind()  { return wind; }
    public PitchController pitch() { return pitch; }
    public WindTurbineModel turbine() { return turbine; }
    public boolean isTorqueMode() { return torqueMode; }
    public void setLogging(boolean on) { logging = on; }

    public void shutdown() { shutdown = true; }

    // ------------------------------------------------------------------ real-time loop
    public void run() {
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        final long periodNs = cfg.cyclePeriodMs() * 1_000_000L;
        long next = System.nanoTime();
        long prev = next;
        openLog();
        while (!shutdown) {
            long now = System.nanoTime();
            double dt = Math.max(1e-4, Math.min((now - prev) * 1e-9, 0.25));
            prev = now;
            cycle(dt);
            double cycleMs = (System.nanoTime() - now) / 1e6;
            publish(cycleMs);

            next += periodNs;
            long sleep = next - System.nanoTime();
            if (sleep > 0) LockSupport.parkNanos(sleep);
            else next = System.nanoTime(); // overrun: re-synchronise
        }
        // leave the drive in a safe state
        if (drive != null) {
            try { drive.exchange(CW_SHUTDOWN, 0, 0); } catch (IOException ignored) { }
            drive.close();
        }
        closeLog();
    }

    private void cycle(double dt) {
        time += dt;
        stateTimer += dt;
        handleCommands();

        double v = wind.step(dt);

        // ---------- measured shaft (motor side)
        omegaMotorMeas = act1ToOmega(fb.act1);
        double accRaw = (omegaMotorMeas - omegaMotorPrev) / dt;
        omegaMotorPrev = omegaMotorMeas;
        accelFilt += (accRaw - accelFilt) * Math.min(1.0, dt / cfg.accelFilterTau());
        double tMotorMeas = fb.act2 / 10000.0 * cfg.act2FullScalePct() / 100.0 * turbine.motorNomTorque;

        boolean active = state == State.RUNNING || state == State.STOPPING;
        if (!active) omegaRotorVirtual = turbine.motorToRotorSpeed(omegaMotorMeas);

        double omegaRotor = torqueMode ? turbine.motorToRotorSpeed(omegaMotorMeas) : omegaRotorVirtual;

        // ---------- pitch & aerodynamics
        boolean feather = state != State.RUNNING || v > cfg.cutOutWind();
        double beta = pitch.step(dt, omegaRotor, turbine.ratedOmegaRotor, feather);
        rotorAngle = (rotorAngle + omegaRotor * dt) % (2 * Math.PI);
        double tAero = turbine.aeroTorque(v, omegaRotor, beta) * turbine.towerShadowFactor(rotorAngle);
        double tAeroMotor = turbine.rotorToMotorTorque(tAero);

        // generator torque estimate (motor side): T_motor = J_rig·α + B·ω + T_gen
        double tGenEst = tMotorMeas - cfg.rigInertia() * accelFilt - cfg.rigFriction() * omegaMotorMeas;

        // ---------- references
        double torqueCmdNm = 0;
        double speedCmd = 0;
        if (active) {
            if (torqueMode) {
                // Observer-based inertia emulation. Wanted: J_emu·α = T_aero − T_gen.
                // Rig: J_rig·α = T_ref − T_gen − B·ω  ⇒  T_ref = T_gen + B·ω + r·(T_aero − T_gen), r = J_rig/J_emu
                // Unlike "T_aero − (J_emu−J_rig)·α" this stays stable for large inertia ratios.
                double jRig = cfg.rigInertia();
                double jEmu = jRig + cfg.inertiaCompGain() * Math.max(0, turbine.emulatedInertiaMotorSide - jRig);
                double r = Math.min(1.0, jRig / jEmu);
                torqueCmdNm = tGenEst + cfg.rigFriction() * omegaMotorMeas + r * (tAeroMotor - tGenEst);
            } else {
                // virtual rotor: J_rotor·dω/dt = T_aero − T_gen (all on the rotor side)
                double tGenRotor = turbine.motorToRotorTorque(tGenEst);
                omegaRotorVirtual += (tAero - tGenRotor) / turbine.rotorInertia * dt;
                omegaRotorVirtual = Math.max(0, omegaRotorVirtual);
                speedCmd = turbine.rotorToMotorSpeed(omegaRotorVirtual);
            }
        }
        double tMax = turbine.motorNomTorque * cfg.maxTorquePct() / 100.0;
        torqueCmdNm = Math.max(-tMax, Math.min(tMax, torqueCmdNm));
        double rampStep = turbine.motorNomTorque * cfg.torqueRampPctPerS() / 100.0 * dt;
        torqueRefNm += Math.max(-rampStep, Math.min(rampStep, torqueCmdNm - torqueRefNm));
        if (!active) torqueRefNm = 0;

        // ---------- state machine -> control word
        int cw = stateMachine();
        int ext = torqueMode && cfg.torqueOnExt2() ? CW_EXT_CTRL_LOC : 0;
        if (cw == CW_RUN || cw == CW_SWITCH_ON) cw |= ext;

        int ref1 = 0, ref2 = 0;
        if (active) {
            if (torqueMode) {
                ref2 = (int) Math.round(torqueRefNm / turbine.motorNomTorque * 100 / cfg.ref2FullScalePct() * 10000);
            } else {
                double rpm = Math.min(speedCmd * 30 / Math.PI, cfg.maxMotorRpm());
                ref1 = (int) Math.round(rpm / cfg.ref1FullScaleRpm() * 20000);
            }
        }

        // ---------- safety: overspeed
        double motorRpm = omegaMotorMeas * 30 / Math.PI;
        if (Math.abs(motorRpm) > cfg.maxMotorRpm() && state != State.ESTOP && state != State.DISCONNECTED) {
            enter(State.ESTOP, String.format("OVERSPEED %.0f rpm > %.0f rpm – OFF3 issued", motorRpm, cfg.maxMotorRpm()));
            cw = CW_EMERGENCY;
            ref1 = ref2 = 0;
        }

        // ---------- exchange with the drive
        if (drive != null && state != State.DISCONNECTED && state != State.COMM_LOST) {
            try {
                DriveLink.Feedback f = drive.exchange(cw, DriveLink.toRegister(ref1), DriveLink.toRegister(ref2));
                fb = new Feedback(f.statusWord(), f.act1(), f.act2());
                commErrors = 0;
            } catch (IOException e) {
                commErrors++;
                if (commErrors >= cfg.maxCommErrors()) {
                    enter(State.COMM_LOST, "Communication lost: " + e.getMessage()
                            + " – drive stops on its own fieldbus timeout (51.17 / 30.18)");
                    drive.close();
                }
            }
        }

        lastCw = cw; lastRef1 = ref1; lastRef2 = ref2; lastActive = active;
        lastWind = v; lastBeta = beta; lastTAero = tAero; lastOmegaRotor = omegaRotor;
        lastTGenEst = tGenEst; lastTMotorMeas = tMotorMeas; lastSpeedCmd = speedCmd;
    }

    // scratch values for telemetry
    private int lastCw, lastRef1, lastRef2;
    private boolean lastActive;
    private double lastWind, lastBeta, lastTAero, lastOmegaRotor, lastTGenEst, lastTMotorMeas, lastSpeedCmd;

    private int stateMachine() {
        int sw = fb.sw;
        switch (state) {
            case DISCONNECTED, COMM_LOST:
                return CW_SHUTDOWN;
            case IDLE:
                if (has(sw, SW_TRIPPED)) enter(State.FAULT, "Drive fault (status " + hex(sw) + ")");
                return CW_SHUTDOWN;
            case STARTING:
                if (has(sw, SW_TRIPPED)) { enter(State.FAULT, "Drive tripped during start"); return CW_SHUTDOWN; }
                if (!has(sw, SW_REMOTE) && stateTimer > 1.0) {
                    enter(State.FAULT, "Drive not in REMOTE – check 98.02 COMM. MODULE LINK and local/remote key");
                    return CW_SHUTDOWN;
                }
                if (stateTimer > 8.0) { enter(State.FAULT, "Start timeout, status " + describe(sw)); return CW_SHUTDOWN; }
                if (has(sw, SW_RDY_REF)) { enter(State.RUNNING, "Emulating turbine"); return CW_RUN; }
                if (has(sw, SW_RDY_ON) && !has(sw, SW_SWC_ON_INHIB)) return CW_RUN; // 0x047F
                return CW_SHUTDOWN;                                                   // 0x0476
            case RUNNING:
                if (has(sw, SW_TRIPPED)) { enter(State.FAULT, "Drive fault while running (" + hex(sw) + ")"); return CW_SHUTDOWN; }
                if (!has(sw, SW_RDY_REF) && stateTimer > 0.5) {
                    enter(State.FAULT, "Drive left OPERATING state: " + describe(sw));
                    return CW_SHUTDOWN;
                }
                return CW_RUN;
            case STOPPING:
                // blades feathered by the pitch controller, wait for the rotor to slow down, then OFF1
                if (has(sw, SW_TRIPPED)) { enter(State.FAULT, "Drive fault while stopping"); return CW_SHUTDOWN; }
                double pu = Math.abs(omegaMotorMeas) / (turbine.ratedOmegaRotor * turbine.gearRatio);
                if (pu < 0.05 || stateTimer > 60) { enter(State.IDLE, "Stopped (OFF1)"); return CW_SHUTDOWN; }
                return CW_RUN;
            case ESTOP:
                return CW_EMERGENCY;
            case FAULT:
                if (resetCycles > 0) {
                    resetCycles--;
                    if (resetCycles == 0) {
                        if (has(sw, SW_TRIPPED)) message = "Reset sent but drive still tripped – check drive panel";
                        else enter(State.IDLE, "Fault reset");
                    }
                    return resetCycles > 3 ? CW_SHUTDOWN : CW_RESET_CMD; // 0 -> 1 edge on bit 7
                }
                return CW_SHUTDOWN;
            default:
                return CW_SHUTDOWN;
        }
    }

    private void handleCommands() {
        Cmd c;
        while ((c = commands.poll()) != null) {
            switch (c) {
                case CONNECT -> doConnect();
                case DISCONNECT -> {
                    if (drive != null) {
                        try { drive.exchange(CW_SHUTDOWN, 0, 0); } catch (IOException ignored) { }
                        drive.close();
                    }
                    drive = null;
                    fb = new Feedback(0, 0, 0);
                    enter(State.DISCONNECTED, "Disconnected");
                }
                case START -> {
                    if (state == State.IDLE) {
                        enter(State.STARTING, "Starting: 0x0476 → 0x047F");
                    }
                }
                case STOP -> {
                    if (state == State.RUNNING) enter(State.STOPPING, "Stopping: feathering blades");
                    else if (state == State.STARTING) enter(State.IDLE, "Start aborted");
                }
                case ESTOP -> {
                    if (state != State.DISCONNECTED && state != State.COMM_LOST)
                        enter(State.ESTOP, "EMERGENCY STOP (OFF3) – press Reset to recover");
                }
                case RESET -> {
                    if (state == State.ESTOP) {
                        if (Math.abs(omegaMotorMeas) < 1) enter(State.FAULT, "Resetting…");
                        else { message = "Wait for standstill before reset"; break; }
                    }
                    if (state == State.FAULT) resetCycles = 8;
                    if (state == State.COMM_LOST) doConnect();
                }
            }
        }
    }

    private void doConnect() {
        try {
            if (drive != null) drive.close();
            drive = cfg.simulateDrive()
                    ? new SimulatedDriveLink(cfg, turbine::generatorTorqueMotorSide)
                    : new RetaDriveLink(cfg);
            drive.connect();
            commErrors = 0;
            DriveLink.Feedback f = drive.exchange(CW_SHUTDOWN, 0, 0);
            fb = new Feedback(f.statusWord(), f.act1(), f.act2());
            enter(has(fb.sw, SW_TRIPPED) ? State.FAULT : State.IDLE, "Connected to " + drive.name());
        } catch (IOException e) {
            if (drive != null) drive.close();
            drive = null;
            enter(State.DISCONNECTED, "Connect failed: " + e.getMessage());
        }
    }

    private void enter(State s, String msg) {
        state = s;
        message = msg;
        stateTimer = 0;
        System.out.printf("[%8.2f s] %-12s %s%n", time, s, msg);
    }

    private double act1ToOmega(int act1) {
        return act1 / 20000.0 * cfg.act1FullScaleRpm() * Math.PI / 30;
    }

    private void publish(double cycleMs) {
        double lambda = turbine.tipSpeedRatio(lastWind, lastOmegaRotor);
        double tn = turbine.motorNomTorque;
        Telemetry t = new Telemetry(
                time, state.name(), message + (drive != null && cfg.simulateDrive() ? "  [SIM]" : ""),
                lastWind, lastBeta, lambda, turbine.cp(lambda, lastBeta),
                lastOmegaRotor * 30 / Math.PI, lastTAero / 1e3, lastTAero * lastOmegaRotor / 1e3,
                omegaMotorMeas * 30 / Math.PI,
                (torqueMode ? torqueRefNm : lastActive ? turbine.rotorToMotorTorque(lastTAero) : 0) / tn * 100,
                lastTMotorMeas / tn * 100,
                lastSpeedCmd * 30 / Math.PI,
                lastTGenEst / tn * 100,
                turbine.generatorTorqueMotorSide(omegaMotorMeas) / tn * 100,
                lastCw, fb.sw, (short) lastRef1, (short) lastRef2, fb.act1, fb.act2, cycleMs, commErrors);
        last = t;
        listener.accept(t);
        if (log != null && logging && (state == State.RUNNING || state == State.STOPPING || state == State.STARTING)) {
            try { log.write(t.toCsv()); log.newLine(); } catch (IOException ignored) { }
        }
    }

    private void openLog() {
        String f = cfg.logFile();
        if (f == null || f.isBlank()) return;
        try {
            log = Files.newBufferedWriter(Path.of(f));
            log.write(Telemetry.CSV_HEADER);
            log.newLine();
        } catch (IOException e) {
            System.err.println("Cannot open log " + f + ": " + e.getMessage());
        }
    }

    private void closeLog() {
        if (log != null) try { log.close(); } catch (IOException ignored) { }
    }

    public State state() { return state; }
}
