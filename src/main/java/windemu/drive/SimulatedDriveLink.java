package windemu.drive;

import windemu.config.EmulatorConfig;

import java.util.function.DoubleUnaryOperator;

import static windemu.drive.AbbDrivesProfile.*;

/**
 * Offline stand-in for "RETA-01 + ACS800 + motor + load generator".
 * Lets you test the whole emulator (state machine, scaling, safety, GUI) without hardware.
 *
 * Plant:  J_rig * dω/dt = T_motor - T_generator(ω) - B*ω
 * T_generator(ω) is supplied by the caller (normally the MPPT law of the load drive, scaled).
 */
public final class SimulatedDriveLink implements DriveLink {

    private final EmulatorConfig cfg;
    private final DoubleUnaryOperator generatorTorque; // motor-side Nm as a function of ω [rad/s]
    private final double tNom;
    private final double jRig;
    private final double bRig;

    // drive state
    private boolean tripped;
    private boolean switchOnInhibited = true;
    private boolean readyOn;
    private boolean operating;
    private boolean prevReset;
    private boolean remote;
    private boolean ext2;
    private boolean braking;      // OFF1 / OFF3 ramp stop in progress
    private String faultText = "";

    // plant state
    private double omega;      // rad/s
    private double torque;     // Nm (motor electromagnetic)
    private double speedInt;   // speed-controller integrator
    private long lastNs;

    public SimulatedDriveLink(EmulatorConfig cfg, DoubleUnaryOperator generatorTorque) {
        this.cfg = cfg;
        this.generatorTorque = generatorTorque;
        this.tNom = cfg.motorNomPowerW() / (cfg.motorNomRpm() * Math.PI / 30.0);
        this.jRig = cfg.rigInertia();
        this.bRig = cfg.rigFriction();
    }

    @Override
    public void connect() {
        lastNs = System.nanoTime();
    }

    @Override
    public synchronized Feedback exchange(int cw, int ref1, int ref2) {
        long now = System.nanoTime();
        double dt = lastNs == 0 ? 0.0 : Math.min((now - lastNs) * 1e-9, 0.2);
        lastNs = now;

        handleControlWord(cw);

        // integrate the mechanics in 1 ms sub-steps
        int steps = Math.max(1, (int) Math.ceil(dt / 0.001));
        double h = dt / steps;
        for (int i = 0; i < steps && dt > 0; i++) step(h, ref1, ref2);

        int sw = statusWord();
        int act1 = (int) Math.round(omega * 30 / Math.PI / cfg.act1FullScaleRpm() * 20000);
        int act2 = (int) Math.round(torque / tNom * 100 / cfg.act2FullScalePct() * 10000);
        return new Feedback(sw, act1, act2);
    }

    private void handleControlWord(int cw) {
        remote = has(cw, CW_REMOTE_CMD);
        ext2 = has(cw, CW_EXT_CTRL_LOC);
        boolean reset = has(cw, CW_RESET);
        if (reset && !prevReset && tripped) {
            tripped = false;
            faultText = "";
            switchOnInhibited = true;
        }
        prevReset = reset;
        if (tripped || !remote) { operating = false; readyOn = false; return; }

        if (!has(cw, CW_OFF2_CONTROL)) {               // OFF2: coast
            operating = false; readyOn = false; switchOnInhibited = true; braking = false;
            return;
        }
        if (!has(cw, CW_OFF3_CONTROL)) {               // OFF3: emergency ramp to zero
            if (operating) braking = true;
            operating = false; readyOn = false; switchOnInhibited = true;
            return;
        }
        if (!has(cw, CW_OFF1_CONTROL)) {               // OFF1: ramp stop
            if (operating) braking = true;
            switchOnInhibited = false;
            readyOn = true;
            operating = false;
            return;
        }
        if (switchOnInhibited) return; // need an OFF1 (0x0476) first
        operating = readyOn && has(cw, CW_INHIBIT_OPERATION);
    }

    private void step(double h, int ref1, int ref2) {
        double tCmd = 0;
        double tLim = tNom * cfg.maxTorquePct() / 100.0;
        if (operating) {
            boolean torqueMode = cfg.torqueOnExt2() ? ext2 : "TORQUE".equals(cfg.controlMode());
            if (torqueMode) {
                tCmd = DriveLink.toSigned(ref2) / 10000.0 * cfg.ref2FullScalePct() / 100.0 * tNom;
            } else {
                double wRef = DriveLink.toSigned(ref1) / 20000.0 * cfg.ref1FullScaleRpm() * Math.PI / 30;
                double e = wRef - omega;
                double kp = jRig * 60, ki = jRig * 600;  // ~60 rad/s speed-loop bandwidth
                speedInt = clamp(speedInt + ki * e * h, -tLim, tLim);
                tCmd = kp * e + speedInt;
            }
            tCmd = clamp(tCmd, -tLim, tLim);
            // DTC: very fast torque response (~2 ms)
            torque += (tCmd - torque) * Math.min(1.0, h / 0.002);
        } else if (braking && !tripped) {
            // ramp stop: decelerate with half the torque limit, then stop modulating
            torque = -Math.signum(omega) * tLim * 0.5;
            if (Math.abs(omega) < 0.5) { braking = false; torque = 0; omega = 0; }
            speedInt = 0;
        } else {
            torque = 0;
            speedInt = 0;
        }
        double tGen = generatorTorque.applyAsDouble(omega);
        double acc = (torque - tGen - bRig * omega) / jRig;
        omega += acc * h;
        if (!operating && Math.abs(omega) < 0.05 && Math.abs(tGen) < 1e-6) omega = 0;

        double rpm = Math.abs(omega * 30 / Math.PI);
        if (rpm > cfg.maxMotorRpm() * 1.15 && !tripped) {
            tripped = true;
            faultText = "OVERSPEED";
            operating = false;
        }
    }

    private int statusWord() {
        int sw = SW_OFF_2_STA | SW_OFF_3_STA;
        if (tripped) sw |= SW_TRIPPED;
        if (switchOnInhibited) sw |= SW_SWC_ON_INHIB;
        if (readyOn && !tripped) sw |= SW_RDY_ON;
        if (operating) sw |= SW_RDY_RUN | SW_RDY_REF | SW_AT_SETPOINT;
        if (remote) sw |= SW_REMOTE | SW_EXT_RUN_ENABLE;
        if (ext2) sw |= SW_EXT_CTRL_LOC;
        return sw;
    }

    @Override
    public synchronized int readParameter(int group, int index) {
        // 01.02 SPEED [rpm], 01.05 TORQUE [% * 10], 01.06 POWER [% * 10] – just enough for testing
        if (group == 1 && index == 2) return (int) Math.round(omega * 30 / Math.PI);
        if (group == 1 && index == 5) return (int) Math.round(torque / tNom * 1000);
        if (group == 1 && index == 6) return (int) Math.round(torque * omega / cfg.motorNomPowerW() * 1000);
        return 0;
    }

    @Override
    public void writeParameter(int group, int index, int value) { /* accepted, ignored */ }

    @Override
    public String name() {
        return "SIMULATED ACS800 (offline)" + (faultText.isEmpty() ? "" : " – fault: " + faultText);
    }

    @Override
    public void close() { }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
