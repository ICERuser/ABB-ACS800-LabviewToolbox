package windemu.turbine;

import windemu.config.EmulatorConfig;

/**
 * Aerodynamic rotor + scaling from the (large) emulated turbine to the (small) lab motor.
 *
 * Scaling (power-preserving in per-unit):
 *   ω_motor = G · ω_rotor          G  = n_motor,nom / n_rotor,rated
 *   P_motor = k_p · P_rotor        k_p = usage · P_motor,nom / P_turbine,rated
 *   T_motor = T_rotor · k_p / G
 *   J_motor = J_rotor · k_p / G²   (keeps the stored kinetic energy in per-unit, i.e. same time constants)
 */
public final class WindTurbineModel {

    public final double radius, rho, ratedPower, ratedOmegaRotor, rotorInertia;
    public final double gearRatio, powerScale, emulatedInertiaMotorSide;
    public final double lambdaOpt, cpMax, kOpt, ratedRotorTorque;
    public final double motorNomTorque;
    private final CpModel cpModel = new CpModel();
    private final double towerShadowAmp;
    private final int blades;

    public WindTurbineModel(EmulatorConfig c) {
        radius = c.rotorRadius();
        rho = c.airDensity();
        ratedPower = c.turbineRatedPowerW();
        ratedOmegaRotor = c.turbineRatedRpm() * Math.PI / 30;
        rotorInertia = c.turbineInertia();
        gearRatio = c.motorNomRpm() / c.turbineRatedRpm();
        powerScale = c.powerUsage() * c.motorNomPowerW() / ratedPower;
        emulatedInertiaMotorSide = rotorInertia * powerScale / (gearRatio * gearRatio);
        motorNomTorque = c.motorNomPowerW() / (c.motorNomRpm() * Math.PI / 30);
        towerShadowAmp = c.towerShadow();
        blades = c.bladeCount();

        double[] opt = cpModel.optimum();
        lambdaOpt = opt[0];
        cpMax = opt[1];
        // MPPT: T = k_opt ω²  with  k_opt = ½ ρ π R⁵ Cp,max / λopt³
        kOpt = 0.5 * rho * Math.PI * Math.pow(radius, 5) * cpMax / Math.pow(lambdaOpt, 3);
        ratedRotorTorque = ratedPower / ratedOmegaRotor;
    }

    public double tipSpeedRatio(double wind, double omegaRotor) {
        return wind < 0.1 ? 0 : omegaRotor * radius / wind;
    }

    public double cp(double lambda, double beta) { return cpModel.cp(lambda, beta); }

    /** Aerodynamic torque on the rotor shaft [Nm]: T = ½ ρ π R³ v² Cq(λ,β). */
    public double aeroTorque(double wind, double omegaRotor, double betaDeg) {
        if (wind < 0.1) return 0;
        double lambda = tipSpeedRatio(wind, Math.max(omegaRotor, 0));
        double t = 0.5 * rho * Math.PI * Math.pow(radius, 3) * wind * wind * cpModel.cq(lambda, betaDeg);
        // braking torque fades out towards standstill – the wind never drives the rotor backwards
        if (t < 0) t *= Math.min(1.0, Math.max(0, omegaRotor) / (0.05 * ratedOmegaRotor));
        return t;
    }

    /** Tower-shadow torque modulation (dip each time a blade passes the tower). */
    public double towerShadowFactor(double rotorAngle) {
        if (towerShadowAmp <= 0) return 1;
        double c = 0.5 + 0.5 * Math.cos(blades * rotorAngle);
        return 1 - towerShadowAmp * Math.pow(c, 6);
    }

    public double rotorToMotorTorque(double tRotor) { return tRotor * powerScale / gearRatio; }
    public double motorToRotorTorque(double tMotor) { return tMotor * gearRatio / powerScale; }
    public double motorToRotorSpeed(double wMotor)  { return wMotor / gearRatio; }
    public double rotorToMotorSpeed(double wRotor)  { return wRotor * gearRatio; }

    /**
     * Reference load law for the generator side (MPPT region 2, constant torque region 3).
     * Used by the offline simulation and shown in the GUI as the recommended load-drive torque.
     */
    public double generatorTorqueRotor(double omegaRotor) {
        if (omegaRotor <= 0) return 0;
        double t = kOpt * omegaRotor * omegaRotor;
        return Math.min(t, ratedRotorTorque);
    }

    public double generatorTorqueMotorSide(double omegaMotor) {
        return rotorToMotorTorque(generatorTorqueRotor(motorToRotorSpeed(omegaMotor)));
    }

    public String summary() {
        return String.format(
                "Turbine: R=%.1f m, P=%.0f kW, n_rated=%.1f rpm, λopt=%.2f, Cp,max=%.3f, k_opt=%.3g Nm/(rad/s)²%n"
              + "Scaling: G=%.2f, k_p=%.4g, T_rotor,rated=%.0f kNm -> %.1f Nm at motor (%.0f %% of T_nom=%.1f Nm)%n"
              + "Inertia: J_rotor=%.3g kgm² -> %.3f kgm² at motor shaft",
                radius, ratedPower / 1e3, ratedOmegaRotor * 30 / Math.PI, lambdaOpt, cpMax, kOpt,
                gearRatio, powerScale, ratedRotorTorque / 1e3, rotorToMotorTorque(ratedRotorTorque),
                rotorToMotorTorque(ratedRotorTorque) / motorNomTorque * 100, motorNomTorque,
                rotorInertia, emulatedInertiaMotorSide);
    }
}
