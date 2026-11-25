package windemu.control;

/** Immutable snapshot of one emulator cycle (shown in the GUI and written to the CSV log). */
public record Telemetry(
        double time,
        String state,
        String message,
        double wind,
        double pitchDeg,
        double lambda,
        double cp,
        double rotorRpm,
        double aeroTorqueRotorKNm,
        double aeroPowerKW,
        double motorRpm,
        double motorTorqueRefPct,
        double motorTorqueActPct,
        double motorSpeedRefRpm,
        double generatorTorqueEstPct,
        double recommendedGenTorquePct,
        int controlWord,
        int statusWord,
        int ref1,
        int ref2,
        int act1,
        int act2,
        double cycleMs,
        int commErrors) {

    public static final String CSV_HEADER =
            "time_s,state,wind_ms,pitch_deg,lambda,cp,rotor_rpm,aero_torque_kNm,aero_power_kW,"
          + "motor_rpm,torque_ref_pct,torque_act_pct,speed_ref_rpm,gen_torque_est_pct,gen_torque_rec_pct,"
          + "cw,sw,ref1,ref2,act1,act2,cycle_ms,comm_errors";

    public String toCsv() {
        return String.format(java.util.Locale.ROOT,
                "%.3f,%s,%.3f,%.2f,%.3f,%.4f,%.3f,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%.2f,%d",
                time, state, wind, pitchDeg, lambda, cp, rotorRpm, aeroTorqueRotorKNm, aeroPowerKW,
                motorRpm, motorTorqueRefPct, motorTorqueActPct, motorSpeedRefRpm, generatorTorqueEstPct,
                recommendedGenTorquePct, controlWord, statusWord, ref1, ref2, act1, act2, cycleMs, commErrors);
    }
}
