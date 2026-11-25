package windemu.turbine;

/**
 * Blade pitch: PI speed limiter above rated rotor speed, rate-limited actuator,
 * storm shutdown (feather) above cut-out wind, and a manual mode.
 */
public final class PitchController {

    private final double kp, ki, rateLimit, maxDeg;
    private double integ;
    private double beta;          // actual pitch [deg]
    private boolean manual;
    private double manualSetpoint;

    public PitchController(double kp, double ki, double rateLimitDegS, double maxDeg) {
        this.kp = kp;
        this.ki = ki;
        this.rateLimit = rateLimitDegS;
        this.maxDeg = maxDeg;
    }

    /**
     * @param omegaRotor   rotor speed [rad/s]
     * @param omegaRated   rated rotor speed [rad/s]
     * @param feather      true → go to max pitch (cut-out / stop)
     */
    public double step(double dt, double omegaRotor, double omegaRated, boolean feather) {
        double target;
        if (feather) {
            target = maxDeg;
            integ = 0;
        } else if (manual) {
            target = manualSetpoint;
            integ = manualSetpoint;
        } else {
            // per-unit speed error; kp in deg/pu, ki in deg/(s·pu)
            double e = (omegaRotor - omegaRated) / omegaRated;
            integ = Math.max(0, Math.min(integ + ki * e * dt, maxDeg)); // clamped = anti wind-up
            target = Math.max(0, Math.min(integ + kp * e, maxDeg));
        }
        double d = target - beta;
        double maxStep = rateLimit * dt;
        beta += Math.max(-maxStep, Math.min(maxStep, d));
        return beta;
    }

    public double beta() { return beta; }
    public void setManual(boolean m) { manual = m; manualSetpoint = beta; }
    public boolean isManual() { return manual; }
    public void setManualSetpoint(double deg) { manualSetpoint = Math.max(0, Math.min(deg, maxDeg)); }
    public void reset() { integ = 0; beta = 0; }
}
