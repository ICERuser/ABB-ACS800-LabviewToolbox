package windemu.turbine;

/**
 * Power coefficient Cp(λ, β) – the widely used Heier / Slootweg analytical approximation:
 *
 *   Cp = c1 (c2/λi − c3 β − c4) e^(−c5/λi) + c6 λ
 *   1/λi = 1/(λ + 0.08 β) − 0.035/(β³ + 1)
 *
 * With the default coefficients Cp,max ≈ 0.48 at λ ≈ 8.1, β = 0.
 */
public final class CpModel {

    private final double c1, c2, c3, c4, c5, c6;

    public CpModel() {
        this(0.5176, 116, 0.4, 5, 21, 0.0068);
    }

    public CpModel(double c1, double c2, double c3, double c4, double c5, double c6) {
        this.c1 = c1; this.c2 = c2; this.c3 = c3; this.c4 = c4; this.c5 = c5; this.c6 = c6;
    }

    public double cp(double lambda, double betaDeg) {
        double l = Math.max(lambda, 1e-3);
        double inv = 1.0 / (l + 0.08 * betaDeg) - 0.035 / (betaDeg * betaDeg * betaDeg + 1.0);
        double cp = c1 * (c2 * inv - c3 * betaDeg - c4) * Math.exp(-c5 * inv) + c6 * l;
        return Math.max(-0.2, Math.min(cp, 0.593)); // never beyond the Betz limit
    }

    /**
     * Torque coefficient Cq = Cp/λ. The analytical Cp fit is not valid near standstill, so below λ = 1
     * Cq is held at its λ = 1 value: this gives a finite start-up torque (larger with pitched blades,
     * as on a real rotor) instead of a singularity.
     */
    public double cq(double lambda, double betaDeg) {
        double l = Math.max(lambda, 1.0);
        return cp(l, betaDeg) / l;
    }

    /** Numerically locate the optimum tip-speed ratio and Cp at β = 0. */
    public double[] optimum() {
        double bestL = 0, bestCp = -1;
        for (double l = 1; l <= 15; l += 0.01) {
            double c = cp(l, 0);
            if (c > bestCp) { bestCp = c; bestL = l; }
        }
        return new double[]{bestL, bestCp};
    }
}
