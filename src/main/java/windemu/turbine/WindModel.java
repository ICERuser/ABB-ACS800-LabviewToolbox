package windemu.turbine;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Hub-height wind speed:  v(t) = mean(t) + turbulence(t) + gust(t)
 *
 *  - mean:        manual set-point (GUI slider) or a time series loaded from CSV ("time_s,wind_ms")
 *  - turbulence:  first-order (Gauss-Markov) process with time constant L / v_mean and
 *                 standard deviation  I * v_mean  (a cheap approximation of a Kaimal spectrum)
 *  - gust:        IEC 61400-1 style "1-cos" (Mexican hat simplified) extreme operating gust
 */
public final class WindModel {

    private final Random rnd = new Random();
    private double meanSetpoint;
    private double turbulenceIntensity;
    private final double lengthScale;

    private double turb;               // current turbulence component
    private double time;               // s since start

    private double gustStart = -1, gustDuration, gustAmplitude;

    private List<double[]> profile;    // {time, speed}
    private boolean useProfile;
    private double profileTime;

    public WindModel(double mean, double intensity, double lengthScale) {
        this.meanSetpoint = mean;
        this.turbulenceIntensity = intensity;
        this.lengthScale = lengthScale;
    }

    public synchronized double step(double dt) {
        time += dt;
        double mean = meanSetpoint;
        if (useProfile && profile != null && !profile.isEmpty()) {
            profileTime += dt;
            mean = interpolateProfile(profileTime);
        }

        // Gauss-Markov turbulence
        double sigma = turbulenceIntensity * Math.max(mean, 0.5);
        double tau = lengthScale / Math.max(mean, 1.0);
        double a = Math.exp(-dt / tau);
        turb = a * turb + sigma * Math.sqrt(1 - a * a) * rnd.nextGaussian();

        double gust = 0;
        if (gustStart >= 0) {
            double tg = time - gustStart;
            if (tg <= gustDuration) {
                gust = gustAmplitude * 0.5 * (1 - Math.cos(2 * Math.PI * tg / gustDuration));
            } else {
                gustStart = -1;
            }
        }
        return Math.max(0, mean + turb + gust);
    }

    /** Trigger a 1-cos gust (e.g. +6 m/s over 10.5 s as in IEC EOG). */
    public synchronized void triggerGust(double amplitude, double duration) {
        gustAmplitude = amplitude;
        gustDuration = duration;
        gustStart = time;
    }

    public synchronized void setMean(double v) { meanSetpoint = v; }
    public synchronized double getMean() { return meanSetpoint; }
    public synchronized void setTurbulenceIntensity(double i) { turbulenceIntensity = i; }
    public synchronized double getTurbulenceIntensity() { return turbulenceIntensity; }

    public synchronized void loadProfile(Path csv) throws IOException {
        List<double[]> pts = new ArrayList<>();
        for (String line : Files.readAllLines(csv)) {
            line = line.trim();
            if (line.isEmpty() || line.startsWith("#") || Character.isLetter(line.charAt(0))) continue;
            String[] f = line.split("[,;\\s]+");
            pts.add(new double[]{Double.parseDouble(f[0]), Double.parseDouble(f[1])});
        }
        if (pts.isEmpty()) throw new IOException("No data in " + csv);
        profile = pts;
        profileTime = 0;
        useProfile = true;
    }

    public synchronized void setUseProfile(boolean on) {
        useProfile = on && profile != null;
        profileTime = 0;
    }

    public synchronized boolean isUsingProfile() { return useProfile; }

    private double interpolateProfile(double t) {
        double end = profile.get(profile.size() - 1)[0];
        if (end > 0) t = t % end; // loop the profile
        for (int i = 1; i < profile.size(); i++) {
            double[] a = profile.get(i - 1), b = profile.get(i);
            if (t <= b[0]) {
                double f = (t - a[0]) / Math.max(b[0] - a[0], 1e-9);
                return a[1] + f * (b[1] - a[1]);
            }
        }
        return profile.get(profile.size() - 1)[1];
    }
}
