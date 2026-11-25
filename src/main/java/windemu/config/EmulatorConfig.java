package windemu.config;

import java.io.IOException;
import java.io.InputStream;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Properties;

/**
 * All tunable settings of the emulator, loaded from a .properties file.
 * Every value has a sensible default so the program also runs without a file.
 */
public final class EmulatorConfig {

    private final Properties p = new Properties();

    public static EmulatorConfig load(Path file) throws IOException {
        EmulatorConfig c = new EmulatorConfig();
        if (file != null && Files.exists(file)) {
            try (Reader r = Files.newBufferedReader(file)) {
                c.p.load(r);
            }
        } else {
            try (InputStream in = EmulatorConfig.class.getResourceAsStream("/emulator.properties")) {
                if (in != null) c.p.load(in);
            }
        }
        return c;
    }

    /** Override a value (command-line options). */
    public void set(String key, String value) { p.setProperty(key, value); }

    public String str(String key, String def) {
        String v = p.getProperty(key);
        return v == null ? def : v.trim();
    }

    public double dbl(String key, double def) {
        String v = p.getProperty(key);
        return v == null || v.isBlank() ? def : Double.parseDouble(v.trim());
    }

    public int integer(String key, int def) {
        String v = p.getProperty(key);
        if (v == null || v.isBlank()) return def;
        v = v.trim();
        return v.startsWith("0x") || v.startsWith("0X") ? Integer.parseInt(v.substring(2), 16) : Integer.parseInt(v);
    }

    public boolean bool(String key, boolean def) {
        String v = p.getProperty(key);
        return v == null || v.isBlank() ? def : Boolean.parseBoolean(v.trim());
    }

    // ---------------------------------------------------------------- connection
    public String  driveHost()          { return str("drive.host", "192.168.0.16"); }
    public int     drivePort()          { return integer("drive.port", 502); }
    public int     unitId()             { return integer("drive.unitId", 1); }
    public int     ioTimeoutMs()        { return integer("drive.timeoutMs", 200); }
    /** Modbus PDU address of register 40001 (normally 0). */
    public int     registerBase()       { return integer("drive.registerBase", 0); }
    /** Register number (1-based, 4xxxx minus 40000) of data word 1.1 = Control Word. */
    public int     regControlWord()     { return integer("drive.reg.controlWord", 1); }
    public int     regStatusWord()      { return integer("drive.reg.statusWord", 4); }
    public boolean useReadWriteFc23()   { return bool("drive.useFc23", false); }
    public boolean simulateDrive()      { return bool("drive.simulate", true); }

    // ---------------------------------------------------------------- drive scaling
    public String  controlMode()        { return str("emulator.controlMode", "TORQUE").toUpperCase(); }
    public boolean torqueOnExt2()       { return bool("drive.torqueOnExt2", true); }
    public double  motorNomPowerW()     { return dbl("motor.nominalPowerW", 15000); }
    public double  motorNomRpm()        { return dbl("motor.nominalRpm", 1475); }
    public double  rigInertia()         { return dbl("motor.rigInertiaKgm2", 0.35); }
    public double  rigFriction()        { return dbl("motor.rigViscousNmPerRadS", 0.02); }
    /** rpm that REF1 = 20000 represents (ACS800 par. 11.05 EXT REF1 MAXIMUM). */
    public double  ref1FullScaleRpm()   { return dbl("drive.ref1FullScaleRpm", 1500); }
    /** % of nominal torque that REF2 = 10000 represents (ACS800 par. 11.08 EXT REF2 MAXIMUM). */
    public double  ref2FullScalePct()   { return dbl("drive.ref2FullScalePct", 100); }
    /** rpm that ACT1 = 20000 represents (speed scaling of the drive). */
    public double  act1FullScaleRpm()   { return dbl("drive.act1FullScaleRpm", 1500); }
    /** % of nominal torque that ACT2 = 10000 represents. */
    public double  act2FullScalePct()   { return dbl("drive.act2FullScalePct", 100); }

    // ---------------------------------------------------------------- turbine
    public double  rotorRadius()        { return dbl("turbine.rotorRadiusM", 35); }
    public double  airDensity()         { return dbl("turbine.airDensity", 1.225); }
    public double  turbineRatedPowerW() { return dbl("turbine.ratedPowerW", 1.5e6); }
    public double  turbineRatedRpm()    { return dbl("turbine.ratedRotorRpm", 20.0); }
    public double  turbineInertia()     { return dbl("turbine.rotorInertiaKgm2", 4.0e6); }
    public double  cutInWind()          { return dbl("turbine.cutInWind", 3.0); }
    public double  cutOutWind()         { return dbl("turbine.cutOutWind", 25.0); }
    public double  towerShadow()        { return dbl("turbine.towerShadowAmplitude", 0.03); }
    public int     bladeCount()         { return integer("turbine.blades", 3); }

    /** Fraction of the motor rating used for the turbine's rated power (head-room for gusts). */
    public double  powerUsage()         { return dbl("scaling.powerUsage", 0.8); }
    public double  inertiaCompGain()    { return dbl("scaling.inertiaCompensationGain", 1.0); }
    public double  accelFilterTau()     { return dbl("scaling.accelFilterTauS", 0.05); }

    // ---------------------------------------------------------------- pitch
    public double  pitchKp()            { return dbl("pitch.kpDegPerPu", 60.0); }
    public double  pitchKi()            { return dbl("pitch.kiDegPerSPerPu", 30.0); }
    public double  pitchRateDegS()      { return dbl("pitch.rateLimitDegPerS", 8.0); }
    public double  pitchMaxDeg()        { return dbl("pitch.maxDeg", 90.0); }

    // ---------------------------------------------------------------- wind
    public double  windMean()           { return dbl("wind.meanSpeed", 8.0); }
    public double  turbulence()         { return dbl("wind.turbulenceIntensity", 0.10); }
    public double  turbLengthScale()    { return dbl("wind.turbulenceLengthScaleM", 150); }
    public String  windProfileCsv()     { return str("wind.profileCsv", ""); }

    // ---------------------------------------------------------------- loop & safety
    public int     cyclePeriodMs()      { return integer("loop.periodMs", 20); }
    public double  maxMotorRpm()        { return dbl("safety.maxMotorRpm", 1700); }
    public double  maxTorquePct()       { return dbl("safety.maxTorquePct", 150); }
    public double  torqueRampPctPerS()  { return dbl("safety.torqueRampPctPerS", 300); }
    public int     maxCommErrors()      { return integer("safety.maxConsecutiveCommErrors", 5); }
    public String  logFile()            { return str("log.csv", "emulator_log.csv"); }
}
