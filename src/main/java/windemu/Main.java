package windemu;

import windemu.config.EmulatorConfig;
import windemu.control.EmulatorController;
import windemu.control.Telemetry;
import windemu.ui.EmulatorGui;

import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import java.nio.file.Path;

/**
 * Wind turbine emulator for an ABB ACS800 (DTC) controlled through a RETA-01 Ethernet adapter.
 *
 * Usage:
 *   java -jar wind-emulator.jar [config.properties] [--headless SECONDS] [--sim | --real] [--host IP]
 */
public final class Main {

    public static void main(String[] args) throws Exception {
        Path cfgFile = Path.of("emulator.properties");
        Integer headlessSeconds = null;
        Boolean simulate = null;
        String host = null;
        String mode = null;

        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--headless" -> headlessSeconds = Integer.parseInt(args[++i]);
                case "--sim" -> simulate = true;
                case "--real" -> simulate = false;
                case "--host" -> host = args[++i];
                case "--mode" -> mode = args[++i];
                case "-h", "--help" -> { usage(); return; }
                default -> cfgFile = Path.of(args[i]);
            }
        }

        EmulatorConfig cfg = EmulatorConfig.load(cfgFile);
        if (simulate != null) cfg.set("drive.simulate", simulate.toString());
        if (host != null) cfg.set("drive.host", host);
        if (mode != null) cfg.set("emulator.controlMode", mode);

        EmulatorController ctl = new EmulatorController(cfg);
        System.out.println(ctl.turbine().summary());
        System.out.println("Mode: " + cfg.controlMode() + ", drive: "
                + (cfg.simulateDrive() ? "SIMULATED" : cfg.driveHost() + ":" + cfg.drivePort()));

        Thread rt = new Thread(ctl::run, "emulator-rt");
        rt.setDaemon(false);
        rt.start();
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            ctl.shutdown();
            try { rt.join(2000); } catch (InterruptedException ignored) { }
        }));

        if (headlessSeconds != null) {
            runHeadless(ctl, headlessSeconds);
            ctl.shutdown();
            rt.join();
            return;
        }

        final EmulatorConfig fc = cfg;
        SwingUtilities.invokeLater(() -> {
            try { UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName()); } catch (Exception ignored) { }
            EmulatorGui gui = new EmulatorGui(ctl, fc.maxMotorRpm());
            gui.addWindowListener(new java.awt.event.WindowAdapter() {
                @Override public void windowClosed(java.awt.event.WindowEvent e) {
                    ctl.shutdown();
                    System.exit(0);
                }
            });
            gui.setVisible(true);
        });
    }

    /** Connect, start, run for the given time printing once per second, then stop. */
    private static void runHeadless(EmulatorController ctl, int seconds) throws InterruptedException {
        ctl.connect();
        Thread.sleep(500);
        ctl.start();
        System.out.println("   t[s] state      wind  pitch    λ     Cp  rotor[rpm] motor[rpm]  Tref%  Tact%  Pa[kW]  CW     SW");
        for (int s = 0; s < seconds; s++) {
            Thread.sleep(1000);
            Telemetry t = ctl.last();
            if (t == null) continue;
            System.out.printf("%7.1f %-9s %5.2f %6.1f %5.2f %6.3f %10.2f %10.0f %6.1f %6.1f %7.1f 0x%04X 0x%04X%n",
                    t.time(), t.state(), t.wind(), t.pitchDeg(), t.lambda(), t.cp(), t.rotorRpm(), t.motorRpm(),
                    t.motorTorqueRefPct(), t.motorTorqueActPct(), t.aeroPowerKW(), t.controlWord(), t.statusWord());
        }
        ctl.stop();
        Thread.sleep(1500);
        ctl.disconnect();
        Thread.sleep(200);
    }

    private static void usage() {
        System.out.println("""
                Wind turbine emulator for ABB ACS800 + RETA-01 (Modbus/TCP)
                  java -jar wind-emulator.jar [config.properties] [options]
                Options:
                  --sim               use the built-in simulated drive (no hardware)
                  --real              talk to the real RETA-01
                  --host IP           RETA-01 IP address (overrides drive.host)
                  --mode TORQUE|SPEED emulation mode (overrides emulator.controlMode)
                  --headless SECONDS  no GUI: connect, start, run, stop, print values
                """);
    }
}
