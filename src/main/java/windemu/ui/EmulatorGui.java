package windemu.ui;

import windemu.control.EmulatorController;
import windemu.control.Telemetry;
import windemu.drive.AbbDrivesProfile;

import javax.swing.*;
import javax.swing.border.EmptyBorder;
import javax.swing.border.TitledBorder;
import java.awt.*;
import java.io.File;
import java.util.LinkedHashMap;
import java.util.Map;

/** Swing operator panel: commands, wind settings, live values and trend charts. */
public final class EmulatorGui extends JFrame {

    private final EmulatorController ctl;
    private final Map<String, JLabel> values = new LinkedHashMap<>();
    private final JLabel stateLabel = new JLabel("DISCONNECTED");
    private final JLabel msgLabel = new JLabel(" ");
    private final StripChart windChart, shaftChart, torqueChart;
    private final double maxRpm;
    private int sampleDivider;

    public EmulatorGui(EmulatorController ctl, double maxMotorRpm) {
        super("Wind Turbine Emulator – ACS800 via RETA-01 (Modbus/TCP)");
        this.ctl = ctl;
        this.maxRpm = maxMotorRpm;
        setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);

        double ratedRotorRpm = ctl.turbine().ratedOmegaRotor * 30 / Math.PI;
        windChart = new StripChart("Wind & pitch", 1500)
                .add("wind", "m/s", new Color(0x58A6FF), 0, 30)
                .add("pitch", "deg", new Color(0xF0883E), 0, 90)
                .add("Cp×100", "-", new Color(0x7EE787), 0, 60);
        shaftChart = new StripChart("Shaft speed", 1500)
                .add("rotor", "rpm", new Color(0xD2A8FF), 0, Math.ceil(ratedRotorRpm * 1.5))
                .add("motor", "rpm", new Color(0x56D4DD), 0, maxMotorRpm)
                .add("speed ref", "rpm", new Color(0x8B949E), 0, maxMotorRpm);
        torqueChart = new StripChart("Motor torque", 1500)
                .add("ref", "%Tn", new Color(0xFF7B72), -20, 150)
                .add("actual", "%Tn", new Color(0xFFA657), -20, 150)
                .add("gen est.", "%Tn", new Color(0x7EE787), -20, 150);

        JPanel charts = new JPanel(new GridLayout(3, 1, 0, 4));
        charts.add(windChart);
        charts.add(shaftChart);
        charts.add(torqueChart);

        JPanel side = new JPanel();
        side.setLayout(new BoxLayout(side, BoxLayout.Y_AXIS));
        side.setBorder(new EmptyBorder(6, 6, 6, 6));
        side.add(commandPanel());
        side.add(windPanel());
        side.add(pitchPanel());
        side.add(valuesPanel());
        side.add(Box.createVerticalGlue());

        JPanel status = new JPanel(new BorderLayout(10, 0));
        status.setBorder(new EmptyBorder(4, 8, 4, 8));
        stateLabel.setFont(stateLabel.getFont().deriveFont(Font.BOLD, 14f));
        status.add(stateLabel, BorderLayout.WEST);
        status.add(msgLabel, BorderLayout.CENTER);

        getContentPane().setLayout(new BorderLayout());
        getContentPane().add(new JScrollPane(side), BorderLayout.WEST);
        getContentPane().add(charts, BorderLayout.CENTER);
        getContentPane().add(status, BorderLayout.SOUTH);
        setSize(1280, 820);
        setLocationRelativeTo(null);

        ctl.setListener(t -> {
            if (++sampleDivider % 2 == 0) SwingUtilities.invokeLater(() -> update(t));
        });
    }

    private JPanel commandPanel() {
        JPanel p = titled("Drive commands", new GridLayout(3, 2, 4, 4));
        p.add(button("Connect", ctl::connect));
        p.add(button("Disconnect", ctl::disconnect));
        p.add(button("Start", ctl::start));
        p.add(button("Stop (feather)", ctl::stop));
        JButton estop = button("EMERGENCY STOP", ctl::emergencyStop);
        estop.setBackground(new Color(0xC62828));
        estop.setForeground(Color.WHITE);
        estop.setOpaque(true);
        p.add(estop);
        p.add(button("Reset fault", ctl::reset));
        return p;
    }

    private JPanel windPanel() {
        JPanel p = titled("Wind", new GridLayout(0, 1, 2, 2));
        JLabel meanL = new JLabel();
        JSlider mean = new JSlider(0, 300, (int) Math.round(ctl.wind().getMean() * 10));
        mean.addChangeListener(e -> {
            ctl.wind().setMean(mean.getValue() / 10.0);
            meanL.setText(String.format("Mean wind: %.1f m/s", mean.getValue() / 10.0));
        });
        meanL.setText(String.format("Mean wind: %.1f m/s", ctl.wind().getMean()));

        JLabel tiL = new JLabel();
        JSlider ti = new JSlider(0, 30, (int) Math.round(ctl.wind().getTurbulenceIntensity() * 100));
        ti.addChangeListener(e -> {
            ctl.wind().setTurbulenceIntensity(ti.getValue() / 100.0);
            tiL.setText("Turbulence intensity: " + ti.getValue() + " %");
        });
        tiL.setText("Turbulence intensity: " + ti.getValue() + " %");

        JPanel gust = new JPanel(new GridLayout(1, 2, 4, 0));
        gust.add(button("Gust +5 m/s", () -> ctl.wind().triggerGust(5, 10.5)));
        gust.add(button("Lull −3 m/s", () -> ctl.wind().triggerGust(-3, 8)));

        JCheckBox profile = new JCheckBox("Use wind profile (CSV)");
        profile.setSelected(ctl.wind().isUsingProfile());
        profile.addActionListener(e -> {
            if (profile.isSelected() && !ctl.wind().isUsingProfile()) {
                JFileChooser fc = new JFileChooser(new File("."));
                if (fc.showOpenDialog(this) == JFileChooser.APPROVE_OPTION) {
                    try {
                        ctl.wind().loadProfile(fc.getSelectedFile().toPath());
                    } catch (Exception ex) {
                        JOptionPane.showMessageDialog(this, ex.getMessage(), "Profile", JOptionPane.ERROR_MESSAGE);
                        profile.setSelected(false);
                    }
                } else profile.setSelected(false);
            } else ctl.wind().setUseProfile(profile.isSelected());
        });

        p.add(meanL); p.add(mean); p.add(tiL); p.add(ti); p.add(gust); p.add(profile);
        return p;
    }

    private JPanel pitchPanel() {
        JPanel p = titled("Pitch", new GridLayout(0, 1, 2, 2));
        JCheckBox manual = new JCheckBox("Manual pitch");
        JLabel l = new JLabel("Manual set-point: 0°");
        JSlider s = new JSlider(0, 90, 0);
        s.setEnabled(false);
        manual.addActionListener(e -> {
            ctl.pitch().setManual(manual.isSelected());
            s.setEnabled(manual.isSelected());
            ctl.pitch().setManualSetpoint(s.getValue());
        });
        s.addChangeListener(e -> {
            ctl.pitch().setManualSetpoint(s.getValue());
            l.setText("Manual set-point: " + s.getValue() + "°");
        });
        p.add(manual); p.add(l); p.add(s);
        return p;
    }

    private JPanel valuesPanel() {
        JPanel p = titled("Live values", new GridLayout(0, 2, 6, 1));
        String[] keys = {"Wind", "Pitch", "Tip-speed ratio λ", "Cp", "Rotor speed", "Aero torque (rotor)",
                "Aero power", "Motor speed", "Speed ref", "Torque ref", "Torque actual", "Gen. torque est.",
                "Gen. torque (MPPT rec.)", "Control word", "Status word", "REF1 / REF2", "ACT1 / ACT2",
                "Drive state", "Cycle time", "Mode"};
        for (String k : keys) {
            JLabel v = new JLabel("–");
            v.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
            values.put(k, v);
            p.add(new JLabel(k));
            p.add(v);
        }
        return p;
    }

    private void update(Telemetry t) {
        set("Wind", "%.2f m/s", t.wind());
        set("Pitch", "%.1f °", t.pitchDeg());
        set("Tip-speed ratio λ", "%.2f", t.lambda());
        set("Cp", "%.3f", t.cp());
        set("Rotor speed", "%.2f rpm", t.rotorRpm());
        set("Aero torque (rotor)", "%.1f kNm", t.aeroTorqueRotorKNm());
        set("Aero power", "%.1f kW", t.aeroPowerKW());
        set("Motor speed", "%.0f rpm", t.motorRpm());
        set("Speed ref", "%.0f rpm", t.motorSpeedRefRpm());
        set("Torque ref", "%.1f %%", t.motorTorqueRefPct());
        set("Torque actual", "%.1f %%", t.motorTorqueActPct());
        set("Gen. torque est.", "%.1f %%", t.generatorTorqueEstPct());
        set("Gen. torque (MPPT rec.)", "%.1f %%", t.recommendedGenTorquePct());
        values.get("Control word").setText(AbbDrivesProfile.hex(t.controlWord()));
        values.get("Status word").setText(AbbDrivesProfile.hex(t.statusWord()));
        values.get("REF1 / REF2").setText(t.ref1() + " / " + t.ref2());
        values.get("ACT1 / ACT2").setText(t.act1() + " / " + t.act2());
        values.get("Drive state").setText(AbbDrivesProfile.stateName(t.statusWord()));
        values.get("Drive state").setToolTipText(AbbDrivesProfile.describe(t.statusWord()));
        set("Cycle time", "%.2f ms", t.cycleMs());
        values.get("Mode").setText(ctl.isTorqueMode() ? "TORQUE (REF2)" : "SPEED (REF1)");

        stateLabel.setText(t.state());
        stateLabel.setForeground(switch (t.state()) {
            case "RUNNING" -> new Color(0x2E7D32);
            case "FAULT", "ESTOP", "COMM_LOST" -> new Color(0xC62828);
            case "STARTING", "STOPPING" -> new Color(0xEF6C00);
            default -> Color.DARK_GRAY;
        });
        msgLabel.setText(t.message());

        windChart.push(t.wind(), t.pitchDeg(), t.cp() * 100);
        shaftChart.push(t.rotorRpm(), t.motorRpm(), t.motorSpeedRefRpm());
        torqueChart.push(t.motorTorqueRefPct(), t.motorTorqueActPct(), t.generatorTorqueEstPct());
    }

    private void set(String key, String fmt, double v) {
        values.get(key).setText(String.format(fmt, v));
    }

    private static JPanel titled(String title, LayoutManager lm) {
        JPanel p = new JPanel(lm);
        p.setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createTitledBorder(BorderFactory.createEtchedBorder(), title,
                        TitledBorder.LEFT, TitledBorder.TOP),
                new EmptyBorder(4, 6, 6, 6)));
        p.setAlignmentX(Component.LEFT_ALIGNMENT);
        p.setMaximumSize(new Dimension(380, Integer.MAX_VALUE));
        return p;
    }

    private static JButton button(String text, Runnable r) {
        JButton b = new JButton(text);
        b.addActionListener(e -> r.run());
        return b;
    }
}
