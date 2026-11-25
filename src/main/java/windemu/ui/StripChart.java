package windemu.ui;

import javax.swing.JPanel;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.util.ArrayList;
import java.util.List;

/** Lightweight scrolling chart (no external libraries). Each series has its own fixed range. */
public final class StripChart extends JPanel {

    private final String title;
    private final int capacity;
    private final List<Series> series = new ArrayList<>();
    private int count, head;

    private static final class Series {
        final String name; final Color color; final double min, max; final String unit;
        final double[] data;
        Series(String n, String u, Color c, double min, double max, int cap) {
            name = n; unit = u; color = c; this.min = min; this.max = max; data = new double[cap];
        }
    }

    public StripChart(String title, int capacity) {
        this.title = title;
        this.capacity = capacity;
        setBackground(new Color(0x14181F));
        setPreferredSize(new Dimension(700, 220));
    }

    public StripChart add(String name, String unit, Color c, double min, double max) {
        series.add(new Series(name, unit, c, min, max, capacity));
        return this;
    }

    /** Append one sample (one value per series, in the order the series were added). */
    public synchronized void push(double... values) {
        for (int i = 0; i < series.size() && i < values.length; i++) series.get(i).data[head] = values[i];
        head = (head + 1) % capacity;
        if (count < capacity) count++;
        repaint();
    }

    @Override
    protected synchronized void paintComponent(Graphics g0) {
        super.paintComponent(g0);
        Graphics2D g = (Graphics2D) g0;
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        int w = getWidth(), h = getHeight();
        int left = 8, right = 8, top = 22, bottom = 8;
        int pw = w - left - right, ph = h - top - bottom;

        g.setColor(new Color(0x2A313C));
        for (int i = 0; i <= 4; i++) {
            int y = top + ph * i / 4;
            g.drawLine(left, y, left + pw, y);
        }
        g.setFont(getFont().deriveFont(Font.BOLD, 12f));
        g.setColor(new Color(0xC9D1D9));
        g.drawString(title, left, 15);

        int x = left + g.getFontMetrics().stringWidth(title) + 16;
        g.setFont(getFont().deriveFont(11f));
        for (Series s : series) {
            g.setColor(s.color);
            String label = String.format("%s [%s] %.0f…%.0f", s.name, s.unit, s.min, s.max);
            g.fillRect(x, 7, 10, 3);
            g.drawString(label, x + 14, 15);
            x += g.getFontMetrics().stringWidth(label) + 26;
        }
        if (count < 2) return;

        g.setStroke(new BasicStroke(1.6f));
        for (Series s : series) {
            g.setColor(s.color);
            int px = 0, py = 0;
            for (int i = 0; i < count; i++) {
                int idx = (head - count + i + capacity) % capacity;
                double f = (s.data[idx] - s.min) / (s.max - s.min);
                f = Math.max(0, Math.min(1, f));
                int cx = left + (int) ((long) pw * (capacity - count + i) / (capacity - 1));
                int cy = top + ph - (int) (f * ph);
                if (i > 0) g.drawLine(px, py, cx, cy);
                px = cx; py = cy;
            }
        }
    }
}
