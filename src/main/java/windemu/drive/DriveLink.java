package windemu.drive;

import java.io.Closeable;
import java.io.IOException;

/**
 * Cyclic process-data exchange with the drive:
 * writes data set 1 (Control Word, REF1, REF2) and reads data set 2 (Status Word, ACT1, ACT2).
 */
public interface DriveLink extends Closeable {

    void connect() throws IOException;

    /** Raw 16-bit words; references are signed (two's complement). */
    Feedback exchange(int controlWord, int ref1, int ref2) throws IOException;

    /** Read a drive parameter / actual signal (e.g. group 1 index 2 = 01.02 SPEED). Raw fieldbus integer. */
    int readParameter(int group, int index) throws IOException;

    /** Write a drive parameter (raw fieldbus integer). */
    void writeParameter(int group, int index, int value) throws IOException;

    String name();

    @Override
    void close();

    record Feedback(int statusWord, int act1, int act2) { }

    /** 16-bit register value -> signed int. */
    static int toSigned(int reg) { return (short) (reg & 0xFFFF); }

    /** signed int -> 16-bit register value, saturating to ±32767. */
    static int toRegister(long v) {
        long c = Math.max(-32767, Math.min(32767, v));
        return (int) (c & 0xFFFF);
    }
}
