package windemu.drive;

/**
 * ABB Drives communication profile (the only profile RETA-01 uses on Modbus/TCP).
 * Bit definitions per the ACS800 firmware manual, "Fieldbus control" chapter.
 */
public final class AbbDrivesProfile {
    private AbbDrivesProfile() { }

    // ---- Control Word bits
    public static final int CW_OFF1_CONTROL      = 1 << 0;  // 1 = ON (ready to operate), 0 = OFF1 ramp stop
    public static final int CW_OFF2_CONTROL      = 1 << 1;  // 0 = OFF2 coast stop
    public static final int CW_OFF3_CONTROL      = 1 << 2;  // 0 = OFF3 emergency (fastest) stop
    public static final int CW_INHIBIT_OPERATION = 1 << 3;  // 1 = operation enabled (modulating)
    public static final int CW_RAMP_OUT_ZERO     = 1 << 4;  // 1 = normal
    public static final int CW_RAMP_HOLD         = 1 << 5;  // 1 = normal
    public static final int CW_RAMP_IN_ZERO      = 1 << 6;  // 1 = normal
    public static final int CW_RESET             = 1 << 7;  // 0 -> 1 edge = fault reset
    public static final int CW_REMOTE_CMD        = 1 << 10; // 1 = fieldbus control enabled
    public static final int CW_EXT_CTRL_LOC      = 1 << 11; // 0 = EXT1, 1 = EXT2

    /** Bits that must be 1 for normal operation: OFF2, OFF3, ramp bits, remote. */
    private static final int BASE = CW_OFF2_CONTROL | CW_OFF3_CONTROL | CW_RAMP_OUT_ZERO
            | CW_RAMP_HOLD | CW_RAMP_IN_ZERO | CW_REMOTE_CMD;

    public static final int CW_SHUTDOWN   = BASE;                                             // 0x0476
    public static final int CW_SWITCH_ON  = BASE | CW_OFF1_CONTROL;                           // 0x0477
    public static final int CW_RUN        = BASE | CW_OFF1_CONTROL | CW_INHIBIT_OPERATION;    // 0x047F
    public static final int CW_RESET_CMD  = BASE | CW_RESET;                                  // 0x04F6
    public static final int CW_EMERGENCY  = BASE & ~CW_OFF3_CONTROL;                          // 0x0472 (OFF3)
    public static final int CW_COAST      = BASE & ~CW_OFF2_CONTROL;                          // 0x0474 (OFF2)

    // ---- Status Word bits
    public static final int SW_RDY_ON        = 1 << 0;
    public static final int SW_RDY_RUN       = 1 << 1;
    public static final int SW_RDY_REF       = 1 << 2;  // operating, references followed
    public static final int SW_TRIPPED       = 1 << 3;
    public static final int SW_OFF_2_STA     = 1 << 4;  // 1 = OFF2 inactive
    public static final int SW_OFF_3_STA     = 1 << 5;  // 1 = OFF3 inactive
    public static final int SW_SWC_ON_INHIB  = 1 << 6;  // switch-on inhibited
    public static final int SW_ALARM         = 1 << 7;
    public static final int SW_AT_SETPOINT   = 1 << 8;
    public static final int SW_REMOTE        = 1 << 9;
    public static final int SW_ABOVE_LIMIT   = 1 << 10;
    public static final int SW_EXT_CTRL_LOC  = 1 << 11;
    public static final int SW_EXT_RUN_ENABLE = 1 << 12;

    public static boolean has(int word, int bit) { return (word & bit) != 0; }

    /** Human readable state, following the ABB Drives state machine. */
    public static String stateName(int sw) {
        if (has(sw, SW_TRIPPED)) return "FAULT";
        if (has(sw, SW_SWC_ON_INHIB)) return "SWITCH-ON INHIBITED";
        if (!has(sw, SW_OFF_2_STA)) return "OFF2 ACTIVE";
        if (!has(sw, SW_OFF_3_STA)) return "OFF3 ACTIVE";
        if (has(sw, SW_RDY_REF)) return "OPERATING";
        if (has(sw, SW_RDY_RUN)) return "READY TO OPERATE";
        if (has(sw, SW_RDY_ON)) return "READY TO SWITCH ON";
        return "NOT READY TO SWITCH ON";
    }

    public static String describe(int sw) {
        StringBuilder b = new StringBuilder(stateName(sw));
        if (has(sw, SW_ALARM)) b.append(" | ALARM");
        if (!has(sw, SW_REMOTE)) b.append(" | LOCAL (not remote!)");
        if (has(sw, SW_AT_SETPOINT)) b.append(" | AT SETPOINT");
        if (has(sw, SW_ABOVE_LIMIT)) b.append(" | ABOVE LIMIT");
        b.append(has(sw, SW_EXT_CTRL_LOC) ? " | EXT2" : " | EXT1");
        return b.toString();
    }

    public static String hex(int w) { return String.format("0x%04X", w & 0xFFFF); }
}
