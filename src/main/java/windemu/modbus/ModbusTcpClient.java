package windemu.modbus;

import java.io.Closeable;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;

/**
 * Minimal, dependency-free Modbus/TCP master.
 * Implements exactly the function codes supported by the ABB RETA-01:
 * 03 Read Holding Registers, 06 Preset Single Register,
 * 16 Preset Multiple Registers, 23 Read/Write Multiple Registers.
 */
public final class ModbusTcpClient implements Closeable {

    private final String host;
    private final int port;
    private final int unitId;
    private final int timeoutMs;

    private Socket socket;
    private DataInputStream in;
    private DataOutputStream out;
    private int transactionId;

    public ModbusTcpClient(String host, int port, int unitId, int timeoutMs) {
        this.host = host;
        this.port = port;
        this.unitId = unitId;
        this.timeoutMs = timeoutMs;
    }

    public synchronized void connect() throws IOException {
        close();
        socket = new Socket();
        socket.setTcpNoDelay(true);
        socket.connect(new InetSocketAddress(host, port), Math.max(timeoutMs, 1000));
        socket.setSoTimeout(timeoutMs);
        in = new DataInputStream(socket.getInputStream());
        out = new DataOutputStream(socket.getOutputStream());
    }

    public synchronized boolean isConnected() {
        return socket != null && socket.isConnected() && !socket.isClosed();
    }

    /** FC03 – read {@code count} holding registers starting at PDU address {@code addr}. */
    public synchronized int[] readHoldingRegisters(int addr, int count) throws IOException {
        byte[] pdu = {0x03, hi(addr), lo(addr), hi(count), lo(count)};
        byte[] resp = transact(pdu);
        return parseRegisters(resp, 1, count);
    }

    /** FC06 – write a single register. */
    public synchronized void writeSingleRegister(int addr, int value) throws IOException {
        byte[] pdu = {0x06, hi(addr), lo(addr), hi(value), lo(value)};
        transact(pdu);
    }

    /** FC16 – write consecutive registers. */
    public synchronized void writeMultipleRegisters(int addr, int[] values) throws IOException {
        byte[] pdu = new byte[6 + values.length * 2];
        pdu[0] = 0x10;
        pdu[1] = hi(addr); pdu[2] = lo(addr);
        pdu[3] = hi(values.length); pdu[4] = lo(values.length);
        pdu[5] = (byte) (values.length * 2);
        for (int i = 0; i < values.length; i++) {
            pdu[6 + 2 * i] = hi(values[i]);
            pdu[7 + 2 * i] = lo(values[i]);
        }
        transact(pdu);
    }

    /** FC23 – write registers and read registers in one transaction (write is executed first). */
    public synchronized int[] readWriteMultipleRegisters(int readAddr, int readCount,
                                                         int writeAddr, int[] values) throws IOException {
        byte[] pdu = new byte[10 + values.length * 2];
        pdu[0] = 0x17;
        pdu[1] = hi(readAddr); pdu[2] = lo(readAddr);
        pdu[3] = hi(readCount); pdu[4] = lo(readCount);
        pdu[5] = hi(writeAddr); pdu[6] = lo(writeAddr);
        pdu[7] = hi(values.length); pdu[8] = lo(values.length);
        pdu[9] = (byte) (values.length * 2);
        for (int i = 0; i < values.length; i++) {
            pdu[10 + 2 * i] = hi(values[i]);
            pdu[11 + 2 * i] = lo(values[i]);
        }
        byte[] resp = transact(pdu);
        return parseRegisters(resp, 1, readCount);
    }

    private byte[] transact(byte[] pdu) throws IOException {
        if (!isConnected()) throw new IOException("Not connected");
        int tid = (++transactionId) & 0xFFFF;

        // MBAP header (transaction id, protocol id 0, length, unit id) + PDU, sent as ONE segment –
        // embedded Modbus stacks often reject an ADU split over several TCP segments.
        byte[] adu = new byte[7 + pdu.length];
        adu[0] = hi(tid); adu[1] = lo(tid);
        adu[2] = 0; adu[3] = 0;
        adu[4] = hi(pdu.length + 1); adu[5] = lo(pdu.length + 1);
        adu[6] = (byte) unitId;
        System.arraycopy(pdu, 0, adu, 7, pdu.length);
        out.write(adu);
        out.flush();

        int rTid, rProto, rLen;
        while (true) {
            rTid = in.readUnsignedShort();
            rProto = in.readUnsignedShort();
            rLen = in.readUnsignedShort();
            if (rLen < 2 || rLen > 260) throw new IOException("Bad MBAP length " + rLen);
            in.readUnsignedByte(); // unit id
            byte[] body = new byte[rLen - 1];
            in.readFully(body);
            if (rProto != 0) throw new IOException("Bad protocol id " + rProto);
            if (rTid != tid) continue; // stale answer from an earlier timed-out request – discard
            if ((body[0] & 0x80) != 0) {
                int code = body.length > 1 ? body[1] & 0xFF : -1;
                throw new ModbusException(body[0] & 0x7F, code);
            }
            if (body[0] != pdu[0]) throw new IOException("Function code mismatch");
            return body;
        }
    }

    private static int[] parseRegisters(byte[] body, int offset, int count) throws IOException {
        int byteCount = body[offset] & 0xFF;
        if (byteCount != count * 2 || body.length < offset + 1 + byteCount)
            throw new IOException("Unexpected byte count " + byteCount);
        int[] regs = new int[count];
        for (int i = 0; i < count; i++) {
            regs[i] = ((body[offset + 1 + 2 * i] & 0xFF) << 8) | (body[offset + 2 + 2 * i] & 0xFF);
        }
        return regs;
    }

    private static byte hi(int v) { return (byte) ((v >> 8) & 0xFF); }
    private static byte lo(int v) { return (byte) (v & 0xFF); }

    @Override
    public synchronized void close() {
        if (socket != null) {
            try { socket.close(); } catch (IOException ignored) { }
        }
        socket = null;
    }

    /** Modbus exception response (01 illegal function, 02 illegal address, 03 illegal value). */
    public static final class ModbusException extends IOException {
        public final int function;
        public final int code;

        public ModbusException(int function, int code) {
            super("Modbus exception: function 0x" + Integer.toHexString(function) + " code " + code
                    + switch (code) {
                        case 1 -> " (illegal function)";
                        case 2 -> " (illegal data address)";
                        case 3 -> " (illegal data value)";
                        default -> "";
                    });
            this.function = function;
            this.code = code;
        }
    }
}
