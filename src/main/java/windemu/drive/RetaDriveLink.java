package windemu.drive;

import windemu.config.EmulatorConfig;
import windemu.modbus.ModbusTcpClient;

import java.io.IOException;

/**
 * Real ACS800 through the RETA-01 Ethernet adapter, protocol Modbus/TCP (par. 51.16 = 0).
 *
 * RETA-01 register map (user's manual, "Modbus/TCP" chapter):
 *   40001..40096  data sets: 4 0001 = DS1.1, 4 0002 = DS1.2, 4 0003 = DS1.3, 4 0004 = DS2.1 ...
 *   4GGII         drive parameter GG.II (e.g. 40102 = 01.02 SPEED, 42202 = 22.02)
 *
 * With the ACS800 standard firmware and 98.02 = FIELDBUS, 98.07 = ABB DRIVES:
 *   DS1 (master -> drive): Control Word, REF1, REF2   -> 40001..40003
 *   DS2 (drive -> master): Status Word,  ACT1, ACT2   -> 40004..40006
 */
public final class RetaDriveLink implements DriveLink {

    private final EmulatorConfig cfg;
    private final ModbusTcpClient mb;
    private final int cwAddr;
    private final int swAddr;

    public RetaDriveLink(EmulatorConfig cfg) {
        this.cfg = cfg;
        this.mb = new ModbusTcpClient(cfg.driveHost(), cfg.drivePort(), cfg.unitId(), cfg.ioTimeoutMs());
        this.cwAddr = cfg.registerBase() + cfg.regControlWord() - 1;
        this.swAddr = cfg.registerBase() + cfg.regStatusWord() - 1;
    }

    @Override
    public void connect() throws IOException {
        mb.connect();
    }

    @Override
    public Feedback exchange(int controlWord, int ref1, int ref2) throws IOException {
        int[] write = {controlWord & 0xFFFF, ref1 & 0xFFFF, ref2 & 0xFFFF};
        int[] r;
        if (cfg.useReadWriteFc23()) {
            r = mb.readWriteMultipleRegisters(swAddr, 3, cwAddr, write);
        } else {
            mb.writeMultipleRegisters(cwAddr, write);
            r = mb.readHoldingRegisters(swAddr, 3);
        }
        return new Feedback(r[0], DriveLink.toSigned(r[1]), DriveLink.toSigned(r[2]));
    }

    private int paramAddr(int group, int index) {
        return cfg.registerBase() + group * 100 + index - 1;
    }

    @Override
    public int readParameter(int group, int index) throws IOException {
        return DriveLink.toSigned(mb.readHoldingRegisters(paramAddr(group, index), 1)[0]);
    }

    @Override
    public void writeParameter(int group, int index, int value) throws IOException {
        mb.writeSingleRegister(paramAddr(group, index), value & 0xFFFF);
    }

    @Override
    public String name() {
        return "RETA-01 Modbus/TCP " + cfg.driveHost() + ":" + cfg.drivePort();
    }

    @Override
    public void close() {
        mb.close();
    }
}
