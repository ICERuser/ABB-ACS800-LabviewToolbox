# Minimal fake RETA-01 (Modbus/TCP) for testing the emulator without a drive.
# Usage: python3 fake_reta01.py 502   (then set drive.simulate=false, host 127.0.0.1)
# Echoes REF1/REF2 back as ACT1/ACT2 and answers the ABB Drives state machine.
import socket, struct, sys, threading
regs = [0]*10000
log = open('fake_reta01_frames.log','w')
def status(cw):
    if not cw & 0x400: return 0x0030 | 0x40
    if not cw & 1: return 0x0231          # RDY_ON, remote
    if cw & 8: return 0x1337 | (cw & 0x800)  # operating
    return 0x0233
def handle(c):
    while True:
        h = c.recv(7)
        if len(h) < 7: return
        tid, pid, ln, uid = struct.unpack('>HHHB', h)
        pdu = c.recv(ln-1); fc = pdu[0]
        def ok(body): c.sendall(struct.pack('>HHHB', tid, 0, len(body)+1, uid) + body)
        if fc == 0x10 or fc == 0x17:
            if fc == 0x10: wa, wn = struct.unpack('>HH', pdu[1:5]); data = pdu[6:]
            else: ra, rn, wa, wn = struct.unpack('>HHHH', pdu[1:9]); data = pdu[10:]
            for i in range(wn): regs[wa+i] = struct.unpack('>H', data[2*i:2*i+2])[0]
            regs[3] = status(regs[0]); regs[4] = regs[1]; regs[5] = regs[2]  # echo refs as actuals
            log.write(f"fc{fc} write@{wa} {[hex(x) for x in regs[wa:wa+wn]]}\n"); log.flush()
            if fc == 0x10: ok(pdu[0:5]); continue
            vals = regs[ra:ra+rn]; ok(bytes([fc, 2*rn]) + b''.join(struct.pack('>H', v) for v in vals))
        elif fc == 0x03:
            a, n = struct.unpack('>HH', pdu[1:5])
            if a >= 9999: ok(bytes([0x83, 2])); continue
            ok(bytes([3, 2*n]) + b''.join(struct.pack('>H', v) for v in regs[a:a+n]))
        else: ok(bytes([fc | 0x80, 1]))
s = socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('127.0.0.1', int(sys.argv[1]))); s.listen(8)
while True:
    c, _ = s.accept(); threading.Thread(target=handle, args=(c,), daemon=True).start()
