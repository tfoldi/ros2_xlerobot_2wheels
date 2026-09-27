#!/usr/bin/env python3
"""Fake Feetech STS servo buses on two ptys, to run the real sts_hardware_interface without hardware.

Speaks what the driver sends (ping, read, write, sync read 0x82, sync write 0x83) and simulates
servos that hold or move toward their goal while torque is on and stay put while it is off. Servos
power up with torque off and a stale goal (2047), like the real ones, and start at the arms' folded
rest pose. Every torque and goal write is logged in order with the present position, which is how
to check that nothing jumps at activation, e-stop release or a bad trajectory.

  python3 tools/fake_sts_bus.py fake.log         prints the two pty paths (bus1, bus2)
  ros2 launch xlerobot_bringup bringup.launch.py bus1_port:=<pty1> bus2_port:=<pty2> \\
      with_arms:=true with_head:=true torque_off:=false

Moving a servo by hand while torque is off: write lines "<servo id> <steps>" to a file called
nudge.txt next to the log, it is applied and removed.
"""
import os, pty, sys, threading, time, tty

LOG = open(sys.argv[1], "w", buffering=1)
T0 = time.time()

def log(msg):
    LOG.write(f"{time.time() - T0:8.3f} {msg}\n")

def sm(v, bit):
    return ((1 if v < 0 else 0) << bit) | abs(v)

# raw present positions (servo-corrected, as `status` reports) at the folded rest pose
REST = {
    "bus1": {1: 1955, 2: 1404, 3: 3058, 4: 1982, 5: 1706, 6: 1575, 7: 1998, 8: 2021},
    "bus2": {1: 1986, 2: 1583, 3: 3102, 4: 1890, 5: 2019, 6: 1522, 9: 0, 10: 0},
}

class Servo:
    def __init__(self, sid, pos):
        self.id = sid
        self.mem = bytearray(256)
        self.pos = float(pos)
        self.set16(56, pos)          # present position
        self.set16(42, 2047)         # stale goal from before, torque is off at power up
        self.mem[40] = 0             # torque off
        self.mem[62] = 121           # 12.1 V
        self.mem[63] = 30

    def set16(self, addr, v):
        self.mem[addr] = v & 0xFF
        self.mem[addr + 1] = (v >> 8) & 0xFF

    def get16(self, addr):
        return self.mem[addr] | (self.mem[addr + 1] << 8)

    def write(self, name, addr, data):
        for k, b in enumerate(data):
            self.mem[addr + k] = b
        if addr <= 40 < addr + len(data):
            log(f"{name} servo {self.id}: torque {'ON' if self.mem[40] else 'off'} (present {int(self.pos)}, goal {self.get16(42)})")
        if addr <= 42 < addr + len(data):
            log(f"{name} servo {self.id}: goal {self.get16(42)} speed {self.get16(46)} (present {int(self.pos)}, torque {'on' if self.mem[40] else 'off'})")

    def step(self, dt):
        if self.mem[40]:
            goal = self.get16(42)
            speed = self.get16(46) or 3400
            d = goal - self.pos
            m = speed * dt
            self.pos += max(-m, min(m, d))
        self.set16(56, int(round(self.pos)) & 0x7FFF)

class Bus:
    def __init__(self, name):
        self.name = name
        self.servos = {i: Servo(i, p) for i, p in REST[name].items()}
        self.master, slave = pty.openpty()
        tty.setraw(self.master); tty.setraw(slave)
        self.path = os.ttyname(slave)
        os.close(slave)   # the node stays while the master is open, and no process holds the slave
        threading.Thread(target=self.serve, daemon=True).start()
        threading.Thread(target=self.physics, daemon=True).start()

    def physics(self):
        while True:
            for s in self.servos.values():
                s.step(0.002)
            time.sleep(0.002)

    @staticmethod
    def pack(sid, err, data=b""):
        body = bytes([sid, len(data) + 2, err]) + bytes(data)
        return b"\xff\xff" + body + bytes([~sum(body) & 0xFF])

    def serve(self):
        buf = b""
        while True:
            try:
                buf += os.read(self.master, 512)
            except OSError:      # nobody has the slave open yet
                time.sleep(0.01); continue
            while len(buf) >= 4:
                if buf[:2] != b"\xff\xff":
                    buf = buf[1:]; continue
                n = buf[3]
                if len(buf) < 4 + n:
                    break
                pk, buf = buf[:4 + n], buf[4 + n:]
                sid, inst, params = pk[2], pk[4], pk[5:-1]
                out = b""
                if inst == 0x82:                       # sync read: addr, len, ids...
                    addr, ln = params[0], params[1]
                    for i in params[2:]:
                        s = self.servos.get(i)
                        if s:
                            out += self.pack(i, 0, s.mem[addr:addr + ln])
                elif inst == 0x83:                     # sync write: addr, len, (id, data)...
                    addr, ln = params[0], params[1]
                    rest = params[2:]
                    for k in range(0, len(rest), ln + 1):
                        i = rest[k]
                        if i in self.servos:
                            self.servos[i].write("SYNC", addr, rest[k + 1:k + 1 + ln])
                elif sid in self.servos:
                    s = self.servos[sid]
                    if inst == 0x01:
                        out = self.pack(sid, 0)
                    elif inst == 0x02:
                        out = self.pack(sid, 0, s.mem[params[0]:params[0] + params[1]])
                    elif inst == 0x03:
                        s.write("WRITE", params[0], params[1:])
                        out = self.pack(sid, 0)
                if out:
                    os.write(self.master, out)

b1, b2 = Bus("bus1"), Bus("bus2")

def nudger():
    path = os.path.join(os.path.dirname(sys.argv[1]), "nudge.txt")
    while True:
        if os.path.exists(path):
            for line in open(path).read().split("\n"):
                if line.strip():
                    sid, delta = (int(x) for x in line.split())
                    for b in (b1, b2):
                        sv = b.servos.get(sid)
                        if sv and not sv.mem[40]:
                            sv.pos += delta
                            log(f"HAND servo {sid} moved by {delta} while torque off -> present {int(sv.pos)}")
            os.remove(path)
        time.sleep(0.05)
threading.Thread(target=nudger, daemon=True).start()
print(b1.path, b2.path, flush=True)
while True:
    time.sleep(1)
