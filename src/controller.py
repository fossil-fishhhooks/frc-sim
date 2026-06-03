#!/usr/bin/env python3
"""
frc_sim_controller.py — swerve keyboard controller for frc_sim
Python hosts NT4 server. External sim connects as client.

  W/S    — forward/back
  A/D    — strafe left/right
  Q/E    — rotate left/right
  T/G    — tilt shooter up/down
  F/H    — pan shooter left/right
  +/-    — fire speed
  Space  — fire
  X      — stop
  ESC    — quit

Install:  pip install robotpy
Run:      python frc_sim_controller.py
Then start frc_sim with --nt 127.0.0.1:5810
"""

import math, time, sys, tty, termios, select, os

try:
    import ntcore
except ImportError:
    print("pip install robotpy"); sys.exit(1)

# ── Config ────────────────────────────────────────────────────────────────────
NT_PORT     = 5810
DRIVE_V     = 0.7    # max drive fraction [-1, 1]
ROTATE_V    = 0.5    # max rotate fraction
AIM_SPEED   = 1.5    # rad/s
SHOOT_SPEED = 12.0   # m/s
SPEED_STEP  = 1.0
TICK_HZ     = 50

# ── NT4 server ────────────────────────────────────────────────────────────────
inst = ntcore.NetworkTableInstance.getDefault()

vpubs = [inst.getFloatTopic(f"/sim/motors/{i}/voltage").publish()     for i in range(4)]
spubs = [inst.getFloatTopic(f"/sim/motors/{i}/steer_angle").publish() for i in range(4)]
fire_pub  = inst.getBooleanTopic("/sim/shooter/fire").publish()
speed_pub = inst.getFloatTopic("/sim/shooter/speed").publish()
dir_pub   = inst.getFloatArrayTopic("/sim/shooter/direction").publish()

# ── Raw terminal ──────────────────────────────────────────────────────────────
class RawTerminal:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self
    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

def read_keys():
    keys = set()
    while select.select([sys.stdin], [], [], 0)[0]:
        ch = os.read(sys.stdin.fileno(), 1)
        if not ch:
            break
        if ch == b'\x1b' or ch == b'\x03':
            keys.add('esc')
        elif ch == b' ':
            keys.add('space')
        else:
            c = ch.decode('utf-8', errors='ignore').lower()
            if c:
                keys.add(c)
    return keys

# ── Swerve kinematics ─────────────────────────────────────────────────────────
# Robot frame: +X = forward, +Z = left (right-hand, Y-up).
# Module indices from scene JSON attachments (x, z):
#   0: (+0.28,  0.28) = FL    1: (-0.28,  0.28) = BL
#   2: (+0.28, -0.28) = FR    3: (-0.28, -0.28) = BR

def swerve(fwd, strafe, rot):
    modules = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
    result = []
    for (mx, mz) in modules:
        vx = fwd    + rot * (-mz)
        vz = -strafe + rot * ( mx)
        speed = min(math.hypot(vx, vz), 1.0)
        angle = math.atan2(vz, vx)
        result.append((speed, angle))
    return result

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def aim_dir(tilt, pan):
    x = -(math.cos(tilt) * math.cos(pan))
    y =   math.sin(tilt)
    z =  -math.cos(tilt) * math.sin(pan)
    n = math.sqrt(x*x + y*y + z*z)
    return [x/n, y/n, z/n]

# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    inst.startServer()

    dt          = 1.0 / TICK_HZ
    shoot_speed = SHOOT_SPEED
    shoot_tilt  = 0.3
    shoot_pan   = 0.0
    prev_plus   = False
    prev_minus  = False
    tick        = 0

    print("frc_sim swerve controller — type keys in THIS terminal window")
    print("W/S=fwd  A/D=strafe  Q/E=rotate  T/G=tilt  F/H=pan  Space=fire  +/-=speed  X=stop  ESC=quit\n")

    KEY_DECAY = 4
    key_age: dict[str, int] = {}

    with RawTerminal():
        while True:
            t0   = time.monotonic()
            tick += 1

            key_age = {k: v + 1 for k, v in key_age.items() if v + 1 <= KEY_DECAY}
            for k in read_keys():
                key_age[k] = 0

            if key_age.get('esc', KEY_DECAY + 1) <= KEY_DECAY:
                break

            def h(*keys):
                return any(key_age.get(k, KEY_DECAY + 1) <= KEY_DECAY for k in keys)

            fwd    = (DRIVE_V  if h('w') else 0.0) - (DRIVE_V  if h('s') else 0.0)
            strafe = (DRIVE_V  if h('a') else 0.0) - (DRIVE_V  if h('d') else 0.0)
            rot    = (ROTATE_V if h('q') else 0.0) - (ROTATE_V if h('e') else 0.0)
            stop   = h('x')

            modules = swerve(fwd, strafe, rot)
            for i, (speed, angle) in enumerate(modules):
                vpubs[i].set(0.0 if stop else speed)
                spubs[i].set(angle)

            shoot_tilt = clamp(
                shoot_tilt + AIM_SPEED * dt * ((1 if h('t') else 0) - (1 if h('g') else 0)),
                -math.pi/2, math.pi/2)
            shoot_pan = clamp(
                shoot_pan  + AIM_SPEED * dt * ((1 if h('f') else 0) - (1 if h('h') else 0)),
                -math.pi, math.pi)

            plus_now  = h('=')
            minus_now = h('-')
            if plus_now and not prev_plus:
                shoot_speed = clamp(shoot_speed + SPEED_STEP, 1.0, 30.0)
                print(f"\r\n  shoot speed: {shoot_speed:.1f} m/s\n")
            if minus_now and not prev_minus:
                shoot_speed = clamp(shoot_speed - SPEED_STEP, 1.0, 30.0)
                print(f"\r\n  shoot speed: {shoot_speed:.1f} m/s\n")
            prev_plus, prev_minus = plus_now, minus_now

            firing = h('space')
            fire_pub.set(firing)
            speed_pub.set(shoot_speed)
            dir_pub.set(aim_dir(shoot_tilt, shoot_pan))

            if tick % 10 == 0:
                connected = len(inst.getConnections()) > 0
                held_str = ','.join(sorted(k for k, v in key_age.items() if v <= KEY_DECAY))
                print(f"\r  {'SIM:connected' if connected else 'SIM:waiting.. '}  "
                      f"fwd={fwd:+.1f} str={strafe:+.1f} rot={rot:+.1f}  "
                      f"tilt={math.degrees(shoot_tilt):+.0f}° pan={math.degrees(shoot_pan):+.0f}°  "
                      f"spd={shoot_speed:.0f}m/s  {'FIRE' if firing else '    '}  "
                      f"keys=[{held_str}]   ",
                      end='', flush=True)

            elapsed = time.monotonic() - t0
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)

    for p in vpubs: p.set(0.0)
    for p in spubs: p.set(0.0)
    fire_pub.set(False)
    speed_pub.set(0.0)
    time.sleep(0.1)
    inst.stopServer()
    print("\nstopped.")

if __name__ == "__main__":
    main()
