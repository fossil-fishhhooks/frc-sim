#!/usr/bin/env python3
"""
multiplayer-dashboard.py — GUI swerve controller + video stream for frc_sim

  W/S    — forward/back        T/G  — tilt aim
  A/D    — strafe              F/H  — pan aim
  Q/E    — rotate              +/-  — fire speed
  Space  — fire                X    — stop

Install:  pip install robotpy opencv-python pillow
"""

import math, time, sys, os, socket, threading, queue
import tkinter as tk
from tkinter import font as tkfont

# ── Optional deps ─────────────────────────────────────────────────────────────
try:
    import ntcore
except ImportError:
    print("Missing: pip install robotpy"); sys.exit(1)

try:
    import cv2
    import numpy as np
    from PIL import Image, ImageTk
    HAS_VIDEO = True
except ImportError:
    HAS_VIDEO = False

# ── Config ────────────────────────────────────────────────────────────────────
NT_PORT     = 5810
DRIVE_V     = 0.7
ROTATE_V    = 0.5
AIM_SPEED   = 1.5
SHOOT_SPEED = 12.0
SPEED_STEP  = 1.0
TICK_HZ     = 50
KEY_DECAY   = 4

# ── NT4 publishers ────────────────────────────────────────────────────────────
inst  = ntcore.NetworkTableInstance.getDefault()
vpubs = [inst.getFloatTopic(f"/sim/motors/{i}/voltage").publish()     for i in range(4)]
spubs = [inst.getFloatTopic(f"/sim/motors/{i}/steer_angle").publish() for i in range(4)]
fire_pub  = inst.getBooleanTopic("/sim/shooter/fire").publish()
speed_pub = inst.getFloatTopic("/sim/shooter/speed").publish()
dir_pub   = inst.getFloatArrayTopic("/sim/shooter/direction").publish()

# ── Kinematics (unchanged from controller.py) ─────────────────────────────────
def swerve(fwd, strafe, rot):
    modules = [(1,1),(-1,1),(1,-1),(-1,-1)]
    out = []
    for mx, mz in modules:
        vx = fwd    + rot * (-mz)
        vz = -strafe + rot * ( mx)
        out.append((min(math.hypot(vx, vz), 1.0), math.atan2(vz, vx)))
    return out

def clamp(v, lo, hi): return max(lo, min(hi, v))

def aim_dir(tilt, pan):
    x = -(math.cos(tilt) * math.cos(pan))
    y =   math.sin(tilt)
    z =  -math.cos(tilt) * math.sin(pan)
    n = math.sqrt(x*x + y*y + z*z)
    return [x/n, y/n, z/n]

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"

# ── Video thread ──────────────────────────────────────────────────────────────
class VideoThread(threading.Thread):
    def __init__(self, url, frame_q):
        super().__init__(daemon=True)
        self.url     = url
        self.frame_q = frame_q
        self.running = True

    def run(self):
        cap = self._open()
        frame_bytes = None

        while self.running:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                cap.release()
                cap = self._open()
                frame_bytes = None
                continue

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Always discard stale frames — only keep the absolute latest
            while not self.frame_q.empty():
                try: self.frame_q.get_nowait()
                except queue.Empty: break
            self.frame_q.put(frame)

        cap.release()

    def _open(self):
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        # Minimize opencv's internal decode buffer — 1 = smallest allowed
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Tell ffmpeg backend to not buffer (passed through CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
        # Disable any frame dropping compensation
        cap.set(cv2.CAP_PROP_FPS, 0)
        return cap

    def stop(self):
        self.running = False

# ── Control tick thread ───────────────────────────────────────────────────────
class ControlThread(threading.Thread):
    """Runs at TICK_HZ, reads shared state dict, publishes to NT."""
    def __init__(self, state):
        super().__init__(daemon=True)
        self.state   = state
        self.running = True

    def run(self):
        dt          = 1.0 / TICK_HZ
        shoot_tilt  = 0.3
        shoot_pan   = 0.0
        prev_plus   = False
        prev_minus  = False
        key_age     = {}

        while self.running:
            t0 = time.monotonic()

            # pull freshly-pressed keys from GUI
            fresh = self.state.pop('keys_fresh', set())
            key_age = {k: v + 1 for k, v in key_age.items() if v + 1 <= KEY_DECAY}
            for k in fresh:
                key_age[k] = 0

            def h(*keys):
                return any(key_age.get(k, KEY_DECAY+1) <= KEY_DECAY for k in keys)

            fwd    = (DRIVE_V  if h('w') else 0.0) - (DRIVE_V  if h('s') else 0.0)
            strafe = (DRIVE_V  if h('a') else 0.0) - (DRIVE_V  if h('d') else 0.0)
            rot    = (ROTATE_V if h('q') else 0.0) - (ROTATE_V if h('e') else 0.0)
            stop   = h('x')

            for i, (speed, angle) in enumerate(swerve(fwd, strafe, rot)):
                vpubs[i].set(0.0 if stop else speed)
                spubs[i].set(angle)

            shoot_tilt = clamp(shoot_tilt + AIM_SPEED * dt *
                ((1 if h('t') else 0) - (1 if h('g') else 0)), -math.pi/2, math.pi/2)
            shoot_pan  = clamp(shoot_pan  + AIM_SPEED * dt *
                ((1 if h('f') else 0) - (1 if h('h') else 0)), -math.pi, math.pi)

            shoot_speed = self.state.get('shoot_speed', SHOOT_SPEED)
            plus_now, minus_now = h('='), h('-')
            if plus_now  and not prev_plus:
                shoot_speed = clamp(shoot_speed + SPEED_STEP, 1.0, 30.0)
            if minus_now and not prev_minus:
                shoot_speed = clamp(shoot_speed - SPEED_STEP, 1.0, 30.0)
            prev_plus, prev_minus = plus_now, minus_now
            self.state['shoot_speed'] = shoot_speed

            firing = h('space')
            fire_pub.set(firing)
            speed_pub.set(shoot_speed)
            dir_pub.set(aim_dir(shoot_tilt, shoot_pan))

            self.state.update({
                'fwd': fwd, 'strafe': strafe, 'rot': rot,
                'tilt_deg': math.degrees(shoot_tilt),
                'pan_deg':  math.degrees(shoot_pan),
                'firing':    firing,
                'connected': len(inst.getConnections()) > 0,
                'held':      sorted(k for k, v in key_age.items() if v <= KEY_DECAY),
            })

            elapsed = time.monotonic() - t0
            rem = dt - elapsed
            if rem > 0:
                time.sleep(rem)

    def stop(self):
        self.running = False

# ── Dashboard GUI ─────────────────────────────────────────────────────────────
BG     = "#0b0d11"
PANEL  = "#11151c"
BORDER = "#1c2230"
ACCENT = "#00e5ff"
RED    = "#ff3d71"
GREEN  = "#00e676"
TEXT   = "#c8d6e5"
DIM    = "#3a4a5c"
MONO   = "Courier New"

def label(parent, text, fg=TEXT, size=9, bold=False, **kw):
    weight = "bold" if bold else "normal"
    return tk.Label(parent, text=text, bg=parent["bg"],
                    fg=fg, font=(MONO, size, weight), **kw)

def divider(parent):
    tk.Frame(parent, bg=BORDER, height=1).pack(fill="x", padx=8, pady=3)

def btn(parent, text, command):
    b = tk.Label(parent, text=text, bg=ACCENT, fg=BG,
                 font=(MONO, 8, "bold"), padx=10, pady=4,
                 cursor="hand2", relief="flat")
    b.bind("<Button-1>", lambda e: command())
    b.bind("<Enter>",    lambda e: b.config(bg=RED))
    b.bind("<Leave>",    lambda e: b.config(bg=ACCENT))
    return b

class Dashboard:
    def __init__(self, root):
        self.root      = root
        self.state     = {'shoot_speed': SHOOT_SPEED}
        self.frame_q   = queue.Queue(maxsize=2)
        self.vid_thread = None
        self.photo      = None
        self.ctrl       = ControlThread(self.state)

        root.title("FRC SIM 3D by Arin J — Multiplayer Dashboard")
        root.configure(bg=BG)
        root.minsize(960, 580)

        self._build()
        self._bind_keys()

        inst.startServer()
        self.ctrl.start()

        root.after(16,  self._tick_video)
        root.after(80,  self._tick_status)
        root.protocol("WM_DELETE_WINDOW", self._shutdown)

    # ── Layout ────────────────────────────────────────────────────────────
    def _build(self):
        # ── Top bar ──────────────────────────────────────────────────────
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=14, pady=(8,4))

        label(top, "FRC SIM", fg=ACCENT, size=16, bold=True).pack(side="left")
        label(top, "  MULTIPLAYER DASHBOARD", fg=DIM, size=8).pack(side="left", pady=(5,0))
        self.conn_pill = label(top, "● WAITING", fg=RED, size=9, bold=True)
        self.conn_pill.pack(side="right")

        # ── Main body ─────────────────────────────────────────────────────
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=14, pady=(0,10))
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=0, minsize=240)
        body.rowconfigure(0, weight=1)

        # Left video panel
        self._build_video(body)
        # Right controls panel
        self._build_controls(body)

    def _build_video(self, parent):
        f = tk.Frame(parent, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        f.grid(row=0, column=0, sticky="nsew", padx=(0,8))

        # Stream URL row
        top = tk.Frame(f, bg=PANEL)
        top.pack(fill="x", padx=10, pady=(8,4))
        label(top, "VIDEO STREAM", fg=DIM, size=7).pack(side="left")
        self.stream_lbl = label(top, "DISCONNECTED", fg=RED, size=8, bold=True)
        self.stream_lbl.pack(side="right")

        url_row = tk.Frame(f, bg=PANEL)
        url_row.pack(fill="x", padx=10, pady=(0,6))
        label(url_row, "udp://", fg=DIM, size=10).pack(side="left")

        self.ip_var   = tk.StringVar(value="127.0.0.1")
        self.port_var = tk.StringVar(value="5000")

        def entry(var, w):
            e = tk.Entry(url_row, textvariable=var, width=w, bg=BORDER, fg=TEXT,
                         insertbackground=ACCENT, relief="flat",
                         font=(MONO, 10), bd=4)
            e.pack(side="left")
            return e

        entry(self.ip_var, 13)
        label(url_row, ":", fg=DIM, size=10).pack(side="left")
        entry(self.port_var, 6)
        btn(url_row, "CONNECT", self._connect).pack(side="left", padx=(8,0))

        # Canvas
        self.canvas = tk.Canvas(f, bg="#000", bd=0, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=(0,10))
        self._placeholder = self.canvas.create_text(
            320, 180, text="No stream\nConnect a udp:// source above",
            fill=DIM, font=(MONO, 11), justify="center")

    def _build_controls(self, parent):
        f = tk.Frame(parent, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        f.grid(row=0, column=1, sticky="nsew")

        # My IP
        ip_f = tk.Frame(f, bg=PANEL)
        ip_f.pack(fill="x", padx=12, pady=(12,4))
        label(ip_f, "YOUR IP", fg=DIM, size=7).pack(anchor="w")
        label(ip_f, get_local_ip(), fg=ACCENT, size=14, bold=True).pack(anchor="w")
        label(ip_f, f"NT server on :{NT_PORT}", fg=DIM, size=7).pack(anchor="w")

        divider(f)

        # Telemetry grid
        telem_f = tk.Frame(f, bg=PANEL)
        telem_f.pack(fill="x", padx=12, pady=4)
        label(telem_f, "TELEMETRY", fg=DIM, size=7).pack(anchor="w", pady=(0,4))

        self.telem = {}
        rows = [
            ("FWD",   'fwd',       "{:+.2f}"),
            ("STR",   'strafe',    "{:+.2f}"),
            ("ROT",   'rot',       "{:+.2f}"),
            ("TILT",  'tilt_deg',  "{:+.0f}°"),
            ("PAN",   'pan_deg',   "{:+.0f}°"),
            ("SPD",   'shoot_speed',"{:.1f} m/s"),
        ]
        for lbl_text, key, fmt in rows:
            row = tk.Frame(telem_f, bg=PANEL)
            row.pack(fill="x", pady=1)
            label(row, f"{lbl_text:<6}", fg=DIM, size=9).pack(side="left")
            var = tk.StringVar(value="—")
            self.telem[key] = (var, fmt)
            tk.Label(row, textvariable=var, bg=PANEL, fg=TEXT,
                     font=(MONO, 9, "bold")).pack(side="right")

        divider(f)

        # Fire indicator
        self.fire_lbl = tk.Label(f, text="■  SHOOT  ■", bg=PANEL, fg=BORDER,
                                 font=(MONO, 18, "bold"))
        self.fire_lbl.pack(pady=8)

        divider(f)

        # Bindings
        bind_f = tk.Frame(f, bg=PANEL)
        bind_f.pack(fill="x", padx=12, pady=4)
        label(bind_f, "BINDINGS", fg=DIM, size=7).pack(anchor="w", pady=(0,3))
        for k, desc in [("W/S","fwd/back"),("A/D","strafe"),("Q/E","rotate"),
                         ("T/G","tilt"),("F/H","pan"),("SPC","fire"),
                         ("+/-","speed"),("X","stop")]:
            r = tk.Frame(bind_f, bg=PANEL)
            r.pack(fill="x", pady=1)
            label(r, f"{k:<5}", fg=ACCENT, size=8, bold=True).pack(side="left")
            label(r, desc, fg=DIM, size=8).pack(side="left")

        divider(f)

        # Active keys
        keys_f = tk.Frame(f, bg=PANEL)
        keys_f.pack(fill="x", padx=12, pady=(4,12))
        label(keys_f, "HELD", fg=DIM, size=7).pack(anchor="w")
        self.keys_lbl = tk.Label(keys_f, text="—", bg=PANEL, fg=RED,
                                 font=(MONO, 11, "bold"), anchor="w")
        self.keys_lbl.pack(anchor="w")

    # ── Key input ─────────────────────────────────────────────────────────
    def _bind_keys(self):
        self.root.bind("<KeyPress>",   self._kp)
        self.root.bind("<KeyRelease>", self._kr)
        self._held_gui = set()

    def _resolve(self, event):
        sym = event.keysym.lower()
        ch  = event.char.lower() if event.char else ""
        if sym == "space":  return "space"
        if sym == "escape": return "esc"
        if ch in "wsadqetgfhx=-": return ch
        return None

    def _kp(self, event):
        k = self._resolve(event)
        if k:
            self._held_gui.add(k)
            fresh = self.state.setdefault('keys_fresh', set())
            fresh.add(k)

    def _kr(self, event):
        k = self._resolve(event)
        if k:
            self._held_gui.discard(k)

    # ── Video ─────────────────────────────────────────────────────────────
    def _connect(self):
        if not HAS_VIDEO:
            self.stream_lbl.config(text="NEED opencv-python pillow", fg=RED)
            return
        if self.vid_thread:
            self.vid_thread.stop()
        url = f"udp://{self.ip_var.get().strip()}:{self.port_var.get().strip()}"
        self.stream_lbl.config(text="CONNECTING…", fg=DIM)
        self.frame_q   = queue.Queue(maxsize=2)
        self.vid_thread = VideoThread(url, self.frame_q)
        self.vid_thread.start()

    def _tick_video(self):
        if HAS_VIDEO:
            frame = None
            # Drain entire queue, keep only last
            while True:
                try:
                    frame = self.frame_q.get_nowait()
                except queue.Empty:
                    break

            if frame is not None:
                cw = self.canvas.winfo_width()
                ch = self.canvas.winfo_height()
                if cw > 1 and ch > 1:
                    img   = Image.fromarray(frame)
                    scale = min(cw / img.width, ch / img.height)
                    img   = img.resize((int(img.width*scale), int(img.height*scale)),
                                       Image.BILINEAR)
                    self.photo = ImageTk.PhotoImage(img)
                    self.canvas.delete("all")
                    self.canvas.create_image(cw//2, ch//2, image=self.photo, anchor="center")
                    self.stream_lbl.config(text="● LIVE", fg=GREEN)

        self.root.after(16, self._tick_video)

    # ── Status ────────────────────────────────────────────────────────────
    def _tick_status(self):
        s = self.state

        # NT pill
        if s.get('connected'):
            self.conn_pill.config(text="● CONNECTED", fg=GREEN)
        else:
            self.conn_pill.config(text="● WAITING",   fg=RED)

        # Telemetry
        for key, (var, fmt) in self.telem.items():
            try:    var.set(fmt.format(s.get(key, 0.0)))
            except: var.set("—")

        # Fire
        self.fire_lbl.config(fg=RED if s.get('firing') else BORDER)

        # Keys
        held = s.get('held', [])
        self.keys_lbl.config(text=" ".join(k.upper() for k in held) if held else "—")

        self.root.after(80, self._tick_status)

    # ── Shutdown ──────────────────────────────────────────────────────────
    def _shutdown(self):
        self.ctrl.stop()
        if self.vid_thread:
            self.vid_thread.stop()
        for p in vpubs: p.set(0.0)
        for p in spubs: p.set(0.0)
        fire_pub.set(False)
        speed_pub.set(0.0)
        time.sleep(0.05)
        inst.stopServer()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    Dashboard(root)
    root.mainloop()
