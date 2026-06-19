#!/usr/bin/env python3
"""
multiplayer-dashboard-xbox.py — Xbox controller dashboard for frc_sim

  Left stick   — drive (fwd/back + strafe)     A        — fire
  Right stick  — rotate (X) + tilt aim (Y)     B        — stop
  LB/RB        — fire speed down/up            X/Y      — pan aim

Install:  pip install robotpy opencv-pillow pygame
"""

import math, time, sys, os, socket, threading, queue
import tkinter as tk
from tkinter import font as tkfont

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

try:
    os.environ["SDL_VIDEODRIVER"] = "dummy"
    import pygame
    pygame.init()
    pygame.joystick.init()
    HAS_GAMEPAD = pygame.joystick.get_count() > 0
    if HAS_GAMEPAD:
        _gp = pygame.joystick.Joystick(0)
        _gp.init()
except Exception:
    HAS_GAMEPAD = False

NT_PORT     = 5810
DRIVE_V     = 0.7
ROTATE_V    = 0.5
AIM_SPEED   = 1.5
SHOOT_SPEED = 12.0
SPEED_STEP  = 1.0
TICK_HZ     = 50
DEADZONE    = 0.12

inst  = ntcore.NetworkTableInstance.getDefault()
vpubs = [inst.getFloatTopic(f"/sim/motors/{i}/voltage").publish()     for i in range(4)]
spubs = [inst.getFloatTopic(f"/sim/motors/{i}/steer_angle").publish() for i in range(4)]
fire_pub  = inst.getBooleanTopic("/sim/shooter/fire").publish()
speed_pub = inst.getFloatTopic("/sim/shooter/speed").publish()
dir_pub   = inst.getFloatArrayTopic("/sim/shooter/direction").publish()

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

def dz(v):
    return 0.0 if abs(v) < DEADZONE else v

class VideoThread(threading.Thread):
    def __init__(self, url, frame_q):
        super().__init__(daemon=True)
        self.url     = url
        self.frame_q = frame_q
        self.running = True

    def run(self):
        cap = self._open()
        while self.running:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                cap.release()
                cap = self._open()
                continue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            while not self.frame_q.empty():
                try: self.frame_q.get_nowait()
                except queue.Empty: break
            self.frame_q.put(frame)
        cap.release()

    def _open(self):
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
        cap.set(cv2.CAP_PROP_FPS, 0)
        return cap

    def stop(self):
        self.running = False

class ControlThread(threading.Thread):
    def __init__(self, state):
        super().__init__(daemon=True)
        self.state   = state
        self.running = True

    def run(self):
        dt          = 1.0 / TICK_HZ
        shoot_tilt  = 0.3
        shoot_pan   = 0.0
        prev_lb     = False
        prev_rb     = False

        pose_sub = inst.getFloatTopic("/sim/robot/x").subscribe(0.0)

        while self.running:
            t0 = time.monotonic()

            if HAS_GAMEPAD:
                pygame.event.pump()
                lx = dz(_gp.get_axis(0))
                ly = dz(_gp.get_axis(1))
                rx = dz(_gp.get_axis(2))
                hat = _gp.get_hat(0)
                a  = _gp.get_button(0)
                b  = _gp.get_button(1)
                x  = _gp.get_button(2)
                y  = _gp.get_button(3)
                lb = _gp.get_button(4)
                rb = _gp.get_button(5)
            else:
                lx = ly = rx = 0.0
                hat = (0, 0)
                a = b = x = y = lb = rb = False

            fwd    = -ly * DRIVE_V
            strafe =  lx * DRIVE_V
            rot    =  rx * ROTATE_V
            stop   = b

            for i, (speed, angle) in enumerate(swerve(fwd, strafe, rot)):
                vpubs[i].set(0.0 if stop else speed)
                spubs[i].set(angle)

            shoot_tilt = clamp(shoot_tilt + hat[1] * AIM_SPEED * dt,
                               -math.pi/2, math.pi/2)
            shoot_pan  = clamp(shoot_pan  - hat[0] * AIM_SPEED * dt,
                               -math.pi, math.pi)

            shoot_speed = self.state.get('shoot_speed', SHOOT_SPEED)
            if lb and not prev_lb:
                shoot_speed = clamp(shoot_speed - SPEED_STEP, 1.0, 30.0)
            if rb and not prev_rb:
                shoot_speed = clamp(shoot_speed + SPEED_STEP, 1.0, 30.0)
            prev_lb, prev_rb = lb, rb
            self.state['shoot_speed'] = shoot_speed

            firing = bool(a)
            fire_pub.set(firing)
            speed_pub.set(shoot_speed)
            dir_pub.set(aim_dir(shoot_tilt, shoot_pan))

            self.state.update({
                'fwd': fwd, 'strafe': strafe, 'rot': rot,
                'tilt_deg': math.degrees(shoot_tilt),
                'pan_deg':  math.degrees(shoot_pan),
                'firing':    firing,
                'connected': pose_sub.getAtomic().time > 0,
            })

            elapsed = time.monotonic() - t0
            rem = dt - elapsed
            if rem > 0:
                time.sleep(rem)

    def stop(self):
        self.running = False

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
        self.root       = root
        self.state      = {'shoot_speed': SHOOT_SPEED}
        self.frame_q    = queue.Queue(maxsize=2)
        self.vid_thread = None
        self.photo      = None
        self.ctrl       = ControlThread(self.state)

        root.title("FRC SIM 3D by Arin J — Xbox Dashboard")
        root.configure(bg=BG)
        root.minsize(960, 580)
        self._build()
        inst.startServer()
        self.ctrl.start()
        root.after(16,  self._tick_video)
        root.after(80,  self._tick_status)
        root.protocol("WM_DELETE_WINDOW", self._shutdown)

    def _build(self):
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=14, pady=(8,4))
        label(top, "FRC SIM", fg=ACCENT, size=16, bold=True).pack(side="left")
        label(top, "  XBOX DASHBOARD", fg=DIM, size=8).pack(side="left", pady=(5,0))
        self.conn_pill = label(top, "● WAITING", fg=RED, size=9, bold=True)
        self.conn_pill.pack(side="right")

        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=14, pady=(0,10))
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=0, minsize=240)
        body.rowconfigure(0, weight=1)
        self._build_video(body)
        self._build_controls(body)

    def _build_video(self, parent):
        f = tk.Frame(parent, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        f.grid(row=0, column=0, sticky="nsew", padx=(0,8))
        top = tk.Frame(f, bg=PANEL)
        top.pack(fill="x", padx=10, pady=(8,4))
        label(top, "VIDEO STREAM", fg=DIM, size=7).pack(side="left")
        self.stream_lbl = label(top, "DISCONNECTED", fg=RED, size=8, bold=True)
        self.stream_lbl.pack(side="right")
        url_row = tk.Frame(f, bg=PANEL)
        url_row.pack(fill="x", padx=10, pady=(0,6))
        label(url_row, "http://", fg=DIM, size=10).pack(side="left")
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
        self.canvas = tk.Canvas(f, bg="#000", bd=0, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=(0,10))
        self._placeholder = self.canvas.create_text(
            320, 180, text="No stream\nConnect a http:// source above",
            fill=DIM, font=(MONO, 11), justify="center")

    def _build_controls(self, parent):
        f = tk.Frame(parent, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        f.grid(row=0, column=1, sticky="nsew")
        ip_f = tk.Frame(f, bg=PANEL)
        ip_f.pack(fill="x", padx=12, pady=(12,4))
        label(ip_f, "YOUR IP", fg=DIM, size=7).pack(anchor="w")
        label(ip_f, get_local_ip(), fg=ACCENT, size=14, bold=True).pack(anchor="w")
        label(ip_f, f"NT server on :{NT_PORT}", fg=DIM, size=7).pack(anchor="w")
        divider(f)
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
        self.fire_lbl = tk.Label(f, text="■  SHOOT  ■", bg=PANEL, fg=BORDER,
                                 font=(MONO, 18, "bold"))
        self.fire_lbl.pack(pady=8)
        divider(f)
        bind_f = tk.Frame(f, bg=PANEL)
        bind_f.pack(fill="x", padx=12, pady=4)
        label(bind_f, "XBOX CONTROLS", fg=DIM, size=7).pack(anchor="w", pady=(0,3))
        for k, desc in [("L-STICK","drive"),("R-STICK","rotate"),
                         ("D-PAD","tilt / pan"),("A","fire"),
                         ("B","stop"),("LB/RB","speed")]:
            r = tk.Frame(bind_f, bg=PANEL)
            r.pack(fill="x", pady=1)
            label(r, f"{k:<10}", fg=ACCENT, size=8, bold=True).pack(side="left")
            label(r, desc, fg=DIM, size=8).pack(side="left")
        divider(f)
        status_f = tk.Frame(f, bg=PANEL)
        status_f.pack(fill="x", padx=12, pady=(4,12))
        label(status_f, "GAMEPAD", fg=DIM, size=7).pack(anchor="w")
        fg_col = RED if not HAS_GAMEPAD else GREEN
        self.gp_lbl = tk.Label(status_f,
                               text="NOT FOUND" if not HAS_GAMEPAD else "● CONNECTED",
                               bg=PANEL, fg=fg_col,
                               font=(MONO, 11, "bold"), anchor="w")
        self.gp_lbl.pack(anchor="w")

    def _connect(self):
        self.root.focus_set()
        if not HAS_VIDEO:
            self.stream_lbl.config(text="NEED opencv-python pillow", fg=RED)
            return
        if self.vid_thread:
            self.vid_thread.stop()
        url = f"http://{self.ip_var.get().strip()}:{self.port_var.get().strip()}"
        self.stream_lbl.config(text="CONNECTING…", fg=DIM)
        self.frame_q   = queue.Queue(maxsize=2)
        self.vid_thread = VideoThread(url, self.frame_q)
        self.vid_thread.start()

    def _tick_video(self):
        if HAS_VIDEO:
            frame = None
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
                    self.canvas.create_image(cw//2, ch//2,
                                             image=self.photo, anchor="center")
                    self.stream_lbl.config(text="● LIVE", fg=GREEN)
        self.root.after(16, self._tick_video)

    def _tick_status(self):
        s = self.state
        if s.get('connected'):
            self.conn_pill.config(text="● CONNECTED", fg=GREEN)
        else:
            self.conn_pill.config(text="● WAITING",   fg=RED)
        for key, (var, fmt) in self.telem.items():
            try:    var.set(fmt.format(s.get(key, 0.0)))
            except: var.set("—")
        self.fire_lbl.config(fg=RED if s.get('firing') else BORDER)
        self.root.after(80, self._tick_status)

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
