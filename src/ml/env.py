"""
env.py — Gym environment wrapping frc_sim over NT4.

Observation vector layout (obs_dim = scalar_dim + n_rays = 23 + 58 = 81):
  [0:9]    robot pose  — x/field, z/field, qx, qy, qz, qw, intake_ix, intake_iz, speed
  [9:21]   motors      — 4× (omega/free_speed, steer_angle/pi, last_voltage)
  [21:23]  intake      — held_frac, has_piece (binary)
  [23:55]  rays piece  — 32 floats, 0=hit at origin 1=no hit/max_dist (11.25° spacing)
  [55:73]  rays field low  — 18 floats, horizontal field geometry (20° spacing)
  [73:81]  rays field high — 8 floats, elevated field geometry (45° spacing, pitch -40°)

Action vector (3 continuous, tanh → [-1,1]):
  [0] fwd    [1] strafe    [2] rot
"""

import collections
import math
import threading
import time

import numpy as np
import gymnasium as gym
from gymnasium import spaces

try:
    import ntcore
except ImportError:
    raise ImportError("pip install robotpy")

from config import Config


# ── Helpers ───────────────────────────────────────────────────────────────────

def swerve(fwd: float, strafe: float, rot: float):
    modules = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
    result  = []
    for mx, mz in modules:
        vx    = fwd     + rot * (-mz)
        vz    = -strafe + rot * ( mx)
        speed = min(math.hypot(vx, vz), 1.0)
        angle = math.atan2(vz, vx)
        result.append((speed, angle))
    return result


def aim_dir(tilt: float, pan: float):
    x = -(math.cos(tilt) * math.cos(pan))
    y =   math.sin(tilt)
    z =  -math.cos(tilt) * math.sin(pan)
    n = math.sqrt(x*x + y*y + z*z)
    return [x/n, y/n, z/n]


# ── Environment ───────────────────────────────────────────────────────────────

class FRCSimEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, cfg: Config | None = None, port: int | None = None):
        super().__init__()
        self.cfg = cfg or Config()
        c = self.cfg

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(c.obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32
        )

        self._inst     = ntcore.NetworkTableInstance.create()
        self._nt_port  = port or c.nt_port
        self._setup_nt()

        self._first_reset  = True
        self._shoot_tilt   = 0.3
        self._shoot_pan    = 0.0
        self._prev_raw: dict | None = None
        self._step_count   = 0
        self._step_times: list[float] = []
        self._intake_count = 0
        self._last_voltages = [0.0] * 4
        self._low_speed_steps = 0

        self._last_vpub = [0.0] * 4
        self._last_spub = [0.0] * 4
        self._last_fire = False
        self._nt_alive  = True
        self._nt_thread = threading.Thread(target=self._nt_publisher, daemon=True)
        self._nt_thread.start()

        self._dbg = self._fresh_dbg()

        self._last_match_time  = 0.0
        self._stale_deadline: float | None = None
        self._action_mismatch_steps = 0
        self._action_grace_steps    = 0
        self._match_done  = False
        self._last_obs: np.ndarray | None = None

    def _fresh_dbg(self) -> dict:
        return {"rate": 0.0, "idle": 0.0, "spin": 0.0, "oob": 0.0, "n": 0}

    def connected_count(self) -> int:
        return len(self._inst.getConnections())

    # ── NT setup ──────────────────────────────────────────────────────────────

    def _setup_nt(self):
        inst = self._inst
        inst.startServer(port4=self._nt_port)

        self._vpubs    = [inst.getFloatTopic(f"/sim/motors/{i}/voltage").publish()
                          for i in range(4)]
        self._spubs    = [inst.getFloatTopic(f"/sim/motors/{i}/steer_angle").publish()
                          for i in range(4)]
        self._fire_pub  = inst.getBooleanTopic("/sim/shooter/fire").publish()
        self._speed_pub = inst.getFloatTopic("/sim/shooter/speed").publish()
        self._dir_pub   = inst.getFloatArrayTopic("/sim/shooter/direction").publish()
        self._reset_pub = inst.getBooleanTopic("/sim/reset").publish()
        self._reset_sub = inst.getBooleanTopic("/sim/reset").subscribe(False)

        self._pose_x_sub  = inst.getFloatTopic("/sim/robot/x").subscribe(0.0)
        self._pose_z_sub  = inst.getFloatTopic("/sim/robot/z").subscribe(0.0)
        self._pose_qx_sub = inst.getFloatTopic("/sim/robot/qx").subscribe(0.0)
        self._pose_qy_sub = inst.getFloatTopic("/sim/robot/qy").subscribe(0.0)
        self._pose_qz_sub = inst.getFloatTopic("/sim/robot/qz").subscribe(0.0)
        self._pose_qw_sub = inst.getFloatTopic("/sim/robot/qw").subscribe(1.0)
        self._vx_sub      = inst.getFloatTopic("/sim/robot/vx").subscribe(0.0)
        self._vz_sub      = inst.getFloatTopic("/sim/robot/vz").subscribe(0.0)
        self._oob_sub     = inst.getBooleanTopic("/sim/robot/oob").subscribe(False)

        self._omega_subs = [inst.getFloatTopic(f"/sim/motors/{i}/omega").subscribe(0.0)
                            for i in range(4)]
        self._steer_subs = [inst.getFloatTopic(f"/sim/motors/{i}/steer_angle").subscribe(0.0)
                            for i in range(4)]

        self._intake_held_sub = inst.getIntegerTopic("/sim/intake/held").subscribe(0)
        self._match_time_sub  = inst.getFloatTopic("/sim/match/time").subscribe(0.0)
        self._phase_sub       = inst.getStringTopic("/sim/match/phase").subscribe("waiting")

        # Raycast hits — flat float[] published by sim, one value per ray
        self._raycast_sub = inst.getFloatArrayTopic(self.cfg.ray_topic).subscribe([])

    # ── Background publisher ──────────────────────────────────────────────────

    def _nt_publisher(self):
        while self._nt_alive:
            for i in range(4):
                self._vpubs[i].set(self._last_vpub[i])
                self._spubs[i].set(self._last_spub[i])
            self._fire_pub.set(self._last_fire)
            time.sleep(0.02)

    # ── Heartbeat / freeze detection ──────────────────────────────────────────

    def _sim_alive(self, match_time: float, match_ended: bool) -> bool:
        if match_ended:
            self._stale_deadline = None
            return True
        if match_time > self._last_match_time:
            self._last_match_time = match_time
            self._stale_deadline  = None
        else:
            now = time.monotonic()
            if self._stale_deadline is None:
                self._stale_deadline = now + 2.0
            elif now > self._stale_deadline:
                return False
        return True

    def _action_accepted(self, curr_raw: dict, fwd: float, strafe: float,
                         rot: float, match_ended: bool) -> bool:
        if match_ended:
            self._action_mismatch_steps = 0
            return True
        phase = curr_raw["phase"]
        if phase not in ("auto", "teleop"):
            self._action_mismatch_steps = 0
            return True
        GRACE_STEPS = 200
        if self._action_grace_steps < GRACE_STEPS:
            self._action_grace_steps += 1
            return True
        action_mag = abs(fwd) + abs(strafe) + abs(rot)
        max_omega  = max(abs(o) for o in curr_raw["omegas"])
        if action_mag > 0.15 and max_omega < 0.02:
            self._action_mismatch_steps += 1
            if self._action_mismatch_steps >= 15:
                print(f"  [INPUT FREEZE] port={self._nt_port} "
                      f"action_mag={action_mag:.2f} max_omega={max_omega:.3f}")
                return False
        else:
            self._action_mismatch_steps = 0
        return True

    # ── Observation ───────────────────────────────────────────────────────────

    def _read_raw(self) -> dict:
        c = self.cfg

        robot_x  = self._pose_x_sub.get()
        robot_z  = self._pose_z_sub.get()
        robot_vx = self._vx_sub.get()
        robot_vz = self._vz_sub.get()
        qx = self._pose_qx_sub.get()
        qy = self._pose_qy_sub.get()
        qz = self._pose_qz_sub.get()
        qw = self._pose_qw_sub.get()

        yaw = math.atan2(2.0*(qw*qy - qx*qz), 1.0 - 2.0*(qy*qy + qz*qz))
        ix  = math.cos(yaw)    # intake faces robot forward (X-axis)
        iz  = math.sin(yaw)

        # Raycast hits — clamp to [0,1] defensively
        raw_hits = self._raycast_sub.get()
        n_expected = c.n_rays
        if len(raw_hits) >= n_expected:
            hits = [max(0.0, min(1.0, raw_hits[i])) for i in range(n_expected)]
        else:
            # Not yet received — default to no-hit (1.0 = nothing detected)
            hits = [1.0] * n_expected

        return {
            "robot_x":      robot_x,
            "robot_z":      robot_z,
            "robot_vx":     robot_vx,
            "robot_vz":     robot_vz,
            "oob":          self._oob_sub.get(),
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            "yaw":          yaw,
            "intake_ix":    ix,
            "intake_iz":    iz,
            "omegas":       [s.get() for s in self._omega_subs],
            "steer_angles": [s.get() for s in self._steer_subs],
            "intake_held":  int(self._intake_held_sub.get()),
            "match_time":   self._match_time_sub.get(),
            "phase":        self._phase_sub.get(),
            "hits":         hits,   # flat list of n_rays floats
        }

    def _normalize(self, raw: dict, last_voltages: list[float]) -> np.ndarray:
        c   = self.cfg
        obs = []

        # Pose (9)
        obs += [
            raw["robot_x"]  / c.field_half_x,
            raw["robot_z"]  / c.field_half_z,
            raw["qx"], raw["qy"], raw["qz"], raw["qw"],
            raw["intake_ix"],
            raw["intake_iz"],
            math.sqrt(raw["robot_vx"]**2 + raw["robot_vz"]**2) / 5.0,
        ]

        # Motors (12)
        for i in range(4):
            obs += [
                raw["omegas"][i]       / c.free_speed,
                raw["steer_angles"][i] / math.pi,
                last_voltages[i],
            ]

        # Intake (2)
        obs += [
            raw["intake_held"] / c.max_capacity,
            float(raw["intake_held"] > 0),
        ]

        # Raycasts (n_rays = 58)
        # Already normalised 0..1 by the sim. No further scaling needed.
        # Short ray = obstacle nearby; 1.0 = nothing in range.
        obs += raw["hits"]

        return np.array(obs, dtype=np.float32)

    # ── Reward ────────────────────────────────────────────────────────────────

    def compute_reward(self, prev: dict, curr: dict,
                       held_delta: int, action: np.ndarray) -> float:
        c = self.cfg
        r = 0.0

        # 1. Intake — primary signal
        r_rate = held_delta * c.w_intake_rate if held_delta > 0 else 0.0
        r += r_rate

        # 2. Idle penalty
        speed  = math.sqrt(curr["robot_vx"]**2 + curr["robot_vz"]**2)
        r_idle = -c.w_idle_penalty * max(0.0, 1.0 - speed)
        r += r_idle

        # 3. Spin penalty
        rotation    = abs(float(action[2]))
        translation = math.sqrt(float(action[0])**2 + float(action[1])**2)
        r_spin = 0.0
        if rotation > c.spin_rot_threshold and translation < c.spin_tran_threshold:
            r_spin = -c.w_spin_penalty * (rotation - c.spin_rot_threshold)
        r += r_spin

        # 4. Wall shaping
        margin = min(c.field_half_x - abs(curr["robot_x"]),
                     c.field_half_z - abs(curr["robot_z"]))
        r_wall = -c.w_wall_shaping * max(0.0, (1.0 - margin)**2) if margin < 1.0 else 0.0
        r += r_wall

        # 5. OOB terminal
        r_oob = -c.w_oob_penalty if curr["oob"] else 0.0
        r += r_oob

        d = self._dbg
        d["rate"] += r_rate
        d["idle"] += r_idle
        d["spin"] += r_spin
        d["oob"]  += r_oob
        d["n"]    += 1

        return r

    # ── Gym interface ─────────────────────────────────────────────────────────

    def _reset_pose_ok(self, raw: dict) -> bool:
        c = self.cfg
        return (abs(raw["robot_x"]) <= c.field_half_x - 0.3 and
                abs(raw["robot_z"]) <= c.field_half_z - 0.3)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        for p in self._vpubs: p.set(0.0)
        for p in self._spubs: p.set(0.0)
        self._fire_pub.set(False)
        self._last_vpub = [0.0] * 4
        self._last_spub = [0.0] * 4
        self._last_fire = False

        c   = self.cfg
        raw = None
        for attempt in range(3):
            offset_us      = self._inst.getServerTimeOffset()
            pulse_local_us = int(time.monotonic() * 1e6)
            pulse_nt_us    = (pulse_local_us + offset_us) if offset_us is not None else pulse_local_us

            self._reset_pub.set(True)
            time.sleep(0.25)
            self._reset_pub.set(False)

            deadline         = time.monotonic() + 2.0
            confirmed_running = False
            while time.monotonic() < deadline:
                time.sleep(0.01)
                ts = self._phase_sub.getAtomic()
                if ts.value in ("auto",) and ts.time > pulse_nt_us:
                    confirmed_running = True
                    break

            if not confirmed_running:
                continue

            time.sleep(0.05)
            raw = self._read_raw()
            if self._reset_pose_ok(raw):
                break
        else:
            raise RuntimeError(
                f"reset did not converge after 3 attempts "
                f"(x={raw['robot_x']:.2f}, z={raw['robot_z']:.2f})"
            )

        self._shoot_tilt   = 0.3
        self._shoot_pan    = 0.0
        self._step_count   = 0
        self._step_times.clear()
        self._intake_count = 0
        self._last_voltages = [0.0] * 4
        self._dbg          = self._fresh_dbg()
        self._low_speed_steps = 0
        self._prev_raw     = raw
        self._last_match_time  = 0.0
        self._stale_deadline   = None
        self._action_mismatch_steps = 0
        self._action_grace_steps    = 0
        self._match_done  = False

        obs = self._normalize(raw, self._last_voltages)
        self._last_obs = obs
        return obs, {}

    def step(self, action: np.ndarray):
        if self._match_done:
            return self._last_obs, 0.0, False, False, {
                "phase": "ended", "score": 0,
                "intake_count": self._intake_count,
                "match_time": self._prev_raw["match_time"] if self._prev_raw else 0.0,
                "oob": False, "match_ended": True,
            }

        c = self.cfg
        fwd, strafe, rot = float(action[0]), float(action[1]), float(action[2]) * 0.5

        modules = swerve(fwd, strafe, rot)
        self._last_voltages = [v for v, _ in modules]
        for i, (v, a) in enumerate(modules):
            self._vpubs[i].set(float(v))
            self._spubs[i].set(float(a))
            self._last_vpub[i] = float(v)
            self._last_spub[i] = float(a)
        self._fire_pub.set(False)
        self._last_fire = False
        self._speed_pub.set(15.0)
        self._dir_pub.set(aim_dir(self._shoot_tilt, self._shoot_pan))

        curr_raw   = self._read_raw()
        held_delta = curr_raw["intake_held"] - self._prev_raw["intake_held"]
        if held_delta > 0:
            self._intake_count += held_delta

        # Stuck-wall softlock detection
        margin = min(c.field_half_x - abs(curr_raw["robot_x"]),
                     c.field_half_z - abs(curr_raw["robot_z"]))
        speed  = math.sqrt(curr_raw["robot_vx"]**2 + curr_raw["robot_vz"]**2)
        if margin < 0.0 and speed < 0.05:
            self._low_speed_steps += 1
        else:
            self._low_speed_steps = 0
        if self._low_speed_steps >= 100:
            curr_raw["oob"] = True

        reward = self.compute_reward(self._prev_raw, curr_raw, held_delta, action)

        self._prev_raw    = curr_raw
        self._step_count += 1

        obs   = self._normalize(curr_raw, self._last_voltages)
        self._last_obs = obs
        phase = curr_raw["phase"]

        oob         = curr_raw["oob"]
        match_ended = (phase == "ended")
        if not self._sim_alive(curr_raw["match_time"], match_ended):
            print(f"  [SIM FREEZE] port={self._nt_port} match_time={curr_raw['match_time']:.1f}")
            match_ended = True
        if not self._action_accepted(curr_raw, fwd, strafe, rot, match_ended):
            match_ended = True

        terminated = oob
        truncated  = self._step_count >= c.max_episode_steps

        if terminated or truncated or match_ended:
            for i in range(4):
                self._vpubs[i].set(0.0)
                self._last_vpub[i] = 0.0

        if match_ended and not terminated and not truncated:
            self._match_done = True
            self._last_obs   = obs

        return obs, reward, terminated, truncated, {
            "phase":        phase,
            "score":        0,
            "intake_count": self._intake_count,
            "match_time":   curr_raw["match_time"],
            "oob":          oob,
            "match_ended":  match_ended,
        }

    def close(self):
        self._nt_alive = False
        self._nt_thread.join(timeout=1.0)
        for p in self._vpubs: p.set(0.0)
        for p in self._spubs: p.set(0.0)
        self._fire_pub.set(False)
        self._inst.stopServer()