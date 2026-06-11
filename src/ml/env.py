"""
env.py — Gym environment wrapping frc_sim over NT4.

The sim is the physics engine; this file is pure wiring and reward.
All reward weights live in config.py — don't put magic numbers here.

Observation vector layout (obs_dim = 7 + 12 + 2 + n_pieces*3 + 4 + 3):
  [0:7]              robot pose  — x, z, qx, qy, qz, qw, speed
  [7:19]             motors      — 4× (omega, steer_angle, last_voltage)
  [19:21]            intake      — held_frac, has_piece (binary)
  [21:21+n*3]        game pieces — n× (dx, dz, dist) sorted nearest-first
  [21+n*3:25+n*3]    active zone — dx, dz, dist, points_norm
  [25+n*3:28+n*3]    match       — time_frac, score_mine_norm, score_opp_norm

Action vector (7 continuous, tanh → all in [-1,1]):
  [0] fwd            forward / back drive
  [1] strafe         left / right drive
  [2] rot            rotation
  [3] tilt_delta     shooter tilt rate  (integrated internally)
  [4] pan_delta       shooter pan rate   (integrated internally)
  [5] shoot_speed    remapped [-1,1] → [1, 30] m/s
  [6] fire           > 0 = fire
"""

import math
import time
import numpy as np
import gymnasium as gym
from gymnasium import spaces

try:
    import ntcore
except ImportError:
    raise ImportError("pip install robotpy")

from config import Config


# ── Hardcoded scoring zones (mirrors 955.json) ────────────────────────────────
# Each entry: (team, points, cx, cz, active_start, active_end)
# cx/cz are the X and Z coords of zone center (Y is height, irrelevant for 2D dist).
_ZONES = [
    (0, 1,  3.62,  0.0,   0,  30),  # auto_and_transition_blue
    (1, 1, -3.62,  0.0,   0,  30),  # auto_and_transition_red
    (1, 1, -3.62,  0.0,  30,  55),  # s1_red
    (0, 1,  3.62,  0.0,  55,  80),  # s2_blue
    (1, 1, -3.62,  0.0,  80, 105),  # s3_red
    (0, 1,  3.62,  0.0,  80, 105),  # s4_blue
    (0, 1,  3.62,  0.0, 105, 135),  # endgame_blue
    (1, 1, -3.62,  0.0, 105, 135),  # endgame_red
]


# ── Helpers (same as controller.py) ──────────────────────────────────────────

def swerve(fwd: float, strafe: float, rot: float):
    """Returns list of (speed, angle) per module. Module order matches robot.json."""
    modules = [(1, 1), (-1, 1), (1, -1), (-1, -1)]
    result = []
    for mx, mz in modules:
        vx = fwd    + rot * (-mz)
        vz = -strafe + rot * ( mx)
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

    def __init__(self, cfg: Config | None = None):
        super().__init__()
        self.cfg = cfg or Config()
        c = self.cfg

        # Observation / action spaces
        n = c.n_gamepieces
        obs_dim = 7 + 12 + 2 + n * 3 + 4 + 3
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(7,), dtype=np.float32
        )

        # NT setup
        self._inst = ntcore.NetworkTableInstance.getDefault()
        self._setup_nt()

        # Internal state
        self._shoot_tilt = 0.3
        self._shoot_pan  = 0.0
        self._prev_raw: dict | None = None
        self._prev_score_mine  = 0
        self._prev_score_opp   = 0
        self._prev_intake_held = 0
        self._step_count       = 0

        # Curriculum weight (0 → 1, set externally by train.py)
        self.efficiency_weight: float = 0.0

    # ── NT setup ──────────────────────────────────────────────────────────────

    def _setup_nt(self):
        inst = self._inst
        inst.startServer()

        # Outputs (we publish)
        self._vpubs = [inst.getFloatTopic(f"/sim/motors/{i}/voltage").publish()
                       for i in range(4)]
        self._spubs = [inst.getFloatTopic(f"/sim/motors/{i}/steer_angle").publish()
                       for i in range(4)]
        self._fire_pub   = inst.getBooleanTopic("/sim/shooter/fire").publish()
        self._speed_pub  = inst.getFloatTopic("/sim/shooter/speed").publish()
        self._dir_pub    = inst.getFloatArrayTopic("/sim/shooter/direction").publish()
        self._reset_pub  = inst.getBooleanTopic("/sim/reset").publish()

        # Inputs (sim publishes)
        self._pose_x_sub   = inst.getFloatTopic("/sim/robot/x").subscribe(0.0)
        self._pose_z_sub   = inst.getFloatTopic("/sim/robot/z").subscribe(0.0)
        self._pose_qx_sub  = inst.getFloatTopic("/sim/robot/qx").subscribe(0.0)
        self._pose_qy_sub  = inst.getFloatTopic("/sim/robot/qy").subscribe(0.0)
        self._pose_qz_sub  = inst.getFloatTopic("/sim/robot/qz").subscribe(0.0)
        self._pose_qw_sub  = inst.getFloatTopic("/sim/robot/qw").subscribe(1.0)
        self._vx_sub       = inst.getFloatTopic("/sim/robot/vx").subscribe(0.0)
        self._vz_sub       = inst.getFloatTopic("/sim/robot/vz").subscribe(0.0)

        self._omega_subs   = [inst.getFloatTopic(f"/sim/motors/{i}/omega").subscribe(0.0)
                               for i in range(4)]
        self._steer_subs   = [inst.getFloatTopic(f"/sim/motors/{i}/steer_angle").subscribe(0.0)
                               for i in range(4)]

        self._intake_held_sub = inst.getIntegerTopic("/sim/intake/held").subscribe(0)
        self._gamepieces_sub  = inst.getFloatArrayTopic("/sim/gamepieces").subscribe([])

        self._score0_sub    = inst.getIntegerTopic("/sim/score/team0").subscribe(0)
        self._score1_sub    = inst.getIntegerTopic("/sim/score/team1").subscribe(0)
        self._match_time_sub = inst.getFloatTopic("/sim/match/time").subscribe(0.0)
        self._phase_sub     = inst.getStringTopic("/sim/match/phase").subscribe("waiting")

        # NOTE: update this if your robot is team 1
        self._my_team = 0

    # ── Observation ───────────────────────────────────────────────────────────

    def _read_raw(self) -> dict:
        """Read all NT topics into a plain dict. No normalization here."""
        c = self.cfg

        robot_x  = self._pose_x_sub.get()
        robot_z  = self._pose_z_sub.get()
        robot_vx = self._vx_sub.get()
        robot_vz = self._vz_sub.get()

        # Parse gamepieces flat array → list of (dx, dz, dist) sorted by dist
        gp_flat = self._gamepieces_sub.get()
        pieces = []
        for i in range(0, len(gp_flat) - 2, 3):
            px, py, pz = gp_flat[i], gp_flat[i+1], gp_flat[i+2]
            dx = px - robot_x
            dz = pz - robot_z
            dist = math.sqrt(dx*dx + dz*dz)
            pieces.append((dx, dz, dist))
        pieces.sort(key=lambda p: p[2])
        pieces = pieces[:c.n_gamepieces]
        # Pad with zeros if fewer than n_gamepieces
        while len(pieces) < c.n_gamepieces:
            pieces.append((0.0, 0.0, c.max_piece_dist))

        scores = [self._score0_sub.get(), self._score1_sub.get()]
        match_time = self._match_time_sub.get()

        return {
            "robot_x":  robot_x,
            "robot_z":  robot_z,
            "robot_vx": robot_vx,
            "robot_vz": robot_vz,
            "qx": self._pose_qx_sub.get(),
            "qy": self._pose_qy_sub.get(),
            "qz": self._pose_qz_sub.get(),
            "qw": self._pose_qw_sub.get(),
            "omegas":       [s.get() for s in self._omega_subs],
            "steer_angles": [s.get() for s in self._steer_subs],
            "intake_held":  int(self._intake_held_sub.get()),
            "pieces":       pieces,          # list of (dx, dz, dist)
            "score_mine":   scores[self._my_team],
            "score_opp":    scores[1 - self._my_team],
            "match_time":   match_time,
            "phase":        self._phase_sub.get(),
            # nearest active zone — populated by _update_zone_obs() below
            "zone_dx":   0.0,
            "zone_dz":   0.0,
            "zone_dist": c.field_half_x * 2,
            "zone_pts":  1,
        }

    def _update_zone(self, raw: dict):
        """
        Find the nearest active zone for this robot's team and write it into raw.
        Uses hardcoded _ZONES — replace with NT subscriber when sim publishes them.
        """
        t         = raw["match_time"]
        robot_x   = raw["robot_x"]
        robot_z   = raw["robot_z"]
        best_dist = self.cfg.field_half_x * 4  # large sentinel
        best_dx   = 0.0
        best_dz   = 0.0
        best_pts  = 1

        for team, pts, cx, cz, t_start, t_end in _ZONES:
            if team != self._my_team:
                continue
            if not (t_start <= t < t_end):
                continue
            dx   = cx - robot_x
            dz   = cz - robot_z
            dist = math.sqrt(dx*dx + dz*dz)
            if dist < best_dist:
                best_dist = dist
                best_dx   = dx
                best_dz   = dz
                best_pts  = pts

        raw["zone_dx"]   = best_dx
        raw["zone_dz"]   = best_dz
        raw["zone_dist"] = best_dist
        raw["zone_pts"]  = best_pts

    def _normalize(self, raw: dict, last_voltages: list[float]) -> np.ndarray:
        c = self.cfg
        obs = []

        # Robot pose (7)
        obs += [
            raw["robot_x"]  / c.field_half_x,
            raw["robot_z"]  / c.field_half_z,
            raw["qx"], raw["qy"], raw["qz"], raw["qw"],
            math.sqrt(raw["robot_vx"]**2 + raw["robot_vz"]**2) / 5.0,
        ]

        # Motors (12 = 4 × 3)
        for i in range(4):
            obs += [
                raw["omegas"][i]       / c.free_speed,
                raw["steer_angles"][i] / math.pi,
                last_voltages[i],       # already in [-1,1]
            ]

        # Intake (2)
        held_frac = raw["intake_held"] / c.max_capacity
        obs += [held_frac, float(raw["intake_held"] > 0)]

        # Nearest game pieces (n × 3)
        for dx, dz, dist in raw["pieces"]:
            obs += [
                dx   / c.field_half_x,
                dz   / c.field_half_z,
                min(dist, c.max_piece_dist) / c.max_piece_dist,
            ]

        # Nearest active zone (4)
        obs += [
            raw["zone_dx"]   / c.field_half_x,
            raw["zone_dz"]   / c.field_half_z,
            min(raw["zone_dist"], c.max_piece_dist) / c.max_piece_dist,
            raw["zone_pts"]  / 3.0,
        ]

        # Match state (3)
        obs += [
            raw["match_time"] / 150.0,
            raw["score_mine"] / 20.0,
            raw["score_opp"]  / 20.0,
        ]

        return np.array(obs, dtype=np.float32)

    # ── Reward ────────────────────────────────────────────────────────────────

    def compute_reward(self, prev: dict, curr: dict, action: np.ndarray) -> float:
        """
        Reward function. All weights come from self.cfg.
        Edit this freely — it's the primary training knob.
        """
        c   = self.cfg
        r   = 0.0

        # ── Scoring (sparse) ─────────────────────────────────────────────
        score_delta = curr["score_mine"] - prev["score_mine"]
        if score_delta > 0:
            r += score_delta * c.w_score

        # ── Pickup ───────────────────────────────────────────────────────
        held_delta = curr["intake_held"] - prev["intake_held"]
        if held_delta > 0:
            r += held_delta * c.w_pickup

        # ── Approach shaping ─────────────────────────────────────────────
        # Only one of these fires per step depending on whether we're holding.
        if curr["intake_held"] == 0 and prev["pieces"] and curr["pieces"]:
            prev_dist = prev["pieces"][0][2]
            curr_dist = curr["pieces"][0][2]
            approach  = prev_dist - curr_dist          # positive = getting closer
            r += approach * c.w_approach_piece

        if curr["intake_held"] > 0:
            prev_zone = prev["zone_dist"]
            curr_zone = curr["zone_dist"]
            r += (prev_zone - curr_zone) * c.w_approach_zone

        # ── Motor efficiency penalty ──────────────────────────────────────
        # Penalizes spinning motors without producing robot movement.
        # Ramped in via self.efficiency_weight (0→1 over curriculum).
        fwd, strafe, rot = action[0], action[1], action[2]
        motor_effort = abs(fwd) + abs(strafe) + abs(rot)
        robot_speed  = math.sqrt(curr["robot_vx"]**2 + curr["robot_vz"]**2)
        # Expected speed at full effort ≈ 5 m/s; penalty = 0 when moving at that rate
        efficiency_shortfall = max(0.0, 1.0 - robot_speed / 5.0)
        r -= motor_effort * efficiency_shortfall * c.w_efficiency_penalty * self.efficiency_weight

        # ── Action smoothness ─────────────────────────────────────────────
        # Flat squared-norm penalty; discourages high-frequency jitter.
        r -= float(np.dot(action[:3], action[:3])) * c.w_action_penalty

        # ── Step penalty ─────────────────────────────────────────────────
        r -= c.w_step_penalty

        # ── Out of bounds ─────────────────────────────────────────────────
        if abs(curr["robot_x"]) > c.field_half_x or abs(curr["robot_z"]) > c.field_half_z:
            r -= c.w_oob_penalty

        return r

    # ── Gym interface ─────────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        # Zero all motors first
        for p in self._vpubs: p.set(0.0)
        for p in self._spubs: p.set(0.0)
        self._fire_pub.set(False)

        # Trigger sim reset — rising edge, sim will auto-start match + countdown
        self._reset_pub.set(True)
        time.sleep(0.1)
        self._reset_pub.set(False)

        # Wait for countdown (3s) + a little margin before collecting first obs
        time.sleep(3.5)

        self._shoot_tilt       = 0.3
        self._shoot_pan        = 0.0
        self._prev_score_mine  = 0
        self._prev_score_opp   = 0
        self._prev_intake_held = 0
        self._step_count       = 0
        self._last_voltages    = [0.0] * 4

        raw = self._read_raw()
        self._prev_raw = raw
        return self._normalize(raw, self._last_voltages), {}

    def step(self, action: np.ndarray):
        t0 = time.monotonic()
        c  = self.cfg

        # Unpack action
        fwd, strafe, rot = float(action[0]), float(action[1]), float(action[2])
        self._shoot_tilt = float(np.clip(
            self._shoot_tilt + action[3] * 0.05, -math.pi/2, math.pi/2))
        self._shoot_pan = float(np.clip(
            self._shoot_pan  + action[4] * 0.05, -math.pi,   math.pi))
        shoot_speed = float(np.interp(action[5], [-1.0, 1.0], [1.0, 30.0]))
        fire        = bool(action[6] > 0.0)

        # Compute per-module voltages / angles and publish
        modules = swerve(fwd, strafe, rot)
        self._last_voltages = [v for v, _ in modules]
        for i, (v, a) in enumerate(modules):
            self._vpubs[i].set(float(v))
            self._spubs[i].set(float(a))
        self._fire_pub.set(fire)
        self._speed_pub.set(shoot_speed)
        self._dir_pub.set(aim_dir(self._shoot_tilt, self._shoot_pan))

        # Sleep to hit tick rate
        elapsed = time.monotonic() - t0
        sleep_t = max(0.0, (1.0 / c.tick_hz) - elapsed)
        if sleep_t > 0:
            time.sleep(sleep_t)

        # Read new state
        curr_raw = self._read_raw()
        reward   = self.compute_reward(self._prev_raw, curr_raw, action)
        self._prev_raw = curr_raw
        self._step_count += 1

        obs     = self._normalize(curr_raw, self._last_voltages)
        phase   = curr_raw["phase"]
        done    = phase in ("ended",)
        if c.max_episode_steps is not None:
            done = done or (self._step_count >= c.max_episode_steps)
        truncated = False

        return obs, reward, done, truncated, {"phase": phase, "score": curr_raw["score_mine"]}

    def close(self):
        for p in self._vpubs: p.set(0.0)
        for p in self._spubs: p.set(0.0)
        self._fire_pub.set(False)
        self._inst.stopServer()