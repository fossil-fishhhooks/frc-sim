"""
env.py — Gym environment wrapping frc_sim over NT4.

The sim is the physics engine; this file is pure wiring and reward.
All reward weights live in config.py.

Observation vector layout (obs_dim = 160):
  [0:9]       robot pose      — x, z, qx, qy, qz, qw, intake_ix, intake_iz, speed
  [9:21]      motors          — 4× (omega, steer_angle, last_voltage)
  [21:23]     intake          — held_frac, has_piece (binary)
  [23:83]     nearest 20 pieces — dx/norm, dz/norm, dist/norm (robot-relative)
  [83:87]     active zone     — dx, dz, dist, points_norm
  [87:90]     match state     — time_frac, score_mine_norm, score_opp_norm
  [90:135]    coarse grid     — 9×5 global piece-density map
  [135:160]   fine grid       — 5×5 local piece-density map (0.5m cells, robot-centred)

Action vector (3 continuous, tanh → [-1,1]):
  [0] fwd    [1] strafe    [2] rot
"""

import collections
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
_ZONES = [
    (0, 1,  3.62,  0.0,   0,  30),
    (1, 1, -3.62,  0.0,   0,  30),
    (1, 1, -3.62,  0.0,  30,  55),
    (0, 1,  3.62,  0.0,  55,  80),
    (1, 1, -3.62,  0.0,  80, 105),
    (0, 1,  3.62,  0.0,  80, 105),
    (0, 1,  3.62,  0.0, 105, 135),
    (1, 1, -3.62,  0.0, 105, 135),
]


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

    def __init__(self, cfg: Config | None = None):
        super().__init__()
        self.cfg = cfg or Config()
        c = self.cfg

        n         = c.n_gamepieces
        grid_size = c.coarse_cols * c.coarse_rows + c.fine_cols * c.fine_rows
        obs_dim   = 9 + 12 + 2 + n * 3 + 4 + 3 + grid_size

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(3,), dtype=np.float32
        )

        self._inst = ntcore.NetworkTableInstance.getDefault()
        self._setup_nt()

        self._shoot_tilt    = 0.3
        self._shoot_pan     = 0.0
        self._prev_raw: dict | None = None
        self._step_count    = 0
        self._intake_count  = 0
        self._last_voltages = [0.0] * 4

        # Per-episode reward component accumulators — printed at episode end
        self._dbg = self._fresh_dbg()

        # Pre-filled with zeros — no warmup bias on rate reward
        self._pickup_window: collections.deque = collections.deque(
            [0] * c.intake_rate_window, maxlen=c.intake_rate_window
        )

    def _fresh_dbg(self) -> dict:
        return {"rate": 0.0, "align": 0.0, "idle": 0.0,
                "spin": 0.0, "score": 0.0, "oob": 0.0, "n": 0}

    # ── NT setup ──────────────────────────────────────────────────────────────

    def _setup_nt(self):
        inst = self._inst
        inst.startServer()

        self._vpubs    = [inst.getFloatTopic(f"/sim/motors/{i}/voltage").publish()
                          for i in range(4)]
        self._spubs    = [inst.getFloatTopic(f"/sim/motors/{i}/steer_angle").publish()
                          for i in range(4)]
        self._fire_pub  = inst.getBooleanTopic("/sim/shooter/fire").publish()
        self._speed_pub = inst.getFloatTopic("/sim/shooter/speed").publish()
        self._dir_pub   = inst.getFloatArrayTopic("/sim/shooter/direction").publish()
        self._reset_pub = inst.getBooleanTopic("/sim/reset").publish()

        self._pose_x_sub  = inst.getFloatTopic("/sim/robot/x").subscribe(0.0)
        self._pose_z_sub  = inst.getFloatTopic("/sim/robot/z").subscribe(0.0)
        self._pose_qx_sub = inst.getFloatTopic("/sim/robot/qx").subscribe(0.0)
        self._pose_qy_sub = inst.getFloatTopic("/sim/robot/qy").subscribe(0.0)
        self._pose_qz_sub = inst.getFloatTopic("/sim/robot/qz").subscribe(0.0)
        self._pose_qw_sub = inst.getFloatTopic("/sim/robot/qw").subscribe(1.0)
        self._vx_sub      = inst.getFloatTopic("/sim/robot/vx").subscribe(0.0)
        self._vz_sub      = inst.getFloatTopic("/sim/robot/vz").subscribe(0.0)

        self._omega_subs  = [inst.getFloatTopic(f"/sim/motors/{i}/omega").subscribe(0.0)
                              for i in range(4)]
        self._steer_subs  = [inst.getFloatTopic(f"/sim/motors/{i}/steer_angle").subscribe(0.0)
                              for i in range(4)]

        self._intake_held_sub = inst.getIntegerTopic("/sim/intake/held").subscribe(0)
        self._gamepieces_sub  = inst.getFloatArrayTopic("/sim/gamepieces").subscribe([])
        self._score0_sub      = inst.getIntegerTopic("/sim/score/team0").subscribe(0)
        self._score1_sub      = inst.getIntegerTopic("/sim/score/team1").subscribe(0)
        self._match_time_sub  = inst.getFloatTopic("/sim/match/time").subscribe(0.0)
        self._phase_sub       = inst.getStringTopic("/sim/match/phase").subscribe("waiting")

        self._my_team = 0

    # ── Observation ───────────────────────────────────────────────────────────

    def _read_raw(self) -> dict:
        c = self.cfg

        robot_x  = self._pose_x_sub.get()
        robot_z  = self._pose_z_sub.get()
        robot_vx = self._vx_sub.get()
        robot_vz = self._vz_sub.get()

        qx  = self._pose_qx_sub.get()
        qy  = self._pose_qy_sub.get()
        qz  = self._pose_qz_sub.get()
        qw  = self._pose_qw_sub.get()
        
        self._oob_sub = inst.getBooleanTopic("/sim/robot/oob").subscribe(False)
        
        yaw = math.atan2(2.0*(qw*qy + qx*qz), 1.0 - 2.0*(qy*qy + qz*qz))
        # intake faces backward (-forward)
        ix = -math.sin(yaw)
        iz = -math.cos(yaw)

        gp_flat = self._gamepieces_sub.get()

        # World positions collected before sorting — used for grids
        pieces_world = []
        pieces       = []
        for i in range(0, len(gp_flat) - 2, 3):
            px, py, pz = gp_flat[i], gp_flat[i+1], gp_flat[i+2]
            pieces_world.append((px, pz))
            dx        = px - robot_x
            dz        = pz - robot_z
            dist      = math.sqrt(dx*dx + dz*dz)
            alignment = (ix*dx + iz*dz) / dist if dist > 1e-3 else 0.0
            pieces.append((dx, dz, dist, alignment))

        # Sort by intake score: prefer close + already facing intake
        ALIGN_WEIGHT = 3.0
        pieces.sort(key=lambda p: p[2] - ALIGN_WEIGHT * max(0.0, p[3]) * p[2])

        pieces_obs = pieces[:c.n_gamepieces]
        while len(pieces_obs) < c.n_gamepieces:
            pieces_obs.append((0.0, 0.0, c.max_piece_dist, 0.0))

        scores     = [self._score0_sub.get(), self._score1_sub.get()]
        match_time = self._match_time_sub.get()

        raw = {
            "robot_x":      robot_x,
            "robot_z":      robot_z,
            "robot_vx":     robot_vx,
            "robot_vz":     robot_vz,
            "oob": self._oob_sub.get(),
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            "yaw":          yaw,
            "intake_ix":    ix,
            "intake_iz":    iz,
            "omegas":       [s.get() for s in self._omega_subs],
            "steer_angles": [s.get() for s in self._steer_subs],
            "intake_held":  int(self._intake_held_sub.get()),
            "pieces":       pieces_obs,    # nearest N, robot-relative, sorted
            "pieces_world": pieces_world,  # all pieces, world coords, unsorted
            "score_mine":   scores[self._my_team],
            "score_opp":    scores[1 - self._my_team],
            "match_time":   match_time,
            "phase":        self._phase_sub.get(),
            "zone_dx":   0.0,
            "zone_dz":   0.0,
            "zone_dist": c.field_half_x * 2,
            "zone_pts":  1,
        }
        self._update_zone(raw)
        return raw

    def _update_zone(self, raw: dict):
        t       = raw["match_time"]
        robot_x = raw["robot_x"]
        robot_z = raw["robot_z"]
        best_dist = self.cfg.field_half_x * 4
        best_dx = best_dz = 0.0
        best_pts = 1

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

    def _build_grids(self, raw: dict) -> np.ndarray:
        """
        Two-scale piece-density grids built from world-coordinate piece positions.

        Coarse (9×5): global — ~1.78m×1.8m cells covering the whole field.
                      Agent uses this to navigate toward dense regions.

        Fine (5×5):   local — 0.5m cells in a 2.5m×2.5m window centred on robot.
                      Sub-intake-width resolution so the agent can precisely
                      orient toward pieces in reach. Robot is always at centre.
        """
        c  = self.cfg
        rx = raw["robot_x"]
        rz = raw["robot_z"]

        # ── Coarse ───────────────────────────────────────────────────────────
        coarse = np.zeros((c.coarse_rows, c.coarse_cols), dtype=np.float32)
        cw = (2.0 * c.coarse_half_x) / c.coarse_cols
        ch = (2.0 * c.coarse_half_z) / c.coarse_rows
        for wx, wz in raw["pieces_world"]:
            col = int((wx + c.coarse_half_x) / cw)
            row = int((wz + c.coarse_half_z) / ch)
            col = max(0, min(c.coarse_cols - 1, col))
            row = max(0, min(c.coarse_rows - 1, row))
            coarse[row, col] += 1.0
        np.clip(coarse / c.coarse_norm, 0.0, 1.0, out=coarse)

        # ── Fine ─────────────────────────────────────────────────────────────
        fine     = np.zeros((c.fine_rows, c.fine_cols), dtype=np.float32)
        half_w   = c.fine_cell_size * c.fine_cols / 2.0
        half_h   = c.fine_cell_size * c.fine_rows / 2.0
        origin_x = rx - half_w
        origin_z = rz - half_h
        for wx, wz in raw["pieces_world"]:
            col = int((wx - origin_x) / c.fine_cell_size)
            row = int((wz - origin_z) / c.fine_cell_size)
            if 0 <= col < c.fine_cols and 0 <= row < c.fine_rows:
                fine[row, col] += 1.0
        np.clip(fine / c.fine_norm, 0.0, 1.0, out=fine)

        return np.concatenate([coarse.flatten(), fine.flatten()])

    def _normalize(self, raw: dict, last_voltages: list[float]) -> np.ndarray:
        c   = self.cfg
        obs = []

        # Robot pose (9)
        # Includes intake-facing unit vector (intake_ix, intake_iz) directly —
        # previously the network only saw raw quaternion components and had
        # to derive "which way is my intake pointing" via trig itself, despite
        # the alignment reward depending directly on this relationship with
        # piece dx/dz (also in obs). Handing it over precomputed should make
        # the approach-toward-piece relationship much more learnable.
        obs += [
            raw["robot_x"] / c.field_half_x,
            raw["robot_z"] / c.field_half_z,
            raw["qx"], raw["qy"], raw["qz"], raw["qw"],
            raw["intake_ix"], raw["intake_iz"],
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
        obs += [raw["intake_held"] / c.max_capacity, float(raw["intake_held"] > 0)]

        # Nearest N pieces (n×3)
        for dx, dz, dist, _ in raw["pieces"]:
            obs += [
                dx  / c.field_half_x,
                dz  / c.field_half_z,
                min(dist, c.max_piece_dist) / c.max_piece_dist,
            ]

        # Active zone (4)
        obs += [
            raw["zone_dx"]  / c.field_half_x,
            raw["zone_dz"]  / c.field_half_z,
            min(raw["zone_dist"], c.max_piece_dist) / c.max_piece_dist,
            raw["zone_pts"] / 3.0,
        ]

        # Match state (3)
        obs += [
            raw["match_time"] / 150.0,
            raw["score_mine"] / 20.0,
            raw["score_opp"]  / 20.0,
        ]

        grids = self._build_grids(raw)
        return np.concatenate([np.array(obs, dtype=np.float32), grids])

    # ── Reward ────────────────────────────────────────────────────────────────

    def compute_reward(self, prev: dict, curr: dict,
                       held_delta: int, action: np.ndarray) -> float:
        c = self.cfg
        r = 0.0

        # ── 1. INTAKE RATE ────────────────────────────────────────────────────
        # Rolling window fraction of steps that had a pickup ∈ [0, 1].
        # Pre-filled with zeros so there's no warmup inflation.
        # Rewards sustained throughput — wandering scores low regardless of
        # total count; systematic sweeping scores high.
        self._pickup_window.append(1 if held_delta > 0 else 0)
        rate = sum(self._pickup_window) / len(self._pickup_window)
        r_rate = rate * c.w_intake_rate
        r += r_rate

        # ── 2. INTAKE ALIGNMENT ───────────────────────────────────────────────
        # Reward facing intake toward the nearest piece WHILE moving toward it.
        # Previously fired on orientation alone, which let the agent sit
        # still facing a piece as a stable (if useless) source of reward.
        # Multiplying by speed means standing still gives ~0 — the agent has
        # to actually be moving to collect this.
        r_align = 0.0
        if curr["intake_held"] == 0 and curr["pieces"]:
            _, _, _, align0 = curr["pieces"][0]
            speed_now = math.sqrt(curr["robot_vx"]**2 + curr["robot_vz"]**2)
            r_align = max(0.0, align0) * min(1.0, speed_now) * c.w_intake_alignment
        r += r_align

        # ── 3. IDLE PENALTY ───────────────────────────────────────────────────
        # Penalise low translational speed. Kept small so a brief slowdown to
        # line up an intake isn't punished meaningfully.
        speed  = math.sqrt(curr["robot_vx"]**2 + curr["robot_vz"]**2)
        r_idle = -c.w_idle_penalty * max(0.0, 1.0 - speed / 1.0)
        r += r_idle

        # ── 4. SPIN PENALTY ───────────────────────────────────────────────────
        # Penalise high rotation with low translation — spinning in place is
        # always wrong. Rotating while driving is fine and necessary for swerve.
        rotation    = abs(float(action[2]))
        translation = math.sqrt(float(action[0])**2 + float(action[1])**2)
        r_spin = 0.0
        if rotation > c.spin_rot_threshold and translation < c.spin_tran_threshold:
            r_spin = -c.w_spin_penalty * (rotation - c.spin_rot_threshold)
        r += r_spin

        # ── 5. SCORING ────────────────────────────────────────────────────────
        r_score = 0.0
        score_delta = curr["score_mine"] - prev["score_mine"]
        if score_delta > 0:
            r_score = score_delta * c.w_score
        r += r_score

        # ── 6. OOB ────────────────────────────────────────────────────────────
        # One-time terminal penalty — step() ends the episode the moment the
        # boundary is crossed, so this fires only on the final step.
        # Boundary matches step() termination check: field_half - 0.3.
        r_oob = -c.w_oob_penalty if (curr["oob"] or prev["oob"]) else 0.0

        # Accumulate per-component totals for end-of-episode debug print
        d = self._dbg
        d["rate"]  += r_rate
        d["align"] += r_align
        d["idle"]  += r_idle
        d["spin"]  += r_spin
        d["score"] += r_score
        d["oob"]   += r_oob
        d["n"]     += 1

        return r

    # ── Gym interface ─────────────────────────────────────────────────────────

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        for p in self._vpubs: p.set(0.0)
        for p in self._spubs: p.set(0.0)
        self._fire_pub.set(False)

        c = self.cfg

        # Pulse /sim/reset and poll until the robot's pose is actually back
        # in-bounds. A single pulse can race with the sim — phase flips back
        # to "running" before the robot's transform has actually been
        # teleported, so _read_raw() was returning the *previous* (OOB)
        # pose. That pose then immediately re-triggers OOB termination on
        # step 1, with a full OOB penalty, every single reset — a death
        # spiral that floods the buffer with unearned penalties and gives
        # the policy nothing to learn from. Re-pulse a few times, with the
        # original phase-settle wait after each pulse, until the pose
        # clears the boundary or we give up.
        raw = None
        for attempt in range(5):
            self._reset_pub.set(True)
            time.sleep(0.5)
            self._reset_pub.set(False)

            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                time.sleep(0.05)
                if self._phase_sub.get() not in ("", "ended", "waiting"):
                    break

            time.sleep(0.05)
            raw = self._read_raw()

            if (abs(raw["robot_x"]) <= c.field_half_x - 0.3 and
                    abs(raw["robot_z"]) <= c.field_half_z - 0.3):
                break

            print(
                f"  [WARN] robot still OOB after reset attempt {attempt+1} "
                f"(x={raw['robot_x']:.2f}, z={raw['robot_z']:.2f}) — retrying"
            )
        else:
            print(
                f"  [WARN] giving up after 5 reset attempts, robot still OOB "
                f"(x={raw['robot_x']:.2f}, z={raw['robot_z']:.2f})"
            )

        self._shoot_tilt    = 0.3
        self._shoot_pan     = 0.0
        self._step_count    = 0
        self._intake_count  = 0
        self._last_voltages = [0.0] * 4
        self._dbg           = self._fresh_dbg()

        self._pickup_window = collections.deque(
            [0] * c.intake_rate_window, maxlen=c.intake_rate_window
        )

        self._prev_raw = raw

        return self._normalize(raw, self._last_voltages), {}

    def step(self, action: np.ndarray):
        t0 = time.monotonic()
        c  = self.cfg

        fwd, strafe, rot = float(action[0]), float(action[1]), float(action[2]) * 0.5

        modules = swerve(fwd, strafe, rot)
        self._last_voltages = [v for v, _ in modules]
        for i, (v, a) in enumerate(modules):
            self._vpubs[i].set(float(v))
            self._spubs[i].set(float(a))
        self._fire_pub.set(False)
        self._speed_pub.set(15.0)
        self._dir_pub.set(aim_dir(self._shoot_tilt, self._shoot_pan))

        elapsed = time.monotonic() - t0
        sleep_t = max(0.0, (1.0 / (c.tick_hz / c.sim_speedup)) - elapsed)
        if sleep_t > 0:
            time.sleep(sleep_t)

        curr_raw   = self._read_raw()
        held_delta = curr_raw["intake_held"] - self._prev_raw["intake_held"]

        if held_delta > 0:
            self._intake_count += held_delta

        reward = self.compute_reward(self._prev_raw, curr_raw, held_delta, action)

        self._prev_raw    = curr_raw
        self._step_count += 1

        obs   = self._normalize(curr_raw, self._last_voltages)
        phase = curr_raw["phase"]

        # OOB is terminal — prevents the robot sitting frozen against the wall
        # accumulating penalties and starving intake_count for the episode.
        oob = curr_raw["oob"] or self._prev_raw["oob"]

        terminated = phase == "ended" or oob
        truncated  = self._step_count >= c.max_episode_steps

        if terminated or truncated:
            d = self._dbg
            print(f"    [dbg] steps={d['n']}  "
                  f"rate={d['rate']:+.1f}  align={d['align']:+.1f}  "
                  f"idle={d['idle']:+.1f}  spin={d['spin']:+.1f}  "
                  f"score={d['score']:+.1f}  oob={d['oob']:+.1f}")

        return obs, reward, terminated, truncated, {
            "phase":        phase,
            "score":        curr_raw["score_mine"],
            "intake_count": self._intake_count,
            "match_time":   curr_raw["match_time"],
            "oob":          oob,
        }

    def close(self):
        for p in self._vpubs: p.set(0.0)
        for p in self._spubs: p.set(0.0)
        self._fire_pub.set(False)
        self._inst.stopServer()