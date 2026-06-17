"""
env_phase1.py — Phase 1: pure piece-chasing, no OOB termination.

Identical obs/action space to env.py — checkpoints transfer directly.
Strips: OOB termination, wall shaping.
Keeps:  intake reward, idle penalty, spin penalty.
"""

import numpy as np
from env    import FRCSimEnv
from config import Config


class Phase1Env(FRCSimEnv):

    def compute_reward(self, prev: dict, curr: dict,
                       held_delta: int, action: np.ndarray) -> float:
        import math
        c = self.cfg
        r = 0.0

        # 1. Intake
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

        d = self._dbg
        d["rate"] += r_rate
        d["idle"] += r_idle
        d["spin"] += r_spin
        d["n"]    += 1

        return r

    def _reset_pose_ok(self, raw: dict) -> bool:
        return True   # no walls in phase 1

    def step(self, action: np.ndarray):
        obs, reward, terminated, truncated, info = super().step(action)
        terminated      = False   # never OOB-terminate in phase 1
        info["oob"]     = False
        return obs, reward, terminated, truncated, info