from dataclasses import dataclass, field


@dataclass
class Config:
    # ── NT connection ─────────────────────────────────────────────────────────
    nt_host:     str   = "127.0.0.1"
    nt_port:     int   = 6810
    sim_speedup: float = 16.0
    tick_hz:     float = 1000

    # ── Field dimensions ──────────────────────────────────────────────────────
    field_half_x:   float = 7.35
    field_half_z:   float = 4.05
    free_speed:     float = 594.0
    max_capacity:   int   = 40

    # ── Raycast observation layout ────────────────────────────────────────────
    # Matches raycast_ml.json exactly. Update these if you change the JSON.
    #
    #   rays[0:32]   — piece ring,      22.5° spacing, 360°, max_dist=10m
    #   rays[32:50]  — field low ring,  20°   spacing, 360°, max_dist=6m
    #   rays[50:58]  — field high ring, 45°   spacing, 360°, max_dist=7m, pitch=-40°
    #
    n_rays_piece:      int = 32
    n_rays_field_low:  int = 18
    n_rays_field_high: int = 8
    ray_topic:         str = "/sim/raycast/hits"

    # ── Scalar observation layout ─────────────────────────────────────────────
    # pose(9) + motors(12) + intake(2) = 23
    # No zone, no match state — this is phase 1 (piece chasing only).
    obs_scalar_dim: int = 23

    # ── Derived obs dim (assert in model) ────────────────────────────────────
    @property
    def n_rays(self) -> int:
        return self.n_rays_piece + self.n_rays_field_low + self.n_rays_field_high

    @property
    def obs_dim(self) -> int:
        return self.obs_scalar_dim + self.n_rays

    # ── Action space ──────────────────────────────────────────────────────────
    act_dim: int = 3   # fwd, strafe, rot

    # ── PPO hyperparameters ───────────────────────────────────────────────────
    lr:               float = 3e-4
    gamma:            float = 0.99
    gae_lambda:       float = 0.97
    clip_eps:         float = 0.2
    entropy_coef:     float = 0.001
    value_coef:       float = 1.0
    max_grad_norm:    float = 0.5
    n_epochs:         int   = 10
    steps_per_update: int   = 4096
    minibatch_size:   int   = 512

    # ── Reward weights ────────────────────────────────────────────────────────
    w_intake_rate:      float = 8.0    # per piece picked up
    w_idle_penalty:     float = 0.08
    w_spin_penalty:     float = 0.02
    spin_rot_threshold: float = 0.3
    spin_tran_threshold:float = 0.3
    w_oob_penalty:      float = 60.0
    w_wall_shaping:     float = 0.04

    # ── Checkpoint ────────────────────────────────────────────────────────────
    reward_version: int = 8   # bumped: new obs layout

    # ── Vectorised envs ───────────────────────────────────────────────────────
    num_envs: int = 16

    # ── Training ──────────────────────────────────────────────────────────────
    total_steps:       int = 300_000_000
    checkpoint_every:  int = 300_000
    checkpoint_dir:    str = "checkpoints"
    max_episode_steps: int = 8_000