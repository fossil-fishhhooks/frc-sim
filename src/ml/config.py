from dataclasses import dataclass, field


@dataclass
class Config:
    # ── NT connection ─────────────────────────────────────────────────────────
    nt_host: str   = "127.0.0.1"
    nt_port: int   = 5810
    tick_hz: float = 50.0          # must match sim tick rate

    # ── Observation ───────────────────────────────────────────────────────────
    n_gamepieces:  int   = 5       # nearest N game pieces included in obs
    field_half_x:  float = 8.2     # metres — field bounds for normalization
    field_half_z:  float = 4.1
    free_speed:    float = 594.0   # NEO free speed rad/s
    max_capacity:  int   = 40      # intake capacity (from scene JSON)
    max_piece_dist: float = 12.0   # clip distance for normalization

    # ── PPO hyperparameters ───────────────────────────────────────────────────
    lr:               float = 3e-4
    gamma:            float = 0.99
    gae_lambda:       float = 0.95
    clip_eps:         float = 0.2
    entropy_coef:     float = 0.02   # start higher, decay during training
    value_coef:       float = 0.5
    max_grad_norm:    float = 0.5
    n_epochs:         int   = 10
    steps_per_update: int   = 2048
    minibatch_size:   int   = 256

    # ── Reward weights ────────────────────────────────────────────────────────
    # These are the knobs — tune these without touching anything else.
    w_score:              float = 10.0   # per point scored
    w_pickup:             float = 2.0    # per piece picked up
    w_approach_piece:     float = 0.5    # per metre closed on nearest piece (when empty)
    w_approach_zone:      float = 0.5    # per metre closed on nearest zone (when holding)
    w_efficiency_penalty: float = 0.3    # motor effort with no resulting velocity
    w_action_penalty:     float = 0.02   # flat ||action||^2 — discourages jitter
    w_step_penalty:       float = 0.01   # per step — encourages faster solutions
    w_oob_penalty:        float = 5.0    # out of bounds

    # ── Curriculum ────────────────────────────────────────────────────────────
    # efficiency_penalty ramped from 0 → w_efficiency_penalty over this many steps.
    # Prevents the agent freezing early when it hasn't learned to move yet.
    efficiency_ramp_steps: int = 200_000

    # ── Training ──────────────────────────────────────────────────────────────
    total_steps:       int   = 10_000_000
    checkpoint_every:  int   = 100_000
    checkpoint_dir:    str   = "checkpoints"
    max_episode_steps: int   = 7500      # 150s × 50Hz; None = full match