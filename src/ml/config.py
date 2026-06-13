from dataclasses import dataclass


@dataclass
class Config:
    # ── NT connection ─────────────────────────────────────────────────────────
    nt_host:     str   = "127.0.0.1"
    nt_port:     int   = 5810
    sim_speedup: float = 8.0
    tick_hz:     float = 200.0

    # ── Observation ───────────────────────────────────────────────────────────
    n_gamepieces:   int   = 20
    field_half_x:   float = 7.35
    field_half_z:   float = 4.05
    free_speed:     float = 594.0
    max_capacity:   int   = 40
    max_piece_dist: float = 12.0

    # ── Field grid — two-scale ────────────────────────────────────────────────
    # Coarse: global navigation — which region has pieces.
    # Fine:   local precision  — sub-intake-width (0.5m < 0.6m intake) around robot.
    # Total grid obs = 9*5 + 5*5 = 70 floats.
    coarse_cols:    int   = 9
    coarse_rows:    int   = 5
    coarse_half_x:  float = 8.0
    coarse_half_z:  float = 4.5
    coarse_norm:    float = 15.0   # piece count that saturates a coarse cell

    fine_cols:      int   = 5
    fine_rows:      int   = 5
    fine_cell_size: float = 0.5    # metres — less than intake width of 0.6m
    fine_norm:      float = 3.0    # piece count that saturates a fine cell

    # ── Observation layout (must match env.py's _normalize concat order) ──────
    # scalars = pose(9) + motors(12) + intake(2) + zone(4) + match(3) = 30
    obs_scalar_dim:   int   = 30
    obs_piece_dim:    int   = 3          # dx_norm, dz_norm, dist_norm per piece
    obs_coarse_shape: tuple = (5, 9)     # (rows, cols) == (coarse_rows, coarse_cols)
    obs_fine_shape:   tuple = (5, 5)     # (rows, cols) == (fine_rows, fine_cols)

    # ── Action space ──────────────────────────────────────────────────────────
    act_dim: int = 3               # fwd, strafe, rot only

    # ── PPO hyperparameters ───────────────────────────────────────────────────
    lr:               float = 3e-4
    gamma:            float = 0.98
    gae_lambda:       float = 0.95
    clip_eps:         float = 0.2
    entropy_coef:     float = 0.003
    value_coef:       float = 1.0
    max_grad_norm:    float = 0.5
    n_epochs:         int   = 10
    steps_per_update: int   = 32768
    minibatch_size:   int   = 512

    # ── Reward weights ────────────────────────────────────────────────────────
    #
    # Reward accounting at good-agent pace (~150 intakes / 8000 steps):
    #   One pickup every ~53 steps → rate ≈ 0.019 in 150-step window
    #
    #   w_intake_rate:      0.019 * 8.0  = +0.15/step  → +1200 total  (main signal)
    #   w_intake_alignment: align≈0.7   * 0.04 = +0.028/step → +225 total  (shaping)
    #   w_idle_penalty:     10% slow    * 0.08 = -0.008/step → -64  total  (movement)
    #   w_spin_penalty:     5% spinning * 0.05 = -0.002/step → -14  total  (direction)
    #   w_oob_penalty:      0 hits      * 20.0 =  0/step               (categorical)
    #
    # Bad agent (~50 intakes, wandering, 20% spinning):
    #   rate ≈ 0.007 → +0.056/step → +450 total
    #   alignment ≈ 30% → +90 total
    #   idle 20% → -128 total
    #   spin 20% → -56 total
    #   occasional OOB hits
    #
    # Gap: ~750+ in favor of systematic directed sweeping.
    # No single signal dominates pathologically.

    w_score: float = 200.0   # per match point (future curriculum)

    # Primary signal: sustained pickup throughput over rolling window.
    # Rewards rate not count — wandering that stumbles on pieces scores low.
    w_intake_rate:      float = 48.0
    intake_rate_window: int   = 150   # steps (~3s at 50 real-Hz)

    # Shaping: reward facing intake toward nearest piece WHILE closing on it.
    # Now multiplied by speed (see env.py) so standing still gives ~0 —
    # weight raised to compensate and keep this a meaningful dense signal
    # that pulls the agent toward pieces rather than just toward facing them.
    w_intake_alignment: float = 0.35

    # Movement: penalize being stationary. Full penalty below 1 m/s.
    # Kept small so it doesn't overpower rate signal.
    w_idle_penalty: float = 0.08

    # Direction: penalize high rotation with low translation.
    # Spinning in place is always wrong. Rotating while moving is fine.
    # Threshold 0.3 on both axes avoids penalizing gentle course corrections.
    w_spin_penalty:      float = 0.02
    spin_rot_threshold:  float = 0.3   # above this rotation triggers check
    spin_tran_threshold: float = 0.3   # below this translation confirms spin

    # Categorical: OOB is immediately and severely penalized.
    # Previously 5.0 → typical hit was only -5..-9 (oob_x/z are tiny since
    # the episode terminates the instant the threshold is crossed), which
    # was smaller than the alignment reward accumulated while *driving
    # toward* an edge piece (often +10..+75/episode). Net effect: chasing
    # pieces near the wall and going OOB was reward-neutral or positive,
    # so the agent had no incentive to avoid it (oob_rate observed ~0.90).
    #
    # Raised 10x so a bare-minimum OOB hit (~-50..-90) outweighs essentially
    # any alignment/idle/spin total a short episode can accrue, making OOB
    # a clearly dominant negative outcome regardless of how it happened.
    w_oob_penalty: float = 50.0

    # ── Checkpoint / resumability ─────────────────────────────────────────────
    # Bump when reward structure changes. train.py skips restoring the reward
    # normalizer if versions don't match, preventing distribution mismatch.
    reward_version: int = 7

    # ── Training ──────────────────────────────────────────────────────────────
    total_steps:       int = 10_000_000
    checkpoint_every:  int = 40_000
    checkpoint_dir:    str = "checkpoints"
    max_episode_steps: int = 8_000