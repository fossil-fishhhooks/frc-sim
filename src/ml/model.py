"""
model.py — Actor-critic network for PPO, with structure-aware encoders.

The flat 160-dim observation is internally split into three parts that get
different treatment, then fused into a shared trunk:

  1. Scalars (30)        — pose, motors, intake state, zone, match state.
                            Passed straight through.

  2. Pieces (20 x 3)      — nearest-N gamepieces, robot-relative (dx, dz, dist).
                            Encoded with a DeepSet (per-piece MLP + max-pool).
                            This fixes a real structural problem: the pieces
                            list is sorted by a heuristic (distance vs.
                            alignment) that reorders slots frame-to-frame even
                            when the world barely changes. A plain MLP over a
                            flattened, order-sensitive list has to treat
                            "piece in slot 7" as a different feature each
                            frame even when it's physically the same piece.
                            A DeepSet pools over pieces with a permutation-
                            invariant op (max), so the encoding only depends
                            on the *set* of nearby pieces, not their slot
                            order.

  3. Grids (45 + 25)      — coarse 5x9 and fine 5x5 piece-density maps.
                            Reshaped to 2D and run through a small CNN. These
                            maps have real spatial locality (adjacent cells =
                            adjacent field regions) that a flattened dense
                            layer has to relearn from scratch; conv layers
                            exploit that structure directly with far fewer
                            parameters.

Design choices retained from the previous version:
  - LayerNorm instead of BatchNorm: batch size varies during minibatch updates.
  - ELU activations: smooth gradients, no dying-ReLU problem.
  - Orthogonal init, small gain on policy head: keeps early actions near zero.
  - Learned log_std initialised to -1.0 (std~0.37): std=1 made early actions
    on this [-1,1] action space near-uniform-random, reliably driving the
    robot OOB from spawn before any intake-related reward could accrue.
  - tanh squashing on policy mean.
"""

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal

from config import Config


class PieceEncoder(nn.Module):
    """DeepSet encoder over the nearest-N gamepieces.

    Each piece (dx_norm, dz_norm, dist_norm) is independently embedded by a
    shared MLP, then pooled across pieces with max (permutation invariant).
    Max-pool is preferred over mean: "is there a piece very close in this
    direction" should be dominated by the single nearest relevant piece, not
    diluted by the other ~19 mostly-irrelevant ones.
    """
    def __init__(self, piece_dim: int, embed_dim: int = 32, out_dim: int = 64):
        super().__init__()
        self.per_piece = nn.Sequential(
            nn.Linear(piece_dim, embed_dim), nn.LayerNorm(embed_dim), nn.ELU(),
            nn.Linear(embed_dim, embed_dim), nn.ELU(),
        )
        self.out = nn.Sequential(
            nn.Linear(embed_dim, out_dim), nn.ELU(),
        )

    def forward(self, pieces: torch.Tensor) -> torch.Tensor:
        # pieces: (batch, n_pieces, piece_dim)
        embedded = self.per_piece(pieces)   # (batch, n_pieces, embed_dim)
        pooled, _ = embedded.max(dim=1)     # (batch, embed_dim)
        return self.out(pooled)             # (batch, out_dim)


class GridEncoder(nn.Module):
    """Small CNN over the coarse + fine piece-density grids.

    Each grid runs through its own small conv stack (different spatial
    scales/semantics — global navigation vs. local precision — so separate
    weights make sense), globally pooled, and concatenated.
    """
    def __init__(self, coarse_shape: tuple, fine_shape: tuple, out_dim: int = 64):
        super().__init__()
        self.coarse_shape = coarse_shape
        self.fine_shape   = fine_shape

        def make_stack():
            return nn.Sequential(
                nn.Conv2d(1, 8, kernel_size=3, padding=1), nn.ELU(),
                nn.Conv2d(8, 16, kernel_size=3, padding=1), nn.ELU(),
                nn.AdaptiveAvgPool2d(1),
            )

        self.coarse_cnn = make_stack()
        self.fine_cnn   = make_stack()
        self.out = nn.Sequential(
            nn.Linear(32, out_dim), nn.ELU(),
        )

    def forward(self, coarse: torch.Tensor, fine: torch.Tensor) -> torch.Tensor:
        c = coarse.unsqueeze(1)
        f = fine.unsqueeze(1)
        c_feat = self.coarse_cnn(c).flatten(1)
        f_feat = self.fine_cnn(f).flatten(1)
        return self.out(torch.cat([c_feat, f_feat], dim=-1))


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int, cfg: Config | None = None):
        super().__init__()
        self.cfg = cfg or Config()
        c = self.cfg

        self.scalar_dim   = c.obs_scalar_dim
        self.n_pieces     = c.n_gamepieces
        self.piece_dim    = c.obs_piece_dim
        self.coarse_shape = tuple(c.obs_coarse_shape)
        self.fine_shape   = tuple(c.obs_fine_shape)

        pieces_flat = self.n_pieces * self.piece_dim
        coarse_flat = self.coarse_shape[0] * self.coarse_shape[1]
        fine_flat   = self.fine_shape[0] * self.fine_shape[1]
        expected    = self.scalar_dim + pieces_flat + coarse_flat + fine_flat
        assert expected == obs_dim, (
            f"obs_dim mismatch: config slices sum to {expected}, "
            f"but env reports obs_dim={obs_dim}. Update obs_* fields in "
            f"config.py to match env.py's _normalize() layout."
        )

        # ── Encoders ──────────────────────────────────────────────────────────
        self.piece_encoder = PieceEncoder(self.piece_dim, embed_dim=32, out_dim=64)
        self.grid_encoder  = GridEncoder(self.coarse_shape, self.fine_shape, out_dim=64)

        # Scalars get their own small projection before fusion, so the trunk
        # sees three roughly-equal-sized streams rather than 30 raw scalars
        # competing for relevance against two 64-dim learned encodings.
        self.scalar_proj = nn.Sequential(
            nn.Linear(self.scalar_dim, 64), nn.LayerNorm(64), nn.ELU(),
        )

        fused_dim = 64 + 64 + 64  # scalars + pieces + grids

        # ── Shared trunk ──────────────────────────────────────────────────────
        self.trunk = nn.Sequential(
            nn.Linear(fused_dim, 256), nn.LayerNorm(256), nn.ELU(),
            nn.Linear(256, 128),                          nn.ELU(),
        )

        # ── Policy head ───────────────────────────────────────────────────────
        self.policy_mean   = nn.Linear(128, act_dim)
        self.policy_logstd = nn.Parameter(torch.full((act_dim,), -1.0))

        # ── Value head ────────────────────────────────────────────────────────
        self.value_head = nn.Linear(128, 1)

        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
                nn.init.zeros_(m.bias)
            elif isinstance(m, nn.Conv2d):
                nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
                nn.init.zeros_(m.bias)
        nn.init.orthogonal_(self.policy_mean.weight, gain=0.01)
        nn.init.zeros_(self.policy_mean.bias)
        nn.init.orthogonal_(self.value_head.weight, gain=1.0)

    # ── Splitting ─────────────────────────────────────────────────────────────

    def _split_obs(self, obs: torch.Tensor):
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)

        i = 0
        scalars = obs[:, i:i + self.scalar_dim]
        i += self.scalar_dim

        pieces_flat = obs[:, i:i + self.n_pieces * self.piece_dim]
        i += self.n_pieces * self.piece_dim
        pieces = pieces_flat.reshape(-1, self.n_pieces, self.piece_dim)

        cr, cc = self.coarse_shape
        coarse_flat = obs[:, i:i + cr * cc]
        i += cr * cc
        coarse = coarse_flat.reshape(-1, cr, cc)

        fr, fc = self.fine_shape
        fine_flat = obs[:, i:i + fr * fc]
        i += fr * fc
        fine = fine_flat.reshape(-1, fr, fc)

        return scalars, pieces, coarse, fine

    # ── Forward ───────────────────────────────────────────────────────────────

    def forward(self, obs: torch.Tensor) -> tuple:
        was_1d = obs.dim() == 1
        scalars, pieces, coarse, fine = self._split_obs(obs)

        s_feat = self.scalar_proj(scalars)
        p_feat = self.piece_encoder(pieces)
        g_feat = self.grid_encoder(coarse, fine)

        fused = torch.cat([s_feat, p_feat, g_feat], dim=-1)
        h     = self.trunk(fused)

        mean  = torch.tanh(self.policy_mean(h))
        std   = self.policy_logstd.exp().expand_as(mean)
        dist  = Normal(mean, std)
        value = self.value_head(h)

        if was_1d:
            return dist, value.squeeze(0)
        return dist, value

    # ── Convenience methods ───────────────────────────────────────────────────

    @torch.no_grad()
    def act(self, obs: torch.Tensor, deterministic: bool = False) -> tuple:
        was_1d = obs.dim() == 1
        dist, value = self(obs)
        action      = dist.mean if deterministic else dist.sample()
        action      = action.clamp(-1.0, 1.0)
        log_prob    = dist.log_prob(action).sum(dim=-1)
        if was_1d:
            # _split_obs always adds a batch dim, so dist/action come back as
            # (1, act_dim) even for a single observation — squeeze it back to
            # (act_dim,) so train.py's float(action[i]) indexing works.
            action   = action.squeeze(0)
            log_prob = log_prob.squeeze(0)
        return action, log_prob, value.squeeze(-1)

    def evaluate(self, obs: torch.Tensor, action: torch.Tensor) -> tuple:
        dist, value = self(obs)
        log_prob    = dist.log_prob(action).sum(dim=-1)
        entropy     = dist.entropy().sum(dim=-1)
        return log_prob, value.squeeze(-1), entropy