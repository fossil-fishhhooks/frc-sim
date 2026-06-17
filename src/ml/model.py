"""
model.py — Actor-critic for PPO with raycast observations.

Two input streams fused into a shared trunk:

  1. Scalars (23)   — pose, motors, intake state.
                      Small projection MLP before fusion.

  2. Raycasts (58)  — 32 piece rays + 18 field-low rays + 8 field-high rays.
                      Each value is already normalised 0..1 by the sim.
                      Split into piece/field-low/field-high sub-vectors,
                      each gets its own small MLP, then concatenated.
                      Splitting lets the network learn different
                      representations for "where are pieces" vs
                      "where are walls" vs "where are elevated obstacles"
                      without mixing their semantics in a single dense layer.

Replaces the DeepSet + CNN architecture entirely — no longer needed
since raycasts give direct spatial geometry the network can read linearly.
"""

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal

from config import Config


class RayEncoder(nn.Module):
    """Encode one ring of raycast hits into a fixed-size feature vector."""

    def __init__(self, n_rays: int, hidden: int = 64, out_dim: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(n_rays, hidden), nn.LayerNorm(hidden), nn.ELU(),
            nn.Linear(hidden, out_dim),                       nn.ELU(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int, cfg: Config | None = None):
        super().__init__()
        self.cfg = cfg or Config()
        c = self.cfg

        assert obs_dim == c.obs_dim, (
            f"obs_dim mismatch: model expects {c.obs_dim} "
            f"(scalar={c.obs_scalar_dim} + rays={c.n_rays}), "
            f"but env reports obs_dim={obs_dim}. "
            f"Check n_rays_piece/field_low/field_high in config.py."
        )

        self.scalar_dim      = c.obs_scalar_dim
        self.n_rays_piece    = c.n_rays_piece
        self.n_rays_field_low  = c.n_rays_field_low
        self.n_rays_field_high = c.n_rays_field_high

        # ── Encoders ──────────────────────────────────────────────────────────
        self.scalar_enc = nn.Sequential(
            nn.Linear(self.scalar_dim, 64), nn.LayerNorm(64), nn.ELU(),
        )
        self.piece_enc     = RayEncoder(self.n_rays_piece,     hidden=64, out_dim=64)
        self.field_low_enc = RayEncoder(self.n_rays_field_low, hidden=32, out_dim=32)
        self.field_hi_enc  = RayEncoder(self.n_rays_field_high,hidden=16, out_dim=16)

        fused_dim = 64 + 64 + 32 + 16   # = 176

        # ── Shared trunk ──────────────────────────────────────────────────────
        self.trunk = nn.Sequential(
            nn.Linear(fused_dim, 256), nn.LayerNorm(256), nn.ELU(),
            nn.Linear(256, 128),                          nn.ELU(),
        )

        # ── Heads ─────────────────────────────────────────────────────────────
        self.policy_mean   = nn.Linear(128, act_dim)
        self.policy_logstd = nn.Parameter(torch.full((act_dim,), -0.5))
        self.value_head    = nn.Linear(128, 1)

        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
                nn.init.zeros_(m.bias)
        nn.init.orthogonal_(self.policy_mean.weight, gain=0.01)
        nn.init.zeros_(self.policy_mean.bias)
        nn.init.orthogonal_(self.value_head.weight, gain=1.0)

    def _split(self, obs: torch.Tensor):
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)
        i = 0
        scalars = obs[:, i:i + self.scalar_dim];        i += self.scalar_dim
        p_rays  = obs[:, i:i + self.n_rays_piece];      i += self.n_rays_piece
        fl_rays = obs[:, i:i + self.n_rays_field_low];  i += self.n_rays_field_low
        fh_rays = obs[:, i:i + self.n_rays_field_high]
        return scalars, p_rays, fl_rays, fh_rays

    def forward(self, obs: torch.Tensor) -> tuple:
        was_1d = obs.dim() == 1
        scalars, p_rays, fl_rays, fh_rays = self._split(obs)

        fused = torch.cat([
            self.scalar_enc(scalars),
            self.piece_enc(p_rays),
            self.field_low_enc(fl_rays),
            self.field_hi_enc(fh_rays),
        ], dim=-1)

        h    = self.trunk(fused)
        mean = torch.tanh(self.policy_mean(h))
        std  = self.policy_logstd.clamp(-2.0, 0.5).exp().expand_as(mean)
        dist = Normal(mean, std)
        val  = self.value_head(h)

        if was_1d:
            return dist, val.squeeze(0)
        return dist, val

    @torch.no_grad()
    def act(self, obs: torch.Tensor, deterministic: bool = False) -> tuple:
        was_1d = obs.dim() == 1
        dist, value = self(obs)
        action   = dist.mean if deterministic else dist.sample()
        action   = action.clamp(-1.0, 1.0)
        log_prob = dist.log_prob(action).sum(dim=-1)
        if was_1d:
            action   = action.squeeze(0)
            log_prob = log_prob.squeeze(0)
        return action, log_prob, value.squeeze(-1)

    def evaluate(self, obs: torch.Tensor, action: torch.Tensor) -> tuple:
        dist, value = self(obs)
        log_prob = dist.log_prob(action).sum(dim=-1)
        entropy  = dist.entropy().sum(dim=-1)
        return log_prob, value.squeeze(-1), entropy