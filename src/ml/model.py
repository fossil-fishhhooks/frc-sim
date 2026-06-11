"""
model.py — Actor-critic network for PPO.

Shared trunk → policy head (Gaussian) + value head.

Design choices:
  - LayerNorm instead of BatchNorm: batch size varies during minibatch updates
  - ELU activations: smooth gradients, no dying-ReLU problem
  - Orthogonal init with small gain on policy head: keeps early actions near zero
  - Learned log_std as a parameter (not input-dependent): simpler, works well for
    continuous control; switch to state-dependent std if policy collapses
  - tanh squashing on mean: actions already bounded to [-1,1] by action_space,
    but squashing prevents the policy from saturating against hard boundaries
"""

import numpy as np
import torch
import torch.nn as nn
from torch.distributions import Normal


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()

        # ── Shared trunk ──────────────────────────────────────────────────────
        self.trunk = nn.Sequential(
            nn.Linear(obs_dim, 256), nn.LayerNorm(256), nn.ELU(),
            nn.Linear(256, 256),     nn.LayerNorm(256), nn.ELU(),
            nn.Linear(256, 128),                        nn.ELU(),
        )

        # ── Policy head ───────────────────────────────────────────────────────
        self.policy_mean   = nn.Linear(128, act_dim)
        # Learned log_std: one value per action dim, not input-dependent.
        # Initialised to 0 → std=1 at start (wide exploration).
        self.policy_logstd = nn.Parameter(torch.zeros(act_dim))

        # ── Value head ────────────────────────────────────────────────────────
        self.value_head = nn.Linear(128, 1)

        # ── Initialisation ────────────────────────────────────────────────────
        self._init_weights()

    def _init_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
                nn.init.zeros_(m.bias)
        # Small gain on policy mean: keeps initial actions near zero so the
        # agent doesn't immediately drive into walls or fire randomly.
        nn.init.orthogonal_(self.policy_mean.weight, gain=0.01)
        nn.init.zeros_(self.policy_mean.bias)
        # Value head: standard gain
        nn.init.orthogonal_(self.value_head.weight, gain=1.0)

    # ── Forward ───────────────────────────────────────────────────────────────

    def forward(self, obs: torch.Tensor) -> tuple[Normal, torch.Tensor]:
        """
        Returns (action_distribution, value_estimate).
        obs shape: (batch, obs_dim) or (obs_dim,)
        """
        h     = self.trunk(obs)
        mean  = torch.tanh(self.policy_mean(h))
        std   = self.policy_logstd.exp().expand_as(mean)
        dist  = Normal(mean, std)
        value = self.value_head(h)
        return dist, value

    # ── Convenience methods ───────────────────────────────────────────────────

    @torch.no_grad()
    def act(self, obs: torch.Tensor, deterministic: bool = False) -> tuple:
        """
        Sample an action for environment stepping.
        Returns (action, log_prob, value) all as tensors.
        """
        dist, value = self(obs)
        action      = dist.mean if deterministic else dist.sample()
        action      = action.clamp(-1.0, 1.0)
        log_prob    = dist.log_prob(action).sum(dim=-1)
        return action, log_prob, value.squeeze(-1)

    def evaluate(self, obs: torch.Tensor, action: torch.Tensor) -> tuple:
        """
        Evaluate stored actions during PPO update.
        Returns (log_prob, value, entropy) — all (batch,) tensors.
        """
        dist, value = self(obs)
        log_prob    = dist.log_prob(action).sum(dim=-1)
        entropy     = dist.entropy().sum(dim=-1)
        return log_prob, value.squeeze(-1), entropy