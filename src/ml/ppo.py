"""
ppo.py — Rollout buffer and PPO update.

Kept separate from train.py so the update logic is testable in isolation.
"""

import math
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from config import Config
from model  import ActorCritic


# ── Running reward normalizer ────────────────────────────────────────────────
class RewardNormalizer:
    """
    Fixed-scale reward scaling. Previously did return-based normalization,
    but that divided by a std dominated by OOB-penalty spikes, crushing the
    intake-rate signal for long episodes (see history). Kept as a class for
    train.py interface compatibility; the gamma/ret/mean/var state is unused.
    """
    def __init__(self, gamma: float, clip: float = 10.0):
        self.gamma  = gamma
        self.clip   = clip
        self._mean  = 0.0
        self._var   = 1.0
        self._count = 0
        self._ret   = 0.0

    def update_and_normalize(self, reward: float, done: bool) -> float:
        FIXED_SCALE = 5.0
        return float(np.clip(reward / FIXED_SCALE, -self.clip, self.clip))


# ── Rollout buffer ────────────────────────────────────────────────────────────

class RolloutBuffer:
    """2-D rollout buffer (steps_per_env × num_envs) for vectorised training."""

    def __init__(self, steps_per_env: int, num_envs: int, obs_dim: int, act_dim: int):
        self.steps_per_env = steps_per_env
        self.num_envs      = num_envs
        self.obs      = np.zeros((steps_per_env, num_envs, obs_dim),  dtype=np.float32)
        self.actions  = np.zeros((steps_per_env, num_envs, act_dim),  dtype=np.float32)
        self.log_probs= np.zeros((steps_per_env, num_envs),           dtype=np.float32)
        self.rewards  = np.zeros((steps_per_env, num_envs),           dtype=np.float32)
        self.dones    = np.zeros((steps_per_env, num_envs),           dtype=np.float32)
        self.values   = np.zeros((steps_per_env, num_envs),           dtype=np.float32)
        self.returns    = np.zeros((steps_per_env, num_envs),         dtype=np.float32)
        self.advantages = np.zeros((steps_per_env, num_envs),         dtype=np.float32)
        self._ptr = 0

    def add(self, obs_batch, action_batch, log_prob_batch, reward_batch,
            done_batch, value_batch):
        i = self._ptr
        self.obs[i]       = obs_batch
        self.actions[i]   = action_batch
        self.log_probs[i] = log_prob_batch
        self.rewards[i]   = reward_batch
        self.dones[i]     = done_batch.astype(np.float32)
        self.values[i]    = value_batch
        self._ptr += 1

    def full(self) -> bool:
        return self._ptr >= self.steps_per_env

    def reset(self):
        self._ptr = 0

    def compute_returns_and_advantages(self, last_values, gamma: float,
                                       gae_lambda: float):
        """Per-env GAE-Lambda.  last_values: shape (num_envs,)."""
        for e in range(self.num_envs):
            adv    = 0.0
            next_v = last_values[e]
            for t in reversed(range(self._ptr)):
                mask      = 1.0 - self.dones[t, e]
                delta     = self.rewards[t, e] + gamma * next_v * mask - self.values[t, e]
                adv       = delta + gamma * gae_lambda * mask * adv
                self.advantages[t, e] = adv
                self.returns[t, e]    = adv + self.values[t, e]
                next_v                 = self.values[t, e]

        valid = self.advantages[:self._ptr]
        self.advantages[:self._ptr] = (valid - valid.mean()) / (valid.std() + 1e-8)

    def iterate_minibatches(self, minibatch_size: int, device: torch.device):
        """Flatten then yield random minibatches."""
        n       = self._ptr * self.num_envs
        indices = np.random.permutation(n)
        for start in range(0, n, minibatch_size):
            idx  = indices[start : start + minibatch_size]
            sidx = idx // self.num_envs
            eidx = idx % self.num_envs
            yield {
                "obs":        torch.FloatTensor(self.obs[sidx, eidx]).to(device),
                "actions":    torch.FloatTensor(self.actions[sidx, eidx]).to(device),
                "log_probs":  torch.FloatTensor(self.log_probs[sidx, eidx]).to(device),
                "returns":    torch.FloatTensor(self.returns[sidx, eidx]).to(device),
                "advantages": torch.FloatTensor(self.advantages[sidx, eidx]).to(device),
            }


# ── PPO update ────────────────────────────────────────────────────────────────

def ppo_update(model:        ActorCritic,
               optimizer:    torch.optim.Optimizer,
               buffer:       RolloutBuffer,
               cfg:          Config,
               device:       torch.device,
               entropy_coef: float | None = None) -> dict:
    """
    Run cfg.n_epochs passes of PPO over the rollout buffer.
    Returns dict of mean losses for logging.

    entropy_coef overrides cfg.entropy_coef when given, so callers can decay
    it over training. Without decay, even a tiny constant entropy bonus
    accumulates over millions of steps and pushes policy_logstd toward its
    clamp ceiling — entropy rises instead of falling, and the policy never
    sharpens (observed: entropy climbed 2.8 -> 5.0 over a 5M-step run while
    mean intakes plateaued well below target).
    """
    coef = cfg.entropy_coef if entropy_coef is None else entropy_coef
    policy_losses, value_losses, entropy_vals = [], [], []

    for _ in range(cfg.n_epochs):
        for batch in buffer.iterate_minibatches(cfg.minibatch_size, device):

            log_prob, value, entropy = model.evaluate(batch["obs"], batch["actions"])

            # ── Policy loss (clipped surrogate) ──────────────────────────
            ratio    = (log_prob - batch["log_probs"]).exp()
            adv      = batch["advantages"]
            pg_loss  = torch.max(
                -adv * ratio,
                -adv * ratio.clamp(1.0 - cfg.clip_eps, 1.0 + cfg.clip_eps),
            ).mean()

            # ── Value loss ────────────────────────────────────────────────
            v_loss = F.mse_loss(value, batch["returns"])

            # ── Combined loss ─────────────────────────────────────────────
            loss = pg_loss + cfg.value_coef * v_loss - coef * entropy.mean()

            optimizer.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), cfg.max_grad_norm)
            optimizer.step()

            policy_losses.append(pg_loss.item())
            value_losses.append(v_loss.item())
            entropy_vals.append(entropy.mean().item())

    return {
        "policy_loss": np.mean(policy_losses),
        "value_loss":  np.mean(value_losses),
        "entropy":     np.mean(entropy_vals),
    }