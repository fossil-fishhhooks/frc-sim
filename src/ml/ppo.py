"""
ppo.py — Rollout buffer and PPO update.

Kept separate from train.py so the update logic is testable in isolation.
"""

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from config import Config
from model  import ActorCritic


# ── Rollout buffer ────────────────────────────────────────────────────────────

class RolloutBuffer:
    """Stores one rollout, computes GAE advantages, yields minibatches."""

    def __init__(self, capacity: int, obs_dim: int, act_dim: int):
        self.capacity = capacity
        self.obs      = np.zeros((capacity, obs_dim),  dtype=np.float32)
        self.actions  = np.zeros((capacity, act_dim),  dtype=np.float32)
        self.log_probs= np.zeros(capacity,              dtype=np.float32)
        self.rewards  = np.zeros(capacity,              dtype=np.float32)
        self.dones    = np.zeros(capacity,              dtype=np.float32)
        self.values   = np.zeros(capacity,              dtype=np.float32)
        # Filled by compute_returns_and_advantages:
        self.returns    = np.zeros(capacity,            dtype=np.float32)
        self.advantages = np.zeros(capacity,            dtype=np.float32)
        self._ptr = 0

    def add(self, obs, action, log_prob: float, reward: float,
            done: bool, value: float):
        i = self._ptr
        self.obs[i]       = obs
        self.actions[i]   = action
        self.log_probs[i] = log_prob
        self.rewards[i]   = reward
        self.dones[i]     = float(done)
        self.values[i]    = value
        self._ptr += 1

    def full(self) -> bool:
        return self._ptr >= self.capacity

    def reset(self):
        self._ptr = 0

    def compute_returns_and_advantages(self, last_value: float,
                                       gamma: float, gae_lambda: float):
        """
        GAE-Lambda advantage estimation.
        last_value: V(s_{T+1}) bootstrapped from the model if episode not done.
        """
        adv   = 0.0
        next_v = last_value
        for t in reversed(range(self._ptr)):
            mask   = 1.0 - self.dones[t]
            delta  = self.rewards[t] + gamma * next_v * mask - self.values[t]
            adv    = delta + gamma * gae_lambda * mask * adv
            self.advantages[t] = adv
            self.returns[t]    = adv + self.values[t]
            next_v             = self.values[t]

        # Normalize advantages over the rollout — stabilizes updates
        valid = self.advantages[:self._ptr]
        self.advantages[:self._ptr] = (valid - valid.mean()) / (valid.std() + 1e-8)

    def iterate_minibatches(self, minibatch_size: int, device: torch.device):
        """Yields random minibatches as dicts of tensors."""
        n       = self._ptr
        indices = np.random.permutation(n)
        for start in range(0, n, minibatch_size):
            idx = indices[start : start + minibatch_size]
            yield {
                "obs":        torch.FloatTensor(self.obs[idx]).to(device),
                "actions":    torch.FloatTensor(self.actions[idx]).to(device),
                "log_probs":  torch.FloatTensor(self.log_probs[idx]).to(device),
                "returns":    torch.FloatTensor(self.returns[idx]).to(device),
                "advantages": torch.FloatTensor(self.advantages[idx]).to(device),
            }


# ── PPO update ────────────────────────────────────────────────────────────────

def ppo_update(model:     ActorCritic,
               optimizer: torch.optim.Optimizer,
               buffer:    RolloutBuffer,
               cfg:       Config,
               device:    torch.device) -> dict:
    """
    Run cfg.n_epochs passes of PPO over the rollout buffer.
    Returns dict of mean losses for logging.
    """
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
            loss = pg_loss + cfg.value_coef * v_loss - cfg.entropy_coef * entropy.mean()

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