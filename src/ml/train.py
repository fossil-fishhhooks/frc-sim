"""
train.py — Training loop.

Usage:
    python train.py
    python train.py --resume checkpoints/model_100000.pt

The sim must be running before this starts:
    ./frc_sim --scene assets/scenes/955.json --robot assets/bodies/robot.json@127.0.0.1:5810 --speed 4

Install deps:
    pip install robotpy torch gymnasium numpy
"""

import argparse
import os
import time
import numpy as np
import torch

from config import Config
from env    import FRCSimEnv
from model  import ActorCritic
from ppo import RolloutBuffer, RewardNormalizer, ppo_update

import sys

class Tee:
    def __init__(self, *files):
        self.files = files
    def write(self, obj):
        for f in self.files:
            f.write(obj)
            f.flush()
    def flush(self):
        for f in self.files:
            f.flush()

log_file = open("training.log", "a")
sys.stdout = Tee(sys.__stdout__, log_file)
sys.stderr = Tee(sys.__stderr__, log_file)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to checkpoint .pt to resume from")
    args = parser.parse_args()

    cfg    = Config()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    os.makedirs(cfg.checkpoint_dir, exist_ok=True)

    # ── Environment ───────────────────────────────────────────────────────────
    env = FRCSimEnv(cfg)
    obs_dim = env.observation_space.shape[0]
    act_dim = env.action_space.shape[0]  # 3: fwd/strafe/rot; shooter frozen
    print(f"obs_dim={obs_dim}  act_dim={act_dim}")

    # ── Model ─────────────────────────────────────────────────────────────────
    model     = ActorCritic(obs_dim, act_dim).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.lr)
    
    reward_normalizer = RewardNormalizer(gamma=cfg.gamma)

    start_step = 0
#     if args.resume:
#         ckpt = torch.load(args.resume, map_location=device, weights_only=False)
#         model.load_state_dict(ckpt["model"])
#         optimizer.load_state_dict(ckpt["optimizer"])
#         start_step = ckpt.get("total_steps", 0)
#         if "reward_norm" in ckpt:
#             rn = ckpt["reward_norm"]
#             reward_normalizer._mean  = rn["mean"]
#             reward_normalizer._var   = rn["var"]
#             reward_normalizer._count = rn["count"]
#             reward_normalizer._ret   = rn["ret"]
    if args.resume:
        ckpt = torch.load(args.resume, map_location=device, weights_only=False)
        model.load_state_dict(ckpt["model"])
        optimizer.load_state_dict(ckpt["optimizer"])
        start_step = ckpt.get("total_steps", 0)
        # reward_norm intentionally not restored — let it rebuild if reward
        # structure has changed since the checkpoint was saved

    buffer = RolloutBuffer(cfg.steps_per_update, obs_dim, act_dim)
    

    # ── Training state ────────────────────────────────────────────────────────
    total_steps     = start_step
    episode_reward  = 0.0
    episode_steps   = 0
    episode_count   = 0
    ep_rewards      = []      # recent episode rewards for logging
    ep_intakes      = []      # recent episode intake counts for logging
    ep_oob          = []      # recent episode OOB-termination flags

    obs, _ = env.reset()

    print("Waiting for sim connection...")
    time.sleep(2.0)
    print("Starting training.")

    # ── Main loop ─────────────────────────────────────────────────────────────
    while total_steps < cfg.total_steps:

        # ── Collect rollout ───────────────────────────────────────────────
        model.eval()
        for _ in range(cfg.steps_per_update):
            obs_t  = torch.FloatTensor(obs).to(device)
            action, log_prob, value = model.act(obs_t)

            next_obs, reward, terminated, truncated, info = env.step(action.cpu().numpy())
            done = terminated or truncated
            reward = reward_normalizer.update_and_normalize(reward, done)

            buffer.add(
                obs      = obs,
                action   = action.cpu().numpy(),
                log_prob = log_prob.item(),
                reward   = reward,
                done     = done,
                value    = value.item(),
            )

            obs             = next_obs
            episode_reward += reward
            episode_steps  += 1
            total_steps    += 1

            if done:
                ep_rewards.append(episode_reward)
                ep_intakes.append(info['intake_count'])
                ep_oob.append(1 if info.get('oob') else 0)
                episode_count += 1
                print(
                    f"ep={episode_count:4d}  "
                    f"steps={total_steps:8d}  "
                    f"reward={episode_reward:7.1f}  "
                    f"intakes={info['intake_count']}  "
                    f"score={info['score']}  "
                    f"ep_len={episode_steps}  "
                    f"oob={'Y' if info.get('oob') else 'N'}  "
                    f"t={info['match_time']:.1f}s"
                )
                episode_reward = 0.0
                episode_steps  = 0
                obs, _ = env.reset()

            if buffer.full():
                break

        # Bootstrap last value for GAE
        with torch.no_grad():
            _, last_value = model(torch.FloatTensor(obs).to(device))
        buffer.compute_returns_and_advantages(
            last_value.item(), cfg.gamma, cfg.gae_lambda
        )

        # ── PPO update ────────────────────────────────────────────────────
        model.train()
        stats = ppo_update(model, optimizer, buffer, cfg, device)
        buffer.reset()

        # ── Logging ───────────────────────────────────────────────────────
        if len(ep_rewards) >= 5:
            print(
                f"  [update]  steps={total_steps}  "
                f"mean_ep_reward={np.mean(ep_rewards[-10:]):.1f}  "
                f"mean_intakes={np.mean(ep_intakes[-10:]):.1f}  "
                f"oob_rate={np.mean(ep_oob[-10:]):.2f}  "
                f"policy_loss={stats['policy_loss']:.4f}  "
                f"value_loss={stats['value_loss']:.4f}  "
                f"entropy={stats['entropy']:.3f}"
            )

        # ── Checkpoint ────────────────────────────────────────────────────
        if total_steps % cfg.checkpoint_every < cfg.steps_per_update:
            path = os.path.join(cfg.checkpoint_dir, f"model_{total_steps}.pt")
            torch.save({
                    "model":       model.state_dict(),
                    "optimizer":   optimizer.state_dict(),
                    "total_steps": total_steps,
                    "config":      vars(cfg),
                    "reward_norm": {
                        "mean":  reward_normalizer._mean,
                        "var":   reward_normalizer._var,
                        "count": reward_normalizer._count,
                        "ret":   reward_normalizer._ret,
                    },
                }, path)
            print(f"  [ckpt] saved {path}")

    env.close()
    print("Training complete.")


if __name__ == "__main__":
    main()