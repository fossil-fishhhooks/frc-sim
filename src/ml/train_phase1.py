"""
train_phase1.py — Phase 1 curriculum: pure piece-chasing, raycast obs.

One full synchronized episode per PPO update — all envs finish naturally
before the update runs so sims never stall mid-episode.
"""

import argparse
import os
import sys
import time

import numpy as np
import torch

from config     import Config
from vec_env    import VecEnv
from env_phase1 import Phase1Env
from model      import ActorCritic
from ppo        import RolloutBuffer, RewardNormalizer, ppo_update


class Tee:
    def __init__(self, *files):
        self.files = files
    def write(self, obj):
        for f in self.files:
            f.write(obj); f.flush()
    def flush(self):
        for f in self.files: f.flush()


log_file   = open("training_phase1.log", "a")
sys.stdout = Tee(sys.__stdout__, log_file)
sys.stderr = Tee(sys.__stderr__, log_file)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to checkpoint .pt to resume from")
    args = parser.parse_args()

    cfg = Config()
    cfg.max_episode_steps = 4_000
    cfg.checkpoint_dir    = "checkpoints_p1"
    cfg.total_steps       = 10_000_000

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Phase 1  [device={device}  envs={cfg.num_envs}  obs={cfg.obs_dim}]")

    os.makedirs(cfg.checkpoint_dir, exist_ok=True)

    vec_env  = VecEnv(cfg, env_class=Phase1Env)
    num_envs = vec_env.num_envs
    vec_env.wait_connected()

    obs_buf = vec_env.reset()
    obs_dim = obs_buf.shape[1]
    act_dim = cfg.act_dim

    assert obs_dim == cfg.obs_dim, (
        f"obs mismatch: env returned {obs_dim}, config expects {cfg.obs_dim}"
    )

    model     = ActorCritic(obs_dim, act_dim, cfg=cfg).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.lr)
    reward_normalizer = RewardNormalizer(gamma=cfg.gamma)

    start_step = 0
    if args.resume:
        ckpt = torch.load(args.resume, map_location=device, weights_only=False)
        # Only load model/optimizer — obs layout changed, reward norm is stale
        model.load_state_dict(ckpt["model"])
        optimizer.load_state_dict(ckpt["optimizer"])
        start_step = ckpt.get("total_steps", 0)
        saved_ver  = ckpt.get("config", {}).get("reward_version", -1)
        if saved_ver != cfg.reward_version:
            print(f"  reward_version mismatch ({saved_ver} vs {cfg.reward_version}) "
                  f"— reward normalizer reset")
        else:
            rn = ckpt.get("reward_norm", {})
            if rn:
                reward_normalizer._mean  = rn.get("mean",  0.0)
                reward_normalizer._var   = rn.get("var",   1.0)
                reward_normalizer._count = rn.get("count", 0)
                reward_normalizer._ret   = rn.get("ret",   0.0)
        print(f"Resumed from {args.resume} at step {start_step}")

    # Buffer sized for one full episode + headroom
    max_rollout = cfg.max_episode_steps + 256
    buffer = RolloutBuffer(max_rollout, num_envs, obs_dim, act_dim)

    total_steps   = start_step
    ep_rewards    = [0.0] * num_envs
    ep_steps      = [0]   * num_envs
    ep_intakes    = [0]   * num_envs
    episode_count = 0
    ep_buf        = []   # (reward, intakes, steps, match_time)
    prev_done     = np.zeros(num_envs, dtype=bool)
    start_time    = time.monotonic()

    print(f"obs_dim={obs_dim}  act_dim={act_dim}  "
          f"rays=piece:{cfg.n_rays_piece} "
          f"field_low:{cfg.n_rays_field_low} "
          f"field_high:{cfg.n_rays_field_high}")

    while total_steps < cfg.total_steps:
        model.eval()
        buffer.reset()
        prev_done[:] = False

        # ── Rollout loop — run until all envs finish their episode ────────────
        while True:
            with torch.no_grad():
                actions, log_probs, values = model.act(
                    torch.FloatTensor(obs_buf).to(device))

            next_obs_buf, rewards, term, trunc, infos = vec_env.step(
                actions.cpu().numpy())

            dones    = term | trunc
            new_done = dones & ~prev_done
            prev_done = dones.copy()

            normed = np.array([
                reward_normalizer.update_and_normalize(rewards[i], bool(dones[i]))
                for i in range(num_envs)], dtype=np.float32)

            if not buffer.full():
                buffer.add(
                    obs_buf,
                    actions.cpu().numpy(),
                    log_probs.detach().cpu().numpy(),
                    normed,
                    dones,
                    values.detach().cpu().numpy())

            for i in range(num_envs):
                ep_rewards[i] += normed[i]
                ep_steps[i]   += 1
                total_steps   += 1

                if new_done[i]:
                    info_i = {k: infos[k][i] for k in infos
                              if not k.startswith("_") and k != "final_obs"}
                    intakes = int(info_i.get("intake_count", 0))
                    ep_intakes[i] = intakes
                    ep_buf.append((
                        ep_rewards[i],
                        intakes,
                        ep_steps[i],
                        float(info_i.get("match_time", 0))))
                    episode_count += 1
                    print(
                        f"  ep {episode_count:4d}  |  "
                        f"steps {total_steps:8d}  |  "
                        f"reward {ep_rewards[i]:+7.1f}  |  "
                        f"intakes {intakes:3d}  |  "
                        f"len {ep_steps[i]:5d}  |  "
                        f"t {info_i.get('match_time', 0):.0f}s"
                    )
                    ep_rewards[i] = 0.0
                    ep_steps[i]   = 0

            obs_buf = next_obs_buf

            # VecEnv sync-reset fires when all matches done → term[:]=True
            if np.all(term):
                break

        # ── PPO update ────────────────────────────────────────────────────────
        with torch.no_grad():
            _, last_val = model(torch.FloatTensor(obs_buf).to(device))
        buffer.compute_returns_and_advantages(
            last_val.squeeze(-1).cpu().numpy(), cfg.gamma, cfg.gae_lambda)

        model.train()
        progress     = min(total_steps / cfg.total_steps, 1.0)
        # Fast early decay: entropy_coef → 0 by ~10% of training
        entropy_coef = cfg.entropy_coef * max(0.0, 1.0 - progress * 10.0)
        stats = ppo_update(model, optimizer, buffer, cfg, device,
                           entropy_coef=entropy_coef)

        if ep_buf:
            elapsed = time.monotonic() - start_time
            spm     = (total_steps - start_step) / max(elapsed / 60.0, 1e-6)
            recent  = ep_buf[-min(len(ep_buf), num_envs * 3):]
            mean_r  = np.mean([e[0] for e in recent])
            mean_i  = np.mean([e[1] for e in recent])
            print(
                f"  -- update -- "
                f"steps {total_steps:8d}  |  "
                f"{spm:.0f} spm  |  "
                f"reward {mean_r:+6.1f}  |  "
                f"intakes {mean_i:5.1f}  |  "
                f"policy loss {stats['policy_loss']:.4f}  |  "
                f"value loss {stats['value_loss']:.4f}  |  "
                f"entropy {stats['entropy']:.3f}  |  "
                f"entropy_coef {entropy_coef:.5f}"
            )

        # ── Checkpoint ────────────────────────────────────────────────────────
        if total_steps % cfg.checkpoint_every < num_envs * max_rollout:
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
            print(f"   checkpoint → {path}")

    vec_env.close()
    print("Phase 1 complete.")


if __name__ == "__main__":
    main()