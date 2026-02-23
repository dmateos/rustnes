import argparse
import collections
import random
import sys

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
except ImportError:
    print("Requires torch (`pip install -r python/requirements.txt`).", file=sys.stderr)
    sys.exit(1)

from rustnes_uds_client import ACTION_SET, RustNesUdsClient


class TinyPolicy(nn.Module):
    def __init__(self, n_actions: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Flatten(),
            nn.Linear(80 * 84, 256),
            nn.ReLU(),
            nn.Linear(256, n_actions),
        )

    def forward(self, x):
        return self.net(x)


def compute_intrinsic_reward(prev_obs, obs, prev_ram, ram):
    # Reward visible state change and RAM change so the agent prefers "doing something"
    # over sitting on one screen. This is a demo signal, not task-specific game reward.
    obs_change = float(np.mean(np.abs(obs.astype(np.float32) - prev_obs.astype(np.float32)))) / 255.0
    ram_change = float(np.count_nonzero(ram != prev_ram)) / 256.0
    return (1.5 * obs_change) + (0.5 * ram_change) - 0.005


def main():
    parser = argparse.ArgumentParser(description="Intrinsic-reward RL demo over RustNES UDS API")
    parser.add_argument("--socket", required=True)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--steps", type=int, default=128)
    parser.add_argument("--frame-skip", type=int, default=4)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    action_names = ["NOOP", "LEFT", "RIGHT", "A", "B", "LEFT_A", "RIGHT_A", "START"]
    action_values = [ACTION_SET[name] for name in action_names]

    client = RustNesUdsClient(args.socket)
    client.ping()

    model = TinyPolicy(len(action_values))
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    recent_returns = collections.deque(maxlen=10)

    for ep in range(args.episodes):
        client.reset()
        obs = client.get_frame_gray_80x84()
        ram = np.frombuffer(client.get_ram(0, 2048), dtype=np.uint8).copy()

        ep_reward = 0.0
        log_probs = []
        rewards = []
        action_counts = collections.Counter()

        for t in range(args.steps):
            x = torch.from_numpy(obs.astype(np.float32) / 255.0).unsqueeze(0)
            logits = model(x)
            dist = torch.distributions.Categorical(logits=logits)

            # Small exploration schedule.
            eps = max(0.05, 0.25 * (1.0 - (ep / max(1, args.episodes))))
            if random.random() < eps:
                action_idx = random.randrange(len(action_values))
                log_prob = dist.log_prob(torch.tensor(action_idx))
            else:
                sample = dist.sample()
                action_idx = int(sample.item())
                log_prob = dist.log_prob(sample)

            # Early episodes often need START to leave menus/title screens.
            if ep < 3 and t < 10 and random.random() < 0.30:
                action_idx = action_names.index("START")
                log_prob = dist.log_prob(torch.tensor(action_idx))

            step_info = client.step(action_values[action_idx], frame_skip=args.frame_skip)
            next_obs = client.get_frame_gray_80x84()
            next_ram = np.frombuffer(client.get_ram(0, 2048), dtype=np.uint8).copy()

            reward = compute_intrinsic_reward(obs, next_obs, ram, next_ram)
            ep_reward += reward

            log_probs.append(log_prob)
            rewards.append(reward)
            action_counts[action_names[action_idx]] += 1

            obs = next_obs
            ram = next_ram

            if t % 32 == 0:
                print(
                    f"ep={ep:02d} step={t:03d} frame={step_info['frame_no']} "
                    f"a={action_names[action_idx]:>7} r={reward:+.3f}"
                )

        # REINFORCE update (enough for a demo loop).
        returns = []
        g = 0.0
        for r in reversed(rewards):
            g = r + 0.99 * g
            returns.append(g)
        returns.reverse()
        returns = torch.tensor(returns, dtype=torch.float32)
        if len(returns) > 1:
            returns = (returns - returns.mean()) / (returns.std() + 1e-6)

        loss = 0.0
        for log_prob, ret in zip(log_probs, returns):
            loss = loss + (-log_prob * ret)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        recent_returns.append(ep_reward)
        top_actions = ", ".join(f"{k}:{v}" for k, v in action_counts.most_common(4))
        avg10 = sum(recent_returns) / len(recent_returns)
        print(
            f"episode={ep:02d} return={ep_reward:.3f} avg10={avg10:.3f} "
            f"loss={float(loss):.4f} actions=[{top_actions}]"
        )

    client.close()


if __name__ == "__main__":
    main()
