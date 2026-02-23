import argparse
import random
import sys

import numpy as np

try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
except ImportError:
    print("This smoke test requires torch (`pip install torch`).", file=sys.stderr)
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


def preprocess(frame_rgba: np.ndarray) -> np.ndarray:
    gray = frame_rgba[..., :3].mean(axis=2).astype(np.float32) / 255.0
    ds = gray[::3, ::3]
    return ds[:80, :84]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", required=True, help="Unix socket path used by rl_api")
    parser.add_argument("--episodes", type=int, default=2)
    parser.add_argument("--steps", type=int, default=128)
    parser.add_argument("--frame-skip", type=int, default=4)
    args = parser.parse_args()

    action_names = ["NOOP", "LEFT", "RIGHT", "A", "LEFT_A", "RIGHT_A", "START"]
    action_values = [ACTION_SET[name] for name in action_names]

    client = RustNesUdsClient(args.socket)
    client.ping()

    model = TinyPolicy(len(action_values))
    optimizer = optim.Adam(model.parameters(), lr=1e-3)

    for ep in range(args.episodes):
        client.reset()
        prev_ram = np.frombuffer(client.get_ram(0, 2048), dtype=np.uint8).copy()
        ep_reward = 0.0
        log_probs = []
        rewards = []

        for t in range(args.steps):
            frame = client.get_frame()
            obs = preprocess(frame)
            x = torch.from_numpy(obs).unsqueeze(0)
            logits = model(x)
            dist = torch.distributions.Categorical(logits=logits)
            action_idx = int(dist.sample().item())

            # Occasionally force START to get past title screens during smoke tests.
            if t < 8 and random.random() < 0.25:
                action_idx = action_names.index("START")

            step_info = client.step(action_values[action_idx], frame_skip=args.frame_skip)
            ram = np.frombuffer(client.get_ram(0, 2048), dtype=np.uint8).copy()

            # Placeholder reward: count changed RAM bytes (integration smoke only).
            reward = float(np.count_nonzero(ram != prev_ram)) / 100.0
            prev_ram = ram
            ep_reward += reward

            log_probs.append(dist.log_prob(torch.tensor(action_idx)))
            rewards.append(reward)

            if t % 16 == 0:
                print(
                    f"ep={ep} step={t} action={action_names[action_idx]} "
                    f"frame={step_info['frame_no']} reward={reward:.3f}"
                )

        # Simple REINFORCE-style update for smoke validation.
        returns = []
        g = 0.0
        for r in reversed(rewards):
            g = r + 0.99 * g
            returns.append(g)
        returns.reverse()
        returns = torch.tensor(returns, dtype=torch.float32)
        if returns.numel() > 1:
            returns = (returns - returns.mean()) / (returns.std() + 1e-6)

        loss = torch.tensor(0.0)
        for lp, ret in zip(log_probs, returns):
            loss = loss - lp * ret

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        print(f"episode={ep} total_reward={ep_reward:.3f} loss={loss.item():.4f}")

    client.close()


if __name__ == "__main__":
    main()
