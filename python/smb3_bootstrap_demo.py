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


OVERWORLD_SIGNATURE = [0x059A, 0x0511, 0x053C, 0x0719]
LEVEL_SIGNATURE = [0x055C, 0x055D, 0x0565, 0x05F1, 0x058F, 0x0590, 0x0591, 0x0592]

# First-pass candidate state keys from your captures.
OVERWORLD_KEY_ADDRS = [0x0069, 0x006A, 0x0300, 0x0301, 0x005E]
LEVEL_KEY_ADDRS = [0x0565, 0x055D, 0x0574, 0x05F1, 0x00EC]


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


def detect_mode(prev_ram: np.ndarray, ram: np.ndarray) -> str:
    ow = sum(int(prev_ram[a] != ram[a]) for a in OVERWORLD_SIGNATURE)
    lv = sum(int(prev_ram[a] != ram[a]) for a in LEVEL_SIGNATURE)
    if lv >= 2 and lv > ow:
        return "level"
    if ow >= 1 and ow >= lv:
        return "overworld"
    return "unknown"


def obs_intrinsic(prev_obs: np.ndarray, obs: np.ndarray) -> float:
    return float(np.mean(np.abs(obs.astype(np.float32) - prev_obs.astype(np.float32)))) / 255.0


def key_for_mode(ram: np.ndarray, mode: str):
    if mode == "overworld":
        return tuple(int(ram[a]) for a in OVERWORLD_KEY_ADDRS)
    if mode == "level":
        return tuple(int(ram[a]) for a in LEVEL_KEY_ADDRS)
    return None


def auto_advance_to_gameplay(client: RustNesUdsClient, max_steps: int = 80, frame_skip: int = 4):
    """
    After reset, spam START / short waits until the game reaches overworld or a level.
    Returns (obs, ram, detected_mode) or (None, None, None) if it fails.
    """
    obs = client.get_frame_gray_80x84()
    ram = np.frombuffer(client.get_ram(0, 2048), dtype=np.uint8).copy()

    # Heuristic schedule that tends to leave title/menus.
    bootstrap_actions = [
        ACTION_SET["START"],
        ACTION_SET["NOOP"],
        ACTION_SET["START"],
        ACTION_SET["NOOP"],
        ACTION_SET["A"],
        ACTION_SET["NOOP"],
    ]

    last_non_unknown = None
    for i in range(max_steps):
        action = bootstrap_actions[i % len(bootstrap_actions)]
        client.step(action, frame_skip=frame_skip)
        next_obs = client.get_frame_gray_80x84()
        next_ram = np.frombuffer(client.get_ram(0, 2048), dtype=np.uint8).copy()
        mode = detect_mode(ram, next_ram)

        if mode in ("overworld", "level"):
            last_non_unknown = mode
            return next_obs, next_ram, mode

        obs = next_obs
        ram = next_ram

    return None, None, last_non_unknown


def main():
    parser = argparse.ArgumentParser(description="SMB3 bootstrap RL demo (overworld + level aware)")
    parser.add_argument("--socket", required=True)
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--steps", type=int, default=160)
    parser.add_argument("--frame-skip", type=int, default=4)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # START is intentionally excluded from policy actions to prevent menu-loop exploitation.
    action_names = ["NOOP", "LEFT", "RIGHT", "A", "B", "LEFT_A", "RIGHT_A"]
    action_values = [ACTION_SET[n] for n in action_names]

    client = RustNesUdsClient(args.socket)
    client.ping()

    model = TinyPolicy(len(action_values))
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    returns_window = collections.deque(maxlen=10)

    for ep in range(args.episodes):
        client.reset()
        obs, ram, boot_mode = auto_advance_to_gameplay(
            client, max_steps=80, frame_skip=args.frame_skip
        )
        if obs is None:
            print(f"episode={ep:02d} bootstrap_failed (could not leave menu/title)")
            continue
        print(f"episode={ep:02d} bootstrap_mode={boot_mode}")

        visited_overworld = set()
        visited_level = set()
        mode_counts = collections.Counter()
        action_counts = collections.Counter()
        ep_reward = 0.0
        log_probs = []
        rewards = []
        recent_unknown = 0
        recent_keys = collections.deque(maxlen=16)

        for t in range(args.steps):
            x = torch.from_numpy(obs.astype(np.float32) / 255.0).unsqueeze(0)
            logits = model(x)
            dist = torch.distributions.Categorical(logits=logits)

            eps = max(0.05, 0.30 * (1.0 - ep / max(1, args.episodes)))
            if random.random() < eps:
                action_idx = random.randrange(len(action_values))
                log_prob = dist.log_prob(torch.tensor(action_idx))
            else:
                sample = dist.sample()
                action_idx = int(sample.item())
                log_prob = dist.log_prob(sample)

            # START is excluded from policy actions; bootstrap handles menu exit.

            step_info = client.step(action_values[action_idx], frame_skip=args.frame_skip)
            next_obs = client.get_frame_gray_80x84()
            next_ram = np.frombuffer(client.get_ram(0, 2048), dtype=np.uint8).copy()

            mode = detect_mode(ram, next_ram)
            mode_counts[mode] += 1
            recent_unknown = recent_unknown + 1 if mode == "unknown" else 0

            reward = 0.0
            reward += 0.6 * obs_intrinsic(obs, next_obs)

            key = key_for_mode(next_ram, mode)
            if mode == "overworld":
                if key is not None and key not in visited_overworld:
                    visited_overworld.add(key)
                    reward += 0.40
                elif key is not None:
                    reward -= 0.01
                if int(ram[0x0069]) != int(next_ram[0x0069]) or int(ram[0x006A]) != int(next_ram[0x006A]):
                    reward += 0.08
            elif mode == "level":
                if key is not None and key not in visited_level:
                    visited_level.add(key)
                    reward += 0.25
                # Reward changes in likely progress proxies from your capture.
                for a in [0x0565, 0x05F1, 0x0574]:
                    if int(ram[a]) != int(next_ram[a]):
                        reward += 0.03
                if int(next_ram[0x00EC]) != int(ram[0x00EC]):
                    reward += 0.05
            else:
                # Unknown often means menu/transitions/static screens; penalize to push escape.
                reward -= 0.03

            # Penalize tight local loops in discovered state keys.
            if key is not None:
                recent_keys.append((mode, key))
                if recent_keys.count((mode, key)) >= 4:
                    reward -= 0.03

            # If we appear stuck in unknown/menu states, inject a START press once in a while.
            if recent_unknown >= 12 and (t % 8 == 0):
                client.step(ACTION_SET["START"], frame_skip=1)
                recent_unknown = 0

            reward -= 0.003  # time penalty

            ep_reward += reward
            log_probs.append(log_prob)
            rewards.append(reward)
            action_counts[action_names[action_idx]] += 1

            obs = next_obs
            ram = next_ram

            if t % 40 == 0:
                print(
                    f"ep={ep:02d} t={t:03d} frame={step_info['frame_no']} "
                    f"mode={mode:9s} a={action_names[action_idx]:>7} r={reward:+.3f}"
                )

        # REINFORCE
        returns = []
        g = 0.0
        for r in reversed(rewards):
            g = r + 0.99 * g
            returns.append(g)
        returns.reverse()
        returns = torch.tensor(returns, dtype=torch.float32)
        if returns.numel() > 1:
            returns = (returns - returns.mean()) / (returns.std() + 1e-6)

        loss = 0.0
        for lp, ret in zip(log_probs, returns):
            loss = loss + (-lp * ret)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        returns_window.append(ep_reward)
        avg10 = sum(returns_window) / len(returns_window)
        print(
            f"episode={ep:02d} return={ep_reward:.3f} avg10={avg10:.3f} "
            f"modes={dict(mode_counts)} ow_novel={len(visited_overworld)} lv_novel={len(visited_level)} "
            f"top_actions={action_counts.most_common(4)} loss={float(loss):.4f}"
        )

    client.close()


if __name__ == "__main__":
    main()
