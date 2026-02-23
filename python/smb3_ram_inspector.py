import argparse
import collections
import csv
import signal
import sys
import time
from dataclasses import dataclass

import numpy as np

from rustnes_uds_client import RustNesUdsClient


STOP = False


def _handle_sigint(_sig, _frame):
    global STOP
    STOP = True


signal.signal(signal.SIGINT, _handle_sigint)


@dataclass
class AddrStats:
    changes: int = 0
    abs_delta_sum: int = 0
    distinct_values: set = None
    last_seen: int = 0

    def __post_init__(self):
        if self.distinct_values is None:
            self.distinct_values = set()


def parse_watch_list(values):
    out = []
    for item in values:
        item = item.strip().lower()
        if item.startswith("0x"):
            out.append(int(item, 16))
        else:
            out.append(int(item))
    return out


def print_top(stats, current_ram, top_n, watch_addrs, label):
    ranked = sorted(
        stats.items(),
        key=lambda kv: (kv[1].changes, kv[1].abs_delta_sum, len(kv[1].distinct_values)),
        reverse=True,
    )[:top_n]

    print("\n=== SMB3 RAM Inspector Summary ===")
    if label:
        print(f"label={label}")
    print("top changing addresses:")
    for addr, s in ranked:
        print(
            f"  0x{addr:04X}  val={current_ram[addr]:3d}  "
            f"changes={s.changes:5d}  absΔ={s.abs_delta_sum:6d}  distinct={len(s.distinct_values):4d}"
        )

    if watch_addrs:
        print("watch:")
        for addr in watch_addrs:
            if 0 <= addr < len(current_ram):
                s = stats.get(addr)
                if s is None:
                    print(f"  0x{addr:04X} val={current_ram[addr]:3d} changes=0")
                else:
                    print(
                        f"  0x{addr:04X} val={current_ram[addr]:3d} changes={s.changes} "
                        f"absΔ={s.abs_delta_sum} distinct={len(s.distinct_values)}"
                    )


def main():
    parser = argparse.ArgumentParser(
        description="Passive SMB3 RAM inspector over RustNES UDS (use while playing in live window)."
    )
    parser.add_argument("--socket", required=True, help="UDS path (same as --api-socket in rustnes)")
    parser.add_argument("--poll-hz", type=float, default=10.0, help="RAM polls per second")
    parser.add_argument("--seconds", type=float, default=0.0, help="Stop after N seconds (0=until Ctrl-C)")
    parser.add_argument("--top", type=int, default=24, help="How many ranked addresses to print")
    parser.add_argument(
        "--watch",
        nargs="*",
        default=[],
        help="Specific addresses to display every summary (e.g. 0x0010 0x0057)",
    )
    parser.add_argument(
        "--summary-every",
        type=float,
        default=2.0,
        help="Print rolling summary every N seconds",
    )
    parser.add_argument(
        "--label",
        default="",
        help="Session label for logs (e.g. overworld, level, title)",
    )
    parser.add_argument(
        "--event-log",
        default="",
        help="CSV path for byte-change events (timestamp,label,addr,old,new,delta)",
    )
    parser.add_argument(
        "--min-delta",
        type=int,
        default=1,
        help="Only log/print changes with abs(delta) >= this threshold",
    )
    args = parser.parse_args()

    watch_addrs = parse_watch_list(args.watch)
    period = 1.0 / max(args.poll_hz, 0.1)

    client = RustNesUdsClient(args.socket)
    client.ping()

    prev_ram = np.frombuffer(client.get_ram(0, 2048), dtype=np.uint8).copy()
    stats = {}
    changes_recent = collections.deque(maxlen=128)

    csv_writer = None
    csv_file = None
    if args.event_log:
        csv_file = open(args.event_log, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["timestamp", "label", "addr_hex", "addr", "old", "new", "delta"])

    print("Connected. Play SMB3 in the live emulator window now.")
    print("Use separate runs with --label overworld and --label level to build reward maps.")
    print("Ctrl-C to stop.\n")

    t0 = time.time()
    next_summary = t0 + args.summary_every

    while not STOP:
        now = time.time()
        if args.seconds > 0 and (now - t0) >= args.seconds:
            break

        ram = np.frombuffer(client.get_ram(0, 2048), dtype=np.uint8).copy()
        diff_mask = ram != prev_ram
        changed_idxs = np.flatnonzero(diff_mask)

        tick_changes = 0
        for addr in changed_idxs.tolist():
            old = int(prev_ram[addr])
            new = int(ram[addr])
            delta = new - old
            if abs(delta) < args.min_delta:
                continue
            tick_changes += 1

            s = stats.get(addr)
            if s is None:
                s = AddrStats()
                stats[addr] = s
            s.changes += 1
            s.abs_delta_sum += abs(delta)
            s.distinct_values.add(new)
            s.distinct_values.add(old)
            s.last_seen = new

            if csv_writer is not None:
                csv_writer.writerow(
                    [f"{now:.6f}", args.label, f"0x{addr:04X}", addr, old, new, delta]
                )

        if tick_changes:
            changes_recent.append(tick_changes)

        if now >= next_summary:
            avg_recent = (sum(changes_recent) / len(changes_recent)) if changes_recent else 0.0
            print(
                f"[t={now - t0:6.1f}s] changed_addrs_this_window~{avg_recent:.1f}/poll "
                f"tracked={len(stats)}"
            )
            print_top(stats, ram, args.top, watch_addrs, args.label)
            next_summary = now + args.summary_every

        prev_ram = ram
        sleep_for = period - (time.time() - now)
        if sleep_for > 0:
            time.sleep(sleep_for)

    print("\nFinal summary:")
    print_top(stats, prev_ram, args.top, watch_addrs, args.label)

    if csv_file is not None:
        csv_file.flush()
        csv_file.close()
        print(f"\nWrote event log: {args.event_log}")

    client.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
