#!/usr/bin/env python3

import os
import yaml
from pathlib import Path


def load_metadata(bags_dir: Path):
    episodes = []
    for episode_dir in sorted(bags_dir.iterdir()):
        metadata_file = episode_dir / "metadata.yaml"
        if not episode_dir.is_dir() or not metadata_file.exists():
            continue
        with open(metadata_file) as f:
            data = yaml.safe_load(f)
        info = data["rosbag2_bagfile_information"]
        episodes.append({
            "name": episode_dir.name,
            "duration_ns": info["duration"]["nanoseconds"],
            "start_ns": info["starting_time"]["nanoseconds_since_epoch"],
        })
    return episodes


def print_duration_stats(episodes):
    if not episodes:
        print("No episodes found.")
        return

    durations_s = [e["duration_ns"] / 1e9 for e in episodes]
    total_s = sum(durations_s)
    min_s = min(durations_s)
    max_s = max(durations_s)
    mean_s = total_s / len(durations_s)

    min_ep = episodes[durations_s.index(min_s)]["name"]
    max_ep = episodes[durations_s.index(max_s)]["name"]

    def fmt(s):
        m, sec = divmod(s, 60)
        return f"{int(m)}m {sec:.2f}s" if m else f"{sec:.2f}s"

    print(f"Episodes:          {len(episodes)}")
    print(f"Total duration:    {fmt(total_s)}")
    print(f"Mean duration:     {fmt(mean_s)}")
    print(f"Shortest episode:  {fmt(min_s)}  ({min_ep})")
    print(f"Longest episode:   {fmt(max_s)}  ({max_ep})")
    print()
    print(f"{'Episode':<20} {'Duration':>10}")
    print("-" * 32)
    for ep, dur in zip(episodes, durations_s):
        print(f"{ep['name']:<20} {fmt(dur):>10}")


if __name__ == "__main__":
    bags_dir = Path(__file__).parent / "bags"
    episodes = load_metadata(bags_dir)
    print_duration_stats(episodes)
