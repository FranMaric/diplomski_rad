#!/usr/bin/env python3

import sys
from pathlib import Path
import rosbag2_py


def analyze_gaps(bag_path: Path):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    last_ts: dict[str, int] = {}
    max_gap: dict[str, float] = {}
    gap_count: dict[str, int] = {}

    while reader.has_next():
        topic, _, timestamp_ns = reader.read_next()
        if topic in last_ts:
            gap_s = (timestamp_ns - last_ts[topic]) / 1e9
            if gap_s > max_gap.get(topic, 0.0):
                max_gap[topic] = gap_s
            if gap_s > 0.1:
                gap_count[topic] = gap_count.get(topic, 0) + 1
        last_ts[topic] = timestamp_ns

    reader.close()
    return max_gap, gap_count


def print_gap_report(bag_path: Path):
    print(f"Bag: {bag_path.parent.name}")
    print(f"{'Topic':<45} {'Max gap (s)':>12} {'Gaps >0.1s':>11}")
    print("-" * 71)
    gaps, gap_count = analyze_gaps(bag_path)
    for topic, gap in sorted(gaps.items(), key=lambda x: -x[1]):
        flag = "  <-- WARNING" if gap > 0.5 else ""
        count = gap_count.get(topic, 0)
        print(f"{topic:<45} {gap:>12.4f} {count:>11}{flag}")


if __name__ == "__main__":
    bags_dir = Path(__file__).parent / "bags"

    if len(sys.argv) > 1:
        episode_dirs = [bags_dir / sys.argv[1]]
    else:
        episode_dirs = sorted(
            (d for d in bags_dir.iterdir() if d.is_dir() and d.name.startswith("episode_")),
            key=lambda d: int(d.name.split("_")[1]),
        )

    for bag_dir in episode_dirs:
        mcap_files = list(bag_dir.glob("*.mcap"))
        if not mcap_files:
            print(f"No .mcap file found in {bag_dir}, skipping.")
            continue
        print_gap_report(mcap_files[0])
        print()
