#!/usr/bin/env python3

import sys
from pathlib import Path
import rosbag2_py


def analyze_gaps(bag_path: Path):
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    first_ts: dict[str, int] = {}
    last_ts: dict[str, int] = {}
    msg_count: dict[str, int] = {}
    max_gap: dict[str, float] = {}
    gap_count: dict[str, int] = {}

    while reader.has_next():
        topic, _, timestamp_ns = reader.read_next()
        if topic not in first_ts:
            first_ts[topic] = timestamp_ns
        if topic in last_ts:
            gap_s = (timestamp_ns - last_ts[topic]) / 1e9
            if gap_s > max_gap.get(topic, 0.0):
                max_gap[topic] = gap_s
            if gap_s > 0.1:
                gap_count[topic] = gap_count.get(topic, 0) + 1
        last_ts[topic] = timestamp_ns
        msg_count[topic] = msg_count.get(topic, 0) + 1

    reader.close()

    avg_freq: dict[str, float] = {}
    for topic in msg_count:
        span_s = (last_ts[topic] - first_ts[topic]) / 1e9
        avg_freq[topic] = (msg_count[topic] - 1) / span_s if span_s > 0 else 0.0

    return max_gap, gap_count, avg_freq


def format_gap_report(bag_path: Path) -> str:
    lines = []
    lines.append(f"Bag: {bag_path.parent.name}")
    lines.append(f"{'Topic':<45} {'Max gap (s)':>12} {'Gaps >0.1s':>11} {'Avg freq (Hz)':>14}")
    lines.append("-" * 86)
    gaps, gap_count, avg_freq = analyze_gaps(bag_path)
    for topic, gap in sorted(gaps.items()):
        flag = "  <-- WARNING" if gap > 0.5 else ""
        count = gap_count.get(topic, 0)
        freq = avg_freq.get(topic, 0.0)
        lines.append(f"{topic:<45} {gap:>12.4f} {count:>11} {freq:>14.2f}{flag}")
    return "\n".join(lines)


if __name__ == "__main__":
    bags_dir = Path(__file__).parent / "bags"
    report_path = Path(__file__).parent / "gap_analysis_report.txt"

    if len(sys.argv) > 1:
        episode_dirs = [bags_dir / sys.argv[1]]
    else:
        episode_dirs = sorted(
            (d for d in bags_dir.iterdir() if d.is_dir() and d.name.startswith("episode_")),
            key=lambda d: int(d.name.split("_")[1]),
        )

    sections = []
    total = len(episode_dirs)
    for i, bag_dir in enumerate(episode_dirs, 1):
        print(f"{i}/{total} {bag_dir.name}", flush=True)
        mcap_files = list(bag_dir.glob("*.mcap"))
        if not mcap_files:
            sections.append(f"No .mcap file found in {bag_dir}, skipping.")
            continue
        sections.append(format_gap_report(mcap_files[0]))

    report = "\n\n".join(sections) + "\n"
    report_path.write_text(report)
    print(f"Report written to {report_path}")
