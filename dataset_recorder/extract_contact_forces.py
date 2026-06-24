#!/usr/bin/env python3
"""
Extract Z-force readings from /optoforce_0 during "contact" metadata state.

Reads /metadata (std_msgs/msg/String) and /optoforce_0
(geometry_msgs/msg/WrenchStamped) in timestamp order. Writes a CSV
(index, timestamp, z_force) for every WrenchStamped sample whose
current metadata state is "contact".

Output: <out_dir>/<episode>_z_force.csv

Usage
-----
  python extract_contact_forces.py --bags-dir line_bags
  python extract_contact_forces.py --bags-dir line_bags -o /contact_forces
"""

import argparse
import csv
import re
from datetime import datetime, timezone
from pathlib import Path

import rosbag2_py
from geometry_msgs.msg import WrenchStamped
from rclpy.serialization import deserialize_message
from std_msgs.msg import String

METADATA_TOPIC = "/metadata"
FORCE_TOPIC = "/optoforce_0"

MCAP_GLOB = (
    "episode_*/episode_*_30hz/episode_*_kalup_frame/episode_*_kalup_frame_0.mcap" # with all forces
    # "episode_*/episode_*_30hz/episode_*_kalup_frame/episode_*_0_30hz_0_kalup_frame_0_fz_only/episode_*_0_30hz_0_kalup_frame_0_fz_only_0.mcap"
)

def _discover_mcaps(bags_dir: Path) -> list[Path]:
    mcaps = sorted(
        bags_dir.glob(MCAP_GLOB),
        key=lambda p: int(p.parts[len(bags_dir.parts)].split("_")[1]),
    )
    print("[" + ",\n".join(f'  "{p}"' for p in mcaps) + "\n]")
    print(f"Found {len(mcaps)} episodes under {bags_dir}")
    return mcaps


def ns_to_iso(ts_ns: int) -> str:
    return datetime.fromtimestamp(ts_ns / 1e9, tz=timezone.utc).isoformat()


def episode_name(mcap_path: Path) -> str:
    name = mcap_path.parent.name
    name = re.sub(r"_metadata$", "", name)
    name = re.sub(r"_30hz$", "", name)
    return name


def process_bag(mcap_in: Path, out_dir: Path) -> None:
    stor_r = rosbag2_py.StorageOptions(uri=str(mcap_in), storage_id="mcap")
    conv = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(stor_r, conv)

    type_map = {ti.name: ti.type for ti in reader.get_all_topics_and_types()}

    if FORCE_TOPIC not in type_map:
        print(f"WARNING: {FORCE_TOPIC} not found in {mcap_in.parent.name}, skipping.")
        return
    # if METADATA_TOPIC not in type_map:
    #     print(f"WARNING: {METADATA_TOPIC} not found in {mcap_in.parent.name}, skipping.")
    #     return

    msgs = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic in (METADATA_TOPIC, FORCE_TOPIC):
            msgs.append((topic, data, ts))
    del reader

    out_dir.mkdir(parents=True, exist_ok=True)
    ep = episode_name(mcap_in)
    out_csv = out_dir / f"{ep}_z_force.csv"

    current_state = None
    index = 0

    with open(out_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "timestamp", "z_force"])
        for topic, data, ts_ns in msgs:
            # if topic == METADATA_TOPIC:
            #     current_state = deserialize_message(data, String).data
            if topic == FORCE_TOPIC:
                z = deserialize_message(data, WrenchStamped).wrench.force.z
                writer.writerow([index, ns_to_iso(ts_ns), z])
                index += 1

    print(f"Wrote {index} contact-force row(s) → {out_csv}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Extract Z-force during 'contact' state from MCAP bags."
    )
    parser.add_argument("--bags-dir", type=Path, required=True,
                        help="Root bags directory (contains episode_N subdirs).")
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=Path("/dataset_recorder/contact_forces"),
        help="Output directory for CSVs (default: /dataset_recorder/contact_forces).",
    )
    args = parser.parse_args()

    mcaps = _discover_mcaps(args.bags_dir.resolve())
    if not mcaps:
        raise SystemExit(f"No mcap files found under {args.bags_dir}\nPattern: {MCAP_GLOB}")

    out_dir = args.output.resolve()
    for mcap_in in mcaps:
        process_bag(mcap_in, out_dir)


if __name__ == "__main__":
    main()
