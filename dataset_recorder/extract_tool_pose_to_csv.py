#!/usr/bin/env python3
"""
Extract /tool_center_pose messages from MCAP bags to CSV.

Output columns: x,y,z,rx,ry,rz,timestamp

Usage:
    python3 extract_tool_pose_to_csv.py <bags_dir> [-o <output.csv>]
"""

import argparse
import csv
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

POSE_TOPIC = "/tool_center_pose"

MCAP_GLOB = (
    "episode_*/episode_*_30hz/episode_*_kalup_frame/episode_*_kalup_frame_0.mcap"
)


def _discover_mcaps(bags_dir: Path) -> list[Path]:
    mcaps = sorted(
        bags_dir.glob(MCAP_GLOB),
        key=lambda p: int(p.parts[len(bags_dir.parts)].split("_")[1]),
    )
    print("[" + ",\n".join(f'  "{p}"' for p in mcaps) + "\n]")
    print(f"Found {len(mcaps)} episodes under {bags_dir}")
    return mcaps


def _quat_to_euler(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main():
    parser = argparse.ArgumentParser(
        description="Extract /tool_center_pose from MCAP bags to CSV"
    )
    parser.add_argument("bags_dir", type=Path, help="Root directory containing episode bags")
    parser.add_argument("-o", "--output", type=Path, default=Path("tool_poses.csv"),
                        help="Output CSV file (default: tool_poses.csv)")
    args = parser.parse_args()

    mcaps = _discover_mcaps(args.bags_dir)
    if not mcaps:
        print("No MCAP files found.")
        return

    conv = rosbag2_py.ConverterOptions("", "")
    total_rows = 0

    with open(args.output, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z", "rx", "ry", "rz", "timestamp"])

        for mcap_path in mcaps:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(mcap_path), storage_id="mcap"), conv)

            print(f"Processing {mcap_path} ...")
            type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}

            if POSE_TOPIC not in type_map:
                print(f"WARNING: {POSE_TOPIC!r} not found in {mcap_path}, skipping.")
                reader.close()
                continue

            count = 0
            while reader.has_next():
                topic, data, ts_ns = reader.read_next()
                if topic != POSE_TOPIC:
                    continue

                msg: PoseStamped = deserialize_message(data, PoseStamped)
                p = msg.pose.position
                o = msg.pose.orientation
                rx, ry, rz = _quat_to_euler(o.x, o.y, o.z, o.w)
                timestamp = ts_ns * 1e-9

                writer.writerow([p.x, p.y, p.z, rx, ry, rz, timestamp])
                count += 1

            reader.close()
            print(f"  {mcap_path.name}: {count} pose messages")
            total_rows += count

    print(f"\nWrote {total_rows} rows to {args.output}")


if __name__ == "__main__":
    main()
