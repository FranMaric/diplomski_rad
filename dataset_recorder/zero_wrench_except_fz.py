#!/usr/bin/env python3
"""
Zero all wrench dimensions on /optoforce_0.wrench except force.z.

Usage:
    python3 zero_wrench_except_fz.py <input.mcap> [-o <output.mcap>]
"""

import argparse
import copy
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import WrenchStamped

WRENCH_TOPIC = '/optoforce_0'

MCAP_GLOB = (
    "episode_*/episode_*_30hz/episode_*_kalup_frame/episode_*_kalup_frame_0.mcap" # with all forces
)

def _discover_mcaps(bags_dir: Path) -> list[Path]:
    mcaps = sorted(
        bags_dir.glob(MCAP_GLOB),
        key=lambda p: int(p.parts[len(bags_dir.parts)].split("_")[1]),
    )
    print("[" + ",\n".join(f'  "{p}"' for p in mcaps) + "\n]")
    print(f"Found {len(mcaps)} episodes under {bags_dir}")
    return mcaps


def main():
    parser = argparse.ArgumentParser(
        description='Zero all wrench fields except force.z on ' + WRENCH_TOPIC
    )
    parser.add_argument('input', type=Path, help='Input .mcap file or bag directory')
    args = parser.parse_args()

    for mcap_in in _discover_mcaps(args.input):
        mcap_out = mcap_in.parent / f'{mcap_in.stem}_fz_only'

        if mcap_out.exists():
            sys.exit(f'ERROR: output already exists: {mcap_out}')

        conv = rosbag2_py.ConverterOptions('', '')

        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(mcap_in), storage_id='mcap'), conv)

        type_map = {info.name: info.type for info in reader.get_all_topics_and_types()}

        if WRENCH_TOPIC not in type_map:
            sys.exit(f'ERROR: topic {WRENCH_TOPIC!r} not found in bag. '
                    f'Available topics: {sorted(type_map)}')

        all_msgs = []
        while reader.has_next():
            topic, data, ts = reader.read_next()
            all_msgs.append((ts, topic, data))
        reader.close()

        print(f'Read {len(all_msgs)} messages from {mcap_in}')

        wrench_count = sum(1 for _, t, _ in all_msgs if t == WRENCH_TOPIC)
        print(f'  {WRENCH_TOPIC}: {wrench_count} messages (will zero all fields except force.z)')
        for t in sorted(type_map):
            if t != WRENCH_TOPIC:
                n = sum(1 for _, topic, _ in all_msgs if topic == t)
                print(f'  {t}: {n} messages (pass-through)')

        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(mcap_out), storage_id='mcap'), conv)

        for idx, (name, msg_type) in enumerate(sorted(type_map.items())):
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=idx, name=name, type=msg_type, serialization_format='cdr'
            ))

        for ts, topic, data in all_msgs:
            if topic == WRENCH_TOPIC:
                msg = deserialize_message(data, WrenchStamped)
                fz = msg.wrench.force.z
                out = copy.deepcopy(msg)
                out.wrench.force.x  = 0.0
                out.wrench.force.y  = 0.0
                out.wrench.force.z  = fz
                out.wrench.torque.x = 0.0
                out.wrench.torque.y = 0.0
                out.wrench.torque.z = 0.0
                data = serialize_message(out)
            writer.write(topic, data, ts)

        writer.close()
        print(f'Done. Written to {mcap_out}')


if __name__ == '__main__':
    main()
