#!/usr/bin/env python3
"""
Post-processes an MCAP bag: re-expresses brusilica pose in the kalup frame.

Reads:
  /vrpn_mocap/Kalup/pose     - mold rigid body in world frame
  /vrpn_mocap/Brusilica/pose - grinder tool in world frame

Writes:
  /tool_center_pose - grinder pose expressed in the kalup (cap-center) frame

Usage:
  python3 mcap_postprocess.py <bag_dir_or_mcap> [-o output_bag_dir]
"""

import argparse
import copy
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from geometry_msgs.msg import PoseStamped

# Mirrors kalup_pose_transformer.py with slight modifications
_t = [-(0.0277 - 0.006 + 0.03505 + 0.0128), 
      0.011 + 0.17905, 
      -(0.00925 + 0.076 + 0.01415)]
TRANSLATION = np.array([_t[0] + 0.02, _t[1] - 0.02, _t[2] + 0.04])

ROT_X_NEG90 = Rotation.from_euler('x', -90, degrees=True)

MCAP_GLOB = "episode_*/episode_*_30hz/episode_*_30hz_0.mcap"

KALUP_TOPIC    = '/vrpn_mocap_rotated/Kalup/pose'
BRUSILICA_TOPIC = '/vrpn_mocap_rotated/Brusilica/pose'
OUT_TOPIC      = '/tool_center_pose'
MSG_TYPE       = 'geometry_msgs/msg/PoseStamped'


def _discover_mcaps(bags_dir: Path) -> list[Path]:
    mcaps = sorted(
        bags_dir.glob(MCAP_GLOB),
        key=lambda p: int(p.parts[len(bags_dir.parts)].split("_")[1]),
    )
    print("[" + ",\n".join(f'  "{p}"' for p in mcaps) + "\n]")
    print(f"Found {len(mcaps)} episodes under {bags_dir}")
    return mcaps


def cap_center_world(pos: np.ndarray, quat: np.ndarray) -> np.ndarray:
    """Return cap-center position in world frame given kalup body pose."""
    return pos + Rotation.from_quat(quat).apply(TRANSLATION)


def to_kalup_frame(p_b, q_b, p_cap, q_kalup):
    """Express brusilica pose (p_b, q_b) relative to the kalup frame."""
    R_inv = Rotation.from_quat(q_kalup).inv()
    pos = R_inv.apply(p_b - p_cap)
    rot = (R_inv * Rotation.from_quat(q_b)).as_quat()
    return pos, rot


def nearest_idx(sorted_ts: np.ndarray, ts: int) -> int:
    idx = int(np.searchsorted(sorted_ts, ts))
    if idx == 0:
        return 0
    if idx >= len(sorted_ts):
        return len(sorted_ts) - 1
    return idx - 1 if (ts - sorted_ts[idx - 1]) <= (sorted_ts[idx] - ts) else idx


def main():
    parser = argparse.ArgumentParser(
        description='Re-express brusilica pose in kalup frame and write to new MCAP bag.'
    )
    parser.add_argument('input', type=Path, help='Bags root directory (searched with MCAP_GLOB)')
    args = parser.parse_args()

    conv = rosbag2_py.ConverterOptions('', '')

    for mcap_in in _discover_mcaps(args.input.resolve()):
        out_dir = mcap_in.parent / f'{mcap_in.stem}_kalup_frame'

        if out_dir.exists():
            sys.exit(f'ERROR: output already exists: {out_dir}')

        # ── read all messages ──────────────────────────────────────────────────
        print(f'Reading {mcap_in}')
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(mcap_in), storage_id='mcap'), conv)

        type_map      = {info.name: info.type for info in reader.get_all_topics_and_types()}
        kalup_raw     = []
        brusilica_raw = []
        passthrough   = {}

        while reader.has_next():
            topic, data, ts = reader.read_next()
            if topic == KALUP_TOPIC:
                kalup_raw.append((ts, data))
            elif topic == BRUSILICA_TOPIC:
                brusilica_raw.append((ts, data))
            elif not topic.startswith('/vrpn_mocap'):
                passthrough.setdefault(topic, []).append((ts, data))
        reader.close()

        if not kalup_raw:
            sys.exit(f'ERROR: topic {KALUP_TOPIC} not found in bag')
        if not brusilica_raw:
            sys.exit(f'ERROR: topic {BRUSILICA_TOPIC} not found in bag')

        print(f'  {KALUP_TOPIC}: {len(kalup_raw)} messages')
        print(f'  {BRUSILICA_TOPIC}: {len(brusilica_raw)} messages')
        for t in sorted(passthrough):
            print(f'  {t}: {len(passthrough[t])} messages (pass-through)')

        # ── deserialise kalup, build lookup arrays ─────────────────────────────
        kalup_ts   = np.empty(len(kalup_raw), dtype=np.int64)
        kalup_pos  = np.empty((len(kalup_raw), 3), dtype=np.float64)
        kalup_quat = np.empty((len(kalup_raw), 4), dtype=np.float64)

        for i, (ts, data) in enumerate(kalup_raw):
            msg = deserialize_message(data, PoseStamped)
            p, q = msg.pose.position, msg.pose.orientation
            kalup_ts[i]   = ts
            kalup_pos[i]  = [p.x, p.y, p.z]
            kalup_quat[i] = (Rotation.from_quat([q.x, q.y, q.z, q.w]) * ROT_X_NEG90).as_quat()

        # ── transform brusilica messages ───────────────────────────────────────
        print('Transforming ...')
        output_msgs = []

        for ts, data in brusilica_raw:
            msg = deserialize_message(data, PoseStamped)
            p, q = msg.pose.position, msg.pose.orientation
            p_b = np.array([p.x, p.y, p.z])
            q_b = (Rotation.from_quat([q.x, q.y, q.z, q.w]) * ROT_X_NEG90).as_quat()

            idx   = nearest_idx(kalup_ts, ts)
            p_cap = cap_center_world(kalup_pos[idx], kalup_quat[idx])
            pos, rot = to_kalup_frame(p_b, q_b, p_cap, kalup_quat[idx])

            out = copy.deepcopy(msg)
            out.header.frame_id    = 'kalup'
            out.pose.position.x    = float(pos[0])
            out.pose.position.y    = float(pos[1])
            out.pose.position.z    = float(pos[2])
            out.pose.orientation.x = float(rot[0])
            out.pose.orientation.y = float(rot[1])
            out.pose.orientation.z = float(rot[2])
            out.pose.orientation.w = float(rot[3])

            output_msgs.append((ts, serialize_message(out)))

        # ── merge transformed + passthrough, sorted by timestamp ──────────────
        all_msgs = [(ts, OUT_TOPIC, data) for ts, data in output_msgs]
        for topic, msgs in passthrough.items():
            for ts, data in msgs:
                all_msgs.append((ts, topic, data))
        all_msgs.sort(key=lambda x: x[0])

        print(f'Writing {out_dir} ({len(all_msgs)} messages) ...')
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(out_dir), storage_id='mcap'), conv)

        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name=OUT_TOPIC, type=MSG_TYPE, serialization_format='cdr'
        ))
        for idx, topic in enumerate(sorted(passthrough), start=1):
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=idx, name=topic, type=type_map[topic], serialization_format='cdr'
            ))

        for ts, topic, data in all_msgs:
            writer.write(topic, data, ts)

        writer.close()
        print(f'Done. Written to {out_dir}')


if __name__ == '__main__':
    main()
