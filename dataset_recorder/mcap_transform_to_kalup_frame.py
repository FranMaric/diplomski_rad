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

# Mirrors kalup_pose_transformer.py exactly
_t = [-(0.0277 - 0.006 + 0.03505 + 0.0128), 0.011 + 0.17905, -(0.00925 + 0.076 + 0.01415)]
TRANSLATION = np.array([_t[0] + 0.0, _t[1] + 0.004, _t[2] + 0.02])

KALUP_TOPIC    = '/vrpn_mocap/Kalup/pose'
BRUSILICA_TOPIC = '/vrpn_mocap/Brusilica/pose'
OUT_TOPIC      = '/tool_center_pose'
MSG_TYPE       = 'geometry_msgs/msg/PoseStamped'


def find_mcap(path: Path) -> Path:
    if path.is_file() and path.suffix == '.mcap':
        return path
    if path.is_dir():
        mcaps = sorted(path.glob('*.mcap'))
        if mcaps:
            return mcaps[0]
    raise FileNotFoundError(f'No .mcap file found at: {path}')


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
    parser.add_argument('input',  type=Path, help='Input bag directory or .mcap file')
    parser.add_argument('-o', '--output', type=Path, default=None,
                        help='Output bag directory (default: <stem>_kalup_frame)')
    args = parser.parse_args()

    mcap_in = find_mcap(args.input.resolve())
    out_dir = args.output.resolve() if args.output else mcap_in.parent / f'{mcap_in.stem}_kalup_frame'

    if out_dir.exists():
        sys.exit(f'ERROR: output already exists: {out_dir}')

    # ── read all messages ──────────────────────────────────────────────────────
    print(f'Reading {mcap_in}')
    stor_r = rosbag2_py.StorageOptions(uri=str(mcap_in), storage_id='mcap')
    conv   = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(stor_r, conv)

    type_map      = {info.name: info.type for info in reader.get_all_topics_and_types()}
    kalup_raw     = []              # [(ts_ns, raw_bytes)]
    brusilica_raw = []              # [(ts_ns, raw_bytes)]
    passthrough   = {}              # topic -> [(ts_ns, raw_bytes)]  (non-vrpn topics)

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

    # ── deserialise kalup, build lookup arrays ─────────────────────────────────
    kalup_ts   = np.empty(len(kalup_raw), dtype=np.int64)
    kalup_pos  = np.empty((len(kalup_raw), 3), dtype=np.float64)
    kalup_quat = np.empty((len(kalup_raw), 4), dtype=np.float64)

    for i, (ts, data) in enumerate(kalup_raw):
        msg = deserialize_message(data, PoseStamped)
        p, q = msg.pose.position, msg.pose.orientation
        kalup_ts[i]     = ts
        kalup_pos[i]    = [p.x, p.y, p.z]
        kalup_quat[i]   = [q.x, q.y, q.z, q.w]

    # ── transform brusilica messages ───────────────────────────────────────────
    print('Transforming ...')
    output_msgs = []   # [(ts_ns, raw_bytes)]

    for ts, data in brusilica_raw:
        msg = deserialize_message(data, PoseStamped)
        p, q = msg.pose.position, msg.pose.orientation
        p_b = np.array([p.x, p.y, p.z])
        q_b = np.array([q.x, q.y, q.z, q.w])

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

    # ── merge transformed + passthrough, sorted by timestamp ──────────────────
    all_msgs = [(ts, OUT_TOPIC, data) for ts, data in output_msgs]
    for topic, msgs in passthrough.items():
        for ts, data in msgs:
            all_msgs.append((ts, topic, data))
    all_msgs.sort(key=lambda x: x[0])

    total = len(all_msgs)
    print(f'Writing {out_dir} ({total} messages) ...')
    stor_w = rosbag2_py.StorageOptions(uri=str(out_dir), storage_id='mcap')
    writer = rosbag2_py.SequentialWriter()
    writer.open(stor_w, conv)

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
    print('Done.')


if __name__ == '__main__':
    main()
