#!/usr/bin/env python3
"""
Resample all topics in an MCAP bag to 30 Hz.

Usage:
    python resample_30hz.py <bag_dir_or_mcap_file> [-o output_bag_dir]

Rules by message type and source frequency:

  sensor_msgs/msg/Image          → nearest-neighbour  (always)
  sensor_msgs/msg/CameraInfo     → nearest-neighbour  (always)
  geometry_msgs/msg/PoseStamped  → avg position + Rotation.mean()   if src > 30 Hz
                                   linear position + slerp rotation  if src ≤ 30 Hz
  geometry_msgs/msg/WrenchStamped → field average                    if src > 30 Hz
                                    field-wise linear interpolation  if src ≤ 30 Hz
  anything else                  → nearest-neighbour  (fallback)
"""

import argparse
import copy
import importlib
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message

# ── constants ──────────────────────────────────────────────────────────────────

TARGET_HZ = 30
PERIOD_NS = int(round(1e9 / TARGET_HZ))   # 33_333_333 ns
HALF_NS = PERIOD_NS // 2                  # 16_666_666 ns
DOWNSAMPLE_MARGIN = 1.10                   # > 10% above target → downsample

_IMAGE   = "sensor_msgs/msg/Image"
_CAM    = "sensor_msgs/msg/CameraInfo"
_POSE   = "geometry_msgs/msg/PoseStamped"
_WRENCH = "geometry_msgs/msg/WrenchStamped"

_RAW_NN_TYPES = {_IMAGE, _CAM}  # skip deserialization, use raw CDR bytes


# ── helpers ────────────────────────────────────────────────────────────────────

def _get_class(type_str: str):
    pkg, _, cls = type_str.split("/")
    return getattr(importlib.import_module(f"{pkg}.msg"), cls)


def _freq(timestamps_ns: list) -> float:
    if len(timestamps_ns) < 2:
        return 0.0
    span = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    return (len(timestamps_ns) - 1) / span if span > 0 else 0.0


def _stamp(msg, ts_ns: int):
    """Update header.stamp in-place."""
    msg.header.stamp.sec = int(ts_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(ts_ns % 1_000_000_000)


def find_mcap(path: Path) -> Path:
    if path.is_file() and path.suffix == ".mcap":
        return path
    if path.is_dir():
        mcaps = sorted(path.glob("*.mcap"))
        if mcaps:
            return mcaps[0]
    raise FileNotFoundError(f"No .mcap file found at: {path}")


# ── nearest-neighbour (raw bytes, no deserialisation) ─────────────────────────

def nn_raw(raw_list: list, targets: np.ndarray) -> list:
    """Return [(target_ts_ns, raw_bytes), ...] using nearest source."""
    ts = np.array([t for t, _ in raw_list], dtype=np.int64)
    out = []
    for tgt in targets:
        idx = int(np.argmin(np.abs(ts - tgt)))
        out.append((int(tgt), raw_list[idx][1]))
    return out


# ── WrenchStamped ──────────────────────────────────────────────────────────────

def _w_arr(msg) -> np.ndarray:
    w = msg.wrench
    return np.array([w.force.x, w.force.y, w.force.z,
                     w.torque.x, w.torque.y, w.torque.z], dtype=np.float64)


def _apply_wrench(template, arr: np.ndarray, ts_ns: int):
    m = copy.deepcopy(template)
    m.wrench.force.x  = float(arr[0])
    m.wrench.force.y  = float(arr[1])
    m.wrench.force.z  = float(arr[2])
    m.wrench.torque.x = float(arr[3])
    m.wrench.torque.y = float(arr[4])
    m.wrench.torque.z = float(arr[5])
    _stamp(m, ts_ns)
    return m


def resample_wrench_avg(msgs: list, targets: np.ndarray) -> list:
    """Average all wrench fields within each ±half-period window."""
    ts   = np.array([t for t, _ in msgs], dtype=np.int64)
    data = np.array([_w_arr(m) for _, m in msgs])
    out  = []
    for tgt in targets:
        mask = (ts >= tgt - HALF_NS) & (ts < tgt + HALF_NS)
        if mask.any():
            avg = data[mask].mean(axis=0)
            template = msgs[int(np.argmin(np.abs(ts - tgt)))][1]
            m = _apply_wrench(template, avg, tgt)
        else:
            # gap in window → nearest neighbour
            m = copy.deepcopy(msgs[int(np.argmin(np.abs(ts - tgt)))][1])
            _stamp(m, tgt)
        out.append((int(tgt), serialize_message(m)))
    return out


def resample_wrench_interp(msgs: list, targets: np.ndarray) -> list:
    """Linearly interpolate wrench between the two bracketing messages."""
    ts   = np.array([t for t, _ in msgs], dtype=np.int64)
    data = np.array([_w_arr(m) for _, m in msgs])
    out  = []
    for tgt in targets:
        ir = int(np.searchsorted(ts, tgt))
        if ir == 0:
            val, tmpl = data[0], msgs[0][1]
        elif ir >= len(ts):
            val, tmpl = data[-1], msgs[-1][1]
        else:
            t0, t1 = float(ts[ir - 1]), float(ts[ir])
            α = (tgt - t0) / (t1 - t0)
            val  = data[ir - 1] * (1.0 - α) + data[ir] * α
            tmpl = msgs[ir - 1][1]
        m = _apply_wrench(tmpl, val, tgt)
        out.append((int(tgt), serialize_message(m)))
    return out


# ── PoseStamped ────────────────────────────────────────────────────────────────

def _p_parts(msg):
    p = msg.pose.position
    o = msg.pose.orientation
    return (np.array([p.x, p.y, p.z], dtype=np.float64),
            np.array([o.x, o.y, o.z, o.w], dtype=np.float64))


def _apply_pose(template, pos: np.ndarray, quat: np.ndarray, ts_ns: int):
    m = copy.deepcopy(template)
    m.pose.position.x    = float(pos[0])
    m.pose.position.y    = float(pos[1])
    m.pose.position.z    = float(pos[2])
    m.pose.orientation.x = float(quat[0])
    m.pose.orientation.y = float(quat[1])
    m.pose.orientation.z = float(quat[2])
    m.pose.orientation.w = float(quat[3])
    _stamp(m, ts_ns)
    return m


def resample_pose_avg(msgs: list, targets: np.ndarray) -> list:
    """Average position (centroid) + mean rotation (chordal mean) per window."""
    ts   = np.array([t for t, _ in msgs], dtype=np.int64)
    pos  = np.array([_p_parts(m)[0] for _, m in msgs])
    quat = np.array([_p_parts(m)[1] for _, m in msgs])
    out  = []
    for tgt in targets:
        mask = (ts >= tgt - HALF_NS) & (ts < tgt + HALF_NS)
        if mask.any():
            avg_pos = pos[mask].mean(axis=0)
            avg_q   = Rotation.from_quat(quat[mask]).mean().as_quat()
            tmpl    = msgs[int(np.argmin(np.abs(ts - tgt)))][1]
            m = _apply_pose(tmpl, avg_pos, avg_q, tgt)
        else:
            idx = int(np.argmin(np.abs(ts - tgt)))
            m   = copy.deepcopy(msgs[idx][1])
            _stamp(m, tgt)
        out.append((int(tgt), serialize_message(m)))
    return out


def resample_pose_interp(msgs: list, targets: np.ndarray) -> list:
    """Linear interpolation for translation, slerp for rotation."""
    ts   = np.array([t for t, _ in msgs], dtype=np.int64)
    pos  = np.array([_p_parts(m)[0] for _, m in msgs])
    quat = np.array([_p_parts(m)[1] for _, m in msgs])

    rots     = Rotation.from_quat(quat)
    slerp_fn = Slerp(ts.astype(np.float64), rots)

    out = []
    for tgt in targets:
        t_clamped = float(np.clip(tgt, ts[0], ts[-1]))
        interp_q  = slerp_fn(t_clamped).as_quat()

        ir = int(np.searchsorted(ts, tgt))
        if ir == 0:
            interp_pos, tmpl = pos[0], msgs[0][1]
        elif ir >= len(ts):
            interp_pos, tmpl = pos[-1], msgs[-1][1]
        else:
            t0, t1 = float(ts[ir - 1]), float(ts[ir])
            α          = (tgt - t0) / (t1 - t0)
            interp_pos = pos[ir - 1] * (1.0 - α) + pos[ir] * α
            tmpl       = msgs[ir - 1][1]

        m = _apply_pose(tmpl, interp_pos, interp_q, tgt)
        out.append((int(tgt), serialize_message(m)))
    return out


# ── gap filling (before resampling) ───────────────────────────────────────────

def fill_gaps_wrench(msgs: list, avg_hz: float) -> list:
    """Insert linearly interpolated messages into gaps > 1/avg_hz."""
    if len(msgs) < 2 or avg_hz <= 0:
        return msgs
    period_ns = int(round(1e9 / avg_hz))
    result = []
    for i in range(len(msgs) - 1):
        t0, m0 = msgs[i]
        t1, m1 = msgs[i + 1]
        result.append((t0, m0))
        gap_ns = t1 - t0
        if gap_ns > period_ns:
            n_insert = round(gap_ns / period_ns) - 1
            arr0 = _w_arr(m0)
            arr1 = _w_arr(m1)
            for k in range(1, n_insert + 1):
                α = k / (n_insert + 1)
                ts_s = t0 + int(gap_ns * α)
                result.append((ts_s, _apply_wrench(m0, arr0 * (1 - α) + arr1 * α, ts_s)))
    result.append(msgs[-1])
    return result


def fill_gaps_pose(msgs: list, avg_hz: float) -> list:
    """Insert linearly/slerp-interpolated messages into gaps > 1/avg_hz."""
    if len(msgs) < 2 or avg_hz <= 0:
        return msgs
    period_ns = int(round(1e9 / avg_hz))
    result = []
    for i in range(len(msgs) - 1):
        t0, m0 = msgs[i]
        t1, m1 = msgs[i + 1]
        result.append((t0, m0))
        gap_ns = t1 - t0
        if gap_ns > period_ns:
            n_insert = round(gap_ns / period_ns) - 1
            pos0, q0 = _p_parts(m0)
            pos1, q1 = _p_parts(m1)
            slerp_fn = Slerp([0.0, 1.0], Rotation.concatenate(
                [Rotation.from_quat(q0), Rotation.from_quat(q1)]
            ))
            for k in range(1, n_insert + 1):
                α = k / (n_insert + 1)
                ts_s = t0 + int(gap_ns * α)
                result.append((ts_s, _apply_pose(
                    m0,
                    pos0 * (1 - α) + pos1 * α,
                    slerp_fn(α).as_quat(),
                    ts_s,
                )))
    result.append(msgs[-1])
    return result


# ── dispatch ───────────────────────────────────────────────────────────────────

def resample_topic(
    type_str: str,
    msgs: list,            # [(ts_ns, deserialized_msg), ...]
    raw_list: list,        # [(ts_ns, raw_bytes), ...]  (populated only for RAW_NN types)
    targets: np.ndarray,
    source_hz: float,
) -> list:                 # [(ts_ns, raw_bytes), ...]
    if type_str in _RAW_NN_TYPES:
        return nn_raw(raw_list, targets)

    if not msgs:
        return []

    downsampling = source_hz > TARGET_HZ * DOWNSAMPLE_MARGIN

    if type_str == _POSE:
        return resample_pose_avg(msgs, targets) if downsampling else resample_pose_interp(msgs, targets)

    if type_str == _WRENCH:
        return resample_wrench_avg(msgs, targets) if downsampling else resample_wrench_interp(msgs, targets)

    # Fallback: nearest-neighbour on deserialized messages
    ts = np.array([t for t, _ in msgs], dtype=np.int64)
    out = []
    for tgt in targets:
        idx = int(np.argmin(np.abs(ts - tgt)))
        out.append((int(tgt), serialize_message(msgs[idx][1])))
    return out


# ── main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Resample all topics in an MCAP bag to 30 Hz."
    )
    parser.add_argument(
        "input",
        type=Path,
        help="Path to a bag directory or .mcap file.",
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help="Output bag directory. Default: <input_parent>/<stem>_30hz",
    )
    args = parser.parse_args()

    mcap_in = find_mcap(args.input.resolve())

    if args.output is not None:
        out_dir = args.output.resolve()
    else:
        out_dir = mcap_in.parent / f"{mcap_in.stem}_30hz"

    if out_dir.exists():
        print(f"ERROR: output already exists: {out_dir}", file=sys.stderr)
        sys.exit(1)

    # ── read ──────────────────────────────────────────────────────────────────
    print(f"Reading  {mcap_in}")
    stor_r = rosbag2_py.StorageOptions(uri=str(mcap_in), storage_id="mcap")
    conv   = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(stor_r, conv)

    type_map: dict[str, str] = {
        info.name: info.type for info in reader.get_all_topics_and_types()
    }

    raw_by_topic: dict[str, list] = defaultdict(list)   # topic → [(ts, bytes)]
    while reader.has_next():
        topic, data, ts = reader.read_next()
        raw_by_topic[topic].append((ts, data))
    reader.close()

    # ── deserialise non-image topics ──────────────────────────────────────────
    print("Deserialising ...")
    msgs_by_topic: dict[str, list] = {}
    for topic, raw_list in raw_by_topic.items():
        type_str = type_map[topic]
        if type_str in _RAW_NN_TYPES:
            msgs_by_topic[topic] = []          # not needed
        else:
            cls = _get_class(type_str)
            msgs_by_topic[topic] = [
                (ts, deserialize_message(data, cls)) for ts, data in raw_list
            ]

    # ── gap filling ───────────────────────────────────────────────────────────
    print("Filling gaps ...")
    for topic, type_str in type_map.items():
        if type_str not in (_POSE, _WRENCH):
            continue
        msgs = msgs_by_topic[topic]
        hz   = _freq([t for t, _ in msgs])
        if hz <= 0:
            continue
        period_ns = int(round(1e9 / hz))
        n_gaps = sum(
            1 for i in range(len(msgs) - 1)
            if msgs[i + 1][0] - msgs[i][0] > period_ns
        )
        if n_gaps == 0:
            continue
        n_before = len(msgs)
        if type_str == _POSE:
            msgs_by_topic[topic] = fill_gaps_pose(msgs, hz)
        else:
            msgs_by_topic[topic] = fill_gaps_wrench(msgs, hz)
        n_inserted = len(msgs_by_topic[topic]) - n_before
        print(f"  {topic}: {n_gaps} gap(s) → inserted {n_inserted} synthetic messages")

    # ── time range: intersection of all topics ────────────────────────────────
    t_start = max(v[0][0]  for v in raw_by_topic.values())
    t_end   = min(v[-1][0] for v in raw_by_topic.values())

    if t_start >= t_end:
        print("ERROR: no overlapping time range across topics.", file=sys.stderr)
        sys.exit(1)

    targets  = np.arange(t_start, t_end, PERIOD_NS, dtype=np.int64)
    duration = (t_end - t_start) / 1e9
    print(f"Time range {duration:.2f}s → {len(targets)} frames @ {TARGET_HZ} Hz")

    # ── resample ──────────────────────────────────────────────────────────────
    print("Resampling ...")
    output_msgs: list = []   # [(topic, raw_bytes, ts_ns)]

    for topic in sorted(type_map):
        raw_list = raw_by_topic[topic]
        ts_list  = [t for t, _ in raw_list]
        hz       = _freq(ts_list)
        type_str = type_map[topic]

        strategy = "nearest-raw" if type_str in _RAW_NN_TYPES else (
            "avg"    if hz > TARGET_HZ * DOWNSAMPLE_MARGIN else "interp"
        )
        print(f"  {topic:<45}  {type_str:<42}  {hz:>7.1f} Hz  [{strategy}]")

        resampled = resample_topic(
            type_str=type_str,
            msgs=msgs_by_topic[topic],
            raw_list=raw_list,
            targets=targets,
            source_hz=hz,
        )
        for ts_ns, data in resampled:
            output_msgs.append((topic, data, ts_ns))

    # ── write ──────────────────────────────────────────────────────────────────
    output_msgs.sort(key=lambda x: x[2])

    print(f"Writing  {out_dir}  ({len(output_msgs)} messages) ...")
    stor_w = rosbag2_py.StorageOptions(uri=str(out_dir), storage_id="mcap")
    writer = rosbag2_py.SequentialWriter()
    writer.open(stor_w, conv)

    for idx, (topic, type_str) in enumerate(type_map.items()):
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=idx,
                name=topic,
                type=type_str,
                serialization_format="cdr",
            )
        )

    for topic, data, ts_ns in output_msgs:
        writer.write(topic, data, ts_ns)

    writer.close()
    print("Done.")


if __name__ == "__main__":
    main()
