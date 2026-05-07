#!/usr/bin/env python3
"""
Convert /vrpn_mocap/Brusilica/pose from a ROS2 bag into a sensor_msgs/PointCloud2.

Modes:
  Default   — write a new bag containing only /trajectory_cloud next to the input bag.
  --in-place — rewrite the original bag with /trajectory_cloud added alongside all
               existing topics. The original is replaced atomically (temp + rename).

Usage:
  python3 pose_to_pointcloud.py <bag_path> [options]

  --topic      Pose topic to read   (default: /vrpn_mocap/Brusilica/pose)
  --output     Output directory     (default: <bag_dir>/pose_cloud)
  --storage    mcap or sqlite3      (auto-detected if omitted)
  --overwrite  Overwrite existing output directory (default mode only)
  --in-place   Add cloud topic to the original bag, replacing it
"""

import argparse
import os
import struct
import sys
import tempfile

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointCloud2, PointField


CLOUD_TOPIC = "/trajectory_cloud"
CLOUD_TYPE  = "sensor_msgs/msg/PointCloud2"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def open_reader(bag_path: str, storage_id: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


# ---------------------------------------------------------------------------
# Reading
# ---------------------------------------------------------------------------

def read_poses(bag_path: str, topic: str, storage_id: str):
    """Return (frame_id, [(timestamp_ns, x, y, z, qx, qy, qz, qw), ...])."""
    reader = open_reader(bag_path, storage_id)
    available = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in available:
        print(f"ERROR: topic '{topic}' not found. Available topics:")
        for name, ttype in available.items():
            print(f"  {name}  [{ttype}]")
        sys.exit(1)

    MsgClass = get_message(available[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    points, frame_id = [], "world"
    while reader.has_next():
        _, raw, ts = reader.read_next()
        msg = deserialize_message(raw, MsgClass)
        if not points:
            frame_id = msg.header.frame_id
        p, q = msg.pose.position, msg.pose.orientation
        points.append((ts, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return frame_id, points


def read_all_messages(bag_path: str, storage_id: str):
    """Return (topic_metadata_list, [(topic, raw_bytes, timestamp_ns), ...])."""
    reader = open_reader(bag_path, storage_id)
    topic_meta = reader.get_all_topics_and_types()
    messages = []
    while reader.has_next():
        messages.append(reader.read_next())
    return topic_meta, messages


# ---------------------------------------------------------------------------
# Building the cloud message
# ---------------------------------------------------------------------------

def build_pointcloud2(points, frame_id: str) -> tuple[PointCloud2, int]:
    n = len(points)
    point_step = 12  # 3 × float32
    data = bytearray(n * point_step)
    for i, (_, x, y, z, *_) in enumerate(points):
        struct.pack_into("<fff", data, i * point_step, float(x), float(y), float(z))

    ts_ns = points[0][0]
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = ts_ns // 1_000_000_000
    msg.header.stamp.nanosec = ts_ns % 1_000_000_000
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * n
    msg.data = bytes(data)
    msg.is_dense = True
    return msg, ts_ns


# ---------------------------------------------------------------------------
# Writers
# ---------------------------------------------------------------------------

def _open_writer(output_dir: str) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_dir, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return writer


def write_point_cloud_bag(points, frame_id: str, output_dir: str):
    """New bag containing only /trajectory_cloud."""
    if os.path.exists(output_dir):
        print(f"ERROR: output already exists: {output_dir}")
        sys.exit(1)

    writer = _open_writer(output_dir)
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0, name=CLOUD_TOPIC, type=CLOUD_TYPE, serialization_format="cdr",
    ))
    cloud_msg, ts_ns = build_pointcloud2(points, frame_id)
    writer.write(CLOUD_TOPIC, serialize_message(cloud_msg), ts_ns)
    del writer

    print(f"Wrote {len(points)} points → {output_dir}/")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("bag_path", help="Input bag directory or .mcap/.db3 file")
    parser.add_argument("--topic", default="/vrpn_mocap/Brusilica/pose")
    parser.add_argument("--output", default=None,
                        help="Output directory (default: <bag_dir>/pose_cloud)")

    args = parser.parse_args()

    bag_path = os.path.abspath(args.bag_path)
    if not os.path.exists(bag_path):
        print(f"ERROR: input bag does not exist: {bag_path}")
        sys.exit(1)

    storage_id = "mcap"
    print(f"Reading '{args.topic}' from {bag_path} …")

    frame_id, points = read_poses(bag_path, args.topic, storage_id)
    print(f"Found {len(points)} pose messages.  frame: '{frame_id}'")

    if not points:
        print("No messages found — nothing to write.")
        sys.exit(0)

   
    output_dir = os.path.abspath(
        args.output if args.output else
        os.path.join(os.path.dirname(bag_path), "pose_cloud")
    )
    write_point_cloud_bag(points, frame_id, output_dir)

    print("Done.")


if __name__ == "__main__":
    main()
