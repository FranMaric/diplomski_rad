#!/usr/bin/env python3
"""
Manages metadata for ROS bag files via a latched /bag_metadata topic (std_msgs/String, JSON).

Usage:
  python3 bag_metadata.py generate  -- scan bags dir, create/update metadata.json with defaults
  python3 bag_metadata.py write     -- read metadata.json, write to each bag
  python3 bag_metadata.py           -- auto: generate if no JSON exists, write otherwise
"""

import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

METADATA_TOPIC = "/bag_metadata"

DEFAULT_METADATA = {
    "model": "all_forces/40000",
    "TemporalEnsembler": True,
}


def find_bags_dir():
    # Docker mount takes priority
    if Path("/bags").is_dir():
        return Path("/bags")
    local = Path(__file__).parent / "bags"
    if local.is_dir():
        return local
    raise RuntimeError("Bags directory not found. Expected /bags (Docker) or ./bags (local).")


def generate(bags_dir: Path):
    metadata_json = bags_dir / "metadata.json"
    bag_files = sorted(bags_dir.glob("*.bag"))

    existing = {}
    if metadata_json.exists():
        with open(metadata_json) as f:
            existing = json.load(f)

    updated = dict(existing)
    new_bags = []
    for bag in bag_files:
        if bag.name not in updated:
            updated[bag.name] = dict(DEFAULT_METADATA)
            new_bags.append(bag.name)

    with open(metadata_json, "w") as f:
        json.dump(updated, f, indent=2)

    print(f"Found {len(bag_files)} bags, {len(new_bags)} new entries added.")
    if new_bags:
        for name in new_bags:
            print(f"  + {name}")
    print(f"Metadata file: {metadata_json}")


def _write_latched(bag, topic, msg, t):
    from std_msgs.msg import String
    connection_header = {
        "topic": topic,
        "type": String._type,
        "md5sum": String._md5sum,
        "message_definition": String._full_text,
        "latching": "1",
    }
    bag.write(topic, msg, t, connection_header=connection_header)


def write(bags_dir: Path):
    import rosbag
    from std_msgs.msg import String

    metadata_json = bags_dir / "metadata.json"
    if not metadata_json.exists():
        print(f"ERROR: {metadata_json} not found. Run 'generate' first.")
        sys.exit(1)

    with open(metadata_json) as f:
        metadata = json.load(f)

    total = len(metadata)
    for i, (bag_name, bag_meta) in enumerate(metadata.items(), 1):
        prefix = f"[{i}/{total}]"
        bag_path = bags_dir / bag_name
        if not bag_path.exists():
            print(f"{prefix} SKIP  {bag_name} (file not found)")
            continue

        try:
            with rosbag.Bag(str(bag_path)) as inbag:
                topics = inbag.get_type_and_topic_info().topics
                has_metadata = METADATA_TOPIC in topics
                start_time = inbag.get_start_time()

            import rospy
            t = rospy.Time.from_sec(start_time)
            meta_msg = String(data=json.dumps(bag_meta))

            if not has_metadata:
                # Fast path: just append
                with rosbag.Bag(str(bag_path), "a") as outbag:
                    _write_latched(outbag, METADATA_TOPIC, meta_msg, t)
                print(f"{prefix} OK    {bag_name} (append)")
            else:
                # Full rewrite: copy everything except old /bag_metadata, add new one
                tmp = bag_path.with_suffix(".tmp.bag")
                try:
                    with rosbag.Bag(str(tmp), "w") as outbag:
                        _write_latched(outbag, METADATA_TOPIC, meta_msg, t)
                        with rosbag.Bag(str(bag_path)) as inbag:
                            for topic, msg, t_msg in inbag.read_messages(raw=True):
                                if topic != METADATA_TOPIC:
                                    outbag.write(topic, msg, t_msg, raw=True)
                    shutil.move(str(tmp), str(bag_path))
                    print(f"{prefix} OK    {bag_name} (overwrite)")
                except Exception:
                    if tmp.exists():
                        tmp.unlink()
                    raise

        except Exception as e:
            print(f"{prefix} ERROR {bag_name}: {e}")


def main():
    bags_dir = find_bags_dir()

    metadata_json = bags_dir / "metadata.json"
    if len(sys.argv) < 2:
        mode = "generate" if not metadata_json.exists() else "write"
    else:
        mode = sys.argv[1]

    if mode == "generate":
        generate(bags_dir)
    elif mode == "write":
        write(bags_dir)
    else:
        print(f"Unknown mode: {mode!r}. Use 'generate' or 'write'.")
        sys.exit(1)


if __name__ == "__main__":
    main()
