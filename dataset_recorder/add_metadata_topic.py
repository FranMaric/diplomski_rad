#!/usr/bin/env python3
"""
Add (or overwrite) a latched /metadata topic to an MCAP bag.

Two-phase workflow
------------------
Phase 1 – init (default, no --write flag):
  Reads the .mcap, extracts the bag start timestamp, and writes a
  <stem>_states.txt file next to the .mcap for you to edit.
  The .mcap is NOT modified.

Phase 2 – write (--write flag):
  Reads the edited <stem>_states.txt, injects one std_msgs/msg/String
  message per state transition into /metadata, and produces a new bag.
  All original topics are copied byte-for-byte unchanged.
  The /metadata topic is stored with transient-local (latching) QoS so
  only N messages are written — one per transition, not one per frame.
  If a /metadata topic already exists in the input it is replaced.

Usage
-----
  python add_metadata_topic.py <bag_dir_or_mcap>               # init
  python add_metadata_topic.py <bag_dir_or_mcap> --write       # write
  python add_metadata_topic.py <bag_dir_or_mcap> --write -o <out_dir>
"""

import argparse
import json
import re
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message
from std_msgs.msg import String

METADATA_TOPIC = "/metadata"
METADATA_TYPE  = "std_msgs/msg/String"

# Transient-local (latching) QoS profile stored in the bag metadata.
# Subscribers with matching durability will receive the last message on connect.
LATCHED_QOS = """\
- history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
  deadline:
    sec: 2147483647
    nsec: 4294967295
  lifespan:
    sec: 2147483647
    nsec: 4294967295
  liveliness: automatic
  liveliness_lease_duration:
    sec: 2147483647
    nsec: 4294967295
  avoid_ros_namespace_conventions: false
"""

# ── helpers ────────────────────────────────────────────────────────────────────

def find_mcap(path: Path) -> Path:
    if path.is_file() and path.suffix == ".mcap":
        return path
    if path.is_dir():
        mcaps = sorted(path.glob("*.mcap"))
        if mcaps:
            return mcaps[0]
    raise FileNotFoundError(f"No .mcap file found at: {path}")


def ns_to_datetime_str(ts_ns: int) -> str:
    """Nanosecond timestamp → 'YYYY-MM-DD HH:MM:SS.mmm' (UTC, ms precision)."""
    dt = datetime.fromtimestamp(ts_ns / 1e9, tz=timezone.utc)
    return dt.strftime("%Y-%m-%d %H:%M:%S.") + f"{dt.microsecond // 1000:03d}"


def datetime_str_to_ns(s: str) -> int:
    """'YYYY-MM-DD HH:MM:SS.mmm[mmm]' (UTC) → nanoseconds."""
    s = s.strip()
    # strptime %f handles 1-6 fractional digits, padding right with zeros
    dt = datetime.strptime(s, "%Y-%m-%d %H:%M:%S.%f")
    dt = dt.replace(tzinfo=timezone.utc)
    return int(dt.timestamp() * 1e9)


def states_file_path(mcap_path: Path) -> Path:
    return mcap_path.parent / f"{mcap_path.stem}_states.txt"


# ── states-file parsing ────────────────────────────────────────────────────────

# Accepts single or double-digit hours, 1–6 fractional second digits
LINE_RE = re.compile(
    r"timestamp:\s*(\d{4}-\d{2}-\d{2}\s+\d{1,2}:\d{2}:\d{2}\.\d+)\s+state:\s*(\S+)"
)


def parse_states_file(path: Path) -> list[tuple[int, str]]:
    """Return [(ts_ns, state), ...] sorted ascending by timestamp."""
    entries = []
    with open(path) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            m = LINE_RE.match(line)
            if not m:
                print(
                    f"WARNING: line {lineno} skipped (unexpected format): {line!r}",
                    file=sys.stderr,
                )
                continue
            entries.append((datetime_str_to_ns(m.group(1)), m.group(2)))
    if not entries:
        raise ValueError(f"No valid state entries found in {path}")
    entries.sort(key=lambda x: x[0])
    return entries


# ── phase 1: init ──────────────────────────────────────────────────────────────

def cmd_init(mcap_in: Path) -> None:
    states_file = states_file_path(mcap_in)
    if states_file.exists():
        print(f"States file already exists, not overwriting: {states_file}")
        print("Delete it and re-run to regenerate, or edit it directly.")
        return

    stor_r = rosbag2_py.StorageOptions(uri=str(mcap_in), storage_id="mcap")
    conv   = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(stor_r, conv)

    first_ts = None
    while reader.has_next():
        _, _, ts = reader.read_next()
        first_ts = ts
        break
    del reader

    if first_ts is None:
        print("ERROR: .mcap appears to be empty.", file=sys.stderr)
        sys.exit(1)

    t0_str = ns_to_datetime_str(first_ts)
    # Example transitions spaced ~3 s apart so the user has realistic entries to edit
    t1_str = ns_to_datetime_str(first_ts + 3_000_000_000)
    t2_str = ns_to_datetime_str(first_ts + 6_000_000_000)
    t3_str = ns_to_datetime_str(first_ts + 9_000_000_000)

    lines = [
        f"timestamp: {t0_str} state: approach",
        f"timestamp: {t1_str} state: contact",
        f"timestamp: {t2_str} state: retraction",
        f"timestamp: {t3_str} state: approach",
    ]

    states_file.write_text("\n".join(lines) + "\n")
    print(f"Created: {states_file}")
    print("Edit the timestamps and states, then re-run with --write.")
    print("Valid states: approach, contact, retraction")


# ── stats JSON accumulator ─────────────────────────────────────────────────────

def _stats_json_path(report_path: Path) -> Path:
    return report_path.parent / "states_stats.json"


def _append_episode_stats(report_path: Path, episode: str, durations: dict[str, int], first_state: str) -> None:
    sjson = _stats_json_path(report_path)
    existing: list = []
    if sjson.exists():
        try:
            existing = json.loads(sjson.read_text())
        except json.JSONDecodeError:
            existing = []
    existing.append({"episode": episode, "first_state": first_state, "durations": durations})
    sjson.write_text(json.dumps(existing, indent=2))


# ── state duration report ──────────────────────────────────────────────────────

def compute_durations(transitions: list[tuple[int, str]], bag_end_ns: int) -> dict[str, int]:
    """Return {state: total_ns} for each state across all active intervals."""
    durations: dict[str, int] = {}
    for i, (ts, state) in enumerate(transitions):
        next_ts = transitions[i + 1][0] if i + 1 < len(transitions) else bag_end_ns
        dur = max(0, next_ts - ts)
        durations[state] = durations.get(state, 0) + dur
    return durations


def format_duration_lines(durations: dict[str, int]) -> list[str]:
    total_ns = sum(durations.values())
    col = max(len(s) for s in durations)
    lines = []
    for state in ["approach", "contact", "retraction"] + [
        s for s in durations if s not in ("approach", "contact", "retraction")
    ]:
        if state not in durations:
            continue
        dur_s = durations[state] / 1e9
        pct   = 100.0 * durations[state] / total_ns if total_ns else 0.0
        lines.append(f"  {state:<{col}}  {dur_s:>8.3f} s  ({pct:5.1f}%)")
    return lines


def format_report_block(label: str, durations: dict[str, int], first_state: str) -> str:
    total_s = sum(durations.values()) / 1e9
    lines = [f"=== {label} ===", f"Duration: {total_s:.3f} s", f"First state: {first_state}"]
    lines += format_duration_lines(durations)
    lines.append("")
    return "\n".join(lines)


# ── phase 2: write ─────────────────────────────────────────────────────────────

def cmd_write(mcap_in: Path, out_dir: Path, report_path: Path | None) -> None:
    states_file = states_file_path(mcap_in)
    if not states_file.exists():
        print(f"ERROR: states file not found: {states_file}", file=sys.stderr)
        print("Run without --write first to generate it.", file=sys.stderr)
        sys.exit(1)

    transitions = parse_states_file(states_file)
    print(f"Loaded {len(transitions)} state transition(s) from {states_file.name}:")
    for ts_ns, state in transitions:
        print(f"  {ns_to_datetime_str(ts_ns)}  →  {state}")

    # ── read all messages ──────────────────────────────────────────────────────
    print(f"\nReading {mcap_in} ...")
    stor_r = rosbag2_py.StorageOptions(uri=str(mcap_in), storage_id="mcap")
    conv   = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(stor_r, conv)

    topic_infos: list[rosbag2_py.TopicMetadata] = reader.get_all_topics_and_types()
    type_map: dict[str, str] = {ti.name: ti.type for ti in topic_infos}
    qos_map:  dict[str, str] = {ti.name: ti.offered_qos_profiles for ti in topic_infos}

    raw_msgs: list[tuple[str, bytes, int]] = []
    while reader.has_next():
        topic, data, ts = reader.read_next()
        raw_msgs.append((topic, data, ts))
    del reader

    n_original = len(raw_msgs)
    print(f"Read {n_original} messages across {len(type_map)} topic(s).")

    # ── strip any existing /metadata (topic + messages) ───────────────────────
    if METADATA_TOPIC in type_map:
        print(f"Removing existing {METADATA_TOPIC} topic ({sum(1 for t,_,_ in raw_msgs if t == METADATA_TOPIC)} msg(s)).")
        type_map.pop(METADATA_TOPIC)
        qos_map.pop(METADATA_TOPIC)
        raw_msgs = [(t, d, ts) for t, d, ts in raw_msgs if t != METADATA_TOPIC]

    # bag end = last message timestamp among original (non-metadata) messages
    bag_end_ns = max(ts for _, _, ts in raw_msgs)

    # ── state duration report ──────────────────────────────────────────────────
    durations   = compute_durations(transitions, bag_end_ns)
    first_state = transitions[0][1]
    report_block = format_report_block(mcap_in.parent.name, durations, first_state)
    print()
    print(report_block)

    if report_path is not None:
        report_path.parent.mkdir(parents=True, exist_ok=True)
        with open(report_path, "a") as f:
            f.write(report_block + "\n")
        _append_episode_stats(report_path, mcap_in.parent.name, durations, first_state)

    # ── inject /metadata messages ─────────────────────────────────────────────
    for ts_ns, state in transitions:
        msg = String()
        msg.data = state
        raw_msgs.append((METADATA_TOPIC, serialize_message(msg), ts_ns))

    raw_msgs.sort(key=lambda x: x[2])

    # ── write ──────────────────────────────────────────────────────────────────
    if out_dir.exists():
        print(f"Removing existing output: {out_dir}")
        shutil.rmtree(out_dir)

    total = len(raw_msgs)
    print(f"Writing {out_dir} ({total} messages) ...")

    stor_w = rosbag2_py.StorageOptions(uri=str(out_dir), storage_id="mcap")
    writer = rosbag2_py.SequentialWriter()
    writer.open(stor_w, conv)

    for topic, type_str in type_map.items():
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=topic,
                type=type_str,
                serialization_format="cdr",
                offered_qos_profiles=qos_map[topic],
            )
        )

    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=METADATA_TOPIC,
            type=METADATA_TYPE,
            serialization_format="cdr",
            offered_qos_profiles=LATCHED_QOS,
        )
    )

    for topic, data, ts_ns in raw_msgs:
        writer.write(topic, data, ts_ns)

    del writer
    print("Done.")


# ── summary across all episodes ───────────────────────────────────────────────

def cmd_summarize(report_path: Path) -> None:
    sjson = _stats_json_path(report_path)
    if not sjson.exists():
        print(f"ERROR: stats JSON not found: {sjson}", file=sys.stderr)
        print("Run --write on episodes first to accumulate stats.", file=sys.stderr)
        sys.exit(1)

    all_stats: list[dict] = json.loads(sjson.read_text())
    if not all_stats:
        print("ERROR: stats JSON is empty.", file=sys.stderr)
        sys.exit(1)

    totals: dict[str, int] = {}
    first_counts: dict[str, int] = {}
    for ep in all_stats:
        for state, ns in ep["durations"].items():
            totals[state] = totals.get(state, 0) + ns
        fs = ep.get("first_state", "unknown")
        first_counts[fs] = first_counts.get(fs, 0) + 1

    total_ns = sum(totals.values())
    n_eps    = len(all_stats)

    lines = [
        f"=== SUMMARY — {n_eps} episode(s) ===",
        f"Total annotated duration: {total_ns / 1e9:.3f} s",
    ]
    lines += format_duration_lines(totals)
    lines.append("First state per episode:")
    col2 = max(len(s) for s in first_counts)
    for state, count in sorted(first_counts.items(), key=lambda x: -x[1]):
        pct = 100.0 * count / n_eps
        lines.append(f"  {state:<{col2}}  {count:>3} episode(s)  ({pct:5.1f}%)")
    lines.append("")
    block = "\n".join(lines) + "\n"

    print(block)
    with open(report_path, "a") as f:
        f.write(block)
    print(f"Summary appended to {report_path}")


# ── entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Add a latched /metadata topic to an MCAP bag."
    )
    parser.add_argument(
        "input",
        type=Path,
        nargs="?",
        default=None,
        help="Path to a bag directory or .mcap file (not required for --summarize).",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="Write mode: inject /metadata messages from the edited states file.",
    )
    parser.add_argument(
        "--summarize",
        action="store_true",
        help="Append a total summary block to the report (requires --report).",
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=None,
        help=(
            "Output bag directory (write mode only). "
            "Default: <input_parent>/<stem>_metadata"
        ),
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=None,
        help="Append state duration stats to this file (write/summarize modes).",
    )
    args = parser.parse_args()

    if args.summarize:
        if args.report is None:
            parser.error("--summarize requires --report")
        cmd_summarize(args.report.resolve())
        return

    if args.input is None:
        parser.error("input path is required")

    mcap_in = find_mcap(args.input.resolve())

    if not args.write:
        cmd_init(mcap_in)
    else:
        out_dir = (
            args.output.resolve()
            if args.output is not None
            else mcap_in.parent / f"{mcap_in.stem}_metadata"
        )
        report_path = args.report.resolve() if args.report is not None else None
        cmd_write(mcap_in, out_dir, report_path)


if __name__ == "__main__":
    main()
