"""
mcap_to_lerobot.py
------------------
Converts ROS 2 Jazzy .mcap recordings to LeRobotDataset (codebase_version=v2.1).

Assumption: all topics are published at the same 30 Hz — no synchronisation needed.
Messages are consumed in arrival order and zipped by position.

Topics expected:
  /tool_center_pose              -> geometry_msgs/PoseStamped
  /optoforce_0                   -> geometry_msgs/WrenchStamped
  /camera/camera/color/image_raw -> sensor_msgs/Image  (scene camera)
  /image_raw                     -> sensor_msgs/Image  (wrist camera)

Usage
-----
    # Convert all episodes then write metadata
    python mcap_to_lerobot.py --bags-dir dataset_recorder/bags --repo-id fran/force_vla

    # Regenerate stats + info.json from existing parquets (no re-encoding)
    python mcap_to_lerobot.py --bags-dir dataset_recorder/bags --repo-id fran/force_vla --fix-stats

Dependencies
------------
    pip install rosbags opencv-python numpy pyarrow datasets
    ffmpeg must be on PATH
"""

from __future__ import annotations

import json
import math
import subprocess
from collections import defaultdict
from pathlib import Path

import cv2
import datasets as hf_datasets
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CODEBASE_VERSION = "v2.1"

POSE_TOPIC  = "/tool_center_pose"
FORCE_TOPIC = "/optoforce_0"
SCENE_TOPIC = "/camera/camera/color/image_raw"
WRIST_TOPIC = "/image_raw"
WANTED_TOPICS = {POSE_TOPIC, FORCE_TOPIC, SCENE_TOPIC, WRIST_TOPIC}

STATE_DIM  = 13
ACTION_DIM = 7

# HuggingFace `datasets` feature spec — must match what load_hf_dataset() expects.
# Uses Sequence(Value("float32")) which serialises as fixed_size_list in parquet.
HF_FEATURES = hf_datasets.Features({
    "observation.state": hf_datasets.Sequence(
        hf_datasets.Value("float32")),
    "action": hf_datasets.Sequence(
        hf_datasets.Value("float32")),
    "timestamp":     hf_datasets.Value("float32"),
    "frame_index":   hf_datasets.Value("int64"),
    "episode_index": hf_datasets.Value("int64"),
    "index":         hf_datasets.Value("int64"),
    "task_index":    hf_datasets.Value("int64"),
})


# ===========================================================================
# Helpers
# ===========================================================================

def _quat_to_euler(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _ros_image_to_bgr(msg) -> np.ndarray | None:
    enc = msg.encoding.lower()
    h, w = msg.height, msg.width
    raw = bytes(msg.data)
    if enc in ("rgb8", "rgb"):
        return cv2.cvtColor(np.frombuffer(raw, np.uint8).reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if enc in ("bgr8", "bgr"):
        return np.frombuffer(raw, np.uint8).reshape(h, w, 3)
    if enc == "mono8":
        return cv2.cvtColor(np.frombuffer(raw, np.uint8).reshape(h, w), cv2.COLOR_GRAY2BGR)
    if enc in ("16uc1", "mono16"):
        return cv2.cvtColor((np.frombuffer(raw, np.uint16).reshape(h, w) >> 8).astype(np.uint8),
                            cv2.COLOR_GRAY2BGR)
    if enc == "bayer_rggb8":
        return cv2.cvtColor(np.frombuffer(raw, np.uint8).reshape(h, w), cv2.COLOR_BayerRG2BGR)
    print(f"[WARN] Unsupported encoding: {enc!r}")
    return None


def _pose_to_vec(msg) -> list[float]:
    p = msg.pose.position
    o = msg.pose.orientation
    rx, ry, rz = _quat_to_euler(o.x, o.y, o.z, o.w)
    return [p.x, p.y, p.z, rx, ry, rz]


def _force_to_vec(msg) -> list[float]:
    wrench = getattr(msg, "wrench", msg)
    f, t = wrench.force, wrench.torque
    return [f.x, f.y, f.z, t.x, t.y, t.z]


def _stat(arr: np.ndarray) -> dict:
    """Per-feature stats with the `count` scalar LeRobot's aggregate_stats() requires."""
    return {
        "mean":  arr.mean(0).tolist(),
        "std":   arr.std(0).tolist(),
        "min":   arr.min(0).tolist(),
        "max":   arr.max(0).tolist(),
        "count": [int(arr.shape[0])],   # shape (1,) — a single frame count
    }


# ===========================================================================
# Main converter
# ===========================================================================

class McapToLeRobot:
    """
    Converts one .mcap file (= one episode) into a LeRobotDataset v2.1 directory.

    Multi-episode workflow
    ----------------------
        for i, path in enumerate(mcap_files):
            McapToLeRobot(path, "dataset/").convert(episode_index=i)
        McapToLeRobot.finalise("dataset/", fps=30, task_prompt="sand the mold")

    Or use the convenience wrapper:
        convert_mcap_list(mcap_files, "dataset/")
    """

    def __init__(
        self,
        mcap_path: str | Path,
        output_dir: str | Path,
        task_prompt: str = "sand the mold",
        fps: int = 30,
        target_h: int = 480,
        target_w: int = 640,
        video_codec: str = "libx264",
        video_crf: int = 18,
    ):
        self.mcap_path   = Path(mcap_path)
        self.output_dir  = Path(output_dir)
        self.task_prompt = task_prompt
        self.fps         = fps
        self.target_h    = target_h
        self.target_w    = target_w
        self.video_codec = video_codec
        self.video_crf   = video_crf
        self.output_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    def convert(self, episode_index: int = 0) -> None:
        print(f"[INFO] Reading {self.mcap_path} …")
        buffers = self._read_topics()

        n_frames = min(len(b) for b in buffers.values())
        if n_frames == 0:
            raise ValueError("No messages found for one or more required topics.")
        print(f"[INFO] {n_frames} frames (shortest topic buffer)")

        self._write_episode(
            pose_msgs   = buffers[POSE_TOPIC][:n_frames],
            force_msgs  = buffers[FORCE_TOPIC][:n_frames],
            scene_msgs  = buffers[SCENE_TOPIC][:n_frames],
            wrist_msgs  = buffers[WRIST_TOPIC][:n_frames],
            episode_index = episode_index,
        )

    @staticmethod
    def finalise(
        output_dir: str | Path,
        fps: int,
        task_prompt: str,
        robot_type: str = "manipulator",
    ) -> None:
        """Write meta/info.json, stats.json, and re-sort jsonl files."""
        McapToLeRobot._write_meta(Path(output_dir), fps, task_prompt, robot_type)

    # ------------------------------------------------------------------
    # Reading
    # ------------------------------------------------------------------

    def _read_topics(self) -> dict[str, list]:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import Stores, get_typestore

        typestore = get_typestore(Stores.ROS2_HUMBLE)
        buffers: dict[str, list] = defaultdict(list)

        with Reader(self.mcap_path) as reader:
            available = {c.topic for c in reader.connections}
            missing   = WANTED_TOPICS - available
            if missing:
                raise RuntimeError(f"Topics missing from bag: {missing}")

            connections = [c for c in reader.connections if c.topic in WANTED_TOPICS]
            for conn, _ts_ns, rawdata in reader.messages(connections=connections):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                buffers[conn.topic].append(msg)

        for topic, buf in buffers.items():
            print(f"[INFO]  {topic}: {len(buf)} messages")
        return buffers

    # ------------------------------------------------------------------
    # Writing one episode
    # ------------------------------------------------------------------

    def _write_episode(self, pose_msgs, force_msgs, scene_msgs, wrist_msgs,
                       episode_index: int) -> None:
        n = len(pose_msgs)
        gi_path = self.output_dir / ".global_index"
        global_offset = int(gi_path.read_text()) if gi_path.exists() else 0

        rows: list[dict] = []
        scene_frames: list[np.ndarray] = []
        wrist_frames: list[np.ndarray] = []

        for i in range(n):
            pose_vec  = _pose_to_vec(pose_msgs[i])
            force_vec = _force_to_vec(force_msgs[i])
            state  = pose_vec + [0.0] + force_vec          # gripper = 0
            next_pose = _pose_to_vec(pose_msgs[i + 1]) if i + 1 < n else pose_vec
            action = next_pose + [0.0]                      # gripper = 0

            rows.append({
                "observation.state": state,
                "action":            action,
                "timestamp":         float(i) / self.fps,
                "frame_index":       i,
                "episode_index":     episode_index,
                "index":             global_offset + i,
                "task_index":        0,
            })

            bgr = _ros_image_to_bgr(scene_msgs[i])
            scene_frames.append(cv2.resize(
                bgr if bgr is not None else np.zeros((self.target_h, self.target_w, 3), np.uint8),
                (self.target_w, self.target_h)))

            wbgr = _ros_image_to_bgr(wrist_msgs[i])
            wrist_frames.append(cv2.resize(
                wbgr if wbgr is not None else np.zeros((self.target_h, self.target_w, 3), np.uint8),
                (self.target_w, self.target_h)))

        ep_str    = f"episode_{episode_index:06d}"
        chunk_dir = self.output_dir / "data" / "chunk-000"
        chunk_dir.mkdir(parents=True, exist_ok=True)

        # --- parquet (written via HF datasets to embed correct schema metadata) ---
        self._write_parquet(rows, chunk_dir / f"{ep_str}.parquet")
        print(f"[INFO] Wrote data/chunk-000/{ep_str}.parquet")

        # --- videos ---
        for cam_key, frames in [
            ("observation.image",       scene_frames),
            ("observation.wrist_image", wrist_frames),
        ]:
            vid_dir = self.output_dir / "videos" / "chunk-000" / cam_key
            vid_dir.mkdir(parents=True, exist_ok=True)
            self._write_video(frames, vid_dir / f"{ep_str}.mp4")
            print(f"[INFO] Wrote videos/chunk-000/{cam_key}/{ep_str}.mp4")

        # --- per-episode stats + meta (appended immediately) ---
        ep_states  = np.array([r["observation.state"] for r in rows], np.float32)
        ep_actions = np.array([r["action"]            for r in rows], np.float32)

        meta_dir = self.output_dir / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)

        with open(meta_dir / "episodes.jsonl", "a") as f:
            f.write(json.dumps({
                "episode_index": episode_index,
                "tasks":  [self.task_prompt],
                "length": n,
            }) + "\n")

        with open(meta_dir / "episodes_stats.jsonl", "a") as f:
            f.write(json.dumps({
                "episode_index": episode_index,
                "stats": {
                    "observation.state": _stat(ep_states),
                    "action":            _stat(ep_actions),
                },
            }) + "\n")

        gi_path.write_text(str(global_offset + n))
        print(f"[INFO] Appended episode {episode_index} to episodes.jsonl + episodes_stats.jsonl")

    # ------------------------------------------------------------------
    # Parquet — use HF datasets so the parquet footer has the correct
    # HuggingFace schema metadata that LeRobot's load_dataset() expects.
    # The key difference vs raw pyarrow: Sequence(Value("float32"), length=N)
    # serialises as fixed_size_list<float>[N], which is what the cast expects.
    # ------------------------------------------------------------------

    @staticmethod
    def _write_parquet(rows: list[dict], path: Path) -> None:
        hf_ds = hf_datasets.Dataset.from_dict(
            {
                "observation.state": [r["observation.state"] for r in rows],
                "action":            [r["action"]            for r in rows],
                "timestamp":         [r["timestamp"]         for r in rows],
                "frame_index":       [r["frame_index"]       for r in rows],
                "episode_index":     [r["episode_index"]     for r in rows],
                "index":             [r["index"]             for r in rows],
                "task_index":        [r["task_index"]        for r in rows],
            },
            features=HF_FEATURES,
        )
        hf_ds.to_parquet(str(path))

    # ------------------------------------------------------------------
    # Video — streamed to ffmpeg via stdin pipe
    # ------------------------------------------------------------------

    def _write_video(self, bgr_frames: list[np.ndarray], path: Path) -> None:
        if not bgr_frames:
            return
        h, w = bgr_frames[0].shape[:2]
        cmd = [
            "ffmpeg", "-y",
            "-f", "rawvideo", "-vcodec", "rawvideo",
            "-s", f"{w}x{h}", "-pix_fmt", "bgr24",
            "-r", str(self.fps), "-i", "pipe:0",
            "-vcodec", self.video_codec,
            "-crf", str(self.video_crf),
            "-pix_fmt", "yuv420p",
            "-movflags", "+faststart",
            str(path),
        ]
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for frame in bgr_frames:
            proc.stdin.write(frame.tobytes())
        proc.stdin.close()
        proc.wait()
        if proc.returncode != 0:
            raise RuntimeError(f"ffmpeg failed writing {path}")

    # ------------------------------------------------------------------
    # Meta — call once after all episodes are written
    # ------------------------------------------------------------------

    @staticmethod
    def _write_meta(output_dir: Path, fps: int, task_prompt: str,
                    robot_type: str) -> None:
        meta_dir = output_dir / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)

        parquet_files = sorted((output_dir / "data" / "chunk-000").glob("*.parquet"))
        if not parquet_files:
            raise FileNotFoundError("No parquet files found — run convert() first.")

        all_states:  list[np.ndarray] = []
        all_actions: list[np.ndarray] = []
        total_frames = 0

        for pf in parquet_files:
            tbl    = pq.read_table(pf)
            length = len(tbl)
            total_frames += length
            all_states.append(
                np.array([r.as_py() for r in tbl["observation.state"]], np.float32))
            all_actions.append(
                np.array([r.as_py() for r in tbl["action"]], np.float32))

        states  = np.concatenate(all_states,  axis=0)
        actions = np.concatenate(all_actions, axis=0)
        total_episodes = len(parquet_files)
        total_chunks   = math.ceil(total_episodes / 1000)

        info = {
            "codebase_version": CODEBASE_VERSION,
            "robot_type":       robot_type,
            "total_episodes":   total_episodes,
            "total_frames":     total_frames,
            "total_tasks":      1,
            "total_chunks":     total_chunks,
            "total_videos":     total_episodes * 2,   # 2 cameras × N episodes
            "chunks_size":      1000,
            "fps":              fps,
            "splits":           {"train": f"0:{total_episodes}"},
            "data_path":        "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path":       "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features": {
                "observation.state": {
                    "dtype": "float32",
                    "shape": [STATE_DIM],
                    "names": [["ee_x", "ee_y", "ee_z", "ee_rx", "ee_ry", "ee_rz",
                               "gripper", "fx", "fy", "fz", "tx", "ty", "tz"]],
                },
                "action": {
                    "dtype": "float32",
                    "shape": [ACTION_DIM],
                    "names": [["ee_x", "ee_y", "ee_z", "ee_rx", "ee_ry", "ee_rz", "gripper"]],
                },
                "observation.image": {
                    "dtype": "video",
                    "shape": [3, 480, 640],
                    "names": ["channels", "height", "width"],
                    "video_info": {
                        "video.fps":          fps,
                        "video.codec":        "av1",
                        "video.pix_fmt":      "yuv420p",
                        "video.is_depth_map": False,
                        "has_audio":          False,
                    },
                },
                "observation.wrist_image": {
                    "dtype": "video",
                    "shape": [3, 480, 640],
                    "names": ["channels", "height", "width"],
                    "video_info": {
                        "video.fps":          fps,
                        "video.codec":        "av1",
                        "video.pix_fmt":      "yuv420p",
                        "video.is_depth_map": False,
                        "has_audio":          False,
                    },
                },
                "timestamp":     {"dtype": "float32", "shape": [1], "names": None},
                "frame_index":   {"dtype": "int64",   "shape": [1], "names": None},
                "episode_index": {"dtype": "int64",   "shape": [1], "names": None},
                "index":         {"dtype": "int64",   "shape": [1], "names": None},
                "task_index":    {"dtype": "int64",   "shape": [1], "names": None},
            },
        }

        stats = {
            "observation.state": _stat(states),
            "action":            _stat(actions),
        }

        (meta_dir / "info.json").write_text(json.dumps(info, indent=2))
        (meta_dir / "stats.json").write_text(json.dumps(stats, indent=2))

        with open(meta_dir / "tasks.jsonl", "w") as f:
            f.write(json.dumps({"task_index": 0, "task": task_prompt}) + "\n")

        # Re-sort the incrementally-written jsonl files
        for fname in ("episodes.jsonl", "episodes_stats.jsonl"):
            fpath = meta_dir / fname
            if fpath.exists():
                lines = [json.loads(l) for l in fpath.read_text().splitlines() if l.strip()]
                lines.sort(key=lambda e: e["episode_index"])
                fpath.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

        print(f"[INFO] meta/ written ({total_episodes} episodes, {total_frames} frames total)")

# ===========================================================================
# Convenience wrapper
# ===========================================================================

def convert_mcap_list(
    mcap_paths: list[str | Path],
    output_dir: str | Path,
    task_prompt: str = "sand the mold",
    fps: int = 30,
    target_h: int = 480,
    target_w: int = 640,
    video_codec: str = "libx264",
    robot_type: str = "manipulator",
) -> None:
    output_dir = Path(output_dir)

    # Clear state files so a re-run starts clean
    for stale in (".global_index", "meta/episodes.jsonl", "meta/episodes_stats.jsonl"):
        p = output_dir / stale
        if p.exists():
            p.unlink()

    for ep_idx, mcap_path in enumerate(mcap_paths):
        print(f"\n{'='*60}\nEpisode {ep_idx}/{len(mcap_paths)-1}: {mcap_path}\n{'='*60}")
        McapToLeRobot(
            mcap_path=mcap_path, output_dir=output_dir,
            task_prompt=task_prompt, fps=fps,
            target_h=target_h, target_w=target_w,
            video_codec=video_codec,
        ).convert(episode_index=ep_idx)

    McapToLeRobot.finalise(output_dir=output_dir, fps=fps,
                           task_prompt=task_prompt, robot_type=robot_type)


# ===========================================================================
# CLI
# ===========================================================================

TASK_PROMPT = "sand the mold"
FPS         = 30
TARGET_H    = 480
TARGET_W    = 640
VIDEO_CODEC = "libx264"
ROBOT_TYPE  = "manipulator"

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


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Convert ROS 2 .mcap episodes to LeRobotDataset v2.1"
    )
    parser.add_argument("--bags-dir", type=Path, required=True,
                        help="Root bags directory (contains episode_N subdirs)")
    parser.add_argument("--repo-id",  type=str, default="fran/force_vla",
                        help="Dataset repo-id (used as output directory name)")
    args = parser.parse_args()

    output_dir = args.bags_dir.parent / args.repo_id.replace("/", "__")

    mcaps = _discover_mcaps(args.bags_dir)
    if not mcaps:
        raise SystemExit(
            f"No mcap files found under {args.bags_dir}\nPattern: {MCAP_GLOB}")
    convert_mcap_list(
        mcap_paths=mcaps, output_dir=output_dir,
        task_prompt=TASK_PROMPT, fps=FPS,
        target_h=TARGET_H, target_w=TARGET_W,
        video_codec=VIDEO_CODEC, robot_type=ROBOT_TYPE,
    )