#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import termios
import time
import tty

"""
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p camera_name:=wrist_camera --remap __ns:=/wrist_camera
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2 -p camera_name:=scene_camera --remap __ns:=/scene_camera
ros2 run optoforce_wrapper optoforce_node
ros2 launch dataset_recorder/mocap_with_rotation.launch.py
ros2 launch foxglove_bridge foxglove_bridge_launch.xml 
"""

TOPICS = [
    "/optoforce_0",
    # "/optoforce_0/wrench_filtered",
    # "/force_estimation",
    "/vrpn_mocap_rotated/Brusilica/pose",
    "/vrpn_mocap_rotated/Kalup/pose",
    "/scene_camera/image_raw",
    "/scene_camera/camera_info",
    "/wrist_camera/image_raw",
    "/wrist_camera/camera_info",
    # "/kalup_cep",
]

BAGS_DIR_NAME = "kalup_test_bags"
BAGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), BAGS_DIR_NAME)

def reset_terminal():
    fd = sys.stdout.fileno()
    attrs = termios.tcgetattr(fd)
    attrs[1] |= termios.OPOST  # re-enable \n -> \r\n translation
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def wait_for_space(prompt):
    print(prompt, end="", flush=True)
    while True:
        ch = getch()
        if ch == " ":
            print()
            return
        if ch in ("\x03", "\x1b"):  # Ctrl+C or Escape
            print("\nAborted.")
            sys.exit(0)


def record_episode(episode_num):
    bag_path = os.path.join(BAGS_DIR, f"episode_{episode_num}")
    cmd = (
        ["ros2", "bag", "record", 
         "-o", bag_path,
         "--max-cache-size", "524288000"]
        + TOPICS
    )
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "6"

    print(f"Recording episode_{episode_num} -> {bag_path}")
    proc = subprocess.Popen(cmd, env=env, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return proc


def check_topics():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "6"
    print("Checking available topics...")
    try:
        result = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, env=env, timeout=10, stdin=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        reset_terminal()
        print("ERROR: 'ros2 topic list' timed out. Is ROS 2 running?")
        sys.exit(1)

    reset_terminal()
    available = set(result.stdout.splitlines())
    missing = [t for t in TOPICS if t not in available]

    if missing:
        print("ERROR: The following topics are not available:")
        for t in missing:
            print(f"  {t}")
        sys.exit(1)

    print("All topics available.")


def main():
    os.makedirs(BAGS_DIR, exist_ok=True)
    check_topics()
    episode = 1

    print("Episode recorder ready. Press SPACE to start/stop episodes, ESC/Ctrl+C to quit.")

    total_duration = 0.0

    while True:
        wait_for_space(f"\nPress SPACE to start episode_{episode}...")
        proc = record_episode(episode)
        t_start = time.monotonic()
        wait_for_space(f"Recording episode_{episode}. Press SPACE to stop...")
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        reset_terminal()
        duration = time.monotonic() - t_start
        total_duration += duration
        print(f"episode_{episode} saved.  Duration: {duration:.1f}s  |  Total: {total_duration:.1f}s")
        episode += 1


if __name__ == "__main__":
    main()
