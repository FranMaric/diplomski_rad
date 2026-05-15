#!/usr/bin/env python3
import os
import subprocess
import sys
import termios
import time
import tty

TOPICS = [
    "/optoforce_0",
    # "/optoforce_0/wrench_filtered",
    # "/force_estimation",
    "/vrpn_mocap/Brusilica/pose",
    "/vrpn_mocap/Kalup/pose",
    "/camera_info",
    "/image_raw",
    "/camera/camera/color/image_raw",
    "/camera/camera/color/camera_info",
    # "/kalup_cep",
]

BAGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bags")


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
        ["ros2", "bag", "record", "-o", bag_path]
        + TOPICS
    )
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "6"

    print(f"Recording episode_{episode_num} -> {bag_path}")
    proc = subprocess.Popen(cmd, env=env)
    return proc


def check_topics():
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "6"
    print("Checking available topics...")
    try:
        result = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True, env=env, timeout=10)
    except subprocess.TimeoutExpired:
        print("ERROR: 'ros2 topic list' timed out. Is ROS 2 running?")
        sys.exit(1)

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
        proc.terminate()
        proc.wait()
        duration = time.monotonic() - t_start
        total_duration += duration
        print(f"episode_{episode} saved.  Duration: {duration:.1f}s  |  Total: {total_duration:.1f}s")
        episode += 1


if __name__ == "__main__":
    main()
