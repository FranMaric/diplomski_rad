#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

docker run --rm \
  -v /home/rf-blue/Desktop/fran_ws/control-example/bags:/bags \
  -v "${SCRIPT_DIR}/bag_metadata.py":/bag_metadata.py \
  control-example:latest \
  /bin/bash -c "source /opt/ros/noetic/setup.bash && python3 /bag_metadata.py ${1:-}"
