#!/bin/bash

source /opt/ros/noetic/setup.bash

BAG_NAME="/bags/inference_$(date +%Y%m%d_%H%M%S)"

rosbag record \
  /joint_states \
  /ee_camera/image_raw \
  /scene_camera/image_raw \
  /cartesian_impedance_controller/equilibrium_pose \
  /optoforce_0 \
  /tf \
  /tf_static \
  /action_chunk_vis \
  -O "$BAG_NAME"
