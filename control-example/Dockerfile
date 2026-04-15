FROM osrf/ros:noetic-desktop-full-focal

ENV DISABLE_ROS1_EOL_WARNINGS=1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-noetic-catkin \
	ros-noetic-franka-gripper \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws/src

RUN source /opt/ros/noetic/setup.bash && catkin_create_pkg robot_control_pkg rospy geometry_msgs franka_gripper

COPY robot_control.py /catkin_ws/src/robot_control_pkg/scripts/robot_control.py

RUN chmod +x /catkin_ws/src/robot_control_pkg/scripts/robot_control.py

WORKDIR /catkin_ws

RUN source /opt/ros/noetic/setup.bash && catkin_make

CMD ["bash", "-lc", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun robot_control_pkg robot_control.py"]
