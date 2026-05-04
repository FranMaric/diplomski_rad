FROM osrf/ros:noetic-desktop-full-focal

ENV DISABLE_ROS1_EOL_WARNINGS=1
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-noetic-catkin \
	ros-noetic-franka-gripper \
	python3-pip \
	&& rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip setuptools wheel

WORKDIR /catkin_ws/src

COPY ./openpi-client ./openpi-client

RUN cd openpi-client && pip install -e .

RUN source /opt/ros/noetic/setup.bash && catkin_create_pkg robot_control_pkg rospy geometry_msgs franka_gripper

COPY *.py /catkin_ws/src/robot_control_pkg/scripts/

RUN chmod +x /catkin_ws/src/robot_control_pkg/scripts/*.py

WORKDIR /catkin_ws

RUN source /opt/ros/noetic/setup.bash && catkin_make

RUN pip3 install typing_extensions

CMD ["bash", "-lc", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun robot_control_pkg joint_velocity_control.py"]
