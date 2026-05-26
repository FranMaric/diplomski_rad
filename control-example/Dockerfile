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

# Copy optoforce catkin package (the inner directory is the actual catkin package)
COPY ./optoforce/optoforce /catkin_ws/src/optoforce

WORKDIR /catkin_ws

RUN source /opt/ros/noetic/setup.bash && catkin_make

RUN pip3 install typing_extensions pyserial

RUN mkdir -p /catkin_ws/src/robot_control_pkg/launch
COPY ./launch/robot_control.launch /catkin_ws/src/robot_control_pkg/launch/

CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && roslaunch robot_control_pkg robot_control.launch"]
