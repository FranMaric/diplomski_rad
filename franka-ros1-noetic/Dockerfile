FROM osrf/ros:noetic-desktop-full-focal

ENV DISABLE_ROS1_EOL_WARNINGS=1

RUN apt update && apt install -y \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-controller-manager \
    ros-noetic-hardware-interface \
    ros-noetic-transmission-interface \
    ros-noetic-combined-robot-hw \
    ros-noetic-boost-sml \
    git \
    iputils-ping

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

RUN mkdir -p /root/catkin_ws/src

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make"

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws/build && \
    cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel"

RUN cd /root/catkin_ws/src && git clone https://github.com/frankarobotics/franka_ros -b noetic-devel

COPY ./libfranka-0.8.0-amd64.deb .
RUN dpkg -i libfranka-0.8.0-amd64.deb

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make"

RUN cd /root/catkin_ws/src && catkin_create_pkg my_custom_launcher
COPY ./my_launch_file.launch /root/catkin_ws/src/my_custom_launcher/
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make --pkg my_custom_launcher"

CMD bash -c "ulimit -r 99 && ulimit -l unlimited && \
             source /root/catkin_ws/devel/setup.bash && \                
             roslaunch my_custom_launcher \
             my_launch_file.launch \
             robot_ip:=172.16.0.2 \
             sim:=false"