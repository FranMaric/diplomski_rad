xhost +local:docker
docker run -it \
  --rm \
  --name fran-ros-noetic \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged \
  --network host \
  --cap-add=SYS_NICE \
  --cap-add=IPC_LOCK \
  --ulimit rtprio=99 \
  --ulimit memlock=-1 \
  -v /dev/shm:/dev/shm \
  -v .:/root/catkin_ws/src/franka_ros \
  -v franka_build_cache:/root/catkin_ws/build \
  -v franka_devel_cache:/root/catkin_ws/devel \
  fran/ros-noetic:latest
