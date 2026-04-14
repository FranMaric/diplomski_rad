xhost +local:docker
docker run -d -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged \
  --network host \
  --cap-add=SYS_NICE \
  --cap-add=IPC_LOCK \
  --ulimit rtprio=99 \
  --ulimit memlock=-1 \
  -v /dev/shm:/dev/shm \
  fran/ros-noetic:latest
