xhost +local:docker
docker run -it --init \
  --rm \
  --name control-example-node \
  --network host \
  --device /dev/video0 \
  --device /dev/video2 \
  control-example:latest
