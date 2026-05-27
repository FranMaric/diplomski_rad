xhost +local:docker
docker run -it --init \
  --rm \
  --name control-example-node \
  --network host \
  -v /home/rf-blue/Desktop/fran_ws/control-example/bags:/bags \
  --device /dev/video0 \
  --device /dev/video2 \
  --device /dev/ttyACM0 \
  control-example:latest
