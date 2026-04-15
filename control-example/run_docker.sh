xhost +local:docker
docker run -it \
  --rm \
  --name control-example-node \
  --network host \
  control-example:latest
