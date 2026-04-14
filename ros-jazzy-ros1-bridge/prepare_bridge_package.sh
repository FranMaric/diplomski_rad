# build takes a long time
docker build . -t ros-jazzy-ros1-bridge-builder --network=host
docker run --rm ros-jazzy-ros1-bridge-builder | tar xvzf -

# simple demo to check if bridge is functional
# inside ros noeti container: `rosrun rospy_tutorials talker`
# run bridge with: ./run_bridge.sh
# on host ubuntu 24: ros2 run demo_nodes_cpp listener