ros2 bag record -o ./bags/bag_$(date +%F_%H-%M-%S) \
	/optoforce_0 \
	/optoforce_0/wrench_filtered \
	/force_estimation \
	/vrpn_mocap/Brusilica/pose
