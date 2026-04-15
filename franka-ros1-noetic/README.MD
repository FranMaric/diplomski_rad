### Primjeri tocaka
header: 
  frame_id: "panda_link0"
pose: 
  position: 
    x: 0.3110661948654686
    y: -0.31788369018220675
    z: 0.35673353367990346
  orientation: 
    x: -0.5564601896870426
    y: 0.7444760478714425
    z: -0.3687206906386416
    w: 0.012350049965138218

header: 
  seq: 17986
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: "panda_link0"
pose: 
  position: 
    x: 0.17708426713943481
    y: -0.47800061106681824
    z: 0.5208727121353149
  orientation: 
    x: -0.5564602017402649
    y: 0.7444760203361511
    z: -0.368720680475235
    w: 0.012350049801170826

### ROS1 noetic usage

rostopic pub -r 60 /cartesian_impedance_example_controller/equilibrium_pose geometry_msgs/PoseStamped "
header: 
  frame_id: "panda_link0"
pose: 
  position: 
    x: 0.17708426713943481
    y: -0.47800061106681824
    z: 0.5208727121353149
  orientation: 
    x: -0.5564602017402649
    y: 0.7444760203361511
    z: -0.368720680475235
    w: 0.012350049801170826
"

### ROS2 jazzy usage

ros2 topic pub -r 60 /cartesian_impedance_example_controller/equilibrium_pose geometry_msgs/PoseStamped "
header: 
  frame_id: "panda_link0"
pose: 
  position: 
    x: 0.3110661948654686
    y: -0.31788369018220675
    z: 0.35673353367990346
  orientation: 
    x: -0.5564602017402649
    y: 0.7444760203361511
    z: -0.368720680475235
    w: 0.012350049801170826
"
