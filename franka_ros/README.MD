### Build
`cd franka-ros1-noetic`
`chmod +x build_docker.sh`
`./build_docker.sh`

### Run after succesful build
`cd franka-ros1-noetic`
`chmod +x run_docker.sh`
`./run_docker.sh`
then the franka controller is expecting commands on `/cartesian_impedance_example_controller/equilibrium_pose` 

### Primjeri tocaka
- gripping pozicija
header: 
  frame_id: "panda_link0"
pose: 
  position: 
    x: 0.44941010301570267
    y: -0.4740930479616219
    z: 0.015079897364934036
  orientation: 
    x: 0.9338922374767828
    y: -0.35438191842380573
    z: -0.022341097189847583
    w: 0.041947825345603644

- u zraku pozicija
header: 
  frame_id: "panda_link0"
pose: 
  position: 
    x: 0.5859527673747369
    y: -0.6055794976683551
    z: 0.5813498998428407
  orientation: 
    x: 0.6946832369780841
    y: -0.33802445299361306
    z: 0.6173720999500003
    w: 0.14834540654616335

## ROS1 noetic usage

### setting pose

rostopic pub -r 60 /cartesian_impedance_example_controller/equilibrium_pose geometry_msgs/PoseStamped "
header: 
  frame_id: "panda_link0"
pose: 
  position: 
    x: 0.44941010301570267
    y: -0.4740930479616219
    z: 0.015079897364934036
  orientation: 
    x: 0.9338922374767828
    y: -0.35438191842380573
    z: -0.022341097189847583
    w: 0.041947825345603644
"

### Gripper actions
- rupa tj dio koji nedostaje na gripperu je 17.5 mm

### Spread gripper to defined width
rostopic pub -1 /franka_gripper/move/goal franka_gripper/MoveActionGoal "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
goal_id:
  stamp: {secs: 0, nsecs: 0}
  id: ''
goal:
  width: 0.07
  speed: 0.1"

### Grasp with defined force
rostopic pub -1 /franka_gripper/grasp/goal franka_gripper/GraspActionGoal "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
goal_id:
  stamp: {secs: 0, nsecs: 0}
  id: ''
goal:
  width: 0.03
  epsilon:
    inner: 0.005
    outer: 0.005
  speed: 0.05
  force: 5.0"

## ROS2 jazzy usage

### setting pose

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
