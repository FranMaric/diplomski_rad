#!/usr/bin/env python3

ACTION_RATE = 30.0  # Hz — policy query rate
REPLAN_STEPS = 15 # how many steps to execute from each policy inference before re-querying policy server for next action chunk
ENSEMBLE_EXP_WEIGHT = 0.1  # exponential decay for ensemble weights; higher = faster decay (less history)
POLICY_SERVER_HOST = "161.53.68.175" # steffy
MODEL_ENV = "force_vla" #"pi05_droid" # "pi05_libero" or "pi05_droid"
CONTROLLER_TYPE =  "cartesian_impedance" # "cartesian_impedance" or "joint_velocity"
VELOCITY_SCALING = 0.2
SAVE_ACTION_CHUNKS_AS_CSV = False
MODEL_PROMPT = "sand the mold"

import rospy, math
import csv, io
import subprocess
import atexit
import os
import signal
import threading
import tf
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, WrenchStamped, TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from franka_gripper.msg import GraspActionGoal, MoveActionGoal
from openpi_client import image_tools
from openpi_client import websocket_client_policy
from joint_velocity_control import JointVelocityController
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def _quat_to_axis_angle(q_xyzw):
	"""Quaternion (x,y,z,w) -> 3-vector (axis * angle), shortest-path."""
	x, y, z, w = q_xyzw
	if w < 0:
		x, y, z, w = -x, -y, -z, -w
	sin_half = float(np.sqrt(x*x + y*y + z*z))
	angle = 2.0 * float(np.arctan2(sin_half, w))
	if sin_half < 1e-8:
		return np.zeros(3, dtype=np.float64)
	axis = np.array([x, y, z], dtype=np.float64) / sin_half
	return axis * angle


def _quat_to_euler(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class RobotControl:
	"""ROS1 (Noetic) helper for controlling Franka pose and gripper."""

	def __init__(
		self,
		frame_id="panda_link0",
		pose_topic="/cartesian_impedance_controller/equilibrium_pose",
		gripper_move_topic="/franka_gripper/move/goal",
		gripper_grasp_topic="/franka_gripper/grasp/goal",
		joint_states_topic="/joint_states",
		ee_image_topic="/ee_camera/image_raw",
		scene_image_topic="/scene_camera/image_raw",
		pose_publish_rate_hz=60.0,
	):
		if not rospy.core.is_initialized():
			rospy.init_node("robot_control", anonymous=True)

		self.frame_id = frame_id
		self.pose_publish_rate_hz = float(pose_publish_rate_hz)

		self._pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
		self._action_chunk_vis_pub = rospy.Publisher('/action_chunk_vis', MarkerArray, queue_size=1, latch=True)
		self._joint_trajectory_pub = rospy.Publisher('/position_joint_trajectory_controller/command', JointTrajectory, queue_size=1)
		self._gripper_move_pub = rospy.Publisher(gripper_move_topic, MoveActionGoal, queue_size=10)
		self._gripper_grasp_pub = rospy.Publisher(gripper_grasp_topic, GraspActionGoal, queue_size=10)

		# Store latest raw /joint_states values.
		self._joint_names = []
		self._joint_positions = []
		self._joint_velocities = []
		self._joint_efforts = []

		self._joint_states_sub = rospy.Subscriber(
			joint_states_topic,
			JointState,
			self._joint_states_cb,
			queue_size=10,
		)

		self._cv_bridge = CvBridge()
		self._latest_ee_image = None
		self._latest_scene_image = None
		self._ee_image_sub = rospy.Subscriber(
			ee_image_topic, Image, self._ee_image_cb, queue_size=10,
		)
		self._scene_image_sub = rospy.Subscriber(
			scene_image_topic, Image, self._scene_image_cb, queue_size=10,
		)

		self.ARM_JOINTS = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
		self._last_gripper_width = None

		# Edge-triggered gripper state. Start known-open.
		self._gripper_state = "open"

		self._tf_listener = tf.TransformListener()

		self._latest_force = None
		self._force_sub = rospy.Subscriber(
			'/optoforce_0', WrenchStamped, self._force_cb, queue_size=10
		)

		rospy.loginfo("Waiting for initial force reading...")
		while not rospy.is_shutdown() and self._latest_force is None:
			rospy.sleep(0.1)
		rospy.loginfo("Received force reading.")

		# self._vel_ctrl = JointVelocityController(command_hz=ACTION_RATE)

	def _force_cb(self, msg):
		self._latest_force = msg

	def _joint_states_cb(self, msg):
		self._joint_names = list(msg.name)
		self._joint_positions = list(msg.position)
		self._joint_velocities = list(msg.velocity)
		self._joint_efforts = list(msg.effort)

	def get_joint_values_dict(self):
		"""
		Get the latest joint positions as a dictionary.

		Returns:
			dict: {joint_name: joint_value}
		"""
		return dict(zip(self._joint_names, self._joint_positions))
	
	def get_arm_joint_values(self):
		"""
		Get the latest joint positions as a list.

		Returns:
			list: [joint_value]
		"""
		d = self.get_joint_values_dict()
		return [d[j] for j in self.ARM_JOINTS]

	def get_ee_pose(self):
		return self.get_frame_pose(frame='panda_EE')

	def get_frame_pose(self, frame):
		"""Returns (translation, quaternion_xyzw) in self.frame_id, or (None, None) on TF failure."""
		try:
			self._tf_listener.waitForTransform(
				self.frame_id, frame, rospy.Time(0), rospy.Duration(1.0)
			)
			translation, quaternion = self._tf_listener.lookupTransform(
				self.frame_id, frame, rospy.Time(0)
			)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logwarn(f"TF lookup failed: {e}")
			return None, None

		return (translation, quaternion)

	def move_to(self, pose):
		"""
		Publish once a target end-effector pose. Expects a TCP pose defined in panda_link0 frame.

		Args:
			pose (geometry_msgs.msg.Pose): Target pose in self.frame_id.
		"""
		if not isinstance(pose, Pose):
			raise TypeError("move_to expects geometry_msgs.msg.Pose")

		msg = PoseStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "panda_link0"  
		msg.pose = pose

		self._pose_pub.publish(msg)

	def move_tcp_in_kalup_frame(self, pose_in_kalup_frame: PoseStamped):
		try:
			self._tf_listener.waitForTransform(
				"panda_link0", "kalup", rospy.Time(0), rospy.Duration(1.0)
			)
			pose_in_link0 = self._tf_listener.transformPose("panda_link0", pose_in_kalup_frame)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logwarn(f"TF transform failed: {e}")
			return
		self.move_to(pose_in_link0.pose)

	def gripper_open(self, width=0.07, speed=0.2):
		"""
		Open/spread gripper to target width.
		"""
		msg = MoveActionGoal()
		msg.header.stamp = rospy.Time.now()
		msg.goal_id.stamp = rospy.Time.now()
		msg.goal.width = float(width)
		msg.goal.speed = float(speed)

		self._gripper_move_pub.publish(msg)

	def gripper_close(self, width=0.03, speed=0.05, force=5.0, epsilon_inner=0.005, epsilon_outer=0.005):
		"""
		Close/grasp using force control.

		Defaults follow README example.
		"""
		msg = GraspActionGoal()
		msg.header.stamp = rospy.Time.now()
		msg.goal_id.stamp = rospy.Time.now()
		msg.goal.width = float(width)
		msg.goal.epsilon.inner = float(epsilon_inner)
		msg.goal.epsilon.outer = float(epsilon_outer)
		msg.goal.speed = float(speed)
		msg.goal.force = float(force)

		self._gripper_grasp_pub.publish(msg)

	def apply_gripper_action(self, action_g):
		"""
		Edge-triggered absolute gripper command from policy.
		action_g in [-1, +1]: < -0.5 -> open, > +0.5 -> close, between -> hold.
		Avoids spamming action goals at 20 Hz (which causes preempt stutter).
		"""
		desired = self._gripper_state
		if action_g > 0.5:
			desired = "closed"
		elif action_g < -0.5:
			desired = "open"

		if desired == self._gripper_state:
			return

		if desired == "closed":
			self.gripper_close()
		else:
			self.gripper_open()

		self._gripper_state = desired
		rospy.loginfo(f"Gripper -> {desired} (action_g={action_g:+.3f})")

	def _ee_image_cb(self, msg):
		self._latest_ee_image = msg

	def _scene_image_cb(self, msg):
		self._latest_scene_image = msg

	def get_latest_ee_image(self):
		if self._latest_ee_image is None:
			return None
		return self._cv_bridge.imgmsg_to_cv2(self._latest_ee_image, desired_encoding="rgb8")

	def get_latest_scene_image(self):
		if self._latest_scene_image is None:
			return None
		return self._cv_bridge.imgmsg_to_cv2(self._latest_scene_image, desired_encoding="rgb8")

	def get_latest_wrench(self):
		return self._latest_force
	
	def publish_action_chunk_visualization(self, action_chunk):
		now = rospy.Time.now()
		array = MarkerArray()

		# DELETE previous markers first so stale points don't linger
		delete_all = Marker()
		delete_all.header.stamp = now
		delete_all.header.frame_id = "kalup"
		delete_all.ns = "action_chunk"
		delete_all.action = Marker.DELETEALL
		array.markers.append(delete_all)

		for i, action in enumerate(action_chunk):
			m = Marker()
			m.header.stamp = now
			m.header.frame_id = "kalup"
			m.ns = "action_chunk"
			m.id = i
			m.type = Marker.SPHERE
			m.action = Marker.ADD
			m.pose.position.x = float(action[0])
			m.pose.position.y = float(action[1])
			m.pose.position.z = float(action[2])
			m.pose.orientation.w = 1.0
			m.scale.x = m.scale.y = m.scale.z = 0.005  # in meters
			m.color.r = 0.0
			m.color.g = 0.8
			m.color.b = 1.0
			m.color.a = 1.0
			array.markers.append(m)

		self._action_chunk_vis_pub.publish(array)

	def execute_cartesian_action_chunk(self, action_chunk):
		rate = rospy.Rate(ACTION_RATE)

		for i in range(REPLAN_STEPS):
			action = action_chunk[i]
			ee_translation, ee_quat = self.get_frame_pose(frame="panda_EE")
			if ee_translation is None or ee_quat is None:
				rospy.logwarn("Skipping action execution due to missing TF data.")
				continue

			# Apply deltas from action to current EE pose.
			# action[0:3] are NORMALIZED [-1,1]; scale to meters (robosuite OSC default 0.05).
			# action[3:6] are axis-angle deltas in radians; compose via quaternion multiplication.
			ee_translation = np.array(ee_translation) + np.array(action[0:3]) * 0.05

			delta_rot = np.array(action[3:6], dtype=np.float64)
			angle = float(np.linalg.norm(delta_rot))
			if angle > 1e-8:
				axis = delta_rot / angle
				dq = tf.transformations.quaternion_about_axis(angle, axis)  # xyzw
				new_quat = tf.transformations.quaternion_multiply(dq, np.array(ee_quat))
				new_quat = new_quat / np.linalg.norm(new_quat)
			else:
				new_quat = np.array(ee_quat)

			# Convert to Pose message and publish.
			pose = Pose()
			pose.position.x = ee_translation[0]
			pose.position.y = ee_translation[1]
			pose.position.z = ee_translation[2]
			pose.orientation.x = new_quat[0]
			pose.orientation.y = new_quat[1]
			pose.orientation.z = new_quat[2]
			pose.orientation.w = new_quat[3]
			self.move_to(pose)

			# Gripper: absolute command in action[6], edge-triggered.
			self.apply_gripper_action(float(action[6]))

			rate.sleep()

	def execute_joint_velocity_action_chunk(self, action_chunk):
		"""
		Execute a chunk of joint-velocity actions on the position_joint_trajectory_controller.

		Each action row is expected to be [v0..v6, gripper] where v0-v6 are joint
		velocity deltas (rad/s) for ARM_JOINTS. All REPLAN_STEPS points are sent
		as a single JointTrajectory so the controller can plan the full motion
		smoothly. Gripper commands are still applied step-by-step at ACTION_RATE.
		"""

		# DROID convention: >0.5 = open, <=0.5 = close (position-based, edge-triggered)
		rate = rospy.Rate(ACTION_RATE)
		for action in action_chunk[:REPLAN_STEPS]:
			velocities = action[:7]
			velocities = velocities * VELOCITY_SCALING
			self._vel_ctrl.apply_velocity(velocities)

			desired = "open" if action[7] > 0.5 else "closed"
			if desired != self._gripper_state:
				self.gripper_open() if desired == "open" else self.gripper_close()
				self._gripper_state = desired
				rospy.loginfo(f"Gripper -> {desired} (action[7]={action[7]:+.3f})")
			rate.sleep()
	
	def execute_tcp_action_chunk(self, action_chunk):
		rate = rospy.Rate(ACTION_RATE)

		for i in range(REPLAN_STEPS):
			action = action_chunk[i]
			self.execute_single_tcp_action(action)
			rate.sleep()

	def execute_single_tcp_action(self, action):
		x, y, z, rx, ry, rz = float(action[0]), float(action[1]), float(action[2]), float(action[3]), float(action[4]), float(action[5])
		quat = tf.transformations.quaternion_from_euler(rx, ry, rz)
		pose_in_kalup = PoseStamped()
		pose_in_kalup.header.stamp = rospy.Time.now()
		pose_in_kalup.header.frame_id = "kalup"
		pose_in_kalup.pose.position.x = x
		pose_in_kalup.pose.position.y = y
		pose_in_kalup.pose.position.z = z
		pose_in_kalup.pose.orientation.x = quat[0]
		pose_in_kalup.pose.orientation.y = quat[1]
		pose_in_kalup.pose.orientation.z = quat[2]
		pose_in_kalup.pose.orientation.w = quat[3]
		self.move_tcp_in_kalup_frame(pose_in_kalup)


def _build_pose(px, py, pz, ox, oy, oz, ow):
	pose = Pose()
	pose.position.x = float(px)
	pose.position.y = float(py)
	pose.position.z = float(pz)
	pose.orientation.x = float(ox)
	pose.orientation.y = float(oy)
	pose.orientation.z = float(oz)
	pose.orientation.w = float(ow)
	return pose

class TemporalEnsembler:
	"""Temporal Ensembling from ACT (Zhao et al., 2023 — arXiv:2304.13705).

	At each timestep the policy is queried and returns an action chunk of length k.
	Because queries overlap, at step t there are up to k predictions for action a_t
	(one from each of the last k queries).  These are blended with exponential weights:

	    w_i = exp(-exp_weight * i),   i = steps since that query was made

	so the most recent query always dominates and older predictions fade out smoothly.

	Thread-safe: update() is called from the inference thread; get_action() from the
	execution thread.  query_step must be captured *before* inference starts so that
	chunk[offset] correctly indexes the action predicted for the current execution step.
	"""

	def __init__(self, chunk_size, action_dim, exp_weight=0.1):
		self._chunk_size = chunk_size
		self._action_dim = action_dim
		self._exp_weight = exp_weight
		self._chunks = []   # [(query_step: int, chunk: np.ndarray)]
		self._step = 0
		self._lock = threading.Lock()

	@property
	def step(self):
		return self._step   # atomic int read; safe to call from inference thread

	def update(self, action_chunk, query_step):
		"""Register a new chunk. query_step = execution step when the observation was captured."""
		with self._lock:
			self._chunks.append((query_step, np.array(action_chunk, dtype=np.float64)))
			self._chunks = [(s, c) for s, c in self._chunks if self._step - s < self._chunk_size]

	def get_action(self):
		"""Return the blended action for the current step and advance the execution counter."""
		with self._lock:
			actions, weights = [], []
			for query_step, chunk in self._chunks:
				offset = self._step - query_step   # 0 = freshest, larger = older
				if 0 <= offset < len(chunk):
					actions.append(chunk[offset])
					weights.append(np.exp(-self._exp_weight * offset))

			self._step += 1

			if not actions:
				return None
			w = np.array(weights)
			w /= w.sum()
			return (np.stack(actions) * w[:, None]).sum(axis=0)

	def reset(self):
		with self._lock:
			self._chunks.clear()
			self._step = 0


def build_observation(env, prompt, robot_controller):
	raw_ee = robot_controller.get_latest_ee_image()
	raw_scene = robot_controller.get_latest_scene_image()
	if raw_ee is None or raw_scene is None:
		rospy.logwarn_throttle(5.0, "Waiting for camera frames...")
		return None
	wrist_img = image_tools.convert_to_uint8(
		image_tools.resize_with_pad(raw_ee, 224, 224)
	)
	scene_img = image_tools.convert_to_uint8(
		image_tools.resize_with_pad(raw_scene, 224, 224)
	)
	if env == "pi05_libero": # requires ee cartesian controller
		ee_pos, ee_quat = robot_controller.get_ee_pose()
		ee_axis_angle = _quat_to_axis_angle(ee_quat)
		finger_joint = robot_controller.get_joint_values_dict().get("panda_finger_joint1", 0)

		rospy.loginfo(f"Current EE pos & axis-angle: {ee_pos} & {ee_axis_angle}")

		state = np.concatenate([
			ee_pos,           				# idx 0,1,2  (m)
			ee_axis_angle,           		# idx 3,4,5  (rad, axis-angle)
			[finger_joint, -finger_joint],  # idx 6,7    (mirrored, see norm_stats)
		]).astype(np.float32)

		observation = {
			"observation/image": scene_img,
			"observation/wrist_image": wrist_img,
			"observation/state": state,
			"prompt": prompt,
		}
		return observation
	elif env == "pi05_droid": # requires joint trajectory controller
		joint_position = np.array(robot_controller.get_arm_joint_values(), dtype=np.float32)

		gripper_position = np.array([robot_controller.get_joint_values_dict().get("panda_finger_joint1", 0.0)], dtype=np.float32)

		observation = {
			"observation/exterior_image_1_left": scene_img,
			"observation/wrist_image_left": wrist_img,
			"observation/joint_position": joint_position,
			"observation/gripper_position": gripper_position,
			"prompt": prompt,
		}
		return observation
	elif env == "force_vla":
		if robot_controller._latest_force is None:
			rospy.logwarn("No force info available.")
			return None
		try:
			robot_controller._tf_listener.waitForTransform(
				"kalup", "panda_EE", rospy.Time(0), rospy.Duration(1.0)
			)
			ee_pos, ee_quat = robot_controller._tf_listener.lookupTransform(
				"kalup", "panda_EE", rospy.Time(0)
			)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logwarn(f"TF lookup kalup->panda_EE failed: {e}")
			return None
		ee_pos = np.array(ee_pos, dtype=np.float32)

		ee_euler_angle = _quat_to_euler(ee_quat[0], ee_quat[1], ee_quat[2], ee_quat[3])

		rospy.loginfo(f"Current EE pose in kalup frame: {ee_pos} & {ee_euler_angle}")

		wrench = robot_controller._latest_force.wrench
		force = np.array([wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z], dtype=np.float32)

		force = np.array([0, 0, force[2], 0, 0, 0], dtype=np.float32)

		state = np.concatenate([
			ee_pos,           				# idx 0,1,2  (m)
			ee_euler_angle,           		# idx 3,4,5  (rad, axis-angle)
			[0],			# gripper placeholder (no gripper)
			force
		]).astype(np.float32)

		observation = {
			"image": scene_img,
			"wrist_image": wrist_img,
			"state": state,
			"prompt": prompt,
		}
		return observation
	else:
		raise ValueError(f"Unsupported env: {env}")
	

def publish_kalup_transform():
	T_kalup_to_panda_link0 = _build_pose(0.3208041427954132, -0.04823811340748261, 0.5164062837874686, 0,0,0,0)

	broadcaster = tf2_ros.StaticTransformBroadcaster()
	t = TransformStamped()

	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "panda_link0"
	t.child_frame_id = "kalup"

	# Translation (meters) - measure from robot base to kalup center
	t.transform.translation.x = T_kalup_to_panda_link0.position.x
	t.transform.translation.y = T_kalup_to_panda_link0.position.y
	t.transform.translation.z = T_kalup_to_panda_link0.position.z

	rpy = [0, 0, np.pi / 2]
	quat = tf.transformations.quaternion_from_euler(*rpy) 

	t.transform.rotation.x = quat[0]
	t.transform.rotation.y = quat[1]
	t.transform.rotation.z = quat[2]
	t.transform.rotation.w = quat[3]

	broadcaster.sendTransform(t)

def record_bag():
	bag_proc = subprocess.Popen(
		["bash", "/catkin_ws/record_inference_bag.sh"],
		stdout=subprocess.DEVNULL,
		stderr=subprocess.DEVNULL,
		preexec_fn=os.setsid,
	)
	def _stop_bag():
		try:
			os.killpg(os.getpgid(bag_proc.pid), signal.SIGINT)
			bag_proc.wait(timeout=10)
		except Exception:
			bag_proc.kill()
	atexit.register(_stop_bag)
	rospy.loginfo(f"Bag recording started (pid={bag_proc.pid}).")

def move_to_kalup_zero(robot_controller):
	quat = tf.transformations.quaternion_from_euler(0.6, 0, 0)

	pose_in_kalup_frame = PoseStamped()
	pose_in_kalup_frame.pose = _build_pose(0, 0.005, 0.0, quat[0], quat[1], quat[2], quat[3])
	pose_in_kalup_frame.header.stamp = rospy.Time.now()
	pose_in_kalup_frame.header.frame_id = "kalup"

	robot_controller.move_tcp_in_kalup_frame(pose_in_kalup_frame)
	rospy.loginfo("Sent move command to initial pose above kalup.")
	rospy.sleep(3.0)

def recover_from_errors():
	subprocess.run([
		'rostopic', 'pub', '--once',
		'/franka_control/error_recovery/goal',
		'std_msgs/Empty', '{}'
	])

def _inference_loop(policy_client, robot_controller, ensembler, stop_event):
	"""Background thread: continuously query the policy and push chunks into the ensembler.

	query_step is captured before inference so that chunk[offset] aligns with the
	correct execution step even after ~0.5 s of inference latency.
	"""
	chunk_index = 1 # for CSV naming if enabled
	while not stop_event.is_set() and not rospy.is_shutdown():
		observation = build_observation(env=MODEL_ENV, prompt=MODEL_PROMPT, robot_controller=robot_controller)
		if observation is None:
			rospy.sleep(0.05)
			continue

		query_step = ensembler.step   # snapshot before blocking inference call
		try:
			start_time = rospy.Time.now()
			response = policy_client.infer(observation)
			infer_plus_network_time = (rospy.Time.now() - start_time).to_sec() * 1000
			action_chunk = response["actions"]
			infer_ms = response["policy_timing"]["infer_ms"]
		except Exception as e:
			rospy.logwarn(f"Policy inference error: {e}")
			continue
		
		if SAVE_ACTION_CHUNKS_AS_CSV:
			csv_path = os.path.join("/bags", f"action_chunk_{chunk_index}.csv")
			np.savetxt(csv_path, np.array(action_chunk), delimiter=",", header="x,y,z,rx,ry,rz,gripper_width", comments="")
			chunk_index += 1

		ensembler.update(action_chunk, query_step=query_step)
		robot_controller.publish_action_chunk_visualization(action_chunk)
		rospy.loginfo(f"New chunk registered (queried at step {query_step}, arrived at step {ensembler.step}), infer_ms={infer_ms:.3f}, infer+network_ms={infer_plus_network_time:.3f}")


def main():
	recover_from_errors()

	rospy.loginfo("RobotControl node init.")

	robot_controller = RobotControl()

	publish_kalup_transform()

	rospy.loginfo(f"Current EE pose: {robot_controller.get_frame_pose(frame='panda_EE')}")

	input("\n --------- Press Enter to proceed with the script and start recording ---------  \n\n")
	record_bag()

	move_to_kalup_zero(robot_controller)

	rospy.loginfo("RobotControl node connecting to policy server.")

	policy_client = websocket_client_policy.WebsocketClientPolicy(host=POLICY_SERVER_HOST, port=8000)

	rospy.loginfo("RobotControl node connected to policy server.")

	max_iterations = 15 * REPLAN_STEPS
	iteration = 0

	# Prime the ensembler with the first chunk, then hand off inference to a background
	# thread so the execution loop runs at exactly ACTION_RATE regardless of latency.
	rospy.loginfo("Waiting for first policy inference before starting execution...")
	while not rospy.is_shutdown():
		observation = build_observation(env=MODEL_ENV, prompt=MODEL_PROMPT, robot_controller=robot_controller)
		if observation is not None:
			first_chunk = policy_client.infer(observation)["actions"]
			break
		rospy.sleep(0.05)

	chunk_len = len(first_chunk)
	action_dim = np.array(first_chunk[0]).shape[0]
	ensembler = TemporalEnsembler(chunk_len, action_dim, ENSEMBLE_EXP_WEIGHT)
	ensembler.update(first_chunk, query_step=0)
	robot_controller.publish_action_chunk_visualization(first_chunk)
	rospy.loginfo(f"Temporal ensembling: chunk_size={chunk_len}, action_dim={action_dim}, exp_weight={ENSEMBLE_EXP_WEIGHT}")

	stop_event = threading.Event()
	inference_thread = threading.Thread(
		target=_inference_loop,
		args=(policy_client, robot_controller, ensembler, stop_event),
		daemon=True,
	)
	inference_thread.start()

	rate = rospy.Rate(ACTION_RATE)
	try:
		while not rospy.is_shutdown():
			action = ensembler.get_action()

			if action is not None:
				if CONTROLLER_TYPE == "cartesian_impedance" and MODEL_ENV == "force_vla":
					robot_controller.execute_single_tcp_action(action)
			else:
				rospy.logwarn_throttle(1.0, "Ensembler has no actions yet; waiting for inference thread.")

			# iteration += 1
			# if iteration == max_iterations:
			# 	rospy.loginfo(f"Completed {max_iterations} steps, exiting for testing.")
			# 	break

			rate.sleep()
	except Exception as e:
		rospy.logerr(f"Exception in execution loop: {e}")
	finally:
		stop_event.set()
		inference_thread.join(timeout=2.0)

	rospy.loginfo("RobotControl node finished.")

if __name__ == "__main__":
	main()