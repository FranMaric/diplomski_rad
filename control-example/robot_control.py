#!/usr/bin/env python3

USE_REAL_CAMERAS = True  # True = USB webcams via WebCamera; False = ROS image topics from Gazebo

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from franka_gripper.msg import GraspActionGoal, MoveActionGoal
from openpi_client import image_tools
from openpi_client import websocket_client_policy
if USE_REAL_CAMERAS:
    from webcamera import WebCamera
else:
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


class RobotControl:
	"""ROS1 (Noetic) helper for controlling Franka pose and gripper."""

	def __init__(
		self,
		frame_id="panda_link0",
		pose_topic="/cartesian_impedance_example_controller/equilibrium_pose",
		gripper_move_topic="/franka_gripper/move/goal",
		gripper_grasp_topic="/franka_gripper/grasp/goal",
		joint_states_topic="/joint_states",
		camera_indices=[0, 2],
		ee_image_topic="/ee_camera/image_raw",
		scene_image_topic="/scene_camera/image_raw",
		pose_publish_rate_hz=60.0,
	):
		if not rospy.core.is_initialized():
			rospy.init_node("robot_control", anonymous=True)

		self.frame_id = frame_id
		self.pose_publish_rate_hz = float(pose_publish_rate_hz)

		self._pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
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

		if USE_REAL_CAMERAS:
			self._camera = WebCamera(camera_indices=camera_indices)
		else:
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

		# Give publishers a short moment to register with ROS master/subscribers.
		rospy.sleep(0.5)

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

	def get_ee_pose(self, ee_frame="panda_EE"):
		"""Returns (translation, quaternion_xyzw) in self.frame_id, or (None, None) on TF failure."""
		try:
			self._tf_listener.waitForTransform(
				self.frame_id, ee_frame, rospy.Time(0), rospy.Duration(1.0)
			)
			translation, quaternion = self._tf_listener.lookupTransform(
				self.frame_id, ee_frame, rospy.Time(0)
			)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logwarn(f"TF lookup failed: {e}")
			return None, None

		return (translation, quaternion)

	def move_to(self, pose):
		"""
		Publish once a target end-effector pose.

		Args:
			pose (geometry_msgs.msg.Pose): Target pose in self.frame_id.
		"""
		if not isinstance(pose, Pose):
			raise TypeError("move_to expects geometry_msgs.msg.Pose")

		msg = PoseStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = self.frame_id
		msg.pose = pose

		self._pose_pub.publish(msg)

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
		if USE_REAL_CAMERAS:
			return self._camera.get_image(0)
		if self._latest_ee_image is None:
			return None
		return self._cv_bridge.imgmsg_to_cv2(self._latest_ee_image, desired_encoding="bgr8")

	def get_latest_scene_image(self):
		if USE_REAL_CAMERAS:
			return self._camera.get_image(1)
		if self._latest_scene_image is None:
			return None
		return self._cv_bridge.imgmsg_to_cv2(self._latest_scene_image, desired_encoding="bgr8")
	
	def execute_action_chunk(self, action_chunk):
		ACTION_RATE = 20.0  # Hz
		REPLAN_STEPS = 5 # len(action_chunk)

		rate = rospy.Rate(ACTION_RATE)

		for i in range(REPLAN_STEPS):
			action = action_chunk[i]
			ee_translation, ee_quat = self.get_ee_pose()
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

def build_observation(env="pi05_libero", prompt="pick up the white block from the blue plate.", robot_controller=None):
	wrist_img = image_tools.convert_to_uint8(
		image_tools.resize_with_pad(robot_controller.get_latest_ee_image(), 224, 224)
	)
	scene_img = image_tools.convert_to_uint8(
		image_tools.resize_with_pad(robot_controller.get_latest_scene_image(), 224, 224)
	)
	if env == "pi05_libero":
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
	# elif env == "pi05_droid":
	# 	observation = {
	# 		"observation/image": scene_img,
	# 		"observation/wrist_image": wrist_img,
	# 		"observation/state": np.zeros(8, dtype=np.float32),  # Placeholder state vector
	# 		"prompt": prompt,
	# 	}
	# 	return observation
	else:
		raise ValueError(f"Unsupported env: {env}")

def main():
	rospy.loginfo("RobotControl node init.")

	robot_controller = RobotControl()

	# robot_controller.gripper_open()

	# ee_pose = _build_pose(0.5521804078001702, 0.03545474781469954, 0.6770280167101107, -0.9910179470831783, 0.014436862832127084, -0.1317547604250494, -0.017608223402260894)

	# robot_controller.move_to(ee_pose)
	# return

	rospy.loginfo("RobotControl node connecting to policy server.")
	
	policy_client = websocket_client_policy.WebsocketClientPolicy(host="161.53.68.175", port=8000)

	rospy.loginfo("RobotControl node connected to policy server.")
	rospy.loginfo(f"Current EE pose: {robot_controller.get_ee_pose()}")

	iteration = 0
	try:
		while not rospy.is_shutdown():
			observation = build_observation(env="pi05_libero", prompt="pick up the orange object", robot_controller=robot_controller)

			action_chunk = policy_client.infer(observation)["actions"]

			# print(action_chunk)

			robot_controller.execute_action_chunk(action_chunk)

			iteration += 1
	finally:
		if USE_REAL_CAMERAS:
			robot_controller._camera.close()

	rospy.loginfo("RobotControl node finished.")

if __name__ == "__main__":
	main()