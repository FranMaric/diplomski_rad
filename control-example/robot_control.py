#!/usr/bin/env python3

from email.header import Header

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import cv2
import numpy as np
from franka_gripper.msg import GraspActionGoal, MoveActionGoal
from openpi_client import image_tools
from openpi_client import websocket_client_policy

class RobotControl:
	"""ROS1 (Noetic) helper for controlling Franka pose and gripper."""

	def __init__(
		self,
		frame_id="panda_link0",
		pose_topic="/cartesian_impedance_example_controller/equilibrium_pose",
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

		self._ee_image_sub = rospy.Subscriber(
			ee_image_topic,
			Image,
			self._ee_image_cb,
			queue_size=10,
		)

		self._scene_image_sub = rospy.Subscriber(
			scene_image_topic,
			Image,
			self._scene_image_cb,
			queue_size=10,
		)

		self.ARM_JOINTS = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
		self._last_gripper_width = None

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
		"""Returns the current EE pose in self.frame_id, or None on TF failure."""
		try:
			self._tf_listener.waitForTransform(
				self.frame_id, ee_frame, rospy.Time(0), rospy.Duration(1.0)
			)
			translation, rot = self._tf_listener.lookupTransform(
				self.frame_id, ee_frame, rospy.Time(0)
			)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logwarn(f"TF lookup failed: {e}")
			return None

		rpy = tf.transformations.euler_from_quaternion(rot)
		return (translation, rpy)

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
	
	def _ee_image_cb(self, msg):
		self._latest_ee_image = msg

	def _scene_image_cb(self, msg):
		self._latest_scene_image = msg

	def get_latest_ee_image(self):
		if self._latest_ee_image is None:
			return None
		return self._cv_bridge.imgmsg_to_cv2(self._latest_ee_image, desired_encoding="bgr8")

	def get_latest_scene_image(self):
		if self._latest_scene_image is None:
			return None
		return self._cv_bridge.imgmsg_to_cv2(self._latest_scene_image, desired_encoding="bgr8")
	
	def execute_action_chunk(self, action_chunk):
		ACTION_RATE = 20.0  # Hz
		COMPLETE_ACTIONS = len(action_chunk)

		rate = rospy.Rate(ACTION_RATE)

		for i in range(COMPLETE_ACTIONS):
			action = action_chunk[i]
			ee_translation, ee_rpy = self.get_ee_pose()
			if ee_translation is None or ee_rpy is None:
				rospy.logwarn("Skipping action execution due to missing TF data.")
				continue

			# Apply deltas from action to current EE pose.
			ee_translation += tuple(action[0:3])  # delta x,y,z
			ee_rpy += tuple(action[3:6])          # delta roll,pitch,yaw

			# Convert to Pose message and publish.
			pose = Pose()
			pose.position.x = ee_translation[0]
			pose.position.y = ee_translation[1]
			pose.position.z = ee_translation[2]
			q = tf.transformations.quaternion_from_euler(ee_rpy[0], ee_rpy[1], ee_rpy[2])
			pose.orientation.x = q[0]
			pose.orientation.y = q[1]
			pose.orientation.z = q[2]
			pose.orientation.w = q[3]
			self.move_to(pose)

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
		ee_pos, ee_rot = robot_controller.get_ee_pose()
		
		rospy.loginfo(f"Current EE pose & rot: {ee_pos} & {ee_rot}")
		
		state = np.concatenate([
			ee_pos,           # idx 0,1,2  (m)
			ee_rot,           # idx 3,4,5  (rad)
			[1, -1],        # idx 6,7    (mirrored, see norm_stats)
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

	# ee_pose = _build_pose(0.51085583533713035, -0.350026636761991015456, 0.62930005044823294, 0.9999988, 0.0001545, 0.0013909, 0.0007167)

	# robot_controller.move_to(ee_pose)
	# return

	rospy.loginfo("RobotControl node connecting to policy server.")
	
	policy_client = websocket_client_policy.WebsocketClientPolicy(host="161.53.68.175", port=8000)

	rospy.loginfo("RobotControl node connected to policy server.")
	rospy.loginfo(f"Current EE pose: {robot_controller.get_ee_pose()}")

	iteration = 0
	while True:
		observation = build_observation(env="pi05_libero", prompt="pick up the white block", robot_controller=robot_controller)

		action_chunk = policy_client.infer(observation)["actions"]

		# print(action_chunk)

		robot_controller.execute_action_chunk(action_chunk)

		iteration += 1

	rospy.loginfo("RobotControl node finished.")

if __name__ == "__main__":
	main()
