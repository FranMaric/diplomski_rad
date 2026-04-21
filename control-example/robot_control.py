#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
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
	
	def get_joint_values_list(self):
		"""
		Get the latest joint positions as a list.

		Returns:
			list: [joint_value]
		"""
		return list(self._joint_positions)

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
		return self._cv_bridge.imgmsg_to_cv2(self._latest_ee_image, desired_encoding="bgr8")

	def get_latest_scene_image(self):
		return self._cv_bridge.imgmsg_to_cv2(self._latest_scene_image, desired_encoding="bgr8")


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


def main():
	rospy.loginfo("RobotControl node init.")

	robot_controller = RobotControl()

	rospy.loginfo("RobotControl node connecting to policy server.")
	
	policy_client = websocket_client_policy.WebsocketClientPolicy(host="161.53.68.175", port=8000)

	rospy.loginfo("RobotControl node connected to policy server.")

	observation = {
		"observation/exterior_image_1_left": image_tools.convert_to_uint8(
			image_tools.resize_with_pad(robot_controller.get_latest_ee_image(), 224, 224)
		),
		"observation/wrist_image_left": image_tools.convert_to_uint8(
			image_tools.resize_with_pad(robot_controller.get_latest_scene_image(), 224, 224)
		),
		"observation/joint_position": robot_controller.get_joint_values_list(),
		"observation/gripper_position": robot_controller.get_joint_values_dict().get("panda_finger_joint1", 0.0) > 0.02,
		"prompt": "Pick up the white block located on the blue plate.",
	}

	action_chunk = policy_client.infer(observation)["actions"]

	print(action_chunk)

	rospy.loginfo("RobotControl node finished.")

if __name__ == "__main__":
	main()
