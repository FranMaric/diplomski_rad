#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from franka_gripper.msg import GraspActionGoal, MoveActionGoal


class RobotControl:
	"""ROS1 (Noetic) helper for controlling Franka pose and gripper."""

	def __init__(
		self,
		frame_id="panda_link0",
		pose_topic="/cartesian_impedance_example_controller/equilibrium_pose",
		gripper_move_topic="/franka_gripper/move/goal",
		gripper_grasp_topic="/franka_gripper/grasp/goal",
		joint_states_topic="/joint_states",
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

		# Give publishers a short moment to register with ROS master/subscribers.
		rospy.sleep(0.5)

	def _joint_states_cb(self, msg):
		self._joint_names = list(msg.name)
		self._joint_positions = list(msg.position)
		self._joint_velocities = list(msg.velocity)
		self._joint_efforts = list(msg.effort)

	def get_joint_values(self):
		"""
		Get the latest joint positions as a dictionary.

		Returns:
			dict: {joint_name: joint_value}
		"""
		return dict(zip(self._joint_names, self._joint_positions))

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
	controller = RobotControl()
	controller.get_joint_values()
	
	print("Current joint values:", controller.get_joint_values())
	
	grip_pose = _build_pose(
		0.44941010301570267,
		-0.4740930479616219,
		0.015079897364934036,
		0.9338922374767828,
		-0.35438191842380573,
		-0.022341097189847583,
		0.041947825345603644,
	)
	in_air_pose = _build_pose(
		0.5859527673747369,
		-0.6055794976683551,
		0.5813498998428407,
		0.6946832369780841,
		-0.33802445299361306,
		0.6173720999500003,
		0.14834540654616335,
	)

	rospy.loginfo("RobotControl node started.")

	# controller.gripper_open()
	# controller.move_to(grip_pose)
	# rospy.sleep(3)
	# controller.gripper_close()
	# rospy.sleep(2)
	# controller.move_to(in_air_pose)
	# rospy.sleep(4)
	# controller.gripper_open()

	rospy.loginfo("RobotControl node finished.")

if __name__ == "__main__":
	main()
