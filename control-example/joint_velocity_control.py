#!/usr/bin/env python3
"""
Joint velocity control for Franka Panda via position_joint_trajectory_controller.

Designed for 15-20 Hz velocity commands (e.g. from a VLA model). Each call to
apply_velocity() integrates vel*dt into a position target and sends a single-point
JointTrajectory. The controller's cubic interpolator handles smooth motion between
commands — no background timer needed.

Usage:
    ctrl = JointVelocityController(command_hz=15.0)
    # in your inference loop at ~15 Hz:
    ctrl.apply_velocity(vel_7dof)
"""

import numpy as np
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
]

JOINT_LIMITS_LOW  = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
JOINT_LIMITS_HIGH = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

MAX_JOINT_VEL = 0.8  # rad/s hard cap per joint


class JointVelocityController:
    def __init__(self, command_hz=15.0, traj_topic='/position_joint_trajectory_controller/command'):
        if not rospy.core.is_initialized():
            rospy.init_node('joint_velocity_controller', anonymous=True)

        self._step_dt  = 1.0 / command_hz
        self._joint_state = None   # latest measured joint positions (from /joint_states)
        self._commanded_joint_state = None  # integration state — what we last commanded

        self._traj_pub = rospy.Publisher(traj_topic, JointTrajectory, queue_size=1)
        self._joint_state_sub = rospy.Subscriber('/joint_states', JointState, self._joint_state_cb, queue_size=1)

        rospy.loginfo("Waiting for /joint_states...")
        while self._joint_state is None and not rospy.is_shutdown():
            rospy.sleep(0.01)
        rospy.loginfo("JointVelocityController ready.")

    def _joint_state_cb(self, msg):
        positions = []
        for name in JOINT_NAMES:
            if name in msg.name:
                positions.append(msg.position[msg.name.index(name)])
        if len(positions) == 7:
            self._joint_state = np.array(positions)
            if self._commanded_joint_state is None:
                self._commanded_joint_state = self._joint_state.copy()

    def apply_velocity(self, velocities, step_dt=None):
        """
        Integrate velocity for one step and send to the controller.

        Call this at your inference rate (e.g. 15-20 Hz). Integrates from the
        commanded state (not measured) so targets form a smooth curve regardless
        of tracking error.

        velocities: length-7 array (rad/s)
        step_dt:    override step duration (seconds); defaults to 1/command_hz
        """
        if self._commanded_joint_state is None:
            rospy.logwarn("No joint state received yet; cannot apply velocity.")
            return

        dt  = step_dt if step_dt is not None else self._step_dt
        vel = np.clip(np.array(velocities, dtype=np.float64), -MAX_JOINT_VEL, MAX_JOINT_VEL)
        target = self._commanded_joint_state + vel * dt
        target_clipped = np.clip(target, JOINT_LIMITS_LOW, JOINT_LIMITS_HIGH)

        clip_delta = target - target_clipped
        if np.any(clip_delta != 0.0):
            rospy.logwarn_throttle(1.0, f"Target clipped by {clip_delta}")

        self._commanded_joint_state = target_clipped

        pt = JointTrajectoryPoint()
        pt.positions  = target_clipped.tolist()
        pt.velocities = vel.tolist()     # velocity hint enables cubic (not linear) interpolation
        pt.time_from_start = rospy.Duration(dt * 2)

        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names  = JOINT_NAMES
        traj.points       = [pt]
        self._traj_pub.publish(traj)

    def stop(self):
        """Hold current position (zero velocity)."""
        self.apply_velocity(np.zeros(7))

    def get_joint_positions(self):
        return self._joint_state.copy() if self._joint_state is not None else None


# ---------------------------------------------------------------------------
# Demo: sinusoidal oscillation on joint 1 at 15 Hz
# ---------------------------------------------------------------------------
def demo():
    ctrl = JointVelocityController(command_hz=15.0)
    rate = rospy.Rate(15)
    t = 0.0
    dt = 1.0 / 15.0

    rospy.loginfo("Demo: oscillating joint 1 at 15 Hz. Ctrl-C to stop.")
    try:
        while not rospy.is_shutdown():
            vel = np.zeros(7)
            vel[0] = 0.5 * np.sin(0.7 * t)
            ctrl.apply_velocity(vel)
            t += dt
            rate.sleep()
            # break
    finally:
        ctrl.stop()
        rospy.loginfo("Demo finished.")


if __name__ == '__main__':
    demo()
