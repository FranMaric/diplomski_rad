// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_controller.h>

#include <algorithm>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool CartesianPoseController::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& node_handle) {
  auto* pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (pose_interface == nullptr) {
    ROS_ERROR("CartesianPoseController: Could not get Cartesian Pose interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        pose_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("CartesianPoseController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  sub_target_pose_ = node_handle.subscribe(
      "equilibrium_pose", 1, &CartesianPoseController::poseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void CartesianPoseController::starting(const ros::Time& /*time*/) {
  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> mat(current_pose_.data());
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_pos_ = mat.block<3, 1>(0, 3);
  target_quat_ = Eigen::Quaterniond(mat.block<3, 3>(0, 0));
  target_quat_.normalize();
  current_lin_vel_.setZero();
  current_ang_vel_.setZero();
}

void CartesianPoseController::update(const ros::Time& /*time*/,
                                     const ros::Duration& period) {
  // Clamp dt against missed cycles to avoid large one-off steps
  const double dt = std::min(std::max(period.toSec(), 0.001), 0.005);

  Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> cur(current_pose_.data());
  Eigen::Vector3d current_pos = cur.block<3, 1>(0, 3);
  Eigen::Quaterniond current_quat(cur.block<3, 3>(0, 0));
  current_quat.normalize();

  Eigen::Vector3d t_pos;
  Eigen::Quaterniond t_quat;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    t_pos = target_pos_;
    t_quat = target_quat_;
  }

  if (current_quat.dot(t_quat) < 0.0) {
    t_quat.coeffs() = -t_quat.coeffs();
  }

  // --- Linear ---
  // P-controller on position error → desired velocity, then rate-limit the velocity change
  // so the Franka motion generator never sees an acceleration spike.
  const double max_lin_vel = 0.1;   // m/s
  const double max_lin_acc = 2.0;   // m/s² — well within Franka's ~13 m/s² Cartesian limit

  Eigen::Vector3d vel_des = 2.0 * (t_pos - current_pos);  // P gain = 2
  if (vel_des.norm() > max_lin_vel) {
    vel_des = vel_des.normalized() * max_lin_vel;
  }
  Eigen::Vector3d dv = vel_des - current_lin_vel_;
  double max_dv = max_lin_acc * dt;
  if (dv.norm() > max_dv) {
    dv = dv.normalized() * max_dv;
  }
  current_lin_vel_ += dv;
  Eigen::Vector3d new_pos = current_pos + current_lin_vel_ * dt;

  // --- Angular ---
  // Same idea: compute axis-angle error → desired angular velocity → rate-limit.
  const double max_ang_vel = 0.5;   // rad/s
  const double max_ang_acc = 5.0;   // rad/s²

  Eigen::Quaterniond q_err = t_quat * current_quat.inverse();
  q_err.normalize();
  Eigen::AngleAxisd aa(q_err);
  Eigen::Vector3d ang_vel_des = 2.0 * aa.angle() * aa.axis();  // P gain = 2
  if (ang_vel_des.norm() > max_ang_vel) {
    ang_vel_des = ang_vel_des.normalized() * max_ang_vel;
  }
  Eigen::Vector3d dw = ang_vel_des - current_ang_vel_;
  double max_dw = max_ang_acc * dt;
  if (dw.norm() > max_dw) {
    dw = dw.normalized() * max_dw;
  }
  current_ang_vel_ += dw;

  double ang_step = current_ang_vel_.norm() * dt;
  Eigen::Quaterniond new_quat = current_quat;
  if (ang_step > 1e-8) {
    Eigen::AngleAxisd step_aa(ang_step, current_ang_vel_.normalized());
    new_quat = Eigen::Quaterniond(step_aa) * current_quat;
    new_quat.normalize();
  }

  std::array<double, 16> new_pose = current_pose_;
  Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> new_mat(new_pose.data());
  new_mat.block<3, 3>(0, 0) = new_quat.toRotationMatrix();
  new_mat.block<3, 1>(0, 3) = new_pos;

  cartesian_pose_handle_->setCommand(new_pose);
  current_pose_ = new_pose;
}

void CartesianPoseController::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  target_pos_ = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
  target_quat_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                    msg->pose.orientation.y, msg->pose.orientation.z);
  target_quat_.normalize();
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseController,
                       controller_interface::ControllerBase)
