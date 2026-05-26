// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <mutex>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

namespace franka_example_controllers {

// Topic-following Cartesian pose controller using libfranka's CartesianPose interface.
// Subscribes to ~equilibrium_pose (PoseStamped) and interpolates toward it with
// velocity limiting so no sudden jumps occur. Sub-mm accuracy at goal.
class CartesianPoseController : public controller_interface::MultiInterfaceController<
                                    franka_hw::FrankaPoseCartesianInterface,
                                    franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;

  // Internal integrator state (updated every 1 kHz tick)
  std::array<double, 16> current_pose_{};
  Eigen::Vector3d current_lin_vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_ang_vel_{Eigen::Vector3d::Zero()};

  // Target pose — written by ROS callback, read by RT update
  Eigen::Vector3d target_pos_;
  Eigen::Quaterniond target_quat_;
  std::mutex target_mutex_;

  ros::Subscriber sub_target_pose_;
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
};

}  // namespace franka_example_controllers
