// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_torque_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <ros/console.h>

namespace franka_example_controllers {

bool JointTorqueController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {

  sub_torque_ = node_handle.subscribe(
      "joint_torque", 20, &JointTorqueController::torqueCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointTorqueController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointTorqueController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointTorqueController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  return true;
}

void JointTorqueController::starting(const ros::Time& /*time*/) {
}

void JointTorqueController::update(const ros::Time& time,
                                                 const ros::Duration& /*period*/) {
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(0.0);
  }
}

}

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointTorqueController,
                       controller_interface::ControllerBase)
