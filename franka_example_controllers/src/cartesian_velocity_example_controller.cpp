// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// cartesian_velocity_example_controller.cpp

#include <franka_example_controllers/cartesian_velocity_example_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

namespace franka_example_controllers
{

  bool CartesianVelocityExampleController::init(hardware_interface::RobotHW *robot_hardware,
                                                ros::NodeHandle &node_handle)
  {
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
      return false;
    }

    velocity_cartesian_interface_ =
        robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
    if (velocity_cartesian_interface_ == nullptr)
    {
      ROS_ERROR(
          "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
          "hardware");
      return false;
    }
    try
    {
      velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
          velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
      return false;
    }

    auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
      return false;
    }

    try
    {
      auto state_handle = state_interface->getHandle(arm_id + "_robot");

      std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      for (size_t i = 0; i < q_start.size(); i++)
      {
        if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1)
        {
          ROS_ERROR_STREAM(
              "CartesianVelocityExampleController: Robot is not in the expected starting position "
              "for running this example. Run `roslaunch franka_example_controllers "
              "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
              "first.");
          return false;
        }
      }
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM(
          "CartesianVelocityExampleController: Exception getting state handle: " << e.what());
      return false;
    }

    // Subscritor del comando de velocidad
    ros::NodeHandle n;
    velocity_subscriber_ = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &CartesianVelocityExampleController::cmdVelCallback, this);

    return true;
  }

  void CartesianVelocityExampleController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
  {
    cmd_vel_ = *msg;
  }

  void CartesianVelocityExampleController::starting(const ros::Time & /* time */)
  {
    elapsed_time_ = ros::Duration(0.0);
  }

  void CartesianVelocityExampleController::update(const ros::Time & /* time */,
                                                  const ros::Duration &period)
  {
    std::array<double, 6> command = {{cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.linear.z,
                                      cmd_vel_.angular.x, cmd_vel_.angular.y, cmd_vel_.angular.z}};

    velocity_cartesian_handle_->setCommand(command);
  }

  void CartesianVelocityExampleController::stopping(const ros::Time & /*time*/)
  {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
  }

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerBase)
