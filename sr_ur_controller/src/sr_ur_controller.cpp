/*
 * Copyright (c) 2014, Shadow Robot Company, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */

/*
 * sr_ur_controller.cpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#include <pthread.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include "sr_ur_controller/sr_ur_controller.hpp"

PLUGINLIB_EXPORT_CLASS(sr_ur::UrArmController, controller_interface::ControllerBase)

using namespace std;

namespace sr_ur
{
UrArmController::UrArmController() :
    robot_(NULL), loop_count_(0)
{
}

bool UrArmController::init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
{
  ROS_ASSERT(robot);
  robot_ = robot;
  node_  = n;

  if (node_.getParam("robot_id", robot_id_) && (!robot_id_.empty()))
  {
    joint_prefix_ = robot_id_ + '_';
  }

  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    joint_states_[i] = robot_->getJointState(joint_prefix_ + ur_joints[i]);
    if (!joint_states_[i])
    {
      ROS_ERROR_STREAM("UrArmController could not find joint named : " << joint_prefix_ + ur_joints[i]);
      return false;
    }
    else
    {
      ROS_INFO_STREAM("UrArmController added joint " << joint_prefix_ + ur_joints[i]);
    }
    joint_states_[i]->calibrated_ = true;
  }

  string robot_ip_address;
  if (!node_.getParam("robot_ip_address", robot_ip_address))
  {
    ROS_ERROR("No IP address specified for sending commands to UrArmController");
    return false;
  }

  string control_pc_ip_address;
  if (!node_.getParam("control_pc_ip_address", control_pc_ip_address))
  {
    ROS_ERROR("No IP address specified for receiving state from UrArmController");
    return false;
  }

  string robot_program_path_param;
  if (!node_.getParam("robot_program_path", robot_program_path_param))
  {
    ROS_ERROR("No robot program path specified for UrArmController");
    return false;
  }

  robot_address      = strdup(robot_ip_address.c_str());
  host_address       = strdup(control_pc_ip_address.c_str());
  robot_program_path = strdup(robot_program_path_param.c_str());

  return true;
}

void UrArmController::starting(const ros::Time&)
{
  sub_command_ = node_.subscribe<std_msgs::Float64MultiArray>("command",
                                                              1,
                                                              &UrArmController::setCommandCB,
                                                              this);
  start_communication_with_robot();
}

void UrArmController::stopping(const ros::Time&)
{
  stop_communication_with_robot();
  sub_command_.shutdown();
}

void UrArmController::update(const ros::Time&, const ros::Duration& dt)
{
  if (loop_count_++ > 8)
  {
    pthread_mutex_lock(&robot_state_mutex);
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
    {
      joint_states_[i]->position_ = robot_joint_positions[i];
      joint_states_[i]->velocity_ = robot_joint_velocities[i];
      joint_states_[i]->effort_   = robot_joint_motor_currents[i];
    }
    pthread_mutex_unlock(&robot_state_mutex);

    pthread_mutex_lock(&write_mutex);
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
    {
      target_positions[i] = joint_states_[i]->commanded_position_;
    }
    pthread_mutex_unlock(&write_mutex);

    send_command_to_robot();

    loop_count_ = 0;
  }
}

void UrArmController::setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    joint_states_[i]->commanded_position_ = msg->data[i];
  }
}

}
