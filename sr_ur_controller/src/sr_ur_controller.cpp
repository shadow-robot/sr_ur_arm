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

  ur_.robot_address      = strdup(robot_ip_address.c_str());
  ur_.host_address       = strdup(control_pc_ip_address.c_str());
  ur_.robot_program_path = strdup(robot_program_path_param.c_str());

  return true;
}

void UrArmController::starting(const ros::Time&)
{
  sub_command_ = node_.subscribe<std_msgs::Float64MultiArray>("command",
                                                              1,
                                                              &UrArmController::setCommandCB,
                                                              this);
  start_communication_with_robot(&ur_);
}

void UrArmController::stopping(const ros::Time&)
{
  stop_communication_with_robot(&ur_);
  sub_command_.shutdown();
}

void UrArmController::update(const ros::Time&, const ros::Duration&)
{
  if (loop_count_++ > 16)
  {
    pthread_mutex_lock(&ur_.robot_state_mutex);
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
    {
      joint_states_[i]->position_ = ur_.joint_positions[i];
      joint_states_[i]->velocity_ = ur_.joint_velocities[i];
      joint_states_[i]->effort_   = ur_.joint_motor_currents[i];
      if (!ur_.robot_ready_to_move)
      {
        joint_states_[i]->commanded_position_ = ur_.target_positions[i];
      }
    }
    pthread_mutex_unlock(&ur_.robot_state_mutex);

    if (!ur_.robot_ready_to_move)
    {
      return;
    }

    pthread_mutex_lock(&ur_.write_mutex);
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
    {
      ur_.target_positions[i] = joint_states_[i]->commanded_position_;
    }
    pthread_mutex_unlock(&ur_.write_mutex);

    send_command_to_robot(&ur_);

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
