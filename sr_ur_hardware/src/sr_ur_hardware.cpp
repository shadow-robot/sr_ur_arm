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
 * sr_ur_hardware.cpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis, Dan Greenwald
 */

#include <pthread.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include "sr_ur_robot_hw/sr_ur_hardware.hpp"


using namespace std;

namespace sr_ur
{
UrArmHardware::UrArmHardware() :
    loop_count_(0), ur_(), teach_mode_(false)
{
}

bool UrArmHardware::init(ros::NodeHandle &n, ros::NodeHandle &robot_hw_nh)
{
  node_  = n;

  if (node_.getParam("robot_id", robot_id_) && (!robot_id_.empty()))
  {
    joint_prefix_ = robot_id_ + '_';
  }

  using namespace hardware_interface;

  joint_position_command_.resize(NUM_OF_JOINTS);
  joint_position_.resize(NUM_OF_JOINTS);
  joint_effort_.resize(NUM_OF_JOINTS);
  joint_velocity_.resize(NUM_OF_JOINTS);

  std::string robot_state_name;
  node_.param<std::string>("robot_state_name", robot_state_name, "unique_robot_hw");

  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    std::string joint_name = joint_prefix_ + ur_joints[i];
    joint_position_[i] = joint_effort_[i] = joint_velocity_[i] = 0.0;
    joint_state_interface_.registerHandle(
      JointStateHandle(joint_name, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
    registerInterface(&joint_state_interface_);
  }

  string robot_ip_address;
  if (!node_.getParam("robot_ip_address", robot_ip_address))
  {
    ROS_ERROR("No IP address specified for sending commands to UrArmHardware");
    return false;
  }

  string control_pc_ip_address;
  if (!node_.getParam("control_pc_ip_address", control_pc_ip_address))
  {
    ROS_ERROR("No IP address specified for receiving state from UrArmHardware");
    return false;
  }

  string robot_program_path_param;
  if (!node_.getParam("robot_program_path", robot_program_path_param))
  {
    ROS_ERROR("No robot program path specified for UrArmHardware");
    return false;
  }

  ur_.robot_side_         = strdup(robot_id_[0] == 'r' ? "RIGHT" : "LEFT");
  ur_.robot_address_      = strdup(robot_ip_address.c_str());
  ur_.host_address_       = strdup(control_pc_ip_address.c_str());
  ur_.robot_program_path_ = strdup(robot_program_path_param.c_str());

  set_teach_mode_server_ = node_.advertiseService("set_teach_mode", &UrArmHardware::setTeachMode, this);
  return true;
}


void UrArmHardware::read(const ros::Time& time, const ros::Duration& period)
{
  if (++loop_count_ >= UR_PERIOD)
  {
    pthread_mutex_lock(&ur_.robot_state_mutex_);
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
    {
      joint_position_[i] = ur_.joint_positions_[i];
      joint_velocity_[i] = ur_.joint_velocities_[i];
      joint_effort_[i] = ur_.joint_motor_currents_[i];
      if (teach_mode_ && ur_.robot_ready_to_move_)
      {
	 joint_position_command_[i] = joint_position_[i];
      }
      else if (!teach_mode_ && !ur_.robot_ready_to_move_)
      {
	 joint_position_command_[i] = ur_.target_positions_[i];
      }
    }
    pthread_mutex_unlock(&ur_.robot_state_mutex_);
 }
}

void UrArmHardware::write(const ros::Time& time, const ros::Duration& period)
{
  if (loop_count_ >= UR_PERIOD && ur_.robot_ready_to_move_)
  {
    pthread_mutex_lock(&ur_.write_mutex_);
    for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
    {
      ur_.target_positions_[i] = joint_position_command_[i];
    }
    pthread_mutex_unlock(&ur_.write_mutex_);
    ur_.send_command();
    loop_count_ = 0;
  }
}

bool UrArmHardware::setTeachMode(sr_ur_msgs::SetTeachMode::Request &req, sr_ur_msgs::SetTeachMode::Response &resp)
{
  teach_mode_ = req.teach_mode;
  ur_.send_teach_mode_command(teach_mode_);
  resp.success = true;
  return true;
}

}

PLUGINLIB_EXPORT_CLASS( sr_ur::UrArmHardware, hardware_interface::RobotHW)
