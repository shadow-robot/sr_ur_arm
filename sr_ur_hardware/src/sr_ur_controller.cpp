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
    robot_(NULL), loop_count_(0), ur_(), teach_mode_(false)
{
}

bool UrArmController::init(ros_ethercat_model::RobotStateInterface* robot, ros::NodeHandle &n)
{
  ROS_ASSERT(robot);
  robot_ = robot;
  node_  = n;

  if (node_.getParam("robot_id", robot_id_) && (!robot_id_.empty()))
  {
    joint_prefix_ = robot_id_ + '_';
  }


  std::string robot_state_name;
  node_.param<std::string>("robot_state_name", robot_state_name, "unique_robot_hw");

  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    joint_states_[i] = robot_->getHandle(robot_state_name).getState()->getJointState(joint_prefix_ + ur_joints[i]);
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

  ur_.robot_side_         = strdup(robot_id_[0] == 'r' ? "RIGHT" : "LEFT");
  ur_.robot_address_      = strdup(robot_ip_address.c_str());
  ur_.host_address_       = strdup(control_pc_ip_address.c_str());
  ur_.robot_program_path_ = strdup(robot_program_path_param.c_str());

  set_teach_mode_server_ = node_.advertiseService("set_teach_mode", &UrArmController::setTeachMode, this);
  return true;
}

void UrArmController::starting(const ros::Time&)
{
  sub_command_ = node_.subscribe<std_msgs::Float64MultiArray>("command",
                                                              1,
                                                              &UrArmController::setCommandCB,
                                                              this);
  ur_.start();
}

void UrArmController::stopping(const ros::Time&)
{
  ur_.stop();
  sub_command_.shutdown();
}

void UrArmController::update(const ros::Time&, const ros::Duration&)
{
  if (++loop_count_ >= UR_PERIOD)
  {
    if (teach_mode_)
    {
      pthread_mutex_lock(&ur_.robot_state_mutex_);
      for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
      {
        joint_states_[i]->position_ = ur_.joint_positions_     [i];
        joint_states_[i]->velocity_ = ur_.joint_velocities_    [i];
        joint_states_[i]->effort_   = ur_.joint_motor_currents_[i];
        // TODO replace this condition to access rs_client->robot_state_received
        // it would make more sense here but now is not accessible.
        // robot_ready_to_move_ will do the job as it is set after robot_state_received
        if (ur_.robot_ready_to_move_)
        {
          // Keep updating commanded_position_ so that when teach mode is switched off the robot stays where it was
          joint_states_[i]->commanded_position_ = ur_.joint_positions_[i];
        }
      }
      pthread_mutex_unlock(&ur_.robot_state_mutex_);
    }
    else
    {
      pthread_mutex_lock(&ur_.robot_state_mutex_);
      for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
      {
        joint_states_[i]->position_ = ur_.joint_positions_     [i];
        joint_states_[i]->velocity_ = ur_.joint_velocities_    [i];
        joint_states_[i]->effort_   = ur_.joint_motor_currents_[i];
        if (!ur_.robot_ready_to_move_)
        {
          joint_states_[i]->commanded_position_ = ur_.target_positions_[i];
        }
      }
      pthread_mutex_unlock(&ur_.robot_state_mutex_);

      if (!ur_.robot_ready_to_move_)
      {
        return;
      }

      pthread_mutex_lock(&ur_.write_mutex_);
      for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
      {
        ur_.target_positions_[i] = joint_states_[i]->commanded_position_;
      }
      pthread_mutex_unlock(&ur_.write_mutex_);

      ur_.send_command();
    }

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

void UrArmController::enforceLimits(double *targets)
{
  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    targets[i] = min(targets[i], joint_states_[i]->joint_->limits->upper);
    targets[i] = max(targets[i], joint_states_[i]->joint_->limits->lower);

    double desired_velocity = (targets[i] - ur_.previous_targets_[i]) / (UR_PERIOD/1000.0);
    if ((targets[i] > ur_.previous_targets_[i] &&
         desired_velocity >  joint_states_[i]->joint_->limits->velocity) ||
        (targets[i] < ur_.previous_targets_[i] &&
         desired_velocity < -joint_states_[i]->joint_->limits->velocity))
    {
      targets[i] = UR_PERIOD*joint_states_[i]->joint_->limits->velocity + ur_.previous_targets_[i];
    }
  }
}

bool UrArmController::setTeachMode(sr_ur_msgs::SetTeachMode::Request &req, sr_ur_msgs::SetTeachMode::Response &resp)
{
  teach_mode_ = req.teach_mode;
  ur_.send_teach_mode_command(teach_mode_);
  resp.success = true;
  return true;
}

}
