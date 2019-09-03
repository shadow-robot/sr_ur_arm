/*
* Copyright 2014 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/*
* sr_ur_robot_hw.cpp
*
*  Created on: 20 Oct 2014
*      Author: Manos Nikolaidis, Dan Greenwald
*/

#include <pthread.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include "sr_ur_robot_hw/sr_ur_robot_hw.hpp"
#include "sr_ur_robot_hw/sr_ur_robot_state_client.hpp"

#include <string>
#include <vector>

PLUGINLIB_EXPORT_CLASS(sr_ur_robot_hw::UrArmRobotHW, hardware_interface::RobotHW)

namespace sr_ur_robot_hw
{
UrArmRobotHW::UrArmRobotHW() :
    loop_count_(0), first_read_(true), ur_(), teach_mode_(false)
{
}

bool UrArmRobotHW::init(ros::NodeHandle &n, ros::NodeHandle &robot_hw_nh)
{
  node_  = robot_hw_nh;

  if (node_.getParam("robot_id", robot_id_) && (!robot_id_.empty()))
  {
    joint_prefix_ = robot_id_ + '_';
  }

  joint_position_command_.resize(NUM_OF_JOINTS);
  joint_position_.resize(NUM_OF_JOINTS);
  joint_effort_.resize(NUM_OF_JOINTS);
  joint_velocity_.resize(NUM_OF_JOINTS);

  std::string robot_state_name;
  node_.param<std::string>("robot_state_name", robot_state_name, "unique_robot_hw");

  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    std::string joint_name = joint_prefix_ + ur_joints[i];
    ROS_INFO_STREAM("Joint state interface for arm joint " << joint_name);
    joint_position_[i] = joint_effort_[i] = joint_velocity_[i] = 0.0;
    joint_state_interface_.registerHandle(
      hardware_interface::JointStateHandle(joint_name, &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
    position_joint_interface_.registerHandle(
      hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_name), &joint_position_command_[i]));
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);


  std::string robot_ip_address;
  if (!node_.getParam("robot_ip_address", robot_ip_address))
  {
    ROS_ERROR("No IP address specified for sending commands to UrArmRobotHW");
    return false;
  }

  std::string control_pc_ip_address;
  if (!node_.getParam("control_pc_ip_address", control_pc_ip_address))
  {
    ROS_ERROR("No IP address specified for receiving state from UrArmRobotHW");
    return false;
  }

  std::string robot_program_path_param;
  if (!node_.getParam("robot_program_path", robot_program_path_param))
  {
    ROS_ERROR("No robot program path specified for UrArmRobotHW");
    return false;
  }

  ur_.robot_side_         = strdup(robot_id_[0] == 'r' ? "RIGHT" : "LEFT");
  ur_.robot_address_      = strdup(robot_ip_address.c_str());
  ur_.host_address_       = strdup(control_pc_ip_address.c_str());
  ur_.robot_program_path_ = strdup(robot_program_path_param.c_str());

  float payload_mass_kg_param;
  if (!node_.getParam("payload_mass_kg", payload_mass_kg_param))
  {
    ROS_WARN("No payload mass specified for UrArmRobotHW. Assuming 0.0kg.");
    payload_mass_kg_param = 0.0;
  }

  std::vector<float> payload_center_of_mass_m_param(3, 0.0);
  if (!node_.getParam("payload_center_of_mass_m", payload_center_of_mass_m_param))
  {
    ROS_WARN("No payload centre of mass specified for UrArmRobotHW. Assuming [0.0, 0.0, 0.0].");
  }
  if (payload_center_of_mass_m_param.size() != 3)
  {
    ROS_ERROR("Payload centre specified to UrArmRobotHW must be specified in 3 dimensions.");
    return false;
  }

  float speed_param;
  if (!node_.getParam("speed_scale", speed_param))
  {
    ROS_WARN("No speed scale specified for UrArmRobotHW. Assuming 0.5.");
    speed_param = 0.5;
  }
  if (!ur_.set_payload(payload_mass_kg_param, payload_center_of_mass_m_param))
  {
    ROS_ERROR("Failed to set initial robot payload.");
    return false;
  }
  if (!ur_.set_speed(speed_param))
  {
    ROS_ERROR("Failed to set initial robot speed.");
    return false;
  }

  set_teach_mode_server_ = node_.advertiseService("set_teach_mode", &UrArmRobotHW::setTeachMode, this);
  set_payload_server_ = node_.advertiseService("set_payload", &UrArmRobotHW::setPayload, this);
  // To be reinstated once speed-setting in moveit is connected up. See #SRC-1135.
  // set_speed_server_ = node_.advertiseService("set_speed", &UrArmRobotHW::setSpeed, this);
  ur_.start();

  latch_on_ = true;
  arms_ready_pub_ = n.advertise<std_msgs::Bool>(robot_id_ + "_arm_ready", 1, latch_on_);

  return true;
}


void UrArmRobotHW::read(const ros::Time& time, const ros::Duration& period)
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
    if (ur_.rs_client_->robot_state_received && first_read_)
    {
      arm_message_.data = true;
      arms_ready_pub_.publish(arm_message_);
      first_read_ = false;
    }
  }
}

void UrArmRobotHW::write(const ros::Time& time, const ros::Duration& period)
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

bool UrArmRobotHW::setTeachMode(sr_ur_msgs::SetTeachMode::Request &req, sr_ur_msgs::SetTeachMode::Response &resp)
{
  teach_mode_ = req.teach_mode;
  ur_.send_teach_mode_command(teach_mode_);
  resp.success = true;
  return true;
}

bool UrArmRobotHW::setPayload(sr_ur_msgs::SetPayload::Request &req, sr_ur_msgs::SetPayload::Response &resp)
{
  float mass_kg = req.mass_kg;
  float centre_of_mass_m[3] = {req.centre_of_mass_m.x, req.centre_of_mass_m.y, req.centre_of_mass_m.z};
  std::vector<float> payload_center_of_mass_m(centre_of_mass_m, centre_of_mass_m+3);
  if (!ur_.set_payload(mass_kg, payload_center_of_mass_m))
  {
    resp.success = false;
    return false;
  }
  ur_.send_payload_command();
  resp.success = true;
  return true;
}

bool UrArmRobotHW::setSpeed(sr_ur_msgs::SetSpeed::Request &req, sr_ur_msgs::SetSpeed::Response &resp)
{
  float speed = req.speed;
  if (!ur_.set_speed(speed))
  {
    resp.success = false;
    return false;
  }
  ur_.send_speed_command();
  resp.success = true;
  return true;
}
}  // namespace sr_ur_robot_hw
