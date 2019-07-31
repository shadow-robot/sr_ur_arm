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
 * sr_ur_robot_hw.hpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis, Dan Greenwald
 */

#ifndef SR_UR_ROBOT_HW_HPP_
#define SR_UR_ROBOT_HW_HPP_

#include <ros/node_handle.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ur_robot_hw/sr_ur_driver.hpp>
#include <sr_ur_msgs/SetTeachMode.h>
#include <sr_ur_msgs/SetPayload.h>
#include <sr_ur_msgs/SetSpeed.h>
#include <string>
#include <vector>

namespace sr_ur_robot_hw
{
class UrArmRobotHW : public hardware_interface::RobotHW
{
public:
  UrArmRobotHW();
  virtual ~UrArmRobotHW() {}

  virtual bool init(ros::NodeHandle &n, ros::NodeHandle &robot_hw_nh);
  virtual void read(const ros::Time& time, const ros::Duration& period);
  virtual void write(const ros::Time& time, const ros::Duration& period);

protected:
  ros::NodeHandle node_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;

  ros::Publisher arms_ready_pub_;

  std::vector<double> joint_position_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  // used for throttling loop at 8ms with master loop at 1ms
  int loop_count_;

  // used to flag first read, for to signal ready to load trajectory_controller
  bool first_read_;

  // identify left or right arm
  std::string robot_id_;

  // left or right arm prefix for joints
  std::string joint_prefix_;

  UrRobotDriver ur_;

  bool setTeachMode(sr_ur_msgs::SetTeachMode::Request &req, sr_ur_msgs::SetTeachMode::Response &resp);

  bool setPayload(sr_ur_msgs::SetPayload::Request &req, sr_ur_msgs::SetPayload::Response &resp);

  bool setSpeed(sr_ur_msgs::SetSpeed::Request &req, sr_ur_msgs::SetSpeed::Response &resp);

  bool teach_mode_;
  ros::ServiceServer set_teach_mode_server_;
  ros::ServiceServer set_payload_server_;
  ros::ServiceServer set_speed_server_;
};
}  // namespace sr_ur_robot_hw
#endif
