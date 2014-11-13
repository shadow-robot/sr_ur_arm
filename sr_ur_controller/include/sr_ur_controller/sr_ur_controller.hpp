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
 * sr_ur_controller.hpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#ifndef SR_UR_CONTROLLER_HPP_
#define SR_UR_CONTROLLER_HPP_

#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <ros_ethercat_model/robot_state.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ur_controller/sr_ur_driver.hpp>

namespace sr_ur
{
class UrArmController : public controller_interface::Controller<ros_ethercat_model::RobotState>
{
public:
  UrArmController();

  virtual bool init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);
  virtual void starting(const ros::Time&);
  virtual void stopping(const ros::Time&);
  virtual void update(const ros::Time&, const ros::Duration&);

protected:
  ros::NodeHandle node_;

  ros_ethercat_model::RobotState *robot_;
  ros_ethercat_model::JointState *joint_states_[NUM_OF_JOINTS];

  // used for throttling loop at 8ms with master loop at 1ms
  int loop_count_;

  // identify left or right arm
  std::string robot_id_;

  // left or right arm prefix for joints
  std::string joint_prefix_;

  UrRobotDriver ur_;

  void enforceLimits(double *targets);
  virtual void setCommandCB(const std_msgs::Float64MultiArrayConstPtr& msg);

  ros::Subscriber sub_command_;
};
}

#endif
