/* Copyright (c) 2014, Shadow Robot Company, All rights reserved.
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
 * sr_ur_driver.cpp
 *
 *  Created on: 6 Nov 2014
 *      Author: Manos Nikolaidis
 */

#define ROS_ASSERT_ENABLED
#include <ros/ros.h>

#include "sr_ur_controller/sr_ur_driver.hpp"
#include "sr_ur_controller/sr_ur_robot_state_client.hpp"
#include "sr_ur_controller/sr_ur_event_loop.hpp"
#include "sr_ur_controller/sr_ur_control_server.hpp"
#include "sr_ur_controller/sr_ur_program_loader.hpp"

void UrRobotDriver::start()
{
  ROS_INFO("UrArmController starts communication with robot");

  rs_client_   = new UrRobotStateClient();
  ctrl_server_ = new UrControlServer();
  pr_loader_   = new UrProgramLoader();
  el_          = new UrEventLoop();

  rs_client_  ->ur_ = this;
  ctrl_server_->ur_ = this;
  pr_loader_  ->ur_ = this;
  el_         ->ur_ = this;

  rs_client_->start();
  el_       ->start();
}

void UrRobotDriver::stop()
{
  ROS_INFO("UrArmController stops communicating with the %s robot", robot_side_);

  el_         ->stop();
  ctrl_server_->stop();
  rs_client_  ->stop();

  delete rs_client_;
  delete el_;
  delete ctrl_server_;
  delete pr_loader_;
}

void UrRobotDriver::restart()
{
  ROS_INFO("UrArmController restarting communication with the %s robot", robot_side_);
  stop();
  start();
}

void UrRobotDriver::send_command()
{
  ctrl_server_->send_servo_command();
}
