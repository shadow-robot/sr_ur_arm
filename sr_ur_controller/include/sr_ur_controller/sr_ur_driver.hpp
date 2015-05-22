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
 * sr_ur_common.hpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#ifndef SR_UR_DRIVER_HPP_
#define SR_UR_DRIVER_HPP_

#include <stdint.h>
#include <pthread.h>

const size_t NUM_OF_JOINTS = 6;

// it is expected that joints with this name are configured in the urdf
// for bimanual systems a prefix may be added
const char ur_joints[NUM_OF_JOINTS][sizeof("shoulder_lift_joint")] = {"shoulder_pan_joint",
                                                                      "shoulder_lift_joint",
                                                                      "elbow_joint",
                                                                      "wrist_1_joint",
                                                                      "wrist_2_joint",
                                                                      "wrist_3_joint"};

const int UR_PERIOD = 8; // msec

struct UrEventLoop;
struct UrControlServer;
struct UrRobotStateClient;
struct UrProgramLoader;

struct UrRobotDriver
{
  bool robot_ready_to_move_;
  double target_positions_[NUM_OF_JOINTS];     // radians
  double previous_targets_[NUM_OF_JOINTS];     // radians

  // synchronize target_positions between controller and driver
  pthread_mutex_t write_mutex_;

  double joint_positions_     [NUM_OF_JOINTS]; // radians
  double joint_velocities_    [NUM_OF_JOINTS]; // radians/sec
  double joint_motor_currents_[NUM_OF_JOINTS]; // Amperes

  // synchronize robot state between controller and driver
  pthread_mutex_t robot_state_mutex_;

  // IP address of the robot
  char *robot_address_;

  // IP address of host PC for the connection to the robot
  char *host_address_;

  // path to a file containing a robot program
  char *robot_program_path_;

  // left or right
  char *robot_side_;

  UrEventLoop        *el_;
  UrControlServer    *ctrl_server_;
  UrRobotStateClient *rs_client_;
  UrProgramLoader    *pr_loader_;

  // The robot controller calls this to start communication with the robot controller.
  // A series of call-backs will be called and when everything is ready the flag robot_ready_to_move will be set.
  // The initialisation steps are :
  // 1. Connect to a server on the robot to get the robot state
  // 2. Start a server on the host to send commands to the robot and read responses
  // 3. Connect to a server on the robot and send the robot program
  // 4. The robot program will connect to the server on the host started before
  // 5. After that robot_ready_to_move will be set to true and the controller will be able to send commands
  void start();

  // when the controller is about to stop it should call this
  void stop();

  // restart the robot driver in case of an error
  void restart();

  // write commands to the robot
  void send_command();

  // send the command to set teach mode on/off to the robot
  void send_teach_mode_command(bool teach_mode);
};

#endif
