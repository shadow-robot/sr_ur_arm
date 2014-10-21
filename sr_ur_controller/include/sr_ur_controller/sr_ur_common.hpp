/* Copyright (c) 2013, Shadow Robot Company, All rights reserved.
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
 * sr_ur_read_writed.hpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#ifndef SR_UR_COMMON_HPP_
#define SR_UR_COMMON_HPP_

#include <stdint.h>

const size_t NUM_OF_JOINTS = 6;

// it is expected that joints with this name are configured in the urdf
// for bimanual systems a prefix may be added
const char ur_joints[NUM_OF_JOINTS][sizeof("shoulder_lift_joint")] = {"shoulder_pan_joint",
                                                                      "shoulder_lift_joint",
                                                                      "elbow_joint",
                                                                      "wrist_1_joint",
                                                                      "wrist_2_joint",
                                                                      "wrist_3_joint"};

extern double target_positions[NUM_OF_JOINTS]; // radians
extern pthread_mutex_t write_mutex;

extern double robot_joint_positions[NUM_OF_JOINTS]; // radians
extern double robot_joint_velocities[NUM_OF_JOINTS]; // radians/sec
extern double robot_joint_motor_currents[NUM_OF_JOINTS]; // Amperes
extern pthread_mutex_t robot_state_mutex;

extern char *robot_address;
extern char *host_address;
extern char *robot_program_path;

// when the controller is ready it should call this to start communication with the ur arm controller
void start_communication_with_robot();

// when the controller is about to stop it should call this
void stop_communication_with_robot();

// write commands to the robot
void send_command_to_robot();

#endif
