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

/**
 * @file   sr_ur_hardware_messages.hpp
 * @author Manos Nikolaidis <manos@shadowrobot.com>
 * @brief  messages the driver may send to the ur arm controller
 **/

#ifndef SR_UR_HARDWARE_MESSAGES_HPP_
#define SR_UR_HARDWARE_MESSAGES_HPP_

#include <stdint.h>

struct ur_quit
{
  int32_t message_type_;
}__attribute__((packed));

struct ur_stop
{
  int32_t message_type_;
}__attribute__((packed));

struct ur_servoj
{
  int32_t message_type_;
  int32_t commanded_positions_[NUM_OF_JOINTS];
}__attribute__((packed));

struct ur_set_teach_mode
{
  int32_t message_type_;
  int32_t teach_mode_;
}__attribute__((packed));

// size needed to get up actual currents in ur_robot_state
const size_t MIN_SIZE = (8*6+1)*sizeof(double) + sizeof(int32_t);

// the contents of the telegram from the ur arm controller to the control PC
struct ur_robot_state
{
  int32_t message_size_;
  double time_;
  double target_positions[NUM_OF_JOINTS];
  double target_velocities_[NUM_OF_JOINTS];
  double target_accelerations_[NUM_OF_JOINTS];
  double target_currents_[NUM_OF_JOINTS];
  double target_torques_[NUM_OF_JOINTS];
  double actual_positions_[NUM_OF_JOINTS];
  double actual_velocities_[NUM_OF_JOINTS];
  double actual_currents_[NUM_OF_JOINTS];
  double joint_control_torques_[NUM_OF_JOINTS];
  double actual_tool_coordinates_[NUM_OF_JOINTS];
  double actual_tool_speed_[NUM_OF_JOINTS];
  double generalised_tool_forces_[NUM_OF_JOINTS];
  double target_tool_coordinates_[NUM_OF_JOINTS];
  double target_tool_speed_[NUM_OF_JOINTS];
  int64_t digital_pins_;
  double motor_temperatures_[NUM_OF_JOINTS];
  double controller_timer_;
  double test_value_;
  int64_t robot_mode_;
  int64_t joint_modes_[NUM_OF_JOINTS];
  int64_t safety_mode_;
  int64_t unused_1_[6];
  double tool_accelerometer_values_[3];
  int64_t unused_2_[6];
  double speed_scaling_;
  double linear_momentum_norm_;
  int64_t unused_3_[2];
  double masterboard_main_voltage_;
  double masterboard_robot_voltage_;
  double masterboard_robot_current_;
  double actual_joint_voltages_[NUM_OF_JOINTS];
}__attribute__((packed));

#endif
