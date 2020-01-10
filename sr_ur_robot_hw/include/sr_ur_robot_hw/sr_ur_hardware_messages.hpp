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
* @file   sr_ur_hardware_messages.hpp
* @author Manos Nikolaidis <manos@shadowrobot.com>
* @brief  messages the driver may send to the ur arm controller
**/

#ifndef SR_UR_HARDWARE_MESSAGES_HPP_
#define SR_UR_HARDWARE_MESSAGES_HPP_

#include <stdint.h>

// message to quit the current robot program
struct __attribute__((packed)) ur_quit
{
  int32_t message_type_;
};

// message to (abruptly) stop the motion of the robot
struct __attribute__((packed)) ur_stop
{
  int32_t message_type_;
};

// message to move the robot to a new waypoint in joint space
struct __attribute__((packed)) ur_servoj
{
  int32_t message_type_;
  int32_t commanded_positions_[NUM_OF_JOINTS];
};

// message to set or reset the teach mode
struct __attribute__((packed)) ur_set_teach_mode
{
  int32_t message_type_;
  int32_t teach_mode_;
};

// message to set the robot payload.
struct __attribute__((packed)) ur_set_payload
{
  int32_t message_type_;
  int32_t payload_mass_g_;
  int32_t payload_coords_mm_[3];
};

// message to set the robot speed scale.
struct __attribute__((packed)) ur_set_speed
{
  int32_t message_type_;
  int32_t speed_;
};

// the contents of the telegram from the ur arm controller to the control PC
// This is valid for versions 3.0 and 3.1 of the arm controller SW
struct __attribute__((packed)) ur_robot_state_v3_0
{
  int32_t message_size_;
  double time_;
  double target_positions_[NUM_OF_JOINTS];
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
};

// the contents of the telegram from the ur arm controller to the control PC
// This is valid for version 3.2 to 3.4 of the arm controller SW
struct __attribute__((packed)) ur_robot_state_v3_2
{
  int32_t message_size_;
  double time_;
  double target_positions_[NUM_OF_JOINTS];
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
  double digital_outputs_;
  double program_state_;
};

// the contents of the telegram from the ur arm controller to the control PC
// This is valid for version 3.5, 3.6, 3.7, 3.8, 3.9 of the arm controller SW
struct __attribute__((packed)) ur_robot_state_v3_5
{
  int32_t message_size_;
  double time_;
  double target_positions_[NUM_OF_JOINTS];
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
  double digital_outputs_;
  double program_state_;
  double elbow_position_[3];
  double elbow_velocity_[3];
};

// the contents of the telegram from the ur arm controller to the control PC
// This is valid for version 3.10, 3.11, 3.? of the arm controller SW
struct __attribute__((packed)) ur_robot_state_v3_10
{
  int32_t message_size_;
  double time_;
  double target_positions_[NUM_OF_JOINTS];
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
  double digital_outputs_;
  double program_state_;
  double elbow_position_[3];
  double elbow_velocity_[3];
  double safety_status_;
};

// shortened telegram of ur_robot_state
struct __attribute__((packed)) ur_short_robot_state
{
  int32_t message_size_;
  double time_;
  double target_positions_[NUM_OF_JOINTS];
  double target_velocities_[NUM_OF_JOINTS];
  double target_accelerations_[NUM_OF_JOINTS];
  double target_currents_[NUM_OF_JOINTS];
  double target_torques_[NUM_OF_JOINTS];
  double actual_positions_[NUM_OF_JOINTS];
  double actual_velocities_[NUM_OF_JOINTS];
  double actual_currents_[NUM_OF_JOINTS];
};
#endif
