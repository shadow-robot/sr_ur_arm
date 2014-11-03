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
 * sr_ur_read_robot_state.cpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#include <ros/ros.h>
#include "sr_ur_controller/sr_ur_event_loop.hpp"
#include "sr_ur_controller/sr_ur_common.hpp"
#include "sr_ur_controller/sr_ur_hardware_messages.hpp"
#include "sr_ur_controller/sr_ur_read_write.hpp"

#define ROS_ASSERT_ENABLED

double robot_joint_positions[NUM_OF_JOINTS];      // rad
double robot_joint_velocities[NUM_OF_JOINTS];     // rad/sec
double robot_joint_motor_currents[NUM_OF_JOINTS]; // Amperes
pthread_mutex_t robot_state_mutex;

static uv_connect_t robot_state_client_connection_request;
static uv_tcp_t     robot_state_stream;
static uv_buf_t     robot_state_buffer;
static bool         robot_state_received;

// reuse the preallocated buffer for storing the robot state data
// that the server in the robot at port 30003 reports back
static uv_buf_t alloc_robot_state_buf(uv_handle_t* robot_state_stream_handle, size_t)
{
  ROS_ASSERT(robot_state_stream_handle);
  return robot_state_buffer;
}

// swap the endianess of a double (64bit)
// this is application specific as it is known that the host uses little
// and the network big endian. It may not work in other systems.
static double ntohd(double *big_endian_number)
{
  double little_endian_number;
  char *big_endian_data    = (char*)big_endian_number;
  char *little_endian_data = (char*)&little_endian_number;

  size_t length = sizeof(double);
  for (size_t i = 0; i < length; ++i)
  {
    little_endian_data[i] = big_endian_data[length - i - 1];
  }
  return little_endian_number;
}

// parse the robot state telegram into the containers that the controller can access
// this is called whenever the server in the robot at 30003 sends a telegram
static void robot_state_received_cb(uv_stream_t* p_robot_state_stream,
                                    ssize_t      number_of_chars_received,
                                    uv_buf_t     in_robot_state_buffer)
{
  ROS_ASSERT(p_robot_state_stream);
  ROS_ASSERT(in_robot_state_buffer.base);
  ROS_ASSERT(sizeof(ur_short_robot_state) <= number_of_chars_received);

  pthread_mutex_lock(&robot_state_mutex);
  ur_robot_state *robot_state = (ur_robot_state*)in_robot_state_buffer.base;
  ROS_ASSERT(sizeof(ur_short_robot_state) <= robot_state->message_size_);

  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    robot_joint_positions[i]      = ntohd(&robot_state->actual_positions_[i]);
    robot_joint_velocities[i]     = ntohd(&robot_state->actual_velocities_[i]);
    robot_joint_motor_currents[i] = ntohd(&robot_state->actual_currents_[i]);
    if (!robot_state_received)
    {
      target_positions[i] = robot_joint_positions[i];
    }
  }
  pthread_mutex_unlock(&robot_state_mutex);

  if (!robot_state_received)
  {
    start_read_write();
    robot_state_received = true;
    ROS_INFO("UrArmController started receiving robot state");
  }
}

// a client in the host PC has successfully connected to a server in the robot at 30003
// to receive the robot state every 8ms
static void robot_state_client_connected_cb(uv_connect_t* connection_request, int status)
{
  ROS_ASSERT(connection_request == &robot_state_client_connection_request);
  ROS_ASSERT(0 == status);

  robot_state_buffer.base = (char*)malloc(sizeof(ur_robot_state));
  robot_state_buffer.len  = sizeof(ur_robot_state);

  status = uv_read_start(connection_request->handle,
                         alloc_robot_state_buf,
                         robot_state_received_cb);
  ROS_ASSERT(0 == status);
}

static void start_reading_robot_state()
{
  pthread_mutex_init(&robot_state_mutex, NULL);

  // initialise stream for client that receives robot state
  int status = uv_tcp_init(get_event_loop(), &robot_state_stream);
  ROS_ASSERT(0 == status);
  uv_tcp_nodelay(&robot_state_stream, 0);

  sockaddr_in robot_state_client_address = uv_ip4_addr(robot_address, 30003);
  status = uv_tcp_connect(&robot_state_client_connection_request,
                          &robot_state_stream,
                          robot_state_client_address,
                          robot_state_client_connected_cb);
  ROS_ASSERT(0 == status);
}

static void stop_reading_robot_state()
{
  pthread_mutex_destroy(&robot_state_mutex);
  uv_close((uv_handle_t*)&robot_state_stream, NULL);
  free(robot_state_buffer.base);
  stop_read_write();
}

// The robot controller calls this to start communication with the robot controller.
// A series of call-backs will be called and when everything is ready the flag robot_ready_to_move will be set.
// The initialisation steps are :
// 1. Connect to a server on the robot to get the robot state
// 2. Start a server on the host to send commands to the robot and read responses
// 3. Connect to a server on the robot and send the robot program
// 4. The robot program will connect to the server on the host started before
// 5. After that robot_ready_to_move will be set to true and the controller will be able to send commands
void start_communication_with_robot()
{
  ROS_INFO("UrArmController starts communication with robot");
  ROS_ASSERT(robot_address);
  ROS_ASSERT(host_address);

  start_reading_robot_state();
  start_event_loop();
}

void stop_communication_with_robot()
{
  ROS_INFO("UrArmController stops communicating with robot");
  stop_reading_robot_state();
}
