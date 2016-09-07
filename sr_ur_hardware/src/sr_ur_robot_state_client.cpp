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

#define ROS_ASSERT_ENABLED
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>

#include <sr_ur_robot_hw/sr_ur_driver.hpp>
#include "sr_ur_robot_hw/sr_ur_hardware_messages.hpp"
#include "sr_ur_robot_hw/sr_ur_event_loop.hpp"
#include "sr_ur_robot_hw/sr_ur_control_server.hpp"
#include "sr_ur_robot_hw/sr_ur_robot_state_client.hpp"

const int ROBOT_STATE_PORT = 30003;

// reuse the preallocated buffer for storing the robot state data
// that the server in the robot at port ROBOT_STATE_PORT reports back to
static uv_buf_t allocate_robot_state_buffer(uv_handle_t* state_stream, size_t)
{
  ROS_ASSERT(state_stream);
  ROS_ASSERT(state_stream->data);
  UrRobotStateClient *rs_client = (UrRobotStateClient*) state_stream->data;
  return rs_client->buffer_;
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
// this is called whenever the server in the robot at ROBOT_STATE_PORT sends a telegram
static void robot_state_received_cb(uv_stream_t* state_stream,
                                    ssize_t      number_of_chars_received,
                                    uv_buf_t     buffer)
{
  ROS_ASSERT(state_stream);
  ROS_ASSERT(state_stream->data);
  UrRobotStateClient *rs_client = (UrRobotStateClient*) state_stream->data;

  ROS_ASSERT(buffer.base);

  pthread_mutex_lock(&rs_client->ur_->robot_state_mutex_);
  char *pdata;

  if (rs_client->protocol_version.empty())
  {
    if (sizeof(ur_robot_state_v3_0) == number_of_chars_received)
    {
      ROS_WARN_STREAM("UR robot state protocol v3.0 or v3.1");
      rs_client->protocol_version = "3.0";
      pdata = buffer.base;
    }
    else if (sizeof(ur_robot_state) == number_of_chars_received)
    {
      ROS_WARN_STREAM("UR robot state protocol v3.2");
      rs_client->protocol_version = "3.2";
      pdata = buffer.base;
    }
  }
  else if (rs_client->protocol_version == "3.0")
  {
    if (sizeof(ur_robot_state_v3_0) == number_of_chars_received)
    {
      pdata = buffer.base;
    }
    else if (sizeof(ur_short_robot_state) + sizeof(ur_robot_state_v3_0) <= number_of_chars_received)
    {
      pdata = buffer.base + sizeof(ur_robot_state_v3_0);
    }
    else
    {
      pthread_mutex_unlock(&rs_client->ur_->robot_state_mutex_);
      return;
    }
  }
  else if (rs_client->protocol_version == "3.2")
  {
    if (sizeof(ur_robot_state) == number_of_chars_received)
    {
      pdata = buffer.base;
    }
    else if (sizeof(ur_short_robot_state) + sizeof(ur_robot_state) <= number_of_chars_received)
    {
      pdata = buffer.base + sizeof(ur_robot_state);
    }
    else
    {
      pthread_mutex_unlock(&rs_client->ur_->robot_state_mutex_);
      return;
    }
  }

  ur_robot_state *robot_state = (ur_robot_state*) pdata;

  // the range of motion of each robot joint is 2 full rotations
  // if a value is reported outside these limits it's a communication error
  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    if (ntohd(&robot_state->actual_positions_[i]) >  4*M_PI ||
        ntohd(&robot_state->actual_positions_[i]) < -4*M_PI)
    {
      pthread_mutex_unlock(&rs_client->ur_->robot_state_mutex_);
      return;
    }
  }

  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    rs_client->ur_->joint_positions_     [i] = ntohd(&robot_state->actual_positions_ [i]);
    rs_client->ur_->joint_velocities_    [i] = ntohd(&robot_state->actual_velocities_[i]);
    rs_client->ur_->joint_motor_currents_[i] = ntohd(&robot_state->actual_currents_  [i]);
    if (!rs_client->robot_state_received)
    {
      rs_client->ur_->target_positions_[i] = rs_client->ur_->joint_positions_[i];
      rs_client->ur_->previous_targets_[i] = rs_client->ur_->joint_positions_[i];
    }
  }
  pthread_mutex_unlock(&rs_client->ur_->robot_state_mutex_);

  if (!rs_client->robot_state_received)
  {
    rs_client->ur_->ctrl_server_->start();
    rs_client->robot_state_received = true;
    ROS_WARN("UrArmController of %s robot started receiving robot state from address %s and port %d",
             rs_client->ur_->robot_side_, rs_client->ur_->robot_address_, ROBOT_STATE_PORT);
  }
}

// a client in the host PC has successfully connected to a server in the robot at ROBOT_STATE_PORT
static void robot_state_client_connected_cb(uv_connect_t* connection_request, int status)
{
  UrRobotStateClient *rs_client = (UrRobotStateClient*) connection_request->data;
  ROS_ASSERT(0 == status);
  ROS_ASSERT(connection_request == &rs_client->connection_request_);

  rs_client->buffer_.base = (char*)malloc(2*sizeof(ur_robot_state));
  rs_client->buffer_.len  = 2*sizeof(ur_robot_state);

  status = uv_read_start(connection_request->handle,
                         allocate_robot_state_buffer,
                         robot_state_received_cb);
  ROS_ASSERT(0 == status);
}

void UrRobotStateClient::start()
{
  ROS_ASSERT(ur_);
  ROS_ASSERT(ur_->robot_address_);
  ROS_ASSERT(ur_->host_address_);

  tcp_stream_.data         = (void*) this;
  connection_request_.data = (void*) this;

  pthread_mutex_init(&ur_->robot_state_mutex_, NULL);

  // initialise stream for client that receives robot state
  int status = uv_tcp_init(ur_->el_->get_event_loop(), &tcp_stream_);
  ROS_ASSERT(0 == status);
  uv_tcp_nodelay(&tcp_stream_, 0);

  sockaddr_in robot_state_client_address = uv_ip4_addr(ur_->robot_address_, ROBOT_STATE_PORT);
  status = uv_tcp_connect(&connection_request_,
                          &tcp_stream_,
                          robot_state_client_address,
                          robot_state_client_connected_cb);
  ROS_ASSERT(0 == status);
}

void UrRobotStateClient::stop()
{
  pthread_mutex_destroy(&ur_->robot_state_mutex_);
  uv_close((uv_handle_t*)&tcp_stream_, NULL);
  free(buffer_.base);
  ur_->ctrl_server_->stop();
}
