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
 * sr_ur_read_write.cpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#define ROS_ASSERT_ENABLED
#include <ros/assert.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <uv.h>
#include "sr_ur_controller/sr_ur_common.hpp"
#include "sr_ur_controller/sr_ur_event_loop.hpp"
#include "sr_ur_controller/sr_ur_hardware_messages.hpp"
#include "sr_ur_controller/sr_ur_program_loader.hpp"

const int32_t MSG_QUIT           = 1;
const int32_t MSG_STOPJ          = 3;
const int32_t MSG_SERVOJ         = 5;
const int32_t MSG_SET_TEACH_MODE = 7;
const double  MULT_JOINTSTATE    = 10000.0;

static uv_write_t command_write_request;

static uv_tcp_t   server_stream;
static uv_tcp_t   command_stream;

static uv_buf_t   command_buffer;
static uv_buf_t   response_to_command_buffer;

// reuse the preallocated buffer for storing the robot's response
// when a command is send from the server in the host to the client in the robot
static uv_buf_t alloc_response_to_command_buf(uv_handle_t* command_stream_handle, size_t)
{
  ROS_ASSERT(command_stream_handle);
  return response_to_command_buffer;
}

// this is called after a new command has been sent to the robot
// it just checks the status
static void command_sent_cb(uv_write_t* p_command_write_request, int status)
{
  ROS_ASSERT(p_command_write_request);
  ROS_ASSERT(0 == status);
}

// send a command to the robot to quit the currently running program
// meant to be sent at system shutdown
static void send_ur_quit()
{
  ur_quit *telegram = (ur_quit*)command_buffer.base;
  memset(telegram, 0, sizeof(ur_servoj));
  telegram->message_type_ = htonl(MSG_QUIT);

  int status = uv_write(&command_write_request,
                        (uv_stream_t*)&command_stream,
                        &command_buffer,
                        1,
                        command_sent_cb);
  ROS_ASSERT(0 == status);
}

// send a command to the robot to stop if it is currently moving
// meant to be sent at system startup and shutdown
static void send_ur_stop()
{
  ur_stop *telegram = (ur_stop*)command_buffer.base;
  memset(telegram, 0, sizeof(ur_servoj));
  telegram->message_type_ = htonl(MSG_STOPJ);

  int status = uv_write(&command_write_request,
                        (uv_stream_t*)&command_stream,
                        &command_buffer,
                        1,
                        command_sent_cb);
  ROS_ASSERT(0 == status);
}

// send a command to the robot to set/reset the teach mode according to the teach_mode argument
// the teach mode is using for recording trajectories and if not reset will inhibit servo commands
// meant to be sent at system startup
static void send_ur_set_teach_mode(bool teach_mode)
{
  ur_set_teach_mode *telegram = (ur_set_teach_mode*)command_buffer.base;
  memset(telegram, 0, sizeof(ur_servoj));
  telegram->message_type_ = htonl(MSG_SET_TEACH_MODE);
  int32_t mode = teach_mode ? htonl(1) : htonl(0);
  telegram->teach_mode_ = htonl(mode);

  int status = uv_write(&command_write_request,
                        (uv_stream_t*)&command_stream,
                        &command_buffer,
                        1,
                        command_sent_cb);
  ROS_ASSERT(0 == status);
}

// Called when the robot replies to a command that was sent earlier from the server to the host
static void received_response_to_command_cb(uv_stream_t* p_command_stream,
                                            ssize_t      number_of_chars_received,
                                            uv_buf_t     in_response_to_command_buffer)
{
  ROS_ASSERT(p_command_stream);
  ROS_ASSERT(p_command_stream->data);
  UrRobotData *ur = (UrRobotData*) p_command_stream->data;

  ROS_ASSERT(in_response_to_command_buffer.base);

  in_response_to_command_buffer.base[number_of_chars_received] = '\0';
  ROS_INFO("Robot replied : %s", in_response_to_command_buffer.base);

  // During startup these messages are expected in this order
  if (strcmp(in_response_to_command_buffer.base, "Connected") == 0)
  {
    ROS_INFO("Asking the robot to stop");
    send_ur_stop();
  }
  else if (!ur->robot_ready_to_move && strcmp(in_response_to_command_buffer.base, "Stop") == 0)
  {
    ROS_INFO("Asking to reset the teach mode");
    send_ur_set_teach_mode(false);
  }
  else if (!ur->robot_ready_to_move && strcmp(in_response_to_command_buffer.base, "Teach mode") == 0)
  {
    ROS_INFO("Robot is ready to receive servo commands");
    ur->robot_ready_to_move = true;
  }
}

// a client in the robot controller successfully connected to a listening server at the host
// commands can be send to the robot through this connection
static void command_server_received_connection_cb(uv_stream_t* p_server_stream, int status)
{
  ROS_INFO("Received connection from robot");
  ROS_ASSERT(p_server_stream);
  ROS_ASSERT(p_server_stream == (uv_stream_t*)&server_stream);
  ROS_ASSERT(0 == status);

  command_buffer.base = (char*)malloc(sizeof(ur_servoj));
  command_buffer.len  = sizeof(ur_servoj);
  response_to_command_buffer.base = (char*)malloc(512);
  // reserve extra byte for null termination with a full buffer
  response_to_command_buffer.len  = 512 - 1;

  status = uv_tcp_init(get_event_loop(), &command_stream);
  ROS_ASSERT(0 == status);
  uv_tcp_nodelay(&command_stream, 0);

  status = uv_accept(p_server_stream, (uv_stream_t*)&command_stream);
  ROS_ASSERT(0 == status);

  command_stream.data = server_stream.data;
  status = uv_read_start((uv_stream_t*)&command_stream,
                         alloc_response_to_command_buf,
                         received_response_to_command_cb);
  ROS_ASSERT(0 == status);

  ROS_INFO("Robot controller successfully connected to the host");
}

void start_read_write(UrRobotData* ur)
{
  pthread_mutex_init(&ur->write_mutex, NULL);

  // Initialise stream for server that sends commands to the robot.
  // After this server accepts a connection from a client in the robot
  // a new stream will be created that will actually be used for the commands
  int status = uv_tcp_init(get_event_loop(), &server_stream);
  ROS_ASSERT(0 == status);
  uv_tcp_nodelay(&server_stream, 0);

  server_stream.data = (void*) ur;

  // assign port to zero and let the OS select an available one
  sockaddr_in server_address = uv_ip4_addr(ur->host_address, 0);
  status = uv_tcp_bind(&server_stream, server_address);
  ROS_ASSERT(0 == status);

  // get the port that the OS assigned
  int length;
  sockaddr address_with_bound_port;
  status = uv_tcp_getsockname(&server_stream, &address_with_bound_port, &length);
  ROS_ASSERT(0 == status);
  int reverse_port = ntohs(((struct sockaddr_in*)&address_with_bound_port)->sin_port);

  status = uv_listen((uv_stream_t*)&server_stream, 1, command_server_received_connection_cb);
  ROS_ASSERT(0 == status);
  ROS_WARN("UrArmController started server on address %s and listening on port %d",
           ur->host_address, reverse_port);

  // after the robot program is loaded and has started running
  // it will attempt to connect on server_stream
  load_robot_program(reverse_port);

  ROS_ASSERT(0 == status);
}

void stop_read_write(UrRobotData* ur)
{
  ROS_INFO("UrArmController stops read and write");
  send_ur_stop();
  send_ur_quit();

  pthread_mutex_destroy(&ur->write_mutex);

  uv_close((uv_handle_t*)&server_stream, NULL);
  uv_close((uv_handle_t*)&command_stream, NULL);

  stop_event_loop();

  free(command_buffer.base);
  free(response_to_command_buffer.base);
  free(ur->robot_address);
  free(ur->host_address);
}

void send_command_to_robot(UrRobotData* ur)
{
  ur_servoj *telegram = (ur_servoj*)command_buffer.base;
  telegram->message_type_ = htonl(MSG_SERVOJ);

  pthread_mutex_lock(&ur->write_mutex);
  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    telegram->commanded_positions_[i] = htonl((int32_t)(MULT_JOINTSTATE * ur->target_positions[i]));
  }
  pthread_mutex_unlock(&ur->write_mutex);

  int status = uv_write(&command_write_request,
                        (uv_stream_t*)&command_stream,
                        &command_buffer,
                        1,
                        command_sent_cb);
  ROS_ASSERT(0 == status);
}
