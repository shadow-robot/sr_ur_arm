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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <netinet/in.h>

#include "sr_ur_controller/sr_ur_driver.hpp"
#include "sr_ur_controller/sr_ur_event_loop.hpp"
#include "sr_ur_controller/sr_ur_hardware_messages.hpp"
#include "sr_ur_controller/sr_ur_program_loader.hpp"
#include "sr_ur_controller/sr_ur_control_server.hpp"

const int32_t MSG_QUIT           = 1;
const int32_t MSG_STOPJ          = 3;
const int32_t MSG_SERVOJ         = 5;
const int32_t MSG_SET_TEACH_MODE = 7;
const double MULT_JOINTSTATE     = 10000.0;

// reuse the preallocated buffer for storing the robot's response
// when a command is send from the server in the host to the client in the robot
static uv_buf_t allocate_response_buffer(uv_handle_t* command_stream, size_t)
{
  ROS_ASSERT(command_stream);
  ROS_ASSERT(command_stream->data);

  UrControlServer *ctrl_server = (UrControlServer*)command_stream->data;
  return ctrl_server->response_buffer_;
}

// this is called after a new command has been sent to the robot
// it just checks the status
static void command_sent_cb(uv_write_t* write_request, int status)
{
  ROS_ASSERT(0 == status);
  ROS_ASSERT(write_request);
}

// Called when the robot replies to a command that was sent earlier from the server to the host
static void received_response_cb(uv_stream_t* command_stream,
                                 ssize_t      number_of_chars_received,
                                 uv_buf_t     buffer)
{
  ROS_ASSERT(command_stream);
  ROS_ASSERT(command_stream->data);

  UrControlServer *ctrl_server = (UrControlServer*)command_stream->data;
  ROS_ASSERT(command_stream == (uv_stream_t* )&ctrl_server->command_stream_);
  ROS_ASSERT(buffer.base);

  // the robot occasionally sends meaningless empty messages without further consequences
  if (number_of_chars_received < 2)
  {
    return;
  }

  buffer.base[number_of_chars_received] = '\0';
  ROS_INFO("Robot at %s replied : %s", ctrl_server->ur_->robot_address_, buffer.base);

  // During startup these messages are expected in this order
  if (strcmp(buffer.base, "Connected") == 0)
  {
    ROS_INFO("Asking the robot to stop");
    ctrl_server->send_ur_stop();
  }
  else if (!ctrl_server->ur_->robot_ready_to_move_ && strcmp(buffer.base, "Stop") == 0)
  {
    ROS_INFO("Asking to reset the teach mode");
    ctrl_server->send_ur_set_teach_mode(false);
  }
  else if (!ctrl_server->ur_->robot_ready_to_move_ && strcmp(buffer.base, "Teach mode") == 0)
  {
    ROS_WARN("Robot at %s is ready to receive servo commands", ctrl_server->ur_->robot_address_);
    ctrl_server->ur_->robot_ready_to_move_ = true;
  }
}

// a client in the robot controller successfully connected to a listening server at the host
// commands can be send to the robot through this connection
static void received_connection_cb(uv_stream_t* server_stream, int status)
{
  ROS_ASSERT(server_stream);
  UrControlServer *ctrl_server = (UrControlServer*)server_stream->data;
  ROS_ASSERT(server_stream == (uv_stream_t* )&ctrl_server->server_stream_);

  ROS_INFO("Received connection from robot");

  status = uv_tcp_init(ctrl_server->ur_->el_->get_event_loop(), &ctrl_server->command_stream_);
  ROS_ASSERT(0 == status);
  uv_tcp_nodelay(&ctrl_server->command_stream_, 0);

  status = uv_accept(server_stream, (uv_stream_t*)&ctrl_server->command_stream_);
  ROS_ASSERT(0 == status);

  status = uv_read_start((uv_stream_t*)&ctrl_server->command_stream_,
                         allocate_response_buffer,
                         received_response_cb);
  ROS_ASSERT(0 == status);

  ROS_INFO("Robot controller successfully connected to the host");
}

void UrControlServer::start()
{
  ROS_ASSERT(ur_);
  pthread_mutex_init(&ur_->write_mutex_, NULL);

  // Initialise stream for server that sends commands to the robot.
  // After this server accepts a connection from a client in the robot
  // a new stream will be created that will actually be used for the commands
  int status = uv_tcp_init(ur_->el_->get_event_loop(), &server_stream_);
  ROS_ASSERT(0 == status);
  uv_tcp_nodelay(&server_stream_, 0);

  // assign port to zero and let the OS select an available one
  sockaddr_in server_address = uv_ip4_addr(ur_->host_address_, 0);
  status = uv_tcp_bind(&server_stream_, server_address);
  ROS_ASSERT(0 == status);

  // get the port that the OS assigned
  sockaddr address_with_bound_port;
  int length = (int)sizeof(sockaddr);
  status = uv_tcp_getsockname(&server_stream_, &address_with_bound_port, &length);
  ROS_ASSERT(0 == status);
  int reverse_port = ntohs(((struct sockaddr_in*)&address_with_bound_port)->sin_port);

  server_stream_.data   = (void*)this;
  command_stream_.data  = (void*)this;
  write_request_.data   = (void*)this;

  command_buffer_.base  = (char*)malloc(sizeof(ur_servoj));
  command_buffer_.len   = sizeof(ur_servoj);
  response_buffer_.base = (char*)malloc(512);
  response_buffer_.len  = 512 - 1;

  status = uv_listen((uv_stream_t*)&server_stream_, 1, received_connection_cb);
  ROS_ASSERT(0 == status);
  ROS_WARN("UrArmController started server on address %s and listening on port %d",
           ur_->host_address_,
           reverse_port);

  // after the robot program is loaded and has started running
  // it will attempt to connect on server_stream_
  ur_->pr_loader_->send_program(reverse_port);

  ROS_ASSERT(0 == status);
}

void UrControlServer::stop()
{
  ROS_ASSERT(ur_);
  ROS_INFO("UrArmController stops the control server");
  send_ur_stop();
  send_ur_quit();

  pthread_mutex_destroy(&ur_->write_mutex_);

  uv_close((uv_handle_t*)&server_stream_, NULL);
  uv_close((uv_handle_t*)&command_stream_, NULL);

  ur_->el_->stop();

  free(command_buffer_.base);
  free(response_buffer_.base);
}

void UrControlServer::send_command()
{
  ROS_ASSERT(ur_);

  ur_servoj *telegram = (ur_servoj*)command_buffer_.base;
  memset(telegram, 0, sizeof(ur_servoj));
  telegram->message_type_ = htonl(MSG_SERVOJ);

  pthread_mutex_lock(&ur_->write_mutex_);
  for (size_t i = 0; i < NUM_OF_JOINTS; ++i)
  {
    telegram->commanded_positions_[i] =
        htonl((int32_t)(MULT_JOINTSTATE * ur_->target_positions_[i]));
  }
  pthread_mutex_unlock(&ur_->write_mutex_);

  int status = uv_write(&write_request_,
                        (uv_stream_t*)&command_stream_,
                        &command_buffer_,
                        1,
                        command_sent_cb);
  ROS_ASSERT(0 == status);
}

void UrControlServer::send_ur_quit()
{
  ROS_ASSERT(ur_);

  ur_quit *telegram = (ur_quit*)command_buffer_.base;
  memset(telegram, 0, sizeof(ur_servoj));
  telegram->message_type_ = htonl(MSG_QUIT);

  int status = uv_write(&write_request_,
                        (uv_stream_t*)&command_stream_,
                        &command_buffer_,
                        1,
                        command_sent_cb);
  ROS_ASSERT(0 == status);
}

void UrControlServer::send_ur_stop()
{
  ROS_ASSERT(ur_);

  ur_stop *telegram = (ur_stop*)command_buffer_.base;
  memset(telegram, 0, sizeof(ur_servoj));
  telegram->message_type_ = htonl(MSG_STOPJ);

  int status = uv_write(&write_request_,
                        (uv_stream_t*)&command_stream_,
                        &command_buffer_,
                        1,
                        command_sent_cb);
  ROS_ASSERT(0 == status);
}

void UrControlServer::send_ur_set_teach_mode(bool teach_mode)
{
  ROS_ASSERT(ur_);

  ur_set_teach_mode *telegram = (ur_set_teach_mode*)command_buffer_.base;
  memset(telegram, 0, sizeof(ur_servoj));
  telegram->message_type_ = htonl(MSG_SET_TEACH_MODE);
  int32_t mode = teach_mode ? htonl(1) : htonl(0);
  telegram->teach_mode_ = htonl(mode);

  int status = uv_write(&write_request_,
                        (uv_stream_t*)&command_stream_,
                        &command_buffer_,
                        1,
                        command_sent_cb);
  ROS_ASSERT(0 == status);
}
