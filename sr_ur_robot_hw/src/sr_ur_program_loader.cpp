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
 * sr_ur_program_loader.cpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#define ROS_ASSERT_ENABLED
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/assert.h>

#include "sr_ur_robot_hw/sr_ur_driver.hpp"
#include "sr_ur_robot_hw/sr_ur_event_loop.hpp"
#include "sr_ur_robot_hw/sr_ur_control_server.hpp"
#include "sr_ur_robot_hw/sr_ur_program_loader.hpp"

const int ROBOT_PORT = 30002;

// this is called after the robot program has been sent
static void file_sent_cb(uv_write_t* file_request, int status)
{
  ROS_ASSERT(0 == status);
  ROS_ASSERT(file_request);
  ROS_ASSERT(file_request->data);

  UrProgramLoader *urpl = reinterpret_cast<UrProgramLoader*>(file_request->data);
  ROS_ASSERT(file_request == &urpl->send_file_request);

  if (!urpl->main_program_currently)
  {
    urpl->main_program_currently = true;
    urpl->load_file_from_disk();
  }
  else
  {
    uv_fs_req_cleanup(&urpl->file_request);
    uv_close(reinterpret_cast<uv_handle_t*>(&urpl->send_file_stream), NULL);
    free(urpl->ur_->robot_program_path_);
    free(urpl->file_path);
    free(urpl->file_buffer.base);
    ROS_INFO("UrArmController of %s robot finished sending robot program", urpl->ur_->robot_side_);
    sleep(0.2);
  }
}

// connected to the server on the robot that will receive the robot program
// so start sending the program
static void client_connected_cb(uv_connect_t* connection_request, int status)
{
  ROS_ASSERT(0 == status);
  ROS_ASSERT(connection_request);
  ROS_ASSERT(connection_request->data);

  UrProgramLoader *urpl = reinterpret_cast<UrProgramLoader*>(connection_request->data);
  ROS_ASSERT(connection_request == &urpl->connect_to_robot_request);

  status = uv_write(&urpl->send_file_request,
                    reinterpret_cast<uv_handle_t*>(&urpl->send_file_stream),
                    &urpl->file_buffer,
                    1,
                    file_sent_cb);
  ROS_ASSERT(0 == status);
}

// disk file containing the robot program has been loaded
// so connect to server in robot to send
static void file_loaded_cb(uv_fs_t *file_request)
{
  ROS_ASSERT(file_request);
  ROS_ASSERT(file_request->data);

  UrProgramLoader *urpl = reinterpret_cast<UrProgramLoader*>(file_request->data);
  ROS_ASSERT(file_request == &urpl->file_request);

  if (urpl->main_program_currently)
  {
    urpl->prepare_file_buffer();
    int status = uv_write(&urpl->send_file_request,
                      reinterpret_cast<uv_handle_t*>(&urpl->send_file_stream),
                      &urpl->file_buffer,
                      1,
                      file_sent_cb);
    ROS_ASSERT(0 == status);
  }
  else
  {
    sockaddr_in server_address = uv_ip4_addr(urpl->ur_->robot_address_, ROBOT_PORT);
    int status = uv_tcp_connect(&urpl->connect_to_robot_request,
                                &urpl->send_file_stream,
                                server_address,
                                client_connected_cb);
    ROS_ASSERT(0 == status);
  }
}

// opened the disk file containing the robot program
// so read the file
static void file_opened_cb(uv_fs_t *file_request)
{
  ROS_ASSERT(file_request);
  ROS_ASSERT(file_request->data);

  UrProgramLoader *urpl = reinterpret_cast<UrProgramLoader*>(file_request->data);
  ROS_ASSERT(file_request == &urpl->file_request);
  ROS_ASSERT(-1 != file_request->result);

  int status = uv_fs_read(urpl->ur_->el_->get_event_loop(),
                          file_request,
                          file_request->result,
                          urpl->file_buffer.base,
                          urpl->file_buffer.len,
                          -1,
                          file_loaded_cb);
  ROS_ASSERT(0 == status);
}

void UrProgramLoader::prepare_file_buffer()
{
  ROS_ASSERT(ur_);

  ROS_ASSERT(file_buffer.base);
  ROS_ASSERT(ur_->host_address_);
  ROS_ASSERT(host_port);

  char *temp_buffer = NULL;
  size_t allocated_size = asprintf(&temp_buffer, file_buffer.base, ur_->host_address_, host_port);
  ROS_ASSERT(allocated_size > 0);
  if (temp_buffer == NULL)
  {
    allocated_size = asprintf(&temp_buffer, file_buffer.base, ur_->host_address_, host_port);
  }
  ROS_ASSERT(temp_buffer);

  free(file_buffer.base);
  file_buffer.base = NULL;
  allocated_size = asprintf(&file_buffer.base, "%s", temp_buffer);
  ROS_ASSERT(allocated_size > 0);
  free(temp_buffer);
  ROS_ASSERT(file_buffer.base);

  file_buffer.len = allocated_size;
}

void UrProgramLoader::load_file_from_disk()
{
  ROS_ASSERT(ur_);
  sleep(0.2);

  // the main program is ur_robot_program that is doing the actual work
  // but ur_reset_program must be sent before that
  const char *file_name = main_program_currently ? "ur_robot_program" : "ur_reset_program";
  free(file_path);
  file_path = NULL;
  ROS_ASSERT(ur_->robot_program_path_);
  int status = asprintf(&file_path, "%s%s", ur_->robot_program_path_, file_name);
  ROS_ASSERT(status > 0);
  ROS_ASSERT(file_path);

  // allocate buffer according to file size
  struct stat st;
  size_t file_size = 0;
  if (0 == stat(file_path, &st))
  {
    file_size = st.st_size;
  }
  ROS_ASSERT(file_size > 0);

  file_buffer.base = reinterpret_cast<char*>(realloc(reinterpret_cast<char*>(file_buffer.base), file_size));
  file_buffer.len = file_size;
  ROS_ASSERT(file_buffer.base);

  ROS_INFO("UrArmController of %s robot loading robot program file %s with size %zu",
           ur_->robot_side_, file_path, file_size);

  status = uv_fs_open(ur_->el_->get_event_loop(),
                      &file_request,
                      file_path,
                      O_RDONLY,
                      0,
                      file_opened_cb);
  ROS_ASSERT(0 == status);
}

void UrProgramLoader::send_program(int reverse_port)
{
  ROS_ASSERT(ur_);

  connect_to_robot_request.data = reinterpret_cast<char*>(this);
  send_file_request.data        = reinterpret_cast<char*>(this);
  send_file_stream.data         = reinterpret_cast<char*>(this);
  file_request.data             = reinterpret_cast<char*>(this);

  host_port = reverse_port;
  // initialise stream for client that writes a robot program
  int status = uv_tcp_init(ur_->el_->get_event_loop(), &send_file_stream);
  ROS_ASSERT(0 == status);
  status = uv_tcp_nodelay(&send_file_stream, 0);
  ROS_ASSERT(0 == status);

  load_file_from_disk();
}
