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

#include <ros/ros.h>
#include "sr_ur_controller/sr_ur_event_loop.hpp"
#include "sr_ur_controller/sr_ur_common.hpp"
#include "sr_ur_controller/sr_ur_program_loader.hpp"
#include <sr_ur_controller/sr_ur_read_write.hpp>

#define ROS_ASSERT_ENABLED

char *robot_program_path;

static uv_connect_t connect_to_robot_request;
static uv_write_t send_file_request;
static uv_tcp_t send_file_stream;
static uv_buf_t file_buffer;
static uv_fs_t file_request;

static char *file_path;
static bool main_program_currently;

static int local_port;

static void load_file_from_disk();

// this is called after the reset robot program has been sent
static void file_sent_cb(uv_write_t* send_file_request, int status)
{
  ROS_ASSERT(send_file_request);
  ROS_ASSERT(send_file_request == &file_request);
  ROS_ASSERT(0 == status);

  if (!main_program_currently)
  {
    main_program_currently = true;
    load_file_from_disk();
  }
  else
  {
    uv_fs_req_cleanup(&file_request);
    uv_close((uv_handle_t*)&send_file_stream, NULL);
    free(robot_program_path);
    free(file_path);
    free(file_buffer.base);
    ROS_INFO("UrArmController finished sending robot program");
  }
}

static void prepare_file_buffer()
{
  if (!main_program_currently)
    return;

  char *temp_buffer;
  size_t allocated_size = asprintf(&temp_buffer, file_buffer.base, host_address, local_port);
  ROS_ASSERT(allocated_size > 0);

  free(file_buffer.base);
  allocated_size = asprintf(&file_buffer.base, "%s", temp_buffer);
  ROS_ASSERT(allocated_size > 0);
  file_buffer.len = allocated_size;
}

static void client_connected_cb(uv_connect_t* connection_request, int status)
{
  ROS_ASSERT(connection_request);
  ROS_ASSERT(connection_request == &connect_to_robot_request);
  ROS_ASSERT(0 == status);

  prepare_file_buffer();

  status = uv_write(&send_file_request,
                    (uv_stream_t*)&send_file_stream,
                    &file_buffer,
                    1,
                    file_sent_cb);
  ROS_ASSERT(0 == status);
}

static void file_loaded_cb(uv_fs_t *load_file_request)
{
  ROS_ASSERT(load_file_request);
  ROS_ASSERT(load_file_request == &file_request);
  sockaddr_in server_address = uv_ip4_addr(robot_address, 30002);
  int status = uv_tcp_connect(&connect_to_robot_request,
                              &send_file_stream,
                              server_address,
                              client_connected_cb);
  ROS_ASSERT(0 == status);
}

static void file_opened_cb(uv_fs_t *open_file_request)
{
  ROS_ASSERT(open_file_request);
  ROS_ASSERT(open_file_request == &file_request);
  ROS_ASSERT(-1 != open_file_request->result);

  int status = uv_fs_read(get_event_loop(),
                          &file_request,
                          open_file_request->result,
                          file_buffer.base,
                          file_buffer.len,
                          -1,
                          file_loaded_cb);

  ROS_ASSERT(0 == status);
}

static void load_file_from_disk()
{
  const char *file_name = main_program_currently ? "ur_robot_program" : "ur_reset_program";
  free(file_path);
  int status = asprintf(&file_path, "%s%s", robot_program_path, file_name);
  ROS_ASSERT(status > 0);

  // allocate buffer according to file size
  struct stat st;
  size_t file_size = 0;
  if (0 == stat(file_path, &st))
    file_size = st.st_size;
  file_buffer.base = (char*)realloc((void*)file_buffer.base, file_size);
  file_buffer.len = file_size;

  ROS_INFO("UrArmController loading robot program file %s with size %zu", file_path, file_size);

  status = uv_fs_open(get_event_loop(),
                      &file_request,
                      file_path,
                      O_RDONLY,
                      0,
                      file_opened_cb);
  ROS_ASSERT(0 == status);
}

void load_robot_program(int reverse_port)
{
  local_port = reverse_port;
  load_file_from_disk();

  // initialise stream for client that writes a robot program
  int status = uv_tcp_init(get_event_loop(), &send_file_stream);
  ROS_ASSERT(0 == status);
  uv_tcp_nodelay(&send_file_stream, 1);
}
