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
 * sr_ur_program_loader.hpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#ifndef SR_UR_PROGRAM_LOADER_HPP_
#define SR_UR_PROGRAM_LOADER_HPP_

#include <uv.h>
#include "sr_ur_robot_hw/sr_ur_driver.hpp"

struct UrProgramLoader
{
  UrRobotDriver *ur_;

  uv_connect_t   connect_to_robot_request;
  uv_write_t     send_file_request;
  uv_tcp_t       send_file_stream;
  uv_buf_t       file_buffer;
  uv_fs_t        file_request;

  // path to file with the robot program
  char *file_path;

  // false if the reset program is currently loaded
  // true if the main robot program is currently loaded
  bool main_program_currently;

  // host server will listen on this port for the robot client to connect
  int host_port;

  // prepare the robot program to send by format printing host address and port
  void prepare_file_buffer();

  // load disk file containing robot program
  void load_file_from_disk();

  // load first a reset and then a main robot program to the robot
  void send_program(int reverse_port);
};

#endif
