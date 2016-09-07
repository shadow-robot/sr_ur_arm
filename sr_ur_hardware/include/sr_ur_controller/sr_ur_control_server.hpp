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
 * sr_ur_contol_server.hpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#ifndef SR_UR_READ_WRITE_HPP_
#define SR_UR_READ_WRITE_HPP_

#include <uv.h>
#include "sr_ur_controller/sr_ur_driver.hpp"

#define WRITE_POOL_SIZE 5

struct UvWritePool
{
  uv_write_t write_request_[WRITE_POOL_SIZE];
  size_t next_;
  size_t size_;

  void init(void* ptr);
  bool is_full();
  void inc();
  void dec();
};

struct UrControlServer
{
  UrRobotDriver *ur_;

  uv_tcp_t   server_stream_;
  uv_tcp_t   command_stream_;

  uv_buf_t   command_buffer_;
  uv_buf_t   response_buffer_;
  uv_buf_t   teach_command_buffer_;

  UvWritePool write_request_pool_;
  uv_write_t write_request_;
  uv_write_t teach_command_write_request_;

  uv_async_t async_;

  void start();
  void stop();

  // send a servo command to the robot
  void send_servo_command();

  // sends a non servo, parameterless message to the robot
  void send_message(int32_t ur_msg_type);

  // sends a teach_mode on-off command to the robot
  void send_teach_mode_command(int32_t teach_mode);
};

#endif
