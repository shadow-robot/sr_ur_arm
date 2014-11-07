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
 * sr_ur_robot_state_client.hpp
 *
 *  Created on: 6 Nov 2014
 *      Author: Manos Nikolaidis
 */

#ifndef SR_UR_READ_ROBOT_STATE_HPP_
#define SR_UR_READ_ROBOT_STATE_HPP_

#include <uv.h>
#include "sr_ur_controller/sr_ur_driver.hpp"

struct UrRobotStateClient
{
  UrRobotDriver *ur_;

  uv_connect_t   connection_request_;
  uv_tcp_t       tcp_stream_;
  uv_buf_t       buffer_;
  bool           robot_state_received;

  void start();
  void stop();
};

#endif
