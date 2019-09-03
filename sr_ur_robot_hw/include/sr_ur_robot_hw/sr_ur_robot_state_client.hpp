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
* sr_ur_robot_state_client.hpp
*
*  Created on: 6 Nov 2014
*      Author: Manos Nikolaidis
*/

#ifndef SR_UR_READ_ROBOT_STATE_HPP_
#define SR_UR_READ_ROBOT_STATE_HPP_

#include <string>
#include <uv.h>
#include "sr_ur_robot_hw/sr_ur_driver.hpp"

struct UrRobotStateClient
{
  UrRobotDriver *ur_;

  uv_connect_t   connection_request_;
  uv_tcp_t       tcp_stream_;
  uv_buf_t       buffer_;
  bool           robot_state_received;
  std::string    protocol_version;

  void start();
  void stop();
};

#endif
