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
* sr_ur_event_loop.hpp
*
*  Created on: 20 Oct 2014
*      Author: Manos Nikolaidis
*/

#ifndef SR_UR_EVENT_LOOP_HPP_
#define SR_UR_EVENT_LOOP_HPP_

#include <uv.h>
#include "sr_ur_robot_hw/sr_ur_driver.hpp"

struct UrEventLoop
{
  UrRobotDriver *ur_;

  uv_loop_t* event_loop_;
  pthread_t  asynchronous_io_;
  uint64_t   last_time_;

  uv_loop_t* get_event_loop();
  double     get_loop_period();
  void       start();
  void       stop();
};


#endif
