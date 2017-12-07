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

#ifndef SR_UR_CONFIG_HPP_
#define SR_UR_CONFIG_HPP_

#include <uv.h>
#include "sr_ur_robot_hw/sr_ur_driver.hpp"

struct UrConfig
{
  UrRobotDriver *ur_;

  // string to store payload
  char *payload;

  // float to store arm speed scaling
  float *speed_scaling

  // set the payload
  void set_payload();

  // set speed scaling
  void set_speed_scaling();
};

#endif
