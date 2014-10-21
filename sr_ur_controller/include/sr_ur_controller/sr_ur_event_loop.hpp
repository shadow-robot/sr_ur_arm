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
 * sr_ur_event_loop.hpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#ifndef SR_UR_EVENT_LOOP_HPP_
#define SR_UR_EVENT_LOOP_HPP_

#include <uv.h>

// gets the event loop or NULL of it hasn't started
uv_loop_t* get_event_loop();

// start the event loop in a separate thread
void start_event_loop();

// signals to stop the event loop that will happen after pending callbacks return
void stop_event_loop();

#endif
