/* Copyright (c) 2013, Shadow Robot Company, All rights reserved.
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
 * sr_ur_event_loop.cpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#include <ros/ros.h>
#include "sr_ur_controller/sr_ur_event_loop.hpp"

#define ROS_ASSERT_ENABLED

static uv_loop_t* event_loop;
static pthread_t asynchronous_io_;

static void *asynchronous_io_loop(void *)
{
  // set maximum real time priority
  sched_param thread_param;
  int policy = SCHED_FIFO;
  thread_param.sched_priority = sched_get_priority_max(policy);
  pthread_setschedparam(pthread_self(), policy, &thread_param);

  uv_run_mode run_mode = UV_RUN_DEFAULT;
  uv_run(event_loop, run_mode);
  return NULL;
}

// returns the event loop or creates a new one
uv_loop_t* get_event_loop()
{
  if (!event_loop)
  {
    ROS_INFO("UrArmController started a new event loop");
    event_loop = uv_loop_new();
  }
  ROS_ASSERT(event_loop);
  return event_loop;
}

// event loop should be started after get_event_loop has been called
// and at least one callback has been registered with the event loop
void start_event_loop()
{
  int thread_status = pthread_create(&asynchronous_io_, NULL, asynchronous_io_loop, event_loop);
  ROS_ASSERT(0 == thread_status);
}

void stop_event_loop()
{
  uv_stop(event_loop);
}

