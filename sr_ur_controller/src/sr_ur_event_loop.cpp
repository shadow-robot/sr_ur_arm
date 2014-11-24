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
 * sr_ur_event_loop.cpp
 *
 *  Created on: 20 Oct 2014
 *      Author: Manos Nikolaidis
 */

#define ROS_ASSERT_ENABLED
#include <ros/assert.h>
#include "sr_ur_controller/sr_ur_event_loop.hpp"

// function for the thread that runs the event loop
static void *asynchronous_io_loop(void *data)
{
  UrEventLoop* el = (UrEventLoop*)data;

  // run the loop
  uv_run_mode run_mode = UV_RUN_DEFAULT;
  uv_run(el->event_loop_, run_mode);
  return NULL;
}

// returns the event loop and creates a new one if needed
uv_loop_t* UrEventLoop::get_event_loop()
{
  ROS_ASSERT(ur_);

  if (!event_loop_)
  {
    event_loop_ = uv_loop_new();
  }
  ROS_ASSERT(event_loop_);
  return event_loop_;
}

// event loop should be started after get_event_loop has been called
// and at least one callback has been registered with the event loop
void UrEventLoop::start()
{
  ROS_ASSERT(ur_);

  pthread_attr_t attr;
  int status = pthread_attr_init(&attr);
  ROS_ASSERT(0 == status);

  // set the scheduling policy to real time
  int real_time_policy = SCHED_FIFO;
  status = pthread_attr_setschedpolicy(&attr, real_time_policy);
  ROS_ASSERT(0 == status);

  // set the real time priority of the thread to maximum
  sched_param thread_param;
  thread_param.__sched_priority = sched_get_priority_max(real_time_policy);
  status = pthread_attr_setschedparam(&attr, &thread_param);
  ROS_ASSERT(0 == status);

  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  status = pthread_create(&asynchronous_io_, &attr, asynchronous_io_loop, this);
  ROS_ASSERT(0 == status);

  status = pthread_attr_destroy(&attr);
  ROS_ASSERT(0 == status);

  last_time_ = uv_hrtime();
}

// the event loop will actually terminate after pending callbacks return
void UrEventLoop::stop()
{
  ROS_ASSERT(ur_);

  uv_stop(event_loop_);

  // wait for event_loop_ to actually terminate before deleting
  void *value;
  pthread_join(asynchronous_io_, &value);

  uv_loop_delete(event_loop_);
}

double UrEventLoop::get_loop_period()
{
  ROS_ASSERT(ur_);

  double interval = uv_hrtime() - last_time_;
  return interval * (1 / 1000000);
}
