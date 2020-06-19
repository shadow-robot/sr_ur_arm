[![No Maintenance Intended](http://unmaintained.tech/badge.svg)](http://unmaintained.tech/)

# ⛔️ DEPRECATED sr_ur_arm
## This is no longer supported, please consider using https://github.com/UniversalRobots/Universal_Robots_ROS_Driver instead

## General Overview

`sr_ur_arm` contains code allowing control of Universal Robots (UR) arms and their combinations with a Shadow Dexterous Hand. UR10, UR5 and UR3 are all supported by this package and can be used with or independently from Shadow Dexterous Hand.

Combined UR robot and hand system requires a special control structure because the UR arm expects position control commands over TCP/IP at 125Hz, and the Shadow Hand expects output over EtherCAT from a 1kHz control loop.

Communication with the UR arm by TCP/IP is handled asynchronously by `libuv`. Communication with the Shadow Hand is handled with the standard drivers using EtherCAT and `eml` as the EtherCAT master.

The control loop in `ros_ethercat` keeps both robots synchronized,  with the hand's controller updated every 1ms and the UR arm's controller updated every 8ms.

The controller for a UR model CB3 arm (6DOF) is provided as a plugin according to the `ros_control` architecture.

This repository is compatible with [ros_control](https://github.com/ros-controls/ros_control) and `CombinedRobotHW` and uses generic [ros_control_robot](https://github.com/shadow-robot/ros_control_robot) multi-robot ros_control loop. Example launch files for a UR10 robot (independent of the Shadow Dexterous Hand) can be found [here](https://github.com/shadow-robot/sr_ur_arm/tree/kinetic-devel/sr_ur_launch/launch).

`sr_ur_arm` depends on Shadow's [fork](https://github.com/shadow-robot/universal_robot.git) of the `ros_industrial` drivers for UR, for things like the URDF description of the UR robot or Shadow Hand drivers for ROS.

This repository has been widely used across many launch files implemented by Shadow. Numerous examples can be found in our [sr_robot_launch](https://github.com/shadow-robot/sr_interface/tree/kinetic-devel/sr_robot_launch) package.

## sr_ur_robot_hw package overview

### sr_ur_program_loader

Connect with the socket to a server at the robot e.g. ("192.168.0.1", 30002). Then open a robot program file send it through the socket.

### sr_ur_event_loop
Create and run the event loop in a thread with real time priority. 

### sr_ur_read_write
Accept a connection from a client in the robot and then send position commands and check response.

### sr_ur_read_robot_state
Connect to the real time state server e.g. ("192.168.0.1", 30003) and then receive and unpack the robot state.

### sr_ur_robot_hw
plug in controller for the controller loop in ros_ethercat.
