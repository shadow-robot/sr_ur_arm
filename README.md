sr_ur_arm
---------

sr_ur_arm  controls a robot system made up from a Shadow Dexterous Hand and a Universal Robots (UR) arm. This requires a special control structure because the UR arm expects position control commands over TCP/IP at 125Hz, and the Shadow Hand expects output over EtherCAT from a 1kHz control loop.

The controller for a UR model CB3 arm (6DOF) is provided as a plugin according to the ros_control architecture.

Communication with the UR arm by TCP/IP is handled asynchronously by libuv. Communication with the Shadow Hand is handled with the standard drivers using EtherCAT and eml as the EtherCAT master.

The control loop in ros_ethercat keeps both robots synchronized,  with the Hand's controller updated every 1ms and the UR arm's controller updated every 8ms.

The repository depends on Shadow's [fork](https://github.com/shadow-robot/universal_robot.git) of the ros_industrial drivers for UR, for things like the URDF description of the UR robot, and the Shadow hand drivers for ROS.

This repository is not needed if you have either a Shadow hand or UR robot by themselves.

### sr_ur_program_loader

Connect with the socket to a server at the robot e.g. ("192.168.0.1", 30002). Then
open a robot program file send it through the socket

### sr_ur_event_loop
Create and run the event loop in a thread with real time priority. 

### sr_ur_read_write
Accept a connection from a client in the robot and then send position commands and check response

### sr_ur_read_robot_state
Connect to the real time state server e.g. ("192.168.0.1", 30003) and then receive and unpack the robot state  

### sr_ur_controller
plug in controller for the controller loop in ros_ethercat
