sr_ur_arm
---------

The source code in this repository is for controlling a Shadow robot hand attached on a Universal Robots arm.
A sr_ur_controller for the complete arm (6DOF) is provided for use by the Shadow robot's control loop.
Ethernet communication is handled by asynchronously by libuv.
The repository depends on Shadow's fork of the ros_industrial drivers for UR https://github.com/shadow-robot/universal_robot.git
for things like the URDF description of the UR robot. Of course it also depends on the Shadow hand drivers for ROS.
This repository is not needed if you have either a Shadow hand or UR robot by themselves.

### sr_ur_program_loader

Connect with the socket to a server at the robot e.g. ("192.168.0.1", 30002). Then
open a robot program file send it to the socket

### sr_ur_event_loop
Create and run the event loop in a thread with real time priority. 

### sr_ur_read_write
accept connection from the robot client and then send position commands and check response

### sr_ur_read_robot_state
connect to the real time state server e.g. ("192.168.0.1", 30003) and then receive and unpack the robot state  

### sr_ur_controller
plug in controller for the controller loop in ros_ethercat
