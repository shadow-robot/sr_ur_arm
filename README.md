sr_ur_arm
---------

The source code in this repository is for controlling a Shadow robot hand attached on a Universal Robots arm.
A sr_ur_controller for the complete arm (6DOF) is provided for use by the Shadow robot's control loop.
Ethernet communication is handled by asynchronously by libuv.
The repository depends on Shadow's fork of the ros_industrial drivers for UR https://github.com/shadow-robot/universal_robot.git
for things like the URDF description of the UR robot. Of course it also depends on the Shadow hand drivers for ROS.
This repository is not needed if you have either a Shadow hand or UR robot by themselves.

### sr_ur_program_loader

open a robot program file
connect with the socket to a server at the robot e.g. ("192.168.0.1", 30002)
send the whole robot program
close the file and socket

### sr_ur_event_loop
create a thread to run the loop in 
create, run, stop, delete the event loop

### sr_ur_read_write
connect to the real time socket at e.g. ("192.168.0.1", 30003)
send position and other commands, receive and report robot state 

### sr_ur_controller
plug in controller for the controller loop in ros_ethercat
