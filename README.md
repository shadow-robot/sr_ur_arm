# sr_ur_arm

`sr_ur_arm` contains code allowing control of Universal Robots (UR) arms. UR10, UR5 and UR3 are all supported by this package.

Communication with the UR arm by TCP/IP is handled asynchronously by `libuv`. The control loop in `ros_ethercat` keeps UR arm's controller updated every 8ms. The controller for a UR model CB3 arm (6DOF) is provided as a plugin according to the `ros_control` architecture.

`sr_ur_arm` depends on Shadow's [fork](https://github.com/shadow-robot/universal_robot.git) of the `ros_industrial` drivers for UR, for things like the URDF description of the UR robot or Shadow Hand drivers for ROS. It is also compatible with [ros_control](https://github.com/ros-controls/ros_control) and and uses generic [ros_control_robot](https://github.com/shadow-robot/ros_control_robot) multi-robot ros_control loop.

There are multiple parameters that contribute to proper control of an UR manipulator:
* robot description - xacro files for UR robot. Examples used within this repository can be found [here](https://github.com/shadow-robot/sr_ur_arm/tree/kinetic-devel/sr_ur_launch/description),
* robot config - yaml file containing configuration parameters. Examples used within this repository can be found [here](https://github.com/shadow-robot/sr_ur_arm/tree/kinetic-devel/sr_ur_robot_hw/config),
* trajectory controller config - yaml file containing parameters used by robot's trajectory controller. Example can be found [here](https://github.com/shadow-robot/sr_ur_arm/tree/kinetic-devel/sr_ur_robot_hw/config),
* robot programs - files containing robot programs. Examples [here](https://github.com/shadow-robot/sr_ur_arm/blob/kinetic-devel/sr_ur_bringup/robot_programs/ur_robot_program).

Nodes that take part in the control process:
* `ros_control_robot` - control loop, implemented [here](https://github.com/shadow-robot/ros_control_robot),
* `joint_state_controller_spawner` - spawning of joint state control, using `controller_manager` package,
* `arm_controller_spawner`  - spawning of arm control, using `controller_manager` package.

Additionally, `robot_state_publisher` can be used for advertising arm state.

All of the above put together in a functional example launch files for a UR10 robot can be found [here](https://github.com/shadow-robot/sr_ur_arm/tree/kinetic-devel/sr_ur_launch/launch).

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
