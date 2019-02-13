## sr_ur_arm with combined hardware

`sr_ur_arm` is compatible with [combined_robot_hw](http://wiki.ros.org/combined_robot_hw), allowing control of multiple robots with one control loop.

As an example one could take driver implemented at Shadow for combined control of UR10 robot together with Shadow Flexible Hand.

Combined UR robot and hand system requires a special control structure because the UR arm expects position control commands over TCP/IP at 125Hz, and the Shadow Hand expects output over EtherCAT from a 1kHz control loop.

Communication with the UR arm by TCP/IP is handled asynchronously by `libuv`. Communication with the Shadow Hand is handled with the standard drivers using EtherCAT and `eml` as the EtherCAT master.

The control loop in [ros_ethercat](https://github.com/shadow-robot/ros_ethercat) keeps both robots synchronized, with the hand's controller updated every 1ms and the UR arm's controller updated every 8ms.

In order to start the robots, in out case [sr-ros-interface-ethercat](https://github.com/shadow-robot/sr-ros-interface-ethercat) repository is used to launch the robots within one launch file. However, multiple parameters from `sr_ur_arm` (robot programs and controllers) are used within it for control of the UR10 robot. An example launch file in which it is used can be found [here](https://github.com/shadow-robot/sr_interface/blob/kinetic-devel/sr_robot_launch/launch/sr_ur_arm_hand.launch).

This repository has been widely used across many launch files with combined hardware implemented by Shadow. Multiple examples can be found in our [sr_robot_launch](https://github.com/shadow-robot/sr_interface/tree/kinetic-devel/sr_robot_launch) package.