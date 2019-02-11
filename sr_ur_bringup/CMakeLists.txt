cmake_minimum_required(VERSION 2.8.3)
project(sr_ur_bringup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    CATKIN_DEPENDS
)

install(DIRECTORY robot_programs/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robot_programs
)
