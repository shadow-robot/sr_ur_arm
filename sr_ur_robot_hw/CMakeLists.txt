# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_ur_robot_hw)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pluginlib
    combined_robot_hw
    hardware_interface
    controller_interface
    ros_ethercat_model
    sr_ur_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(sr_ur_robot_hw
	src/sr_ur_driver.cpp
	src/sr_ur_event_loop.cpp
	src/sr_ur_program_loader.cpp
	src/sr_ur_control_server.cpp
	src/sr_ur_robot_hw.cpp
	src/sr_ur_robot_state_client.cpp
)
add_dependencies(sr_ur_robot_hw
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(sr_ur_robot_hw uv ${catkin_LIBRARIES})

catkin_package(
	CATKIN_DEPENDS
	    roscpp
	    pluginlib
	    combined_robot_hw
	    controller_interface
	    hardware_interface
	    ros_ethercat_model
	    sr_ur_msgs
	    libuv-dev
	INCLUDE_DIRS
	    include
	LIBRARIES
            sr_ur_robot_hw
)

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES robot_hw_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
