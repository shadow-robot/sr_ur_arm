# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_ur_controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pluginlib
    controller_interface
    ros_ethercat_model
    sr_ur_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(sr_ur_controller
	src/sr_ur_driver.cpp
	src/sr_ur_event_loop.cpp
	src/sr_ur_program_loader.cpp
	src/sr_ur_control_server.cpp
	src/sr_ur_controller.cpp
	src/sr_ur_robot_state_client.cpp
)
add_dependencies(sr_ur_controller
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(sr_ur_controller uv ${catkin_LIBRARIES})

catkin_package(
	CATKIN_DEPENDS
	    roscpp
	    pluginlib
	    controller_interface
	    ros_ethercat_model
	    sr_ur_msgs
	INCLUDE_DIRS
	    include
	LIBRARIES
		sr_ur_controller
)

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
