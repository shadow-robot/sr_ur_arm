cmake_minimum_required(VERSION 2.8.3)
project(sr_trajectory_record_playback)

find_package(catkin REQUIRED COMPONENTS
	rospy
	actionlib
	trajectory_msgs
	control_msgs
)

catkin_package(
    CATKIN_DEPENDS
    	rospy
		actionlib
		trajectory_msgs
		control_msgs
		mongodb_store
)

install(PROGRAMS
		scripts/trajectory_recorder
		scripts/pose_mover
		scripts/pose_recorder
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY trajectory_data/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/trajectory_data
)
