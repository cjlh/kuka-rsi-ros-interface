cmake_minimum_required(VERSION 2.8.3)
project(kuka_rsi_ros_interface_msgs)

add_compile_options(-std=c++11)

set(MESSAGE_DEPENDENCIES
	std_msgs
)

find_package(catkin REQUIRED
	COMPONENTS
	roscpp
	message_generation
	${MESSAGE_DEPENDENCIES}
)

add_message_files(
	FILES
	KukaPose.msg
)

add_service_files(
	FILES
	GetPose.srv
	MoveToPose.srv
)

generate_messages(
	DEPENDENCIES
	${MESSAGE_DEPENDENCIES}
)

catkin_package(
	CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES}
)