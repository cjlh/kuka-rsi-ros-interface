cmake_minimum_required(VERSION 2.8.3)
project(kuka_rsi_ros_interface)

add_compile_options(-std=c++11)


set(MESSAGE_DEPENDENCIES
	std_msgs
)


find_package(catkin
	REQUIRED COMPONENTS
	roscpp
	message_generation
	${MESSAGE_DEPENDENCIES}
	cmake_modules
	roslib
)

find_package(TinyXML REQUIRED)


add_message_files(
	FILES
	KukaPose.msg
)


add_service_files(
	FILES
	MoveToPose.srv
	GetPose.srv
)


generate_messages(
	DEPENDENCIES
	${MESSAGE_DEPENDENCIES}
)


catkin_package(
	# INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES} roslib
    DEPENDS TinyXML
)


include_directories(include ${catkin_INCLUDE_DIRS})


set(src_files
	${PROJECT_SOURCE_DIR}/src/main.cpp
	${PROJECT_SOURCE_DIR}/src/RsiCommunicator.cpp
)

set(hdr_files
	${PROJECT_SOURCE_DIR}/src/RsiCommunicator.h
)

add_executable(main ${src_files})


add_dependencies(main
	kuka_rsi_ros_interface_generate_messages_cpp
)


target_link_libraries(main
	${hdr_files}
	${catkin_LIBRARIES}
	${TinyXML_LIBRARIES}
)