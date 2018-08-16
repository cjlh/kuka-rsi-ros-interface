cmake_minimum_required(VERSION 2.8.3)
project(kuka_rsi_ros_interface_core)

add_compile_options(-std=c++11)

find_package(catkin	REQUIRED
	COMPONENTS
	roscpp
	kuka_rsi_ros_interface_msgs
	cmake_modules
	roslib
  	tf
  	angles
)

find_package(TinyXML REQUIRED)

catkin_package(
	# INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_runtime roslib
    DEPENDS TinyXML
)

include_directories(include ${catkin_INCLUDE_DIRS})

set(src_files
	${PROJECT_SOURCE_DIR}/src/main.cpp
	${PROJECT_SOURCE_DIR}/src/KukaRsiRosInterface.cpp
	${PROJECT_SOURCE_DIR}/src/RsiCommunicator.cpp
	${PROJECT_SOURCE_DIR}/src/KukaPose.cpp
)

add_executable(kuka_rsi_ros_interface ${src_files})

add_dependencies(kuka_rsi_ros_interface
	${catkin_EXPORTED_TARGETS}

)

target_link_libraries(kuka_rsi_ros_interface
	${catkin_LIBRARIES}
	${TinyXML_LIBRARIES}
)