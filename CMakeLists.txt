cmake_minimum_required(VERSION 2.8.3)
project(tts)

## C++17
add_compile_options(-std=c++17)

## Find catkin and any catkin packages
find_package(
	catkin
	REQUIRED COMPONENTS
	roscpp
	roslib
	rospy
	std_msgs
	genmsg
)

## declare ROS messages and services
add_message_files(
	FILES
	TextToSpeechMsg.msg
)

add_service_files(
	FILES
	TextToSpeechSrv.srv
)

## generate added messages and services
generate_messages(
	DEPENDENCIES
	std_msgs
)

## declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
	std_msgs
)

## include dirs
include_directories(include ${catkin_INCLUDE_DIRS})

## nodes

### tts node
add_executable(tts src/tts.cpp)
target_link_libraries(tts ${catkin_LIBRARIES})
add_dependencies(tts ${PROJECT_NAME}_generate_messages_cpp)

### hw_node library
add_library(tts_lib src/tts_lib.cpp)
target_link_libraries(tts_lib ${catkin_LIBRARIES})
add_dependencies(tts_lib ${PROJECT_NAME}_generate_messages_cpp)

## install all project namespaced headers (package/include)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)