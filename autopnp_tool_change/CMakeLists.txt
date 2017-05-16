cmake_minimum_required(VERSION 2.8.3)
project(autopnp_tool_change)

set(catkin_RUN_PACKAGES
	roscpp
	rospy
	roslib
	actionlib
	actionlib_msgs
	std_msgs
	geometry_msgs
	control_msgs
	trajectory_msgs
	pr2_controllers_msgs
	cv_bridge
	message_filters
	cob_object_detection_msgs
	tf
	moveit_ros_move_group
	moveit_ros_planning_interface
	moveit_ros_manipulation
	joint_trajectory_action
	cob_phidgets
	cob_generic_states_experimental
)

set(catkin_BUILD_PACKAGES
	${catkin_RUN_PACKAGES}
	cmake_modules
	message_generation
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Creates a bunch of environment variables that may be used later in the script
## e.g. catkin_INCLUDE_DIRS
find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES}
)

find_package(OpenCV REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
## your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
## * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
## * If MSG_DEP_SET isn't empty the following dependencies might have been
## pulled in transitively but can be declared for certainty nonetheless:
## * add a build_depend tag for "message_generation"
## * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
## * add "message_generation" and every package in MSG_DEP_SET to
## find_package(catkin REQUIRED COMPONENTS ...)
## * add "message_runtime" and every package in MSG_DEP_SET to
## catkin_package(CATKIN_DEPENDS ...)
## * uncomment the add_*_files sections below as needed
## and list every .msg/.srv/.action file to be processed
## * uncomment the generate_messages entry below
## * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#	FILES
#		Dirt.msg
#)

# Generate services in the 'srv' folder
#add_service_files(
#	FILES
#		ActivateDirtDetection.srv
#		DeactivateDirtDetection.srv
#		GetDirtMap.srv
#		ValidateCleaningResult.srv
#)

## Generate actions in the 'action' folder
add_action_files(
	DIRECTORY
		action
	FILES
		ToolChange.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		std_msgs
		actionlib_msgs
)

# dynamic reconfigure
#generate_dynamic_reconfigure_options(
#	cfg/DirtDetection.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS
		ros/include
	LIBRARIES
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}
		message_runtime
	DEPENDS
		OpenCV
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
include_directories(
	ros/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


#list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

add_executable(autopnp_tool_change ros/src/autopnp_tool_change.cpp)
target_link_libraries(autopnp_tool_change
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)
add_dependencies(autopnp_tool_change ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} moveit_move_group_interface)
#add_executable(autopnp_tool_change_client ros/src/autopnp_tool_change_client.cpp)


#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS autopnp_tool_change
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY ros/launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros
	PATTERN ".svn" EXCLUDE
)

install(DIRECTORY common/files
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
	PATTERN ".svn" EXCLUDE
)
