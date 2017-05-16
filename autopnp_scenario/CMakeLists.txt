cmake_minimum_required(VERSION 2.8.3)
project(autopnp_scenario)

set(catkin_RUN_PACKAGES
	roscpp
	rospy
	roslib
	actionlib
	actionlib_msgs
	std_msgs
	std_srvs
	sensor_msgs
	geometry_msgs
	visualization_msgs
	diagnostic_msgs
	cob_object_detection_msgs
	cv_bridge
	cob_srvs
	message_filters
	cob_navigation_global
	cob_generic_states_experimental
	cob_trajectory_controller
	cob_map_accessibility_analysis
	cob_fiducials
	cob_phidgets
	autopnp_dirt_detection
	ipa_room_segmentation
	ipa_building_navigation
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

catkin_python_setup()

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
add_message_files(
	FILES
		TrashBinDetection.msg
)

# Generate services in the 'srv' folder
add_service_files(
	DIRECTORY
		srv
	FILES
		ActivateTrashBinDetection.srv
		DeactivateTrashBinDetection.srv
		DetectFiducials.srv
)

## Generate actions in the 'action' folder
add_action_files(
	DIRECTORY
		action
	FILES
		FindNextUnprocessedRoom.action
		GoToRoomLocation.action
		GraspTrashBin.action
		InspectRoom.action
		MapSegmentation.action
		RandomLocationFinder.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		std_msgs
		std_srvs
		sensor_msgs
		geometry_msgs
		visualization_msgs
		diagnostic_msgs
		actionlib_msgs
		cob_object_detection_msgs
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


## map segmentation action server and client
#add_executable(map_segmentation_action_server ros/src/map_segmentation_action_server.cpp)
#target_link_libraries(map_segmentation_action_server
#	${catkin_LIBRARIES}
#	${OpenCV_LIBRARIES}
#)
#add_dependencies(map_segmentation_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(map_segmentation_action_client ros/src/map_segmentation_action_client.cpp)
#target_link_libraries(map_segmentation_action_client
#	${catkin_LIBRARIES}
#)
#add_dependencies(map_segmentation_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# find next room action server
#add_executable(find_next_unprocessed_room_action_server ros/src/find_next_unprocessed_room_action_server.cpp)
#target_link_libraries(find_next_unprocessed_room_action_server
#	${catkin_LIBRARIES}
#)
#add_dependencies(find_next_unprocessed_room_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
		

# go to room location action server
#add_executable(go_to_room_location_action_server ros/src/go_to_room_location_action_server.cpp)
#target_link_libraries(go_to_room_location_action_server
#	${catkin_LIBRARIES}
#)
#add_dependencies(go_to_room_location_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# random location finder server
#add_executable(random_location_finder_action_server ros/src/random_location_finder_action_server.cpp)
#target_link_libraries(random_location_finder_action_server
#	${catkin_LIBRARIES}
#)
#add_dependencies(random_location_finder_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# inspect room server
add_executable(inspect_room_action_server ros/src/inspect_room_action_server.cpp)
target_link_libraries(inspect_room_action_server
	${catkin_LIBRARIES}
)
add_dependencies(inspect_room_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# trash bin detection server
add_executable(trash_bin_detection_service_server ros/src/trash_bin_detection_service_server.cpp)
target_link_libraries(trash_bin_detection_service_server
	${catkin_LIBRARIES}
)
#target_link_libraries(trash_bin_detection_service_server moveit_move_group_interface)
add_dependencies(trash_bin_detection_service_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# activate trash bin detection client
add_executable(activate_trash_bin_detection_client ros/src/activate_trash_bin_detection_client.cpp)
target_link_libraries(activate_trash_bin_detection_client
	${catkin_LIBRARIES}
)
add_dependencies(activate_trash_bin_detection_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


# deactivate trash bin detection client
add_executable(deactivate_trash_bin_detection_client ros/src/deactivate_trash_bin_detection_client.cpp)
target_link_libraries(deactivate_trash_bin_detection_client
	${catkin_LIBRARIES}
)
add_dependencies(deactivate_trash_bin_detection_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



#############
## Install ##
#############
## Mark executables and/or libraries for installation
install(TARGETS inspect_room_action_server trash_bin_detection_service_server activate_trash_bin_detection_client
		deactivate_trash_bin_detection_client  #go_to_room_location_action_server random_location_finder_action_server find_next_unprocessed_room_action_server map_segmentation_action_server map_segmentation_action_client
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

install(DIRECTORY scripts
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	PATTERN ".svn" EXCLUDE
)
