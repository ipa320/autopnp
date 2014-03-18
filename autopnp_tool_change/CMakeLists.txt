cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include
		    ${PROJECT_SOURCE_DIR}/common/include)		    		    

rosbuild_add_executable(autopnp_tool_change ros/src/autopnp_tool_change.cpp)
rosbuild_add_executable(autopnp_tool_change_client ros/src/autopnp_tool_change_client.cpp)
#rosbuild_add_executable(arm_fiducial_calibration ros/src/arm_fiducial_calibration.cpp)
#rosbuild_add_executable(arm_fiducial_calibration_client ros/src/arm_fiducial_calibration_client.cpp)
rosbuild_add_executable(go_to_start_position ros/src/go_to_start_position.cpp)
rosbuild_add_executable(go_to_start_position_client ros/src/go_to_start_position_client.cpp)

target_link_libraries(autopnp_tool_change moveit_move_group_interface)
target_link_libraries(go_to_start_position moveit_move_group_interface)
#target_link_libraries(arm_fiducial_calibration moveit_move_group_interface)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})



