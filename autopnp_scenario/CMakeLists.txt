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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include
					${PROJECT_SOURCE_DIR}/common/include)
					
#rosbuild_add_executable(send_Goal_nav ros/src/send_Goal_nav.cpp)
#rosbuild_add_executable(Exploration_Algorithm ros/src/Exploration_Algorithm.cpp)
#rosbuild_add_library(Exploration_Algorithm_2nd_version_Python_Library ros/src/Exploration_Algorithm_2nd_version_Python_Library.cpp)
#rosbuild_add_executable(myBWlabel ros/src/myBWlabel.cpp)
#rosbuild_add_executable(tf_data ros/src/tf_data.cpp)
#rosbuild_add_executable(help ros/src/help.cpp)
#rosbuild_add_executable(test_test ros/src/test.cpp)

			#Exploration Algorithm Executable
rosbuild_add_executable(Exploration_Algorithm_2nd_version ros/src/Exploration_Algorithm_2nd_version.cpp)

			#Exploration Algorithm Library executable
rosbuild_add_library(Exploration_Algorithm_2nd_version_Action_server_Library ros/src/Exploration_Algorithm_2nd_version_Action_server_Library.cpp)
rosbuild_link_boost(Exploration_Algorithm_2nd_version_Action_server_Library thread)

			#Analyze Map Server and Client
rosbuild_add_executable(Analyze_Map_server ros/src/Analyze_Map_server.cpp)

target_link_libraries(Analyze_Map_server Exploration_Algorithm_2nd_version_Action_server_Library)

rosbuild_add_executable(Analyze_Map_client ros/src/Analyze_Map_client.cpp)

rosbuild_link_boost(Analyze_Map_client thread)


			#Next Room Server and Client
rosbuild_add_executable(Next_Room_server ros/src/Next_Room_server.cpp)

target_link_libraries(Next_Room_server Exploration_Algorithm_2nd_version_Action_server_Library)

rosbuild_add_executable(Next_Room_client ros/src/Next_Room_client.cpp)

rosbuild_link_boost(Next_Room_client thread)


			#Inspect Room Server and Client
rosbuild_add_executable(Inspect_Room_server ros/src/Inspect_Room_server.cpp)

target_link_libraries(Inspect_Room_server Exploration_Algorithm_2nd_version_Action_server_Library)

#rosbuild_add_executable(Inspect_Room_client ros/src/Inspect_Room_client.cpp)

#rosbuild_link_boost(Inspect_Room_client thread)


			#to location Server and Client
rosbuild_add_executable(to_location_server ros/src/to_location_server.cpp)

target_link_libraries(to_location_server Exploration_Algorithm_2nd_version_Action_server_Library)

#rosbuild_add_executable(to_location_client ros/src/to_location_client.cpp)

#rosbuild_link_boost(to_location_client thread)


			#Random Location Server and Client
rosbuild_add_executable(Random_Location_server ros/src/Random_Location_server.cpp)

target_link_libraries(Random_Location_server Exploration_Algorithm_2nd_version_Action_server_Library)

#rosbuild_add_executable(Random_Location_client ros/src/Random_Location_client.cpp)

#rosbuild_link_boost(Random_Location_client thread)




