#
# Copyright (c) 2011-2014, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
# 
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 7586 2014-02-25 15:14:34Z kainz $
#

function (xme_hal_ros_before_use_callback)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        rostime
    )

    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
    )

    set(catkin_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS} CACHE STRING "" FORCE)
    set(catkin_LIBRARIES ${catkin_LIBRARIES} CACHE STRING "" FORCE)
endfunction (xme_hal_ros_before_use_callback)

xme_add_component(
    "xme_hal_ros"
    SOURCES
        xme/hal/ros_arch.cpp
    DEPENDS
        xme_core_node
    TARGETS
        VALUE_OF catkin_EXPORTED_TARGETS
    LINK
        VALUE_OF catkin_LIBRARIES
    CALLBACK
        BEFORE_USE xme_hal_ros_before_use_callback
)

xme_include_directory()
