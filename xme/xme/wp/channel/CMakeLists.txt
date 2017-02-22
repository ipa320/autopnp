#
# Copyright (c) 2011-2013, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 6684 2014-02-07 14:34:26Z geisinger $
#
# File:
#         CMake configuration file for XME channel waypoints.
#

#------------------------------------------------------------------------------#
#-     xme_wp_channel_channelSelector                                         -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_wp_channel_channelSelector"
    include/channelSelector.h
    include/channelSelectorWayPointFunctionWrapper.h
    src/channelSelector.c
    src/channelSelectorWayPointFunctionWrapper.c
    xme_hal_table
    xme_core_log
)

#------------------------------------------------------------------------------#
#-     xme_wp_channel_channelInjector                                         -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_wp_channel_channelInjector"
    include/channelInjector.h
    include/channelInjectorWayPointFunctionWrapper.h
    src/channelInjector.c
    src/channelInjectorWayPointFunctionWrapper.c
    xme_hal_table
    xme_core_log
)

#------------------------------------------------------------------------------#
#-     xme_wp_channel                                                         -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_wp_channel"
    xme_wp_channel_channelSelector
    xme_wp_channel_channelInjector
)

xme_unit_test(
    "xme_wp_channel"
    TYPE smoke
    test/smokeTestChannel.cpp
    test/dataHandlerMock.c
    xme_hal_mem
)

xme_unit_test(
    "xme_wp_channel"
    TYPE measurement
    test/measurementTest.cpp
    xme_wp_testUtil
    xme_core_core
    xme_core_dataHandler
    xme_core_directory_attribute
    xme_core_executionManager
    xme_hal_net
    xme_hal_sharedPtr
    xme_hal_sleep
    xme_hal_sync
    xme_hal_mem
)

