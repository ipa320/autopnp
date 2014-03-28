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
# $Id: CMakeLists.txt 7814 2014-03-13 13:57:57Z geisinger $
#
# File:
#         CMake configuration file for XME UDP waypoints.
#

xme_add_component(
    "xme_wp_testUtil"
    testUtil.h
    testUtil.c
    xme_core_log
    xme_hal_mem
    xme_hal_time
)

xme_add_component(
    "xme_wp_waypointConfigInfrastructure"
    xme_hal_mem
    xme_com_interface
    xme_core_node
    xme_hal_linkedList
)

xme_unit_test(
    "xme_wp_waypointConfigInfrastructure"
    TYPE smoke
    ../waypointConfigInfrastructure.c
    ../waypointConfigInfrastructure.h
    smokeTestWCI.cpp
    mockLayer.c
    xme_core_nodeManager_componentRepository
)

xme_unit_test(
    "xme_wp_waypointConfigInfrastructure"
    TYPE interface
    ../waypointConfigInfrastructure.c
    ../waypointConfigInfrastructure.h
    interfaceTestWCI.cpp
    mockLayer.c
)
