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
# $Id: CMakeLists.txt 5205 2013-09-27 11:41:42Z ruiz $
#
# File:
#         CMake configuration file for XME UDP waypoints.
#

#------------------------------------------------------------------------------#
#-     xme_wp_udp_udpReceive                                                  -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_wp_udp_udpReceive"
    include/udpReceive.h
    include/udpReceiveWayPointFunctionWrapper.h
    src/udpReceive.c
    src/udpReceiveWayPointFunctionWrapper.c
	xme_com_packet
    xme_hal_net
    xme_hal_sharedPtr
    xme_hal_sync
)

#------------------------------------------------------------------------------#
#-     xme_wp_udp_udpSend                                                     -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_wp_udp_udpSend"
    include/udpSend.h
    include/udpSendWayPointFunctionWrapper.h
    src/udpSend.c
    src/udpSendWayPointFunctionWrapper.c
	xme_com_packet
    xme_hal_net
    xme_hal_sharedPtr
    xme_hal_sync
)

#------------------------------------------------------------------------------#
#-     xme_wp_udp                                                             -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_wp_udp"
    xme_wp_udp_udpReceive
    xme_wp_udp_udpSend
)

xme_unit_test(
    "xme_wp_udp"
    TYPE interface
    test/interfaceTestUDP.cpp
    test/dataHandlerMock.c
	xme_wp_udp
	xme_core_core
    xme_hal_net
    xme_hal_sharedPtr
    xme_hal_sleep
    xme_hal_sync
    xme_hal_mem
)
