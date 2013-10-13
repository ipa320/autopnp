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
# $Id: CMakeLists.txt 5038 2013-09-11 08:32:42Z gulati $
#
# File:
#         CMake configuration file for XME Directory.
#

xme_add_component(
    "xme_com_interface"
    interface.h
    interface.c
    xme_hal_net
)

xme_add_component(
    "xme_com_packet"
    packet.c
    packet.h
    xme_hal_sharedPtr
    xme_hal_sync
)

xme_unit_test(
    "xme_com_interface"
    TYPE smoke
    test/smokeTestInterface.cpp
    xme_hal_net
)

xme_unit_test(
    "xme_com_interface"
    TYPE interface
    test/interfaceTestInterface.cpp
    xme_hal_net
)

xme_unit_test(
    "xme_com_packet"
    TYPE smoke
    test/smokeTestPacket.cpp
)
