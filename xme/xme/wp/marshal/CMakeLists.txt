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
# $Id: CMakeLists.txt 5538 2013-10-17 09:12:41Z wiesmueller $
#
# File:
#         CMake configuration file for XME waypoints.
#

#------------------------------------------------------------------------------#
#-     xme_wp_marshaler_marshaler                                             -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_wp_marshal_marshaler"
    include/marshaler.h
    include/marshalerInternalTypes.h
    src/marshaler.c
    src/marshalerInternalTypes.c
)
xme_add_component(
    "xme_wp_marshal_demarshaler"
    include/demarshaler.h
    include/demarshalerInternalTypes.h
    src/demarshaler.c
    src/demarshalerInternalTypes.c
)
xme_add_component(
    "xme_wp_marshal_marshalerTest"
    test/marshaler.h
    test/marshaler.c
    test/deMarshalerTestTopic.h
    test/deMarshalerTestTopicData.h
    xme_hal_net
)

#xme_unit_test(
    #"xme_wp_marshal_marshalerTest"
    #TYPE smoke
    #test/smokeTestMarshaler.cpp
    #MOCK "xme_core_dataHandler" test/deMarshalerDataHandlerMock.c
    #xme_hal_net
#)
xme_unit_test(
    "xme_wp_marshal_marshalerTest"
    TYPE smoke
    xme_hal_net
    MOCK "xme_core_dataHandler" test/deMarshalerDataHandlerMock.c
    test/smokeTestMarshaler.cpp
)

xme_unit_test(
    "xme_wp_marshal_marshalerTest"
    TYPE interface
    test/interfaceTestMarshaler.cpp
    MOCK "xme_core_dataHandler" test/deMarshalerDataHandlerMock.c
)

xme_unit_test(
    "xme_wp_marshal_marshalerTest"
    TYPE integration
    test/integrationTestMarshaler.cpp
    MOCK "xme_core_dataHandler" test/deMarshalerDataHandlerMock.c
)

#------------------------------------------------------------------------------#
#-     xme_wp_marshaler_demarshaler                                           -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_wp_marshal_demarshalerTest"
    test/demarshaler.h
    test/demarshaler.c
    test/deMarshalerTestTopic.h
    test/deMarshalerTestTopicData.h
    xme_hal_net
)

xme_unit_test(
    "xme_wp_marshal_demarshalerTest"
    TYPE smoke
    test/smokeTestDemarshaler.cpp
    MOCK "xme_core_dataHandler" test/deMarshalerDataHandlerMock.c
)
              
xme_unit_test(
    "xme_wp_marshal_demarshalerTest"
    TYPE interface
    test/interfaceTestDemarshaler.cpp
    MOCK "xme_core_dataHandler" test/deMarshalerDataHandlerMock.c
)

xme_unit_test(
    "xme_wp_marshal_demarshalerTest"
    TYPE integration
    test/integrationTestDemarshaler.cpp
    MOCK "xme_core_dataHandler" test/deMarshalerDataHandlerMock.c
)

#------------------------------------------------------------------------------#
#-     xme_wp_marshal                                                         -#
#------------------------------------------------------------------------------#
xme_add_component(
    "xme_wp_marshal"
    xme_wp_marshal_marshalerTest
    xme_wp_marshal_demarshalerTest
)

xme_unit_test(
    "xme_wp_marshal"
    TYPE measurement
    test/measurementTest.cpp
    xme_wp_testUtil
    xme_core_core
    xme_hal_net
    xme_hal_sharedPtr
    xme_hal_sleep
    xme_hal_sync
    xme_hal_mem
    xme_core_dataHandler
    xme_core_executionManager
)
