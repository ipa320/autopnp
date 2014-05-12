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
# $Id: CMakeLists.txt 5090 2013-09-17 12:15:53Z ruiz $
#
# File:
#         CMake configuration file for XME Broker.
#

#------------------------------------------------------------------------------#
#-     xme_core_broker                                                        -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_broker"
    include/broker.h
    include/brokerPnpManagerInterface.h
    include/brokerDataManagerInterface.h
    include/brokerInternalMethods.h
    include/brokerInternalTypes.h
    src/broker.c
    src/brokerPnPManager.c
    src/brokerDataManager.c
    xme_hal_linkedList
    xme_hal_table
    xme_core_log
    xme_core_dataHandler
)

xme_unit_test(
    "xme_core_broker" 
    TYPE smoke
    test/smokeTestBroker.cpp
)

xme_unit_test(
    "xme_core_broker" 
    TYPE interface
    test/interfaceTestBroker.cpp
    MOCK "xme_core_dataHandler" test/brokerDataHandlerMock.c
)

xme_unit_test(
    "xme_core_broker" 
    TYPE integration
    test/integrationTestBroker.cpp
    MOCK "xme_core_dataHandler" test/brokerDataHandlerMock.c
)
