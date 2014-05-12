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
# $Id: CMakeLists.txt 7810 2014-03-13 11:57:23Z geisinger $
#
# File:
#         CMake configuration file for XME Node Manager.
#

#------------------------------------------------------------------------------#
#-     xme_core_nodeManager_binaryManager                                     -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_nodeManager_binaryManager"
    include/binaryManager.h
    src/binaryManager.c
)

#------------------------------------------------------------------------------#
#-     xme_core_nodeManager_neighborhoodDetection                             -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_nodeManager_neighborhoodDetection"
    include/neighborhoodDetection.h
    src/neighborhoodDetection.c
    xme_core_directory_nodeRegistryController
    # TODO: Wrap these files in a separate component
    xme_hal_linkedList
    xme_hal_net
    xme_hal_sharedPtr
    xme_com_interface
    xme_com_packet
)

xme_unit_test(
    "xme_core_nodeManager_neighborhoodDetection"
    TYPE smoke
    test/smokeTestNeighborhoodDetection.cpp
)

xme_unit_test(
    "xme_core_nodeManager_neighborhoodDetection"
    TYPE interface
    test/interfaceTestNeighborhoodDetection.cpp
)

xme_add_component(
    "xme_core_nodeManager_resourceManager"
    include/resourceManager.h
    src/resourceManager.c
)

#------------------------------------------------------------------------------#
#-     xme_core_nodeManager_componentRepository                               -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_nodeManager_componentRepository"
    include/componentRepository.h
    include/componentRepositoryBuilder.h
    include/componentRepositoryPnpClientInterface.h
    include/componentRepositoryPnpManagerInterface.h
    src/componentRepository.c
    xme_core_broker
    xme_core_core
    xme_core_directory_attribute
    xme_core_directory_topicRegistry
    xme_core_manifestRepository
    xme_core_node
    xme_hal_mem
    xme_hal_safeString
    xme_hal_table
)

xme_unit_test(
    "xme_core_nodeManager_componentRepository"
    TYPE interface
    test/interfaceTestComponentRepository.cpp
    MOCK "xme_core_dataHandler" test/mocks.c
    MOCK "xme_core_broker" test/mocks.c
)

xme_code_example(
    "xme_core_nodeManager_componentRepository"
    EXAMPLE "buildComponentInstance"
    FILES
        doc/componentRepository_buildComponentInstance.c
        doc/pnpClientApplicationMock.c
    BUILD
        xme_core_nodeManager_componentRepository
        xme_core_pnp_plugAndPlayClient
        xme_core_pnp_plugAndPlayManager
)
