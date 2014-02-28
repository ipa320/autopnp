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
# $Id: CMakeLists.txt 5106 2013-09-18 15:16:10Z ruiz $
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
#-     xme_core_nodeManager_componentManager                                  -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_nodeManager_componentManager"
    include/componentManager.h
    src/componentManager.c
    xme_core_fallback
    xme_hal_safeString
)

# FIXME: This should be moved to the port part!
# CMAKE_DL_LIBS = 'dl' on Linux, but empty on Windows
if (CMAKE_DL_LIBS)
    xme_get (_XME_TARGET_CPU PROPERTY_GLOBAL XME_TARGET_CPU)
    if (${_XME_TARGET_CPU} STREQUAL "NiosII")
        # on NiosII targets, we don't want the -ldl flag for the linker, even if it is defined
    else ()
        xme_add_component(
            "xme_core_nodeManager_componentManager"
            LINK "${CMAKE_DL_LIBS}"
        )
    endif ()
endif (CMAKE_DL_LIBS)

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
