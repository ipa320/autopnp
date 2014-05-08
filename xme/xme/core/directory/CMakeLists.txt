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
# $Id: CMakeLists.txt 7483 2014-02-18 16:14:01Z wiesmueller $
#
# File:
#         CMake configuration file for XME directory services.
#

#------------------------------------------------------------------------------#
#-     xme_core_directory_attribute                                           -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_directory_attribute"
    include/attribute.h
    include/attributeInternalTypes.h
    include/attributeInternalMethods.h
    src/attribute.c
    xme_hal_table
)

xme_unit_test(
    "xme_core_directory_attribute"
    TYPE interface
    test/interfaceTestAttribute.cpp
)

xme_unit_test(
    "xme_core_directory_attribute"
    TYPE integration
    test/integrationTestAttribute.cpp
)

#------------------------------------------------------------------------------#
#-     xme_core_directory_topicRegistry                                       -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_directory_topicRegistry"
    include/topicRegistry.h
    src/topicRegistry.c
    xme_hal_table
)

xme_unit_test(
    "xme_core_directory_topicRegistry"
    TYPE interface
    test/interfaceTestTopicRegistry.cpp
)

#------------------------------------------------------------------------------#
#-     xme_core_directory_networkTopologyCalculator                           -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_directory_networkTopologyCalculator"
    include/networkTopologyCalculator.h
    src/networkTopologyCalculator.c
    xme_com_interface
    xme_hal_graph
)

xme_unit_test(
    "xme_core_directory_networkTopologyCalculator"
    TYPE smoke
    test/smokeTestNetworkTopologyCalculator.cpp
)

xme_unit_test(
    "xme_core_directory_networkTopologyCalculator"
    TYPE interface
    test/interfaceTestNetworkTopologyCalculator.cpp
)

#------------------------------------------------------------------------------#
#-     xme_core_directory_nodeRegistryController                              -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_directory_nodeRegistryController"
    include/nodeRegistryController.h
    src/nodeRegistryController.c
    xme_hal_table
    xme_hal_mem
    xme_hal_safeString
    xme_core_node
    xme_com_interface
)

xme_unit_test(
    "xme_core_directory_nodeRegistryController"
    TYPE interface
    test/interfaceTestNodeRegistryController.cpp
    xme_hal_random
)

xme_unit_test(
    "xme_core_directory_nodeRegistryController"
    TYPE smoke
    test/smokeTestNodeRegistryController.cpp
    xme_hal_random
)
