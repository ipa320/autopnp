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
# $Id: CMakeLists.txt 5108 2013-09-18 17:50:19Z geisinger $
#
# File:
#         CMake configuration file for XME plug and play.
#

#------------------------------------------------------------------------------#
#-     xme_core_pnp_dataLinkGraph                                             -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_pnp_dataLinkGraph"
    include/dataLinkGraph.h
    src/dataLinkGraph.c
    xme_core_log
    xme_hal_graph
)

#------------------------------------------------------------------------------#
#-     xme_core_pnp_logicalRouteManager                                       -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_pnp_logicalRouteManager"
    include/logicalRouteManager.h
    src/logicalRouteManager.c
    xme_core_directory_attribute
    xme_core_pnp_dataLinkGraph
    xme_hal_linkedList
)

xme_unit_test(
    "xme_core_pnp_logicalRouteManager"
    TYPE interface
    test/interfaceTestLogicalRouteManager.cpp
)

#------------------------------------------------------------------------------#
#-     xme_core_pnp_networkConfigurationCalculator                            -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_pnp_networkConfigurationCalculator"
    include/networkConfigurationCalculator.h
    src/networkConfigurationCalculator.c
    xme_com_interface
    xme_core_directory_nodeRegistryController
    xme_core_pnp_dataLinkGraph
    xme_hal_linkedList
    xme_hal_table
)

xme_unit_test(
    "xme_core_pnp_networkConfigurationCalculator"
    TYPE interface
    test/interfaceTestNetworkConfigurationCalculator.cpp
)

#------------------------------------------------------------------------------#
#-     xme_core_pnp_plugAndPlayManager                                        -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_pnp_plugAndPlayManager"
    include/plugAndPlayManager.h
    include/plugAndPlayManagerInternalTypes.h
    include/plugAndPlayManagerInternalMethods.h
    include-gen/plugAndPlayManagerComponentWrapper.h
    include-gen/plugAndPlayManagerGenerateRuntimeGraphsFunction.h
    include-gen/plugAndPlayManagerGenerateRuntimeGraphsFunctionWrapper.h
    include-gen/plugAndPlayManagerProcessLoginRegistrationFunction.h
    include-gen/plugAndPlayManagerProcessLoginRegistrationFunctionWrapper.h
    src/plugAndPlayManager.c
    src-gen/plugAndPlayManagerComponentWrapper.c
    src-gen/plugAndPlayManagerGenerateRuntimeGraphsFunction.c
    src-gen/plugAndPlayManagerGenerateRuntimeGraphsFunctionWrapper.c
    src-gen/plugAndPlayManagerProcessLoginRegistrationFunction.c
    src-gen/plugAndPlayManagerProcessLoginRegistrationFunctionWrapper.c
    xme_com_interface
    xme_core_pnp_logicalRouteManager
    xme_core_pnp_networkConfigurationCalculator
    xme_core_log
    xme_core_manifestRepository
    xme_hal_linkedList
    xme_hal_table
    xme_hal_safeString
    xme_hal_net
)

xme_unit_test(
    "xme_core_pnp_plugAndPlayManager"
    TYPE interface
    test/interfaceTestPnPManager.cpp
)
 
xme_unit_test(
    "xme_core_pnp_plugAndPlayManager"
    TYPE integration
    test/integrationTestPnPManager.cpp
    xme_hal_random
)

#------------------------------------------------------------------------------#
#-     xme_core_pnp_plugAndPlayClient                                         -#
#------------------------------------------------------------------------------#
 
xme_add_component(
    "xme_core_pnp_plugAndPlayClient"
    include/plugAndPlayClient.h
    include-gen/plugAndPlayClientComponentWrapper.h
    include-gen/plugAndPlayClientProcessGraphFunction.h
    include-gen/plugAndPlayClientProcessGraphFunctionWrapper.h
    include-gen/plugAndPlayClientSendManifestFunction.h
    include-gen/plugAndPlayClientSendManifestFunctionWrapper.h
    src/plugAndPlayClient.c
    src-gen/plugAndPlayClientComponentWrapper.c
    src-gen/plugAndPlayClientProcessGraphFunction.c
    src-gen/plugAndPlayClientProcessGraphFunctionWrapper.c
    src-gen/plugAndPlayClientSendManifestFunction.c
    src-gen/plugAndPlayClientSendManifestFunctionWrapper.c
    xme_com_interface
    xme_core_manifestRepository
    xme_core_pnp_networkConfigurationCalculator
)

xme_unit_test(
    "xme_core_pnp_plugAndPlayClient"
    TYPE smoke
    test/smokeTestPnPClient.cpp
)

xme_unit_test(
    "xme_core_pnp_plugAndPlayClient"
    TYPE integration
    test/integrationTestPnPClient.cpp
    xme_core_pnp_plugAndPlayManager
    MOCK "applicationClient" test/applicationClientMock.c
)
