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
# $Id: CMakeLists.txt 7802 2014-03-13 09:04:01Z geisinger $
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
    xme_core_core
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
    TYPE smoke
    test/smokeTestLogicalRouteManager.cpp
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
    xme_core_nodeManager_componentRepository
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
    include-gen/plugAndPlayManagerProcessLogoutRequestsFunction.h
    include-gen/plugAndPlayManagerProcessLogoutRequestsFunctionWrapper.h
    include-gen/plugAndPlayManagerProcessLogoutAcknowledgmentsFunction.h
    include-gen/plugAndPlayManagerProcessLogoutAcknowledgmentsFunctionWrapper.h
    include-gen/plugAndPlayManagerProcessRemoveComponentRequestFunction.h
    include-gen/plugAndPlayManagerProcessRemoveComponentRequestFunctionWrapper.h
    include-gen/plugAndPlayManagerComponent.h
    include-gen/pnpManagerManifest.h
    ../../wp/waypointConfigInfrastructure.h
    src/plugAndPlayManager.c
    src-gen/plugAndPlayManagerComponentWrapper.c
    src-gen/plugAndPlayManagerGenerateRuntimeGraphsFunction.c
    src-gen/plugAndPlayManagerGenerateRuntimeGraphsFunctionWrapper.c
    src-gen/plugAndPlayManagerProcessLoginRegistrationFunction.c
    src-gen/plugAndPlayManagerProcessLoginRegistrationFunctionWrapper.c
    src-gen/plugAndPlayManagerProcessLogoutRequestsFunction.c
    src-gen/plugAndPlayManagerProcessLogoutRequestsFunctionWrapper.c
    src-gen/plugAndPlayManagerProcessLogoutAcknowledgmentsFunction.c
    src-gen/plugAndPlayManagerProcessLogoutAcknowledgmentsFunctionWrapper.c
    src-gen/plugAndPlayManagerProcessRemoveComponentRequestFunction.c
    src-gen/plugAndPlayManagerProcessRemoveComponentRequestFunctionWrapper.c
    src-gen/plugAndPlayManagerComponent.c
    src-gen/pnpManagerManifest.c
    ../../wp/waypointConfigInfrastructure.c
    xme_com_interface
    xme_core_directory_topicRegistry
    xme_core_log
    xme_core_manifestRepository
    xme_core_nodeManager_componentRepository
    xme_core_pnp_configuratorExtension
    xme_core_pnp_logicalRouteManager
    xme_core_pnp_networkConfigurationCalculator
    xme_hal_linkedList
    xme_hal_table
    xme_hal_safeString
    xme_hal_net
)

xme_unit_test(
    "xme_core_pnp_plugAndPlayManager"
    TYPE interface
    MOCK "xme_core_directory_topicRegistry" test/topicRegistryMock.c
    test/interfaceTestPnPManager.cpp
)
 
xme_unit_test(
    "xme_core_pnp_plugAndPlayManager"
    TYPE integration
    MOCK "xme_core_directory_topicRegistry" test/topicRegistryMock.c
    test/integrationTestPnPManager.cpp
    xme_hal_random
)

#------------------------------------------------------------------------------#
#-     xme_core_pnp_plugAndPlayClient                                         -#
#------------------------------------------------------------------------------#
 
xme_add_component(
    "xme_core_pnp_plugAndPlayClient"
    include/plugAndPlayClient.h
    include/plugAndPlayClientConfiguration.h
    include/plugAndPlayClientScheduling.h
    include-gen/plugAndPlayClientComponentWrapper.h
    include-gen/plugAndPlayClientProcessGraphFunction.h
    include-gen/plugAndPlayClientProcessGraphFunctionWrapper.h
    include-gen/plugAndPlayClientSendManifestFunction.h
    include-gen/plugAndPlayClientSendManifestFunctionWrapper.h
    include-gen/plugAndPlayClientSendLogoutRequestFunction.h
    include-gen/plugAndPlayClientSendLogoutRequestFunctionWrapper.h
    include-gen/plugAndPlayClientSendRemoveComponentRequestFunction.h
    include-gen/plugAndPlayClientSendRemoveComponentRequestFunctionWrapper.h
    include-gen/pnpClientManifest.h
    include-gen/plugAndPlayClientComponent.h
    src/plugAndPlayClient.c
    src/plugAndPlayClientConfiguration.c
    src/plugAndPlayClientScheduling.c
    src-gen/plugAndPlayClientComponentWrapper.c
    src-gen/plugAndPlayClientProcessGraphFunction.c
    src-gen/plugAndPlayClientProcessGraphFunctionWrapper.c
    src-gen/plugAndPlayClientSendManifestFunction.c
    src-gen/plugAndPlayClientSendManifestFunctionWrapper.c
    src-gen/plugAndPlayClientSendLogoutRequestFunction.c
    src-gen/plugAndPlayClientSendLogoutRequestFunctionWrapper.c
    src-gen/plugAndPlayClientSendRemoveComponentRequestFunction.c
    src-gen/plugAndPlayClientSendRemoveComponentRequestFunctionWrapper.c
    src-gen/pnpClientManifest.c
    src-gen/plugAndPlayClientComponent.c
    xme_core_dataHandler
    xme_core_directory_nodeRegistryController
    xme_core_executionManager
    xme_core_manifestRepository
    xme_core_nodeManager_componentRepository
)

xme_unit_test(
    "xme_core_pnp_plugAndPlayClient"
    TYPE interface
    test/interfaceTestPnPClient.cpp
    MOCK "xme_core_dataHandler" test/mocks.h test/mocks.c
    MOCK "xme_core_executionManager" test/mocks.h test/mocks.c
    MOCK "xme_core_broker" test/mocks.h test/mocks.c
)

xme_unit_test(
    "xme_core_pnp_plugAndPlayClient"
    TYPE integration
    test/integrationTestPnPClient.cpp
    test/topicRegistryMock.c
    xme_core_pnp_plugAndPlayManager
    xme_hal_random
    MOCK "xme_core_dataHandler" test/mocks.h test/mocks.c
    MOCK "xme_core_executionManager" test/mocks.h test/mocks.c
    MOCK "xme_core_broker" test/mocks.h test/mocks.c
)

#------------------------------------------------------------------------------#
#-     xme_core_pnp_configuratorExtension                                     -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_pnp_configuratorExtension"
    HEADERS
        include/configuratorExtension.h
    SOURCES
        src/configuratorExtension.c
    DEPENDS
        xme_core_pnp_plugAndPlayManager
        xme_core_log
        xme_hal_mem
)

xme_unit_test(
    "xme_core_pnp_configuratorExtension"
    TYPE interface
    test/interfaceTestConfiguratorExtension.cpp
)

xme_unit_test(
    "xme_core_pnp_configuratorExtension"
    TYPE integration
    test/integrationTestConfiguratorExtension.cpp
    #MOCK "xme_core_pnp_plugAndPlayManager" test/configExtPnpManagerMock.c
)
