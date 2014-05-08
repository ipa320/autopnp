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
#         CMake configuration file for XME login services.
#

#------------------------------------------------------------------------------#
#-     xme_core_login_loginClient                                             -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_login_loginClient"
    include/loginClient.h
    ../../wp/waypointConfigInfrastructure.h
    include-gen/loginClientComponentWrapper.h
    include-gen/loginClientProcessResponseFunction.h
    include-gen/loginClientProcessResponseFunctionWrapper.h
    include-gen/loginClientSendRequestFunction.h
    include-gen/loginClientSendRequestFunctionWrapper.h
    include-gen/loginClientManifest.h
    include-gen/loginClientComponent.h
    src/loginClient.c
    ../../wp/waypointConfigInfrastructure.c
    src-gen/loginClientComponentWrapper.c
    src-gen/loginClientProcessResponseFunction.c
    src-gen/loginClientProcessResponseFunctionWrapper.c
    src-gen/loginClientSendRequestFunction.c
    src-gen/loginClientSendRequestFunctionWrapper.c
    src-gen/loginClientManifest.c
    src-gen/loginClientComponent.c    xme_com_interface
    xme_core_node
    xme_core_pnp_plugAndPlayClient

    xme_hal_time
)

xme_unit_test(
    "xme_core_login_loginClient"
    TYPE smoke
    test/smokeTestLoginClient.cpp
    test/mockLayerForLoginClient.c
)

xme_unit_test(
    "xme_core_login_loginClient"
    TYPE interface
    test/interfaceTestLoginClient.cpp
    test/mockLayerForLoginClient.c
    test/mockLayerForLoginClient.h
)

#------------------------------------------------------------------------------#
#-     xme_core_login_loginManagerProxy                                       -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_login_loginManagerProxy"
    include/loginManagerProxy.h
    #src/loginManagerProxy.c
    xme_core_login_loginClient
)

xme_unit_test(
    "xme_core_login_loginManagerProxy"
    TYPE smoke
    test/smokeTestLoginManagerProxy.cpp
)

xme_unit_test(
    "xme_core_login_loginManagerProxy"
    TYPE interface
    test/interfaceTestLoginManagerProxy.cpp
)

xme_unit_test(
    "xme_core_login_loginManagerProxy"
    TYPE integration
    test/integrationTestLoginManagerProxy.cpp
)

#------------------------------------------------------------------------------#
#-     xme_core_login_loginManager                                            -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_login_loginManager"
    include/loginManager.h
    include/loginManagerInternalMethods.h
    include/loginManagerInternalTypes.h
    include-gen/loginManagerComponentWrapper.h
    include-gen/loginManagerProcessRequestFunction.h
    include-gen/loginManagerProcessRequestFunctionWrapper.h
    include-gen/loginManagerProcessResponseFunction.h
    include-gen/loginManagerProcessResponseFunctionWrapper.h
    include-gen/loginManagerProcessLogoutNotificationFunction.h
    include-gen/loginManagerProcessLogoutNotificationFunctionWrapper.h
    include-gen/loginManagerManifest.h
    include-gen/loginManagerComponent.h
    ../../wp/waypointConfigInfrastructure.h
    src/loginManager.c
    src-gen/loginManagerComponentWrapper.c
    src-gen/loginManagerProcessRequestFunction.c
    src-gen/loginManagerProcessRequestFunctionWrapper.c
    src-gen/loginManagerProcessResponseFunction.c
    src-gen/loginManagerProcessResponseFunctionWrapper.c
    src-gen/loginManagerProcessLogoutNotificationFunction.c
    src-gen/loginManagerProcessLogoutNotificationFunctionWrapper.c
    src-gen/loginManagerManifest.c
    src-gen/loginManagerComponent.c
    ../../wp/waypointConfigInfrastructure.c
    xme_core_directory_nodeRegistryController
    xme_core_nodeManager_componentRepository
)

xme_unit_test(
    "xme_core_login_loginManager"
    TYPE smoke
    test/smokeTestLoginManager.cpp
)

xme_unit_test(
    "xme_core_login_loginManager"
    TYPE interface
    test/interfaceTestLoginManager.cpp
)

# xme_unit_test(
    # "xme_core_login_loginManager"
    # TYPE integration
    # test/integrationTestLoginManager.cpp
    # xme_core_login_loginClient
    # test/adv/loginRequest/include/loginRequestComponentWrapper.h
    # test/adv/loginRequest/include/processLoginRequestFunction.h
    # test/adv/loginRequest/include/processLoginRequestFunctionWrapper.h
    # test/adv/loginRequest/src/loginRequestComponentWrapper.c
    # test/adv/loginRequest/src/processLoginRequestFunction.c
    # test/adv/loginRequest/src/processLoginRequestFunctionWrapper.c
    # test/adv/loginResponse/include/loginResponseComponentWrapper.h
    # test/adv/loginResponse/include/processPnPManagerLoginResponseFunction.h
    # test/adv/loginResponse/include/processPnPManagerLoginResponseFunctionWrapper.h    
    # test/adv/loginResponse/src/loginResponseComponentWrapper.c
    # test/adv/loginResponse/src/processPnPManagerLoginResponseFunction.c
    # test/adv/loginResponse/src/processPnPManagerLoginResponseFunctionWrapper.c
    # MOCK "xme_core_dataHandler" test/loginManagerDataHandlerMock.c
    # MOCK "xme_core_executionManager" test/loginManagerExecutionManagerMock.c
# )

#------------------------------------------------------------------------------#
#-     xme_core_login_loginClientProxy                                        -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_login_loginClientProxy"
    include/loginClientProxy.h
    #src/loginClientProxy.c
    xme_core_login_loginManager
)

xme_unit_test(
    "xme_core_login_loginClientProxy"
    TYPE smoke
    test/smokeTestLoginClientProxy.cpp
)

xme_unit_test(
    "xme_core_login_loginClientProxy"
    TYPE interface
    test/interfaceTestLoginClientProxy.cpp
)

xme_unit_test(
    "xme_core_login_loginClientProxy"
    TYPE integration
    test/integrationTestLoginClientProxy.cpp
)
