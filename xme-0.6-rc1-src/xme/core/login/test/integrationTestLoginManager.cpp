/*
 * Copyright (c) 2011-2013, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: integrationTestLoginManager.cpp 4431 2013-07-31 08:24:55Z ruiz $
 */

/**
 * \file
 *         Login Manager integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/login/include/loginManager.h"
#include "xme/core/directory/include/nodeRegistryController.h"
#include "xme/core/login/test/adv/loginRequest/include/processLoginRequestFunctionWrapper.h"
#include "xme/core/login/test/adv/loginResponse/include/processPnPManagerLoginResponseFunctionWrapper.h"

#include "xme/core/dataHandler/include/dataHandler.h"

#include "xme/core/node.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

#include "xme/core/topicData.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class LoginManagerIntegrationTest: public ::testing::Test
{
    protected:
    // constructor
    LoginManagerIntegrationTest()
        //:nodeId1((xme_core_node_nodeId_t) 1)
        //,nodeId2((xme_core_node_nodeId_t) 2)
        //,nodeIdInvalid(XME_CORE_NODE_INVALID_NODE_ID)
        //,guid0(0)
        //,guid1(512)
        //,guid2(1024)
    {
        pnpLoginRequest = (xme_core_topic_login_pnpLoginRequest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginRequest_t));

        functionDescriptor = (xme_core_exec_functionDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t)); 
        functionDescriptor->componentId = (xme_core_component_t) 1; 
        functionDescriptor->functionId = (xme_core_component_functionId_t) 1; 
    }

    virtual ~LoginManagerIntegrationTest()
    {
        xme_core_login_loginManager_fini();
        xme_core_directory_nodeRegistryController_fini();
    }

#if 0
    xme_core_node_nodeId_t nodeId1;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeIdInvalid;

    xme_core_node_guid_t guid0;
    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;

    xme_core_topic_login_loginRequest_t* loginRequest1;
    xme_com_interface_address_t interfaceAddress1;

    xme_core_topic_login_loginRequest_t* loginRequest2;
    xme_com_interface_address_t interfaceAddress2;

    xme_core_topic_login_loginRequest_t* loginRequest3;
    xme_com_interface_address_t interfaceAddress3;

    xme_core_topic_login_loginRequest_t* loginRequest4;
    xme_com_interface_address_t interfaceAddress4;

    xme_com_interface_address_t interfaceAddress;
#endif 
    xme_core_topic_login_pnpLoginRequest_t* pnpLoginRequest;

    xme_core_exec_functionDescriptor_t* functionDescriptor;
};

//TEST_F(LoginManagerIntegrationTest, TestExecutingFunctionWrapper)
//{
//    login_adv_loginRequest_processLoginRequestFunctionWrapper_execute((void*)functionDescriptor);
//}

TEST_F(LoginManagerIntegrationTest, TestExecutingPnPLoginFunctionWrapper)
{
    login_adv_loginResponse_processPnPManagerLoginResponseFunctionWrapper_execute((void*)functionDescriptor);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
