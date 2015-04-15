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
 * $Id: interfaceTestLoginManager.cpp 6623 2014-02-05 10:43:23Z wiesmueller $
 */

/**
 * \file
 *         Login Manager interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/login/include/loginManager.h"
#include "xme/core/login/include/loginManagerInternalMethods.h"

#include "xme/core/directory/include/nodeRegistryController.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

#include "xme/core/topicData.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class LoginManagerRequestInterfaceTest: public ::testing::Test
{
    protected:
    // constructor
    LoginManagerRequestInterfaceTest()
        :nodeId1((xme_core_node_nodeId_t) 1)
        ,nodeId2((xme_core_node_nodeId_t) 2)
        ,nodeIdInvalid(XME_CORE_NODE_INVALID_NODE_ID)
        ,guid0(0)
        ,guid1(512)
        ,guid2(1024)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        loginRequest1 = (xme_core_topic_login_loginRequest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_loginRequest_t));
        loginRequest2 = (xme_core_topic_login_loginRequest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_loginRequest_t));
        loginRequest3 = (xme_core_topic_login_loginRequest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_loginRequest_t));
        loginRequest4 = (xme_core_topic_login_loginRequest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_loginRequest_t));

        loginRequest1->guid = (xme_core_node_guid_t)256;
        loginRequest1->nodeId = (xme_core_node_nodeId_t)1;
        EXPECT_TRUE(0 < xme_hal_safeString_snprintf(loginRequest1->nodeName, sizeof(loginRequest1->nodeName), "node1"));
        loginRequest2->guid = (xme_core_node_guid_t)512;
        loginRequest2->nodeId = (xme_core_node_nodeId_t)2;
        EXPECT_TRUE(0 < xme_hal_safeString_snprintf(loginRequest2->nodeName, sizeof(loginRequest2->nodeName), "node2"));
        loginRequest3->guid = (xme_core_node_guid_t)1024;
        loginRequest3->nodeId = (xme_core_node_nodeId_t)3;
        EXPECT_TRUE(0 < xme_hal_safeString_snprintf(loginRequest3->nodeName, sizeof(loginRequest3->nodeName), "node3"));
        loginRequest4->guid = (xme_core_node_guid_t)0;
        loginRequest4->nodeId = (xme_core_node_nodeId_t)0;
        EXPECT_TRUE(0 < xme_hal_safeString_snprintf(loginRequest4->nodeName, sizeof(loginRequest4->nodeName), "node4"));

        pnpLoginRequest = (xme_core_topic_login_pnpLoginRequest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginRequest_t));
    }

    virtual ~LoginManagerRequestInterfaceTest()
    {
        xme_core_login_loginManager_fini();
        xme_core_directory_nodeRegistryController_fini();
    }

    xme_core_node_nodeId_t nodeId1;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeIdInvalid;

    xme_core_node_guid_t guid0;
    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;

    xme_core_topic_login_loginRequest_t* loginRequest1;
    xme_core_topic_login_loginRequest_t* loginRequest2;
    xme_core_topic_login_loginRequest_t* loginRequest3;
    xme_core_topic_login_loginRequest_t* loginRequest4;

    xme_core_topic_login_pnpLoginRequest_t* pnpLoginRequest;
};

class LoginManagerResponseInterfaceTest: public ::testing::Test
{
    protected:
    // constructor
    LoginManagerResponseInterfaceTest()
        :nodeId1((xme_core_node_nodeId_t) 1)
        ,nodeId2((xme_core_node_nodeId_t) 2)
        ,nodeIdInvalid(XME_CORE_NODE_INVALID_NODE_ID)
        ,guid0(0)
        ,guid1(256)
        ,guid2(512)
        ,guid3(1024)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        pnpLoginResponse1 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));
        pnpLoginResponse2 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));
        pnpLoginResponse3 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));
        pnpLoginResponse4 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));

        pnpLoginResponse1->nodeId = nodeId1;
        pnpLoginResponse2->nodeId = nodeId1;
        pnpLoginResponse3->nodeId = nodeId2;
        pnpLoginResponse4->nodeId = XME_CORE_NODE_INVALID_NODE_ID;

        ip = "192.168.0.1:33331";
        port = 33331u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse1->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse1->portAddress, &portNetworkByteOrder, 2);

        ip = "192.168.0.2:33332";
        port = 33332u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse2->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse2->portAddress, &portNetworkByteOrder, 2);

        ip = "192.168.0.3:33333";
        port = 33333u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse3->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse3->portAddress, &portNetworkByteOrder, 2);

        ip = "192.168.0.4:33334";
        port = 33334u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse4->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse4->portAddress, &portNetworkByteOrder, 2);

        loginResponse = (xme_core_topic_login_loginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_loginResponse_t));
    }

    virtual ~LoginManagerResponseInterfaceTest()
    {
        xme_core_login_loginManager_fini();
        xme_core_directory_nodeRegistryController_fini();
    }

    xme_core_node_nodeId_t nodeId1;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeIdInvalid;

    xme_core_node_guid_t guid0;
    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;
    xme_core_node_guid_t guid3;

    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse1;
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse2;
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse3;
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse4;

    xme_core_topic_login_loginResponse_t* loginResponse;

    const char* ip;
    uint16_t port;
    uint32_t ipNetworkByteOrder;
    uint16_t portNetworkByteOrder;
    xme_com_interface_address_t interfaceAddress;
};

//----------------------------------------------------------------------------//
//     LoginManager: request processing                                       //
//----------------------------------------------------------------------------//

/// calculateNewNodeId
TEST_F(LoginManagerRequestInterfaceTest, CalculateNewNodeIdWithExistingNodeId)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_calculateNewNodeId(&nodeId1));
}

TEST_F(LoginManagerRequestInterfaceTest, CalculateNewNodeIdWithInvalidNodeId)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_calculateNewNodeId(&nodeIdInvalid));
}

/// checkAlreadyLoggedIn
TEST_F(LoginManagerRequestInterfaceTest, CheckAlreadyLoggedInProvidingGeneratedGUIDs)
{
    xme_core_node_nodeId_t nodeID = XME_CORE_NODE_INVALID_NODE_ID;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_checkAlreadyLoggedIn(guid1, NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_checkAlreadyLoggedIn(guid1, &nodeID));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_checkAlreadyLoggedIn(guid2, NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_checkAlreadyLoggedIn(guid2, &nodeID));
}

// loginRequest
TEST_F(LoginManagerRequestInterfaceTest, LoginRequestWithNullParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginManager_loginRequest(NULL, NULL));
}

TEST_F(LoginManagerRequestInterfaceTest, LoginRequestWithRequestParameterSetToNull)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginManager_loginRequest(NULL, NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginManager_loginRequest(NULL, pnpLoginRequest));
}

TEST_F(LoginManagerRequestInterfaceTest, LoginRequestWithValidRequestAndDifferentNodeValues)
{
    xme_core_node_nodeId_t tmpNodeId;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_loginRequest(loginRequest1, pnpLoginRequest));
    tmpNodeId = (xme_core_node_nodeId_t) pnpLoginRequest->nodeId;
    EXPECT_NE(XME_CORE_NODE_INVALID_NODE_ID, tmpNodeId);
    EXPECT_NE(XME_CORE_NODE_LOCAL_NODE_ID, tmpNodeId);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_loginRequest(loginRequest2, pnpLoginRequest));
    EXPECT_NE(tmpNodeId, (xme_core_node_nodeId_t) pnpLoginRequest->nodeId);
    tmpNodeId = (xme_core_node_nodeId_t) pnpLoginRequest->nodeId;
    EXPECT_NE(XME_CORE_NODE_INVALID_NODE_ID, tmpNodeId);
    EXPECT_NE(XME_CORE_NODE_LOCAL_NODE_ID, tmpNodeId);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_loginRequest(loginRequest3, pnpLoginRequest));
    EXPECT_NE(tmpNodeId, (xme_core_node_nodeId_t) pnpLoginRequest->nodeId);
    EXPECT_NE(XME_CORE_NODE_INVALID_NODE_ID, tmpNodeId);
    EXPECT_NE(XME_CORE_NODE_LOCAL_NODE_ID, tmpNodeId);
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginManager_loginRequest(loginRequest4, pnpLoginRequest));
}

TEST_F(LoginManagerRequestInterfaceTest, LoginRequestWithValidRequestTwoTimes)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_loginRequest(loginRequest1, pnpLoginRequest));
    EXPECT_NE(XME_CORE_NODE_INVALID_NODE_ID, (xme_core_node_nodeId_t) pnpLoginRequest->nodeId);
    EXPECT_NE(XME_CORE_NODE_LOCAL_NODE_ID, (xme_core_node_nodeId_t) pnpLoginRequest->nodeId);
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_login_loginManager_loginRequest(loginRequest1, pnpLoginRequest));
}


TEST_F(LoginManagerRequestInterfaceTest, LoginRequestForTestingGetNodeId)
{
    xme_core_node_nodeId_t nodeId;

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_login_loginManager_getNodeId(loginRequest1->guid, &nodeId));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_loginRequest(loginRequest1, pnpLoginRequest));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_getNodeId(loginRequest1->guid, &nodeId));
    EXPECT_EQ(nodeId, pnpLoginRequest->nodeId);
}

//----------------------------------------------------------------------------//
//     LoginManager: response processing                                      //
//----------------------------------------------------------------------------//

/// loginResponse
TEST_F(LoginManagerResponseInterfaceTest, LoginResponseWithNullParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginManager_loginResponse(NULL, NULL));
}

TEST_F(LoginManagerResponseInterfaceTest, LoginResponseWithPnPResponseParameterSetToNull)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginManager_loginResponse(NULL, NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginManager_loginResponse(NULL, loginResponse));
}

TEST_F(LoginManagerResponseInterfaceTest, LoginResponseWithoutRegistrationOfNode)
{
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_login_loginManager_loginResponse(pnpLoginResponse1, loginResponse));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(pnpLoginResponse1->nodeId, "X", guid1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_loginResponse(pnpLoginResponse1, loginResponse));
}


TEST_F(LoginManagerResponseInterfaceTest, LoginResponseWithValidPnPResponseAndDifferentNodeValues)
{
    xme_core_node_nodeId_t tmpNodeId;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(pnpLoginResponse1->nodeId, "X", guid1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_loginResponse(pnpLoginResponse1, loginResponse));
    tmpNodeId = (xme_core_node_nodeId_t) loginResponse->nodeId;
    EXPECT_NE(XME_CORE_NODE_INVALID_NODE_ID, tmpNodeId);
    EXPECT_EQ(XME_CORE_NODE_LOCAL_NODE_ID, tmpNodeId);
    EXPECT_EQ(guid1, loginResponse->guid);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_loginResponse(pnpLoginResponse2, loginResponse));
    EXPECT_EQ(tmpNodeId, (xme_core_node_nodeId_t) loginResponse->nodeId); // they both belong to the same node. 
    tmpNodeId = (xme_core_node_nodeId_t) loginResponse->nodeId;
    EXPECT_NE(XME_CORE_NODE_INVALID_NODE_ID, tmpNodeId);
    EXPECT_EQ(XME_CORE_NODE_LOCAL_NODE_ID, tmpNodeId);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(pnpLoginResponse3->nodeId, "Y", guid2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_loginResponse(pnpLoginResponse3, loginResponse));
    EXPECT_NE(tmpNodeId, (xme_core_node_nodeId_t) loginResponse->nodeId);
    tmpNodeId = (xme_core_node_nodeId_t) loginResponse->nodeId;
    EXPECT_NE(XME_CORE_NODE_INVALID_NODE_ID, tmpNodeId);
    EXPECT_NE(XME_CORE_NODE_LOCAL_NODE_ID, tmpNodeId);
    
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginManager_loginResponse(pnpLoginResponse4, loginResponse));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
