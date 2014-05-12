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
 * $Id: smokeTestLoginClient.cpp 5266 2013-10-01 12:55:29Z gulati $
 */

/**
 * \file
 *         Login Client smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include "xme/core/login/include/loginClient.h"
#include "xme/com/interface.h"


/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class LoginClientBasicSmokeTest: public ::testing::Test
{
    protected :
    LoginClientBasicSmokeTest()
    {
    }

    ~LoginClientBasicSmokeTest()
    {
    }
    xme_com_interface_address_t intfAddress;
};

class LoginClientSmokeTest: public ::testing::Test
{
    public :
    LoginClientSmokeTest()
    {
        xme_core_node_init();
        xme_com_interface_ipv4StringToGenericAddress("192.168.1.2:33221", &intfAddress);
        xme_core_node_addInterface(intfAddress);
    }
    ~LoginClientSmokeTest()
    {
        xme_core_node_fini();
    }
    xme_com_interface_address_t intfAddress;
};

TEST_F(LoginClientBasicSmokeTest, initWithUninitializedParameter)
{
    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_login_loginClient_init());

    // Init the node component. 
    // TODO: Check what is the workflow for login client initialization. Node component initialization is not documented. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_init());
    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_login_loginClient_init());

    // Add an interface.
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.1.2:33221", &intfAddress));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_addInterface(intfAddress));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_init());

    xme_core_login_loginClient_fini();

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_init());

    xme_core_node_fini();
}

TEST_F(LoginClientSmokeTest, fillLoginRequest)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginClient_fillLoginRequest(NULL));
}

TEST_F(LoginClientSmokeTest, processLoginResponse)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_login_loginClient_processLoginResponse(NULL));
}

TEST_F(LoginClientSmokeTest, sunnyDayTest)
{
    xme_core_topic_login_loginRequest_t lreq;
    xme_core_topic_login_loginResponse_t lres;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_fillLoginRequest(&lreq));
    EXPECT_EQ(0x201a8c0u, lreq.ipAddress);

    lres.guid = lreq.guid;
    lres.nodeId = (xme_core_node_nodeId_t)1;
    lres.ipAddress = 0x201a8c0u;
    lres.portAddress = 33222;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_processLoginResponse(&lres));
    EXPECT_EQ((xme_core_node_nodeId_t)1, xme_core_node_getCurrentNodeId());

    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_login_loginClient_init());

    xme_core_login_loginClient_fini();
}

TEST_F(LoginClientSmokeTest, invalidNodeIdReceived)
{
    xme_core_topic_login_loginRequest_t lreq;
    xme_core_topic_login_loginResponse_t lres;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_fillLoginRequest(&lreq));
    EXPECT_EQ(0x201a8c0u, lreq.ipAddress);

    lres.guid = lreq.guid;
    lres.nodeId = (xme_core_node_nodeId_t)0;
    lres.ipAddress = 0x201a8c0u;
    lres.portAddress = 33222;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_login_loginClient_processLoginResponse(&lres));

    xme_core_login_loginClient_fini();
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
