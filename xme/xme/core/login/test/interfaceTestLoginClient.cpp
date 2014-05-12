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
 * $Id: interfaceTestLoginClient.cpp 7806 2014-03-13 10:17:51Z geisinger $
 */

/**
 * \file
 *         Login Client interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include "xme/core/login/include/loginClient.h"
#include "xme/com/interface.h"
#include "mockLayerForLoginClient.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN
extern xme_status_t status_xme_wp_wci_addConfig;
extern xme_status_t statusDemarshalerWaypointAddConfig;
extern xme_status_t statusXmeWpMarshalMarshalerGetConfig;
extern xme_status_t statusUdpSendWaypointAddConfig;
extern xme_status_t statusXmeCoreExecConfiguratorAddComponentToSchedule;
extern xme_status_t statusXmeWpUdpUdpSendGetConfig;
XME_EXTERN_C_END

class LoginClientInterfaceTest: public ::testing::Test
{
    public :
    LoginClientInterfaceTest()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_node_init());

        xme_com_interface_ipv4StringToGenericAddress("192.168.1.2:33221", &intfAddress);
        xme_core_node_addInterface(intfAddress);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_init());

        xme_wp_marshal_marshaler_getConfigSetStatus(XME_STATUS_SUCCESS);
        xme_core_exec_configurator_addComponentToScheduleSetStatus(XME_STATUS_SUCCESS);
        xme_wp_udp_udpSend_getConfigSetStatus(XME_STATUS_NOT_FOUND);
        udpSendWaypointAddConfigSetStatus(XME_STATUS_SUCCESS);
    }
    ~LoginClientInterfaceTest()
    {
        xme_core_login_loginClient_fini();
        xme_core_node_fini();
    }
    xme_com_interface_address_t intfAddress;
    
};

TEST_F(LoginClientInterfaceTest, invalidGUID)
{
    xme_core_topic_login_loginRequest_t lreq;
    xme_core_topic_login_loginResponse_t lres;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_fillLoginRequest(&lreq));
    EXPECT_EQ(0x201a8c0u, lreq.ipAddress);

    //inject Error
    lres.guid = 0;
    lres.nodeId = (xme_core_node_nodeId_t)1;
    lres.ipAddress = 0x201a8c0u;
    lres.portAddress = 0xC681u;
    lres.channelID = (xme_core_channelId_t) 5000;

    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_login_loginClient_processLoginResponse(&lres));
    EXPECT_EQ(XME_CORE_NODE_INVALID_NODE_ID, xme_core_node_getCurrentNodeId());

    lres.guid = lreq.guid;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_processLoginResponse(&lres));
    EXPECT_EQ((xme_core_node_nodeId_t)1, xme_core_node_getCurrentNodeId());
    xme_core_login_loginClient_fini();
}

TEST_F(LoginClientInterfaceTest, addConfigFailure)
{
    xme_core_topic_login_loginRequest_t lreq;
    xme_core_topic_login_loginResponse_t lres;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginClient_fillLoginRequest(&lreq));
    EXPECT_EQ(0x201a8c0u, lreq.ipAddress);

    lres.guid = lreq.guid;
    lres.nodeId = (xme_core_node_nodeId_t)1;
    lres.ipAddress = 0x201a8c0u;
    lres.portAddress = 0xC681u;
    lres.channelID = (xme_core_channelId_t) 5000;

    //inject Error
    status_xme_wp_wci_addConfig = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_login_loginClient_processLoginResponse(&lres));
    EXPECT_EQ((xme_core_node_nodeId_t)1, xme_core_node_getCurrentNodeId());
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
