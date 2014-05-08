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
 * $Id: interfaceTestWCI.cpp 7483 2014-02-18 16:14:01Z wiesmueller $
 */

/**
 * \file
 *         Waypoint Configuration Infrastructure interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include "xme/com/interface.h"
#include "xme/wp/waypointConfigInfrastructure.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN
extern xme_status_t status_createDemarshalerWaypointInstance;
extern xme_status_t status_demarshalerWaypointAddConfig;
extern xme_status_t status_xme_core_exec_componentRepository_registerComponent;
extern xme_status_t status_xme_core_exec_configurator_addComponentToSchedule;
extern xme_status_t status_xme_core_exec_configurator_rollback;
extern xme_status_t status_xme_core_exec_configurator_commit;
extern xme_status_t status_xme_core_broker_addDataPacketTransferEntry;
extern xme_status_t status_createMarshalerWaypointInstance;
extern xme_status_t status_marshalerWaypointAddConfig;
extern xme_status_t status_createUdpSendWaypointInstance;
extern xme_status_t status_udpSendWaypointAddConfig;
extern xme_status_t status_createUdpReceiveWaypointInstance;
extern xme_status_t status_udpReceiveWaypointAddConfig;
XME_EXTERN_C_END

class WCIInterfaceTest: public ::testing::Test
{
    public :
    WCIInterfaceTest()
    {
        ipAddress = 0x201a8c0u; //192.168.1.2
        portAddress = 0xC681u; //33222
        channelID = (xme_core_channelId_t)5000u;
        status_createDemarshalerWaypointInstance = XME_STATUS_SUCCESS;
        status_demarshalerWaypointAddConfig = XME_STATUS_SUCCESS;
        status_xme_core_exec_componentRepository_registerComponent = XME_STATUS_SUCCESS;
        status_xme_core_exec_configurator_addComponentToSchedule = XME_STATUS_SUCCESS;
        status_xme_core_exec_configurator_rollback = XME_STATUS_SUCCESS;
        status_xme_core_exec_configurator_commit = XME_STATUS_SUCCESS;
        status_xme_core_broker_addDataPacketTransferEntry = XME_STATUS_SUCCESS;
        status_createMarshalerWaypointInstance = XME_STATUS_SUCCESS;
        status_marshalerWaypointAddConfig = XME_STATUS_SUCCESS;
        status_createUdpSendWaypointInstance = XME_STATUS_SUCCESS;
        status_udpSendWaypointAddConfig = XME_STATUS_SUCCESS;
        status_createUdpReceiveWaypointInstance = XME_STATUS_SUCCESS;
        status_udpReceiveWaypointAddConfig = XME_STATUS_SUCCESS;
    }
    ~WCIInterfaceTest()
    {
    }
    uint32_t ipAddress;
    uint16_t portAddress;
    xme_core_channelId_t channelID;
    xme_core_topic_t topic;
};

TEST_F(WCIInterfaceTest, createDemarshalerWaypointInstanceFailure)
{
    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;

    //inject Error
    status_createDemarshalerWaypointInstance = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(0, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, demarshalerWaypointAddConfigFailure)
{
    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;

    //inject Error
    status_demarshalerWaypointAddConfig = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_wp_wci_addConfig(0, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, registerComponentFailure)
{
    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;

    //inject Error
    status_xme_core_exec_componentRepository_registerComponent = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(0, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, addComponentToScheduleFailure)
{
    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;

    //inject Error
    status_xme_core_exec_configurator_addComponentToSchedule = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(0, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, createUdpReceiveWaypointInstanceFailure)
{
    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;

    //inject Error
    status_createUdpReceiveWaypointInstance = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(0, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, udpReceiveWaypointAddConfigFailure)
{
    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;

    //inject Error
    status_udpReceiveWaypointAddConfig = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(0, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, commitFailure)
{
    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;

    //inject Error
    status_xme_core_exec_configurator_commit = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(0, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, addDataPacketTransferEntryFailure)
{
    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;

    //inject Error
    status_xme_core_broker_addDataPacketTransferEntry = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(0, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, createMarshalerWaypointInstanceFailureForRTGM)
{
    topic = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL;

    //inject Error
    status_createMarshalerWaypointInstance = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, marshalerWaypointAddConfigFailureForRTGM)
{
    topic = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL;

    //inject Error
    status_marshalerWaypointAddConfig = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, registerComponentFailureForRTGM)
{
    topic = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL;

    //inject Error
    status_xme_core_exec_componentRepository_registerComponent = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, addComponentToScheduleFailureForRTGM)
{
    topic = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL;

    //inject Error
    status_xme_core_exec_configurator_addComponentToSchedule = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, createUdpSendWaypointInstanceFailureForRTGM)
{
    topic = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL;

    //inject Error
    status_createUdpSendWaypointInstance = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, udpSendWaypointAddConfigFailureForRTGM)
{
    topic = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL;

    //inject Error
    status_udpSendWaypointAddConfig = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, commitFailureForRTGM)
{
    topic = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL;

    //inject Error
    status_xme_core_exec_configurator_commit = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, addDataPacketTransferEntryFailureForRTGM)
{
    topic = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL;

    //inject Error
    status_xme_core_broker_addDataPacketTransferEntry = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, createMarshalerWaypointInstanceFailureForLoginResponse)
{
    topic = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE;

    //inject Error
    status_createMarshalerWaypointInstance = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, marshalerWaypointAddConfigFailureForLoginResponse)
{
    topic = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE;

    //inject Error
    status_marshalerWaypointAddConfig = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, registerComponentFailureForLoginResponse)
{
    topic = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE;

    //inject Error
    status_xme_core_exec_componentRepository_registerComponent = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, addComponentToScheduleFailureForLoginResponse)
{
    topic = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE;

    //inject Error
    status_xme_core_exec_configurator_addComponentToSchedule = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, createUdpSendWaypointInstanceFailureForLoginResponse)
{
    topic = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE;

    //inject Error
    status_createUdpSendWaypointInstance = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, udpSendWaypointAddConfigFailureForLoginResponse)
{
    topic = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE;

    //inject Error
    status_udpSendWaypointAddConfig = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, commitFailureForAnotherTopicForLoginResponse)
{
    topic = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE;

    //inject Error
    status_xme_core_exec_configurator_commit = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

TEST_F(WCIInterfaceTest, addDataPacketTransferEntryFailureForLoginResponse)
{
    topic = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE;

    //inject Error
    status_xme_core_broker_addDataPacketTransferEntry = XME_STATUS_INTERNAL_ERROR;

    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
