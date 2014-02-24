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
 * $Id: interfaceTestUDP.cpp 5190 2013-09-26 13:10:19Z camek $
 */

/**
 * \file
 *         UDP Waypoint interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include <gtest/gtest.h>

#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/log.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/sleep.h"
#include "xme/hal/include/time.h"

#include "xme/wp/udp/include/udpSend.h"
#include "xme/wp/udp/include/udpReceive.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

#define STRING  "ThE qUiCk BrOwN fOx JuMpS oVeR tHe LaZy DoG"
#define STRING2 "this is the second string for broadcast"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class UDPWaypointInterfaceTest : public ::testing::Test
{
protected:
    UDPWaypointInterfaceTest()
    : recvBuffer(NULL)
    , udpReceiveInstanceId1(XME_WP_WAYPOINT_INSTANCEID_INVALID)
    , udpReceiveInstanceId2(XME_WP_WAYPOINT_INSTANCEID_INVALID)
    , dataPortReceive(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)
    , sendBuffer(NULL)
    , udpSendInstanceId1(XME_WP_WAYPOINT_INSTANCEID_INVALID)
    , udpSendInstanceId2(XME_WP_WAYPOINT_INSTANCEID_INVALID)
    , dataPortSend(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)
    , topicSend(NULL)
    , topicRecv(NULL)
    {
        xme_hal_sync_init();
        xme_hal_net_init();
        xme_wp_udp_udpReceive_init();
        xme_wp_udp_udpSend_init();
    }

    virtual ~UDPWaypointInterfaceTest()
    {
        xme_wp_udp_udpSend_fini();
        xme_wp_udp_udpReceive_fini();
        xme_hal_net_fini();
        xme_hal_sync_fini();
    }

    void udpRecvTest(uint32_t port, int topicS, uint8_t key[])
    {
        dataPortReceive = (xme_core_dataManager_dataPacketId_t) 1;

        ASSERT_TRUE(NULL != recvBuffer);

        status = xme_wp_udp_udpReceive_addConfig
        (
            &udpReceiveInstanceId1,
            dataPortReceive,
            topicS,
            recvBuffer,
            topicS + xme_wp_udp_udpReceive_getPackageOverHead(),
            key,
            port
        );
        
        ASSERT_EQ(XME_STATUS_SUCCESS, status);

        ASSERT_NE(XME_WP_WAYPOINT_INSTANCEID_INVALID, udpReceiveInstanceId1);

        XME_LOG(XME_LOG_ALWAYS,"Configured Recv Waypoint\n");
    }

    void udpSendTest(const char* host, uint32_t port, uint16_t topicS, uint8_t key[])
    {
        dataPortSend = (xme_core_dataManager_dataPacketId_t) 2;

        ASSERT_TRUE(NULL != sendBuffer);

        xme_wp_udp_udpSend_addConfig
        (
            &udpSendInstanceId1,
            dataPortSend,
            topicS,
            sendBuffer,
            topicS + xme_wp_udp_udpSend_getPackageOverHead(),
            key,
            host,
            port,
            false
        );

        ASSERT_EQ(XME_STATUS_SUCCESS, status);

        ASSERT_NE(XME_WP_WAYPOINT_INSTANCEID_INVALID, udpSendInstanceId1);

        XME_LOG(XME_LOG_ALWAYS,"Configured Send Waypoint\n");
    }

    void udpRecvTestBroadcast(uint32_t port, int topicS, uint8_t key[])
    {
        dataPortReceive = (xme_core_dataManager_dataPacketId_t) 3;

        ASSERT_TRUE(NULL != recvBuffer);

        status = xme_wp_udp_udpReceive_addConfig
        (
            &udpReceiveInstanceId2,
            dataPortReceive,
            topicS,
            recvBuffer,
            topicS + xme_wp_udp_udpReceive_getPackageOverHead(),
            key,
            port
        );

        ASSERT_EQ(XME_STATUS_SUCCESS, status);

        ASSERT_NE(XME_WP_WAYPOINT_INSTANCEID_INVALID, udpReceiveInstanceId2);

        XME_LOG(XME_LOG_ALWAYS,"Configured Recv Waypoint for Broadcast\n");
    }

    void udpSendTestBroadcast(const char* host, uint32_t port, int topicS, uint8_t key[])
    {
        dataPortSend = (xme_core_dataManager_dataPacketId_t) 4;

        ASSERT_TRUE(NULL != sendBuffer);

        status = xme_wp_udp_udpSend_addConfig
        (
            &udpSendInstanceId2,
            dataPortSend,
            topicS,
            sendBuffer,
            topicS + xme_wp_udp_udpSend_getPackageOverHead(),
            key,
            host,
            port,
            true
        );

        ASSERT_EQ(XME_STATUS_SUCCESS, status);

        ASSERT_NE(XME_WP_WAYPOINT_INSTANCEID_INVALID, udpSendInstanceId2);

        XME_LOG(XME_LOG_ALWAYS,"Configured Send Waypoint for Broadcast\n");
    }

    char* recvBuffer;
    xme_wp_waypoint_instanceId_t udpReceiveInstanceId1;
    xme_wp_waypoint_instanceId_t udpReceiveInstanceId2;
    xme_core_dataManager_dataPacketId_t dataPortReceive;

    char* sendBuffer;
    xme_wp_waypoint_instanceId_t udpSendInstanceId1;
    xme_wp_waypoint_instanceId_t udpSendInstanceId2;
    xme_core_dataManager_dataPacketId_t dataPortSend;

    char* topicSend;
    char* topicRecv;

    xme_status_t status;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     UDPWaypointInterfaceTest                                               //
//----------------------------------------------------------------------------//

TEST_F(UDPWaypointInterfaceTest, sendAndRead)
{
    uint32_t bytesRead, stringLen;
    uint32_t loopCount = 10;
    uint8_t key1[] = { 1, 2, 3, 4 };

    stringLen = strlen(STRING) + 1;
    topicRecv = (char*) xme_hal_mem_alloc(stringLen);
    topicSend = (char*) xme_hal_mem_alloc(stringLen);
    xme_hal_safeString_strncpy(topicSend, STRING, stringLen);

    recvBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpReceive_getPackageOverHead());
    sendBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpSend_getPackageOverHead());

    udpRecvTest(33221u, strlen(topicSend), key1);
    udpSendTest("127.0.0.1", 33221u, (uint16_t) strlen(topicSend), key1);

    xme_core_dataHandler_writeData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicSend, strlen(topicSend));
    xme_wp_udp_udpSend_run(udpSendInstanceId1);
    while (loopCount-- != 0)
    {
        XME_LOG(XME_LOG_ALWAYS," sleeping in sendAndRead with loop count %d\n", loopCount);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
        xme_wp_udp_udpReceive_run(udpReceiveInstanceId1);
    }
    xme_core_dataHandler_readData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicRecv, stringLen, &bytesRead);
    EXPECT_EQ(0, xme_hal_mem_compare(topicRecv, topicSend, strlen(topicSend)));

    xme_hal_mem_free(sendBuffer);
    xme_hal_mem_free(recvBuffer);

    xme_hal_mem_free(topicSend);
    xme_hal_mem_free(topicRecv);
}

TEST_F(UDPWaypointInterfaceTest, sendAndReadBroadcast)
{
    uint32_t bytesRead, stringLen;
    uint32_t loopCount = 10;
    uint8_t key1[] = { 1, 2, 3, 4 };

    stringLen = strlen(STRING) + 1;
    topicRecv = (char*) xme_hal_mem_alloc(stringLen);
    topicSend = (char*) xme_hal_mem_alloc(stringLen);
    xme_hal_safeString_strncpy(topicSend, STRING, stringLen);

    recvBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpReceive_getPackageOverHead());
    sendBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpSend_getPackageOverHead());

    udpRecvTestBroadcast(33221u, strlen(topicSend), key1);
    udpSendTestBroadcast("192.168.17.255", 33221u, strlen(topicSend), key1);

    xme_core_dataHandler_writeData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicSend, strlen(topicSend));
    xme_wp_udp_udpSend_run(udpSendInstanceId2);
    while (loopCount-- != 0)
    {
        XME_LOG(XME_LOG_ALWAYS,"sleeping in sendAndReadBroadcast with loop count %d\n", loopCount);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
        xme_wp_udp_udpReceive_run(udpReceiveInstanceId2);
    }
    xme_core_dataHandler_readData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicRecv, stringLen, &bytesRead);
    EXPECT_EQ(0, xme_hal_mem_compare(topicRecv, topicSend, strlen(topicSend)));

    xme_hal_mem_free(sendBuffer);
    xme_hal_mem_free(recvBuffer);

    xme_hal_mem_free(topicSend);
    xme_hal_mem_free(topicRecv);
}

TEST_F(UDPWaypointInterfaceTest, sendAndReadAndBroadcastTogether)
{
    uint32_t bytesRead, stringLen;
    uint32_t loopCount = 10;
    uint8_t key1[] = { 1, 2, 3, 4 };
    uint8_t key2[] = { 1, 2, 3, 5 };

    ASSERT_GT(strlen(STRING), strlen(STRING2));

    // We use the same topicSend and topicRecv buffer because strlen(STRING) > strlen(STRING2)
    stringLen = strlen(STRING) + 1;
    topicRecv = (char*) xme_hal_mem_alloc(stringLen);
    topicSend = (char*) xme_hal_mem_alloc(stringLen);
    xme_hal_safeString_strncpy(topicSend, STRING, stringLen);

    recvBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpReceive_getPackageOverHead());
    sendBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpSend_getPackageOverHead());

    udpRecvTest(33221u, strlen(topicSend), key1);
    udpSendTest("127.0.0.1", 33221u, (uint16_t) strlen(topicSend), key1);

    xme_core_dataHandler_writeData((xme_core_dataManager_dataPacketId_t)dataPortSend, topicSend, strlen(topicSend));
    xme_wp_udp_udpSend_run(udpSendInstanceId1);
    while (loopCount-- != 0)
    {
        XME_LOG(XME_LOG_ALWAYS,"1. sleeping in sendAndReadBroadcastTogether with loop count %d\n", loopCount);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
        xme_wp_udp_udpReceive_run(udpReceiveInstanceId1);
    }
    xme_core_dataHandler_readData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicRecv, stringLen, &bytesRead);
    EXPECT_EQ(0, xme_hal_mem_compare(topicRecv, topicSend, strlen(topicSend)));

    stringLen = strlen(STRING2) + 1;
    xme_hal_safeString_strncpy(topicSend, STRING2, stringLen);
    udpRecvTestBroadcast(33222u, strlen(topicSend), key2);
    udpSendTestBroadcast("192.168.17.255", 33222u, strlen(topicSend), key2);

    xme_core_dataHandler_writeData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicSend, strlen(topicSend));
    xme_wp_udp_udpSend_run(udpSendInstanceId2);
    loopCount = 10;
    while (loopCount-- != 0)
    {
        XME_LOG(XME_LOG_ALWAYS,"2. sleeping in sendAndReadBroadcastTogether with loop count %d\n", loopCount);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
        xme_wp_udp_udpReceive_run(udpReceiveInstanceId2);
    }
    xme_core_dataHandler_readData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicRecv, strlen(STRING2) + 1, &bytesRead);
    EXPECT_EQ(0, xme_hal_mem_compare(topicRecv, topicSend, strlen(topicSend)));

    xme_hal_mem_free(sendBuffer);
    xme_hal_mem_free(recvBuffer);

    xme_hal_mem_free(topicSend);
    xme_hal_mem_free(topicRecv);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
