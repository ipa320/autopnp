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
 * $Id: interfaceTestUDP.cpp 6607 2014-02-03 17:37:50Z gulati $
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
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \brief Topic used in sendAndReadWithAttributes test.
 *        Associated to some attributes, see attributeMock.c.
 */
typedef struct
{
    uint32_t value;
} topic_t;

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
        xme_status_t status;

        dataPortReceive = (xme_core_dataManager_dataPacketId_t) 1;

        ASSERT_TRUE(NULL != recvBuffer);

        status = xme_wp_udp_udpReceive_addConfig
        (
            &udpReceiveInstanceId1,
            dataPortReceive,
            (xme_core_topic_t) 1,
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
        xme_status_t status;

        dataPortSend = (xme_core_dataManager_dataPacketId_t) 2;

        ASSERT_TRUE(NULL != sendBuffer);

        status = xme_wp_udp_udpSend_addConfig
        (
            &udpSendInstanceId1,
            dataPortSend,
            (xme_core_topic_t) 1,
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
        xme_status_t status;

        dataPortReceive = (xme_core_dataManager_dataPacketId_t) 3;

        ASSERT_TRUE(NULL != recvBuffer);

        status = xme_wp_udp_udpReceive_addConfig
        (
            &udpReceiveInstanceId2,
            dataPortReceive,
            (xme_core_topic_t) 1,
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
        xme_status_t status;

        dataPortSend = (xme_core_dataManager_dataPacketId_t) 4;

        ASSERT_TRUE(NULL != sendBuffer);

        status = xme_wp_udp_udpSend_addConfig
        (
            &udpSendInstanceId2,
            dataPortSend,
            (xme_core_topic_t) 1,
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
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     UDPWaypointInterfaceTest                                               //
//----------------------------------------------------------------------------//

TEST_F(UDPWaypointInterfaceTest, sendAndRead)
{
    xme_status_t status;
    uint8_t successCount = 0U;
    uint32_t bytesRead, stringLen;
    uint8_t loopCount = 10U;
    uint8_t key1[] = { 1, 2, 3, 4 };

    stringLen = strlen(STRING) + 1;
    topicRecv = (char*) xme_hal_mem_alloc(stringLen);
    topicSend = (char*) xme_hal_mem_alloc(stringLen);
    xme_hal_safeString_strncpy(topicSend, STRING, stringLen);

    recvBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpReceive_getPackageOverHead());
    sendBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpSend_getPackageOverHead());

    udpRecvTest(33221u, strlen(topicSend), key1);
    udpSendTest("127.0.0.1", 33221u, (uint16_t) strlen(topicSend), key1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicSend, strlen(topicSend)));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_run(udpSendInstanceId1));
    while (loopCount-- > 0U)
    {
        XME_LOG(XME_LOG_ALWAYS,"Waiting for data in sendAndRead (loop count %u)\n", loopCount);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
        status = xme_wp_udp_udpReceive_run(udpReceiveInstanceId1); // returns XME_STATUS_INTERNAL_ERROR if no data available
        if (XME_STATUS_SUCCESS == status) successCount++;
        EXPECT_TRUE(XME_STATUS_SUCCESS == status || XME_STATUS_INTERNAL_ERROR == status);
    }
    ASSERT_EQ(1U, successCount);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicRecv, stringLen, &bytesRead));
    EXPECT_EQ(0, xme_hal_mem_compare(topicRecv, topicSend, strlen(topicSend)));

    xme_hal_mem_free(sendBuffer);
    xme_hal_mem_free(recvBuffer);

    xme_hal_mem_free(topicSend);
    xme_hal_mem_free(topicRecv);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_removeConfig(udpSendInstanceId1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpReceive_removeConfig(udpReceiveInstanceId1));
}

TEST_F(UDPWaypointInterfaceTest, DISABLED_sendAndReadBroadcast)
{
    xme_status_t status;
    uint8_t successCount = 0U;
    uint32_t bytesRead, stringLen;
    uint8_t loopCount = 10U;
    uint8_t key1[] = { 1, 2, 3, 4 };

    stringLen = strlen(STRING) + 1;
    topicRecv = (char*) xme_hal_mem_alloc(stringLen);
    topicSend = (char*) xme_hal_mem_alloc(stringLen);
    xme_hal_safeString_strncpy(topicSend, STRING, stringLen);

    recvBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpReceive_getPackageOverHead());
    sendBuffer = (char*) xme_hal_mem_alloc(strlen(topicSend) + xme_wp_udp_udpSend_getPackageOverHead());

    udpRecvTestBroadcast(33221u, strlen(topicSend), key1);
    udpSendTestBroadcast("255.255.255.255", 33221u, strlen(topicSend), key1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicSend, strlen(topicSend)));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_run(udpSendInstanceId2));
    while (loopCount-- > 0U)
    {
        XME_LOG(XME_LOG_ALWAYS,"Waiting for data in sendAndReadBroadcast (loop count %u)\n", loopCount);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
        status = xme_wp_udp_udpReceive_run(udpReceiveInstanceId2); // returns XME_STATUS_INTERNAL_ERROR if no data available
        if (XME_STATUS_SUCCESS == status) successCount++;
        EXPECT_TRUE(XME_STATUS_SUCCESS == status || XME_STATUS_INTERNAL_ERROR == status);
    }
    ASSERT_EQ(1U, successCount);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicRecv, stringLen, &bytesRead));
    EXPECT_EQ(0, xme_hal_mem_compare(topicRecv, topicSend, strlen(topicSend)));

    xme_hal_mem_free(sendBuffer);
    xme_hal_mem_free(recvBuffer);

    xme_hal_mem_free(topicSend);
    xme_hal_mem_free(topicRecv);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_removeConfig(udpSendInstanceId2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpReceive_removeConfig(udpReceiveInstanceId2));
}

TEST_F(UDPWaypointInterfaceTest, DISABLED_sendAndReadAndBroadcastTogether)
{
    xme_status_t status;
    uint8_t successCount = 0U;
    uint32_t bytesRead, stringLen;
    uint8_t loopCount = 10U;
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

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData((xme_core_dataManager_dataPacketId_t)dataPortSend, topicSend, strlen(topicSend)));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_run(udpSendInstanceId1));
    while (loopCount-- > 0U)
    {
        XME_LOG(XME_LOG_ALWAYS,"Waiting for data in sendAndReadBroadcastTogether (first, loop count %u)\n", loopCount);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
        status = xme_wp_udp_udpReceive_run(udpReceiveInstanceId1); // returns XME_STATUS_INTERNAL_ERROR if no data available
        if (XME_STATUS_SUCCESS == status) successCount++;
        EXPECT_TRUE(XME_STATUS_SUCCESS == status || XME_STATUS_INTERNAL_ERROR == status);
    }
    ASSERT_EQ(1U, successCount);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicRecv, stringLen, &bytesRead));
    EXPECT_EQ(0, xme_hal_mem_compare(topicRecv, topicSend, strlen(topicSend)));

    stringLen = strlen(STRING2) + 1;
    xme_hal_safeString_strncpy(topicSend, STRING2, stringLen);
    udpRecvTestBroadcast(33222u, strlen(topicSend), key2);
    udpSendTestBroadcast("255.255.255.255", 33222u, strlen(topicSend), key2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicSend, strlen(topicSend)));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_run(udpSendInstanceId2));
    loopCount = 10U;
    successCount = 0U;
    while (loopCount-- > 0U)
    {
        XME_LOG(XME_LOG_ALWAYS,"Waiting for data in sendAndReadBroadcastTogether (second, loop count %u)\n", loopCount);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
        status = xme_wp_udp_udpReceive_run(udpReceiveInstanceId2); // returns XME_STATUS_INTERNAL_ERROR if no data available
        if (XME_STATUS_SUCCESS == status) successCount++;
        EXPECT_TRUE(XME_STATUS_SUCCESS == status || XME_STATUS_INTERNAL_ERROR == status);
    }
    ASSERT_EQ(1U, successCount);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData((xme_core_dataManager_dataPacketId_t) dataPortSend, topicRecv, strlen(STRING2) + 1, &bytesRead));
    EXPECT_EQ(0, xme_hal_mem_compare(topicRecv, topicSend, strlen(topicSend)));

    xme_hal_mem_free(sendBuffer);
    xme_hal_mem_free(recvBuffer);

    xme_hal_mem_free(topicSend);
    xme_hal_mem_free(topicRecv);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_removeConfig(udpSendInstanceId1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpReceive_removeConfig(udpReceiveInstanceId1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_removeConfig(udpSendInstanceId2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpReceive_removeConfig(udpReceiveInstanceId2));
}

TEST_F(UDPWaypointInterfaceTest, sendAndReadWithAttributes)
{
    xme_status_t status;
    uint8_t successCount = 0U;
    uint32_t bytesRead;
    uint8_t loopCount = 10U;
    uint16_t bufferSize;
    uint16_t dataSize;
    uint8_t key[] = {0, 0, 0, 2};
    topic_t topicData;
    topic_t topicDataReceived;
    topic_t topicDataExpected;
    uint32_t attr1;
    uint32_t attr1Received;
    uint32_t attr1Expected;
    uint64_t attr2;
    uint64_t attr2Received;
    uint64_t attr2Expected;

    // Init topics and attributes
    topicData.value = 42;
    topicDataExpected = topicData;

    attr1 = 123;
    attr1Expected = attr1;
    attr2 = 5123456789;
    attr2Expected = attr2;

    // Create upd send and receive configurations
    dataSize = sizeof(topic_t);
    bufferSize = dataSize + sizeof(uint32_t) + sizeof(uint64_t) + xme_wp_udp_udpReceive_getPackageOverHead();

    recvBuffer = (char*) xme_hal_mem_alloc(bufferSize);
    sendBuffer = (char*) xme_hal_mem_alloc(bufferSize);

    status = xme_wp_udp_udpSend_addConfig
    (
        &udpSendInstanceId1,
        (xme_core_dataManager_dataPacketId_t)3,
        (xme_core_topic_t)2,
        dataSize,
        sendBuffer,
        bufferSize,
        key,
        "127.0.0.1",
        33221u,
        false
    );
    ASSERT_EQ(XME_STATUS_SUCCESS, status);

    status = xme_wp_udp_udpReceive_addConfig
    (
        &udpReceiveInstanceId1,
        (xme_core_dataManager_dataPacketId_t)4,
        (xme_core_topic_t)2,
        dataSize,
        recvBuffer,
        bufferSize,
        key,
        33221u
    );
    ASSERT_EQ(XME_STATUS_SUCCESS, status);

    // Write data to mock dataHandler

    status = xme_core_dataHandler_writeData
    (
        (xme_core_dataManager_dataPacketId_t)3,
        &topicData,
        sizeof(topic_t)
    );
    ASSERT_EQ(XME_STATUS_SUCCESS, status);

    status = xme_core_dataHandler_writeAttribute
    (
        (xme_core_dataManager_dataPacketId_t)3,
        (xme_core_attribute_key_t)1,
        &attr1,
        sizeof(attr2)
    );
    ASSERT_EQ(XME_STATUS_SUCCESS, status);

    status = xme_core_dataHandler_writeAttribute
    (
        (xme_core_dataManager_dataPacketId_t)3,
        (xme_core_attribute_key_t)2,
        &attr2,
        sizeof(attr2)
    );
    ASSERT_EQ(XME_STATUS_SUCCESS, status);

    // Run udp send
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_run(udpSendInstanceId1));
    while (loopCount-- > 0U)
    {
        XME_LOG(XME_LOG_ALWAYS, "Waiting for data in sendAndReadWithAttributes (loop count %u)\n", loopCount);
        xme_hal_sleep_sleep(xme_hal_time_timeIntervalFromMilliseconds(100));
        status = xme_wp_udp_udpReceive_run(udpReceiveInstanceId1); // returns XME_STATUS_INTERNAL_ERROR if no data available
        if (XME_STATUS_SUCCESS == status) successCount++;
        EXPECT_TRUE(XME_STATUS_SUCCESS == status || XME_STATUS_INTERNAL_ERROR == status);
    }
    ASSERT_EQ(1U, successCount);

    // Read values of udp receive via data handler mock
    status = xme_core_dataHandler_readData
    (
        (xme_core_dataManager_dataPacketId_t)4,
        &topicDataReceived,
        sizeof(topicDataReceived),
        &bytesRead
    );
    EXPECT_EQ(XME_STATUS_SUCCESS, status);

    status = xme_core_dataHandler_readAttribute
    (
        (xme_core_dataManager_dataPacketId_t)4,
        (xme_core_attribute_key_t)1,
        &attr1Received,
        sizeof(attr1),
        &bytesRead
    );
    EXPECT_EQ(XME_STATUS_SUCCESS, status);

    status = xme_core_dataHandler_readAttribute
    (
        (xme_core_dataManager_dataPacketId_t)4,
        (xme_core_attribute_key_t)2,
        &attr2Received,
        sizeof(attr2),
        &bytesRead
    );
    EXPECT_EQ(XME_STATUS_SUCCESS, status);

    // Check if expected and received values match
    EXPECT_EQ(0, xme_hal_mem_compare(&topicDataExpected, &topicDataReceived, sizeof(topicData)));
    EXPECT_EQ(0, xme_hal_mem_compare(&attr1Expected, &attr1Received, sizeof(attr1)));
    EXPECT_EQ(0, xme_hal_mem_compare(&attr2Expected, &attr2Received, sizeof(attr2)));

    xme_hal_mem_free(sendBuffer);
    xme_hal_mem_free(recvBuffer);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpSend_removeConfig(udpSendInstanceId1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_udp_udpReceive_removeConfig(udpReceiveInstanceId1));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
