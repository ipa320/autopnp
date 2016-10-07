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
 * $Id: measurementTest.cpp 6684 2014-02-07 14:34:26Z geisinger $
 */

/**
 * \file
 *         UDP Waypoint interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/log.h"

#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"
#include "xme/core/broker/include/broker.h"

#include "xme/wp/test/testUtil.h"
#include "xme/wp/udp/include/udpSend.h"
#include "xme/wp/udp/include/udpReceive.h"

#include "xme/hal/include/time.h"
#include "xme/hal/include/mem.h"

#include <inttypes.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define CONFIG_COUNT 100 ///< How many configurations will be added to the waypoints.
#define WCET_IN_NS_UDP_SEND    100000000 ///< Error triggered, when maximum measured wcet of xme_wp_udp_udpSend_run exceeds this value.
#define WCET_IN_NS_UDP_RECEIVE 100000000 ///< Error triggered, when maximum measured wcet of xme_wp_udp_udpReceive_run exceeds this value.

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \brief Used as data type for test topic.
 */
typedef struct
{
    uint16_t x;
} testStrcut_t;

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
class MeasurementTest : public ::testing::Test
{
protected:

    xme_wp_waypoint_instanceId_t udpSendInstanceIds[CONFIG_COUNT];
    xme_wp_waypoint_instanceId_t udpReceiveInstanceIds[CONFIG_COUNT];

    MeasurementTest()
    {
        xme_hal_sync_init();
        xme_hal_net_init();
        //xme_core_dataHandler_init(18);
        xme_core_dataHandler_init();
        xme_core_broker_init(NULL);
        xme_wp_udp_udpReceive_init();
        xme_wp_udp_udpSend_init();

        addUdpSendConfigs();
        addUdpReceiveConfigs();
    }

    virtual ~MeasurementTest()
    {
        xme_wp_udp_udpSend_fini();
        xme_wp_udp_udpReceive_fini();
        xme_core_broker_fini();
        xme_core_dataHandler_fini();
        xme_hal_net_fini();
        xme_hal_sync_fini();
    }

    /**
     * \brief Add CONFIG_COUNT configurations to udp send waypoint.
     */
    void addUdpSendConfigs()
    {

        xme_core_dataManager_dataPacketId_t dataPacketId;
        void* buffer;
        uint32_t i;
        uint8_t key[] = {0, 0, 0, 1};
#if 0
        xme_core_attribute_descriptor_list_t metadata;
        xme_core_attribute_descriptor_t metadata_elements[1];
#endif // #if 0
        xme_status_t status;

#if 0
        metadata_elements[0].key = (xme_core_attribute_key_t) 0;
        metadata_elements[0].size = (size_t) 0;
        metadata.length = 1;
        metadata.element = metadata_elements;
#endif // #if 0

        buffer = xme_hal_mem_alloc(sizeof(testStrcut_t) + xme_wp_udp_udpSend_getPackageOverHead());

        status = xme_core_dataHandler_createDataPacket(sizeof(testStrcut_t), &dataPacketId);

#if 0
        // Create subscription, which will be used in all configurations
        xme_core_dataHandler_createPort
        (
             (xme_core_component_t)1,
             XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
             (xme_core_topic_t) 5000,
             sizeof(testStrcut_t),
             metadata,
             1,
             false,
             false,
             0,
             &dataPacketId
        );
#endif // #if 0
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        xme_core_dataHandler_configure();

        // Add configurations
        // Every configuration will use the same port, buffer and topic (this should not influence execution time)
        for (i = 0; i < CONFIG_COUNT; i++)
        {
            xme_wp_udp_udpSend_addConfig
            (
                &udpSendInstanceIds[i],
                dataPacketId,
                (xme_core_topic_t)XME_CORE_TOPIC_USER,
                sizeof(testStrcut_t),
                buffer,
                sizeof(testStrcut_t) + xme_wp_udp_udpSend_getPackageOverHead(),
                key,
                "127.0.0.1",
                12345,
                false
            );
        }
    }

    /**
     * \brief Add CONFIG_COUNT configurations to udp send waypoint.
     */
    void addUdpReceiveConfigs()
    {
        xme_core_dataManager_dataPacketId_t dataPacketId;
        void* buffer;
        uint32_t i;
        uint8_t key[] = {0, 0, 0, 1};
#if 0
        xme_core_attribute_descriptor_list_t metadata;
        xme_core_attribute_descriptor_t metadata_elements[1];
#endif // #if 0
        xme_status_t status;

#if 0
        metadata_elements[0].key = (xme_core_attribute_key_t) 0;
        metadata_elements[0].size = (size_t) 0;
        metadata.length = 1;
        metadata.element = metadata_elements;
#endif // #if 0

        buffer = xme_hal_mem_alloc(sizeof(testStrcut_t) + xme_wp_udp_udpReceive_getPackageOverHead());

        status = xme_core_dataHandler_createDataPacket(sizeof(testStrcut_t), &dataPacketId);

#if 0
        // Create subscription, which will be used in all configurations
        xme_core_dataHandler_createPort
        (
             (xme_core_component_t)1,
             XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
             (xme_core_topic_t) 5000,
             sizeof(testStrcut_t),
             metadata,
             1,
             false,
             false,
             0,
             &dataPacketId
        );
#endif // #if 0
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        xme_core_dataHandler_configure();

        // Add configurations
        // Every configuration will use the same port, buffer and topic (this should not influence execution time)
        for (i = 0; i < CONFIG_COUNT; i++)
        {
            xme_wp_udp_udpReceive_addConfig
            (
                &udpReceiveInstanceIds[i],
                (xme_core_topic_t)XME_CORE_TOPIC_USER,
                dataPacketId,
                sizeof(testStrcut_t),
                buffer,
                sizeof(testStrcut_t) + xme_wp_udp_udpReceive_getPackageOverHead(),
                key,
                12345
            );
        }
    }

};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
TEST_F(MeasurementTest, udpSendRunMeasurement)
{
    EXPECT_FALSE
    (
        xme_wp_testUtil_measureExecutionTime
        (
            10000,
            &xme_wp_udp_udpSend_run,
            NULL,
            NULL,
            "xme_wp_udp_udpSend_run",
            udpSendInstanceIds,
            (uint16_t)CONFIG_COUNT,
            (xme_hal_time_timeInterval_t)WCET_IN_NS_UDP_SEND
        )
    ) << "Expected WCET of xme_wp_udp_udpSend_run() exceeded!";
}

TEST_F(MeasurementTest, udpReceiveRunMeasurement)
{
    EXPECT_FALSE
    (
        xme_wp_testUtil_measureExecutionTime
        (
            10000,
            &xme_wp_udp_udpReceive_run,
            NULL,
            NULL,
            "xme_wp_udp_udpReceive_run",
            udpReceiveInstanceIds,
            (uint16_t)CONFIG_COUNT,
            (xme_hal_time_timeInterval_t)WCET_IN_NS_UDP_RECEIVE
        )
    ) << "Expected WCET of xme_wp_udp_udpReceive_run() exceeded!";
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
