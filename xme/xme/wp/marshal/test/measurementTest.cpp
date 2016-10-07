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
#include "xme/wp/marshal/test/marshaler.h"
#include "xme/wp/marshal/include/marshaler.h"
#include "xme/wp/marshal/test/demarshaler.h"
#include "xme/wp/marshal/include/demarshaler.h"
#include "xme/wp/marshal/test/deMarshalerTestTopic.h"
#include "xme/wp/marshal/test/deMarshalerTestTopicData.h"

#include "xme/hal/include/time.h"
#include "xme/hal/include/mem.h"

#include <inttypes.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define CONFIG_COUNT 1000 ///< How many configurations will be added to the waypoints.
#define WCET_IN_NS_MARSHALER 100000000 ///< Error triggered, when maximum measured wcet of xme_wp_udp_marshaler_run exceeds this value.
#define WCET_IN_NS_DEMARSHALER 100000000 ///< Error triggered, when maximum measured wcet of xme_wp_udp_demarshaler_run exceeds this value.

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
class MeasurementTest : public ::testing::Test
{
protected:

    xme_wp_waypoint_instanceId_t marshalerInstanceIds[CONFIG_COUNT];
    xme_wp_waypoint_instanceId_t demarshalerInstanceIds[CONFIG_COUNT];

    MeasurementTest()
    {
        //xme_core_dataHandler_init(18);
        xme_core_dataHandler_init();
        xme_core_broker_init(NULL);
        xme_wp_marshal_marshaler_init();
        xme_wp_marshal_demarshaler_init();

        addMarshalerConfigs();
    }

    virtual ~MeasurementTest()
    {
        xme_wp_marshal_demarshaler_fini();
        xme_wp_marshal_marshaler_fini();
        xme_core_broker_fini();
        xme_core_dataHandler_fini();
    }

    /**
     * \brief Add CONFIG_COUNT configurations to marshaler waypoint.
     */
    void addMarshalerConfigs()
    {
        xme_core_dataManager_dataPacketId_t subDataPacketId;
        xme_core_dataManager_dataPacketId_t pubDataPacketId;
        uint32_t i;
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

        status = xme_core_dataHandler_createDataPacket(sizeof(xme_wp_deMarshalerTest_topic_test_t), &subDataPacketId);

#if 0
        // Create subscription, which will be used in all configurations
        xme_core_dataHandler_createPort
        (
            (xme_core_component_t)1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            (xme_core_topic_t) XME_WP_DEMARSHALERTEST_TOPIC_TEST,
            sizeof(xme_wp_deMarshalerTest_topic_test_t),
            metadata,
            1,
            false,
            false,
            0,
            &subDataPacketId
        );
#endif // #if 0
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        status = xme_core_dataHandler_createDataPacket(sizeof(xme_wp_deMarshalerTest_topic_test_t), &pubDataPacketId);

#if 0
        // Create publication, which will be used in all configurations
        xme_core_dataHandler_createPort
        (
            (xme_core_component_t)1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            (xme_core_topic_t) XME_WP_DEMARSHALERTEST_TOPIC_TEST,
            sizeof(xme_wp_deMarshalerTest_topic_test_t),
            metadata,
            1,
            false,
            false,
            0,
            &pubDataPacketId
        );
#endif // #if 0
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        xme_core_dataHandler_configure();
        // Add configurations
        // Every configuration will use the same port and topic (this should not influence execution time)
        for (i = 0; i < CONFIG_COUNT; i++)
        {
            xme_wp_marshal_marshaler_addConfig
            (
                &marshalerInstanceIds[i],
                (xme_core_topic_t) 5000,
                subDataPacketId,
                pubDataPacketId
            );
        }
    }

    /**
     * \brief Add CONFIG_COUNT configurations to demarshaler waypoint.
     */
    void addDemarshalerConfigs()
    {
        xme_core_dataManager_dataPacketId_t subDataPacketId;
        xme_core_dataManager_dataPacketId_t pubDataPacketId;
        uint32_t i;
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

        status = xme_core_dataHandler_createDataPacket(sizeof(xme_wp_deMarshalerTest_topic_test_t), &subDataPacketId);

#if 0
        // Create subscription, which will be used in all configurations
        xme_core_dataHandler_createPort
        (
            (xme_core_component_t)1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            (xme_core_topic_t) XME_WP_DEMARSHALERTEST_TOPIC_TEST,
            sizeof(xme_wp_deMarshalerTest_topic_test_t),
            metadata,
            1,
            false,
            false,
            0,
            &subDataPacketId
        );
#endif // #if 0
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        status = xme_core_dataHandler_createDataPacket(sizeof(xme_wp_deMarshalerTest_topic_test_t), &pubDataPacketId);

#if 0
        // Create publication, which will be used in all configurations
        xme_core_dataHandler_createPort
        (
            (xme_core_component_t)1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            (xme_core_topic_t) XME_WP_DEMARSHALERTEST_TOPIC_TEST,
            sizeof(xme_wp_deMarshalerTest_topic_test_t),
            metadata,
            1,
            false,
            false,
            0,
            &pubDataPacketId
        );
#endif // #if 0
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        xme_core_dataHandler_configure();

        // Add configurations
        // Every configuration will use the same port and topic (this should not influence execution time)
        for (i = 0; i < CONFIG_COUNT; i++)
        {
            xme_wp_marshal_demarshaler_addConfig
            (
                &demarshalerInstanceIds[i],
                (xme_core_topic_t) 5000,
                subDataPacketId,
                pubDataPacketId
            );
        }
    }

};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
TEST_F(MeasurementTest, marshalerRunMeasurement)
{
    EXPECT_FALSE
    (
        xme_wp_testUtil_measureExecutionTime
        (
            10000,
            &xme_wp_marshal_marshaler_run,
            NULL,
            NULL,
            "xme_wp_marshal_marshaler_run",
            marshalerInstanceIds,
            (uint16_t)CONFIG_COUNT,
            (xme_hal_time_timeInterval_t)WCET_IN_NS_MARSHALER
        )
    ) << "Expected WCET of xme_wp_marshal_marshaler_run() exceeded!";
}

TEST_F(MeasurementTest, demarshalerRunMeasurement)
{
    EXPECT_FALSE
    (
        xme_wp_testUtil_measureExecutionTime
        (
            10000,
            &xme_wp_marshal_demarshaler_run,
            NULL,
            NULL,
            "xme_wp_marshal_demarshaler_run",
            demarshalerInstanceIds,
            (uint16_t)CONFIG_COUNT,
            (xme_hal_time_timeInterval_t)WCET_IN_NS_DEMARSHALER
        )
    ) << "Expected WCET of xme_wp_marshal_demarshaler_run() exceeded!";
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
