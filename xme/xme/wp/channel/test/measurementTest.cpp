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
 * $Id: measurementTest.cpp 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         UDP Waypoint interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/defines.h"

#include "xme/core/broker/include/broker.h"
#include "xme/core/broker/include/brokerPnpManagerInterface.h"
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"
#include "xme/core/directory/include/attribute.h"
#include "xme/core/testUtils.h"

#include "xme/wp/test/testUtil.h"
#include "xme/wp/channel/include/channelInjector.h"
#include "xme/wp/channel/include/channelSelector.h"

#include "xme/hal/include/time.h"
#include "xme/hal/include/mem.h"

#include <inttypes.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define CONFIG_COUNT 254 ///< How many configurations will be added to the waypoints.
#define WCET_IN_NS_INJECTOR 100000000 ///< Error triggered, when maximum measured wcet of xme_wp_channel_channelInjector_run exceeds this value.
#define WCET_IN_NS_SELECTOR 100000000 ///< Error triggered, when maximum measured wcet of xme_wp_channel_channelSelector_run exceeds this value.

#define MEASUREMENT_TOPIC 0x00001001 ///< Id of topic used in this test.

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief Data type for topic MEASUREMENT_TOPIC. We will register attrDescList as attributes for this topic.
 */
typedef struct
{
    uint32_t value;
} measuremenTest_topic_t;

/**
 * \brief Attribute descriptors of attrDescList.
 *
 * \note When modifying this, do not forget to adjust buffer sizes in MeasurementTest::addSelectorConfigs()
 *       and MeasurementTest::addInjectorConfigs()
 */
xme_core_attribute_descriptor_t attrDescs[] =
{
    {
        (xme_core_attribute_key_t)XME_CORE_ATTRIBUTE_KEY_CHANNELID,
        sizeof(uint32_t),
    },
    {
        (xme_core_attribute_key_t)(XME_CORE_ATTRIBUTE_KEY_CHANNELID + 1),
        sizeof(uint32_t),
    },
    {
        (xme_core_attribute_key_t)(XME_CORE_ATTRIBUTE_KEY_CHANNELID + 2),
        sizeof(uint64_t),
    },
    {
        (xme_core_attribute_key_t)(XME_CORE_ATTRIBUTE_KEY_CHANNELID + 3),
        sizeof(uint64_t),
    },
    {
        (xme_core_attribute_key_t)(XME_CORE_ATTRIBUTE_KEY_CHANNELID + 4),
        sizeof(uint64_t),
    }
};

/**
 * \brief Attribute descriptor list for test topic MEASUREMENT_TOPIC.
 */
xme_core_attribute_descriptor_list_t attrDescList =
{
    (sizeof(attrDescs) / sizeof(attrDescs[0])),
    attrDescs
};

uint8_t* injectorBuffer; ///< Pointer to buffer for injector waypoint (see MeasurementTest::addInjectorConfigs()).
uint8_t* selectorBuffer; ///< Pointer to buffer for selector waypoint (see MeasurementTest::addSelectorConfigs()).

xme_core_dataManager_dataPacketId_t subDataPacketId; ///< Used in MeasurementTest::addSelectorConfigs() and channelSelectorRunMeasurementBeforeRun()

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
class MeasurementTestCommon: public xme::testing::Test
{
protected:
    MeasurementTestCommon() { }

    virtual ~MeasurementTestCommon() { }

    virtual void AssertionCheckedSetUp()
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_channel_channelInjector_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_channel_channelSelector_init());

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_registerAttributeDescriptorList
        (
            (xme_core_topic_t)MEASUREMENT_TOPIC,
            &attrDescList,
            false
        ));
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_wp_channel_channelSelector_fini();
        xme_wp_channel_channelInjector_fini();
        xme_core_directory_attribute_fini();
        xme_core_broker_fini();
        xme_core_dataHandler_fini();
    }
};

class ChannelInjectorMeasurementTest : public MeasurementTestCommon
{
protected:

    xme_wp_waypoint_instanceId_t injectorInstanceIds[CONFIG_COUNT];

    ChannelInjectorMeasurementTest() { }

    virtual ~ChannelInjectorMeasurementTest() { }

    virtual void AssertionCheckedSetUp()
    {
        MeasurementTestCommon::AssertionCheckedSetUp();

        addInjectorConfigs();
    }

    virtual void AssertionCheckedTearDown()
    {
        MeasurementTestCommon::AssertionCheckedTearDown();

        xme_hal_mem_free(injectorBuffer);
    }

    /**
     * \brief Add CONFIG_COUNT configurations to channel injector waypoint.
     */
    void addInjectorConfigs()
    {
        xme_status_t status;
        xme_core_dataManager_dataPacketId_t pubDataPacketId;
        xme_core_dataManager_dataPacketId_t postSubDataPacketId;
        uint32_t i;
        uint32_t bufferSize;
        measuremenTest_topic_t subDataPacketData = { 12345u };

        bufferSize = sizeof(measuremenTest_topic_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint64_t) + sizeof(uint64_t) + sizeof(uint64_t);

        injectorBuffer = (uint8_t*)xme_hal_mem_alloc(bufferSize);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(measuremenTest_topic_t), &subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+1, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+2, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+3, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+4, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketPersistent(subDataPacketId));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(measuremenTest_topic_t), &pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID, pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+1, pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+2, pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+3, pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+4, pubDataPacketId));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(measuremenTest_topic_t), &postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID, postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+1, postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+2, postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+3, postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+4, postSubDataPacketId));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(subDataPacketId, &subDataPacketData, sizeof(measuremenTest_topic_t)));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(subDataPacketId));

        status = xme_core_broker_addDataPacketTransferEntry
        (
            pubDataPacketId,
            postSubDataPacketId
        );
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        // Add configurations
        for (i = 0; i < CONFIG_COUNT; i++)
        {
            status = xme_wp_channel_channelInjector_addConfig
            (
                &injectorInstanceIds[i],
                subDataPacketId,
                pubDataPacketId,
                (xme_core_topic_t)MEASUREMENT_TOPIC,
                sizeof(measuremenTest_topic_t),
                injectorBuffer,
                bufferSize,
                (xme_core_channelId_t)1
            );
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
        }
    }

};

class ChannelSelectorMeasurementTest : public MeasurementTestCommon
{
protected:

    xme_wp_waypoint_instanceId_t selectorInstanceIds[CONFIG_COUNT];

    ChannelSelectorMeasurementTest() { }

    virtual ~ChannelSelectorMeasurementTest() { }

    virtual void AssertionCheckedSetUp()
    {
        MeasurementTestCommon::AssertionCheckedSetUp();

        addSelectorConfigs();
    }

    virtual void AssertionCheckedTearDown()
    {
        MeasurementTestCommon::AssertionCheckedTearDown();

        xme_hal_mem_free(selectorBuffer);
    }

    /**
     * \brief Add CONFIG_COUNT configurations to channel selector waypoint.
     */
    void addSelectorConfigs()
    {
        xme_status_t status;
        xme_core_dataManager_dataPacketId_t pubDataPacketId;
        xme_core_dataManager_dataPacketId_t postSubDataPacketId;
        uint32_t i;
        uint32_t bufferSize;

        bufferSize = sizeof(measuremenTest_topic_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint64_t) + sizeof(uint64_t) + sizeof(uint64_t);

        selectorBuffer = (uint8_t*)xme_hal_mem_alloc(bufferSize);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(measuremenTest_topic_t), &subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+1, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+2, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+3, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+4, subDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketPersistent(subDataPacketId));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(measuremenTest_topic_t), &pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID, pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+1, pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+2, pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+3, pubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+4, pubDataPacketId));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(measuremenTest_topic_t), &postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID, postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+1, postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+2, postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+3, postSubDataPacketId));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint64_t), XME_CORE_ATTRIBUTE_KEY_CHANNELID+4, postSubDataPacketId));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

        {
            uint32_t srcChannel = 1;
            measuremenTest_topic_t tempTopic;

            tempTopic.value = 12345U;

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(subDataPacketId));

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData
            (
                subDataPacketId,
                (void *) &tempTopic,
                sizeof(measuremenTest_topic_t)
            ));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute
            (
                subDataPacketId,
                (xme_core_attribute_key_t)XME_CORE_ATTRIBUTE_KEY_CHANNELID,
                &srcChannel,
                sizeof(uint32_t)
            ));
            srcChannel++;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute
            (
                subDataPacketId,
                (xme_core_attribute_key_t)XME_CORE_ATTRIBUTE_KEY_CHANNELID+1,
                &srcChannel,
                sizeof(uint32_t)
            ));
            srcChannel++;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute
            (
                subDataPacketId,
                (xme_core_attribute_key_t)XME_CORE_ATTRIBUTE_KEY_CHANNELID+2,
                &srcChannel,
                sizeof(uint64_t)
            ));
            srcChannel++;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute
            (
                subDataPacketId,
                (xme_core_attribute_key_t)XME_CORE_ATTRIBUTE_KEY_CHANNELID+3,
                &srcChannel,
                sizeof(uint64_t)
            ));
            srcChannel++;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute
            (
                subDataPacketId,
                (xme_core_attribute_key_t)XME_CORE_ATTRIBUTE_KEY_CHANNELID+4,
                &srcChannel,
                sizeof(uint64_t)
            ));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(subDataPacketId));
        }

        status = xme_core_broker_addDataPacketTransferEntry
        (
            pubDataPacketId,
            postSubDataPacketId
        );
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        // Add configurations
        for (i = 0; i < CONFIG_COUNT; i++)
        {
            status = xme_wp_channel_channelSelector_addConfig
            (
                &selectorInstanceIds[i],
                subDataPacketId,
                pubDataPacketId,
                (xme_core_topic_t)MEASUREMENT_TOPIC,
                sizeof(measuremenTest_topic_t),
                selectorBuffer,
                bufferSize,
                (xme_core_channelId_t)1,
                (xme_core_channelId_t)(i + 1)
            );
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
        }
    }

};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
TEST_F(ChannelInjectorMeasurementTest, channelInjectorRunMeasurement)
{
    EXPECT_FALSE
    (
        xme_wp_testUtil_measureExecutionTime
        (
            10000,
            &xme_wp_channel_channelInjector_run,
            NULL,
            NULL,
            "xme_wp_channel_channelInjector_run",
            injectorInstanceIds,
            (uint16_t)CONFIG_COUNT,
            (xme_hal_time_timeInterval_t)WCET_IN_NS_INJECTOR
        )
    ) << "Expected WCET of xme_wp_channel_channelInjector_run() exceeded!";
}

TEST_F(ChannelSelectorMeasurementTest, channelSelectorRunMeasurement)
{
    EXPECT_FALSE
    (
        xme_wp_testUtil_measureExecutionTime
        (
            10000,
            &xme_wp_channel_channelSelector_run,
            NULL,
            NULL,
            "xme_wp_channel_channelSelector_run",
            selectorInstanceIds,
            (uint16_t)CONFIG_COUNT,
            (xme_hal_time_timeInterval_t)WCET_IN_NS_SELECTOR
        )
    ) << "Expected WCET of xme_wp_channel_channelSelector_run() exceeded!";
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
