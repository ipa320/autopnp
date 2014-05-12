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
 * $Id: integrationTestMarshaler.cpp 4779 2013-08-26 13:28:25Z wiesmueller $
 */

/**
 * \file
 *         Marshaler waypoint test.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "marshaler.h"
#include "deMarshalerTestTopic.h"
#include "deMarshalerTestTopicData.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

#include "xme/wp/waypoint.h"

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

extern xme_wp_deMarshalerTest_topic_test_t
xme_wp_dataHandlerMock_demarshaledData;

extern uint8_t
xme_wp_dataHandlerMock_marshaledData[];

extern xme_wp_deMarshalerTest_topic_test_attribute_attribute0_t
xme_wp_dataHandlerMock_demarshaledAttribute_attribute0;

extern uint8_t
xme_wp_dataHandlerMock_marshaledAttribute_attribute0[];

extern xme_wp_deMarshalerTest_topic_test_attribute_attribute1_t
xme_wp_dataHandlerMock_demarshaledAttribute_attribute1;

extern uint8_t
xme_wp_dataHandlerMock_marshaledAttribute_attribute1[];

extern bool 
xme_wp_dataHandlerMock_writePortAfterCompleteWriteOperation;

extern uint32_t 
xme_wp_dataHandlerMock_completeWriteOperationCallCount;

XME_EXTERN_C_END

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

size_t
xme_wp_dataHandlerMock_getDemarshaledDataSize(void);

size_t
xme_wp_dataHandlerMock_getMarshaledDataSize(void);

size_t
xme_wp_dataHandlerMock_getMarshaledAttributeDataSize
(
    xme_core_attribute_key_t key
);

size_t
xme_wp_dataHandlerMock_getDemarshaledAttributeDataSize
(
    xme_core_attribute_key_t key
);

XME_EXTERN_C_END

/**
 * \brief  Utility function for appending data in hex format to a given stringstream.
 * 
 * \param  str Given stringstream that will be appended.
 * \param  size Size of the given data in bytes.
 * \param  ptr Pointer to data that will be formatted in hex format and appended
 *         to stringstream.
 */
static void 
appendHex
(
    std::stringstream* str, 
    size_t const size, 
    void const * const ptr
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
static void
appendHex
(
    std::stringstream* str, 
    size_t const size, 
    void const * const ptr
)
{
    uint8_t* byte;
    uint32_t i;

    byte = (uint8_t*)ptr;

    *str << std::hex << std::setfill('0');

    for (i = 0; i < size; i++)
    {
        *str << std::setw(2) << static_cast<unsigned>(byte[i]);

        if ((i + 1) % 16 == 0)
        {
            *str << "\n";
        }
        else 
        {
            *str << " ";
        }
    }

    *str << "\n";
    *str << std::dec;
}

class MarshalerIntegrationTestGetConfig : public testing::Test
{
protected:
    xme_wp_waypoint_instanceId_t instanceId;
    xme_wp_waypoint_instanceId_t instanceIds[3];
    xme_core_topic_t topic;
    xme_core_dataManager_dataPacketId_t inputPort;
    xme_core_dataManager_dataPacketId_t outputPort;
    xme_status_t status;
    xme_core_dataManager_dataPacketId_t ports[6];

    MarshalerIntegrationTestGetConfig()
    {
        ports[0] = (xme_core_dataManager_dataPacketId_t)1;
        ports[1] = (xme_core_dataManager_dataPacketId_t)2;
        ports[2] = (xme_core_dataManager_dataPacketId_t)3;
        ports[3] = (xme_core_dataManager_dataPacketId_t)4;
        ports[4] = (xme_core_dataManager_dataPacketId_t)5;
        ports[5] = (xme_core_dataManager_dataPacketId_t)6;

        xme_wp_marshal_marshaler_init();

        xme_wp_marshal_marshaler_addConfig
        (
            &instanceIds[0],
            XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC0),
            ports[0],
            ports[1]
        );

        xme_wp_marshal_marshaler_addConfig
        (
            &instanceIds[1],
            XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC1),
            ports[2],
            ports[3]
        );

        xme_wp_marshal_marshaler_addConfig
        (
            &instanceIds[2],
            XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC2),
            ports[4],
            ports[5]
        );
    }

    ~MarshalerIntegrationTestGetConfig()
    {
        xme_wp_marshal_marshaler_fini();
    }
};

class MarshalerIntegrationTest : public testing::Test
{
protected:
    size_t demarshaledDataSize;
    size_t marshaledDataSize;
    size_t demarshaledAttr0Size;
    size_t marshaledAttr0Size;
    size_t demarshaledAttr1Size;
    size_t marshaledAttr1Size;

    MarshalerIntegrationTest()
    {
        demarshaledDataSize = xme_wp_dataHandlerMock_getDemarshaledDataSize();
        marshaledDataSize = xme_wp_dataHandlerMock_getMarshaledDataSize();
        demarshaledAttr0Size = 
            xme_wp_dataHandlerMock_getDemarshaledAttributeDataSize
            (
                XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0)
            );
        marshaledAttr0Size = 
            xme_wp_dataHandlerMock_getMarshaledAttributeDataSize
            (
                XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE0)
            );
        demarshaledAttr1Size = 
            xme_wp_dataHandlerMock_getDemarshaledAttributeDataSize
            (
                XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1)
            );
        marshaledAttr1Size = 
            xme_wp_dataHandlerMock_getMarshaledAttributeDataSize
            (
                XME_CORE_ATTRIBUTES(XME_WP_DEMARSHALERTEST_TOPIC_ATTRIBUTE_KEY_ATTRIBUTE1)
            );
    }

    ~MarshalerIntegrationTest()
    {
        // Do nothing
    }
};

/**
 * \brief  Test that initializes the marshaler, adds a configuration and 
 *         marshals topic data for this configuration and compares the result
 *         to a provided expected value.        
 */
TEST_F(MarshalerIntegrationTest, xme_tests_wp_marshaler)
{
    std::stringstream errorMsg;
    void* expectedResult;
    void* expectedResultAttribute0;
    void* expectedResultAttribute1;
    xme_wp_waypoint_instanceId_t instanceId;

    // Copy marshaled reference data to local variables
    expectedResult = xme_hal_mem_alloc(marshaledDataSize);
    xme_hal_mem_copy
    (
        expectedResult, 
        &xme_wp_dataHandlerMock_marshaledData, 
        marshaledDataSize
    );
    expectedResultAttribute0 = xme_hal_mem_alloc(marshaledAttr0Size);
    xme_hal_mem_copy
    (
        expectedResultAttribute0, 
        &xme_wp_dataHandlerMock_marshaledAttribute_attribute0, 
        marshaledAttr0Size
    );
    expectedResultAttribute1 = xme_hal_mem_alloc(marshaledAttr1Size);
    xme_hal_mem_copy
    (
        expectedResultAttribute1, 
        &xme_wp_dataHandlerMock_marshaledAttribute_attribute1, 
        marshaledAttr1Size
    );

    // Initialize marshaler
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_wp_marshal_marshaler_init()
    );

    // Add configuration for topic XME_WP_DEMARSHALERTEST_TOPIC_TEST
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_wp_marshal_marshaler_addConfig
        (
            &instanceId, 
            XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), 
            (xme_core_dataManager_dataPacketId_t)0, 
            (xme_core_dataManager_dataPacketId_t)0
        )
    );

    // Run marshaler for topic XME_WP_DEMARSHALERTEST_TOPIC_TEST
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_wp_marshal_marshaler_run(instanceId)
    );

    // Prepare error message which prints demarshaled data, expected marshaled data and actual marshaled
    // data in hex format
    errorMsg << "Demarshaled data (size = " << demarshaledDataSize << ")\n";
    appendHex(&errorMsg, demarshaledDataSize, &xme_wp_dataHandlerMock_demarshaledData);
    errorMsg << "Expected marshaled data (size = " << marshaledDataSize << ")\n";
    appendHex(&errorMsg, marshaledDataSize, expectedResult);
    errorMsg << "Actual marshaled data (size = " << marshaledDataSize << ")\n";
    appendHex(&errorMsg, marshaledDataSize, &xme_wp_dataHandlerMock_marshaledData);

    // Compare actual result of marshaled data with expected result
    ASSERT_EQ(0,
        xme_hal_mem_compare
        (
            &xme_wp_dataHandlerMock_marshaledData, 
            expectedResult, 
            marshaledDataSize
        )
    ) << errorMsg.str();

    errorMsg.str(""); // clear stringstream
    errorMsg << "Demarshaled attribute0 (size = " << demarshaledAttr0Size << ")\n";
    appendHex(&errorMsg, demarshaledAttr0Size, &xme_wp_dataHandlerMock_demarshaledAttribute_attribute0);
    errorMsg << "Expected marshaled attribute0 (size = " << marshaledAttr0Size << ")\n";
    appendHex(&errorMsg, marshaledAttr0Size, expectedResultAttribute0);
    errorMsg << "Actual marshaled attribute0 (size = " << marshaledAttr0Size << ")\n";
    appendHex(&errorMsg, marshaledAttr0Size, &xme_wp_dataHandlerMock_marshaledAttribute_attribute0);

    // Compare actual result of marshaled attribute with expected result
    ASSERT_EQ(0,
        xme_hal_mem_compare
        (
            &xme_wp_dataHandlerMock_marshaledAttribute_attribute0, 
            expectedResultAttribute0, 
            marshaledAttr0Size
        )
    ) << errorMsg.str();

    errorMsg.str("");
    errorMsg << "Demarshaled attribute1(size = " << demarshaledAttr1Size << ")\n";
    appendHex(&errorMsg, demarshaledAttr1Size, &xme_wp_dataHandlerMock_demarshaledAttribute_attribute1);
    errorMsg << "Expected marshaled attribute1 (size = " << marshaledAttr1Size << ")\n";
    appendHex(&errorMsg, marshaledAttr1Size, expectedResultAttribute1);
    errorMsg << "Actual marshaled attribute1 (size = " << marshaledAttr1Size << ")\n";
    appendHex(&errorMsg, marshaledAttr1Size, &xme_wp_dataHandlerMock_marshaledAttribute_attribute1);

    // Compare actual result of marshaled attribute with expected result
    ASSERT_EQ(0,
        xme_hal_mem_compare
        (
            &xme_wp_dataHandlerMock_marshaledAttribute_attribute1, 
            expectedResultAttribute1, 
            marshaledAttr1Size
        )
    ) << errorMsg.str();

    // Add configuration for topic XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_wp_marshal_marshaler_addConfig
        (
            &instanceId, 
            XME_CORE_TOPIC(XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL), 
            XME_CORE_DATAMANAGER_DATAPACKETID_MAX, 
            XME_CORE_DATAMANAGER_DATAPACKETID_MAX
        )
    );

    // Run marshaler for topic XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_wp_marshal_marshaler_run(instanceId)
    );

    // Finalization
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_wp_marshal_marshaler_fini()
    );

    ASSERT_TRUE
    (
        !xme_wp_dataHandlerMock_writePortAfterCompleteWriteOperation
    ) << "Marshaler has called writePort after completeWriteOperation. The latter indicates that writing has finished.";

    ASSERT_TRUE
    (
        xme_wp_dataHandlerMock_completeWriteOperationCallCount > 0
    ) << "Marshaler must call completeWriteOperaion at least once.";
}

TEST_F(MarshalerIntegrationTestGetConfig, testGetConfigInvalidValues) {
    instanceId = XME_WP_WAYPOINT_INSTANCEID_INVALID;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    inputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    outputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

    status = xme_wp_marshal_marshaler_getConfig
    (
        &instanceId,
        &topic,
        &inputPort,
        &outputPort
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, status);
}

TEST_F(MarshalerIntegrationTestGetConfig, testGetConfigSearchForNonExistingTopic) {
    instanceId = XME_WP_WAYPOINT_INSTANCEID_INVALID;
    topic = XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST);
    inputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    outputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

    status = xme_wp_marshal_marshaler_getConfig
    (
        &instanceId,
        &topic,
        &inputPort,
        &outputPort
    );

    ASSERT_EQ(XME_STATUS_NOT_FOUND, status);
}

TEST_F(MarshalerIntegrationTestGetConfig, testGetConfigSearchForExistingTopic) {
    instanceId = XME_WP_WAYPOINT_INSTANCEID_INVALID;
    topic = XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC2);
    inputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    outputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

    status = xme_wp_marshal_marshaler_getConfig
    (
        &instanceId,
        &topic,
        &inputPort,
        &outputPort
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, status);
    ASSERT_EQ(instanceIds[2], instanceId);
    ASSERT_EQ(XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC2), topic);
    ASSERT_EQ(ports[4], inputPort);
    ASSERT_EQ(ports[5], outputPort);
}

TEST_F(MarshalerIntegrationTestGetConfig, testGetConfigSearchForExistingInputPort) {
    instanceId = XME_WP_WAYPOINT_INSTANCEID_INVALID;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    inputPort = ports[4];
    outputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

    status = xme_wp_marshal_marshaler_getConfig
    (
        &instanceId,
        &topic,
        &inputPort,
        &outputPort
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, status);
    ASSERT_EQ(instanceIds[2], instanceId);
    ASSERT_EQ(XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC2), topic);
    ASSERT_EQ(ports[4], inputPort);
    ASSERT_EQ(ports[5], outputPort);
}

TEST_F(MarshalerIntegrationTestGetConfig, testGetConfigSearchForExistingOutputPort) {
    instanceId = XME_WP_WAYPOINT_INSTANCEID_INVALID;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    inputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    outputPort = ports[5];

    status = xme_wp_marshal_marshaler_getConfig
    (
        &instanceId,
        &topic,
        &inputPort,
        &outputPort
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, status);
    ASSERT_EQ(instanceIds[2], instanceId);
    ASSERT_EQ(XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC2), topic);
    ASSERT_EQ(ports[4], inputPort);
    ASSERT_EQ(ports[5], outputPort);
}

TEST_F(MarshalerIntegrationTestGetConfig, testGetConfigSearchForExistingInstanceId) {
    instanceId = instanceIds[2];
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    inputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    outputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

    status = xme_wp_marshal_marshaler_getConfig
    (
        &instanceId,
        &topic,
        &inputPort,
        &outputPort
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, status);
    ASSERT_EQ(instanceIds[2], instanceId);
    ASSERT_EQ(XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC2), topic);
    ASSERT_EQ(ports[4], inputPort);
    ASSERT_EQ(ports[5], outputPort);
}

TEST_F(MarshalerIntegrationTestGetConfig, testGetConfigSearchForExistingTopicAndInputPort) {
    instanceId = XME_WP_WAYPOINT_INSTANCEID_INVALID;
    topic = XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC1);
    inputPort = ports[2];
    outputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

    status = xme_wp_marshal_marshaler_getConfig
    (
        &instanceId,
        &topic,
        &inputPort,
        &outputPort
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, status);
    ASSERT_EQ(instanceIds[1], instanceId);
    ASSERT_EQ(XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TOPIC1), topic);
    ASSERT_EQ(ports[2], inputPort);
    ASSERT_EQ(ports[3], outputPort);
}

int main(int argc, char **argv)
{
    int retval;

    ::testing::InitGoogleTest(&argc, argv);
    retval = RUN_ALL_TESTS();
    
    return retval;
}
