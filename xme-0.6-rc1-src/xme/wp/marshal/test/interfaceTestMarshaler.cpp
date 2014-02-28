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
 * $Id: interfaceTestMarshaler.cpp 5117 2013-09-19 10:13:40Z wiesmueller $
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
#include "xme/hal/include/net.h"

#include "xme/wp/waypoint.h"

class MarshalerInterfaceTest : public testing::Test
{
protected:
    MarshalerInterfaceTest()
    {
    }

    ~MarshalerInterfaceTest()
    {
        // Do nothing
    }

    virtual void SetUp()
    {
        xme_wp_marshal_marshaler_init();
    }

    virtual void TearDown()
    {
        xme_wp_marshal_marshaler_fini();
    }
};

TEST_F(MarshalerInterfaceTest, MarshalerCreateValidConfig)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId2));
}

TEST_F(MarshalerInterfaceTest, MarshalerAddConfigWithIndalidTopic)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC_INVALID_TOPIC, dataPacketId1, dataPacketId2));
}

TEST_F(MarshalerInterfaceTest, MarshalerAddConfigWithInvalidDataPacketId)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dataPacketId2));
}

TEST_F(MarshalerInterfaceTest, MarshalerAddConfigWithInvalidDataPacketIdAsSecondParameter)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(MarshalerInterfaceTest, MarshalerAddConfigWithMaximumDataPacketId)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), XME_CORE_DATAMANAGER_DATAPACKETID_MAX, dataPacketId2));
}

TEST_F(MarshalerInterfaceTest, MarshalerAddConfigWithMaximumDataPacketIdAsSecondParameter)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_MAX));
}

TEST_F(MarshalerInterfaceTest, MarshalerAddConfigWithSameInputAndOutputDataPacketId)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId1));
}

TEST_F(MarshalerInterfaceTest, MarshalerAddConfigWithValidParameters)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId2));
}

TEST_F(MarshalerInterfaceTest, MarshalerAddConfigWithDifferentTopics)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    // Unsupported topics
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_LOCAL_ANNOUNCEMENT), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_REMOTE_ANNOUNCEMENT), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_REMOTE_MODIFY_ROUTING_TABLE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_LOCAL_NEIGHBORHOOD_UPDATE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_REMOTE_NEIGHBORHOOD_UPDATE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_REQUEST), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_RESPONSE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_NEW_NODE_REQUEST), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_NEW_NODE_RESPONSE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_MANAGEMENT_CHANNELS_TO_EDGE_NODE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_MANAGEMENT_CHANNELS_TO_NEW_NODE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOG_MESSAGE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_EXEC_CYCLE_COUNTER), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LIFESIGN_HEARTBEAT), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LIFESIGN_ERROR_MESSAGE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_COMMON), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_EVENT), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_FLAG), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_INTEGER), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_UNSIGNED_INTEGER), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_DECIMAL), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_STRING), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_USER), dataPacketId1, dataPacketId2));
    // Supported topics
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL), dataPacketId1, dataPacketId2));
}

TEST_F(MarshalerInterfaceTest, MarshalerRunWithNonAddedConfiguration)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_wp_marshal_marshaler_run(XME_WP_WAYPOINT_INSTANCEID_INVALID));
}

TEST_F(MarshalerInterfaceTest, MarshalerRunWithInvalidTopicConfiguration)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_INVALID_TOPIC), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_wp_marshal_marshaler_run(instanceId));
}

TEST_F(MarshalerInterfaceTest, MarshalerRunWithDifferentCombinationsInDataPacketsConfiguration)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    // TODO Check the following examples, because the behaviour is erratic with different data. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dataPacketId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_marshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId1));
}

TEST_F(MarshalerInterfaceTest, getConfig)
{
    // Without adding configurations, getConfig should always return XME_STATUS_NOT_FOUND

    xme_wp_waypoint_instanceId_t instanceId = (xme_wp_waypoint_instanceId_t)1;
    xme_core_topic_t topic = XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST);
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_wp_marshal_marshaler_getConfig(&instanceId, &topic, &dataPacketId1, &dataPacketId2));
}

int main(int argc, char **argv)
{
    int retval;
    
    ::testing::InitGoogleTest(&argc, argv);
    retval = RUN_ALL_TESTS();
    
    return retval;
}
