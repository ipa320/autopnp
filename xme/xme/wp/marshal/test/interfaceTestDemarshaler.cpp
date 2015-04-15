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
 * $Id: interfaceTestDemarshaler.cpp 5117 2013-09-19 10:13:40Z wiesmueller $
 */

/**
 * \file
 *         Demarshaler waypoint test.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "demarshaler.h"
#include "deMarshalerTestTopic.h"
#include "deMarshalerTestTopicData.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/net.h"

#include "xme/wp/waypoint.h"

class DemarshalerInterfaceTest : public testing::Test
{
protected:
    DemarshalerInterfaceTest()
    {
    }

    ~DemarshalerInterfaceTest()
    {
        // Do nothing
    }

    virtual void SetUp()
    {
        xme_wp_marshal_demarshaler_init();
    }

    virtual void TearDown()
    {
        xme_wp_marshal_demarshaler_fini();
    }
};

TEST_F(DemarshalerInterfaceTest, DemarshalerCreateValidConfig)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId2));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerAddConfigWithIndalidTopic)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC_INVALID_TOPIC, dataPacketId1, dataPacketId2));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerAddConfigWithInvalidDataPacketId)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dataPacketId2));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerAddConfigWithInvalidDataPacketIdAsSecondParameter)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerAddConfigWithMaximumDataPacketId)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), XME_CORE_DATAMANAGER_DATAPACKETID_MAX, dataPacketId2));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerAddConfigWithMaximumDataPacketIdAsSecondParameter)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_MAX));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerAddConfigWithSameInputAndOutputDataPacketId)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId1));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerAddConfigWithValidParameters)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId2));
}

TEST_F(DemarshalerInterfaceTest, getConfig)
{
    // Without adding configurations, getConfig should always return XME_STATUS_NOT_FOUND

    xme_wp_waypoint_instanceId_t instanceId = (xme_wp_waypoint_instanceId_t)1;
    xme_core_topic_t topic = XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST);
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_wp_marshal_demarshaler_getConfig(&instanceId, &topic, &dataPacketId1, &dataPacketId2));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerAddConfigWithDifferentTopics)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    // Unsupported topics
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_LOCAL_ANNOUNCEMENT), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_REMOTE_ANNOUNCEMENT), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_REMOTE_MODIFY_ROUTING_TABLE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_LOCAL_NEIGHBORHOOD_UPDATE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_ROUTES_REMOTE_NEIGHBORHOOD_UPDATE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_REQUEST), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_RESPONSE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_NEW_NODE_REQUEST), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_NEW_NODE_RESPONSE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_MANAGEMENT_CHANNELS_TO_EDGE_NODE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOGIN_MANAGEMENT_CHANNELS_TO_NEW_NODE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LOG_MESSAGE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_EXEC_CYCLE_COUNTER), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LIFESIGN_HEARTBEAT), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_LIFESIGN_ERROR_MESSAGE), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_COMMON), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_EVENT), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_FLAG), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_INTEGER), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_UNSIGNED_INTEGER), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_DECIMAL), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_GENERAL_STRING), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_USER), dataPacketId1, dataPacketId2));
    // Supported topics
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL), dataPacketId1, dataPacketId2));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerRunWithNonAddedConfiguration)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_wp_marshal_demarshaler_run(XME_WP_WAYPOINT_INSTANCEID_INVALID));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerRunWithInvalidTopicConfiguration)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_CORE_TOPIC_INVALID_TOPIC), dataPacketId1, dataPacketId2));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_wp_marshal_demarshaler_run(instanceId));
}

TEST_F(DemarshalerInterfaceTest, DemarshalerRunWithDifferentCombinationsInDataPacketsConfiguration)
{
    xme_wp_waypoint_instanceId_t instanceId;
    xme_core_dataManager_dataPacketId_t dataPacketId1 = (xme_core_dataManager_dataPacketId_t) 1;
    xme_core_dataManager_dataPacketId_t dataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2;

    // TODO Check the following examples, because the behaviour is erratic with different data. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId2));
    // TODO: Check the following tests. They work for marshaler but not for demarshaler.
    //ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dataPacketId2));
    //ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_run(instanceId));
    //ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    //ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_run(instanceId));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_wp_marshal_demarshaler_addConfig(&instanceId, XME_CORE_TOPIC(XME_WP_DEMARSHALERTEST_TOPIC_TEST), dataPacketId1, dataPacketId1));
}


int main(int argc, char **argv)
{
    int retval;
    
    ::testing::InitGoogleTest(&argc, argv);
    retval = RUN_ALL_TESTS();
    
    return retval;
}
