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
 * $Id: interfaceTestLogicalRouteManager.cpp 5110 2013-09-18 17:58:16Z geisinger $
 */

/**
 * \file
 *         Logical Route Manager interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/core/component.h"
#include "xme/core/container.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/core/node.h"
#include "xme/core/manifestTypes.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class LogicalRouteManagerInterfaceTest: public ::testing::Test
{
protected:
    LogicalRouteManagerInterfaceTest()
    : nodeInv(XME_CORE_NODE_INVALID_NODE_ID)
    , node1((xme_core_node_nodeId_t)1)
    , node2((xme_core_node_nodeId_t)2)
    , compInv(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT)
    , comp1((xme_core_component_t)1)
    , comp2((xme_core_component_t)2)
    , dpInv(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)
    , dp1((xme_core_dataManager_dataPacketId_t)1)
    , dp2((xme_core_dataManager_dataPacketId_t)2)
    , topicInv(XME_CORE_TOPIC_INVALID_TOPIC)
    , topic1((xme_core_topic_t)1)
    , topic2((xme_core_topic_t)2)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init((xme_hal_graph_graph_t*)&routes));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

        //createComponentManifestForSensor(&sensorManifest);
        //createComponentManifestForMonitor(&monitorManifest);
    }

    virtual ~LogicalRouteManagerInterfaceTest()
    {
        xme_core_pnp_lrm_fini();

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini((xme_hal_graph_graph_t*)&routes));
    }

    xme_core_node_nodeId_t nodeInv;
    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;

    xme_core_component_t compInv;
    xme_core_component_t comp1;
    xme_core_component_t comp2;

    xme_core_dataManager_dataPacketId_t dpInv;
    xme_core_dataManager_dataPacketId_t dp1;
    xme_core_dataManager_dataPacketId_t dp2;

    xme_core_topic_t topicInv;
    xme_core_topic_t topic1;
    xme_core_topic_t topic2;

    xme_core_pnp_lrm_logicalRoutes_t routes;

    //xme_core_directory_componentManifest_t sensorManifest;
    //xme_core_directory_componentManifest_t monitorManifest;
};

class LogicalRouteManagerInterfaceTestForRACEUseCase: public ::testing::Test
{
protected:
    LogicalRouteManagerInterfaceTestForRACEUseCase()
    : DCCNode((xme_core_node_nodeId_t)1)
    , steeringWheelBSensorNode((xme_core_node_nodeId_t)2)
    , steeringWheelRSensorNode((xme_core_node_nodeId_t)3)
    , steeringBoxBSensorActuatorNode((xme_core_node_nodeId_t)4)
    , steeringBoxRSensorActuatorNode((xme_core_node_nodeId_t)5)
    , brakeSensorActuatorNode((xme_core_node_nodeId_t)6)
    , wheelRotSensorInverterActuatorNode((xme_core_node_nodeId_t)7)
    , accelerationPedalSensorNode((xme_core_node_nodeId_t)8)

    , steeringAppComponent((xme_core_component_t)11)
    , brakeAppComponent((xme_core_component_t)12)
    , accelerationAppComponent((xme_core_component_t)13)
    , steeringWheelBSensorComponent((xme_core_component_t)21)
    , steeringWheelRSensorComponent((xme_core_component_t)31)
    , steeringBoxBSensorActuatorComponent((xme_core_component_t)41)
    , steeringBoxRSensorActuatorComponent((xme_core_component_t)51)
    , brakePedalSensorComponent((xme_core_component_t)61)
    , wheelRotFrequencyFLSensorComponent((xme_core_component_t)62)
    , wheelRotFrequencyFRSensorComponent((xme_core_component_t)63)
    , brakeFLActuatorComponent((xme_core_component_t)64)
    , brakeFRActuatorComponent((xme_core_component_t)65)
    , wheelRotFrequencyBLSensorComponent((xme_core_component_t)71)
    , wheelRotFrequencyBRSensorComponent((xme_core_component_t)72)
    , inverterBLActuatorComponent((xme_core_component_t)73)
    , inverterBRActuatorComponent((xme_core_component_t)74)
    , accelerationPedalSensorComponent((xme_core_component_t)81)

    //, dataPacket111out((xme_core_dataManager_dataPacketId_t)111)
    //, dataPacket112out((xme_core_dataManager_dataPacketId_t)112)
    //, dataPacket121out((xme_core_dataManager_dataPacketId_t)121)
    //, dataPacket122out((xme_core_dataManager_dataPacketId_t)122)
    //, dataPacket131out((xme_core_dataManager_dataPacketId_t)131)
    //, dataPacket141out((xme_core_dataManager_dataPacketId_t)141)
    //, dataPacket142out((xme_core_dataManager_dataPacketId_t)142)
    //, dataPacket143out((xme_core_dataManager_dataPacketId_t)143)
    //, dataPacket144out((xme_core_dataManager_dataPacketId_t)144)
    //, dataPacket132out((xme_core_dataManager_dataPacketId_t)132)
    //, dataPacket211in1((xme_core_dataManager_dataPacketId_t)2111)
    //, dataPacket211in2((xme_core_dataManager_dataPacketId_t)2112)
    //, dataPacket211out3((xme_core_dataManager_dataPacketId_t)2113)
    //, dataPacket221in1((xme_core_dataManager_dataPacketId_t)2211)
    //, dataPacket221in2((xme_core_dataManager_dataPacketId_t)2212)
    //, dataPacket221in3((xme_core_dataManager_dataPacketId_t)2213)
    //, dataPacket221in4((xme_core_dataManager_dataPacketId_t)2214)
    //, dataPacket221in5((xme_core_dataManager_dataPacketId_t)2215)
    //, dataPacket221out6((xme_core_dataManager_dataPacketId_t)2216)
    //, dataPacket221out7((xme_core_dataManager_dataPacketId_t)2217)
    //, dataPacket231in1((xme_core_dataManager_dataPacketId_t)2311)
    //, dataPacket231in2((xme_core_dataManager_dataPacketId_t)2312)
    //, dataPacket231in3((xme_core_dataManager_dataPacketId_t)2313)
    //, dataPacket231in4((xme_core_dataManager_dataPacketId_t)2314)
    //, dataPacket231in5((xme_core_dataManager_dataPacketId_t)2315)
    //, dataPacket231out6((xme_core_dataManager_dataPacketId_t)2316)
    //, dataPacket231out7((xme_core_dataManager_dataPacketId_t)2317)
    //, dataPacket311in((xme_core_dataManager_dataPacketId_t)311)
    //, dataPacket312in((xme_core_dataManager_dataPacketId_t)312)
    //, dataPacket321in((xme_core_dataManager_dataPacketId_t)321)
    //, dataPacket322in((xme_core_dataManager_dataPacketId_t)322)
    //, dataPacket331in((xme_core_dataManager_dataPacketId_t)331)
    //, dataPacket332in((xme_core_dataManager_dataPacketId_t)332)
    , topic1((xme_core_topic_t)1)
    , topic2((xme_core_topic_t)2)
    , topic3((xme_core_topic_t)3)
    , topic4((xme_core_topic_t)4)
    , topic5((xme_core_topic_t)5)
    , topic6((xme_core_topic_t)6)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init((xme_hal_graph_graph_t*)&routes));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
    }

    virtual ~LogicalRouteManagerInterfaceTestForRACEUseCase()
    {
        xme_core_pnp_lrm_fini();

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini((xme_hal_graph_graph_t*)&routes));
    }

    xme_core_node_nodeId_t DCCNode;
    xme_core_node_nodeId_t steeringWheelBSensorNode;
    xme_core_node_nodeId_t steeringWheelRSensorNode;
    xme_core_node_nodeId_t steeringBoxBSensorActuatorNode;
    xme_core_node_nodeId_t steeringBoxRSensorActuatorNode;
    xme_core_node_nodeId_t brakeSensorActuatorNode;
    xme_core_node_nodeId_t wheelRotSensorInverterActuatorNode;
    xme_core_node_nodeId_t accelerationPedalSensorNode;

    xme_core_component_t steeringAppComponent;
    xme_core_component_t brakeAppComponent;
    xme_core_component_t accelerationAppComponent;
    xme_core_component_t steeringWheelBSensorComponent;
    xme_core_component_t steeringWheelRSensorComponent;
    xme_core_component_t steeringBoxBSensorActuatorComponent;
    xme_core_component_t steeringBoxRSensorActuatorComponent;
    xme_core_component_t brakePedalSensorComponent;
    xme_core_component_t wheelRotFrequencyFLSensorComponent;
    xme_core_component_t wheelRotFrequencyFRSensorComponent;
    xme_core_component_t brakeFLActuatorComponent;
    xme_core_component_t brakeFRActuatorComponent;
    xme_core_component_t wheelRotFrequencyBLSensorComponent;
    xme_core_component_t wheelRotFrequencyBRSensorComponent;
    xme_core_component_t inverterBLActuatorComponent;
    xme_core_component_t inverterBRActuatorComponent;
    xme_core_component_t accelerationPedalSensorComponent;

    //xme_core_dataManager_dataPacketId_t dataPacket111out;
    //xme_core_dataManager_dataPacketId_t dataPacket112out;
    //xme_core_dataManager_dataPacketId_t dataPacket121out;
    //xme_core_dataManager_dataPacketId_t dataPacket122out;
    //xme_core_dataManager_dataPacketId_t dataPacket131out;
    //xme_core_dataManager_dataPacketId_t dataPacket141out;
    //xme_core_dataManager_dataPacketId_t dataPacket142out;
    //xme_core_dataManager_dataPacketId_t dataPacket143out;
    //xme_core_dataManager_dataPacketId_t dataPacket144out;
    //xme_core_dataManager_dataPacketId_t dataPacket132out;
    //xme_core_dataManager_dataPacketId_t dataPacket211in1;
    //xme_core_dataManager_dataPacketId_t dataPacket211in2;
    //xme_core_dataManager_dataPacketId_t dataPacket211out3;
    //xme_core_dataManager_dataPacketId_t dataPacket221in1;
    //xme_core_dataManager_dataPacketId_t dataPacket221in2;
    //xme_core_dataManager_dataPacketId_t dataPacket221in3;
    //xme_core_dataManager_dataPacketId_t dataPacket221in4;
    //xme_core_dataManager_dataPacketId_t dataPacket221in5;
    //xme_core_dataManager_dataPacketId_t dataPacket221out6;
    //xme_core_dataManager_dataPacketId_t dataPacket221out7;
    //xme_core_dataManager_dataPacketId_t dataPacket231in1;
    //xme_core_dataManager_dataPacketId_t dataPacket231in2;
    //xme_core_dataManager_dataPacketId_t dataPacket231in3;
    //xme_core_dataManager_dataPacketId_t dataPacket231in4;
    //xme_core_dataManager_dataPacketId_t dataPacket231in5;
    //xme_core_dataManager_dataPacketId_t dataPacket231out6;
    //xme_core_dataManager_dataPacketId_t dataPacket231out7;
    //xme_core_dataManager_dataPacketId_t dataPacket311in;
    //xme_core_dataManager_dataPacketId_t dataPacket312in;
    //xme_core_dataManager_dataPacketId_t dataPacket321in;
    //xme_core_dataManager_dataPacketId_t dataPacket322in;
    //xme_core_dataManager_dataPacketId_t dataPacket331in;
    //xme_core_dataManager_dataPacketId_t dataPacket332in;

    xme_core_topic_t topic1;
    xme_core_topic_t topic2;
    xme_core_topic_t topic3;
    xme_core_topic_t topic4;
    xme_core_topic_t topic5;
    xme_core_topic_t topic6;

    xme_core_pnp_lrm_logicalRoutes_t routes;
};

/******************************************************************************/
/***   Helper Functions                                                     ***/
/******************************************************************************/

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     LogicalRouteManagerInterfaceTest                                       //
//----------------------------------------------------------------------------//

TEST_F(LogicalRouteManagerInterfaceTest, announcePortWithInvalidNodeId)
{
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_lrm_announcePort
        (
            nodeInv,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );
}

TEST_F(LogicalRouteManagerInterfaceTest, announcePortWithInvalidComponentId)
{
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            compInv,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );
}

TEST_F(LogicalRouteManagerInterfaceTest, announcePortWithInvalidPortType)
{
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_INVALID,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );
}

// ----------

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithoutAnnouncement)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithOnePublication)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithTwoPublicationsOfSameTopic)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            2,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithTwoPublicationsOfDifferentTopic)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            2,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithOneSubscription)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithTwoSubscriptionssOfSameTopic)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithTwoSubscriptionssOfDifferentTopic)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithPublicationAndSubscriptionOfSameTopicAndNoAttributes)
{
    xme_core_pnp_dataLinkGraph_vertexData_t* vd1 = NULL;
    xme_core_pnp_dataLinkGraph_vertexData_t* vd2 = NULL;
    xme_core_pnp_dataLinkGraph_vertexData_t* vd3 = NULL;
    xme_core_pnp_dataLinkGraph_edgeData_t* ed1 = NULL;
    xme_core_pnp_dataLinkGraph_edgeData_t* ed2 = NULL;
    xme_hal_graph_vertexId_t vi1;
    xme_hal_graph_vertexId_t vi2;
    xme_hal_graph_vertexId_t vi3;
    xme_hal_graph_edgeId_t ei1;
    xme_hal_graph_edgeId_t ei2;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // --------------------------------------------------
    //  Issue the first call to getLogicalRoutes()
    // --------------------------------------------------

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator((xme_hal_graph_graph_t*)&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi1 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi2 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator((xme_hal_graph_graph_t*)&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    {
        if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1))
        {
            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1);
        }
        else if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2);

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else
        {
            // Either v1 or v2 must have an outgoing edge
            ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1) || xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData((xme_hal_graph_graph_t*)&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    // Check vertex data vd1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
    EXPECT_EQ(vi1, vd1->vertexId);
    EXPECT_EQ(node1, vd1->nodeId);
    EXPECT_EQ(topic1, vd1->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, vd1->vertexData.componentPortVertex.portType);
    // TODO: vd1->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd2

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed1->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed1->edgeData.logicalRouteEdge.established);
    // TODO: ed1->edgeData.logicalRouteEdge (currently not filled with any meaningful data execpt for "channelId" and "established")

    // Commit the logical route

    //ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_setLogicalRoute(ed1->edgeId, ed1->edgeData.logicalRouteEdge.channelId, XME_CORE_INVALID_TRANSACTION_ID));



    // --------------------------------------------------
    //  Issue the second call to getLogicalRoutes()
    //  without any other publications or subscriptions
    // --------------------------------------------------

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator((xme_hal_graph_graph_t*)&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi1 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi2 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator((xme_hal_graph_graph_t*)&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    {
        if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1))
        {
            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1);
        }
        else if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2);

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else
        {
            // Either v1 or v2 must have an outgoing edge
            ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1) || xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData((xme_hal_graph_graph_t*)&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    // Check vertex data vd1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
    EXPECT_EQ(vi1, vd1->vertexId);
    EXPECT_EQ(node1, vd1->nodeId);
    EXPECT_EQ(topic1, vd1->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, vd1->vertexData.componentPortVertex.portType);
    // TODO: vd1->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd2

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed1->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed1->edgeData.logicalRouteEdge.established); ///< Note that the logical route is not established yet. 
    // TODO: ed1->edgeData.logicalRouteEdge (currently not filled with any meaningful data except for "channelId" and "established")



    // --------------------------------------------------
    //  Issue the third call to getLogicalRoutes()
    //  with one more subscription
    // --------------------------------------------------

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp2,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            3,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(3, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator((xme_hal_graph_graph_t*)&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi1 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi2 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi3 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator((xme_hal_graph_graph_t*)&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi3));
    {
        if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1))
        {
            // v1 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1));
            ei2 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1));
        }
        else if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            // v2 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));
            ei2 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3))
        {
            xme_hal_graph_vertexId_t temp;

            // v3 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3));
            ei2 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3));

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi3;
            vi3 = temp;
        }
        else
        {
            // Either v1, v2 or v3 must have two outgoing edges
            ASSERT_TRUE
            (
                xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1) ||
                xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2) ||
                xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3)
            );

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi3, (void**)&vd3));
    ASSERT_TRUE(NULL != vd3);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData((xme_hal_graph_graph_t*)&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData((xme_hal_graph_graph_t*)&routes, ei2, (void**)&ed2));
    ASSERT_TRUE(NULL != ed2);

    // Check vertex data vd1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
    EXPECT_EQ(vi1, vd1->vertexId);
    EXPECT_EQ(node1, vd1->nodeId);
    EXPECT_EQ(topic1, vd1->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, vd1->vertexData.componentPortVertex.portType);
    // TODO: vd1->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Sort v2 and v3: v2 is the one on node2, v3 the one on node1
    ASSERT_TRUE(node1 == vd2->nodeId || node2 == vd2->nodeId);
    if (node1 == vd2->nodeId)
    {
        xme_hal_graph_vertexId_t tempV;
        xme_core_pnp_dataLinkGraph_vertexData_t* tempD;

        // Swap vertex indices such that v2 is the first created vertex
        tempV = vi2;
        vi2 = vi3;
        vi3 = tempV;

        tempD = vd2;
        vd2 = vd3;
        vd3 = tempD;
    }

    // Check vertex data vd2

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd3

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd3->vertexType);
    EXPECT_EQ(vi3, vd3->vertexId);
    EXPECT_EQ(node1, vd3->nodeId);
    EXPECT_EQ(topic1, vd3->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd3->vertexData.componentPortVertex.portType);
    // TODO: vd3->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed1->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed1->edgeData.logicalRouteEdge.established);
    // TODO: ed1->edgeData.logicalRouteEdge (currently not filled with any meaningful data execpt for "channelId" and "established")

    // Check edge data ed2

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed2->edgeType);
    EXPECT_EQ(ei2, ed2->edgeId);
    EXPECT_EQ(topic1, ed2->topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed2->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed2->edgeData.logicalRouteEdge.established);
    // TODO: ed2->edgeData.logicalRouteEdge (currently not filled with any meaningful data execpt for "channelId" and "established")

    // Channel numbers must be disjoint
    EXPECT_NE(ed1->edgeData.logicalRouteEdge.channelId, ed2->edgeData.logicalRouteEdge.channelId);

    // Commit the logical route

    //EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_pnp_lrm_setLogicalRoute(ed1->edgeId, ed1->edgeData.logicalRouteEdge.channelId, XME_CORE_INVALID_TRANSACTION_ID));
    //ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_setLogicalRoute(ed2->edgeId, ed2->edgeData.logicalRouteEdge.channelId, XME_CORE_INVALID_TRANSACTION_ID));



    // --------------------------------------------------
    //  Issue the fourth call to getLogicalRoutes()
    //  without any other publications or subscriptions
    // --------------------------------------------------

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(3, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator((xme_hal_graph_graph_t*)&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi1 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi2 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi3 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator((xme_hal_graph_graph_t*)&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi3));
    {
        if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1))
        {
            // v1 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1));
            ei2 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1));
        }
        else if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            // v2 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));
            ei2 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3))
        {
            xme_hal_graph_vertexId_t temp;

            // v3 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3));
            ei2 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3));

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi3;
            vi3 = temp;
        }
        else
        {
            // Either v1, v2 or v3 must have two outgoing edges
            ASSERT_TRUE
            (
                xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1) ||
                xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2) ||
                xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3)
            );

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi3));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi3, (void**)&vd3));
    ASSERT_TRUE(NULL != vd3);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData((xme_hal_graph_graph_t*)&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData((xme_hal_graph_graph_t*)&routes, ei2, (void**)&ed2));
    ASSERT_TRUE(NULL != ed2);

    // Check vertex data vd1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
    EXPECT_EQ(vi1, vd1->vertexId);
    EXPECT_EQ(node1, vd1->nodeId);
    EXPECT_EQ(topic1, vd1->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, vd1->vertexData.componentPortVertex.portType);
    // TODO: vd1->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Sort v2 and v3: v2 is the one on node2, v3 the one on node1
    ASSERT_TRUE(node1 == vd2->nodeId || node2 == vd2->nodeId);
    if (node1 == vd2->nodeId)
    {
        xme_hal_graph_vertexId_t tempV;
        xme_core_pnp_dataLinkGraph_vertexData_t* tempD;

        // Swap vertex indices such that v2 is the first created vertex
        tempV = vi2;
        vi2 = vi3;
        vi3 = tempV;

        tempD = vd2;
        vd2 = vd3;
        vd3 = tempD;
    }

    // Check vertex data vd2

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd3

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd3->vertexType);
    EXPECT_EQ(vi3, vd3->vertexId);
    EXPECT_EQ(node1, vd3->nodeId);
    EXPECT_EQ(topic1, vd3->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd3->vertexData.componentPortVertex.portType);
    // TODO: vd3->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed1->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed1->edgeData.logicalRouteEdge.established);
    // TODO: ed1->edgeData.logicalRouteEdge (currently not filled with any meaningful data execpt for "channelId" and "established")

    // Check edge data ed2

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed2->edgeType);
    EXPECT_EQ(ei2, ed2->edgeId);
    EXPECT_EQ(topic1, ed2->topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed2->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed2->edgeData.logicalRouteEdge.established);
    // TODO: ed2->edgeData.logicalRouteEdge (currently not filled with any meaningful data execpt for "channelId" and "established")

    // Channel numbers must be disjoint
    EXPECT_NE(ed1->edgeData.logicalRouteEdge.channelId, ed2->edgeData.logicalRouteEdge.channelId);

    // Now, we establish the logical routes, so these appear as established. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_setLogicalRoute(ed1->edgeId, ed1->edgeData.logicalRouteEdge.channelId, XME_CORE_INVALID_TRANSACTION_ID));
    EXPECT_TRUE(ed1->edgeData.logicalRouteEdge.established);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_setLogicalRoute(ed2->edgeId, ed2->edgeData.logicalRouteEdge.channelId, XME_CORE_INVALID_TRANSACTION_ID));
    EXPECT_TRUE(ed2->edgeData.logicalRouteEdge.established);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&routes));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(3, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithPublicationAndSubscriptionOfSameTopicWithNonMatchingAttributes)
{
    xme_core_directory_attributeSetHandle_t attrP;
    xme_core_directory_attributeSetHandle_t attrS;
    int64_t filterValue = 0x0123456789ABCDEFLL;

    // Create publication attribute set (blank)
    attrP = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrP);

    // Create subscription attribute set (numeric '==' attribute filter)
    attrS = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrS);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeFilter(
            attrS,
            (xme_core_attribute_key_t) 1,
            &filterValue,
            1U,
            sizeof(filterValue),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1,
            topic1,
            attrP,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2,
            topic1,
            attrS,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Call getLogicalRoutes()
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));

    // Free resources
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(attrS));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(attrP));
}

TEST_F(LogicalRouteManagerInterfaceTest, getLogicalRoutesWithPublicationAndSubscriptionOfSameTopicWithMatchingAttributes)
{
    xme_core_pnp_dataLinkGraph_vertexData_t* vd1 = NULL;
    xme_core_pnp_dataLinkGraph_vertexData_t* vd2 = NULL;
    xme_core_pnp_dataLinkGraph_edgeData_t* ed1 = NULL;
    xme_hal_graph_vertexId_t vi1;
    xme_hal_graph_vertexId_t vi2;
    xme_hal_graph_edgeId_t ei1;

    xme_core_directory_attributeSetHandle_t attrP;
    xme_core_directory_attributeSetHandle_t attrS;

    // Create publication attribute set (blank)
    attrP = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrP);

    // Create subscription attribute set (blank)
    attrS = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrS);

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1,
            topic1,
            attrP,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2,
            topic1,
            attrS,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Call getLogicalRoutes()
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator((xme_hal_graph_graph_t*)&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi1 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
        vi2 = xme_hal_graph_nextVertex((xme_hal_graph_graph_t*)&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex((xme_hal_graph_graph_t*)&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator((xme_hal_graph_graph_t*)&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    {
        if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1))
        {
            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1);
        }
        else if (xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            ei1 = xme_hal_graph_nextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2);

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else
        {
            // Either v1 or v2 must have an outgoing edge
            ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1) || xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge((xme_hal_graph_graph_t*)&routes, vi2));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator((xme_hal_graph_graph_t*)&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData((xme_hal_graph_graph_t*)&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData((xme_hal_graph_graph_t*)&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    // Check vertex data vd1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
    EXPECT_EQ(vi1, vd1->vertexId);
    EXPECT_EQ(node1, vd1->nodeId);
    EXPECT_EQ(topic1, vd1->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, vd1->vertexData.componentPortVertex.portType);
    // TODO: vd1->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd2

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed1->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed1->edgeData.logicalRouteEdge.established);
    // TODO: ed1->edgeData.logicalRouteEdge (currently not filled with any meaningful data execpt for "channelId" and "established")

    // Commit the logical route

    //ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_setLogicalRoute(ed1->edgeId, ed1->edgeData.logicalRouteEdge.channelId, XME_CORE_INVALID_TRANSACTION_ID));

    // Free resources
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(attrS));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(attrP));
}

////////////////////////////////////////////////////////////////////////////////////
///   Case 1: Self Subscription                                                  ///
////////////////////////////////////////////////////////////////////////////////////
TEST_F(LogicalRouteManagerInterfaceTestForRACEUseCase, announcePortsForSelfSubscriptionFor2SubsAnd1Pub)
{
    // Two times a subscription and one publication of the same topic inside the same component. 
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            3,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(3, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTestForRACEUseCase, announcePortsForSelfSubscriptionFor1SubAnd2Pubs)
{
    // Two times a subscription and one publication of the same topic inside the same component. 
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            2,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            3,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(3, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTestForRACEUseCase, announcePortsForSelfSubscriptionFor2SubsAnd2Pubs)
{
    // Two times a subscription and one publication of the same topic inside the same component. 
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            3,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            4,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4, xme_hal_graph_getVertexCount((xme_hal_graph_graph_t*)&routes));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount((xme_hal_graph_graph_t*)&routes));
}

TEST_F(LogicalRouteManagerInterfaceTestForRACEUseCase, RacePipecleanerExample)
{
    // NODE - Steering Wheel Sensors
    // Publication from Steering Wheel Sensor B
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            steeringWheelBSensorNode,
            steeringWheelBSensorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication from Steering Wheel Sensor R
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            steeringWheelRSensorNode,
            steeringWheelRSensorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE - Steering Box Sensors
    // Publication from Steering Box Sensor B
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            steeringBoxBSensorActuatorNode,
            steeringBoxBSensorActuatorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication from Steering Box Sensor R
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            steeringBoxRSensorActuatorNode,
            steeringBoxRSensorActuatorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );


    // NODE - Break Pedal Sensors + 2 Wheel Rotation Sensors from FL and FR
    // Publication from Brake Pedal Sensor
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            brakeSensorActuatorNode,
            brakePedalSensorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic3,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication from Wheel Rotation Frequency Sensor FL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            brakeSensorActuatorNode,
            wheelRotFrequencyFLSensorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication from Wheel Rotation Frequency Sensor FR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            brakeSensorActuatorNode,
            wheelRotFrequencyFRSensorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication from Wheel Rotation Frequency Sensor BL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            wheelRotSensorInverterActuatorNode,
            wheelRotFrequencyBLSensorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication from Wheel Rotation Frequency Sensor BR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            wheelRotSensorInverterActuatorNode,
            wheelRotFrequencyBRSensorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE - Accel Pedal Sensor
    // Publication from Accel Pedal Sensor
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            accelerationPedalSensorNode,
            accelerationPedalSensorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0,
            topic3,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE - Steering App
    // subscription from Steering Wheel Sensor B
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // subscription from Steering Wheel Sensor R
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication to Steering Box Actuators B and R
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            steeringAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            2,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE - Brake App
    // subscription from Brake Pedal Sensor
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            brakeAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0,
            topic3,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // subscription from wheel Sensor FL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            brakeAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // subscription from wheel Sensor FR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            brakeAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // subscription from wheel Sensor BL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            brakeAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            3,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // subscription from wheel Sensor BR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            brakeAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            4,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication to Brake Actuator FL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            brakeAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            5,
            topic5,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication to Brake Actuator FR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            brakeAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            6,
            topic5,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE - Accel App
    // Subscription from accelPedalSensor
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            accelerationAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0,
            topic3,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // subscription from Wheel FL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            accelerationAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // subscription from Wheel FR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            accelerationAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // subscription from Wheel BL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            accelerationAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            3,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // subscription from Wheel BR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            accelerationAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            4,
            topic4,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication to Inverter Actuator BL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            accelerationAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            5,
            topic5,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Publication to Inverter Actuator BR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            DCCNode,
            accelerationAppComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            6,
            topic5,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE - Steering Box Actuator
    // Subscription from Steering App to Steering Box Actuator B
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            steeringBoxBSensorActuatorNode,
            steeringBoxBSensorActuatorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Subscription from Steering App to Steering Box Actuator R
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            steeringBoxRSensorActuatorNode,
            steeringBoxRSensorActuatorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE - Brake Actuator
    // Subscription from Brake App to Brake Actuator FL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            brakeSensorActuatorNode,
            brakeFLActuatorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0,
            topic5,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Subscription from Brake App to Brake Actuator FR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            brakeSensorActuatorNode,
            brakeFRActuatorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0,
            topic5,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE - Inverter Actuator
    // Subscription from Accel App to Inverter Actuator BL
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            wheelRotSensorInverterActuatorNode,
            inverterBLActuatorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0,
            topic5,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Subscription from Accel App to Inverter Actuator BR
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            wheelRotSensorInverterActuatorNode,
            inverterBRActuatorComponent,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0,
            topic5,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));

    xme_hal_graph_initVertexIterator(&routes);
    {
        xme_core_pnp_dataLinkGraph_vertexData_t *vd1;
        xme_hal_graph_vertexId_t v1;
        int numEdges = 0;

        while(xme_hal_graph_hasNextVertex(&routes))
        {
            v1 = xme_hal_graph_nextVertex(&routes);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v1, (void **)&vd1));

            // Steering Wheel B
            if(vd1->nodeId == steeringWheelBSensorNode
                && vd1->vertexData.componentPortVertex.componentId == steeringWheelBSensorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                EXPECT_EQ(1, numEdges);
            }
            // Steering Wheel R
            else if(vd1->nodeId == steeringWheelRSensorNode
                && vd1->vertexData.componentPortVertex.componentId == steeringWheelRSensorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                EXPECT_EQ(1, numEdges);
            }
            // Steering Box B
            else if(vd1->nodeId == steeringBoxBSensorActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == steeringBoxBSensorActuatorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(3, numEdges);
            }
            // Steering Box R
            else if(vd1->nodeId == steeringBoxRSensorActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == steeringBoxRSensorActuatorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(3, numEdges);
            }
            // Brake Pedal
            else if(vd1->nodeId == brakeSensorActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == brakePedalSensorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(2, numEdges);
            }
            // Wheel Rotation Frequency Sensoe FL - included in brakePedalSensorNode
            else if(vd1->nodeId == brakeSensorActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == wheelRotFrequencyFLSensorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 1)
            {
                // Each Wheel Rotation sensor should ideally send messages to 2 ports, one in
                // Brake App component, and another in Acceleration App component. Since, attributes
                // are not supported by LRM the message is sent to 8 ports, 4 in Brake App and 4 in Accel App.
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 2
                EXPECT_EQ(8, numEdges);
            }
            // Wheel Rotation Frequency Sensoe FR - included in brakePedalSensorNode
            else if(vd1->nodeId == brakeSensorActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == wheelRotFrequencyFRSensorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 2)
            {
                // Each Wheel Rotation sensor should ideally send messages to 2 ports, one in
                // Brake App component, and another in Acceleration App component. Since, attributes
                // are not supported by LRM the message is sent to 8 ports, 4 in Brake App and 4 in Accel App.
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 2
                EXPECT_EQ(8, numEdges);
            }
            // Wheel Rotation Frequency Sensor BL Component
            else if(vd1->nodeId == wheelRotSensorInverterActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == wheelRotFrequencyFRSensorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                // Each Wheel Rotation sensor should ideally send messages to 2 ports, one in
                // Brake App component, and another in Acceleration App component. Since, attributes
                // are not supported by LRM the message is sent to 8 ports, 4 in Brake App and 4 in Accel App.
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 2
                EXPECT_EQ(8, numEdges);
            }
            // Wheel Rotation Frequency Sensor BR Component
            else if(vd1->nodeId == wheelRotSensorInverterActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == wheelRotFrequencyFRSensorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 1)
            {
                // Each Wheel Rotation sensor should ideally send messages to 2 ports, one in
                // Brake App component, and another in Acceleration App component. Since, attributes
                // are not supported by LRM the message is sent to 8 ports, 4 in Brake App and 4 in Accel App.
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 2
                EXPECT_EQ(8, numEdges);
            }
            // Acceleration Pedal
            else if(vd1->nodeId == accelerationPedalSensorNode
                && vd1->vertexData.componentPortVertex.componentId == accelerationPedalSensorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(2, numEdges);
            }

            // Steering App - steering wheel subscription
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == steeringAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                EXPECT_EQ(2, numEdges);
            }
            // Steering App - steering box subscription
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == steeringAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 1)
            {
                // Port Index 1, in Steering App receives messages from Steering Box B and R and
                // since attributes are not supported, also receives messages from its own port
                // with index 2
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 2
                EXPECT_EQ(3, numEdges);
            }
            // Steering App - steering box actuator publication
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == steeringAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 2)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 2
                EXPECT_EQ(3, numEdges);
            }

            // Brake App - Brake Pedal Sensor Subscription Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == brakeAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(2, numEdges);
            }
            // Brake App - Wheel Rotation Frequency FL Subscription Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == brakeAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 1)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Brake App - Wheel Rotation Frequency FR Subscription Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == brakeAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 2)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Brake App - Wheel Rotation Frequency BL Subscription Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == brakeAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 3)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Brake App - Wheel Rotation Frequency BR Subscription Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == brakeAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 4)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Brake App - Brake FL Actuator Publication Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == brakeAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 5)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Brake App - Brake FR Actuator Publication Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == brakeAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 6)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }

            // Acceleration App - Acceleration Pedal Sensor Subscription Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == accelerationAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(2, numEdges);
            }
            // Acceleration App - Wheel Rotation Frequency Sensor FL Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == accelerationAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 1)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Acceleration App - Wheel Rotation Frequency Sensor FR Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == accelerationAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 2)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Acceleration App - Wheel Rotation Frequency Sensor BL Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == accelerationAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 3)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Acceleration App - Wheel Rotation Frequency Sensor BR Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == accelerationAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 4)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Acceleration App - Inverter FL Actuator Publication Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == accelerationAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 5)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Acceleration App - Inverter FL Actuator Publication Port
            else if(vd1->nodeId == DCCNode
                && vd1->vertexData.componentPortVertex.componentId == accelerationAppComponent
                && vd1->vertexData.componentPortVertex.portIndex == 6)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initOutgoingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextOutgoingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextOutgoingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiOutgoingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Steering Box Actuator B
            else if(vd1->nodeId == steeringBoxBSensorActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == steeringBoxBSensorActuatorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(3, numEdges);
            }
            // Steering Box Actuator R
            else if(vd1->nodeId == steeringBoxRSensorActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == steeringBoxRSensorActuatorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(3, numEdges);
            }
            // Brake Actuator FL
            else if(vd1->nodeId == brakeSensorActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == brakeFLActuatorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Brake Actuator FR
            else if(vd1->nodeId == brakeSensorActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == brakeFRActuatorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Inverter Actuator BL
            else if(vd1->nodeId == wheelRotSensorInverterActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == inverterBLActuatorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
            // Inverter Actuator BR
            else if(vd1->nodeId == wheelRotSensorInverterActuatorNode
                && vd1->vertexData.componentPortVertex.componentId == inverterBRActuatorComponent
                && vd1->vertexData.componentPortVertex.portIndex == 0)
            {
                EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                    vd1->vertexData.componentPortVertex.portType);
                xme_hal_graph_initIncomingEdgeIterator(&routes, v1);
                {
                    numEdges = 0;
                    while(xme_hal_graph_hasNextIncomingEdge(&routes, v1))
                    {
                        EXPECT_GE(xme_hal_graph_nextIncomingEdge(&routes, v1), 0);
                        numEdges++;
                    }
                }
                xme_hal_graph_finiIncomingEdgeIterator(&routes, v1);
                // When attributes are supported, expected numEdges should be 1
                EXPECT_EQ(4, numEdges);
            }
        }
    }
    xme_hal_graph_finiVertexIterator(&routes);
}


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
