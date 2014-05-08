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
 * $Id: interfaceTestLogicalRouteManager.cpp 6288 2014-01-10 12:07:24Z wiesmueller $
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
    , node3((xme_core_node_nodeId_t)3)
    , compInv(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT)
    , compTypeInv(XME_CORE_COMPONENT_TYPE_INVALID)
    , comp1((xme_core_component_t)1)
    , compType1((xme_core_componentType_t)1)
    , comp2((xme_core_component_t)2)
    , compType2((xme_core_componentType_t)2)
    , topicInv(XME_CORE_TOPIC_INVALID_TOPIC)
    , topic1((xme_core_topic_t) 1)
    , topic2((xme_core_topic_t) 2)
    , dpInv(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)
    , dp1((xme_core_dataManager_dataPacketId_t)1)
    , dp2((xme_core_dataManager_dataPacketId_t)2)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&routes));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
    }

    virtual ~LogicalRouteManagerInterfaceTest()
    {
        xme_core_pnp_lrm_fini();

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&routes));
    }

    void testInvalidConnectionBounds(xme_core_component_portType_t portType)
    {
        EXPECT_EQ
        (
            XME_STATUS_INVALID_CONFIGURATION,
            xme_core_pnp_lrm_announcePort
            (
                node1,
                comp1,
                compType1,
                portType,
                0U,
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
                topic1,
                XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
                XME_CORE_INVALID_TRANSACTION_ID
            )
        );

        EXPECT_EQ
        (
            XME_STATUS_INVALID_CONFIGURATION,
            xme_core_pnp_lrm_announcePort
            (
                node1,
                comp1,
                compType1,
                portType,
                0U,
                0U,
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
                topic1,
                XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
                XME_CORE_INVALID_TRANSACTION_ID
            )
        );

        EXPECT_EQ
        (
            XME_STATUS_INVALID_CONFIGURATION,
            xme_core_pnp_lrm_announcePort
            (
                node1,
                comp1,
                compType1,
                portType,
                0U,
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
                0U,
                topic1,
                XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
                XME_CORE_INVALID_TRANSACTION_ID
            )
        );
    }

    xme_core_node_nodeId_t nodeInv;
    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node2;
    xme_core_node_nodeId_t node3;

    xme_core_component_t compInv;
    xme_core_componentType_t compTypeInv;
    xme_core_component_t comp1;
    xme_core_componentType_t compType1;
    xme_core_component_t comp2;
    xme_core_componentType_t compType2;

    xme_core_topic_t topicInv;
    xme_core_topic_t topic1;
    xme_core_topic_t topic2;

    xme_core_dataManager_dataPacketId_t dpInv;
    xme_core_dataManager_dataPacketId_t dp1;
    xme_core_dataManager_dataPacketId_t dp2;

    xme_core_pnp_lrm_logicalRoutes_t routes;
};

class LogicalRouteManagerInterfaceTestForDCC: public LogicalRouteManagerInterfaceTest
{
protected:
    LogicalRouteManagerInterfaceTestForDCC()
    {
    }

    virtual ~LogicalRouteManagerInterfaceTestForDCC()
    {
    }
};

class LogicalRouteManagerInterfaceTestForRR: public LogicalRouteManagerInterfaceTest
{
    // Terminology:
    // - A "client" is a component that is a request sender and a response handler.
    // - A "server" is a component that is a request handler and a response sender.
    //
    // Abbreviations:
    // - LR: Logical Route edge
    // - CM: Channel Mapping edge
    // - RqS: Request sender component port vertex
    // - RqH: Request handler component port vertex
    // - RsS: Response sender component port vertex
    // - RsH: Response handler component port vertex

protected:
    LogicalRouteManagerInterfaceTestForRR()
    : requestTopic1((xme_core_topic_t) 11)
    , requestTopic2((xme_core_topic_t) 12)
    , responseTopic1((xme_core_topic_t) 21)
    , responseTopic2((xme_core_topic_t) 22)
    {
    }

    virtual ~LogicalRouteManagerInterfaceTestForRR()
    {
    }

    xme_core_topic_t requestTopic1;
    xme_core_topic_t requestTopic2;
    xme_core_topic_t responseTopic1;
    xme_core_topic_t responseTopic2;
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
    , steeringAppComponentType((xme_core_componentType_t)11)
    , brakeAppComponent((xme_core_component_t)12)
    , brakeAppComponentType((xme_core_componentType_t)12)
    , accelerationAppComponent((xme_core_component_t)13)
    , accelerationAppComponentType((xme_core_componentType_t)13)
    , steeringWheelBSensorComponent((xme_core_component_t)21)
    , steeringWheelBSensorComponentType((xme_core_componentType_t)21)
    , steeringWheelRSensorComponent((xme_core_component_t)31)
    , steeringWheelRSensorComponentType((xme_core_componentType_t)31)
    , steeringBoxBSensorActuatorComponent((xme_core_component_t)41)
    , steeringBoxBSensorActuatorComponentType((xme_core_componentType_t)41)
    , steeringBoxRSensorActuatorComponent((xme_core_component_t)51)
    , steeringBoxRSensorActuatorComponentType((xme_core_componentType_t)51)
    , brakePedalSensorComponent((xme_core_component_t)61)
    , brakePedalSensorComponentType((xme_core_componentType_t)61)
    , wheelRotFrequencyFLSensorComponent((xme_core_component_t)62)
    , wheelRotFrequencyFLSensorComponentType((xme_core_componentType_t)62)
    , wheelRotFrequencyFRSensorComponent((xme_core_component_t)63)
    , wheelRotFrequencyFRSensorComponentType((xme_core_componentType_t)63)
    , brakeFLActuatorComponent((xme_core_component_t)64)
    , brakeFLActuatorComponentType((xme_core_componentType_t)64)
    , brakeFRActuatorComponent((xme_core_component_t)65)
    , brakeFRActuatorComponentType((xme_core_componentType_t)65)
    , wheelRotFrequencyBLSensorComponent((xme_core_component_t)71)
    , wheelRotFrequencyBLSensorComponentType((xme_core_componentType_t)71)
    , wheelRotFrequencyBRSensorComponent((xme_core_component_t)72)
    , wheelRotFrequencyBRSensorComponentType((xme_core_componentType_t)72)
    , inverterBLActuatorComponent((xme_core_component_t)73)
    , inverterBLActuatorComponentType((xme_core_componentType_t)73)
    , inverterBRActuatorComponent((xme_core_component_t)74)
    , inverterBRActuatorComponentType((xme_core_componentType_t)74)
    , accelerationPedalSensorComponent((xme_core_component_t)81)
    , accelerationPedalSensorComponentType((xme_core_componentType_t)81)

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
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&routes));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
    }

    virtual ~LogicalRouteManagerInterfaceTestForRACEUseCase()
    {
        xme_core_pnp_lrm_fini();

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&routes));
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
    xme_core_componentType_t steeringAppComponentType;
    xme_core_component_t brakeAppComponent;
    xme_core_componentType_t brakeAppComponentType;
    xme_core_component_t accelerationAppComponent;
    xme_core_componentType_t accelerationAppComponentType;
    xme_core_component_t steeringWheelBSensorComponent;
    xme_core_componentType_t steeringWheelBSensorComponentType;
    xme_core_component_t steeringWheelRSensorComponent;
    xme_core_componentType_t steeringWheelRSensorComponentType;
    xme_core_component_t steeringBoxBSensorActuatorComponent;
    xme_core_componentType_t steeringBoxBSensorActuatorComponentType;
    xme_core_component_t steeringBoxRSensorActuatorComponent;
    xme_core_componentType_t steeringBoxRSensorActuatorComponentType;
    xme_core_component_t brakePedalSensorComponent;
    xme_core_componentType_t brakePedalSensorComponentType;
    xme_core_component_t wheelRotFrequencyFLSensorComponent;
    xme_core_componentType_t wheelRotFrequencyFLSensorComponentType;
    xme_core_component_t wheelRotFrequencyFRSensorComponent;
    xme_core_componentType_t wheelRotFrequencyFRSensorComponentType;
    xme_core_component_t brakeFLActuatorComponent;
    xme_core_componentType_t brakeFLActuatorComponentType;
    xme_core_component_t brakeFRActuatorComponent;
    xme_core_componentType_t brakeFRActuatorComponentType;
    xme_core_component_t wheelRotFrequencyBLSensorComponent;
    xme_core_componentType_t wheelRotFrequencyBLSensorComponentType;
    xme_core_component_t wheelRotFrequencyBRSensorComponent;
    xme_core_componentType_t wheelRotFrequencyBRSensorComponentType;
    xme_core_component_t inverterBLActuatorComponent;
    xme_core_componentType_t inverterBLActuatorComponentType;
    xme_core_component_t inverterBRActuatorComponent;
    xme_core_componentType_t inverterBRActuatorComponentType;
    xme_core_component_t accelerationPedalSensorComponent;
    xme_core_componentType_t accelerationPedalSensorComponentType;

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

class LogicalRouteManagerInterfaceTestForRACEUseCaseAndAttributes: public ::testing::Test
{
protected:
    LogicalRouteManagerInterfaceTestForRACEUseCaseAndAttributes()
    : node1((xme_core_node_nodeId_t)1)
    , node3((xme_core_node_nodeId_t)3)

    , component4((xme_core_component_t)4)
    , component4Type((xme_core_componentType_t)4)
    , component5((xme_core_component_t)5)
    , component5Type((xme_core_componentType_t)5)
    , component100((xme_core_component_t)100)
    , component100Type((xme_core_componentType_t)100)

    , topic4232((xme_core_topic_t)4232)
    , topic4108((xme_core_topic_t)4108)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&routes));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_init());
    }

    virtual ~LogicalRouteManagerInterfaceTestForRACEUseCaseAndAttributes()
    {
        xme_core_pnp_lrm_fini();

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&routes));

        xme_core_directory_attribute_fini();
    }

    xme_core_node_nodeId_t node1;
    xme_core_node_nodeId_t node3;

    xme_core_component_t component4;
    xme_core_componentType_t component4Type;
    xme_core_component_t component5;
    xme_core_componentType_t component5Type;
    xme_core_component_t component100;
    xme_core_componentType_t component100Type;

    xme_core_topic_t topic4232;
    xme_core_topic_t topic4108;

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

TEST_F(LogicalRouteManagerInterfaceTest, announcePortWithInvalidConnectionBounds)
{
    testInvalidConnectionBounds(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION);
    testInvalidConnectionBounds(XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER);
    testInvalidConnectionBounds(XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER);
    testInvalidConnectionBounds(XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER);
}

TEST_F(LogicalRouteManagerInterfaceTest, announcePortWithInvalidNodeId)
{
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_lrm_announcePort
        (
            nodeInv,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            compTypeInv,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_INVALID,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );
}

TEST_F(LogicalRouteManagerInterfaceTest, announcePortWithArbitraryPortType)
{
    xme_core_component_portType_t arbitraryPortType = (xme_core_component_portType_t) 42;

    // Ensure that this port type is really invalid
    ASSERT_NE(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, arbitraryPortType);
    ASSERT_NE(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, arbitraryPortType);
    ASSERT_NE(XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER, arbitraryPortType);
    ASSERT_NE(XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER, arbitraryPortType);
    ASSERT_NE(XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER, arbitraryPortType);
    ASSERT_NE(XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER, arbitraryPortType);

    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            arbitraryPortType,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
    EXPECT_EQ(0U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));
}

//----------------------------------------------------------------------------//
//     LogicalRouteManagerInterfaceTestForDCC                                 //
//----------------------------------------------------------------------------//

TEST_F(LogicalRouteManagerInterfaceTestForDCC, getLogicalRoutesWithOnePublication)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(1U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    xme_hal_graph_initVertexIterator(&routes);
    {
        xme_hal_graph_vertexId_t vertexId;
        xme_core_pnp_dataLinkGraph_vertexData_t* vertexData;

        vertexId = xme_hal_graph_nextVertex(&routes);
        xme_hal_graph_getVertexData(&routes, vertexId, (void**)&vertexData);

        EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID, vertexData->vertexData.componentPortVertex.lowerConnectionBound);
        EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID, vertexData->vertexData.componentPortVertex.upperConnectionBound);
    }
    xme_hal_graph_finiVertexIterator(&routes);

    // TODO: Inspect vertex more
}

TEST_F(LogicalRouteManagerInterfaceTestForDCC, getLogicalRoutesWithTwoPublicationsOfSameTopic)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    // TODO: Inspect vertices
}

TEST_F(LogicalRouteManagerInterfaceTestForDCC, getLogicalRoutesWithTwoPublicationsOfDifferentTopic)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    // TODO: Inspect vertices
}

TEST_F(LogicalRouteManagerInterfaceTestForDCC, getLogicalRoutesWithOneSubscription)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(1U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    xme_hal_graph_initVertexIterator(&routes);
    {
        xme_hal_graph_vertexId_t vertexId;
        xme_core_pnp_dataLinkGraph_vertexData_t* vertexData;

        vertexId = xme_hal_graph_nextVertex(&routes);
        xme_hal_graph_getVertexData(&routes, vertexId, (void**)&vertexData);

        EXPECT_EQ(0, vertexData->vertexData.componentPortVertex.lowerConnectionBound);
        EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED, vertexData->vertexData.componentPortVertex.upperConnectionBound);
    }
    xme_hal_graph_finiVertexIterator(&routes);

    // TODO: Inspect vertex more
}

TEST_F(LogicalRouteManagerInterfaceTestForDCC, getLogicalRoutesWithTwoSubscriptionssOfSameTopic)
{
    // Port 1
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Port 2
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            2,
            42,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    xme_hal_graph_initVertexIterator(&routes);
    {
        xme_hal_graph_vertexId_t vertexId;
        xme_core_pnp_dataLinkGraph_vertexData_t* vertexData;

        // Port 1
        vertexId = xme_hal_graph_nextVertex(&routes);
        xme_hal_graph_getVertexData(&routes, vertexId, (void**)&vertexData);

        EXPECT_EQ(0, vertexData->vertexData.componentPortVertex.lowerConnectionBound);
        EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED, vertexData->vertexData.componentPortVertex.upperConnectionBound);

        // Port 2
        vertexId = xme_hal_graph_nextVertex(&routes);
        xme_hal_graph_getVertexData(&routes, vertexId, (void**)&vertexData);

        EXPECT_EQ(2, vertexData->vertexData.componentPortVertex.lowerConnectionBound);
        EXPECT_EQ(42, vertexData->vertexData.componentPortVertex.upperConnectionBound);
    }
    xme_hal_graph_finiVertexIterator(&routes);

    // TODO: Inspect vertices more
    // TODO: How to make sure that the first announced port matches the first vertex in the iterator?
    //       A different order of vertices would not be _wrong_, but for the testing the vertex data
    //       we have to associate announced ports and vertices in the graph.
    //       For now the assumption announcment order == order in vertex iterator seems to hold
}

TEST_F(LogicalRouteManagerInterfaceTestForDCC, getLogicalRoutesWithTwoSubscriptionssOfDifferentTopic)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    // TODO: Inspect vertices
}

// ----------

TEST_F(LogicalRouteManagerInterfaceTestForDCC, getLogicalRoutesWithPublicationAndSubscriptionOfSameTopicAndNoAttributes)
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            (USHRT_MAX - 2),
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // --------------------------------------------------
    //  Issue the first call to getLogicalRoutes()
    // --------------------------------------------------

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(1U, xme_hal_graph_getEdgeCount(&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi1 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi2 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    {
        if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi1))
        {
            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);
        }
        else if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else
        {
            // Either v1 or v2 must have an outgoing edge
            ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1) || xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    // Check vertex data vd1

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
    EXPECT_EQ(vi1, vd1->vertexId);
    EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID, vd1->vertexData.componentPortVertex.lowerConnectionBound);
    EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID, vd1->vertexData.componentPortVertex.upperConnectionBound);
    EXPECT_EQ(node1, vd1->nodeId);
    EXPECT_EQ(topic1, vd1->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, vd1->vertexData.componentPortVertex.portType);
    // TODO: vd1->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd2

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(0U, vd2->vertexData.componentPortVertex.lowerConnectionBound);
    EXPECT_EQ((USHRT_MAX - 2), vd2->vertexData.componentPortVertex.upperConnectionBound);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->edgeData.logicalRouteEdge.topicId);
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
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(1U, xme_hal_graph_getEdgeCount(&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi1 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi2 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    {
        if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi1))
        {
            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);
        }
        else if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else
        {
            // Either v1 or v2 must have an outgoing edge
            ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1) || xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    // Check vertex data vd1

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
    EXPECT_EQ(vi1, vd1->vertexId);
    EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID, vd1->vertexData.componentPortVertex.lowerConnectionBound);
    EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID, vd1->vertexData.componentPortVertex.upperConnectionBound);
    EXPECT_EQ(node1, vd1->nodeId);
    EXPECT_EQ(topic1, vd1->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, vd1->vertexData.componentPortVertex.portType);
    // TODO: vd1->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd2

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(0U, vd2->vertexData.componentPortVertex.lowerConnectionBound);
    EXPECT_EQ((USHRT_MAX - 2), vd2->vertexData.componentPortVertex.upperConnectionBound);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->edgeData.logicalRouteEdge.topicId);
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
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0U,
            789U,
            topic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(3U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(2U, xme_hal_graph_getEdgeCount(&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi1 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi2 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi3 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi3));
    {
        if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi1))
        {
            // v1 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
            ei2 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        }
        else if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            // v2 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
            ei2 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi3))
        {
            xme_hal_graph_vertexId_t temp;

            // v3 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi3);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
            ei2 = xme_hal_graph_nextOutgoingEdge(&routes, vi3);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));

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
                xme_hal_graph_hasNextOutgoingEdge(&routes, vi1) ||
                xme_hal_graph_hasNextOutgoingEdge(&routes, vi2) ||
                xme_hal_graph_hasNextOutgoingEdge(&routes, vi3)
            );

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi3, (void**)&vd3));
    ASSERT_TRUE(NULL != vd3);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, ei2, (void**)&ed2));
    ASSERT_TRUE(NULL != ed2);

    // Check vertex data vd1

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
    EXPECT_EQ(vi1, vd1->vertexId);
    EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID, vd1->vertexData.componentPortVertex.lowerConnectionBound);
    EXPECT_EQ(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID, vd1->vertexData.componentPortVertex.upperConnectionBound);
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

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(0U, vd2->vertexData.componentPortVertex.lowerConnectionBound);
    EXPECT_EQ((USHRT_MAX - 2), vd2->vertexData.componentPortVertex.upperConnectionBound);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd3

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd3->vertexType);
    EXPECT_EQ(vi3, vd3->vertexId);
    EXPECT_EQ(0U, vd3->vertexData.componentPortVertex.lowerConnectionBound);
    EXPECT_EQ(789U, vd3->vertexData.componentPortVertex.upperConnectionBound);
    EXPECT_EQ(node1, vd3->nodeId);
    EXPECT_EQ(topic1, vd3->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd3->vertexData.componentPortVertex.portType);
    // TODO: vd3->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->edgeData.logicalRouteEdge.topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed1->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed1->edgeData.logicalRouteEdge.established);
    // TODO: ed1->edgeData.logicalRouteEdge (currently not filled with any meaningful data execpt for "channelId" and "established")

    // Check edge data ed2

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed2->edgeType);
    EXPECT_EQ(ei2, ed2->edgeId);
    EXPECT_EQ(topic1, ed2->edgeData.logicalRouteEdge.topicId);
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
    EXPECT_EQ(3U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(2U, xme_hal_graph_getEdgeCount(&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi1 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi2 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi3 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi3));
    {
        if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi1))
        {
            // v1 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
            ei2 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        }
        else if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            // v2 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
            ei2 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi3))
        {
            xme_hal_graph_vertexId_t temp;

            // v3 must be the source
            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi3);

            EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
            ei2 = xme_hal_graph_nextOutgoingEdge(&routes, vi3);

            EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));

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
                xme_hal_graph_hasNextOutgoingEdge(&routes, vi1) ||
                xme_hal_graph_hasNextOutgoingEdge(&routes, vi2) ||
                xme_hal_graph_hasNextOutgoingEdge(&routes, vi3)
            );

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi3, (void**)&vd3));
    ASSERT_TRUE(NULL != vd3);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, ei2, (void**)&ed2));
    ASSERT_TRUE(NULL != ed2);

    // Check vertex data vd1

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
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

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd3

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd3->vertexType);
    EXPECT_EQ(vi3, vd3->vertexId);
    EXPECT_EQ(node1, vd3->nodeId);
    EXPECT_EQ(topic1, vd3->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd3->vertexData.componentPortVertex.portType);
    // TODO: vd3->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->edgeData.logicalRouteEdge.topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed1->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed1->edgeData.logicalRouteEdge.established);
    // TODO: ed1->edgeData.logicalRouteEdge (currently not filled with any meaningful data execpt for "channelId" and "established")

    // Check edge data ed2

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed2->edgeType);
    EXPECT_EQ(ei2, ed2->edgeId);
    EXPECT_EQ(topic1, ed2->edgeData.logicalRouteEdge.topicId);
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
    EXPECT_EQ(3U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(2U, xme_hal_graph_getEdgeCount(&routes));
}

TEST_F(LogicalRouteManagerInterfaceTestForDCC, getLogicalRoutesWithPublicationAndSubscriptionOfSameTopicWithNonMatchingAttributes)
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            topic1,
            attrS,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Call getLogicalRoutes()
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    // Free resources
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(attrS));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(attrP));
}

TEST_F(LogicalRouteManagerInterfaceTestForDCC, getLogicalRoutesWithPublicationAndSubscriptionOfSameTopicWithMatchingAttributes)
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            topic1,
            attrS,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Call getLogicalRoutes()
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(1U, xme_hal_graph_getEdgeCount(&routes));

    // Check vertices and retrieve vertex identifiers

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi1 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
        vi2 = xme_hal_graph_nextVertex(&routes);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        ASSERT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    // Check edges and retrieve edge identifier

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    {
        if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi1))
        {
            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);
        }
        else if (xme_hal_graph_hasNextOutgoingEdge(&routes, vi2))
        {
            xme_hal_graph_vertexId_t temp;

            ei1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);

            // Swap vertex indices such that v1 is the source vertex
            temp = vi1;
            vi1 = vi2;
            vi2 = temp;
        }
        else
        {
            // Either v1 or v2 must have an outgoing edge
            ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1) || xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));

            // This should never be executed, but just in case of inconsistencies, raise an error here
            ASSERT_TRUE(false);
        }

        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

    // Retrieve vertex and edge data

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi1, (void**)&vd1));
    ASSERT_TRUE(NULL != vd1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, vi2, (void**)&vd2));
    ASSERT_TRUE(NULL != vd2);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, ei1, (void**)&ed1));
    ASSERT_TRUE(NULL != ed1);

    // Check vertex data vd1

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd1->vertexType);
    EXPECT_EQ(vi1, vd1->vertexId);
    EXPECT_EQ(node1, vd1->nodeId);
    EXPECT_EQ(topic1, vd1->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, vd1->vertexData.componentPortVertex.portType);
    // TODO: vd1->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check vertex data vd2

    ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd2->vertexType);
    EXPECT_EQ(vi2, vd2->vertexId);
    EXPECT_EQ(node2, vd2->nodeId);
    EXPECT_EQ(topic1, vd2->topicId);
    EXPECT_EQ(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, vd2->vertexData.componentPortVertex.portType);
    // TODO: vd2->vertexData.componentPortVertex (currently not filled with any meaningful data except for "portType")

    // Check edge data ed1

    EXPECT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed1->edgeType);
    EXPECT_EQ(ei1, ed1->edgeId);
    EXPECT_EQ(topic1, ed1->edgeData.logicalRouteEdge.topicId);
    EXPECT_NE(XME_CORE_INVALID_CHANNEL_ID, ed1->edgeData.logicalRouteEdge.channelId);
    EXPECT_FALSE(ed1->edgeData.logicalRouteEdge.established);
    // TODO: ed1->edgeData.logicalRouteEdge (currently not filled with any meaningful data execpt for "channelId" and "established")

    // Commit the logical route

    //ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_setLogicalRoute(ed1->edgeId, ed1->edgeData.logicalRouteEdge.channelId, XME_CORE_INVALID_TRANSACTION_ID));

    // Free resources
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(attrS));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(attrP));
}

//----------------------------------------------------------------------------//
//     LogicalRouteManagerInterfaceTestForRR                                  //
//----------------------------------------------------------------------------//

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneRequestSender)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v1
    // RqS

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(1U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneRequestHandler)
{
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v2
    // RqH

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(1U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneResponseSender)
{
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v3
    // RsS

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(1U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneResponseHandler)
{
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v4
    // RsH

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(1U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneRequestSenderAndOneRequestHandlerWithMatchingTopicAndNoAttributes)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v1        v2
    // RqS       RqH

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneResponseSenderAndOneResponseHandlerWithMatchingTopicAndNoAttributes)
{
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v3        v4
    // RsS       RsH

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneRequestSenderAndOneRequestHandlerWithNonMatchingTopicAndNoAttributes)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v1        v2
    // RqS       RqH

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneResponseSenderAndOneResponseHandlerWithNonMatchingTopicAndNoAttributes)
{
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v3        v4
    // RsS       RsH

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneRequestSenderAndOneResponseHandlerWithNoAttributes)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v1        v4
    // RqS       RsH

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneRequestHandlerAndOneResponseSenderWithNoAttributes)
{
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v2        v3
    // RqH       RsS

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneClientAndOneServerWithMatchingTopicsAndNoAttributes)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //      LR        CM        LR
    //  v1 -----> v2 - - -> v3 -----> v4
    // RqS       RqH       RsS       RsH

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(3U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

    // Check edge between v1 and v2
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi2));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
        edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

        // TODO: Check channel
        //ASSERT_EQ(TODO, edlr->channelId);
        ASSERT_FALSE(edlr->established);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

    // Check edge between v2 and v3
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi3));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t* edcm;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING, ed->edgeType);
        edcm = (xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t*) ed;

        // TODO: Check channels
        //ASSERT_EQ(TODO, edcm->sourceChannelId);
        //ASSERT_EQ(TODO, edcm->destChannelId);
        ASSERT_NE(edcm->sourceChannelId, edcm->destChannelId);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));

    // Check edge between v3 and v4
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi4));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi4);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
        edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

        // TODO: Check channel
        //ASSERT_EQ(TODO, edlr->channelId);
        ASSERT_FALSE(edlr->established);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi3));
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneClientAndOneServerWithNonMatchingRequestTopicAndNoAttributes)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic2,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v1        v2        v3        v4
    // RqS       RqH       RsS       RsH

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneClientAndOneServerWithNonMatchingResponseTopicAndNoAttributes)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  v1        v2        v3        v4
    // RqS       RqH       RsS       RsH

    // TODO: Decide whether vertex count should be zero instead.
    //       Until now, we assumed that if no matches are present, the vertices are still present.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithTwoClientsAndOneServerWithMatchingTopicsAndNoAttributes)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi5 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi6 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //      LR         CM        LR
    //  v1 -----.     .---.      .-----> v4
    // RqS      _\|  /    _\|   /       RsH
    //            v2       _v3
    //      LR  �/|  \ CM  /|   \  LR
    //  v5 -----�     '---'        `-----> v6
    // RqS      RqH       RsS       RsH

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(6U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(6U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                if (comp1 == vdcp->componentId)
                {
                    ASSERT_EQ(node1, vdcp->announcement->nodeId);
                    vi1 = v;
                }
                else if (comp2 == vdcp->componentId)
                {
                    ASSERT_EQ(node1, vdcp->announcement->nodeId);
                    vi5 = v;
                }
                else
                {
                    // Unexpected component ID
                    ASSERT_TRUE(false);
                }
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                if (comp1 == vdcp->componentId)
                {
                    ASSERT_EQ(node1, vdcp->announcement->nodeId);
                    vi4 = v;
                }
                else if (comp2 == vdcp->componentId)
                {
                    ASSERT_EQ(node1, vdcp->announcement->nodeId);
                    vi6 = v;
                }
                else
                {
                    // Unexpected component ID
                    ASSERT_TRUE(false);
                }
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi5);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi6);

    // Check edges between {v1,v5} and v2
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi5));
    {
        xme_hal_graph_edgeId_t e1, e2, e;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi5));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextOutgoingEdge(&routes, vi5);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi2));
        {
            xme_core_pnp_dataLinkGraph_edgeData_t* ed;
            xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

            for (uint16_t i = 0; i < 2; i++)
            {
                ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
                e = xme_hal_graph_nextIncomingEdge(&routes, vi2);

                ed = NULL;
                ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e, (void**) &ed));
                ASSERT_TRUE(NULL != ed);

                ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
                edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

                if (e1 == e)
                {
                    // TODO: Check channel
                    //ASSERT_EQ(TODO, edlr->channelId);
                    ASSERT_FALSE(edlr->established);
                }
                else if (e2 == e)
                {
                    // TODO: Check channel
                    //ASSERT_EQ(TODO, edlr->channelId);
                    ASSERT_FALSE(edlr->established);
                }
                else
                {
                    // Unexpected edge
                    ASSERT_TRUE(false);
                }
            }

            ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
        }
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi2));

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi5));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

    // Check edge between v2 and v3
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi3));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t* edcm;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING, ed->edgeType);
        edcm = (xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t*) ed;

        // TODO: Check channels
        //ASSERT_EQ(TODO, edcm->sourceChannelId);
        //ASSERT_EQ(TODO, edcm->destChannelId);
        ASSERT_NE(edcm->sourceChannelId, edcm->destChannelId);

        // Second CM
        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING, ed->edgeType);
        edcm = (xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t*) ed;

        // TODO: Check channels
        //ASSERT_EQ(TODO, edcm->sourceChannelId);
        //ASSERT_EQ(TODO, edcm->destChannelId);
        ASSERT_NE(edcm->sourceChannelId, edcm->destChannelId);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));

    // Check edges between v3 and {v4,v6}
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi6));
    {
        xme_hal_graph_edgeId_t e1, e2, e;

        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi6));
        e1 = xme_hal_graph_nextIncomingEdge(&routes, vi4);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi6);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi3));
        {
            xme_core_pnp_dataLinkGraph_edgeData_t* ed;
            xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

            for (uint16_t i = 0; i < 2; i++)
            {
                ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
                e = xme_hal_graph_nextOutgoingEdge(&routes, vi3);

                ed = NULL;
                ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e, (void**) &ed));
                ASSERT_TRUE(NULL != ed);

                ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
                edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

                if (e1 == e)
                {
                    // TODO: Check channel
                    //ASSERT_EQ(TODO, edlr->channelId);
                    ASSERT_FALSE(edlr->established);
                }
                else if (e2 == e)
                {
                    // TODO: Check channel
                    //ASSERT_EQ(TODO, edlr->channelId);
                    ASSERT_FALSE(edlr->established);
                }
                else
                {
                    // Unexpected edge
                    ASSERT_TRUE(false);
                }
            }

            ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
        }
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi3));

        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi6));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi6));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi4));
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneClientAndTwoServersWithMatchingTopicsAndNoAttributes)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi5 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi6 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //      LR         CM       LR
    //     .----> v2 - - -> v3 ----.
    //    /      RqH       RsS     _\|
    //  v1                            v4
    //    \ LR         CM       LR �/|
    //     `----> v5 - - -> v6 ----�
    // RqS       RqH       RsS       RsH

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(6U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(6U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                if (comp2 == vdcp->componentId)
                {
                    ASSERT_EQ(node2, vdcp->announcement->nodeId);
                    vi2 = v;
                }
                else if (comp1 == vdcp->componentId)
                {
                    ASSERT_EQ(node2, vdcp->announcement->nodeId);
                    vi5 = v;
                }
                else
                {
                    // Unexpected component ID
                    ASSERT_TRUE(false);
                }
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                if (comp2 == vdcp->componentId)
                {
                    ASSERT_EQ(node2, vdcp->announcement->nodeId);
                    vi3 = v;
                }
                else if (comp1 == vdcp->componentId)
                {
                    ASSERT_EQ(node2, vdcp->announcement->nodeId);
                    vi6 = v;
                }
                else
                {
                    // Unexpected component ID
                    ASSERT_TRUE(false);
                }
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi5);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi6);

    // Check edges between v1 and {v2,v5}
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi5));
    {
        xme_hal_graph_edgeId_t e1, e2, e;

        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi5));
        e1 = xme_hal_graph_nextIncomingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi5);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
        {
            xme_core_pnp_dataLinkGraph_edgeData_t* ed;
            xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

            for (uint16_t i = 0; i < 2; i++)
            {
                ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
                e = xme_hal_graph_nextOutgoingEdge(&routes, vi1);

                edlr = NULL;
                ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e, (void**) &ed));
                ASSERT_TRUE(NULL != ed);

                ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
                edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

                if (e1 == e)
                {
                    // TODO: Check channel
                    //ASSERT_EQ(TODO, edlr->channelId);
                    ASSERT_FALSE(edlr->established);
                }
                else if (e2 == e)
                {
                    // TODO: Check channel
                    //ASSERT_EQ(TODO, edlr->channelId);
                    ASSERT_FALSE(edlr->established);
                }
                else
                {
                    // Unexpected edge
                    ASSERT_TRUE(false);
                }
            }

            ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        }
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi5));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi2));

    // Check edge between v2 and v3
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi3));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t* edcm;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING, ed->edgeType);
        edcm = (xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t*) ed;

        // TODO: Check channels
        //ASSERT_EQ(TODO, edcm->sourceChannelId);
        //ASSERT_EQ(TODO, edcm->destChannelId);
        ASSERT_NE(edcm->sourceChannelId, edcm->destChannelId);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));

    // Check edges between {v3,v6} and v5
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi6));
    {
        xme_hal_graph_edgeId_t e1, e2, e;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi6));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextOutgoingEdge(&routes, vi6);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi4));
        {
            xme_core_pnp_dataLinkGraph_edgeData_t* ed;
            xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

            for (uint16_t i = 0; i < 2; i++)
            {
                ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
                e = xme_hal_graph_nextIncomingEdge(&routes, vi4);

                ed = NULL;
                ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e, (void**) &ed));
                ASSERT_TRUE(NULL != ed);

                ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
                edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

                if (e1 == e)
                {
                    // TODO: Check channel
                    //ASSERT_EQ(TODO, edlr->channelId);
                    ASSERT_FALSE(edlr->established);
                }
                else if (e2 == e)
                {
                    // TODO: Check channel
                    //ASSERT_EQ(TODO, edlr->channelId);
                    ASSERT_FALSE(edlr->established);
                }
                else
                {
                    // Unexpected edge
                    ASSERT_TRUE(false);
                }
            }

            ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
        }
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi4));

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi6));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi6));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi3));
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneRequestPublisherOneRequestProcessorAndOneDifferentResponseSubscriberWithMatchingTopicsAndNoAttributes)
{
    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node3,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //  RqS      RqH       RsS       RsH
    //  v1        v2        v3       v4
    // (n1)      (n2)      (n2)     (n3)

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneClientAndOneServerAndIncrementalAnnouncementFirstRequestThenResponse)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected INTERMEDIATE logical route graph:
    //
    //  v1        v2
    // RqS       RqH

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //      LR        CM        LR
    //  v1 -----> v2 - - -> v3 -----> v4
    // RqS       RqH       RsS       RsH

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(3U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

    // Check edge between v1 and v2
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi2));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
        edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

        // TODO: Check channel
        //ASSERT_EQ(TODO, edlr->channelId);
        ASSERT_FALSE(edlr->established);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

    // Check edge between v2 and v3
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi3));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t* edcm;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING, ed->edgeType);
        edcm = (xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t*) ed;

        // TODO: Check channels
        //ASSERT_EQ(TODO, edcm->sourceChannelId);
        //ASSERT_EQ(TODO, edcm->destChannelId);
        ASSERT_NE(edcm->sourceChannelId, edcm->destChannelId);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));

    // Check edge between v3 and v4
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi4));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi4);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
        edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

        // TODO: Check channel
        //ASSERT_EQ(TODO, edlr->channelId);
        ASSERT_FALSE(edlr->established);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi3));
}

TEST_F(LogicalRouteManagerInterfaceTestForRR, getLogicalRoutesWithOneClientAndOneServerAndIncrementalAnnouncementFirstResponseThenRequest)
{
    xme_hal_graph_vertexId_t vi1 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi2 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi3 = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    xme_hal_graph_vertexId_t vi4 = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node2,
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            responseTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected INTERMEDIATE logical route graph:
    //
    //  v3        v4
    // RsS       RsH

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

    ASSERT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            comp1,
            compType1,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
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
            comp2,
            compType2,
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            requestTopic1,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Expected logical route graph:
    //
    //      LR        CM        LR
    //  v1 -----> v2 - - -> v3 -----> v4
    // RqS       RqH       RsS       RsH

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(3U, xme_hal_graph_getEdgeCount(&routes));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&routes));
    {
        xme_hal_graph_vertexId_t v;
        xme_core_pnp_dataLinkGraph_vertexData_t* vd;
        xme_core_pnp_dataLinkGraph_componentPortVertexData_t* vdcp;

        // Check vertices
        for (uint16_t i = 0U; i < xme_hal_graph_getVertexCount(&routes); i++)
        {
            ASSERT_TRUE(xme_hal_graph_hasNextVertex(&routes));
            v = xme_hal_graph_nextVertex(&routes);
            ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);

            vd = NULL;
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&routes, v, (void**) &vd));
            ASSERT_TRUE(NULL != vd);

            ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, vd->vertexType);
            vdcp = (xme_core_pnp_dataLinkGraph_componentPortVertexData_t*) &vd->vertexData.componentPortVertex;

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi1 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi2 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == vdcp->portType)
            {
                ASSERT_EQ(comp2, vdcp->componentId);
                ASSERT_EQ(node2, vdcp->announcement->nodeId);
                vi3 = v;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vdcp->portType)
            {
                ASSERT_EQ(comp1, vdcp->componentId);
                ASSERT_EQ(node1, vdcp->announcement->nodeId);
                vi4 = v;
            }
            else
            {
                // Unexpected port type
                ASSERT_TRUE(false);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&routes));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&routes));

    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

    // Check edge between v1 and v2
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi2));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi1);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
        edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

        // TODO: Check channel
        //ASSERT_EQ(TODO, edlr->channelId);
        ASSERT_FALSE(edlr->established);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi1));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi2));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi1));

    // Check edge between v2 and v3
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi3));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t* edcm;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi2);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING, ed->edgeType);
        edcm = (xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t*) ed;

        // TODO: Check channels
        //ASSERT_EQ(TODO, edcm->sourceChannelId);
        //ASSERT_EQ(TODO, edcm->destChannelId);
        ASSERT_NE(edcm->sourceChannelId, edcm->destChannelId);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi2));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi3));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi2));

    // Check edge between v3 and v4
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&routes, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&routes, vi4));
    {
        xme_hal_graph_edgeId_t e1, e2;
        xme_core_pnp_dataLinkGraph_edgeData_t* ed;
        xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t* edlr;

        ASSERT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
        ASSERT_TRUE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
        e1 = xme_hal_graph_nextOutgoingEdge(&routes, vi3);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e1);
        e2 = xme_hal_graph_nextIncomingEdge(&routes, vi4);
        ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, e2);
        ASSERT_EQ(e1, e2);

        ed = NULL;
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&routes, e1, (void**) &ed));
        ASSERT_TRUE(NULL != ed);

        ASSERT_EQ(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ed->edgeType);
        edlr = (xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t*) &ed->edgeData.logicalRouteEdge;

        // TODO: Check channel
        //ASSERT_EQ(TODO, edlr->channelId);
        ASSERT_FALSE(edlr->established);

        ASSERT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&routes, vi3));
        ASSERT_FALSE(xme_hal_graph_hasNextIncomingEdge(&routes, vi4));
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&routes, vi4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&routes, vi3));
}

//----------------------------------------------------------------------------//
//     LogicalRouteManagerInterfaceTestForRACEUseCase                         //
//----------------------------------------------------------------------------//

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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            2U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(3U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(2U, xme_hal_graph_getEdgeCount(&routes));
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            2U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(3U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(2U, xme_hal_graph_getEdgeCount(&routes));
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            2U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            3U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic2,
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(4U, xme_hal_graph_getEdgeCount(&routes));
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
            steeringWheelBSensorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            steeringWheelRSensorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            steeringBoxBSensorActuatorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            steeringBoxRSensorActuatorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            brakePedalSensorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            wheelRotFrequencyFLSensorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            wheelRotFrequencyFRSensorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            wheelRotFrequencyBLSensorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            wheelRotFrequencyBRSensorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            accelerationPedalSensorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            steeringAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            2U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            brakeAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            brakeAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            brakeAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            brakeAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            3U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            brakeAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            4U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            brakeAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            5U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            brakeAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            6U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            accelerationAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            accelerationAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            accelerationAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            2U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            accelerationAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            3U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            accelerationAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            4U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            accelerationAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            5U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            accelerationAppComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            6U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
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
            steeringBoxBSensorActuatorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            steeringBoxRSensorActuatorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            brakeFLActuatorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            brakeFRActuatorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            inverterBLActuatorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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
            inverterBRActuatorComponentType,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
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

TEST_F(LogicalRouteManagerInterfaceTestForRACEUseCaseAndAttributes, RacePipecleanerExample1)
{
    xme_core_directory_attributeSetHandle_t attrP1, attrP2;
    xme_core_directory_attributeSetHandle_t attrS1, attrS2;
    uint16_t attributeValue1 = 31;
    uint16_t attributeValue2 = 32;

    // Create publication attribute set (blank)
    attrP1 = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrP1);

    // Create publication attribute set (blank)
    attrP2 = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrP2);

    // Create subscription attribute set (numeric '==' attribute filter)
    attrS1 = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrS1);

    // Create subscription attribute set (numeric '==' attribute filter)
    attrS2 = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrS2);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeDefinition(
            attrP1,
            (xme_core_attribute_key_t) 1,
            &attributeValue1,
            1U,
            sizeof(attributeValue1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
            false
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeDefinition(
            attrP2,
            (xme_core_attribute_key_t) 1,
            &attributeValue2,
            1U,
            sizeof(attributeValue2),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
            false
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeFilter(
            attrS1,
            (xme_core_attribute_key_t) 1,
            &attributeValue1,
            1U,
            sizeof(attributeValue1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeFilter(
            attrS2,
            (xme_core_attribute_key_t) 1,
            &attributeValue2,
            1U,
            sizeof(attributeValue2),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // NODE 3 (Wheel Speed Sensor)
    // Publication with attribute 'wheel speed rotation' set to 31
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node3,
            component4,
            component4Type,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic4232,
            attrP1,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE 3 (Wheel Speed Sensor)
    // Publication with attribute 'wheel speed rotation' set to 32
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node3,
            component4,
            component4Type,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic4232,
            attrP2,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE 1 (Brake App)
    // Subscription with attribute 'wheel speed rotation' set to 31
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            component100,
            component100Type,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            topic4232,
            attrS1,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE 1 (Wheel Speed Sensor)
    // Subscription with attribute 'wheel speed rotation' set to 32
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            component100,
            component100Type,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            topic4232,
            attrS2,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Call getLogicalRoutes()
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(2U, xme_hal_graph_getEdgeCount(&routes));
}

TEST_F(LogicalRouteManagerInterfaceTestForRACEUseCaseAndAttributes, RacePipecleanerExample)
{
    xme_core_directory_attributeSetHandle_t attrP1, attrP2;
    xme_core_directory_attributeSetHandle_t attrS1, attrS2;
    uint16_t attributeValue1 = 35;
    uint16_t attributeValue2 = 36;

    // Create publication attribute set (blank)
    attrP1 = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrP1);

    // Create publication attribute set (blank)
    attrP2 = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrP2);

    // Create subscription attribute set (numeric '==' attribute filter)
    attrS1 = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrS1);

    // Create subscription attribute set (numeric '==' attribute filter)
    attrS2 = xme_core_directory_attribute_createAttributeSet();
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, attrS2);

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeDefinition(
            attrP1,
            (xme_core_attribute_key_t) 1,
            &attributeValue1,
            1U,
            sizeof(attributeValue1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
            false
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeDefinition(
            attrP2,
            (xme_core_attribute_key_t) 1,
            &attributeValue2,
            1U,
            sizeof(attributeValue2),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
            false
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeFilter(
            attrS1,
            (xme_core_attribute_key_t) 1,
            &attributeValue1,
            1U,
            sizeof(attributeValue1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeFilter(
            attrS2,
            (xme_core_attribute_key_t) 1,
            &attributeValue2,
            1U,
            sizeof(attributeValue2),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // NODE 3 (Wheel Speed Sensor)
    // Publication with attribute 'wheel speed rotation' set to 31
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            component100,
            component100Type,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            0U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic4108,
            attrP1,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE 3 (Wheel Speed Sensor)
    // Publication with attribute 'wheel speed rotation' set to 32
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node1,
            component100,
            component100Type,
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
            1U,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            topic4108,
            attrP2,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE 1 (Brake App)
    // Subscription with attribute 'wheel speed rotation' set to 31
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node3,
            component4,
            component4Type,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            0U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            topic4108,
            attrS1,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // NODE 1 (Wheel Speed Sensor)
    // Subscription with attribute 'wheel speed rotation' set to 32
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_announcePort
        (
            node3,
            component5,
            component5Type,
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
            1U,
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            topic4108,
            attrS2,
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Call getLogicalRoutes()
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &routes));
    EXPECT_EQ(4U, xme_hal_graph_getVertexCount(&routes));
    EXPECT_EQ(2U, xme_hal_graph_getEdgeCount(&routes));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
