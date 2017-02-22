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
 * $Id: smokeTestLogicalRouteManager.cpp 6079 2013-12-12 17:17:24Z wiesmueller $
 */

/**
 * \file
 *         Logical Route Manager smoke tests.
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

class LogicalRouteManagerSmokeTest: public ::testing::Test
{
protected:
    LogicalRouteManagerSmokeTest()
    {

    }

    virtual ~LogicalRouteManagerSmokeTest()
    {

    }

};

/******************************************************************************/
/***   Helper Functions                                                     ***/
/******************************************************************************/

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     LogicalRouteManagerSmokeTest                                       //
//----------------------------------------------------------------------------//

TEST_F(LogicalRouteManagerSmokeTest, SimpleInitAndFini)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
    xme_core_pnp_lrm_fini();
}

TEST_F(LogicalRouteManagerSmokeTest, AnnouncePorts)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

    // Internal Port: XME_STATUS_INVALID_PARAMETER
    // FIXME: Should internal ports be announced in LRM? If not, we should document properly the function. 
    EXPECT_EQ(
        XME_STATUS_INVALID_PARAMETER, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_INTERNAL, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // DCC Publication
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
            1, 
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // DCC Subscription
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
            2, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Request-Response: Request Sender
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER, 
            3, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Request-Response: Request Handler
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER, 
            4, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Request-Response: Response Sender
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER, 
            5, 
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Request-Response: Response Handler
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER, 
            6, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    xme_core_pnp_lrm_fini();
}

TEST_F(LogicalRouteManagerSmokeTest, AnnounceAndUnannouncePortForDCCPublication)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
            0, 
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Unannounce port. 
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_unannouncePortForComponent((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // End the logical route manager. 
    xme_core_pnp_lrm_fini();
}

TEST_F(LogicalRouteManagerSmokeTest, AnnounceAndUnannouncePortForDCCSubscription)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
            0, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Unannounce port. 
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_unannouncePortForComponent((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // End the logical route manager. 
    xme_core_pnp_lrm_fini();
}

TEST_F(LogicalRouteManagerSmokeTest, AnnounceAndUnannouncePortForRRRequest)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

    // Request Sender
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER, 
            0, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Request Handler
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER, 
            1, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Unannounce ports. 
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_unannouncePortForComponent((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // End the logical route manager. 
    xme_core_pnp_lrm_fini();
}

TEST_F(LogicalRouteManagerSmokeTest, AnnounceAndUnannouncePortForRRResponse)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

    // Response Sender
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER, 
            0, 
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Response Handler
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER, 
            1, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Unannounce ports. 
    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_unannouncePortForComponent((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // End the logical route manager. 
    xme_core_pnp_lrm_fini();
}

TEST_F(LogicalRouteManagerSmokeTest, CreateAndEstablishOneLogicalRoute)
{
    xme_core_pnp_lrm_logicalRoutes_t logicalRoutes;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
            0, 
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)2, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_MONITOR, 
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
            0, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Init the logical routes graph. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&logicalRoutes));

    // Get logical routes.
    ASSERT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &logicalRoutes)
    );

    // Check that we have exactly one route with two vertices. 
    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&logicalRoutes));
    EXPECT_EQ(1U, xme_hal_graph_getEdgeCount(&logicalRoutes));

    // Get the routes associated for component 1 and 2, and check that the returing graph is empty. 
    ASSERT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getEstablishedLogicalRoutesForComponent(XME_CORE_INVALID_TRANSACTION_ID, 
            (xme_core_node_nodeId_t) 1, 
            (xme_core_component_t) 1, 
            &logicalRoutes)
    );

    EXPECT_EQ(0U, xme_hal_graph_getVertexCount(&logicalRoutes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&logicalRoutes));

    ASSERT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getEstablishedLogicalRoutesForComponent(XME_CORE_INVALID_TRANSACTION_ID, 
            (xme_core_node_nodeId_t) 2, 
            (xme_core_component_t) 1, 
            &logicalRoutes)
    );

    EXPECT_EQ(0U, xme_hal_graph_getVertexCount(&logicalRoutes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&logicalRoutes));

    // Establish the routes.
    // TODO: This method is not meaningful. We should think about one
    //       generic method for establishing all the routes for a given 
    //       logical route graph. 
    EXPECT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_setLogicalRoute((xme_hal_graph_edgeId_t) 1, 
            (xme_core_channelId_t) 4097, 
            XME_CORE_INVALID_TRANSACTION_ID)
    );

    // Get the routes for node 1 and 2. 
    ASSERT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getEstablishedLogicalRoutesForComponent(XME_CORE_INVALID_TRANSACTION_ID, 
            (xme_core_node_nodeId_t) 1, 
            (xme_core_component_t) 1, 
            &logicalRoutes)
    );

    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&logicalRoutes));
    EXPECT_EQ(1U, xme_hal_graph_getEdgeCount(&logicalRoutes));

    ASSERT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getEstablishedLogicalRoutesForComponent(XME_CORE_INVALID_TRANSACTION_ID, 
            (xme_core_node_nodeId_t) 2, 
            (xme_core_component_t) 1, 
            &logicalRoutes)
    );

    EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&logicalRoutes));
    EXPECT_EQ(1U, xme_hal_graph_getEdgeCount(&logicalRoutes));

    // Delete the route.
    ASSERT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_removeLogicalRoute((xme_core_channelId_t) 4097,
            XME_CORE_INVALID_TRANSACTION_ID)
    );

    // Get again the routes for node 1 and 2. 
    ASSERT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getEstablishedLogicalRoutesForComponent(XME_CORE_INVALID_TRANSACTION_ID, 
            (xme_core_node_nodeId_t) 1, 
            (xme_core_component_t) 1, 
            &logicalRoutes)
    );

    EXPECT_EQ(0U, xme_hal_graph_getVertexCount(&logicalRoutes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&logicalRoutes));

    ASSERT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getEstablishedLogicalRoutesForComponent(XME_CORE_INVALID_TRANSACTION_ID, 
            (xme_core_node_nodeId_t) 2, 
            (xme_core_component_t) 1, 
            &logicalRoutes)
    );

    EXPECT_EQ(0U, xme_hal_graph_getVertexCount(&logicalRoutes));
    EXPECT_EQ(0U, xme_hal_graph_getEdgeCount(&logicalRoutes));

    // Finish the logical routes graph. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&logicalRoutes));

    // End the logical route manager. 
    xme_core_pnp_lrm_fini();
}

TEST_F(LogicalRouteManagerSmokeTest, GetRoutesAssociatedToAnnouncementWithOnePublicationAndTwoSubscriptions)
{
    xme_core_pnp_lrm_logicalRoutes_t logicalRoutes;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)1, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_SENSOR, 
            XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
            0, 
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)2, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_MONITOR, 
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
            0, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    EXPECT_EQ(
        XME_STATUS_SUCCESS, 
        xme_core_pnp_lrm_announcePort((xme_core_node_nodeId_t)3, 
            (xme_core_component_t)1, 
            XME_CORE_COMPONENT_TYPE_MONITOR, 
            XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
            0, 
            0,
            XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
            XME_CORE_TOPIC_GENERAL_COMMON, 
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            XME_CORE_INVALID_TRANSACTION_ID
        )
    );

    // Init the logical routes graph. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&logicalRoutes));

    // Get logical routes.
    ASSERT_EQ(
        XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &logicalRoutes)
    );

    // Check that we have exactly one route with two vertices. 
    EXPECT_EQ(3U, xme_hal_graph_getVertexCount(&logicalRoutes));
    EXPECT_EQ(2U, xme_hal_graph_getEdgeCount(&logicalRoutes));

    // Explore the obtained graph. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&logicalRoutes));

    while(xme_hal_graph_hasNextVertex(&logicalRoutes))
    {
        xme_hal_graph_vertexId_t vertexID;
        xme_core_pnp_dataLinkGraph_vertexData_t* vertexData;
        xme_core_pnp_lrm_logicalRoutes_t associatedRoutes;
        
        vertexID = xme_hal_graph_nextVertex(&logicalRoutes);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&logicalRoutes, vertexID, (void**)&vertexData));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&associatedRoutes));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_getAssociatedVertices(vertexData->vertexData.componentPortVertex.announcement, &associatedRoutes));

        if (vertexData->vertexData.componentPortVertex.portType == XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION)
        {
            EXPECT_EQ(3U, xme_hal_graph_getVertexCount(&associatedRoutes));
            EXPECT_EQ(2U, xme_hal_graph_getEdgeCount(&associatedRoutes));
        }
        else if (vertexData->vertexData.componentPortVertex.portType == XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION)
        {
            EXPECT_EQ(2U, xme_hal_graph_getVertexCount(&associatedRoutes));
            EXPECT_EQ(1U, xme_hal_graph_getEdgeCount(&associatedRoutes));
        }

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&associatedRoutes));
    }

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&logicalRoutes));

    // Finish the logical routes graph. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&logicalRoutes));

    // End the logical route manager. 
    xme_core_pnp_lrm_fini();
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
