/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: networkConfigurationCalculator.c 5364 2013-10-01 17:19:56Z wiesmueller $
 */

/**
 * \file
 *         Network Configuration Calculator.
 */

/**
 * \addtogroup core_pnp_ncc
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/networkConfigurationCalculator.h"

#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/linkedList.h"

#include <inttypes.h>
/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Creates a link key for UDP waypoints.
 *
 * \param[in] topicId the topic id. 
 * \param[out] bArray the output array. 
 */
static void
xme_core_pnp_ncc_createLinkKey
(
    xme_core_topic_t topicId,
    uint8_t* bArray
);

/**
 * \brief Gets the physical route described by the input logical route.
 *
 * \param logicalRouteGraph the logical route graph. 
 * \param physicalRouteGraph the physical route corresponding to the provided logical route. 
 *
 * \retval XME_STATUS_SUCCESS if the data link graph is transformed by the network
 *         configurator calculator. 
 */
static xme_status_t
xme_core_pnp_ncc_createPhysicalRoutes
(
    xme_hal_graph_graph_t* logicalRouteGraph,
    xme_core_pnp_ncc_physicalRoutes_t* physicalRouteGraph
);

/**
 * \brief Provides the callback for comparison of vertex data.
 * \param vertexData1 The first vertex data. 
 * \param vertexData2 The second vertex data. 
 * \return 0 if the vertex data are equal and non zero in other case. 
 */
int 
xme_core_pnp_ncc_vertexCompareCallback
(
    void* vertexData1,
    void* vertexData2
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

static void
xme_core_pnp_ncc_createLinkKey
(
    xme_core_topic_t topicId,
    uint8_t* bArray
)
{
    int i=0;

    bArray[i++] = 0;
    bArray[i++] = 0;
    bArray[i++] = (uint8_t)(topicId >> 8);
    bArray[i]   = (uint8_t) topicId;
}

static xme_status_t
xme_core_pnp_ncc_createPhysicalRoutes
(
    xme_hal_graph_graph_t* logicalRouteGraph,
    xme_core_pnp_ncc_physicalRoutes_t* physicalRouteGraph
)
{
    xme_core_pnp_dataLinkGraph_vertexData_t* sourceVertex;
    xme_core_pnp_dataLinkGraph_vertexData_t* sinkVertex;

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_init((xme_hal_graph_graph_t*)physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_setVertexCompareCallback(physicalRouteGraph, xme_core_pnp_ncc_vertexCompareCallback), XME_STATUS_INTERNAL_ERROR);

    // Iterate over logical routes
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initVertexIterator(logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
    while (xme_hal_graph_hasNextVertex(logicalRouteGraph))
    {
        xme_hal_graph_vertexId_t sourceVertexId = xme_hal_graph_nextVertex(logicalRouteGraph);
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(logicalRouteGraph, sourceVertexId, (void**) &sourceVertex), XME_STATUS_INTERNAL_ERROR);
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initOutgoingEdgeIterator(logicalRouteGraph, sourceVertexId), XME_STATUS_INTERNAL_ERROR);
        while(xme_hal_graph_hasNextOutgoingEdge(logicalRouteGraph,sourceVertexId))
        {
            xme_core_pnp_dataLinkGraph_edgeData_t* edgeData = NULL;
            xme_hal_graph_vertexId_t sinkVertexId;
            xme_core_topic_t topicId;

            xme_hal_graph_edgeId_t tempEdgeId = xme_hal_graph_nextOutgoingEdge(logicalRouteGraph, sourceVertexId);
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(logicalRouteGraph, tempEdgeId, (void**) &edgeData), XME_STATUS_INTERNAL_ERROR);

            // Ignore everything other than logical routes
            XME_CHECK(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType, XME_STATUS_INVALID_CONFIGURATION);

            // If this is already existing infrastructure we don't need to process it
            if (edgeData->edgeData.logicalRouteEdge.established)
            {
                continue;
            }

            sinkVertexId = xme_hal_graph_getSinkVertex(logicalRouteGraph, tempEdgeId);
            topicId = edgeData->topicId;
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(logicalRouteGraph, sinkVertexId, (void**) &sinkVertex), XME_STATUS_INTERNAL_ERROR);

            if (sourceVertex->nodeId != sinkVertex->nodeId)
            {
                // Source and destination are on different nodes.
                // Hence, there is a need for marshaller/udpSend/udpReceive/demarshaller waypoints.
                uint8_t bArray[4];

                xme_core_pnp_dataLinkGraph_vertexData_t* sourceComponentVertexData;
                xme_core_pnp_dataLinkGraph_vertexData_t* marshallerWaypointVertexData;
                xme_core_pnp_dataLinkGraph_vertexData_t* udpSendWaypointVertexData;
                xme_core_pnp_dataLinkGraph_vertexData_t* udpReceiveWaypointVertexData;
                xme_core_pnp_dataLinkGraph_vertexData_t* demarshallerWaypointVertexData;
                xme_core_pnp_dataLinkGraph_vertexData_t* sinkComponentVertexData;
                xme_core_pnp_dataLinkGraph_edgeData_t* memcopyEdgeData;
                xme_core_pnp_dataLinkGraph_edgeData_t* udpLinkEdgeData;
                xme_com_interface_address_t* nodeIPport = NULL;

                xme_hal_graph_vertexId_t vertexId;

                // Get the UDP destination address
                if (XME_STATUS_SUCCESS != xme_core_directory_nodeRegistryController_getInterface(sinkVertex->nodeId, XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, &nodeIPport))
                {
                    // No UDP network interface feasible
                    XME_LOG
                    (
                        XME_LOG_NOTE, 
                        "[NetworkConfigurationCalculator] No UDP network interface address associated to the node %" PRIu16 ". \n", 
                        sinkVertex->nodeId
                    );
                    continue;
                }

                // Create the common link key
                xme_core_pnp_ncc_createLinkKey(topicId, bArray);

                // Create and add the source vertex in physical route graph.
                sourceComponentVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != sourceComponentVertexData);
                sourceComponentVertexData->vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;
                sourceComponentVertexData->nodeId = sourceVertex->nodeId;
                sourceComponentVertexData->topicId = topicId;
                sourceComponentVertexData->vertexData.componentPortVertex.portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                sourceComponentVertexData->vertexData.componentPortVertex.componentId = sourceVertex->vertexData.componentPortVertex.componentId;
                
                // Check if that component port vertex already exists in the graph. 
                vertexId = xme_hal_graph_getNextVertexWithDataComparison(physicalRouteGraph, sourceComponentVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                if (XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId)
                {
                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**)&sourceComponentVertexData), XME_STATUS_INTERNAL_ERROR);
                    sourceComponentVertexData->vertexId = vertexId;
                }
                else
                {
                    sourceComponentVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, sourceComponentVertexData);
                }
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != sourceComponentVertexData->vertexId);

                // Create and add the marshaler waypoint vertex in physical route graph.
                marshallerWaypointVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != marshallerWaypointVertexData);
                marshallerWaypointVertexData->vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER;
                marshallerWaypointVertexData->nodeId = sourceVertex->nodeId;
                marshallerWaypointVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, marshallerWaypointVertexData);
                marshallerWaypointVertexData->topicId = topicId;
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != marshallerWaypointVertexData->vertexId);

                // Create and add the memcopy edge between source and marshaling waypoint to the physical route graph. 
                memcopyEdgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_edgeData_t));
                XME_ASSERT(NULL != memcopyEdgeData);
                memcopyEdgeData->edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY;
                memcopyEdgeData->topicId = topicId; 
                memcopyEdgeData->edgeId = xme_hal_graph_addEdge(physicalRouteGraph, sourceComponentVertexData->vertexId, marshallerWaypointVertexData->vertexId, memcopyEdgeData);
                XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != memcopyEdgeData->edgeId);

                // Create and add the udpSend waypoint vertex
                udpSendWaypointVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != udpSendWaypointVertexData);
                udpSendWaypointVertexData->vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND;
                udpSendWaypointVertexData->nodeId = sourceVertex->nodeId;
                udpSendWaypointVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, udpSendWaypointVertexData);
                udpSendWaypointVertexData->topicId = topicId;
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != udpSendWaypointVertexData->vertexId);

                (void) xme_hal_mem_copy(
                    udpSendWaypointVertexData->vertexData.waypointUdpSendVertex.key, 
                    bArray, 
                    XME_WP_UDP_HEADER_KEY_LENGTH
                );
                (void) xme_hal_mem_copy(
                    (void*) &(udpSendWaypointVertexData->vertexData.waypointUdpSendVertex.destination), 
                    (void*) nodeIPport, 
                    sizeof(xme_com_interface_address_t)
                );

                // Create and add the memcopy edge between marshaling waypoint and UDP send waypoint
                memcopyEdgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_edgeData_t));
                XME_ASSERT(NULL != memcopyEdgeData);
                memcopyEdgeData->edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY;
                memcopyEdgeData->topicId = topicId;
                memcopyEdgeData->edgeId = xme_hal_graph_addEdge(physicalRouteGraph, marshallerWaypointVertexData->vertexId, udpSendWaypointVertexData->vertexId, memcopyEdgeData);
                XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != memcopyEdgeData->edgeId);

                // Create the udpRecv waypoint vertex
                udpReceiveWaypointVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != udpReceiveWaypointVertexData);
                udpReceiveWaypointVertexData->vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE;
                udpReceiveWaypointVertexData->nodeId = sinkVertex->nodeId;
                udpReceiveWaypointVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, udpReceiveWaypointVertexData);
                udpReceiveWaypointVertexData->topicId = topicId;
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != udpReceiveWaypointVertexData->vertexId);

                // Now this is UDP Receive we just need to listen to our port so bascially we already have the destination in nodeIPport
                (void) xme_hal_mem_copy(
                    udpReceiveWaypointVertexData->vertexData.waypointUdpReceiveVertex.key, 
                    bArray, 
                    XME_WP_UDP_HEADER_KEY_LENGTH
                );
                (void) xme_hal_mem_copy(
                    (void*) &(udpReceiveWaypointVertexData->vertexData.waypointUdpReceiveVertex.host), 
                    (void*) nodeIPport, 
                    sizeof(xme_com_interface_address_t)
                );

                // Create the udpLink edge between UDP send waypoint and UDP receive waypoint
                udpLinkEdgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_edgeData_t));
                XME_ASSERT(NULL != udpLinkEdgeData);
                udpLinkEdgeData->edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK;
                udpLinkEdgeData->topicId = topicId;
                udpLinkEdgeData->edgeId = xme_hal_graph_addEdge(physicalRouteGraph, udpSendWaypointVertexData->vertexId, udpReceiveWaypointVertexData->vertexId, udpLinkEdgeData);
                XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != udpLinkEdgeData->edgeId);

                // Set the communication key
                (void) xme_hal_mem_copy(
                    udpLinkEdgeData->edgeData.udpLinkEdge.key, 
                    bArray, 
                    XME_WP_UDP_HEADER_KEY_LENGTH
                );

                // Create and add the demarshallar waypoint vertex
                demarshallerWaypointVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != demarshallerWaypointVertexData);
                demarshallerWaypointVertexData->vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER;
                demarshallerWaypointVertexData->nodeId = sinkVertex->nodeId;
                demarshallerWaypointVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, demarshallerWaypointVertexData);
                demarshallerWaypointVertexData->topicId = topicId;
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != demarshallerWaypointVertexData->vertexId);

                // Create and add the memcopy edge between UDP receive waypoint and demarshaler waypoint
                memcopyEdgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_edgeData_t));
                XME_ASSERT(NULL != memcopyEdgeData);
                memcopyEdgeData->edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY;
                memcopyEdgeData->topicId = topicId;
                memcopyEdgeData->edgeId = xme_hal_graph_addEdge(physicalRouteGraph, udpReceiveWaypointVertexData->vertexId, demarshallerWaypointVertexData->vertexId, memcopyEdgeData);
                XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != memcopyEdgeData->edgeId);

                // Create and add the sink vertex in physical route graph.
                sinkComponentVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != sinkComponentVertexData);
                sinkComponentVertexData->vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;
                sinkComponentVertexData->nodeId = sinkVertex->nodeId;
                sinkComponentVertexData->topicId = topicId;
                sinkComponentVertexData->vertexData.componentPortVertex.portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                sinkComponentVertexData->vertexData.componentPortVertex.componentId = sinkVertex->vertexData.componentPortVertex.componentId;

                // Check if that component port vertex already exists in the graph. 
                vertexId = xme_hal_graph_getNextVertexWithDataComparison(physicalRouteGraph, sinkComponentVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                if (XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId)
                {
                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**)&sinkComponentVertexData), XME_STATUS_INTERNAL_ERROR);
                    sinkComponentVertexData->vertexId = vertexId;
                }
                else
                {
                    sinkComponentVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, sinkComponentVertexData);
                }
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != sinkComponentVertexData->vertexId);

                // Create and add the memcopy edge between demarshaler waypoint and destination
                memcopyEdgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_edgeData_t));
                XME_ASSERT(NULL != memcopyEdgeData);
                memcopyEdgeData->edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY;
                memcopyEdgeData->topicId = topicId;
                memcopyEdgeData->edgeId = xme_hal_graph_addEdge(physicalRouteGraph, demarshallerWaypointVertexData->vertexId, sinkComponentVertexData->vertexId, memcopyEdgeData);
                XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != memcopyEdgeData->edgeId);
            }
            else
            {
                xme_core_pnp_dataLinkGraph_vertexData_t* sourceComponentVertexData;
                xme_core_pnp_dataLinkGraph_vertexData_t* sinkComponentVertexData;
                xme_core_pnp_dataLinkGraph_edgeData_t* memcopyEdgeData;

                xme_hal_graph_vertexId_t vertexId;

                // Create and add the source vertex in physical route graph.
                sourceComponentVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != sourceComponentVertexData);
                sourceComponentVertexData->vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;
                sourceComponentVertexData->nodeId = sourceVertex->nodeId;
                sourceComponentVertexData->topicId = topicId;
                sourceComponentVertexData->vertexData.componentPortVertex.portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                sourceComponentVertexData->vertexData.componentPortVertex.componentId = sourceVertex->vertexData.componentPortVertex.componentId;

                // Check if that component port vertex already exists in the graph. 
                vertexId = xme_hal_graph_getNextVertexWithDataComparison(physicalRouteGraph, sourceComponentVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                if (XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId)
                {
                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**)&sourceComponentVertexData), XME_STATUS_INTERNAL_ERROR);
                    sourceComponentVertexData->vertexId = vertexId;
                }
                else
                {
                    sourceComponentVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, sourceComponentVertexData);
                }
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != sourceComponentVertexData->vertexId);

                // Create and add the sink vertex in physical route graph.
                sinkComponentVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != sinkComponentVertexData);
                sinkComponentVertexData->vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;
                sinkComponentVertexData->nodeId = sinkVertex->nodeId;
                sinkComponentVertexData->topicId = topicId;
                sinkComponentVertexData->vertexData.componentPortVertex.portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                sinkComponentVertexData->vertexData.componentPortVertex.componentId = sinkVertex->vertexData.componentPortVertex.componentId;

                // Check if that component port vertex already exists in the graph. 
                vertexId = xme_hal_graph_getNextVertexWithDataComparison(physicalRouteGraph, sinkComponentVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                if (XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId)
                {
                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**)&sinkComponentVertexData), XME_STATUS_INTERNAL_ERROR);
                    sinkComponentVertexData->vertexId = vertexId;
                }
                else
                {
                    sinkComponentVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, sinkComponentVertexData);
                }
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != sinkComponentVertexData->vertexId);

                // Create the memcopy edge between source and destination at the same node
                memcopyEdgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_edgeData_t));
                XME_ASSERT(NULL != memcopyEdgeData);
                memcopyEdgeData->edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY;
                memcopyEdgeData->topicId = topicId;
                memcopyEdgeData->edgeId = xme_hal_graph_addEdge(physicalRouteGraph, sourceComponentVertexData->vertexId, sinkComponentVertexData->vertexId, memcopyEdgeData);
                XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != memcopyEdgeData->edgeId);
            }
        }
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiOutgoingEdgeIterator(logicalRouteGraph, sourceVertexId), XME_STATUS_INTERNAL_ERROR);
    }
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiVertexIterator(logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_ncc_getPhysicalRoutes
(
    xme_hal_graph_graph_t* logicalRouteGraph,
    xme_core_pnp_ncc_physicalRoutes_t* physicalRouteGraph
)
{
    xme_status_t status;

    XME_CHECK(NULL != logicalRouteGraph, XME_STATUS_INVALID_PARAMETER);

    status = xme_core_pnp_ncc_createPhysicalRoutes(logicalRouteGraph, physicalRouteGraph);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    //xme_core_pnp_ncc_linkGraphElementsToNodes(physicalRouteGraph);

    return XME_STATUS_SUCCESS;
}

int 
xme_core_pnp_ncc_vertexCompareCallback
(
    void* vertexData1,
    void* vertexData2
)
{
    xme_core_pnp_dataLinkGraph_vertexData_t* vd1 = (xme_core_pnp_dataLinkGraph_vertexData_t*) vertexData1;
    xme_core_pnp_dataLinkGraph_vertexData_t* vd2 = (xme_core_pnp_dataLinkGraph_vertexData_t*) vertexData2;

    // This callback only handles component port vertices, everything else does not match
    XME_CHECK(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vd1->vertexType, -1);
    XME_CHECK(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vd2->vertexType, -1);

    return (//vd1->vertexType == vd2->vertexType && <-- not needed because it is stopped in previous check. 
        vd1->nodeId == vd2->nodeId &&
        vd1->topicId == vd2->topicId &&
        vd1->vertexData.componentPortVertex.portType == vd2->vertexData.componentPortVertex.portType) ?
        0 : 1;
}


/**
 * @}
 */
