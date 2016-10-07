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
 * $Id: networkConfigurationCalculator.c 7850 2014-03-14 15:23:48Z wiesmueller $
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

#include "xme/core/nodeManager/include/componentRepositoryPnpManagerInterface.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/hal/include/mem.h"

#include <inttypes.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Creates a link key for UDP waypoints.
 *
 * \param[in] id the input identifier to create the link key. 
 * \param[out] bArray the output array. 
 */
static void
xme_core_pnp_ncc_createLinkKey
(
    uint32_t id,
    uint8_t* bArray
);

/**
 * \brief Adds a memcopy edge to the given physical route graph.
 *
 * \param[in,out] physicalRouteGraph Physical route graph to modify.
 * \param[in] sourceVertexId Identifier of the source vertex in the graph.
 * \param[in] sinkVertexId Identifier of the sink vertex in the graph.
 * \param[in] topicId Identifier of the topic that the edge handles.
 * \param[in] channelID Channel identifier for the physical route. 
 */
static void
xme_core_pnp_ncc_addMemCopyEdge
(
    xme_core_pnp_ncc_physicalRoutes_t* physicalRouteGraph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    xme_core_topic_t topicId,
    xme_core_channelId_t channelID
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
 *
 * \param vertexData1 The first vertex data.
 * \param vertexData2 The second vertex data.
 *
 * \return 0 if the vertex data are equal and non zero in other case. 
 */
int 
xme_core_pnp_ncc_vertexCompareCallback
(
    void* vertexData1,
    void* vertexData2
);

/**
 * \brief Get sum of all component port queue sizes that are reachable
 *        via the given start vertex.
 *        Routes leaving the XME node of the given start vertex will not
 *        be visited.
 *
 *        Example:
 *              .->-W1()->-P2(3)
 *             /
 *        P1(?)
 *             \
 *              '-->----P3(2)
 *
 *        P1 - Start vertex node for which queue size will be calculated.
 *        W1 - A waypoint.
 *        P2 - Component port with queue size 3.
 *        P3 - Component port with queue size 2.
 *        >  - shows the direction of the edge.
 *        ?  - Queue size of the Port P1 which needs to be calculated
 *        After this function is run successfully the calculated queue
 *        size of P1 will be 5.
 *
 * \details In the current implementation a component port may be visited
 *          more then once if there are mutliple valid paths to it (its
 *          queue size will then be added as often as it is visited).
 *
 * \param[in] physicalRouteGraph Given physical route graph, where search
 *            is performed.
 * \param[in] startVertexID Vertex from which to start.
 *
 * \return Queue size sum, as defined above.
 */
static uint8_t
getQueueSizeSumOfAllConnectedComponentPorts
(
    xme_hal_graph_graph_t* const physicalRouteGraph,
    xme_hal_graph_vertexId_t startVertexID
);

/**
 * \brief Derive and set input port queue sizes of vertices.
 *
 * \details For each vertex the sum of all queue sizes of all
 *          incoming vertices is calculated.
 *          If this sum is bigger then the already assigned
 *          queue size, then it will be used instead.
 *
 * \param[in, out] physicalRouteGraph Given physical route graph.
 */
static void
deriveInputPortQueueSizes
(
    xme_hal_graph_graph_t* const physicalRouteGraph
);

/**
 * \brief Derive queue size for input ports connected to a udp receive.
 *
 * \details For these ports (respectively their vertex) getQueueSizeSumOfAllConnectedComponentPorts()
 *          is called to derive the queue size.
 *
 * \param[in, out] physicalRouteGraph Given physical route graph.
 */
static void
deriveUdpReceiveNextInputPortQueueSizes
(
    xme_hal_graph_graph_t* const physicalRouteGraph
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

static void
xme_core_pnp_ncc_createLinkKey
(
    uint32_t id,
    uint8_t* bArray
)
{
    int i=0;

    bArray[i++] = (uint8_t)(id >> 24);
    bArray[i++] = (uint8_t)(id >> 16);
    bArray[i++] = (uint8_t)(id >> 8);
    bArray[i++] = (uint8_t)(id >> 0);
}

static void
xme_core_pnp_ncc_addMemCopyEdge
(
    xme_core_pnp_ncc_physicalRoutes_t* physicalRouteGraph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    xme_core_topic_t topicId,
    xme_core_channelId_t channelID
)
{
    xme_core_pnp_dataLinkGraph_edgeData_t* memcopyEdgeData;

    memcopyEdgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_edgeData_t));
    XME_ASSERT_NORVAL(NULL != memcopyEdgeData);

    xme_core_pnp_dataLinkGraph_initEdgeData
    (
        memcopyEdgeData,
        XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY,
        xme_hal_graph_addEdge(physicalRouteGraph, sourceVertexId, sinkVertexId, memcopyEdgeData)
    );
    xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
    (
        memcopyEdgeData,
        topicId,
        channelID
    );
    XME_ASSERT_NORVAL(XME_HAL_GRAPH_INVALID_EDGE_ID != memcopyEdgeData->edgeId);
}

static uint8_t
getQueueSizeSumOfAllConnectedComponentPorts
(
    xme_hal_graph_graph_t* const physicalRouteGraph,
    xme_hal_graph_vertexId_t startVertexID
)
{
    xme_status_t status;
    uint8_t queueSize = 0u;
    xme_core_node_nodeId_t nodeID;
    xme_core_pnp_dataLinkGraph_vertexData_t* startVertexData = NULL;

    status = xme_hal_graph_getVertexData(physicalRouteGraph, startVertexID, (void**)&startVertexData);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    nodeID = startVertexData->nodeId;

    status = xme_hal_graph_initOutgoingEdgeIterator(physicalRouteGraph, startVertexID);
    XME_ASSERT(XME_STATUS_SUCCESS == status);
    while (xme_hal_graph_hasNextOutgoingEdge(physicalRouteGraph, startVertexID))
    {
        xme_hal_graph_vertexId_t sinkVertex;
        xme_core_pnp_dataLinkGraph_vertexData_t* sinkVertexData = NULL;
        xme_hal_graph_edgeId_t edgeId = xme_hal_graph_nextOutgoingEdge(physicalRouteGraph, startVertexID);

        sinkVertex = xme_hal_graph_getSinkVertex(physicalRouteGraph, edgeId);

        status = xme_hal_graph_getVertexData(physicalRouteGraph, sinkVertex, (void**)&sinkVertexData);
        XME_ASSERT(XME_STATUS_SUCCESS == status);

        if (nodeID != sinkVertexData->nodeId) { continue; }

        if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == sinkVertexData->vertexType)
        {
            xme_core_nodeMgr_compRep_componentHandle_t componentHandle;
            xme_core_nodeMgr_compRep_portHandle_t portHandle;

            status = xme_core_nodeMgr_compRep_getComponentInstance(
                sinkVertexData->nodeId,
                sinkVertexData->vertexData.componentPortVertex.componentId,
                &componentHandle);
            if (XME_STATUS_SUCCESS != status)
            {
                XME_LOG(XME_LOG_WARNING,
                    "[networkConfigurationCalculator] Component (nodeID = %d, componentID = %d) not registered in component repository. Input port queue size calculation of associated demarshaler not possible.\n",
                    sinkVertexData->nodeId, sinkVertexData->vertexData.componentPortVertex.componentId);
            }
            else
            {
                portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, sinkVertexData->vertexData.componentPortVertex.portIndex);
                XME_ASSERT(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE != portHandle);

                queueSize += xme_core_nodeMgr_compRep_getQueueSize(portHandle);
            }
        }
        else
        {
            queueSize += getQueueSizeSumOfAllConnectedComponentPorts(physicalRouteGraph, sinkVertex);
        }
    }
    status = xme_hal_graph_finiOutgoingEdgeIterator(physicalRouteGraph, startVertexID);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    return queueSize;
}

static void
deriveUdpReceiveNextInputPortQueueSizes
(
    xme_hal_graph_graph_t* const physicalRouteGraph
)
{
    xme_status_t status;

    status = xme_hal_graph_initVertexIterator(physicalRouteGraph);
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
    while (xme_hal_graph_hasNextVertex(physicalRouteGraph))
    {
        xme_hal_graph_vertexId_t sourceVertexID = xme_hal_graph_nextVertex(physicalRouteGraph);
        xme_core_pnp_dataLinkGraph_vertexData_t* sourceVertexData;

        status = xme_hal_graph_getVertexData(physicalRouteGraph, sourceVertexID, (void**)&sourceVertexData);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE != sourceVertexData->vertexType) { continue; }

        status = xme_hal_graph_initOutgoingEdgeIterator(physicalRouteGraph, sourceVertexID);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
        while (xme_hal_graph_hasNextOutgoingEdge(physicalRouteGraph, sourceVertexID))
        {
            xme_hal_graph_edgeId_t edgeID = xme_hal_graph_nextOutgoingEdge(physicalRouteGraph, sourceVertexID);
            xme_core_pnp_dataLinkGraph_edgeData_t* edgeData = NULL;
            xme_hal_graph_vertexId_t sinkVertexID;
            xme_core_pnp_dataLinkGraph_vertexData_t* sinkVertexData = NULL;

            status = xme_hal_graph_getEdgeData(physicalRouteGraph, edgeID, (void**)&edgeData);
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
            if (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY != edgeData->edgeType) { continue; }

            sinkVertexID = xme_hal_graph_getSinkVertex(physicalRouteGraph, edgeID);

            status = xme_hal_graph_getVertexData(physicalRouteGraph, sinkVertexID, (void**)&sinkVertexData);
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

            switch (sinkVertexData->vertexType)
            {
                case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER:
                    {
                        sinkVertexData->vertexData.waypointDemarshalerVertex.inputPortQueueSize =
                            getQueueSizeSumOfAllConnectedComponentPorts(physicalRouteGraph, sinkVertexID);
                    }
                    break;
                default:
                    // We do not support several waypoints here, as they are never connected to a udp receive waypoint
                    // If this changes in the future we need to extend the code
                    XME_ASSERT_NORVAL(false);
            }
        }
        status = xme_hal_graph_finiOutgoingEdgeIterator(physicalRouteGraph, sourceVertexID);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
    }
    status = xme_hal_graph_finiVertexIterator(physicalRouteGraph);
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
}

static void
deriveInputPortQueueSizes
(
    xme_hal_graph_graph_t* const physicalRouteGraph
)
{
    xme_status_t status;

    status = xme_hal_graph_initVertexIterator(physicalRouteGraph);
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
    while (xme_hal_graph_hasNextVertex(physicalRouteGraph))
    {
        xme_hal_graph_vertexId_t sinkVertexID = xme_hal_graph_nextVertex(physicalRouteGraph);
        xme_core_pnp_dataLinkGraph_vertexData_t* sinkVertexData;
        uint8_t derivedQueueSize = 0u;

        status = xme_hal_graph_getVertexData(physicalRouteGraph, sinkVertexID, (void**)&sinkVertexData);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        status = xme_hal_graph_initIncomingEdgeIterator(physicalRouteGraph, sinkVertexID);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
        while (xme_hal_graph_hasNextIncomingEdge(physicalRouteGraph, sinkVertexID))
        {
            xme_hal_graph_edgeId_t edgeID = xme_hal_graph_nextIncomingEdge(physicalRouteGraph, sinkVertexID);
            xme_hal_graph_vertexId_t sourceVertexID;
            xme_core_pnp_dataLinkGraph_vertexData_t* sourceVertexData = NULL;
            xme_core_pnp_dataLinkGraph_edgeData_t* edgeData = NULL;

            status = xme_hal_graph_getEdgeData(physicalRouteGraph, edgeID, (void**)&edgeData);
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
            if (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY != edgeData->edgeType) { continue; }

            sourceVertexID = xme_hal_graph_getSourceVertex(physicalRouteGraph, edgeID);

            status = xme_hal_graph_getVertexData(physicalRouteGraph, sourceVertexID, (void**)&sourceVertexData);
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

            // Switch for source type
            switch (sourceVertexData->vertexType)
            {
                case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT:
                    {
                        xme_core_nodeMgr_compRep_componentHandle_t sourceComponentHandle;
                        xme_core_nodeMgr_compRep_componentHandle_t sourcePortHandle;

                        status = xme_core_nodeMgr_compRep_getComponentInstance(sourceVertexData->nodeId,
                            sourceVertexData->vertexData.componentPortVertex.componentId, &sourceComponentHandle);
                        if (XME_STATUS_SUCCESS != status)
                        {
                            XME_LOG(XME_LOG_WARNING,
                                "[networkConfigurationCalculator] Component (nodeID = %d, componentID = %d) not registered in component repository. Input port queue size calculation not possible.\n",
                                sourceVertexData->nodeId, sourceVertexData->vertexData.componentPortVertex.componentId);
                        }
                        else
                        {
                            sourcePortHandle = xme_core_nodeMgr_compRep_getPort(sourceComponentHandle, sourceVertexData->vertexData.componentPortVertex.portIndex);

                            derivedQueueSize += xme_core_nodeMgr_compRep_getQueueSize(sourcePortHandle);
                        }
                    }
                    break;
                case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER:
                case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER:
                case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND:
                case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE:
                case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR:
                case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR:
                    {
                        derivedQueueSize += 1u;
                    }
                    break;
                default:
                    XME_ASSERT_NORVAL(false);
            }
        }
        status = xme_hal_graph_finiIncomingEdgeIterator(physicalRouteGraph, sinkVertexID);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        switch (sinkVertexData->vertexType)
        {
            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT:
                {
                    xme_core_nodeMgr_compRep_componentHandle_t sinkComponentHandle;
                    xme_core_nodeMgr_compRep_componentHandle_t sinkPortHandle;
                    uint16_t originalQueueSize;

                    status = xme_core_nodeMgr_compRep_getComponentInstance(sinkVertexData->nodeId,
                        sinkVertexData->vertexData.componentPortVertex.componentId, &sinkComponentHandle);
                    if (XME_STATUS_SUCCESS != status)
                    {
                        XME_LOG(XME_LOG_WARNING,
                            "[networkConfigurationCalculator] Component (nodeID = %d, componentID = %d) not registered in component repository. Input port queue size calculation not possible.\n",
                            sinkVertexData->nodeId, sinkVertexData->vertexData.componentPortVertex.componentId);
                    }
                    else
                    {
                        sinkPortHandle = xme_core_nodeMgr_compRep_getPort(sinkComponentHandle, sinkVertexData->vertexData.componentPortVertex.portIndex);

                        originalQueueSize = xme_core_nodeMgr_compRep_getQueueSize(sinkPortHandle);

                        if (derivedQueueSize > originalQueueSize)
                        {
                            xme_core_nodeMgr_compRep_state_t componentState =
                                xme_core_nodeMgr_compRep_getState(sinkComponentHandle);
                
                            if (XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED == componentState ||
                                XME_CORE_NODEMGR_COMPREP_STATE_PREPARED == componentState)
                            {
                                xme_core_nodeMgr_compRep_setQueueSize(sinkPortHandle, derivedQueueSize);
                            }
                            else
                            {
                                XME_LOG(XME_LOG_WARNING, "Trying to set queue size of already created component port (componentID = %d, componentType = %d, portTypeID = %d) "
                                    "according to specification of output port queue sizes that are connected to this port. Changing queue size of already created ports currently not supported. "
                                    "Current queue size = %d, intended new queue size = %d.\n",
                                    xme_core_nodeMgr_compRep_getComponentID(sinkComponentHandle),
                                    xme_core_nodeMgr_compRep_getComponentType(sinkComponentHandle),
                                    sinkVertexData->vertexData.componentPortVertex.portIndex,
                                    (int32_t)originalQueueSize,
                                    (int32_t)derivedQueueSize);
                            }
                        }
                    }
                }
                break;
            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER:
                {
                    sinkVertexData->vertexData.waypointMarshalerVertex.inputPortQueueSize = derivedQueueSize;
                }
                break;
            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR:
                {
                    sinkVertexData->vertexData.waypointChannelInjectorVertex.inputPortQueueSize = derivedQueueSize;
                }
                break;
            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR:
                {
                    sinkVertexData->vertexData.waypointChannelSelectorVertex.inputPortQueueSize = derivedQueueSize;
                }
                break;
            default:
                // We do not support setting of input port queue sizes for several of the waypoints
                // For these it should not be necessary to set the queue size, because the default
                // size of 1 (or 0 if the waypoint does not have input ports) should be enough
                // If this changes in the future we need to update the code here
                XME_ASSERT_NORVAL(1u >= derivedQueueSize);
        }
    }
    status = xme_hal_graph_finiVertexIterator(physicalRouteGraph);
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
}

static xme_status_t
xme_core_pnp_ncc_createPhysicalRoutes
(
    xme_hal_graph_graph_t* logicalRouteGraph,
    xme_core_pnp_ncc_physicalRoutes_t* physicalRouteGraph
)
{
    xme_status_t status;
    xme_core_pnp_dataLinkGraph_vertexData_t* sourceVertex;
    xme_core_pnp_dataLinkGraph_vertexData_t* sinkVertex;

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_init((xme_hal_graph_graph_t*)physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_setVertexCompareCallback(physicalRouteGraph, xme_core_pnp_ncc_vertexCompareCallback), XME_STATUS_INTERNAL_ERROR);

    // Iterate over logical routes
    status = xme_hal_graph_initVertexIterator(logicalRouteGraph);
    XME_ASSERT(XME_STATUS_SUCCESS == status);
    while (xme_hal_graph_hasNextVertex(logicalRouteGraph))
    {
        xme_hal_graph_vertexId_t sourceVertexId = xme_hal_graph_nextVertex(logicalRouteGraph);
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(logicalRouteGraph, sourceVertexId, (void**) &sourceVertex), XME_STATUS_INTERNAL_ERROR);

        status = xme_hal_graph_initOutgoingEdgeIterator(logicalRouteGraph, sourceVertexId);
        XME_ASSERT(XME_STATUS_SUCCESS == status);
        while (xme_hal_graph_hasNextOutgoingEdge(logicalRouteGraph, sourceVertexId))
        {
            xme_core_pnp_dataLinkGraph_edgeData_t* edgeData = NULL;
            xme_hal_graph_vertexId_t sinkVertexId;
            xme_core_topic_t topicId;
            xme_core_pnp_dataLinkGraph_vertexData_t* sourceComponentVertexData = NULL;
            xme_core_pnp_dataLinkGraph_vertexData_t* sinkComponentVertexData = NULL;
            xme_core_pnp_dataLinkGraph_vertexData_t* nextVertexAfterSourceData = NULL;

            xme_core_channelId_t channelID; 

            xme_hal_graph_edgeId_t tempEdgeId = xme_hal_graph_nextOutgoingEdge(logicalRouteGraph, sourceVertexId);
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(logicalRouteGraph, tempEdgeId, (void**) &edgeData), XME_STATUS_INTERNAL_ERROR);

            // Ignore everything other than logical routes
            if (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE != edgeData->edgeType)
            {
                continue;
            }

            // If this is already existing infrastructure we don't need to process it
            if (edgeData->edgeData.logicalRouteEdge.established)
            {
                continue;
            }

            sinkVertexId = xme_hal_graph_getSinkVertex(logicalRouteGraph, tempEdgeId);
            topicId = edgeData->edgeData.logicalRouteEdge.topicId;
            channelID = edgeData->edgeData.logicalRouteEdge.channelId;

            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(logicalRouteGraph, sinkVertexId, (void**) &sinkVertex), XME_STATUS_INTERNAL_ERROR);

            if (sourceVertex->nodeId != sinkVertex->nodeId)
            {
                // Source and destination are on different nodes.
                // Hence, there is a need for marshaller/udpSend/udpReceive/demarshaller waypoints.
                uint8_t bArray[4];

                xme_core_pnp_dataLinkGraph_vertexData_t* marshalerWaypointVertexData;
                xme_core_pnp_dataLinkGraph_vertexData_t* udpSendWaypointVertexData;
                xme_core_pnp_dataLinkGraph_vertexData_t* udpReceiveWaypointVertexData;
                xme_core_pnp_dataLinkGraph_vertexData_t* demarshalerWaypointVertexData;
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
                        "[NetworkConfigurationCalculator] No UDP network interface address associated to the node %" PRIu16 ".\n",
                        sinkVertex->nodeId
                    );
                    continue;
                }

                // Create the common link key
                xme_core_pnp_ncc_createLinkKey((uint32_t)channelID, bArray);

                // Create and add the source vertex in physical route graph
                sourceComponentVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != sourceComponentVertexData);

                XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == sourceVertex->vertexType);
                XME_ASSERT(topicId == sourceVertex->topicId);
                (void) xme_hal_mem_copy(sourceComponentVertexData, sourceVertex, sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));

                // Check if that component port vertex already exists in the graph
                vertexId = xme_hal_graph_getNextVertexWithDataComparison(physicalRouteGraph, sourceComponentVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                if (XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId)
                {
                    xme_hal_mem_free(sourceComponentVertexData);
                    sourceComponentVertexData = NULL;

                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**)&sourceComponentVertexData), XME_STATUS_INTERNAL_ERROR);
                    XME_ASSERT(sourceComponentVertexData->vertexId == vertexId);
                }
                else
                {
                    sourceComponentVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, sourceComponentVertexData);
                }
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != sourceComponentVertexData->vertexId);

                // Create and add the marshaler waypoint vertex in physical route graph
                marshalerWaypointVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != marshalerWaypointVertexData);

                xme_core_pnp_dataLinkGraph_initVertexData
                (
                    marshalerWaypointVertexData,
                    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER,
                    xme_hal_graph_addVertex(physicalRouteGraph, marshalerWaypointVertexData),
                    sourceVertex->nodeId,
                    topicId
                );
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != marshalerWaypointVertexData->vertexId);

                // Remember that we still need to create a memcopy edge between source and marshaling waypoint to the physical route graph
                nextVertexAfterSourceData = marshalerWaypointVertexData;

                // Create and add the udpSend waypoint vertex
                udpSendWaypointVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != udpSendWaypointVertexData);

                xme_core_pnp_dataLinkGraph_initVertexData
                (
                    udpSendWaypointVertexData,
                    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND,
                    xme_hal_graph_addVertex(physicalRouteGraph, udpSendWaypointVertexData),
                    sourceVertex->nodeId,
                    topicId
                );
                xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
                (
                    udpSendWaypointVertexData,
                    bArray,
                    nodeIPport
                );
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != udpSendWaypointVertexData->vertexId);

                // Create and add the memcopy edge between marshaling waypoint and UDP send waypoint
                xme_core_pnp_ncc_addMemCopyEdge(physicalRouteGraph, marshalerWaypointVertexData->vertexId, udpSendWaypointVertexData->vertexId, topicId, channelID);

                // Create the udpRecv waypoint vertex
                udpReceiveWaypointVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != udpReceiveWaypointVertexData);

                xme_core_pnp_dataLinkGraph_initVertexData
                (
                    udpReceiveWaypointVertexData,
                    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE,
                    xme_hal_graph_addVertex(physicalRouteGraph, udpReceiveWaypointVertexData),
                    sinkVertex->nodeId,
                    topicId
                );
                // Now this is UDP Receive we just need to listen to our port so bascially we already have the destination in nodeIPport
                xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
                (
                    udpReceiveWaypointVertexData,
                    bArray,
                    nodeIPport
                );
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != udpReceiveWaypointVertexData->vertexId);

                // Create the udpLink edge between UDP send waypoint and UDP receive waypoint
                udpLinkEdgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_edgeData_t));
                XME_ASSERT(NULL != udpLinkEdgeData);

                xme_core_pnp_dataLinkGraph_initEdgeData
                (
                    udpLinkEdgeData,
                    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK,
                    xme_hal_graph_addEdge(physicalRouteGraph, udpSendWaypointVertexData->vertexId, udpReceiveWaypointVertexData->vertexId, udpLinkEdgeData)
                );
                xme_core_pnp_dataLinkGraph_initUdpLinkEdgeData
                (
                    udpLinkEdgeData,
                    topicId,
                    bArray
                );
                XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != udpLinkEdgeData->edgeId);

                // Create and add the demarshaler waypoint vertex
                demarshalerWaypointVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != demarshalerWaypointVertexData);

                xme_core_pnp_dataLinkGraph_initVertexData
                (
                    demarshalerWaypointVertexData,
                    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER,
                    xme_hal_graph_addVertex(physicalRouteGraph, demarshalerWaypointVertexData),
                    sinkVertex->nodeId,
                    topicId
                );
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != demarshalerWaypointVertexData->vertexId);

                // Create and add the memcopy edge between UDP receive waypoint and demarshaler waypoint
                xme_core_pnp_ncc_addMemCopyEdge(physicalRouteGraph, udpReceiveWaypointVertexData->vertexId, demarshalerWaypointVertexData->vertexId, topicId, channelID);

                // Create and add the sink vertex in physical route graph
                sinkComponentVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != sinkComponentVertexData);

                XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == sinkVertex->vertexType);
                XME_ASSERT(topicId == sinkVertex->topicId);
                (void) xme_hal_mem_copy(sinkComponentVertexData, sinkVertex, sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));

                // Check if that component port vertex already exists in the graph. 
                vertexId = xme_hal_graph_getNextVertexWithDataComparison(physicalRouteGraph, sinkComponentVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                if (XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId)
                {
                    xme_hal_mem_free(sinkComponentVertexData);
                    sinkComponentVertexData = NULL;

                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**)&sinkComponentVertexData), XME_STATUS_INTERNAL_ERROR);
                    XME_ASSERT(sinkComponentVertexData->vertexId == vertexId);
                }
                else
                {
                    sinkComponentVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, sinkComponentVertexData);
                }
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != sinkComponentVertexData->vertexId);

                // Create and add the memcopy edge between demarshaler waypoint and destination
                xme_core_pnp_ncc_addMemCopyEdge(physicalRouteGraph, demarshalerWaypointVertexData->vertexId, sinkComponentVertexData->vertexId, topicId, channelID);
            }
            else
            {
                xme_hal_graph_vertexId_t vertexId;

                // Create and add the source vertex in physical route graph.
                sourceComponentVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != sourceComponentVertexData);

                XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == sourceVertex->vertexType);
                XME_ASSERT(topicId == sourceVertex->topicId);
                (void) xme_hal_mem_copy(sourceComponentVertexData, sourceVertex, sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));

                // Check if that component port vertex already exists in the graph. 
                vertexId = xme_hal_graph_getNextVertexWithDataComparison(physicalRouteGraph, sourceComponentVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                if (XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId)
                {
                    xme_hal_mem_free(sourceComponentVertexData);
                    sourceComponentVertexData = NULL;

                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**)&sourceComponentVertexData), XME_STATUS_INTERNAL_ERROR);
                    XME_ASSERT(sourceComponentVertexData->vertexId == vertexId);
                }
                else
                {
                    sourceComponentVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, sourceComponentVertexData);
                }
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != sourceComponentVertexData->vertexId);

                // Create and add the sink vertex in physical route graph.
                sinkComponentVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != sinkComponentVertexData);

                XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == sinkVertex->vertexType);
                XME_ASSERT(topicId == sinkVertex->topicId);
                (void) xme_hal_mem_copy(sinkComponentVertexData, sinkVertex, sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));

                // Check if that component port vertex already exists in the graph. 
                vertexId = xme_hal_graph_getNextVertexWithDataComparison(physicalRouteGraph, sinkComponentVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
                if (XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId)
                {
                    xme_hal_mem_free(sinkComponentVertexData);
                    sinkComponentVertexData = NULL;

                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**)&sinkComponentVertexData), XME_STATUS_INTERNAL_ERROR);
                    XME_ASSERT(sinkComponentVertexData->vertexId == vertexId);
                }
                else
                {
                    sinkComponentVertexData->vertexId = xme_hal_graph_addVertex(physicalRouteGraph, sinkComponentVertexData);
                }
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != sinkComponentVertexData->vertexId);

                // Remember that we still need to create a memcopy edge between source and destination component port vertex
                nextVertexAfterSourceData = sinkComponentVertexData;
            }

            if (XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == sourceComponentVertexData->vertexData.componentPortVertex.portType)
            {
                xme_core_pnp_dataLinkGraph_vertexData_t* channelInjectorVertexData;

                // The sink port is a request handler, add a channel injector waypoint in between
                channelInjectorVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != channelInjectorVertexData);

                xme_core_pnp_dataLinkGraph_initVertexData
                (
                    channelInjectorVertexData,
                    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR,
                    xme_hal_graph_addVertex(physicalRouteGraph, channelInjectorVertexData),
                    sourceComponentVertexData->nodeId,
                    topicId
                );
                xme_core_pnp_dataLinkGraph_initWaypointChannelInjectorVertexData
                (
                    channelInjectorVertexData,
                    edgeData->edgeData.logicalRouteEdge.channelId,
                    1u
                );

                // Create and add the memcopy edge between request channel
                // injector waypoint and request handler. The memcopy edge
                // between request sender and channel injector waypoint is
                // added below according to nextVertexAfterSourceData.
                xme_core_pnp_ncc_addMemCopyEdge(physicalRouteGraph, channelInjectorVertexData->vertexId, nextVertexAfterSourceData->vertexId, topicId, channelID);
                nextVertexAfterSourceData = channelInjectorVertexData;
            }
            else if (XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == sourceComponentVertexData->vertexData.componentPortVertex.portType)
            {
                xme_core_pnp_dataLinkGraph_vertexData_t* channelSelectorVertexData;
                xme_core_channelId_t sourceChannelId = XME_CORE_INVALID_CHANNEL_ID;

                // The source port is a response sender, add a channel selector waypoint in between
                XME_ASSERT(XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == sinkComponentVertexData->vertexData.componentPortVertex.portType);

                // The source channel identifier is the channel identifier of the
                // incoming edge into sourceComponentVertex that has a destination
                // channel identifier of edgeData->edgeData.logicalRouteEdge.channelId
                status = xme_hal_graph_initIncomingEdgeIterator(logicalRouteGraph, sourceVertexId);
                XME_ASSERT(XME_STATUS_SUCCESS == status);
                {
                    while (xme_hal_graph_hasNextIncomingEdge(logicalRouteGraph, sourceVertexId))
                    {
                        xme_hal_graph_edgeId_t cmEdgeId;
                        xme_core_pnp_dataLinkGraph_edgeData_t* cmEdgeData = NULL;

                        cmEdgeId = xme_hal_graph_nextIncomingEdge(logicalRouteGraph, sourceVertexId);
                        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(logicalRouteGraph, cmEdgeId, (void**) &cmEdgeData), XME_STATUS_INTERNAL_ERROR);

                        // We expect only channel mapping edges
                        XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING == cmEdgeData->edgeType);

                        if (cmEdgeData->edgeData.channelMappingEdge.destChannelId == edgeData->edgeData.logicalRouteEdge.channelId)
                        {
                            // Found the correct channel mapping edge
                            sourceChannelId = cmEdgeData->edgeData.channelMappingEdge.sourceChannelId;
                            break;
                        }
                    }
                }
                status = xme_hal_graph_finiIncomingEdgeIterator(logicalRouteGraph, sourceVertexId);
                XME_ASSERT(XME_STATUS_SUCCESS == status);

                // Is is an error if we couldn't find a matching channel mapping edge
                XME_ASSERT(XME_CORE_INVALID_CHANNEL_ID != sourceChannelId);

                channelSelectorVertexData = (xme_core_pnp_dataLinkGraph_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));
                XME_ASSERT(NULL != channelSelectorVertexData);

                xme_core_pnp_dataLinkGraph_initVertexData
                (
                    channelSelectorVertexData,
                    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR,
                    xme_hal_graph_addVertex(physicalRouteGraph, channelSelectorVertexData),
                    sourceComponentVertexData->nodeId,
                    topicId
                );
                xme_core_pnp_dataLinkGraph_initWaypointChannelSelectorVertexData
                (
                    channelSelectorVertexData,
                    sourceChannelId,
                    edgeData->edgeData.logicalRouteEdge.channelId,
                    1u
                );

                // Create and add the memcopy edge between channel selector
                // waypoint and response handler. The memcopy edge between
                // response sender and channel selector waypoint is added
                // below according to nextVertexAfterSourceData.
                xme_core_pnp_ncc_addMemCopyEdge(physicalRouteGraph, channelSelectorVertexData->vertexId, nextVertexAfterSourceData->vertexId, topicId, channelID);
                nextVertexAfterSourceData = channelSelectorVertexData;
            }

            // Finally create and add the memcopy edge between source
            // and the next component port or waypoint, depending on what
            // nextVertexAfterSourceData points to
            XME_ASSERT(NULL != nextVertexAfterSourceData);
            xme_core_pnp_ncc_addMemCopyEdge(physicalRouteGraph, sourceComponentVertexData->vertexId, nextVertexAfterSourceData->vertexId, topicId, channelID);
        }
        status = xme_hal_graph_finiOutgoingEdgeIterator(logicalRouteGraph, sourceVertexId);
        XME_ASSERT(XME_STATUS_SUCCESS == status);
    }
    status = xme_hal_graph_finiVertexIterator(logicalRouteGraph);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    deriveInputPortQueueSizes(physicalRouteGraph);

    deriveUdpReceiveNextInputPortQueueSizes(physicalRouteGraph);

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
        vd1->vertexData.componentPortVertex.componentId == vd2->vertexData.componentPortVertex.componentId &&
        vd1->vertexData.componentPortVertex.portIndex == vd2->vertexData.componentPortVertex.portIndex &&
        vd1->vertexData.componentPortVertex.portType == vd2->vertexData.componentPortVertex.portType) ?
        0 : 1;
}

/**
 * @}
 */
