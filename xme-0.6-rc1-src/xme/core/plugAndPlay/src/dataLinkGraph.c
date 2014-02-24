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
 * $Id: dataLinkGraph.c 4934 2013-09-03 13:00:00Z geisinger $
 */

/**
 * \file
 *         Data link graph abstraction.
 */

/**
 * \addtogroup core_pnp_dataLinkGraph
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_pnp_dataLinkGraph_initVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData,
    xme_core_pnp_dataLinkGraph_vertexTypes_t vertexType,
    xme_hal_graph_vertexId_t vertexId,
    xme_core_node_nodeId_t nodeId,
    xme_core_topic_t topicId // TODO: Remove, defined in vertex and in edge! (but beware of LRM!)
)
{
    XME_ASSERT(NULL != vertexData);

    vertexData->vertexType = vertexType;
    vertexData->vertexId = vertexId;
    vertexData->nodeId = nodeId;
    vertexData->topicId = topicId;

    (void) xme_hal_mem_set(&(vertexData->vertexData), 0, sizeof(vertexData->vertexData));

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_dataLinkGraph_initComponentPortVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData,
    xme_core_component_t componentId,
    xme_core_component_portType_t portType,
    xme_core_pnp_lrm_announcement_t* announcement
)
{
    XME_ASSERT(NULL != vertexData);
    XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vertexData->vertexType);

    vertexData->vertexData.componentPortVertex.componentId = componentId;
    vertexData->vertexData.componentPortVertex.portType = portType;
    vertexData->vertexData.componentPortVertex.announcement = announcement;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData,
    uint8_t key[4],
    const char* ip
)
{
    XME_ASSERT(NULL != vertexData);
    XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND == vertexData->vertexType);

    (void) xme_hal_mem_copy(vertexData->vertexData.waypointUdpSendVertex.key, key, 4);
    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_ipv4StringToGenericAddress(ip, &vertexData->vertexData.waypointUdpSendVertex.destination), XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

// FIXME: Duplicated code. If both UDP-Send and UDP Receive are hosting the same data, do we need to have both of them?
xme_status_t
xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData,
    uint8_t key[4],
    const char* ip
)
{
    XME_ASSERT(NULL != vertexData);
    XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE == vertexData->vertexType);

    (void) xme_hal_mem_copy(vertexData->vertexData.waypointUdpReceiveVertex.key, key, 4);

    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_ipv4StringToGenericAddress(ip, &vertexData->vertexData.waypointUdpReceiveVertex.host), XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_dataLinkGraph_initEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData,
    xme_core_pnp_dataLinkGraph_edgeTypes_t edgeType,
    xme_hal_graph_edgeId_t edgeId,
    xme_core_topic_t topicId
)
{
    XME_ASSERT(NULL != edgeData);

    edgeData->edgeType = edgeType;
    edgeData->edgeId = edgeId;
    edgeData->topicId = topicId;

    (void) xme_hal_mem_set(&(edgeData->edgeData), 0, sizeof(edgeData->edgeData));

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData,
    xme_core_channelId_t channelId,
    bool established
)
{
    XME_ASSERT(NULL != edgeData);
    XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType);

    edgeData->edgeData.logicalRouteEdge.channelId = channelId;
    edgeData->edgeData.logicalRouteEdge.established = established;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_dataLinkGraph_initUdpLinkEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData,
    uint8_t key[4]
)
{
    XME_ASSERT(NULL != edgeData);
    XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK == edgeData->edgeType);

    (void) xme_hal_mem_copy(&edgeData->edgeData.udpLinkEdge.key, &key, sizeof(edgeData->edgeData.udpLinkEdge.key));

    return XME_STATUS_SUCCESS;
}

int
xme_core_pnp_dataLinkGraph_vertexCompareIdenticalVertexData
(
    void* vertexData1,
    void* vertexData2
)
{
    xme_core_pnp_dataLinkGraph_vertexData_t* vd1 = (xme_core_pnp_dataLinkGraph_vertexData_t*) vertexData1;
    xme_core_pnp_dataLinkGraph_vertexData_t* vd2 = (xme_core_pnp_dataLinkGraph_vertexData_t*) vertexData2;

    XME_CHECK(vd1->vertexType == vd2->vertexType, 1);
    XME_CHECK(vd1->nodeId == vd2->nodeId, 1);
    XME_CHECK(vd1->topicId == vd2->topicId, 1);

    switch (vd1->vertexType)
    {
        case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE:
        {
            // Nothing to check
            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT:
        {
            XME_CHECK(vd1->vertexData.componentPortVertex.componentId == vd2->vertexData.componentPortVertex.componentId, 1);
            XME_CHECK(vd1->vertexData.componentPortVertex.portType == vd2->vertexData.componentPortVertex.portType, 1);
            // TODO: announcement member?

            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER:
        {
            // Nothing to check
            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND:
        {
            // TODO: Check if this comparation is valid. 
            XME_CHECK(vd1->vertexData.waypointUdpSendVertex.destination.addressType == vd2->vertexData.waypointUdpSendVertex.destination.addressType, 1);
            XME_CHECK(vd1->vertexData.waypointUdpSendVertex.destination.data.ipv4.ip == vd2->vertexData.waypointUdpSendVertex.destination.data.ipv4.ip, 1);
            XME_CHECK(vd1->vertexData.waypointUdpSendVertex.destination.data.ipv4.port == vd2->vertexData.waypointUdpSendVertex.destination.data.ipv4.port, 1);
            XME_CHECK
            (
                0 == xme_hal_mem_compare
                (
                    vd1->vertexData.waypointUdpSendVertex.key,
                    vd2->vertexData.waypointUdpSendVertex.key,
                    sizeof(xme_core_pnp_dataLinkGraph_vertexData_t)
                ),
                1
            );

            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE:
        {
            // TODO: Check if this comparation is valid. 
            XME_CHECK(vd1->vertexData.waypointUdpReceiveVertex.host.addressType == vd2->vertexData.waypointUdpReceiveVertex.host.addressType, 1);
            XME_CHECK(vd1->vertexData.waypointUdpReceiveVertex.host.data.ipv4.ip == vd2->vertexData.waypointUdpReceiveVertex.host.data.ipv4.ip, 1);
            XME_CHECK(vd1->vertexData.waypointUdpReceiveVertex.host.data.ipv4.port == vd2->vertexData.waypointUdpReceiveVertex.host.data.ipv4.port, 1);
            XME_CHECK
            (
                0 == xme_hal_mem_compare
                (
                    vd1->vertexData.waypointUdpReceiveVertex.key,
                    vd2->vertexData.waypointUdpReceiveVertex.key,
                    sizeof(xme_core_pnp_dataLinkGraph_vertexData_t)
                ),
                1
            );

            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER:
        {
            // Nothing to check
            break;
        }

        default:
        {
            // Indicate an error
            XME_ASSERT_RVAL(false, 0);
        }
    }

    // No differences
    return 0;
}

int
xme_core_pnp_dataLinkGraph_edgeCompareIdenticalEdgeData
(
    void* edgeData1,
    void* edgeData2
)
{
    xme_core_pnp_dataLinkGraph_edgeData_t* ed1 = (xme_core_pnp_dataLinkGraph_edgeData_t*) edgeData1;
    xme_core_pnp_dataLinkGraph_edgeData_t* ed2 = (xme_core_pnp_dataLinkGraph_edgeData_t*) edgeData2;

    XME_CHECK(ed1->edgeType == ed2->edgeType, 1);
    XME_CHECK(ed1->topicId == ed2->topicId, 1);

    switch (ed1->edgeType)
    {
        case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE:
        {
            // Nothing to check
            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE:
        {
            XME_CHECK(ed1->edgeData.logicalRouteEdge.channelId == ed2->edgeData.logicalRouteEdge.channelId, 1);
            XME_CHECK(ed1->edgeData.logicalRouteEdge.established == ed2->edgeData.logicalRouteEdge.established, 1);

            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY:
        {
            // Nothing to check
            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK:
        {
            XME_CHECK
            (
                0 == xme_hal_mem_compare
                (
                    ed1->edgeData.udpLinkEdge.key,
                    ed2->edgeData.udpLinkEdge.key,
                    sizeof(xme_core_pnp_dataLinkGraph_edgeData_t)
                ),
                1
            );

            break;
        }

        default:
        {
            // Indicate an error
            XME_ASSERT_RVAL(false, 0);
        }
    }

    // No differences
    return 0;
}

void
xme_core_pnp_dataLinkGraph_dumpDataLinkGraph
(
    xme_log_severity_t severity,
    const char* graphName,
    xme_hal_graph_graph_t* dataLinkGraph
)
{
    xme_status_t status;

    static const char* const vertexTypeString[] =
    {
        "INVALID",
        "COMPORT",
        "MARSHAL",
        "DEMARSH",
        "UDPSEND",
        "UDPRECV"
    };

    static const char* const edgeTypeString[] =
    {
        "INVALID",
        "LOGICRT",
        "MEMCOPY",
        "UDPLINK"
    };

    static const char* const portTypeString[] =
    {
        "INVALID",
        "INTERNL",
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        "DCCPUBL",
        "DCCSUBS",
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        "RRRQSND",
        "RRRQHND",
        "RRRSSND",
        "RRRSHND"
    };

#ifdef NDEBUG
    // Early exit in release builds if severity is XME_LOG_DEBUG:
    // in this case, no messages would be output anyway.
    // TODO: Is this correct w.r.t. log callback?
    XME_CHECK(XME_LOG_DEBUG != severity, XME_CHECK_RVAL_VOID);
#endif // #ifdef NDEBUG

    XME_LOG(severity, "========== Dump of graph %s:\n", graphName);

    {
        xme_hal_graph_vertexCompareCallback_t vertexCallback;
        xme_hal_graph_vertexId_t currentVertex;
        uint16_t vertexCount = xme_hal_graph_getVertexCount(dataLinkGraph);
        uint32_t i;

        vertexCallback = xme_hal_graph_getVertexCompareCallback(dataLinkGraph);
        status = xme_hal_graph_setVertexCompareCallback(dataLinkGraph, &xme_core_pnp_dataLinkGraph_vertexCompareMatchAlways);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        XME_LOG(severity, "Vertex list (%u total):\n vid |  type   | data\n-----|---------|---------------------------------------------------------------\n", vertexCount);

        // We use a for loop to give the compiler an indication
        // of how many iterations will be run at max
        currentVertex = XME_HAL_GRAPH_INVALID_VERTEX_ID;
        for (i = 0; i < vertexCount; i++)
        {
            xme_core_pnp_dataLinkGraph_vertexData_t* vd;

            currentVertex = xme_hal_graph_getNextVertexWithDataComparison(dataLinkGraph, NULL, currentVertex);
            if (XME_HAL_GRAPH_INVALID_VERTEX_ID == currentVertex)
            {
                break;
            }

            status = xme_hal_graph_getVertexData(dataLinkGraph, currentVertex, (void**) &vd);
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
            XME_ASSERT_NORVAL(NULL != vd);
            XME_ASSERT_NORVAL(vd->vertexId == currentVertex);

            XME_LOG(severity, " %03u | %s | ", (uint32_t) currentVertex, vertexTypeString[vd->vertexType]);

            switch (vd->vertexType)
            {
            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT:
                XME_LOG
                (
                    severity, "nodeId=%u, compId=%u, portType=%s\n",
                    (uint32_t) vd->nodeId,
                    (uint32_t) vd->vertexData.componentPortVertex.componentId,
                    portTypeString[vd->vertexData.componentPortVertex.portType]
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND:
                XME_LOG
                (
                    severity, "dest=%s:%u, key=[%u, %u, %u, %u]\n",
                    vd->vertexData.waypointUdpSendVertex.destination.data.ipv4.ip,
                    vd->vertexData.waypointUdpSendVertex.destination.data.ipv4.port,
                    vd->vertexData.waypointUdpSendVertex.key[0],
                    vd->vertexData.waypointUdpSendVertex.key[1],
                    vd->vertexData.waypointUdpSendVertex.key[2],
                    vd->vertexData.waypointUdpSendVertex.key[3]
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE:
                XME_LOG
                (
                    severity, "host=%s:%u, key=[%u, %u, %u, %u]\n",
                    vd->vertexData.waypointUdpReceiveVertex.host.data.ipv4.ip,
                    vd->vertexData.waypointUdpReceiveVertex.host.data.ipv4.port,
                    vd->vertexData.waypointUdpSendVertex.key[0],
                    vd->vertexData.waypointUdpSendVertex.key[1],
                    vd->vertexData.waypointUdpSendVertex.key[2],
                    vd->vertexData.waypointUdpSendVertex.key[3]
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE:
            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER:
            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER:
            default:
                XME_LOG(severity, "\n");
            }
        }

        XME_LOG(severity, "-----|---------|---------------------------------------------------------------\n");

        status = xme_hal_graph_setVertexCompareCallback(dataLinkGraph, vertexCallback);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
    }

    {
        xme_hal_graph_edgeCompareCallback_t edgeCallback;
        xme_hal_graph_edgeId_t currentEdge;
        uint16_t edgeCount = xme_hal_graph_getEdgeCount(dataLinkGraph);
        uint32_t i;

        edgeCallback = xme_hal_graph_getEdgeCompareCallback(dataLinkGraph);
        status = xme_hal_graph_setEdgeCompareCallback(dataLinkGraph, &xme_core_pnp_dataLinkGraph_edgeCompareMatchAlways);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        XME_LOG(severity, "Edge list (%u total):\n eid | src | snk | type    | data\n-----|-----|-----|---------|---------------------------------------------------\n", edgeCount);

        // We use a for loop to give the compiler an indication
        // of how many iterations will be run at max
        currentEdge = XME_HAL_GRAPH_INVALID_EDGE_ID;
        for (i = 0; i < edgeCount; i++)
        {
            xme_core_pnp_dataLinkGraph_edgeData_t* ed;
            xme_hal_graph_vertexId_t source, sink;

            currentEdge = xme_hal_graph_getNextEdgeWithDataComparison(dataLinkGraph, NULL, currentEdge);
            if (XME_HAL_GRAPH_INVALID_EDGE_ID == currentEdge)
            {
                break;
            }

            status = xme_hal_graph_getEdgeData(dataLinkGraph, currentEdge, (void**) &ed);
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
            XME_ASSERT_NORVAL(NULL != ed);
            XME_ASSERT_NORVAL(ed->edgeId == currentEdge);

            source = xme_hal_graph_getSourceVertex(dataLinkGraph, currentEdge);
            XME_ASSERT_NORVAL(XME_HAL_GRAPH_INVALID_VERTEX_ID != source);

            sink = xme_hal_graph_getSinkVertex(dataLinkGraph, currentEdge);
            XME_ASSERT_NORVAL(XME_HAL_GRAPH_INVALID_VERTEX_ID != sink);

            XME_LOG(severity, " %03u | %03u | %03u | %s | ", (uint32_t) currentEdge, (uint32_t) source, (uint32_t) sink, edgeTypeString[ed->edgeType]);

            switch (ed->edgeType)
            {
            case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE:
                XME_LOG
                (
                    severity, "channel=%u, established=%s\n",
                    (uint32_t) ed->edgeData.logicalRouteEdge.channelId,
                    ed->edgeData.logicalRouteEdge.established ? "true" : "false"
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK:
                XME_LOG
                (
                    severity, "key=[%u, %u, %u, %u]\n",
                    ed->edgeData.udpLinkEdge.key[0],
                    ed->edgeData.udpLinkEdge.key[1],
                    ed->edgeData.udpLinkEdge.key[2],
                    ed->edgeData.udpLinkEdge.key[3]
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE:
            case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY:
            default:
                XME_LOG(severity, "\n");
            }
        }

        XME_LOG(severity, "-----|-----|-----|---------|---------------------------------------------------\n");

        status = xme_hal_graph_setEdgeCompareCallback(dataLinkGraph, edgeCallback);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
    }
}

int
xme_core_pnp_dataLinkGraph_vertexCompareMatchAlways
(
    void* vertexData1,
    void* vertexData2
)
{
    XME_UNUSED_PARAMETER(vertexData1);
    XME_ASSERT_RVAL(NULL == vertexData2, -1);

    return 0;
}

int
xme_core_pnp_dataLinkGraph_edgeCompareMatchAlways
(
    void* edgeData1,
    void* edgeData2
)
{
    XME_UNUSED_PARAMETER(edgeData1);
    XME_ASSERT_RVAL(NULL == edgeData2, -1);

    return 0;
}

int
xme_core_pnp_dataLinkGraph_edgeCompareFilterLogicalRoutes
(
    void* edgeData1,
    void* edgeData2
)
{
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData;

    XME_ASSERT_RVAL(NULL == edgeData2, -1);

    edgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) edgeData1;

    return (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType) ? 0 : -1;
}

/**
 * @}
 */
