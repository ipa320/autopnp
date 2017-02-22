/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: dataLinkGraph.c 7694 2014-03-06 15:56:24Z wiesmueller $
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

#include "xme/core/componentInfo.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     xme_core_pnp_dataLinkGraph_vertexData_t and related                    //
//----------------------------------------------------------------------------//

void
xme_core_pnp_dataLinkGraph_initVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    xme_core_pnp_dataLinkGraph_vertexTypes_t vertexType,
    xme_hal_graph_vertexId_t vertexId,
    xme_core_node_nodeId_t nodeId,
    xme_core_topic_t topicId // TODO: Remove, defined in vertex and in edge! (but beware of LRM!)
)
{
    XME_ASSERT_NORVAL(NULL != vertexData);

    vertexData->vertexType = vertexType;
    vertexData->vertexId = vertexId;
    vertexData->nodeId = nodeId;
    vertexData->topicId = topicId;

    (void) xme_hal_mem_set(&(vertexData->vertexData), 0, sizeof(vertexData->vertexData));
}

void
xme_core_pnp_dataLinkGraph_initComponentPortVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    xme_core_component_t componentId,
    xme_core_componentType_t componentType,
    xme_core_component_portType_t portType,
    uint16_t portIndex,
    xme_core_component_connectionBound_t lowerConnectionBound,
    xme_core_component_connectionBound_t upperConnectionBound,
    xme_core_pnp_lrm_announcement_t* const announcement
)
{
    XME_ASSERT_NORVAL(NULL != vertexData);
    XME_ASSERT_NORVAL(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vertexData->vertexType);

#ifdef DEBUG
    {
        xme_status_t status = xme_core_pnp_dataLinkGraph_checkComponentPort(portType, lowerConnectionBound, upperConnectionBound);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
    }
#endif

    vertexData->vertexData.componentPortVertex.componentId = componentId;
    vertexData->vertexData.componentPortVertex.componentType = componentType;
    vertexData->vertexData.componentPortVertex.portType = portType;
    vertexData->vertexData.componentPortVertex.portIndex = portIndex;
    vertexData->vertexData.componentPortVertex.lowerConnectionBound = lowerConnectionBound;
    vertexData->vertexData.componentPortVertex.upperConnectionBound = upperConnectionBound;
    vertexData->vertexData.componentPortVertex.announcement = announcement;
}

void
xme_core_pnp_dataLinkGraph_initWaypointMarshalerVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    uint8_t inputPortQueueSize
)
{
    XME_ASSERT_NORVAL(NULL != vertexData);

    vertexData->vertexData.waypointMarshalerVertex.inputPortQueueSize = inputPortQueueSize;
}

void
xme_core_pnp_dataLinkGraph_initWaypointDemarshalerVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    uint8_t inputPortQueueSize
)
{
    XME_ASSERT_NORVAL(NULL != vertexData);

    vertexData->vertexData.waypointDemarshalerVertex.inputPortQueueSize = inputPortQueueSize;
}

void
xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    const uint8_t key[4],
    const xme_com_interface_address_t* const destination
)
{
    XME_ASSERT_NORVAL(NULL != vertexData);
    XME_ASSERT_NORVAL(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND == vertexData->vertexType);

    (void) xme_hal_mem_copy(vertexData->vertexData.waypointUdpSendVertex.key, key, XME_WP_UDP_HEADER_KEY_LENGTH);
    (void) xme_hal_mem_copy(&vertexData->vertexData.waypointUdpSendVertex.destination, destination, sizeof(xme_com_interface_address_t));
}

// FIXME: Similar code than above. If both UDP-Send and UDP Receive are hosting the same data, do we need to have both of them?
void
xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    const uint8_t key[4],
    const xme_com_interface_address_t* const host
)
{
    XME_ASSERT_NORVAL(NULL != vertexData);
    XME_ASSERT_NORVAL(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE == vertexData->vertexType);

    (void) xme_hal_mem_copy(vertexData->vertexData.waypointUdpReceiveVertex.key, key, XME_WP_UDP_HEADER_KEY_LENGTH);
    (void) xme_hal_mem_copy(&vertexData->vertexData.waypointUdpReceiveVertex.host, host, sizeof(xme_com_interface_address_t));
}

void
xme_core_pnp_dataLinkGraph_initWaypointChannelInjectorVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    xme_core_channelId_t injectedChannelId,
    uint8_t inputPortQueueSize
)
{
    XME_ASSERT_NORVAL(NULL != vertexData);

    vertexData->vertexData.waypointChannelInjectorVertex.injectedChannelId = injectedChannelId;
    vertexData->vertexData.waypointChannelInjectorVertex.inputPortQueueSize = inputPortQueueSize;
}

void
xme_core_pnp_dataLinkGraph_initWaypointChannelSelectorVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    xme_core_channelId_t sourceChannelId,
    xme_core_channelId_t destinationChannelId,
    uint8_t inputPortQueueSize
)
{
    XME_ASSERT_NORVAL(NULL != vertexData);

    vertexData->vertexData.waypointChannelSelectorVertex.sourceChannelId = sourceChannelId;
    vertexData->vertexData.waypointChannelSelectorVertex.destinationChannelId = destinationChannelId;
    vertexData->vertexData.waypointChannelSelectorVertex.inputPortQueueSize = inputPortQueueSize;
}

//----------------------------------------------------------------------------//
//     xme_core_pnp_dataLinkGraph_edgeData_t and related                      //
//----------------------------------------------------------------------------//

void
xme_core_pnp_dataLinkGraph_initEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_pnp_dataLinkGraph_edgeTypes_t edgeType,
    xme_hal_graph_edgeId_t edgeId
)
{
    XME_ASSERT_NORVAL(NULL != edgeData);

    edgeData->edgeType = edgeType;
    edgeData->edgeId = edgeId;

    (void) xme_hal_mem_set(&(edgeData->edgeData), 0, sizeof(edgeData->edgeData));
}

void
xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_topic_t topicId,
    xme_core_channelId_t channelId,
    bool established
)
{
    XME_ASSERT_NORVAL(NULL != edgeData);
    XME_ASSERT_NORVAL(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType);

    edgeData->edgeData.logicalRouteEdge.topicId = topicId;
    edgeData->edgeData.logicalRouteEdge.channelId = channelId;
    edgeData->edgeData.logicalRouteEdge.established = established;
}

void
xme_core_pnp_dataLinkGraph_initChannelMappingEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_channelId_t sourceChannelId,
    xme_core_channelId_t destChannelId
)
{
    XME_ASSERT_NORVAL(NULL != edgeData);
    XME_ASSERT_NORVAL(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING == edgeData->edgeType);

    edgeData->edgeData.channelMappingEdge.sourceChannelId = sourceChannelId;
    edgeData->edgeData.channelMappingEdge.destChannelId = destChannelId;
}

void
xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_topic_t topicId, 
    xme_core_channelId_t channelID
)
{
    XME_ASSERT_NORVAL(NULL != edgeData);
    XME_ASSERT_NORVAL(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY == edgeData->edgeType);

    edgeData->edgeData.memCopyEdge.topicId = topicId;
    edgeData->edgeData.memCopyEdge.channelID = channelID;
}

void
xme_core_pnp_dataLinkGraph_initUdpLinkEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_topic_t topicId,
    const uint8_t key[4]
)
{
    XME_ASSERT_NORVAL(NULL != edgeData);
    XME_ASSERT_NORVAL(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK == edgeData->edgeType);

    edgeData->edgeData.udpLinkEdge.topicId = topicId;
    if (NULL != key)
    {
        (void) xme_hal_mem_copy(&edgeData->edgeData.udpLinkEdge.key, key, sizeof(edgeData->edgeData.udpLinkEdge.key));
    }
}

//----------------------------------------------------------------------------//
//     Misc                                                                   //
//----------------------------------------------------------------------------//

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


        case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR:
        {
            // Nothing to check
            break;
        }


        case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR:
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

    switch (ed1->edgeType)
    {
        case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE:
        {
            // Nothing to check
            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE:
        {
            XME_CHECK(ed1->edgeData.logicalRouteEdge.topicId == ed2->edgeData.logicalRouteEdge.topicId, 1);
            XME_CHECK(ed1->edgeData.logicalRouteEdge.channelId == ed2->edgeData.logicalRouteEdge.channelId, 1);
            XME_CHECK(ed1->edgeData.logicalRouteEdge.established == ed2->edgeData.logicalRouteEdge.established, 1);

            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY:
        {
            XME_CHECK(ed1->edgeData.memCopyEdge.topicId == ed2->edgeData.memCopyEdge.topicId, 1);

            break;
        }

        case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK:
        {
            XME_CHECK(ed1->edgeData.udpLinkEdge.topicId == ed2->edgeData.udpLinkEdge.topicId, 1);
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

xme_status_t
xme_core_pnp_dataLinkGraph_checkComponentPort
(
    xme_core_component_portType_t portType,
    xme_core_component_connectionBound_t lowerConnectionBound,
    xme_core_component_connectionBound_t upperConnectionBound
)
{
    // Check if lower/upperConnectionBound is valid for given portType.
    // See the documentation of xme_core_component_connectionBound_t in xme/core/component.h for details.
    switch (portType)
    {
        case XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION:
        case XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER:
            XME_CHECK(
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID == lowerConnectionBound &&
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID == upperConnectionBound,
                XME_STATUS_INVALID_CONFIGURATION
            );
            break;

        case XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION:
        case XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER:
        case XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER:
        case XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER:
            XME_CHECK(
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID != lowerConnectionBound &&
                XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED != lowerConnectionBound &&
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID != upperConnectionBound,
                XME_STATUS_INVALID_CONFIGURATION
            );
            break;

        case XME_CORE_COMPONENT_PORTTYPE_INVALID:
        case XME_CORE_COMPONENT_PORTTYPE_INTERNAL:
        default:
            return XME_STATUS_INVALID_CONFIGURATION;
    }

    return XME_STATUS_SUCCESS;
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
        "UDPRECV",
        "CHANINJ",
        "CHANSEL"
    };

    static const char* const edgeTypeString[] =
    {
        "INVALID",
        "LOGICRT",
        "CHANMAP",
        "MEMCOPY",
        "UDPLINK"
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

            XME_LOG(severity, " %03u | %s | ", (unsigned int) currentVertex, vertexTypeString[vd->vertexType]);

            switch (vd->vertexType)
            {
            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT:
                XME_LOG
                (
                    severity, "nodeId=%u, compId=%u, compType=%u, topic=%u, portType=%s\n",
                    (unsigned int) vd->nodeId,
                    (unsigned int) vd->vertexData.componentPortVertex.componentId,
                    (unsigned int) vd->vertexData.componentPortVertex.componentType,
                    (unsigned int) vd->topicId,
                    xme_core_component_getPortTypeString(vd->vertexData.componentPortVertex.portType)
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND:
                XME_LOG
                (
                    severity, "dest=%hu.%hu.%hu.%hu:%hu, key=[%hu, %hu, %hu, %hu]\n",
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.destination.data.ipv4.ip[0],
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.destination.data.ipv4.ip[1],
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.destination.data.ipv4.ip[2],
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.destination.data.ipv4.ip[3],
                    (unsigned short) (vd->vertexData.waypointUdpSendVertex.destination.data.ipv4.port[1] |
                        (vd->vertexData.waypointUdpSendVertex.destination.data.ipv4.port[0] << 8)),
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.key[0],
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.key[1],
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.key[2],
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.key[3]
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE:
                XME_LOG
                (
                    severity, "host=%hu.%hu.%hu.%hu:%hu, key=[%hu, %hu, %hu, %hu]\n",
                    vd->vertexData.waypointUdpReceiveVertex.host.data.ipv4.ip[0],
                    vd->vertexData.waypointUdpReceiveVertex.host.data.ipv4.ip[1],
                    vd->vertexData.waypointUdpReceiveVertex.host.data.ipv4.ip[2],
                    vd->vertexData.waypointUdpReceiveVertex.host.data.ipv4.ip[3],
                    (unsigned short) (vd->vertexData.waypointUdpReceiveVertex.host.data.ipv4.port[1] |
                        (vd->vertexData.waypointUdpReceiveVertex.host.data.ipv4.port[0] << 8)),
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.key[0],
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.key[1],
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.key[2],
                    (unsigned short) vd->vertexData.waypointUdpSendVertex.key[3]
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR:
                XME_LOG
                (
                    severity, "injectedChannel=%u\n",
                    (unsigned int) vd->vertexData.waypointChannelInjectorVertex.injectedChannelId
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR:
                XME_LOG
                (
                    severity, "sourceChannel=%u, destinationChannel=%u\n",
                    (unsigned int) vd->vertexData.waypointChannelSelectorVertex.sourceChannelId,
                    (unsigned int) vd->vertexData.waypointChannelSelectorVertex.destinationChannelId
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
                    severity, "topic=%u, channel=%u, established=%s\n",
                    (unsigned int) ed->edgeData.logicalRouteEdge.topicId,
                    (unsigned int) ed->edgeData.logicalRouteEdge.channelId,
                    ed->edgeData.logicalRouteEdge.established ? "true" : "false"
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING:
                XME_LOG
                (
                    severity, "sourceChannel=%u, destinationChannel=%u\n",
                    (unsigned int) ed->edgeData.channelMappingEdge.sourceChannelId,
                    (unsigned int) ed->edgeData.channelMappingEdge.destChannelId
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY:
                XME_LOG
                (
                    severity, "topic=%u\n",
                    (unsigned int) ed->edgeData.memCopyEdge.topicId
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK:
                XME_LOG
                (
                    severity, "topic=%u, key=[%hu, %hu, %hu, %hu]\n",
                    (unsigned int) ed->edgeData.udpLinkEdge.topicId,
                    (unsigned short) ed->edgeData.udpLinkEdge.key[0],
                    (unsigned short) ed->edgeData.udpLinkEdge.key[1],
                    (unsigned short) ed->edgeData.udpLinkEdge.key[2],
                    (unsigned short) ed->edgeData.udpLinkEdge.key[3]
                );
                break;

            case XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE:
            default:
                XME_LOG(severity, "\n");
            }
        }

        XME_LOG(severity, "-----|-----|-----|---------|---------------------------------------------------\n");

        status = xme_hal_graph_setEdgeCompareCallback(dataLinkGraph, edgeCallback);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
    }
}

/**
 * @}
 */
