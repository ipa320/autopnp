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
 * $Id: dataLinkGraph.h 7828 2014-03-14 09:32:09Z ruiz $
 */

/**
 * \file
 *         Data link graph abstraction.
 */

#ifndef XME_CORE_PNP_DATALINKGRAPH_H
#define XME_CORE_PNP_DATALINKGRAPH_H

/**
 * \defgroup core_pnp_dataLinkGraph Data Link Graph
 * @{
 *
 * \brief Data Link Graph is a helper class for the logical and physical route
 *        calculation, associated to the logic of Plug and Play components.
 *
 * \details The data link graph implements specifically for Plug and Play Manager
 *          the generic implementation of graph.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/component.h"
#include "xme/core/container.h"
#include "xme/core/log.h"
#include "xme/core/manifestTypes.h"
#include "xme/core/node.h"

#include "xme/core/dataHandler/include/dataHandler.h"

#include "xme/core/plugAndPlay/include/logicalRouteManager.h"
#include "xme/core/directory/include/nodeRegistryController.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     xme_core_pnp_dataLinkGraph_vertexData_t and related                    //
//----------------------------------------------------------------------------//

/**
 * \struct xme_core_pnp_dataLinkGraph_componentPortVertexData_t
 *
 * \brief Vertex-specific data associated to a component port vertex in the
 *        data link graph.
 */
typedef struct
{
    xme_core_component_t componentId; ///< Identifier of the component that owns the port this component port vertex represents.
    xme_core_componentType_t componentType; ///< The type of the component.
    xme_core_component_portType_t portType; ///< Type of the component port.
    uint16_t portIndex; ///< Zero-based index of the port within the component.
    xme_core_component_connectionBound_t lowerConnectionBound; ///< Lower connection bound. \see xme_core_component_connectionBound_t
    xme_core_component_connectionBound_t upperConnectionBound; ///< Upper connection bound. \see xme_core_component_connectionBound_t
    xme_core_pnp_lrm_announcement_t* announcement; ///< Announcement data. // TODO: Where is this needed?
}
xme_core_pnp_dataLinkGraph_componentPortVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointMarshalerVertexData_t
 *
 * \brief Vertex-specific data associated to a marshaler waypoint vertex in the
 *        data link graph.
 */
typedef struct
{
    uint8_t inputPortQueueSize; ///< Queue size for the input port of the marshaler configuration.
}
xme_core_pnp_dataLinkGraph_waypointMarshalerVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointDemarshalerVertexData_t
 *
 * \brief Vertex-specific data associated to a UDP demarshaler waypoint vertex
 *        in the data link graph.
 */
typedef struct
{
    uint8_t inputPortQueueSize; ///< Queue size for the input port of the marshaler configuration.
}
xme_core_pnp_dataLinkGraph_waypointDemarshalerVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointUdpSendVertexData_t
 *
 * \brief Vertex-specific data associated to a UDP send waypoint vertex in the
 *        data link graph.
 */
typedef struct
{
    uint8_t key[4]; ///< UPD communication key.
    xme_com_interface_address_t destination; ///< Destination address (IPv4 or IPv6).
}
xme_core_pnp_dataLinkGraph_waypointUdpSendVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointUdpReceiveVertexData_t
 *
 * \brief Vertex-specific data associated to a UDP receive waypoint vertex in
 *        the data link graph.
 */
typedef struct
{
    uint8_t key[4]; ///< UDP communication key.
    xme_com_interface_address_t host; ///< (Local) host address (IPv4 or IPv6).
}
xme_core_pnp_dataLinkGraph_waypointUdpReceiveVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointChannelInjectorVertexData_t
 *
 * \brief Vertex-specific data associated to a channel injector waypoint vertex
 *        in the data link graph.
 */
typedef struct
{
    xme_core_channelId_t injectedChannelId; ///< Injected channel identifier.
    uint8_t inputPortQueueSize; ///< Queue size for the input port of the marshaler configuration.
}
xme_core_pnp_dataLinkGraph_waypointChannelInjectorVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointChannelSelectorVertexData_t
 *
 * \brief Vertex-specific data associated to a channel selector waypoint vertex
 *        in the data link graph.
 */
typedef struct
{
    xme_core_channelId_t sourceChannelId; ///< Source channel identifier to be mapped to a destination channel identifier.
    xme_core_channelId_t destinationChannelId; ///< Destination channel identifier the given source channel identifier is mapped to.
    uint8_t inputPortQueueSize; ///< Queue size for the input port of the marshaler configuration.
}
xme_core_pnp_dataLinkGraph_waypointChannelSelectorVertexData_t;

/**
 * \union xme_core_pnp_dataLinkGraph_vertexSpecificData_t
 *
 * \brief Union of all possible vertex-specific data structure of a data link
 *        graph vertex.
 */
typedef union
{
    xme_core_pnp_dataLinkGraph_componentPortVertexData_t componentPortVertex; ///< Vertex data interpreted as a component port.
    xme_core_pnp_dataLinkGraph_waypointMarshalerVertexData_t waypointMarshalerVertex; ///< Vertex data interpreted as a marshaler waypoint.
    xme_core_pnp_dataLinkGraph_waypointDemarshalerVertexData_t waypointDemarshalerVertex; ///< Vertex data interpreted as a demarshaler waypoint.
    xme_core_pnp_dataLinkGraph_waypointUdpSendVertexData_t waypointUdpSendVertex; ///< Vertex data interpreted as a UDP send waypoint.
    xme_core_pnp_dataLinkGraph_waypointUdpReceiveVertexData_t waypointUdpReceiveVertex; ///< Vertex data interpreted as a UDP receive waypoint.
    xme_core_pnp_dataLinkGraph_waypointChannelInjectorVertexData_t waypointChannelInjectorVertex; ///< Vertex data interpreted as a channel injector waypoint.
    xme_core_pnp_dataLinkGraph_waypointChannelSelectorVertexData_t waypointChannelSelectorVertex; ///< Vertex data interpreted as a channel selector waypoint.
}
xme_core_pnp_dataLinkGraph_vertexSpecificData_t;

/**
 * \enum xme_core_pnp_dataLinkGraph_vertexTypes_t
 *
 * \brief Set of different vertex types that can be stored in the data link graph.
 */
typedef enum
{
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE = 0, ///< Invalid vertex type. An invalid vertex type is used as an error code in functions returning a vertex type or is assigned to empty data structures.
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, ///< Component port vertex type. A component port vertex represents a publisher or subscriber port of a component.
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER, ///< Marshaler waypoint vertex type. A marshaler waypoint vertex models a host-to-network translation of data.
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER, ///< Demarshaler waypoint vertex type. A demarshaler waypoint vertex models a network-to-host translation of data.
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND, ///< UDP send waypoint vertex type. A UDP send waypoint vertex models the sending of data over the network using UDP.
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE, ///< UDP receive waypoint vertex type. A UDP send waypoint vertex models the reception of data from the network using UDP.
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR, ///< Channel injector waypoint vertex type. A channel injector waypoint vertex represents the injection of a logical channel identifier on a node for implementation of the request part of the request-response communication pattern.
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR, ///< Channel selector waypoint vertex type. A channel selector waypoint vertex represents the selection of one of a number of logical communication channels for implementation of the response part of the request-response communication pattern.
}
xme_core_pnp_dataLinkGraph_vertexTypes_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_vertexData_t
 *
 * \brief Generic data associated with a vertex in the data link graph.
 */
typedef struct
{
    // Common data
    xme_core_pnp_dataLinkGraph_vertexTypes_t vertexType; ///< Type of vertex. Depending on this value, select the appropriate value from the vertexData union below.
    xme_hal_graph_vertexId_t vertexId; ///< Identifier of the corresponding vertex in the data link graph.
    xme_core_node_nodeId_t nodeId; ///< Node on which the entity represented by this vertex is deployed to.
    xme_core_topic_t topicId; ///< Topic that the vertex handles.
    // Vertex specific data
    xme_core_pnp_dataLinkGraph_vertexSpecificData_t vertexData; ///< Vertex specific data (depending on the vertex type).
}
xme_core_pnp_dataLinkGraph_vertexData_t;

//----------------------------------------------------------------------------//
//     xme_core_pnp_dataLinkGraph_edgeData_t and related                      //
//----------------------------------------------------------------------------//

/**
 * \struct xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t
 *
 * \brief Edge-specific data associated to a logical route edge in the data
 *        link graph.
 */
typedef struct
{
    xme_core_topic_t topicId; ///< Topic that the edge handles.
    xme_core_channelId_t channelId; ///< Channel identifier associated with the logical route.
    bool established; ///< Whether the route has been established before by calling xme_core_pnp_lrm_setLogicalRoute() on the respective channel.
    //int maxSize; // TODO: Implement
    //int alphaCurve; // TODO: Implement
    //int metric; // TODO: Implement
}
xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t
 *
 * \brief Edge-specific data associated to a channel mapping edge in the data
 *        link graph.
 */
typedef struct
{
    xme_core_channelId_t sourceChannelId; ///< Source channel identifier of the channel mapping.
    xme_core_channelId_t destChannelId; ///< Destination channel identifier of the channel mapping.
}
xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_memCopyEdgeData_t
 *
 * \brief Edge-specific data associated to a memory copy edge in the data link
 *        graph.
 */
typedef struct
{
    xme_core_topic_t topicId; ///< Topic that the edge handles.
    xme_core_channelId_t channelID; ///< The channel associated to the memory copy.
}
xme_core_pnp_dataLinkGraph_memCopyEdgeData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_udpLinkEdgeData_t
 *
 * \brief Edge-specific data associated to a UDP link edge in the data link
 *        graph.
 */
typedef struct
{
    xme_core_topic_t topicId; ///< Topic that the edge handles.
    uint8_t key[4]; ///< UDP communication key.
}
xme_core_pnp_dataLinkGraph_udpLinkEdgeData_t;

/**
 * \union xme_core_pnp_dataLinkGraph_edgeSpecificData_t
 *
 * \brief Union of all possible edge-specific data structure of a data link
 *        graph edge.
 */
typedef union
{
    xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t logicalRouteEdge; ///< Edge data interpreted as logical route information.
    xme_core_pnp_dataLinkGraph_channelMappingEdgeData_t channelMappingEdge; ///< Edge data interpreted as channel mapping information.
    xme_core_pnp_dataLinkGraph_memCopyEdgeData_t memCopyEdge; ///< Edge data interpreted as memory copy information.
    xme_core_pnp_dataLinkGraph_udpLinkEdgeData_t udpLinkEdge; ///< Edge data interpreted as UDP link information.
}
xme_core_pnp_dataLinkGraph_edgeSpecificData_t;

/**
 * \enum xme_core_pnp_dataLinkGraph_edgeTypes_t
 *
 * \brief Set of different edge types that can be stored in the data link graph.
 */
typedef enum
{
    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE = 0, ///< Invalid edge type. An invalid edge type is used as an error code in functions returning an edge type or is assigned to empty data structures.
    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ///< Logical route edge type. A logical route represents a logical communication path between a publisher and a subscriber.
    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_CHANNELMAPPING, ///< Channel mapping edge type. A channel mapping represents a related between an incoming and an outgoing channel for the request response communication pattern.
    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ///< Memory copy edge. A memory copy edge models a intra-node data copying process between two data packets.
    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK ///< UDP link edge. A UDP link edge models a network communication between two nodes using UDP.
}
xme_core_pnp_dataLinkGraph_edgeTypes_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_edgeData_t
 *
 * \brief Generic data associated with an edge in the data link graph.
 */
typedef struct
{
    // Common data
    xme_core_pnp_dataLinkGraph_edgeTypes_t edgeType; ///< Type of edge. Depending on this value, select the appropriate value from the edgeData union below.
    xme_hal_graph_edgeId_t edgeId; ///< Identifier of the corresponding edge in the data link graph.
    // Edge specific data
    xme_core_pnp_dataLinkGraph_edgeSpecificData_t edgeData; ///< Edge specific data (depending on the edge type).
}
xme_core_pnp_dataLinkGraph_edgeData_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN

//----------------------------------------------------------------------------//
//     xme_core_pnp_dataLinkGraph_vertexData_t and related                    //
//----------------------------------------------------------------------------//

/**
 * \brief Initializes the generic members of a structure of type
 *        xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] vertexType Type of vertex.
 * \param[in] vertexId Identifier of the corresponding vertex in the data path graph.
 * \param[in] nodeId Node on which the entity represented by this vertex is deployed to.
 * \param[in] topicId Topic that the vertex handles.
 */
void
xme_core_pnp_dataLinkGraph_initVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    xme_core_pnp_dataLinkGraph_vertexTypes_t vertexType,
    xme_hal_graph_vertexId_t vertexId,
    xme_core_node_nodeId_t nodeId,
    xme_core_topic_t topicId
);

/**
 * \brief Initializes the specific members associated to a component port in
 *        a structure of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] componentId the component identifier.
 * \param[in] componentType The component type. 
 * \param[in] portType the port type of the component port.
 * \param[in] portIndex Zero-based index of the port within the component.
 * \param[in] lowerConnectionBound The lower connection bound.
 * \param[in] upperConnectionBound The upper connection bound.
 * \param[in] announcement associated announcement of the component port in the logical route manager.
 */
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
);

/**
 * \brief Initializes the specific members associated to a marshaler waypoint
 *        in a structure of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] inputPortQueueSize Queue size of input port.
 */
void
xme_core_pnp_dataLinkGraph_initWaypointMarshalerVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    uint8_t inputPortQueueSize
);

/**
 * \brief Initializes the specific members associated to a demarshaler waypoint
 *        in a structure of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] inputPortQueueSize Queue size of input port.
 */
void
xme_core_pnp_dataLinkGraph_initWaypointDemarshalerVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    uint8_t inputPortQueueSize
);

/**
 * \brief Initializes the specific members associated to a UDP send waypoint in
 *        a structure of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] key UDP communication key.
 * \param[in] destination Destination address (IP and port).
 */
void
xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    const uint8_t key[4],
    const xme_com_interface_address_t* const destination
);

/**
 * \brief Initializes the specific members associated to a UDP receive waypoint
 *        in a structure of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] key UDP communication key.
 * \param[in] host (Local) host address (IP and port).
 */
void
xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    const uint8_t key[4],
    const xme_com_interface_address_t* const host
);

/**
 * \brief Initializes the specific members associated to a channel injector
 *        waypoint in a structure of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] injectedChannelId Channel identifier to be injected.
 * \param[in] inputPortQueueSize Queue size of the input port.
 */
void
xme_core_pnp_dataLinkGraph_initWaypointChannelInjectorVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    xme_core_channelId_t injectedChannelId,
    uint8_t inputPortQueueSize
);

/**
 * \brief Initializes the specific members associated to a channel selector
 *        waypoint in a structure of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] sourceChannelId Source channel identifier to be mapped to a
 *            destination channel identifier.
 * \param[in] destinationChannelId Destination channel identifier the given
 *            source channel identifier is mapped to.
 * \param[in] inputPortQueueSize Queue size of the input port.
 */
void
xme_core_pnp_dataLinkGraph_initWaypointChannelSelectorVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    xme_core_channelId_t sourceChannelId,
    xme_core_channelId_t destinationChannelId,
    uint8_t inputPortQueueSize
);

//----------------------------------------------------------------------------//
//     xme_core_pnp_dataLinkGraph_edgeData_t and related                      //
//----------------------------------------------------------------------------//

/**
 * \brief Initializes the generic members of a structure of type
 *        xme_core_pnp_dataLinkGraph_edgeData_t.
 *
 * \param[in,out] edgeData Address of the edge data struct to initialize.
 * \param[in] edgeType Type of edge.
 * \param[in] edgeId Identifier of the corresponding edge in the data path graph.
 */
void
xme_core_pnp_dataLinkGraph_initEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_pnp_dataLinkGraph_edgeTypes_t edgeType,
    xme_hal_graph_edgeId_t edgeId
);

/**
 * \brief Initializes the specific members associated to a logical route
 *        in a structure of type xme_core_pnp_dataLinkGraph_edgeData_t.
 *
 * \param[in,out] edgeData Address of the edge data struct to initialize.
 * \param[in] topicId Topic that the edge handles.
 * \param[in] channelId The channel id associated to the logical route.
 * \param[in] established Determines if the logical route is already established.
 */
void
xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_topic_t topicId,
    xme_core_channelId_t channelId,
    bool established
);

/**
 * \brief Initializes the specific members associated to a channel mapping
 *        in a structure of type xme_core_pnp_dataLinkGraph_edgeData_t.
 *
 * \param[in,out] edgeData Address of the edge data struct to initialize.
 * \param[in] sourceChannelId The source channel ID associated to the channel
 *            mapping.
 * \param[in] destChannelId The destination channel ID associated to the
 *            channel mapping.
 */
void
xme_core_pnp_dataLinkGraph_initChannelMappingEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_channelId_t sourceChannelId,
    xme_core_channelId_t destChannelId
);

/**
 * \brief Initializes the specific members associated to a memcopy edge
 *        in a structure of type xme_core_pnp_dataLinkGraph_edgeData_t.
 *
 * \param[in,out] edgeData Address of the edge data struct to initialize.
 * \param[in] topicId Topic that the edge handles.
 * \param[in] channelID The channel identifier for the edge.
 */
void
xme_core_pnp_dataLinkGraph_initMemcopyEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_topic_t topicId,
    xme_core_channelId_t channelID
);

/**
 * \brief Initializes the specific members associated to a UDP link
 *        in a structure of type xme_core_pnp_dataLinkGraph_edgeData_t.
 *
 * \param[in,out] edgeData Address of the edge data struct to initialize.
 * \param[in] topicId Topic that the edge handles.
 * \param[in] key The key associated to the UDP link.
 */
void
xme_core_pnp_dataLinkGraph_initUdpLinkEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* const edgeData,
    xme_core_topic_t topicId,
    const uint8_t key[4]
);

//----------------------------------------------------------------------------//
//     Misc                                                                   //
//----------------------------------------------------------------------------//

/**
 * \brief  Vertex compare callback function for checking whether the data
 *         associated to a vertex is identical to the one being searched for.
 *
 * \note To be used in conjunction with
 *       ::xme_hal_graph_setVertexCompareCallback().
 *
 * \param[in] vertexData1 Data associated to the first vertex.
 *            Supposed to be of type xme_core_pnp_dataLinkGraph_vertexData_t.
 * \param[in] vertexData2 Data associated to the second vertex.
 *            Supposed to be of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \retval 0 if all members except for the vertex identifier are the same.
 * \retval 1 if at least one member except for the vertex identifier is
 *           different.
 */
int
xme_core_pnp_dataLinkGraph_vertexCompareIdenticalVertexData
(
    void* vertexData1,
    void* vertexData2
);

/**
 * \brief  Edge compare callback function for checking whether the data
 *         associated to an edge is identical to the one being searched for.
 *
 * \note To be used in conjunction with
 *       ::xme_hal_graph_setEdgeCompareCallback().
 *
 * \param[in] edgeData1 Data associated to the first edge.
 *            Supposed to be of type xme_core_pnp_dataLinkGraph_edgeData_t.
 * \param[in] edgeData2 Data associated to the second edge.
 *            Supposed to be of type xme_core_pnp_dataLinkGraph_edgeData_t.
 *
 * \retval 0 if all members except for the edge identifier are the same.
 * \retval 1 if at least one member except for the edge identifier is
 *           different.
 */
int
xme_core_pnp_dataLinkGraph_edgeCompareIdenticalEdgeData
(
    void* edgeData1,
    void* edgeData2
);

/**
 * \brief  Vertex compare callback function that returns always match
 *         between vertices to compare.
 *
 * \param[in] vertexData1 Data associated to the first vertex.
 * \param[in] vertexData2 Data associated to the second vertex.
 *
 * \retval 0 always.
 */
int
xme_core_pnp_dataLinkGraph_vertexCompareMatchAlways
(
    void* vertexData1,
    void* vertexData2
);

/**
 * \brief  Edge compare callback function that returns always match
 *         between edges to compare.
 *
 * \param[in] edgeData1 Data associated to the first edge.
 * \param[in] edgeData2 Data associated to the second edge.
 *
 * \retval 0 always.
 */
int
xme_core_pnp_dataLinkGraph_edgeCompareMatchAlways
(
    void* edgeData1,
    void* edgeData2
);

/**
 * \brief  Edge compare callback function that check if first edge is
 *         of type 'logical route'.
 *
 * \param[in] edgeData1 Data associated to the first edge.
 * \param[in] edgeData2 Data associated to the second edge.
 *
 * \retval 0 if the first edge data is labeled as 'logical route'.
 * \retval -1 if the first edge data is not a logical route, or if the edge
 *         data of the second edge is NULL.
 */
int
xme_core_pnp_dataLinkGraph_edgeCompareFilterLogicalRoutes
(
    void* edgeData1,
    void* edgeData2
);

/**
 * \brief Checks the given combination of component port properties for
 *        validity.
 *
 * \param[in] portType the port type of the component port.
 * \param[in] lowerConnectionBound The lower connection bound.
 * \param[in] upperConnectionBound The upper connection bound.
 *
 * \retval XME_STATUS_SUCCESS if the given combination of port properties
 *         is compatible.
 * \retval XME_STATUS_INVALID_CONFIGURATION if the given combination of port
 *         properties is incompatible.
 */
xme_status_t
xme_core_pnp_dataLinkGraph_checkComponentPort
(
    xme_core_component_portType_t portType,
    xme_core_component_connectionBound_t lowerConnectionBound,
    xme_core_component_connectionBound_t upperConnectionBound
);

/**
 * \brief  Dumps the content of the given data link graph in text form to the
 *         console for debugging purposes.
 *
 * \param[in] severity Log message severity to use for the logged messages.
 *            For example, use XME_LOG_DEBUG to only print messages in debug
 *            builds or XME_LOG_ALWAYS to always print the graph.
 * \param[in] graphName C string containing the name of the graph.
 *            Will be output just in front of the content of the graph.
 * \param[in] dataLinkGraph Data link graph to dump.
 */
void
xme_core_pnp_dataLinkGraph_dumpDataLinkGraph
(
    xme_log_severity_t severity,
    const char* graphName,
    xme_hal_graph_graph_t* dataLinkGraph
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_DATALINKGRAPH_H
