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
 * $Id: dataLinkGraph.h 4656 2013-08-12 15:47:40Z ruiz $
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
#include "xme/core/node.h"

#include "xme/core/dataHandler/include/dataHandler.h"

#include "xme/core/plugAndPlay/include/logicalRouteManager.h"
#include "xme/core/directory/include/nodeRegistryController.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     xme_core_pnp_dataLinkGraph_vertexData_t and related              //
//----------------------------------------------------------------------------//

/**
 * \struct xme_core_pnp_dataLinkGraph_componentPortVertexData_t
 * \brief The component port vertex associated data. 
 * 
 */
typedef struct
{
    xme_core_component_t componentId; ///< the component id. 
    xme_core_component_portType_t portType; ///< the port type: publication, subscription. 
    uint16_t portIndex; ///< the port index inside the component. 
    xme_core_pnp_lrm_announcement_t* announcement; ///< the announcement data. // TODO: Where is this needed?
}
xme_core_pnp_dataLinkGraph_componentPortVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointMarshalerVertexData_t
 * \brief The marshaler waypoint vertex associated data. 
 */
typedef struct
{
    uint8_t thisIsJustAPlaceholder_PutVertexSpecificDataHere; ///< the data associated to the marshaller waypoint: configuration. 
}
xme_core_pnp_dataLinkGraph_waypointMarshalerVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointDemarshalerVertexData_t
 * \brief The demarshaler waypoint vertex associated data. 
 */
typedef struct
{
    uint8_t thisIsJustAPlaceholder_PutVertexSpecificDataHere; ///< the data associated to the demarshaller waypoint: configuration. 
}
xme_core_pnp_dataLinkGraph_waypointDemarshalerVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointUdpSendVertexData_t
 * \brief The UDP send waypoint vertex associated data. 
 */
typedef struct
{
    uint8_t key[4]; ///< UPD key.
    xme_com_interface_address_t destination; ///< the destination address. 
}
xme_core_pnp_dataLinkGraph_waypointUdpSendVertexData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_waypointUdpReceiveVertexData_t
 * \brief The UDP receive waypoint vertex associated data. 
 */
typedef struct
{
    uint8_t key[4]; ///< the key associated to the UDP. 
    xme_com_interface_address_t host; ///< the host address. 
}
xme_core_pnp_dataLinkGraph_waypointUdpReceiveVertexData_t;

/**
 * \union xme_core_pnp_dataLinkGraph_vertexSpecificData_t
 * \brief Union of all possible data types that can be vertices in the graph. 
 */
typedef union
{
    xme_core_pnp_dataLinkGraph_componentPortVertexData_t componentPortVertex; ///< the vertex is a component port vertex.
    xme_core_pnp_dataLinkGraph_waypointMarshalerVertexData_t waypointMarshalerVertex; ///< the vertex is a marshaler waypoint port vertex.
    xme_core_pnp_dataLinkGraph_waypointDemarshalerVertexData_t waypointDemarshalerVertex; ///< the vertex is a demarshaller waypoint port vertex.
    xme_core_pnp_dataLinkGraph_waypointUdpSendVertexData_t waypointUdpSendVertex; ///< the vertex is a UDP send waypoint port vertex.
    xme_core_pnp_dataLinkGraph_waypointUdpReceiveVertexData_t waypointUdpReceiveVertex; ///< the vertex is a UDP receive component port vertex.
}
xme_core_pnp_dataLinkGraph_vertexSpecificData_t;

/**
 * \enum xme_core_pnp_dataLinkGraph_vertexTypes_t
 * \brief the set of different vertices types that can be stored in the data link graph. 
 */
typedef enum
{
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE = 0, ///< invalid vertex type.
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT, ///< component vertex type representing a port. 
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER, ///< marshaler waypoint vertex type. 
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER, ///< demarshaler waypoint vertex type.
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND, ///< UDP send waypoint vertex type. 
    XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE ///< UDP receive waypoint vertex type. 
}
xme_core_pnp_dataLinkGraph_vertexTypes_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_vertexData_t
 * \brief the data associated to a vertex in the data link graph. 
 */
typedef struct
{
    // Common data
    xme_core_pnp_dataLinkGraph_vertexTypes_t vertexType; ///< Type of vertex. Depending on this value, select the appropriate value from the vertexData union below.
    xme_hal_graph_vertexId_t vertexId; ///< Identifier of the corresponding vertex in the data path graph.
    xme_core_node_nodeId_t nodeId; ///< Node on which the entity represented by this vertex is deployed to.
    xme_core_topic_t topicId; ///< Topic that the vertex handles.
    // Vertex specific data
    xme_core_pnp_dataLinkGraph_vertexSpecificData_t vertexData; ///< Vertex specific data (depending on the vertex type). 
}
xme_core_pnp_dataLinkGraph_vertexData_t;

//----------------------------------------------------------------------------//
//     xme_core_pnp_dataLinkGraph_edgeData_t and related                //
//----------------------------------------------------------------------------//

/**
 * \struct xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t
 * \brief this structure establishes the data required for creating logical routes. 
 */
typedef struct
{
    xme_core_channelId_t channelId; ///< the channel id associated to the logical route. 
    //route status
    bool established; ///< True if the route has been established before by calling xme_core_pnp_lrm_setLogicalRoute() on the respective channel.
    //additional information
    //4:maxSize, alpha-curve, 100:metric
//    int maxSize; // TODO: Use!
//    int alphaCurve; // TODO: Use!
//    int metric; // TODO: Use!
}
xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_memCopyEdgeData_t
 * \brief the specific data required for a memory copy operation. 
 */
typedef struct
{
    uint8_t thisIsJustAPlaceholder_PutEdgeSpecificDataHere; ///< generic data associated to mem copy operation. 
}
xme_core_pnp_dataLinkGraph_memCopyEdgeData_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_udpLinkEdgeData_t
 * \brief the specific data required for a UDP link operation. 
 */
typedef struct
{
    uint8_t key[4]; ///< the UDP key. 
}
xme_core_pnp_dataLinkGraph_udpLinkEdgeData_t;

/**
 * \union xme_core_pnp_dataLinkGraph_edgeSpecificData_t
 * \brief this structure establishes the data required for creating logical routes. 
 */
typedef union
{
    xme_core_pnp_dataLinkGraph_logicalRouteEdgeData_t logicalRouteEdge; ///< a logical route edge. 
    xme_core_pnp_dataLinkGraph_memCopyEdgeData_t memCopyEdge; ///< a memory copy edge. 
    xme_core_pnp_dataLinkGraph_udpLinkEdgeData_t udpLinkEdge; ///< a UDP link edge. 
}
xme_core_pnp_dataLinkGraph_edgeSpecificData_t;

/**
 * \enum xme_core_pnp_dataLinkGraph_edgeTypes_t
 * \brief the set of different edges types that can be stored in the data link graph. 
 */
typedef enum
{
    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE = 0, ///< invalid edge type. The edge type is not a valid type of edge for data link graph. 
    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE, ///< the edge type is a logical route. 
    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY, ///< the edge type is a memory copy, between elements inside a given node. 
    XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_UDPLINK ///< the edge type is a UDP link, between two separate nodes. 
}
xme_core_pnp_dataLinkGraph_edgeTypes_t;

/**
 * \struct xme_core_pnp_dataLinkGraph_edgeData_t
 * \brief the data associated to the graph edge in this specific graph implementation. 
 */
typedef struct
{
    // Common data
    xme_core_pnp_dataLinkGraph_edgeTypes_t edgeType; ///< Type of edge. Depending on this value, select the appropriate value from the edgeData union below.
    xme_hal_graph_edgeId_t edgeId; ///< Identifier of the corresponding edge in the data path graph.
    xme_core_topic_t topicId; ///< Topic that the edge handles.
    // Edge specific data
    xme_core_pnp_dataLinkGraph_edgeSpecificData_t edgeData; ///< Edge specific data (depending on the edge type).
}
xme_core_pnp_dataLinkGraph_edgeData_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes a structure of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] vertexType Type of vertex.
 * \param[in] vertexId Identifier of the corresponding vertex in the data path graph.
 * \param[in] nodeId Node on which the entity represented by this vertex is deployed to.
 * \param[in] topicId Topic that the vertex handles.
 * 
 * \retval XME_STATUS_SUCCESS if the structure was successfully initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the structure cannot be initialized. 
 */
xme_status_t
xme_core_pnp_dataLinkGraph_initVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData,
    xme_core_pnp_dataLinkGraph_vertexTypes_t vertexType,
    xme_hal_graph_vertexId_t vertexId,
    xme_core_node_nodeId_t nodeId,
    xme_core_topic_t topicId
);

/**
 * \brief  Initializes the specific data associated to a component port in the
 *          ::xme_core_pnp_dataLinkGraph_vertexData_t data structure.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] componentId the component identifier.
 * \param[in] portType the port type of the component port.
 * \param[in] announcement associated announcement of the component port in the logical route manager.
 * 
 * \retval XME_STATUS_SUCCESS if the structure was successfully initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the structure cannot be initialized. 
 */
xme_status_t
xme_core_pnp_dataLinkGraph_initComponentPortVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData,
    xme_core_component_t componentId,
    xme_core_component_portType_t portType,
    xme_core_pnp_lrm_announcement_t* announcement
);

/**
 * \brief  Initializes the specific data associated to a UDP send waypoint in the
 *          ::xme_core_pnp_dataLinkGraph_vertexData_t data structure.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] key the key associated to the UDP send.
 * \param[in] ip the target IP/Port address (aa.bb.cc.dd:xx) of the UDP send waypoint.
 * 
 * \retval XME_STATUS_SUCCESS if the structure was successfully initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the structure cannot be initialized. 
 */
xme_status_t
xme_core_pnp_dataLinkGraph_initWaypointUdpSendVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData,
    uint8_t key[4],
    const char* ip
);

/**
 * \brief  Initializes the specific data associated to a UDP receive waypoint in the
 *          ::xme_core_pnp_dataLinkGraph_vertexData_t data structure.
 *
 * \param[in,out] vertexData Address of the vertex data struct to initialize.
 * \param[in] key the key associated to the UDP receive.
 * \param[in] ip the host IP/Port address (aa.bb.cc.dd:xx) of the UDP receive waypoint.
 * 
 * \retval XME_STATUS_SUCCESS if the structure was successfully initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the structure cannot be initialized. 
 */
xme_status_t
xme_core_pnp_dataLinkGraph_initWaypointUdpReceiveVertexData
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData,
    uint8_t key[4],
    const char* ip
);

/**
 * \brief  Initializes a structure of type xme_core_pnp_dataLinkGraph_edgeData_t.
 *
 * \param[in,out] edgeData Address of the edge data struct to initialize.
 * \param[in] edgeType Type of edge.
 * \param[in] edgeId Identifier of the corresponding edge in the data path graph.
 * \param[in] topicId Topic that the edge handles.
 * 
 * \retval XME_STATUS_SUCCESS if the structure was successfully initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the structure cannot be initialized. 
 */
xme_status_t
xme_core_pnp_dataLinkGraph_initEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData,
    xme_core_pnp_dataLinkGraph_edgeTypes_t edgeType,
    xme_hal_graph_edgeId_t edgeId,
    xme_core_topic_t topicId
);

/**
 * \brief  Initializes a logical route and stores logical route specifics in the
*          xme_core_pnp_dataLinkGraph_edgeData_t structure.
 *
 * \param[in,out] edgeData Address of the edge data struct to initialize.
 * \param[in] channelId The channel id associated to the logical route.
 * \param[in] established Determines if the logical route is already established.
 * 
 * \retval XME_STATUS_SUCCESS if the structure was successfully initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the structure cannot be initialized. 
*/
xme_status_t
xme_core_pnp_dataLinkGraph_initLogicalRouteEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData,
    xme_core_channelId_t channelId,
    bool established
);

/**
 * \brief  Initializes a UDP link and stores it with using UDP link specific data in the
 *         xme_core_pnp_dataLinkGraph_edgeData_t structure.
 *
 * \param[in,out] edgeData Address of the edge data struct to initialize.
 * \param[in] key The key associated to the UDP link.
 * 
 * \retval XME_STATUS_SUCCESS if the structure was successfully initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the structure cannot be initialized. 
 */
xme_status_t
xme_core_pnp_dataLinkGraph_initUdpLinkEdgeData
(
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData,
    uint8_t key[4]
);

/**
 * \brief  Vertex compare callback function for checking whether the data
 *         link data associated with a vertex is identical to the one
 *         being searched for.
 *
 * \note To be used in conjunction with
 *       ::xme_hal_graph_setVertexCompareCallback().
 *
 * \param[in] vertexData1 Data associated to the first vertex.
 *            Supposed to be of type xme_core_pnp_dataLinkGraph_vertexData_t.
 * \param[in] vertexData2 Data associated to the second vertex.
 *            Supposed to be of type xme_core_pnp_dataLinkGraph_vertexData_t.
 *
 * \retval 0 if all fields except for the vertex identifier are the same.
 * \retval 1 if at least one of the fields except for the vertex identifier is different.
 */
int
xme_core_pnp_dataLinkGraph_vertexCompareIdenticalVertexData
(
    void* vertexData1,
    void* vertexData2
);

/**
 * \brief  Edge compare callback function for checking whether the data
 *         link data associated with an edge is identical to the one
 *         being searched for.
 *
 * \note To be used in conjunction with
 *       ::xme_hal_graph_setEdgeCompareCallback().
 *
 * \param[in] edgeData1 Data associated to the first edge.
 *            Supposed to be of type xme_core_pnp_dataLinkGraph_edgeData_t.
 * \param[in] edgeData2 Data associated to the second edge.
 *            Supposed to be of type xme_core_pnp_dataLinkGraph_edgeData_t.
 *
 * \retval 0 if all fields except for the edge identifier are the same.
 * \retval 1 if at least one of the fields except for the edge identifier is different.
 */
int
xme_core_pnp_dataLinkGraph_edgeCompareIdenticalEdgeData
(
    void* edgeData1,
    void* edgeData2
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
 * \param[in] dataLinkGraph Graph to dump.
 */
void
xme_core_pnp_dataLinkGraph_dumpDataLinkGraph
(
    xme_log_severity_t severity,
    const char* graphName,
    xme_hal_graph_graph_t* dataLinkGraph
);

/**
 * \brief  Vertex compare callback function that returns always match
 *         between vertices to compare.
 *
 * \param[in] vertexData1 Data associated to the first vertex.
 * \param[in] vertexData2 Data associated to the second vertex.
 *
 * \retval 0 return always true.
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
 * \retval 0 return always true.
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
 * \retval -1 if the first edge data not a logical route, or if second edge
 *         data is null-valued.
 */
int
xme_core_pnp_dataLinkGraph_edgeCompareFilterLogicalRoutes
(
    void* edgeData1,
    void* edgeData2
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_DATALINKGRAPH_H
