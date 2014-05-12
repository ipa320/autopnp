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
 * $Id: plugAndPlayManagerInternalMethods.h 7694 2014-03-06 15:56:24Z wiesmueller $
 */

/**
 * \file
 *         Plug and Play Manager Internal Methods.
 */

#ifndef XME_CORE_PNP_PLUGANDPLAYMANAGERINTERNALMETHODS_H
#define XME_CORE_PNP_PLUGANDPLAYMANAGERINTERNALMETHODS_H

/**
 * \addtogroup core_pnp_pnpManager 
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/plugAndPlay/include/plugAndPlayManagerInternalTypes.h"

#include "xme/core/manifestRepository/include/manifestRepository.h"
#include "xme/core/topicData.h"

#include "xme/defines.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Announces all ports in all components in the given manifest to the Logical Route Manager.
 *
 * \details After calling this function, ::xme_core_pnp_lrm_getLogicalRoutes()
 *          may be called to obtain the possible logical routes for the newly announced ports.
 *
 * \param[in] nodeId Unique node identifier of node where to place the component.
 * \param[in] componentId Component identifier which will be used for port announcement
 *            in logical route manager.
 * \param[in] componentType Component type of the given component.
 *
 * \retval XME_STATUS_SUCCESS if the corresponding ports were announced to the LRM.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot announce ports. 
 */
xme_status_t
xme_core_pnp_pnpManager_announcePorts
(
    xme_core_node_nodeId_t nodeId, 
    xme_core_component_t componentId,
    xme_core_componentType_t componentType
);

/**
 * \brief  Splits the input waypointGraph into graphs split according to nodeId
 *
 * \param[in] physicalRouteGraph input waypoint graph, created by the NCC, which can 
 *            possibly contain components from different nodes connected by edges.
 * \param[out] outSplitGraphs array of subgraphs of the input waypointGraph, such 
 *             that each subgraph contains only components from one node, and the
 *             corresponding edges.
 *
 * \retval XME_STATUS_SUCCESS if the array outSplitGraphs has been computed
 * \retval XME_STATUS_INVALID_PARAMETER if the input waypointGraph is not supplied, 
 *         or outSplitGraphs is NULL.
 */
xme_status_t
xme_core_pnp_pnpManager_splitGraph
(
    xme_core_pnp_ncc_physicalRoutes_t* physicalRouteGraph,
    xme_core_pnp_pnpManager_physicalGraphList_t* outSplitGraphs
);

/**
 * \brief Process the input graph and transforms into runtime graph topic. 
 * \note this function should be called with the splitted graphs obtained from
 *       ::xme_core_pnp_pnpManager_splitGraph().
 *
 * \param[in] graph the runtime graph.
 * \param[in] nodeId the targeted node id for the graph. 
 * \param[in] action The operation to do with the graph. 
 *
 * \retval XME_STATUS_SUCCESS if everything is working properly. 
 * \retval others in case of failure (TBD).
 */
xme_status_t 
xme_core_pnp_pnpManager_processGraph
(
    xme_hal_graph_graph_t* graph,
    xme_core_node_nodeId_t nodeId,
    uint32_t action
);

/**
 * \brief Creates a component vertex in the graph topic.
 *
 * \param rtGraph The runtime subgraph model in which is stored the graph topic
 *        information.
 * \param componentPortData The graph vertex data obtained from route calculation.
 * \param componentHandle Handle of the component to which the created component port
 *        vertex belongs.
 * \param internalIndex Index of the component vertex in the rtGraph.vertex array.
 *
 * \retval XME_STATUS_SUCCESS if the component has been added to the graph topic.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the component in the graph topic.
 */
xme_status_t
xme_core_pnp_pnpManager_createComponent
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_core_pnp_dataLinkGraph_componentPortVertexData_t componentPortData,
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    uint8_t internalIndex
);

/**
 * \brief Creates a marshaller waypoint vertex in the graph topic.
 *
 * \param rtGraph The runtime subgraph model in which is stored the graph topic
 *        information.
 * \param internalIndex internal index for the marshaller in the runtime graph topic
 *        that is used to identify the marshaller.
 * \param inputPortQueueSize Queue size of the input port of the respective marshaler
 *        input port.
 *
 * \retval XME_STATUS_SUCCESS if the marshaller vertex has been added to the graph topic.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the marshaller in the graph topic.
 */
xme_status_t
xme_core_pnp_pnpManager_createMarshalerWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t internalIndex,
    uint8_t inputPortQueueSize
);

/**
 * \brief Creates a demarshaler waypoint vertex in the graph topic.
 *
 * \note Currently, marshaler and demarshaler code is similar, except for the
 *       vertex type. In the future, if boths components use the same set of
 *       parameters, we should fusion this method with ::xme_pnpManager_createMarshalerWaypoint().
 *
 * \param rtGraph The runtime subgraph model in which is stored the graph topic
 *        information.
 * \param internalIndex internal index for the demarshaler in the runtime graph topic
 *        that is used to identify the demarshaler.
 * \param inputPortQueueSize Queue size of the demarshaler configuration input port.
 *
 * \retval XME_STATUS_SUCCESS if the demarshaler vertex has been added to the graph topic. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the demarshaler in the graph topic. 
 */
xme_status_t
xme_core_pnp_pnpManager_createDemarshalerWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t internalIndex,
    uint8_t inputPortQueueSize
);

/**
 * \brief Creates a UDP send waypoint vertex in the graph topic.
 *
 * \param rtGraph The runtime subgraph model in which is stored the graph topic
 *        information.
 * \param key the key corresponding to the UDP send waypoint.
 * \param address the IP address targeted to send the data.
 * \param internalIndex internal index for the marshaller in the runtime graph topic
 *        that is used to identify the marshaller.
 *
 * \retval XME_STATUS_SUCCESS if the UDP send vertex has been added to the graph topic. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the UDP send in the graph topic. 
 */
xme_status_t
xme_core_pnp_pnpManager_createUDPSendWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t key[4],
    xme_com_interface_address_t address,
    uint8_t internalIndex
);

/**
 * \brief Creates a UDP receive waypoint vertex in the graph topic.
 *
 * \note Currently, UDP send and UDP receive code is similar, except for the
 *       vertex type. In the future, if boths components use the same set of
 *       vertex config data, we should fusion this method with ::xme_pnpManager_createUDPSendWaypoint().
 *
 * \param rtGraph The runtime subgraph model in which is stored the graph topic
 *        information.
 * \param key the key corresponding to the UDP send waypoint.
 * \param address the IP address targeted to send the data.
 * \param internalIndex internal index for the marshaller in the runtime graph topic
 *        that is used to identify the marshaller.

 * \retval XME_STATUS_SUCCESS if the UDP send vertex has been added to the graph topic.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the UDP send in the graph topic.
 */
xme_status_t
xme_core_pnp_pnpManager_createUDPReceiveWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t key[4],
    xme_com_interface_address_t address,
    uint8_t internalIndex
);

/**
 * \brief Creates a channel injector waypoint vertex in the runtime graph topic.
 *
 * \param rtGraph The runtime subgraph model in which is stored the graph topic
 *        information.
 * \param channelId The channel that is used for the request. This channel will be
 *        used as source channel for the request-response in the channel mapping stored
 *        by the channel selector waypoint.
 * \param inputPortQueueSize Queue size of the demarshaler configuration input port.
 * \param internalIndex internal index for the channel injector in the runtime graph topic
 *        that is used to identify the channel injector waypoint.
 *
 * \retval XME_STATUS_SUCCESS if the channel injector vertex has been added to the graph topic.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the channel injector in the graph topic.
 */
xme_status_t
xme_core_pnp_pnpManager_createChannelInjectorWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_core_channelId_t channelId,
    uint8_t inputPortQueueSize,
    uint8_t internalIndex
);

/**
 * \brief Creates a channel selector waypoint vertex in the graph topic.
 *
 * \param rtGraph The runtime subgraph model in which is stored the graph topic
 *        information.
 * \param sourceChannelId The channel identifying the request channel in the request-response.
 * \param sinkChannelId The channel identifying the response channel in the request-response.
 * \param inputPortQueueSize Queue size of the demarshaler configuration input port.
 * \param internalIndex Internal index for the channel selector in the runtime graph topic
 *        that is used to identify the channel selector waypoint.
 *
 * \retval XME_STATUS_SUCCESS if the channel selector vertex has been added to the graph topic.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the channel selector in the graph topic.
 */
xme_status_t
xme_core_pnp_pnpManager_createChannelSelectorWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_core_channelId_t sourceChannelId,
    xme_core_channelId_t sinkChannelId,
    uint8_t inputPortQueueSize,
    uint8_t internalIndex
);

/**
 * \brief Creates a mem copy edge between two vertices.
 *
 * \param rtGraph The runtime subgraph model in which is stored the graph topic
 *        information.
 * \param srcVertexIndex the source vertex index in the runtime graph topic. 
 * \param dstVertexIndex the destination vertex index in the runtime graph topic. 
 * \param topicId topic identifier for the waypoint. 
 * \param channelID channel identifier for the edge. 
 * \param vertexIndex the linked list with the vertex index data. each item
 *        of the linked list is defined in ::xme_core_pnp_pnpManager_vertexItem struct.
 * \param internalIndex internal index in the runtime graph topic
 *        that is used to identify the edge. 
 * \retval XME_STATUS_SUCCESS if the memory copy edge has been added to the graph topic. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the memory copy edge in the graph topic. 
 */
xme_status_t
xme_core_pnp_pnpManager_createMemCopyEdge
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_hal_graph_vertexId_t srcVertexIndex,
    xme_hal_graph_vertexId_t dstVertexIndex,
    xme_core_topic_t topicId,
    xme_core_channelId_t channelID,
    xme_hal_linkedList_descriptor_t* vertexIndex,
    uint8_t internalIndex
);

#if 0
/**
 * \brief Checks if a component instance is in status running. 
 * 
 * \param nodeID the target node identifier. 
 * \param componentID the component identifier. 
 *
 * \retval true if the component instance is in status running. 
 * \retval false if the component instance is not running or either there is no registry
 *         associated to that component in the instance list. 
 */
bool 
xme_core_pnp_pnpManager_isComponentInstanceRunning
(
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID
);
#endif

// TODO
#if 0
/**
 * \brief Updates the status of a given component instance. 
 * 
 * \param nodeId the node id. 
 * \param componentId the component id. 
 * \param status the targeted status. 
 *
 * \retval XME_STATUS_SUCCESS if the instance status has been update successfully. 
 * \retval XME_STATUS_NOT_FOUND if it does not exists the provided combination of node id and component id. 
 */
xme_status_t
xme_core_pnp_pnpManager_updateInstance
(
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t componentId,
    xme_core_pnp_pnpManager_nodeComponentInstance_status_t status
);
#endif

/**
 * \brief Gets the instance identifier for a from a given graph. 
 * \details The graph should contain only one single component. If there are more than
 *          one component, it will be returned only the first appearance of the instance
 *          in a component. 
 * \param rtGraph the runtime graph. 
 * 
 * \return the instance id of the given runtime graph. If there are no entries,
 *         the XME_CORE_PNP_PNPMANAGER_INVALID_INSTANCE_ID is returned instead. 
 */
xme_core_component_t
xme_core_pnp_pnpManager_getComponentIdFromRTGraph
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph
);

/**
 * \brief Checks if a node is registered in the PnPManager. 
 *
 * \param nodeId the node id to check. 
 *
 * \retval true if the node is registered in the plug and play manager. 
 * \retval false if the node is not registered in the plug and play manager. 
 */
bool
xme_core_pnp_pnpManager_isNodeRegistered
(
    xme_core_node_nodeId_t nodeId
);

/**
 * \brief  Stores all unestablished logical routes in a table.
 *
 * \details The information stored about logical routes are the logical route graph edgeId
 *          and the associated channel id.
 *
 * \note The storage is used later to set the route as established in the 
 *       logical route manager. 
 *       No update of the logical route graph may happen until then, as such an
 *       update can invalidate the vertex and edge IDs stored here.
 *
 * \param  logicalRoutes The graph with the logical routes. 
 *
 * \retval XME_STATUS_SUCCESS if all the logical routes are stored in the table.
 * \retval XME_STATUS_INTERNAL_ERROR if the operation cannot be completed. 
 * \retval XME_STATUS_OUT_OF_RESOURCES if the information cannot be stored in the table. 
 */
xme_status_t
xme_core_pnp_pnpManager_storeUnestablishedLogicalRoutes
(
    xme_core_pnp_lrm_logicalRoutes_t* logicalRoutes
);

/**
 * \brief Gets a new component id for a given node. 
 * \note This method increments the value of last assigned component id in the node. 
 *
 * \param nodeId the node identifier. 
 *
 * \return the new assigned component id. 
 */
xme_core_component_t
xme_core_pnp_pnpManager_generateNewComponentIdForNode
(
    xme_core_node_nodeId_t nodeId
);

/**
 * \brief Update logical routes.
 *
 * \details Peforms the following steps:
 *          - Triggers logical route manager to recalculate routes.
 *          - Calls configuration extensions to update logical route graph.
 *
 * \param[out] logicalRouteGraph Unitialized logical route graph where results
 *             will be stored. May not be NULL.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successful.
 */
xme_status_t
xme_core_pnp_pnpManager_updateLogicalRoutes
(
    xme_core_pnp_lrm_logicalRoutes_t* logicalRouteGraph
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_PLUGANDPLAYMANAGERINTERNALMETHODS_H
