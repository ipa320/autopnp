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
 * $Id: logicalRouteManager.h 7828 2014-03-14 09:32:09Z ruiz $
 */

/**
 * \file
 *         Logical Route Manager.
 */

#ifndef XME_CORE_PNP_LOGICALROUTEMANAGER_H
#define XME_CORE_PNP_LOGICALROUTEMANAGER_H

/**
 * \defgroup core_pnp_lrm Logical Route Manager group. 
 * @{
 *
 * \brief Logical Route Manager stores the information about component announcement
 *        and provides information about logical routes based on these announcements. 
 *
 * \details The logical route manager stores the logical routes between components
 *          publishing data required by other subscriber components. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/component.h"
#include "xme/core/container.h"
#include "xme/core/coreTypes.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/core/directory/include/attribute.h"
#include "xme/core/manifestTypes.h"
#include "xme/core/node.h"

#include "xme/hal/include/graph.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_core_pnp_lrm_logicalRoutes_t
 *
 * \brief  Directed graph containing route candidates output by Logical Route
 *         Manager.
 * \details routeCandidate has to represent { (a, 5, 1, p0) -- (b, 7, 1, p0), 4:maxSize, alpha-curve, 100:metric }
 */
typedef xme_hal_graph_graph_t xme_core_pnp_lrm_logicalRoutes_t;

/**
 * \typedef xme_core_component_portAddress_t
 *
 * \brief  Unique identification of a port.
 */
// TODO: Replace with real typedef (i.e., contains probably node id, container id, component id and port id)
// FIXME: Who assigns the port address?
typedef int xme_core_component_portAddress_t;

/**
 * \struct xme_core_pnp_lrm_announcement_t
 *
 * \brief This is the internal representation of an announcement of a logial route. 
 * \details This structure will contain all the information required by the logical 
 *          route to calculate established routes. 
 */
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< TODO: This is redundant with the respective member in xme_core_pnp_lrm_vertexData_t; it should probably be removed.
    xme_core_component_t componentId; ///< the component identifier. if the component is an non-existing component (e.g. plug phase), this is a temporary component id. 
    xme_core_componentType_t componentType; ///< the component type.
    xme_core_component_portType_t portType; ///< the port type (subscription, publication, etc.)
    uint16_t portIndex; ///< internal port index inside the component.
    xme_core_component_connectionBound_t lowerConnectionBound; ///< Lower connection bound. \see xme_core_component_connectionBound_t
    xme_core_component_connectionBound_t upperConnectionBound; ///< Upper connection bound. \see xme_core_component_connectionBound_t
    xme_core_topic_t topicId; ///< the topic id. 
    xme_core_directory_attributeSetHandle_t attrSet; ///< Attribute set associated to the announcement.
    xme_core_transactionId_t transactionId; ///< the assigned transaction id to the current announcement. 
}
xme_core_pnp_lrm_announcement_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the logical route manager component.
 *
 * \param params Initialization parameters for logical route manager component (TBD).
 * 
 * \retval XME_STATUS_SUCCESS if initialization was successful.
 * \retval XME_STATUS_OUT_OF_RESOURCES if the initialization failed.
 */ 
xme_status_t
xme_core_pnp_lrm_init
(
    void *params
);

/**
 * \brief Frees all resources occupied by the logical route manager component.
 */
void
xme_core_pnp_lrm_fini(void);

/**
 * \brief This function announces the presence of a port to the Logical Route
 *        Manager.
 *
 * \details This function is meant to be called by the Plug & Play Manager.
 *
 * \param[in] nodeId Unique identifier of the node the port is located on.
 * \param[in] componentId Node-local identifier of the component that has the
 *            announced port. If the component is already running, the component
 *            id is not temporary. 
 * \param[in] componentType The component type. 
 * \param[in] portType Type of the port (e.g., publication, subscription).
 * \param[in] portIndex Zero-based index of port inside the component.
 * \param[in] lowerConnectionBound The lower connection bound.
 * \param[in] upperConnectionBound The upper connection bound.
 * \param[in] topicId Topic associated to the port.
 * \param[in] attrSet Attribute set associated to the port.
 * \param[in] transactionId Transaction identifier this announcement belong to.
 *        This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *        in order to indicate that this announcement can not be rolled back.
 *
 * \retval XME_STATUS_SUCCESS if the port is successfully announced.
 * \retval XME_STATUS_INVALID_PARAMETER if a parameter was set to an invalid
 *         value.
 * \retval XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to announce the port.
 */
xme_status_t
xme_core_pnp_lrm_announcePort
(
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t componentId,
    xme_core_componentType_t componentType,
    xme_core_component_portType_t portType,
    uint16_t portIndex,
    xme_core_component_connectionBound_t lowerConnectionBound,
    xme_core_component_connectionBound_t upperConnectionBound,
    xme_core_topic_t topicId,
    xme_core_directory_attributeSetHandle_t attrSet,
    xme_core_transactionId_t transactionId
);

/**
 * \brief This function unannounces all ports associated to a given node identifier.
 *
 * \details The announced ports associated to the node identifier provided as parameter
 *          are removed from the publications/subscriptions.
 *
 * \param nodeID Unique identifier of the node the port is located on.
 * \param componentID The component identifier to unannounce. 
 * \param transactionId Transaction identifier this announcement belong to.
 *        This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *        in order to indicate that this announcement can not be rolled back.
 *
 * \retval XME_STATUS_SUCCESS if all the announcements were unannounced.
 * \retval XME_STATUS_INVALID_PARAMETER if a parameter was set to an invalid
 *         value.
 * \retval XME_STATUS_NOT_FOUND if there are no announcements for the given 
 *         node identifier. 
 * \retval XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to announce the port.
 */
xme_status_t
xme_core_pnp_lrm_unannouncePortForComponent
(
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID,
    xme_core_transactionId_t transactionId
);

/**
 * \brief Notifies the Logical Route Manager that a given node and container
 *        identifier has changed.
 *
 * \param oldNodeId Unique node identifier as it was previously used.
 * \param oldContainerId Node-local container identifier as it was previously
 *        used.
 * \param newNodeId New unique node identifier.
 * \param newContainerId New node-local container identifier.
 * \param transactionId Transaction identifier this operation belong to.
 *        This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *        in order to indicate that this announcement can not be rolled back.
 *
 * \retval XME_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_STATUS_NOT_FOUND if the given combination of node and container
 *         identifier was not found.
 * \retval XME_STATUS_INTERNAL_ERROR if the operation failed.
 */
xme_status_t
xme_core_pnp_lrm_replaceContainerId
(
    xme_core_node_nodeId_t oldNodeId,
    xme_core_container_t oldContainerId,
    xme_core_node_nodeId_t newNodeId,
    xme_core_container_t newContainerId,
    xme_core_transactionId_t transactionId
);

/**
 * \brief This function is used to discard a previously allocated channel
 *        identifier in case the channel could not be instantiated, but the
 *        overall transaction should not be rolled back.
 *
 * \param channelId Channel identifier to discard. 
 *
 * \retval XME_STATUS_SUCCESS if the channel identifier was successfully
 *         discarded. 
 * \retval XME_STATUS_NOT_FOUND if the given combination channel identifier
 *         was not found.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot discard provided channel id. 
 */
xme_status_t
xme_core_pnp_lrm_discardChannelId
(
    xme_core_channelId_t channelId
);

/**
 * \brief Returns a directed graph containing logical data routes in the
 *        network based on the previously issued port announcements.
 *
 * \details Every vertex in the output graph represents one port. An edge connects
 *          an output port with an input port if and only if the topic and
 *          asscociated attributes of the respective ports match.
 *
 * \param[in] transactionId Transaction identifier this operation belong to.
 *            This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *            in order to indicate that this announcement can not be rolled back.
 * \param[out] logicalRoutes Address of a directed graph data structure
 *             that will be filled with the set of logical routes in the
 *             network. The initial graph should be empty and non-null.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successfully executed.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the parameters was invalid.
 * \retval XME_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_pnp_lrm_getLogicalRoutes
(
    xme_core_transactionId_t transactionId,
    xme_core_pnp_lrm_logicalRoutes_t* logicalRoutes
);

/**
 * \brief Gets the a directed graph of logical routes that are associated 
 *        to the provided node identifier and component identifier. 
 *
 * \details Checks all routes that are associated to a given node. This includes
 *          logical routes associated to publications and subscriptions. 
 * \note The returned logical routes are labeled as established routes. 
 *
 * \param[in] transactionId Transaction identifier this operation belong to.
 *            This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *            in order to indicate that this announcement can not be rolled back.
 * \param[in] nodeID The node identifier. 
 * \param[in] componentID The component identifier. 
 * \param[out] logicalRoutes Address of a directed graph data structure
 *             that will be filled with the set of logical routes in the
 *             network. The initial graph should be empty and non-null.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successfully executed.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the parameters was invalid.
 * \retval XME_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_pnp_lrm_getEstablishedLogicalRoutesForComponent
(
    xme_core_transactionId_t transactionId,
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID,
    xme_core_pnp_lrm_logicalRoutes_t* logicalRoutes
);

/**
 * \brief Sets the data channel identifier or the logical route between
 *        the given output port (source) and input port (sink).
 *
 * \param edgeId Edge identifier representing the logical data channel to commit.
 * \param channelId Channel identifier to assign to the logical route.
 * \param transactionId Transaction identifier this operation belong to.
 *        This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *        in order to indicate that this announcement can not be rolled back.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successfully executed. 
 * \retval XME_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_pnp_lrm_setLogicalRoute
(
    xme_hal_graph_edgeId_t edgeId,
    xme_core_channelId_t channelId,
    xme_core_transactionId_t transactionId
);

/**
 * \brief Removes a logical route associated to the given data channel identifier.
 *
 * \details Explores all established logical routes, and removes all thos identified
 *          by the channel identifier. 
 *
 * \param channelId Channel identifier to assign to the logical route.
 * \param transactionId Transaction identifier this operation belong to.
 *        This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *        in order to indicate that this announcement can not be rolled back.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successfully executed. 
 * \retval XME_STATUS_NOT_FOUND if there are no established routes matching the 
 *         channel identifier. 
 * \retval XME_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_pnp_lrm_removeLogicalRoute
(
    xme_core_channelId_t channelId,
    xme_core_transactionId_t transactionId
);

/**
 * \brief Obtains all outgoing or incoming routes associated to a given vertex data.
 * \details Given a publisher or a subscriber vertex data, obtains the set of direct
 *          routes associated to that vertex. If the given vertex data is a publisher,
 *          it obtains the set of subscribers. If the given vertex data is a subscriber,
 *          the returned data is filled with associated publishers. The output variable
 *          will contain as first vertex the provided vertex data, and the edges will
 *          represent all associated data. For publishers, we should explore outgoingEdges
 *          and for subscribers, incomingEdges.
 *
 * \param[in] announcement The announcement associated to the publication or subscription.
 * \param[out] outSubgraph The set of logical routes associated to that vertex data.
 *
 * \retval XME_STATUS_SUCCESS if the generated logical routes are created successfully.
 * \retval XME_STATUS_NOT_FOUND if the vertex data is not associated to any logical route.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot complete the operation due to an internal error.
 * \retval XME_STATUS_INVALID_PARAMETER if the provided parameters are not valid parameters.
 */
xme_status_t
xme_core_pnp_lrm_getAssociatedVertices
(
    xme_core_pnp_lrm_announcement_t* const announcement,
    xme_core_pnp_lrm_logicalRoutes_t* outSubgraph
);

/**
 * \brief Gets a channel identifier for two connected ports.
 *        When a channel ID has already been announced for the given publisher/subscriber
 *        pair, then that value will be returned.
 *        (This table is filled by XMT for channel IDs generated during static route
 *        calculation.)
 *        Otherwise we generate a new channel ID using xme_core_pnp_lrm_generateChannelID.
 *
 * \param[in] transactionID Transaction identifier this operation belong to.
 *            This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *            in order to indicate that this announcement can not be rolled back.
 * \param[in] publicationNodeID Node ID of the publisher port.
 * \param[in] publicationComponentID component ID for the publisher port.
 * \param[in] publicationPortIndex Port index of the publisher port for that component.
 * \param[in] subscriptionNodeID Node ID of the subscriber port.
 * \param[in] subscriptionComponentID component ID for the subscriber port.
 * \param[in] subscriptionPortIndex Port index of the subscriber port for that component.
 * \param[out] outChannelID The generated channel identifier.
 *
 * \retval XME_STATUS_SUCCESS if the channel identifier was successfully
 *         generated.
 * \retval XME_STATUS_INVALID_PARAMETER if the transaction identifier was
 *         invalid or outChannelId was NULL.
 * \retval XME_STATUS_INTERNAL_ERROR if the new channel identifier could not be
 *         generated.
 */
xme_status_t
xme_core_pnp_lrm_getChannelID
(
    xme_core_transactionId_t transactionID,
    xme_core_node_nodeId_t publicationNodeID,
    xme_core_component_t publicationComponentID,
    uint16_t publicationPortIndex,
    xme_core_node_nodeId_t subscriptionNodeID,
    xme_core_component_t subscriptionComponentID,
    uint16_t subscriptionPortIndex,
    xme_core_channelId_t* outChannelID
);

/**
 * \brief This function announces the channel IDs for the given set of publisher
 *        and subscriber ports.
 *
 * \details This function is meant to be called by the Plug & Play Manager.
 *
 * \param[in] publicationNodeID Node ID of the publisher port.
 * \param[in] publicationComponentID component ID for the publisher port.
 * \param[in] publicationPortIndex Portindex of the publisher port for that component.
 * \param[in] subscriptionNodeID Node ID of the subscriber port.
 * \param[in] subscriptionComponentID component ID for the subscriber port.
 * \param[in] subscriptionPortIndex Portindex of the subscriber port for that component.
 * \param[in] channelID The channel identifier.
 *
 * \retval XME_STATUS_SUCCESS if the channelID is successfully announced.
 * \retval XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to announce the channelID.
 */
xme_status_t
xme_core_pnp_lrm_announceChannelID
(
    xme_core_node_nodeId_t publicationNodeID,
    xme_core_component_t publicationComponentID,
    uint16_t publicationPortIndex,
    xme_core_node_nodeId_t subscriptionNodeID,
    xme_core_component_t subscriptionComponentID,
    uint16_t subscriptionPortIndex,
    xme_core_channelId_t channelID
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_LOGICALROUTEMANAGER_H
