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
 * $Id: logicalRouteManager.h 5206 2013-09-27 11:48:07Z ruiz $
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

#include "xme/core/coreTypes.h"
#include "xme/core/node.h"
#include "xme/core/component.h"
#include "xme/core/container.h"

#include "xme/core/dataManagerTypes.h"
#include "xme/core/directory/include/attribute.h"

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
    xme_core_component_portType_t portType; ///< the port type (subscription, publication, etc.)
    uint16_t portIndex; ///< internal port index inside the component. 
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
 * \brief  Initializes the logical route manager component.
 *         Exactly one component of this type must be present on every node.
 * 
 * \param params Initialization parameters for logical route manager component (TBD)
 * 
 * \retval XME_SUCCESS if the logical route manager component has been properly initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if the logical route manager component initialization failed.
 */ 
xme_status_t
xme_core_pnp_lrm_init
(
    void *params
);

/**
 * \brief  Frees all resources occupied by the logical route manager component.
 *         Exactly one component of this type must be present on every node.
 */
void
xme_core_pnp_lrm_fini(void);

/**
 * \brief  This function announces the presence of a port to the Logical Route
 *         Manager.
 *
 * \details This function is meant to be called by the Plug & Play Manager.
 *
 * \param  nodeId Unique identifier of the node the port is announced on.
 * \param  componentId Node-local identifier of the component that has the
 *         announced port. If the component is already running, the component
 *         id is not temporary. 
 * \param  portType Type of the port (e.g., publication, subscription).
 * \param  portIndex Index of port inside the component.
 * \param  topicId Topic associated to the port.
 * \param  attrSet Attribute set associated to the topic.
 * \param  transactionId Transaction identifier this announcement belong to.
 *         This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *         in order to indicate that this announcement can not be rolled back.
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
    xme_core_component_portType_t portType,
    uint16_t portIndex,
    xme_core_topic_t topicId,
    xme_core_directory_attributeSetHandle_t attrSet,
    xme_core_transactionId_t transactionId
);

/**
 * \brief  Notifies the Logical Route Manager that a given node and container
 *         identifier has changed.
 *
 * \param  oldNodeId Unique node identifier as it was previously used.
 * \param  oldContainerId Node-local container identifier as it was previously
 *         used.
 * \param  newNodeId New unique node identifier.
 * \param  newContainerId New node-local container identifier.
 * \param  transactionId Transaction identifier this operation belong to.
 *         This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *         in order to indicate that this announcement can not be rolled back.
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
 * \brief  This function is used to discard a previously allocated channel
 *         identifier in case the channel could not be instantiated, but the
 *         overall transaction should not be rolled back.
 *
 * \param  channelId Channel identifier to discard. 
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
 * \brief  Generates a channel identifier to connect two ports. 
 *
 * \param[in]  transactionId Transaction identifier this operation belong to.
 *         This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *         in order to indicate that this announcement can not be rolled back.
 * \param[out]  outChannelId The generated channel identifier.
 *
 * \retval XME_STATUS_SUCCESS if the channel identifier was successfully
 *         generated.
 * \retval XME_STATUS_INVALID_PARAMETER if the transaction identifier was
 *         invalid or outChannelId was NULL.
 * \retval XME_STATUS_INTERNAL_ERROR if the new channel identifier could not be
 *         generated.
 */
xme_status_t
xme_core_pnp_lrm_generateChannelId
(
    xme_core_transactionId_t transactionId,
    xme_core_channelId_t* outChannelId
);

/**
 * \brief  Returns a directed graph containing logical data routes in the
 *         network based on the previously issued port announcements.
 *
 * Every vertex in the output graph represents one port. An edge connects
 * an output port with an input port if and only if the topic and
 * asscociated attributes of the respective ports match.
 *
 * \param[in]  transactionId Transaction identifier this operation belong to.
 *         This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *         in order to indicate that this announcement can not be rolled back.
 * \param[out]  outRouteCandidates Address of a directed graph data structure
 *         that will be filled with the set of logical routes in the
 *         network. The graph should be empty and non-null.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successfully executed.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the parameters was invalid.
 * \retval XME_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_pnp_lrm_getLogicalRoutes
(
    xme_core_transactionId_t transactionId,
    xme_core_pnp_lrm_logicalRoutes_t* outRouteCandidates
);

/**
 * \brief  Sets the data channel identifier or the logical route between
 *         the given output port (source) and input port (sink).
 *
 * \param  edgeId Edge identifier representing the logical data channel to commit.
 * \param  channelId Channel identifier to assign to the logical route.
 * \param  transactionId Transaction identifier this operation belong to.
 *         This value may be set to XME_CORE_TRANSACTION_INVALID_TRANSACTION_ID
 *         in order to indicate that this announcement can not be rolled back.
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

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_LOGICALROUTEMANAGER_H
