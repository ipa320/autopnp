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
 * $Id: plugAndPlayManagerInternalTypes.h 6179 2013-12-19 15:34:47Z geisinger $
 */

/**
 * \file
 *         Plug and Play Manager internal types.
 */

#ifndef XME_CORE_PNP_PLUGANDPLAYMANAGERINTERNALTYPES_H
#define XME_CORE_PNP_PLUGANDPLAYMANAGERINTERNALTYPES_H

/**
 * \addtogroup core_pnp_pnpManager
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/plugAndPlay/include/dataLinkGraph.h"
#include "xme/core/plugAndPlay/include/networkConfigurationCalculator.h"

#include "xme/core/component.h"
#include "xme/core/node.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/linkedList.h"
#include "xme/hal/include/table.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \struct xme_core_pnp_pnpManager_physicalGraphListItem_t
 *
 * \brief Graph list item.
 *
 * \details Type of the items in the singly linked list of type
 *          xme_core_pnp_pnpManager_graphList_t.
 */
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< the node id. 
    xme_core_pnp_ncc_physicalRoutes_t* splitGraph; ///< the splitted graph.
}
xme_core_pnp_pnpManager_physicalGraphListItem_t;

/**
 * \brief Graph list.
 *
 * \details For use in ::xme_core_pnp_pnpManager_splitGraph().
 */
typedef xme_hal_singlyLinkedList_t(XME_CORE_PNP_PNPMANAGER_PHYSICALROUTES_LIST_SIZE) xme_core_pnp_pnpManager_physicalGraphList_t;

/**
 * \struct xme_core_pnp_pnpManager_vertexItem_t
 * \brief a temporary structure to store the vertex index associated to each
 *        vertex type
 */
typedef struct
{
    uint8_t index; ///< the index in which the vertex is stored. 
    xme_core_pnp_dataLinkGraph_vertexTypes_t vertexType; ///< the vertex type. 
    xme_hal_graph_vertexId_t vertexId; ///< the vertex identifier in original graph. 
} xme_core_pnp_pnpManager_vertexItem_t;

/**
 * \struct xme_core_pnp_pnpManager_rtGraphItem_t
 * \brief structure to store the runtime graph to be sent to other nodes. 
 */
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< the target node id. 
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph; ///< the runtime graph associated to the node. 
    bool announced; ///< temporary hack for getting the announced graphs.
} xme_core_pnp_pnpManager_rtGraphItem_t;

#if 0
/**
 * \enum xme_core_pnp_pnpManager_nodeComponentInstance_status_t
 *
 * \brief Enumerator specifying state of the component in instance list
 */
typedef enum
{
    XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_INVALID_STATUS = 0, ///< the instance cannot be instantiated on target node. 
    XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_REQUESTED, ///< the user has already send a request to develop some component in some node. 
    XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_PREPARED, ///< the topic connections has been calculated and it is prepared to send back to the target node. 
    XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_SUBMITTED, ///< the function has requested the runtime graph. 
    XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_RUNNING, ///< the component was instantiated in the target node and running. 
    XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_FAILURE ///< the component failed in instantiation on the target node. 
}
xme_core_pnp_pnpManager_nodeComponentInstance_status_t;
#endif

/**
 * \struct xme_core_pnp_pnpManager_nodeComponentInstance_t
 *
 * \brief Instance List entry
 */
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< the node identifier. 
    xme_core_component_t componentId; ///< the component identifier. 
    xme_core_componentType_t componentType; ///< the associated component type. 
    //xme_core_pnp_pnpManager_nodeComponentInstance_status_t status; ///< the instance current status. // TODO: To be discussed regarding plug&play and plug-off process.
    char* initializationString; ///< Component-specific initialization data.
}
xme_core_pnp_pnpManager_nodeComponentInstance_t;

/**
 * \struct xme_core_pnp_pnpManager_lastAssignedComponentIdPerNode_t
 *
 * \brief Last assigned component id per node entry. 
 * \details Provides the latest assigned component id per node. This value
 *          will be taken into account to provide new component id for new request. 
 */
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< the node identifier. 
    xme_core_component_t lastAssignedComponentId; ///< the last assigned component id for that node. 
}
xme_core_pnp_pnpManager_lastAssignedComponentIdPerNode_t;

/**
 * \struct xme_core_pnp_pnpManager_unestablishedLogicalRoute_t
 *
 * \brief  This structure stores all the logical routes that were calculated
 *         but that are not marked as established. 
 */
typedef struct
{
    xme_hal_graph_edgeId_t edgeId; ///< The edge id of the logical route. 
    xme_core_channelId_t channelId; ///< The associated channel id.
} xme_core_pnp_pnpManager_unestablishedLogicalRoute_t;

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_PLUGANDPLAYMANAGERINTERNALTYPES_H
