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
 * $Id: plugAndPlayManager.c 7829 2014-03-14 10:29:33Z ruiz $
 */

/**
 * \file
 *         Plug and Play Manager.
 */

/**
 * \addtogroup core_pnp_pnpManager
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/plugAndPlayManager.h"
#include "xme/core/plugAndPlay/include/plugAndPlayManagerInternalMethods.h"

#include "xme/core/directory/include/topicRegistry.h"
#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpManagerInterface.h"
#include "xme/core/plugAndPlay/include/configuratorExtension.h"

#include "xme/core/plugAndPlay/include-gen/pnpManagerManifest.h"

#include "xme/hal/include/net.h"

#include "xme/core/logUtils.h"

#include <inttypes.h>

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \brief Table xme_core_pnp_pnpManager_nodeList.
 * \details This table contains the list of registered nodes by the login manager. 
 *          Only registered nodes can perform operations of plug and play, such as
 *          sending component instance manifests. 
 */
static
XME_HAL_TABLE
(
    xme_core_node_nodeId_t,
    xme_core_pnp_pnpManager_nodeList,
    XME_CORE_PNP_PNPMANAGER_INSTANCE_LIST_SIZE
);

/**
 * \brief  This table stores temporarily the unestablished logical routes. 
 *
 * \details These routes are updated in the logical route manager as soon
 *          as all runtime graphs have been delivered to all nodes. 
 */
static 
XME_HAL_TABLE
(
    xme_core_pnp_pnpManager_unestablishedLogicalRoute_t,
    xme_core_pnp_pnpManager_unestablishedLogicalRoutes,
    XME_CORE_PNP_PNPMANAGER_LOGICALROUTES_LIST_SIZE
);

/**
 * \brief The list of prepared graphs. 
 * \details The runtime graph list is composed of items of the
 *          type ::xme_core_pnp_pnpManager_rtGraphItem_t
 */
static 
xme_hal_singlyLinkedList_t(XME_CORE_PNP_PNPMANAGER_RUNTIMEGRAPHS_LIST_SIZE) rtGraphList;

/**
 * \brief Last assigned component id per node table.
 */
static
XME_HAL_TABLE
(
    xme_core_pnp_pnpManager_lastAssignedComponentIdPerNode_t,
    xme_core_pnp_pnpManager_lastAssignedComponentIdsPerNode,
    XME_CORE_PNP_PNPMANAGER_MAXIMUMNUMBEROFNODES_LIST_SIZE
);

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Edge comparison function that returns 0 if edge1Data denotes a logical route edge.
 *
 * \param[in] edgeData1 Edges data of the edge to test.
 * \param[in] edgeData2 Not used, must be NULL.
 *
 * \retval 0 if edgeData1 denotes a logical route edge.
 * \retval 1 if edgeData1 does not denote a logical route edge.
 */
int
xme_core_pnp_pnpManager_edgeCompareCallbackMatchLogicalRouteEdges
(
    void* edgeData1,
    void* edgeData2
);

/**
 * \brief Create runtime graphs for all unestablished routes in the given graph.
 *
 * \details Peforms the following steps:
 *          - Calls network communication manager to calculate physical routes.
 *          - For non-established routes new runtime graphs will be created.
 *          Runtime graphs will be queued and sent later across the network by the GenerateRuntimeGraphs function.
 *
 * \param[in] logicalRouteGraph Given logical route graph. May not be NULL.
 * \param[in] action The operation to complete with the graph. 
 *
 * \retval XME_STATUS_SUCCESS if the operation was successful.
 */
xme_status_t
xme_core_pnp_pnpManager_createRuntimeGraphs
(
    xme_core_pnp_lrm_logicalRoutes_t* const logicalRouteGraph,
    uint32_t action
);

/******************************************************************************/
/***   Helper functions                                                     ***/
/******************************************************************************/

int
xme_core_pnp_pnpManager_edgeCompareCallbackMatchLogicalRouteEdges
(
    void* edgeData1,
    void* edgeData2
)
{
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) edgeData1;

    XME_ASSERT_RVAL(NULL == edgeData2, 1);

    return (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType) ? 0 : 1;
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_pnp_pnpManager_init(void *params)
{
    XME_UNUSED_PARAMETER(params);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_log_logUtils_init(), XME_STATUS_OUT_OF_RESOURCES);

    XME_HAL_TABLE_INIT(xme_core_pnp_pnpManager_nodeList);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpManager_unestablishedLogicalRoutes);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpManager_lastAssignedComponentIdsPerNode);

    XME_HAL_SINGLYLINKEDLIST_INIT(rtGraphList);

    return XME_STATUS_SUCCESS;
}

void
xme_core_pnp_pnpManager_fini(void)
{
    xme_hal_singlyLinkedList_fini(&rtGraphList);

    XME_HAL_TABLE_FINI(xme_core_pnp_pnpManager_nodeList);
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpManager_unestablishedLogicalRoutes);
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpManager_lastAssignedComponentIdsPerNode);

    xme_core_log_logUtils_fini();
}

xme_status_t
xme_core_pnp_pnpManager_unannounceComponentOnNode
(
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID
)
{
    xme_core_pnp_lrm_logicalRoutes_t outLogicalRouteGraph;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentID, XME_STATUS_INVALID_PARAMETER);

    // If there is a registered component, we should just call LRM's unannouncement. 
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_lrm_unannouncePortForComponent(nodeID, componentID, XME_CORE_INVALID_TRANSACTION_ID), XME_STATUS_INTERNAL_ERROR);

    // After unannouncing, we should generate the runtime graph for deletion. 
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_init(&outLogicalRouteGraph), XME_STATUS_INTERNAL_ERROR);

    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS ==
        xme_core_pnp_lrm_getEstablishedLogicalRoutesForComponent
        (
            XME_CORE_INVALID_TRANSACTION_ID, 
            nodeID, 
            componentID, 
            &outLogicalRouteGraph
        ),
        XME_STATUS_INTERNAL_ERROR,
        {
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(&outLogicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
        }
    );
    
    {
        uint16_t establishedRouteCount = xme_hal_graph_getEdgeCount(&outLogicalRouteGraph);

        if (0 < establishedRouteCount)
        {
            // Do the call to include the corresponding graphs to the initial status. 
            // This call will place the corresponding removal graphs in the list of runtime graphs to be sent to target nodes. 
            XME_CHECK(XME_STATUS_SUCCESS ==
                xme_core_pnp_pnpManager_createRuntimeGraphs
                (
                    &outLogicalRouteGraph,
                    XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_REMOVE
                ),
                XME_STATUS_INTERNAL_ERROR
            );
        }
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_unannounceNode
(
    xme_core_node_nodeId_t nodeID
)
{
    bool found = false;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);

    {
        xme_core_nodeMgr_compRep_componentIteratorInit();

        // Explore all instances for the given node. 
        while (xme_core_nodeMgr_compRep_componentIteratorHasNext())
        {
            xme_status_t status;
            xme_core_nodeMgr_compRep_componentHandle_t componentHandle = xme_core_nodeMgr_compRep_componentIteratorNext();

            if (xme_core_nodeMgr_compRep_getNodeID(componentHandle) != nodeID) { continue; }

            found = true;

            // Note: We will avoid the check of the status, because even if not published, we should remove it. 
            status = xme_core_pnp_pnpManager_unannounceComponentOnNode
            (
                nodeID,
                xme_core_nodeMgr_compRep_getComponentID(componentHandle)
            );

            XME_CHECK
            (
                XME_STATUS_SUCCESS == status || // Everything worked fine (as expected!)
                XME_STATUS_NOT_FOUND == status,  // The announcement was removed but there were no associated logical routes. 
                XME_STATUS_INTERNAL_ERROR
            );

            // Set the runtime graphs associated to the calling node to LOGOUT. 
            XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
                rtGraphList, 
                xme_core_pnp_pnpManager_rtGraphItem_t, 
                rtGraphItem);
            {
                if (rtGraphItem->nodeId == nodeID &&
                    rtGraphItem->announced == false &&
                    rtGraphItem->rtGraph->action == XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_REMOVE)
                {
                    rtGraphItem->rtGraph->action = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_LOGOUT;
                }
            }
            XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
        }

        xme_core_nodeMgr_compRep_componentIteratorFini();
    }

    XME_CHECK(found == true, XME_STATUS_NOT_FOUND);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_updateLogicalRoutes
(
    xme_core_pnp_lrm_logicalRoutes_t* logicalRouteGraph
)
{
    bool rerun = false;

    XME_ASSERT(NULL != logicalRouteGraph);

    do
    {
        // 1) Ask Logical Route Manager to calculate new logical routes and obtain the corresponding routes. 
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_init(logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);

        XME_CHECK_REC
        (
            XME_STATUS_SUCCESS == xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, logicalRouteGraph),
            XME_STATUS_INTERNAL_ERROR,
            {
                XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
            }
        );

        // 2) Execute additional registered configurator
        rerun = xme_core_pnp_configExt_executeConfigurators(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, logicalRouteGraph);

        if (rerun) { xme_hal_graph_clear(logicalRouteGraph); }
    } while (rerun);

    // Log information about logical routes. 
    xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_DEBUG, "LRM Logical Route Graph", logicalRouteGraph);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_storeUnestablishedLogicalRoutes(logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createRuntimeGraphs
(
    xme_core_pnp_lrm_logicalRoutes_t* const logicalRouteGraph,
    uint32_t action
)
{
    xme_core_pnp_ncc_physicalRoutes_t physicalRouteGraph;
    xme_core_pnp_pnpManager_physicalGraphList_t splittedPhysicalGraphList;

    XME_ASSERT(NULL != logicalRouteGraph);

    // 1)
    // - Forward graph to Network Communication Calculator
    // - Obtain complete waypoint graph from Network Communication Calculator
    // - Ask Network Communication Calculator to calculate waypoints
    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS == xme_core_pnp_ncc_getPhysicalRoutes((xme_hal_graph_graph_t*) logicalRouteGraph, &physicalRouteGraph),
        XME_STATUS_INTERNAL_ERROR,
        {
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(&physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
        }
    );

    xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_DEBUG, "NCC Physical Route Graph", &physicalRouteGraph);

    // 2)
    // - Split complete waypoint graph according to nodes and remove unneeded information
    XME_HAL_SINGLYLINKEDLIST_INIT(splittedPhysicalGraphList);

    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_splitGraph(&physicalRouteGraph, &splittedPhysicalGraphList),
        XME_STATUS_INTERNAL_ERROR,
        {
            xme_hal_singlyLinkedList_fini(&splittedPhysicalGraphList);
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini((xme_hal_graph_graph_t*) &physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
        }
    );

    // - Transform every subgraph into the respective topic instance
    {
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
        (
            splittedPhysicalGraphList, 
            xme_core_pnp_pnpManager_physicalGraphListItem_t, 
            splittedPhysicalGraphItem);
        {
            char buf[32];
            xme_status_t status;

            XME_CHECK(0 < xme_hal_safeString_snprintf(buf, sizeof(buf), "Split Graph for Node %u", (uint32_t) splittedPhysicalGraphItem->nodeId), XME_STATUS_INTERNAL_ERROR);
            xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_DEBUG, buf, splittedPhysicalGraphItem->splitGraph);

            status = xme_core_pnp_pnpManager_processGraph(splittedPhysicalGraphItem->splitGraph, splittedPhysicalGraphItem->nodeId, action);
            XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, XME_STATUS_INTERNAL_ERROR);
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    }

    // Finalize the declared physical and logical routes, including splitted graph.
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(&physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        splittedPhysicalGraphList,
        xme_core_pnp_pnpManager_physicalGraphListItem_t,
        physhicalGraphListItem
    )
    {
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(physhicalGraphListItem->splitGraph), XME_STATUS_INTERNAL_ERROR);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_updateConfiguration(void)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_pnp_lrm_logicalRoutes_t logicalRouteGraph;

    status = xme_core_pnp_pnpManager_updateLogicalRoutes(&logicalRouteGraph);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    status = xme_core_pnp_pnpManager_createRuntimeGraphs(&logicalRouteGraph, XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD);

    return status;
}

xme_status_t
xme_core_pnp_pnpManager_instantiateComponentOnNode
(
    xme_core_componentType_t componentType,
    const char* const initializationString,
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t existingComponentID
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle;

    XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentType, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);

    builder = xme_core_nodeMgr_compRep_createBuilder(nodeID, componentType);
    XME_CHECK(NULL != builder, XME_STATUS_INTERNAL_ERROR);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, existingComponentID);
    xme_core_nodeMgr_compRep_builderSetInitializationString(builder, initializationString);
    status = xme_core_nodeMgr_compRep_build(builder, &componentHandle);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    status = xme_core_pnp_pnpManager_plugInNewComponent(componentHandle);

    return status;
}

xme_status_t
xme_core_pnp_pnpManager_announceNewComponentOnNode
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_node_nodeId_t nodeID = xme_core_nodeMgr_compRep_getNodeID(componentHandle);
    xme_core_component_t componentID = xme_core_nodeMgr_compRep_getComponentID(componentHandle);
    xme_core_componentType_t componentType = xme_core_nodeMgr_compRep_getComponentType(componentHandle);
    xme_core_nodeMgr_compRep_state_t componentState = xme_core_nodeMgr_compRep_getState(componentHandle);

    XME_CHECK_MSG_C
    (
        XME_CORE_NODEMGR_COMPREP_STATE_PREPARED == componentState,
        XME_STATUS_INVALID_CONFIGURATION,
        XME_CORE_COMPONENT_TYPE_PNPMANAGER,
        XME_LOG_ERROR,
        "announceNewComponentOnNode(): Component to announce is in invalid state %d.\n",
        componentState
    );

    XME_CHECK_MSG_C
    (
        xme_core_pnp_pnpManager_isNodeRegistered(nodeID),
        XME_STATUS_INVALID_PARAMETER,
        XME_CORE_COMPONENT_TYPE_PNPMANAGER,
        XME_LOG_ERROR,
        "Attempt to add component to node %d, which is not registered.\n",
        nodeID
    );

    if (XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT == componentID)
    {
        // Get the new component id.
        componentID = xme_core_pnp_pnpManager_generateNewComponentIdForNode(nodeID);
        XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentID, XME_STATUS_INTERNAL_ERROR);

        xme_core_nodeMgr_compRep_setComponentID(componentHandle, componentID);
    }

    // Announce component manifest ports to logical route manager.
    status = xme_core_pnp_pnpManager_announcePorts(nodeID, componentID, componentType);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    status = xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_plugInNewComponent
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;

    XME_CHECK(XME_CORE_NODEMGR_COMPREP_STATE_PREPARED == xme_core_nodeMgr_compRep_getState(componentHandle), XME_STATUS_INVALID_PARAMETER); // This will also catch non-existing components

    status = xme_core_pnp_pnpManager_announceNewComponentOnNode(componentHandle);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    status = xme_core_pnp_pnpManager_updateConfiguration();
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_splitGraph
(
    xme_core_pnp_ncc_physicalRoutes_t* physicalRouteGraph,
    xme_core_pnp_pnpManager_physicalGraphList_t* outSplitGraphs
)
{
    xme_status_t status;

    XME_CHECK(NULL != outSplitGraphs, XME_STATUS_INVALID_PARAMETER);

    status = xme_hal_graph_initVertexIterator(physicalRouteGraph);
    XME_ASSERT(XME_STATUS_SUCCESS == status);
    {
        while (xme_hal_graph_hasNextVertex(physicalRouteGraph))
        {
            bool graphListItemExists;

            // Retrieve vertex identifier of the next vertex
            xme_hal_graph_vertexId_t vertexId = xme_hal_graph_nextVertex(physicalRouteGraph);
            xme_core_pnp_dataLinkGraph_vertexData_t *vertexData;
            xme_core_pnp_pnpManager_physicalGraphListItem_t* physicalGraphListItem;

            // Retrieve vertex data
            status = xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**) &vertexData);
            XME_ASSERT(XME_STATUS_SUCCESS == status);

            // Create a new item for each distinct nodeId in the graphList linked list
            graphListItemExists = false;
            XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
            (
                *outSplitGraphs, 
                xme_core_pnp_pnpManager_physicalGraphListItem_t, 
                physicalGraphItem
            );
            {
                if (physicalGraphItem->nodeId == vertexData->nodeId)
                {
                    graphListItemExists = true;
                    break;
                }
            }
            XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

            if (graphListItemExists)
            {
                // This vertex has already been processed in a prevous iteration
                continue;
            }

            // Allocate and initialize a new item with the nodeId
            physicalGraphListItem = (xme_core_pnp_pnpManager_physicalGraphListItem_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_pnpManager_physicalGraphListItem_t));
            physicalGraphListItem->nodeId = vertexData->nodeId;

            // Allocate and initialize the split graph
            physicalGraphListItem->splitGraph = (xme_core_pnp_ncc_physicalRoutes_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_ncc_physicalRoutes_t));
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_init(physicalGraphListItem->splitGraph), XME_STATUS_INTERNAL_ERROR);

            // clone the input graph
            status = xme_hal_graph_clone(physicalRouteGraph, physicalGraphListItem->splitGraph);
            XME_ASSERT(XME_STATUS_SUCCESS == status);

            // Iterate over vertices in the cloned graph, and remove vertices that do not belong to current node
            status = xme_hal_graph_initVertexIterator(physicalGraphListItem->splitGraph); //proper graph parameter
            XME_ASSERT(XME_STATUS_SUCCESS == status);
            {
                while (xme_hal_graph_hasNextVertex(physicalGraphListItem->splitGraph))
                {
                    xme_core_pnp_dataLinkGraph_vertexData_t * splitGraph_vertexData;
                    // get the vertexId of the next vertex
                    xme_hal_graph_vertexId_t splitGraph_vertexId = xme_hal_graph_nextVertex(physicalGraphListItem->splitGraph);

                    // get the vertexData of the vertex
                    status = xme_hal_graph_getVertexData(physicalGraphListItem->splitGraph, splitGraph_vertexId, (void**)&splitGraph_vertexData);
                    XME_ASSERT(XME_STATUS_SUCCESS == status);

                    // if vertex does not belong to the current node
                    if (splitGraph_vertexData->nodeId != vertexData->nodeId)
                    {
                        // remove vertex
                        status = xme_hal_graph_removeVertex(physicalGraphListItem->splitGraph, splitGraph_vertexId);
                        XME_ASSERT(XME_STATUS_SUCCESS == status);
                    }
                }
            }
            status = xme_hal_graph_finiVertexIterator(physicalGraphListItem->splitGraph);
            XME_ASSERT(XME_STATUS_SUCCESS == status);

            // Remove logical route edges that might still be present in the graph
            {
                xme_hal_graph_edgeCompareCallback_t edgeCompareCallback = xme_hal_graph_getEdgeCompareCallback(physicalGraphListItem->splitGraph);

                status = xme_hal_graph_setEdgeCompareCallback(physicalGraphListItem->splitGraph, &xme_core_pnp_pnpManager_edgeCompareCallbackMatchLogicalRouteEdges);
                XME_ASSERT(XME_STATUS_SUCCESS == status);

                status = xme_hal_graph_removeEdgeWithDataComparison(physicalGraphListItem->splitGraph, NULL, true);
                XME_ASSERT(XME_STATUS_SUCCESS == status || XME_STATUS_NOT_FOUND == status);

                status = xme_hal_graph_setEdgeCompareCallback(physicalGraphListItem->splitGraph, edgeCompareCallback);
                XME_ASSERT(XME_STATUS_SUCCESS == status);
            }

            // add the new item to the linked list
            XME_CHECK
            (
                XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(outSplitGraphs, physicalGraphListItem), 
                XME_STATUS_INTERNAL_ERROR
            );
        }
    }
    
    status = xme_hal_graph_finiVertexIterator(physicalRouteGraph);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_pnp_pnpManager_processGraph
(
    xme_core_pnp_ncc_physicalRoutes_t* physicalRouteGraph,
    xme_core_node_nodeId_t nodeId,
    uint32_t action
)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph;
    uint8_t i;
    xme_hal_singlyLinkedList_t(XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH) vertexIndex;
    xme_core_pnp_pnpManager_rtGraphItem_t* rtGraphItem;

    XME_CHECK(NULL != physicalRouteGraph, XME_STATUS_INVALID_PARAMETER);

    rtGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

    XME_CHECK_MSG_C_REC
    (
        xme_hal_graph_getVertexCount(physicalRouteGraph) <= XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH,
        XME_STATUS_UNSUPPORTED,
        { xme_hal_mem_free(rtGraph); },
        XME_CORE_COMPONENT_TYPE_PNPMANAGER,
        XME_LOG_ERROR,
        "Number of vertices (ports and waypoints) in physical route graph for node %d exceeds supported maxmimum (%" PRIu16 " > %d). "
        "Maximum is determined by length of vertex array in runtimeGraphModel topic.\n",
        nodeId, xme_hal_graph_getVertexCount(physicalRouteGraph), XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH
    );
    XME_CHECK_MSG_C_REC
    (
        xme_hal_graph_getEdgeCount(physicalRouteGraph) <= XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH,
        XME_STATUS_UNSUPPORTED,
        { xme_hal_mem_free(rtGraph); },
        XME_CORE_COMPONENT_TYPE_PNPMANAGER,
        XME_LOG_ERROR,
        "[plugAndPlayManager] Number of edges (connections between ports and waypoints) in physical route graph for node %d exceeds supported maxmimum (%" PRIu16 " > %d). "
        "Maximum is determined by length of edge array in runtimeGraphModel topic.\n",
        nodeId, xme_hal_graph_getEdgeCount(physicalRouteGraph), XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH
    );

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initVertexIterator(physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);

    rtGraph->nodeId = nodeId; 

    // 1. Create the topic vertex.
    i = 0;
    XME_HAL_SINGLYLINKEDLIST_INIT(vertexIndex);
    while (xme_hal_graph_hasNextVertex(physicalRouteGraph))
    {
        xme_core_pnp_dataLinkGraph_vertexData_t* routeVertexData;
        xme_core_pnp_pnpManager_vertexItem_t* vertexItem; ///< this data structure will store vertex items to be used in the edge creation.
        xme_hal_graph_vertexId_t vertexId = xme_hal_graph_nextVertex(physicalRouteGraph);

        XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId);
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**) &routeVertexData), XME_STATUS_INTERNAL_ERROR);

        XME_CHECK(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE != routeVertexData->vertexType, 
            XME_STATUS_INTERNAL_ERROR);

        if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT) {
            xme_status_t status;
            xme_core_nodeMgr_compRep_componentHandle_t componentHandle;

            status = xme_core_nodeMgr_compRep_getComponentInstance
            (
                routeVertexData->nodeId,
                routeVertexData->vertexData.componentPortVertex.componentId,
                &componentHandle
            );
            XME_CHECK_MSG_C
            (
                XME_STATUS_NOT_FOUND != status,
                XME_STATUS_INTERNAL_ERROR,
                XME_CORE_COMPONENT_TYPE_PNPMANAGER,
                XME_LOG_ERROR,
                "Data link graph contains component (nodeID = %d, componentID = %d) which is not registered in the component repository. Canceling configuration update.\n",
                routeVertexData->nodeId, routeVertexData->vertexData.componentPortVertex.componentId
            );
            XME_ASSERT(XME_STATUS_SUCCESS == status);

            XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPMANAGER, XME_LOG_DEBUG, "The %d vertex is a component port\n", i);
        
            // Add the component vertex to the runtime graph
            XME_CHECK
            (
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createComponent
                (
                    rtGraph,
                    routeVertexData->vertexData.componentPortVertex,
                    componentHandle,
                    i
                ),
                XME_STATUS_INTERNAL_ERROR
            );
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER)
        {
            XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPMANAGER, XME_LOG_DEBUG, "The %d vertex is a marshaler waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createMarshalerWaypoint(rtGraph,
                    i,
                    routeVertexData->vertexData.waypointMarshalerVertex.inputPortQueueSize),
                XME_STATUS_INTERNAL_ERROR);
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND)
        {
            XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPMANAGER, XME_LOG_DEBUG, "The %d vertex is a UPD send waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createUDPSendWaypoint(rtGraph, 
                    routeVertexData->vertexData.waypointUdpSendVertex.key,
                    routeVertexData->vertexData.waypointUdpSendVertex.destination,
                    i),
                XME_STATUS_INTERNAL_ERROR);
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE)
        {
            XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPMANAGER, XME_LOG_DEBUG, "The %d vertex is a UPD receive waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createUDPReceiveWaypoint(rtGraph, 
                    routeVertexData->vertexData.waypointUdpReceiveVertex.key,
                    routeVertexData->vertexData.waypointUdpReceiveVertex.host,
                    i),
                XME_STATUS_INTERNAL_ERROR);
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER)
        {
            XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPMANAGER, XME_LOG_DEBUG, "The %d vertex is a demarshaler waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createDemarshalerWaypoint
                (
                    rtGraph,
                    i,
                    routeVertexData->vertexData.waypointDemarshalerVertex.inputPortQueueSize
                ),
                XME_STATUS_INTERNAL_ERROR);
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR)
        {
            XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPMANAGER, XME_LOG_DEBUG, "The %d vertex is a channel injector waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createChannelInjectorWaypoint(rtGraph, 
                    routeVertexData->vertexData.waypointChannelInjectorVertex.injectedChannelId,
                    routeVertexData->vertexData.waypointChannelInjectorVertex.inputPortQueueSize,
                    i),
                XME_STATUS_INTERNAL_ERROR);
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR)
        {
            XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPMANAGER, XME_LOG_DEBUG, "The %d vertex is a channel selector waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createChannelSelectorWaypoint(rtGraph, 
                    routeVertexData->vertexData.waypointChannelSelectorVertex.sourceChannelId,
                    routeVertexData->vertexData.waypointChannelSelectorVertex.destinationChannelId,
                    routeVertexData->vertexData.waypointChannelSelectorVertex.inputPortQueueSize,
                    i),
                XME_STATUS_INTERNAL_ERROR);
        }
        else // if (routeVertexData->vertexType == XME_CORE_PNP_PNPMANAGER_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE)
        {
            return XME_STATUS_INTERNAL_ERROR;
        }

        // assumption: one single element for each vertex type can be added per subgraph. 
        vertexItem = (xme_core_pnp_pnpManager_vertexItem_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_pnpManager_vertexItem_t));
        vertexItem->index = i;
        vertexItem->vertexType = routeVertexData->vertexType;
        vertexItem->vertexId = vertexId;
        XME_CHECK
        (
            XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(&vertexIndex, vertexItem), 
            XME_STATUS_INTERNAL_ERROR
        );

        i++;
    }

    // 2. Create edges between vertices. 
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiVertexIterator(physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initVertexIterator(physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
    i = 0;
    while (xme_hal_graph_hasNextVertex(physicalRouteGraph))
    {
        xme_hal_graph_vertexId_t vertexId = xme_hal_graph_nextVertex(physicalRouteGraph);

        XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId); 
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initOutgoingEdgeIterator(physicalRouteGraph, vertexId), XME_STATUS_INTERNAL_ERROR);
        
        while(xme_hal_graph_hasNextOutgoingEdge(physicalRouteGraph, vertexId))
        {
            xme_core_pnp_dataLinkGraph_edgeData_t* routeEdgeData;
            xme_hal_graph_vertexId_t srcVertexId;
            xme_hal_graph_vertexId_t dstVertexId;
            xme_hal_graph_edgeId_t edgeId = xme_hal_graph_nextOutgoingEdge(physicalRouteGraph, vertexId);

            XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != edgeId);

            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(physicalRouteGraph, edgeId, (void**) &routeEdgeData), XME_STATUS_INTERNAL_ERROR);

            XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY == routeEdgeData->edgeType);

            srcVertexId = xme_hal_graph_getSourceVertex(physicalRouteGraph, edgeId);
            dstVertexId = xme_hal_graph_getSinkVertex(physicalRouteGraph, edgeId);
            XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != srcVertexId, XME_STATUS_INVALID_PARAMETER);
            XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != dstVertexId, XME_STATUS_INVALID_PARAMETER);

            XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPMANAGER, XME_LOG_DEBUG, "The %d edge is a mem copy: from %d to %d\n", i, srcVertexId, dstVertexId);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createMemCopyEdge(
                    rtGraph,
                    srcVertexId,
                    dstVertexId,
                    routeEdgeData->edgeData.memCopyEdge.topicId,
                    routeEdgeData->edgeData.memCopyEdge.channelID,
                    (xme_hal_linkedList_descriptor_t*) &vertexIndex,
                    i),
                    XME_STATUS_INTERNAL_ERROR);
            i++;
        }

        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiOutgoingEdgeIterator(physicalRouteGraph, vertexId), XME_STATUS_INTERNAL_ERROR);
    }

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiVertexIterator(physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);

    // Set the action in the runtime graph. 
    rtGraph->action = action;

    // 3. Remove the runtime graph associated to the same operation.
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        rtGraphList,
        xme_core_pnp_pnpManager_rtGraphItem_t,
        rtGraphListItem
    );
    {
        if (rtGraphListItem->nodeId == nodeId && rtGraphListItem->rtGraph->action == action)
        {
            XME_CHECK(0 < xme_hal_singlyLinkedList_removeItem(&rtGraphList, rtGraphListItem, (bool) false), XME_STATUS_INTERNAL_ERROR);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // 4. Include the runtime graph in the runtime graph list. 
    rtGraphItem = (xme_core_pnp_pnpManager_rtGraphItem_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_pnpManager_rtGraphItem_t));
    rtGraphItem->nodeId = nodeId; 
    rtGraphItem->announced = false;
    rtGraphItem->rtGraph = rtGraph;
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(&rtGraphList, rtGraphItem), 
        XME_STATUS_INTERNAL_ERROR
    );

    // 5. before sending back the control to the thread, we should finish the temporary data structure. 
    xme_hal_singlyLinkedList_fini(&vertexIndex);
    
    return XME_STATUS_SUCCESS;
}

bool
xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode
(
    xme_core_node_nodeId_t nodeId
)
{
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, false);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        rtGraphList, 
        xme_core_pnp_pnpManager_rtGraphItem_t, 
        rtGraphItem);
    {
        if (rtGraphItem->nodeId == nodeId &&
            rtGraphItem->announced == false)
        {
            return true;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return false;
}

xme_status_t
xme_core_pnp_pnpManager_getNextRuntimeGraphForNode
(
    xme_core_node_nodeId_t nodeId,
    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph
)
{
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        rtGraphList, 
        xme_core_pnp_pnpManager_rtGraphItem_t, 
        rtGraphItem);
    {
        if (rtGraphItem->nodeId == nodeId &&
            rtGraphItem->announced == false)
        {
            rtGraphItem->announced = true;

            if (NULL != rtGraphItem->rtGraph)
            {
                xme_core_component_t componentId;

                *outGraph = *rtGraphItem->rtGraph;

                componentId = xme_core_pnp_pnpManager_getComponentIdFromRTGraph(rtGraphItem->rtGraph);

                XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INTERNAL_ERROR);

#if 0
                XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_updateInstance(nodeId, componentId, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_SUBMITTED),
                    XME_STATUS_INTERNAL_ERROR);
#endif

                return XME_STATUS_SUCCESS;
            }
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_NOT_FOUND;
}

bool
xme_core_pnp_pnpManager_hasNewRuntimeGraphs(void)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        rtGraphList, 
        xme_core_pnp_pnpManager_rtGraphItem_t, 
        rtGraphItem);
    {
        if (rtGraphItem->announced == false)
        {
            return true;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return false;
}

xme_status_t
xme_core_pnp_pnpManager_getNextRuntimeGraph
(
    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph
)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        rtGraphList, 
        xme_core_pnp_pnpManager_rtGraphItem_t, 
        rtGraphItem);
    {
        if (!(rtGraphItem->announced))
        {
            rtGraphItem->announced = true;

            if (NULL != rtGraphItem->rtGraph)
            {
                xme_core_component_t componentID;

                *outGraph = *rtGraphItem->rtGraph;
                componentID = xme_core_pnp_pnpManager_getComponentIdFromRTGraph(rtGraphItem->rtGraph);

                XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentID, XME_STATUS_INTERNAL_ERROR);
#if 0
                XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_updateInstance(rtGraphItem->nodeId, componentID, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_SUBMITTED),
                    XME_STATUS_INTERNAL_ERROR);
#endif
                return XME_STATUS_SUCCESS;
            }

        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_NOT_FOUND;
}

#if 0
xme_status_t
xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_status_t status
)
{
    xme_core_component_t componentId;

    XME_CHECK(NULL != rtGraph, XME_STATUS_INVALID_PARAMETER);

    componentId = xme_core_pnp_pnpManager_getComponentIdFromRTGraph(rtGraph);

    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_NOT_FOUND);

    if (XME_STATUS_SUCCESS == status)
    {
        return xme_core_pnp_pnpManager_updateInstance(rtGraph->nodeId, componentId, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_RUNNING);
    }
    else 
    {
        return xme_core_pnp_pnpManager_updateInstance(rtGraph->nodeId, componentId, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_FAILURE);
    }
}
#endif

xme_status_t
xme_core_pnp_pnpManager_announcePorts
(
    xme_core_node_nodeId_t nodeId, 
    xme_core_component_t componentId,
    xme_core_componentType_t componentType
)
{
    uint16_t i;
    xme_status_t status;
    xme_core_componentManifest_t componentManifest;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentType, XME_STATUS_INVALID_PARAMETER);

    // Get the component type manifest
    status = xme_core_manifestRepository_findComponentManifest(componentType, &componentManifest);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION);

    // Gets the port information from the component manifest.
    for (i = 0; i < xme_core_manifestRepository_getPortCount(&componentManifest); i++)
    {
        xme_core_componentPortManifest_t* portManifest = &(componentManifest.portManifests[i]);
        XME_ASSERT(NULL != portManifest);

        XME_CHECK
        (
            XME_STATUS_SUCCESS == 
            xme_core_pnp_lrm_announcePort(
                nodeId,
                componentId,
                componentType,
                portManifest->portType,
                i,
                portManifest->lowerConnectionBound,
                portManifest->upperConnectionBound,
                portManifest->topic,
                portManifest->attrSet,
                XME_CORE_INVALID_TRANSACTION_ID
            ),
            XME_STATUS_INTERNAL_ERROR
        );
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished(void)
{
    XME_HAL_TABLE_ITERATE_BEGIN
    (
        xme_core_pnp_pnpManager_unestablishedLogicalRoutes,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_pnp_pnpManager_unestablishedLogicalRoute_t,
        unestablishedLogicalRoute
    );
    {
        XME_ASSERT(unestablishedLogicalRoute != NULL);
        XME_ASSERT(unestablishedLogicalRoute->channelId != XME_CORE_INVALID_CHANNEL_ID);
        XME_ASSERT(unestablishedLogicalRoute->edgeId != XME_HAL_GRAPH_INVALID_EDGE_ID);

        XME_CHECK
        (
            XME_STATUS_SUCCESS == 
                xme_core_pnp_lrm_setLogicalRoute(unestablishedLogicalRoute->edgeId, unestablishedLogicalRoute->channelId, XME_CORE_INVALID_TRANSACTION_ID), 
            XME_STATUS_INTERNAL_ERROR
        );

        XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpManager_unestablishedLogicalRoutes, handle), XME_STATUS_INTERNAL_ERROR);
    }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_SUCCESS;

}

xme_status_t
xme_core_pnp_pnpManager_removeNodeInstances
(
    xme_core_node_nodeId_t nodeID
)
{
    bool found = false;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);

    // Explore the component instance list. 
    {
        xme_core_nodeMgr_compRep_componentIteratorInit();

        // Explore all instances for the given node. 
        while (xme_core_nodeMgr_compRep_componentIteratorHasNext())
        {
            xme_core_nodeMgr_compRep_componentHandle_t componentHandle = xme_core_nodeMgr_compRep_componentIteratorNext();

            if (xme_core_nodeMgr_compRep_getNodeID(componentHandle) != nodeID) { continue; }

            found = true;
            // Deregister component instance
            xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle);
        }

        xme_core_nodeMgr_compRep_componentIteratorFini();
    }

    if(!found)
    {
        return XME_STATUS_NOT_FOUND;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_removeComponentInstance
(
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID
)
{
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentID, XME_STATUS_INVALID_PARAMETER);

    // Explore the component instance list. 
    xme_core_nodeMgr_compRep_componentIteratorInit();

    // Explore all instances for the given node. 
    while (xme_core_nodeMgr_compRep_componentIteratorHasNext())
    {
        xme_core_nodeMgr_compRep_componentHandle_t componentHandle = xme_core_nodeMgr_compRep_componentIteratorNext();

        if (xme_core_nodeMgr_compRep_getNodeID(componentHandle) == nodeID &&
            xme_core_nodeMgr_compRep_getComponentID(componentHandle) == componentID)
        {
            if (xme_core_node_getCurrentNodeId() == xme_core_nodeMgr_compRep_getNodeID(componentHandle))
            {
                // We should let the pnp client to remove this information.
                return XME_STATUS_SUCCESS;
            }
            
            // Deregister one single component instance.
            xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle);

            return XME_STATUS_SUCCESS;
        }
    }

    xme_core_nodeMgr_compRep_componentIteratorFini();

    return XME_STATUS_NOT_FOUND;
}


xme_status_t
xme_core_pnp_pnpManager_generateChannelIDForManifestReception
(
    xme_core_node_nodeId_t nodeID,
    xme_core_channelId_t* channelID
)
{
    xme_status_t status;
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != channelID, XME_STATUS_INVALID_PARAMETER);

    status = xme_core_pnp_lrm_getChannelID
    (
        XME_CORE_INVALID_TRANSACTION_ID,
        XME_CORE_NODE_INVALID_NODE_ID,
        XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT,
        0,
        XME_CORE_NODE_INVALID_NODE_ID,
        XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT,
        0,
        channelID
    );
        
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    XME_ASSERT(*channelID != XME_CORE_INVALID_CHANNEL_ID);

    return XME_STATUS_SUCCESS;

}

///////////////////////////////////////////////////////////////////////////////////////

// TODO
#if 0
bool 
xme_core_pnp_pnpManager_isComponentInstanceRunning
(
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID
)
{
    xme_core_pnp_pnpManager_nodeComponentInstance_t* nodeComponentInstance;
    xme_hal_table_rowHandle_t rowHandle;

    rowHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, false);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentID, false);

    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_pnp_pnpManager_nodeComponentInstanceList, 
        xme_hal_table_rowHandle_t, 
        rowHandle, 
        xme_core_pnp_pnpManager_nodeComponentInstance_t, 
        nodeComponentInstance,
        (nodeID == nodeComponentInstance->nodeId && componentID == nodeComponentInstance->componentId)
    );

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != rowHandle, false);

    // FIXME: The component instance do not reach to the status running, because we do not receive a confirmation
    //        from PnPClient (Issue #3761). 
    if (nodeComponentInstance->status == XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_RUNNING ||
        nodeComponentInstance->status == XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_SUBMITTED)
    {
        return true;
    }
    else
    {
        return false;
    }

    return true;
}
#endif

// TODO
#if 0
xme_status_t
xme_core_pnp_pnpManager_updateInstance
(
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t componentId,
    xme_core_pnp_pnpManager_nodeComponentInstance_status_t status
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_pnp_pnpManager_nodeComponentInstance_t* instanceItem;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    instanceItem = NULL;

    XME_CHECK(nodeId != XME_CORE_NODE_INVALID_NODE_ID, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(componentId != XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(status != XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_INVALID_STATUS, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT(
        xme_core_pnp_pnpManager_nodeComponentInstanceList,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_pnp_pnpManager_nodeComponentInstance_t,
        instanceItem,
        instanceItem->nodeId == nodeId && 
        instanceItem->componentId == componentId
    );

    XME_CHECK(NULL != instanceItem, XME_STATUS_NOT_FOUND);

    instanceItem->status = status;

    return XME_STATUS_SUCCESS;
}
#endif

// TODO: Refactor the code in the following (Issue #3873):
// - Pass the address of rtGraph->vertex[internalIndex] directly to the functions
//   instead of passing rtGraph and internalIndex separately.
// - Move these functions and their corresponding counterparts in PnPClient
//   to common file(s) in order to easier associate and keep in sync
//   the formatting and parsing of the vertex data string.

// TODO: Refactor this implementation to pass initializationString
// only once, not for every port of the respective component!
// Probably introduce a special component vertex in the RT graph,.
// or use a reference to a port handle / component handle in the
// component repository

xme_status_t
xme_core_pnp_pnpManager_createComponent
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_core_pnp_dataLinkGraph_componentPortVertexData_t componentPortData,
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    uint8_t internalIndex
)
{
    xme_core_component_t componentID = xme_core_nodeMgr_compRep_getComponentID(componentHandle);
    xme_core_componentType_t componentType = xme_core_nodeMgr_compRep_getComponentType(componentHandle);
    const char* const initializationString = xme_core_nodeMgr_compRep_getInitializationString(componentHandle);
    xme_core_nodeMgr_compRep_componentHandle_t foreignComponentHandle =
        xme_core_nodeMgr_compRep_getForeignComponentHandle(componentHandle);

    rtGraph->vertex[internalIndex].componentId = componentPortData.componentId;
    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;
    rtGraph->vertex[internalIndex].componentId = componentID;
    rtGraph->vertex[internalIndex].componentType = componentType;
    rtGraph->vertex[internalIndex].componentHandle = foreignComponentHandle;

    // Set port instance-specific data
    {
        uint16_t portIndex = 0u;

        while (true)
        {
            xme_core_nodeMgr_compRep_portHandle_t portHandle =
                xme_core_nodeMgr_compRep_getPort(componentHandle, portIndex);

            if (0u == portHandle) { break; }

            if (portIndex >= sizeof(rtGraph->vertex[internalIndex].portData) / sizeof(rtGraph->vertex[internalIndex].portData[0]))
            {
                XME_LOG_C
                (
                    XME_CORE_COMPONENT_TYPE_PNPMANAGER,
                    XME_LOG_WARNING,
                    "Number of ports of componentType %d exceeds maximum number of port entries in runtime graph model. "
                    "Any port instance-specific data after port %d index will be ignored.\n",
                    componentType,
                    sizeof(rtGraph->vertex[internalIndex].portData) / sizeof(rtGraph->vertex[internalIndex].portData[0]) - 1
                );
                break;
            }

            rtGraph->vertex[internalIndex].portData[portIndex].queueSize =
                (uint8_t)xme_core_nodeMgr_compRep_getQueueSize(portHandle);
            // TODO: uint8_t cast! Consolidate queue size

            portIndex++;
        }
    }

    // Set function instance-specific data
    {
        uint16_t functionIndex = 0u;

        while (true)
        {
            xme_core_nodeMgr_compRep_functionHandle_t functionHandle =
                xme_core_nodeMgr_compRep_getFunction(componentHandle, functionIndex);

            if (0u == functionHandle) { break; }

            rtGraph->vertex[internalIndex].functionData[functionIndex].executionPeriod =
                xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle);

            if (functionIndex >= sizeof(rtGraph->vertex[internalIndex].functionData) / sizeof(rtGraph->vertex[internalIndex].functionData[0]))
            {
                XME_LOG_C
                (
                    XME_CORE_COMPONENT_TYPE_PNPMANAGER,
                    XME_LOG_WARNING,
                    "Number of functions of componentType %d exceeds maximum number of function entries in runtime graph model. "
                    "Any function instance-specific data after function %d index will be ignored.\n",
                    componentType,
                    sizeof(rtGraph->vertex[internalIndex].functionData) / sizeof(rtGraph->vertex[internalIndex].functionData[0]) - 1
                );
                break;
            }

            functionIndex++;
        }
    }

    // Set the component initialization data
    XME_CHECK
    (
        0 < xme_hal_safeString_snprintf
        (
            &rtGraph->vertex[internalIndex].vertexData[0],
            sizeof(rtGraph->vertex[internalIndex].vertexData),
            "%u|%s",
            (unsigned int) componentPortData.portType,
            (NULL == initializationString) ? "" : initializationString
        ),
        XME_STATUS_INTERNAL_ERROR
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createChannelInjectorWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_core_channelId_t channelId,
    uint8_t inputPortQueueSize,
    uint8_t internalIndex
)
{
    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR;
    rtGraph->vertex[internalIndex].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT; // Waypoints do not have an componentId id
    rtGraph->vertex[internalIndex].componentType = XME_CORE_COMPONENT_TYPE_INVALID;

    // Set the injected channel
    XME_CHECK
    (
        0 < xme_hal_safeString_snprintf
        (
            &rtGraph->vertex[internalIndex].vertexData[0],
            sizeof(rtGraph->vertex[internalIndex].vertexData),
            "%u|%u",
            (unsigned int) channelId,
            (unsigned int) inputPortQueueSize
        ),
        XME_STATUS_INTERNAL_ERROR
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createChannelSelectorWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_core_channelId_t sourceChannelId,
    xme_core_channelId_t sinkChannelId,
    uint8_t inputPortQueueSize,
    uint8_t internalIndex
)
{
    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR;
    rtGraph->vertex[internalIndex].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT; // Waypoints do not have an componentId id
    rtGraph->vertex[internalIndex].componentType = XME_CORE_COMPONENT_TYPE_INVALID;

    // Set the channel selector map for this communication
    XME_CHECK
    (
        0 < xme_hal_safeString_snprintf
        (
            &rtGraph->vertex[internalIndex].vertexData[0],
            sizeof(rtGraph->vertex[internalIndex].vertexData),
            "%u|%u|%u",
            (unsigned int) sourceChannelId,
            (unsigned int) sinkChannelId,
            (unsigned int) inputPortQueueSize
        ),
        XME_STATUS_INTERNAL_ERROR
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createMarshalerWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t internalIndex,
    uint8_t inputPortQueueSize
)
{
    rtGraph->vertex[internalIndex].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT; // waypoints does not have a component id. 
    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER;
    rtGraph->vertex[internalIndex].componentType = XME_CORE_COMPONENT_TYPE_INVALID;

    // inputPortQueueSize
    XME_CHECK
    (
        0 < xme_hal_safeString_snprintf
        (
            &rtGraph->vertex[internalIndex].vertexData[0],
            sizeof(rtGraph->vertex[internalIndex].vertexData),
            "%" PRIu32,
            (uint32_t)inputPortQueueSize
        ),
        XME_STATUS_INTERNAL_ERROR
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createDemarshalerWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t internalIndex,
    uint8_t inputPortQueueSize
)
{
    rtGraph->vertex[internalIndex].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT; // waypoints does not have a component id. 
    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER;
    rtGraph->vertex[internalIndex].componentType = XME_CORE_COMPONENT_TYPE_INVALID;

    // inputPortQueueSize
    XME_CHECK
    (
        0 < xme_hal_safeString_snprintf
        (
            &rtGraph->vertex[internalIndex].vertexData[0],
            sizeof(rtGraph->vertex[internalIndex].vertexData),
            "%" PRIu32,
            (uint32_t)inputPortQueueSize
        ),
        XME_STATUS_INTERNAL_ERROR
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createUDPSendWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t key[4],
    xme_com_interface_address_t address,
    uint8_t internalIndex
)
{
    int i;
    size_t currentLength = 0;
    uint32_t ip;
    uint16_t port;

    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND;
    rtGraph->vertex[internalIndex].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT; // Waypoints do not have an componentId id
    rtGraph->vertex[internalIndex].componentType = XME_CORE_COMPONENT_TYPE_INVALID;

    // Get each of the key values, separated by a pipe character
    for (i = 0; i < XME_WP_UDP_HEADER_KEY_LENGTH; i++)
    {
        XME_CHECK
        (        
            0 < xme_hal_safeString_snprintf
            (
                &rtGraph->vertex[internalIndex].vertexData[currentLength],
                sizeof(rtGraph->vertex[internalIndex].vertexData) - currentLength,
                "%u|",
                (unsigned int) key[i]
            ),
            XME_STATUS_INTERNAL_ERROR
        );
        currentLength = strlen(rtGraph->vertex[internalIndex].vertexData);
    }

    // Finally, print the pending parameters
    XME_CHECK
    (    
        XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4(&address, &ip, &port),
        XME_STATUS_INTERNAL_ERROR
    );

    XME_CHECK
    (
        0 < xme_hal_safeString_snprintf
        (
            &rtGraph->vertex[internalIndex].vertexData[currentLength],
            sizeof(rtGraph->vertex[internalIndex].vertexData) - currentLength,
            "%hu|%u.%u.%u.%u",
            (unsigned short int) xme_hal_net_ntohs(port),
            (unsigned int) (ip & 0xFF),
            (unsigned int) ((ip >> 8) & 0xFF),
            (unsigned int) ((ip >> 16) & 0xFF),
            (unsigned int) ((ip >> 24) & 0xFF)
        ),
        XME_STATUS_INTERNAL_ERROR
    );
    
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createUDPReceiveWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t key[4],
    xme_com_interface_address_t address,
    uint8_t internalIndex
)
{
    int i;
    size_t currentLength = 0;
    uint16_t port = 0;

    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE;
    rtGraph->vertex[internalIndex].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT; // Waypoints do not have a component id
    rtGraph->vertex[internalIndex].componentType = XME_CORE_COMPONENT_TYPE_INVALID;

    // Get each of the key values, separated by a pipe. 
    for (i = 0; i < XME_WP_UDP_HEADER_KEY_LENGTH; i++)
    {
        XME_CHECK
        (        
            0 < xme_hal_safeString_snprintf
            (
                &rtGraph->vertex[internalIndex].vertexData[currentLength],
                sizeof(rtGraph->vertex[internalIndex].vertexData) - currentLength,
                "%u|",
                (unsigned int) key[i]
            ),
            XME_STATUS_INTERNAL_ERROR
        );
        currentLength = strlen(rtGraph->vertex[internalIndex].vertexData);
    }

    // Finally, print the pending parameters
    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4(&address, NULL, &port), XME_STATUS_INTERNAL_ERROR);
    XME_CHECK
    (
        0 < xme_hal_safeString_snprintf
        (
            &rtGraph->vertex[internalIndex].vertexData[currentLength],
            sizeof(rtGraph->vertex[internalIndex].vertexData) - currentLength,
            "%hu",
            (unsigned short int) xme_hal_net_ntohs(port)
        ),
        XME_STATUS_INTERNAL_ERROR
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createMemCopyEdge
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_hal_graph_vertexId_t srcVertexId,
    xme_hal_graph_vertexId_t dstVertexId,
    xme_core_topic_t topicId,
    xme_core_channelId_t channelID,
    xme_hal_linkedList_descriptor_t* vertexIndex,
    uint8_t internalIndex
)
{
    uint16_t userTopicSize;

    rtGraph->edge[internalIndex].srcVertexIndex = 0;
    rtGraph->edge[internalIndex].sinkVertexIndex = 0;

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        *vertexIndex, 
        xme_core_pnp_pnpManager_vertexItem_t,
        vertexItem);
    {
        if (srcVertexId == vertexItem->vertexId)
        {
            rtGraph->edge[internalIndex].srcVertexIndex = vertexItem->index + 1; 
            if (0 != rtGraph->edge[internalIndex].sinkVertexIndex)
            {
                break;
            }
        }

        if (dstVertexId == vertexItem->vertexId)
        {
            rtGraph->edge[internalIndex].sinkVertexIndex = vertexItem->index + 1;
            if (0 != rtGraph->edge[internalIndex].srcVertexIndex)
            {
                break;
            }
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_CHECK(0 != rtGraph->edge[internalIndex].srcVertexIndex &&
        0 != rtGraph->edge[internalIndex].sinkVertexIndex, 
        XME_STATUS_INTERNAL_ERROR);

    rtGraph->edge[internalIndex].edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY;

    {
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_directory_topicRegistry_getTopicSize(topicId, &userTopicSize), XME_STATUS_INTERNAL_ERROR);

        // TODO: Error handling
    }

    XME_ASSERT(0 != userTopicSize);

    XME_CHECK
    (
        0 < xme_hal_safeString_snprintf
            (
                rtGraph->edge[internalIndex].edgeData,
                sizeof(rtGraph->edge[internalIndex].edgeData),
                "%"PRIu32"|%"PRIu16"|%"PRIu32,
                topicId,
                userTopicSize,
                channelID
            ),
        XME_STATUS_INTERNAL_ERROR
    );

    return XME_STATUS_SUCCESS;
}

xme_core_component_t
xme_core_pnp_pnpManager_getComponentIdFromRTGraph
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph
)
{
    int i;

    for (i = 0; i < 10; i++)
    {
        if (rtGraph->vertex[i].vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT)
        {
            return rtGraph->vertex[i].componentId;
        }
    }

    return XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT;
}

xme_status_t
xme_core_pnp_pnpManager_registerNode
(
    xme_core_node_nodeId_t nodeId
)
{
    xme_hal_table_rowHandle_t nodeListHandle;
    xme_core_node_nodeId_t* nodeItem;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);

    // Search for an existing item for this node
    nodeListHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;

    // 1. Update the list of registered nodes. 
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_pnp_pnpManager_nodeList, 
        xme_hal_table_rowHandle_t, 
        nodeListHandle, 
        xme_core_node_nodeId_t, 
        nodeItem, 
        *nodeItem == nodeId
    );

    // The node id is already registered
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE == nodeListHandle, XME_STATUS_ALREADY_EXIST);

    nodeListHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpManager_nodeList);
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != nodeListHandle, XME_STATUS_OUT_OF_RESOURCES);

    nodeItem = (xme_core_node_nodeId_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpManager_nodeList, nodeListHandle);
    XME_CHECK(NULL != nodeItem, XME_STATUS_OUT_OF_RESOURCES);

    (void) xme_hal_mem_copy(nodeItem, &nodeId, sizeof(xme_core_node_nodeId_t));

    // 2. Create an entry on last assigned component ids per node. 
    {
        xme_hal_table_rowHandle_t lastComponentIdhandle;
        xme_core_pnp_pnpManager_lastAssignedComponentIdPerNode_t* lastAssignedComponentIdPerNode;

        lastComponentIdhandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
        lastAssignedComponentIdPerNode = NULL;

        XME_HAL_TABLE_GET_NEXT
        (
            xme_core_pnp_pnpManager_lastAssignedComponentIdsPerNode, 
            xme_hal_table_rowHandle_t, 
            lastComponentIdhandle, 
            xme_core_pnp_pnpManager_lastAssignedComponentIdPerNode_t, 
            lastAssignedComponentIdPerNode, 
            lastAssignedComponentIdPerNode->nodeId == nodeId
        );

        // If the node has not an entry, create one. 
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == lastComponentIdhandle)
        {
            lastComponentIdhandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpManager_lastAssignedComponentIdsPerNode);
            XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != lastComponentIdhandle, XME_STATUS_OUT_OF_RESOURCES);

            lastAssignedComponentIdPerNode = (xme_core_pnp_pnpManager_lastAssignedComponentIdPerNode_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpManager_lastAssignedComponentIdsPerNode, lastComponentIdhandle);
            XME_CHECK(NULL != lastAssignedComponentIdPerNode, XME_STATUS_OUT_OF_RESOURCES);

            lastAssignedComponentIdPerNode->nodeId = nodeId;
            lastAssignedComponentIdPerNode->lastAssignedComponentId = (xme_core_component_t) XME_CORE_COMPONENT_PNPMANAGER_INITIAL_ID;
        }
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_deregisterNode
(
    xme_core_node_nodeId_t nodeId
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeId_t* nodeItem;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);

    // Search for an existing item for this node
    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_pnp_pnpManager_nodeList, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_core_node_nodeId_t, 
        nodeItem, 
        *nodeItem == nodeId
    );

    // The node id is not registered
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NOT_FOUND);

    return XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpManager_nodeList, handle);
}

bool
xme_core_pnp_pnpManager_isNodeRegistered
(
    xme_core_node_nodeId_t nodeId
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeId_t* nodeItem;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, false);

    // Search for an existing item for this node
    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_pnp_pnpManager_nodeList, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_core_node_nodeId_t, 
        nodeItem, 
        *nodeItem == nodeId
    );

    // The node id is not registered
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, false);

    return true;
}

xme_status_t
xme_core_pnp_pnpManager_registerInterfaceAddress
(
    xme_core_node_nodeId_t nodeId,
    xme_com_interface_address_t interfaceAddress
)
{
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_core_directory_nodeRegistryController_addInterface(nodeId, interfaceAddress),
        XME_STATUS_INTERNAL_ERROR
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_announceComponentInstanceManifest
(
    xme_core_topic_pnp_componentInstanceManifest_t* manifest
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    uint8_t i = 0u;

    XME_CHECK(NULL != manifest, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != manifest->nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(xme_core_pnp_pnpManager_isNodeRegistered(manifest->nodeId), XME_STATUS_NOT_FOUND);

    for (i = 0u; i < sizeof(manifest->components)/sizeof(manifest->components[0]); i++)
    {
        xme_core_component_t componentID = manifest->components[i].componentId;
        xme_core_componentType_t componentType = manifest->components[i].componentType;
        const char* const initializationString = &manifest->components[i].initializationString[0];
        xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
        xme_core_nodeMgr_compRep_componentHandle_t componentHandle;
        xme_core_nodeMgr_compRep_componentHandle_t foreignComponentHandle = manifest->components[i].componentHandle;

        // If componentType is not set, this item is considered empty
        if (XME_CORE_COMPONENT_TYPE_INVALID == componentType)
        {
            continue;
        }

        builder = xme_core_nodeMgr_compRep_createBuilder(manifest->nodeId, componentType);
        if (NULL == builder)
        {
            XME_LOG_C
            (
                XME_CORE_COMPONENT_TYPE_PNPMANAGER,
                XME_LOG_ERROR,
                "Error building component instance from received instance manifest (nodeId = %d, componentID = %d, componentType = %d).\n",
                manifest->nodeId, componentID, componentType
            );
            continue;
        }
        xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID);
        xme_core_nodeMgr_compRep_builderSetInitializationString(builder, initializationString);
        xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(builder, foreignComponentHandle);
        {
            uint16_t portIndex = 0u;

            for (portIndex = 0u; portIndex < sizeof(manifest->components[0].portData)/sizeof(manifest->components[0].portData[0]); portIndex++)
            {
                uint8_t queueSize = manifest->components[i].portData[portIndex].queueSize;

                if (0 != queueSize)
                {
                    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, portIndex, queueSize);
                }
            }
        }
        {
            uint16_t funcIndex = 0u;

            for (funcIndex = 0u; funcIndex < sizeof(manifest->components[0].functionData)/sizeof(manifest->components[0].functionData[0]); funcIndex++)
            {
                xme_hal_time_timeInterval_t executionPeriod = manifest->components[i].functionData[funcIndex].executionPeriod;

                if (0 != executionPeriod)
                {
                    xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, funcIndex, executionPeriod);
                }
            }
        }
        status = xme_core_nodeMgr_compRep_build(builder, &componentHandle);
        if (XME_STATUS_SUCCESS != status)
        {
            XME_LOG_C
            (
                XME_CORE_COMPONENT_TYPE_PNPMANAGER,
                XME_LOG_ERROR,
                "Error building component instance from received instance manifest (nodeId = %d, componentID = %d, componentType = %d).\n",
                manifest->nodeId, componentID, componentType
            );
            continue;
        }

        XME_CHECK(XME_CORE_NODEMGR_COMPREP_STATE_PREPARED == xme_core_nodeMgr_compRep_getState(componentHandle), XME_STATUS_INVALID_PARAMETER); // This will also catch non-existing components

        status = xme_core_pnp_pnpManager_announceNewComponentOnNode(componentHandle);
        XME_CHECK_MSG_C
        (
            XME_STATUS_SUCCESS == status,
            XME_STATUS_INTERNAL_ERROR,
            XME_CORE_COMPONENT_TYPE_PNPMANAGER,
            XME_LOG_ERROR,
            "The component %d of component type %d with initialization string \"%s\" in the manifest for node %d cannot be announced.\n",
            componentID, componentType, initializationString, manifest->nodeId
        );
    }

    status = xme_core_pnp_pnpManager_updateConfiguration();
    XME_CHECK_MSG_C
    (
        XME_STATUS_SUCCESS == status,
        XME_STATUS_INTERNAL_ERROR,
        XME_CORE_COMPONENT_TYPE_PNPMANAGER,
        XME_LOG_ERROR,
        "Configuration update failed.\n"
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_storeUnestablishedLogicalRoutes
(
    xme_core_pnp_lrm_logicalRoutes_t* logicalRoutes
)
{
    xme_hal_graph_vertexId_t srcVertexId;
    xme_hal_graph_edgeId_t edgeId;
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData = NULL;
    xme_core_pnp_dataLinkGraph_vertexData_t* srcVertexData = NULL;

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initVertexIterator(logicalRoutes), XME_STATUS_INTERNAL_ERROR);

    while (xme_hal_graph_hasNextVertex(logicalRoutes))
    {
        srcVertexId = xme_hal_graph_nextVertex(logicalRoutes);

        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(logicalRoutes, srcVertexId, (void**) &srcVertexData), XME_STATUS_INTERNAL_ERROR);

        XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == srcVertexData->vertexType);

        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initOutgoingEdgeIterator(logicalRoutes, srcVertexId), XME_STATUS_INTERNAL_ERROR);

        while (xme_hal_graph_hasNextOutgoingEdge(logicalRoutes, srcVertexId))
        {
            edgeId = xme_hal_graph_nextOutgoingEdge(logicalRoutes, srcVertexId);
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(logicalRoutes, edgeId, (void**) &edgeData), XME_STATUS_INTERNAL_ERROR);

            if (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType) {

                XME_CHECK(XME_CORE_INVALID_CHANNEL_ID != edgeData->edgeData.logicalRouteEdge.channelId, XME_STATUS_INTERNAL_ERROR);

                if (!edgeData->edgeData.logicalRouteEdge.established) // only include if it is not established. 
                {
                    xme_hal_table_rowHandle_t handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
                    xme_core_pnp_pnpManager_unestablishedLogicalRoute_t* unestablishedLogicalRoute;

                    XME_HAL_TABLE_GET_NEXT
                    (
                        xme_core_pnp_pnpManager_unestablishedLogicalRoutes, 
                        xme_hal_table_rowHandle_t, 
                        handle, 
                        xme_core_pnp_pnpManager_unestablishedLogicalRoute_t, 
                        unestablishedLogicalRoute, 
                        unestablishedLogicalRoute->channelId == edgeData->edgeData.logicalRouteEdge.channelId
                    );

                    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == handle)
                    {
                        // if the entry does not exist, create a new entry in the table. 
                        handle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpManager_unestablishedLogicalRoutes);
                        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_OUT_OF_RESOURCES);
                        unestablishedLogicalRoute = (xme_core_pnp_pnpManager_unestablishedLogicalRoute_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpManager_unestablishedLogicalRoutes, handle);
                        XME_ASSERT(NULL != unestablishedLogicalRoute);
                        unestablishedLogicalRoute->edgeId = edgeId;
                        unestablishedLogicalRoute->channelId = edgeData->edgeData.logicalRouteEdge.channelId;
                    }
                }
            }
        }
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiOutgoingEdgeIterator(logicalRoutes, srcVertexId), XME_STATUS_INTERNAL_ERROR);
    }

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiVertexIterator(logicalRoutes), XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_core_component_t
xme_core_pnp_pnpManager_generateNewComponentIdForNode
(
    xme_core_node_nodeId_t nodeId
)
{
    xme_core_pnp_pnpManager_lastAssignedComponentIdPerNode_t* lastAssignedComponentIdPerNode;
    xme_hal_table_rowHandle_t handle;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT);

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    lastAssignedComponentIdPerNode = NULL;

    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_pnp_pnpManager_lastAssignedComponentIdsPerNode, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_core_pnp_pnpManager_lastAssignedComponentIdPerNode_t, 
        lastAssignedComponentIdPerNode, 
        lastAssignedComponentIdPerNode->nodeId == nodeId
    );

    XME_CHECK(NULL != lastAssignedComponentIdPerNode, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT);

    // just add 1 to the last assigned component in the node.
    lastAssignedComponentIdPerNode->lastAssignedComponentId = (xme_core_component_t) (((xme_maxSystemValue_t)lastAssignedComponentIdPerNode->lastAssignedComponentId) + 1);

    // and return the current value. 
    return lastAssignedComponentIdPerNode->lastAssignedComponentId;
}

xme_status_t
xme_core_pnp_pnpManager_announceChannelId
(
    xme_core_node_nodeId_t publicationNodeID,
    xme_core_component_t publicationComponentID,
    uint16_t publicationPortIndex,
    xme_core_node_nodeId_t subscriptionNodeID,
    xme_core_component_t subscriptionComponentID,
    uint16_t subscriptionPortIndex,
    xme_core_channelId_t channelID
)
{
    return xme_core_pnp_lrm_announceChannelID
    (
        publicationNodeID, publicationComponentID, publicationPortIndex,
        subscriptionNodeID, subscriptionComponentID, subscriptionPortIndex,
        channelID
    );
}
/**
 * @}
 */
