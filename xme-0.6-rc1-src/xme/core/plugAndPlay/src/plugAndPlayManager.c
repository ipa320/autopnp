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
 * $Id: plugAndPlayManager.c 5214 2013-09-27 14:24:34Z ruiz $
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

#include "xme/hal/include/mem.h"
#include "xme/hal/include/net.h"

#include <inttypes.h>

/******************************************************************************/
/***   Static variables                                                     ***/
/******************************************************************************/

/**
 * \brief Table xme_core_pnp_pnpManager_instanceList
 */
XME_HAL_TABLE
(
    xme_core_pnp_pnpManager_nodeComponentInstance_t,
    xme_core_pnp_pnpManager_nodeComponentInstanceList,
    XME_CORE_PNP_PNPMANAGER_INSTANCE_LIST_SIZE
);

/**
 * \brief Table xme_core_pnp_pnpManager_nodeList.
 * \details This table contains the list of registered nodes by the login manager. 
 *          Only registered nodes can perform operations of plug and play, such as
 *          sending component instance manifests. 
 */
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
xme_hal_singlyLinkedList_t(XME_CORE_PNP_PNPMANAGER_RUNTIMEGRAPHS_LIST_SIZE) rtGraphList;

/**
 * \brief Last assigned component id per node table.
 */
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
 * \brief Retrieves the size of a data packet of the given topic.
 *
 * \note This function should be implemented by the user or generated
 *       from CHROMOSOME Modeling Tool (XMT).
 *
 * \param[in] topicId Unique topic identifier.
 *
 * \return Size of the topic.
 */
extern uint16_t
xme_core_topicUtil_getTopicSize
(
    xme_core_topic_t topicId
);

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

    XME_HAL_TABLE_INIT(xme_core_pnp_pnpManager_nodeComponentInstanceList);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpManager_nodeList);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpManager_unestablishedLogicalRoutes);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpManager_lastAssignedComponentIdsPerNode);

    XME_HAL_SINGLYLINKEDLIST_INIT(rtGraphList);

    return XME_STATUS_SUCCESS;
}

void
xme_core_pnp_pnpManager_fini(void)
{
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpManager_nodeComponentInstanceList);
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpManager_nodeList);
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpManager_unestablishedLogicalRoutes);
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpManager_lastAssignedComponentIdsPerNode);

    XME_HAL_SINGLYLINKEDLIST_FINI(rtGraphList);
}

xme_status_t
xme_core_pnp_pnpManager_instantiateComponentOnNode
(
    xme_core_componentType_t componentType,
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t existingComponentId
)
{
    xme_core_pnp_lrm_logicalRoutes_t logicalRouteGraph;
    xme_core_pnp_ncc_physicalRoutes_t physicalRouteGraph;
    xme_core_pnp_pnpManager_physicalGraphList_t splittedPhysicalGraphList;
    xme_core_componentManifest_t* componentManifest; // this is used to store the component manifest. 
    xme_core_component_t currentComponentId;

    // Performs checks on incoming data. 
    XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentType, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    
    // Get the component type manifest
    componentManifest = (xme_core_componentManifest_t*)xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));
    XME_CHECK(XME_STATUS_SUCCESS == 
        xme_core_manifestRepository_findComponentManifest(componentType, componentManifest),
        XME_STATUS_INVALID_CONFIGURATION);
    XME_ASSERT(NULL != componentManifest);

    // Create an entry in the instance list. 
    //instanceId = xme_core_pnp_pnpManager_createInstance(nodeId, xme_core_pnp_pnpManager_nextComponentId);
    // note: we do not mind if there are more instances for the same node of the same component. 
    //       the function caller has specified that he wants to create a component of type 'componentType'
    //       in the node labeled as 'nodeId'
    if (existingComponentId != XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT)
    {
        // already existing component id
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createInstance(nodeId, existingComponentId, componentManifest->componentType), XME_STATUS_INTERNAL_ERROR);

        // Announce component manifest ports to logical route manager.
        XME_CHECK(XME_STATUS_SUCCESS == 
            xme_core_pnp_pnpManager_announcePorts(componentManifest, nodeId, existingComponentId), 
            XME_STATUS_INTERNAL_ERROR);

        currentComponentId = existingComponentId;
    }
    else
    {
        // Get the new component id. 
        xme_core_component_t newComponentId = xme_core_pnp_pnpManager_generateNewComponentIdForNode(nodeId);
        XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != newComponentId, XME_STATUS_INTERNAL_ERROR);

        XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createInstance(nodeId, newComponentId, componentManifest->componentType), XME_STATUS_INTERNAL_ERROR);
        // Announce component manifest ports to logical route manager.
        XME_CHECK
        (
            XME_STATUS_SUCCESS == 
                xme_core_pnp_pnpManager_announcePorts(componentManifest, nodeId, newComponentId), 
            XME_STATUS_INTERNAL_ERROR
        );

        currentComponentId = newComponentId;
    }

    // Ask Logical Route Manager to calculate new logical routes and obtain the corresponding routes. 
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_init((xme_hal_graph_graph_t*) &logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);

    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS == xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &logicalRouteGraph),
        XME_STATUS_INTERNAL_ERROR,
        {
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini((xme_hal_graph_graph_t*) &logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
        }
    );

    // label already established logical routes as established
    if (existingComponentId != XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT)
    {
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_setEstablishedRoutes(&logicalRouteGraph, existingComponentId), XME_STATUS_INTERNAL_ERROR);
    }


    // Log information about logical routes. 
    xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_DEBUG, "LRM Logical Route Graph", &logicalRouteGraph);
    
    // - Forward graph to Network Communication Calculator
    // - Obtain complete waypoint graph from Network Communication Calculator
    // - Ask Network Communication Calculator to calculate waypoints
    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS == xme_core_pnp_ncc_getPhysicalRoutes((xme_hal_graph_graph_t*) &logicalRouteGraph, &physicalRouteGraph),
        XME_STATUS_INTERNAL_ERROR,
        {
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(&physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(&logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
        }
    );

    xme_core_pnp_dataLinkGraph_dumpDataLinkGraph(XME_LOG_DEBUG, "NCC Physical Route Graph", &physicalRouteGraph);

    // - Split complete waypoint graph according to nodes and remove unneeded information
    XME_HAL_SINGLYLINKEDLIST_INIT(splittedPhysicalGraphList);

    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_splitGraph(&physicalRouteGraph, &splittedPhysicalGraphList),
        XME_STATUS_INTERNAL_ERROR,
        {
            XME_HAL_SINGLYLINKEDLIST_FINI(splittedPhysicalGraphList);
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

            status = xme_core_pnp_pnpManager_processGraph(splittedPhysicalGraphItem->splitGraph, splittedPhysicalGraphItem->nodeId);
            XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, XME_STATUS_INTERNAL_ERROR);
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    }

    // Declare the instance as prepared. 
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_updateInstance(nodeId, currentComponentId, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_PREPARED),
        XME_STATUS_INTERNAL_ERROR);

    // Store the unestablished logical routes. 
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_storeUnestablishedLogicalRoutes(&logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);

    // Finalize the declared physical and logical routes, including splitted graph.
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(&physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(&logicalRouteGraph), XME_STATUS_INTERNAL_ERROR);
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
                XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(*outSplitGraphs, physicalGraphListItem), 
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
    xme_core_node_nodeId_t nodeId
)
{
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph;
    uint8_t i;
    xme_hal_singlyLinkedList_t(XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH) vertexIndex;
    xme_core_pnp_pnpManager_rtGraphItem_t* rtGraphItem;

    XME_CHECK(NULL != physicalRouteGraph, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(xme_hal_graph_getVertexCount(physicalRouteGraph) <= XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_PHYSICAL_ROUTE, XME_STATUS_UNSUPPORTED);
    XME_CHECK(xme_hal_graph_getEdgeCount(physicalRouteGraph) <= XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_PHYSICAL_ROUTE, XME_STATUS_UNSUPPORTED);

    rtGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

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

        XME_CHECK(i < XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH, XME_STATUS_UNSUPPORTED);

        XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId);
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(physicalRouteGraph, vertexId, (void**) &routeVertexData), XME_STATUS_INTERNAL_ERROR);

        XME_CHECK(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_INVALID_VERTEXTYPE != routeVertexData->vertexType, 
            XME_STATUS_INTERNAL_ERROR);

        if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT) {
            xme_core_componentType_t componentType;

            XME_LOG(XME_LOG_DEBUG, "The %d vertex is a component port\n", i);
        
            // Get the component type. 
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_getComponentType
                    (routeVertexData->nodeId, 
                    routeVertexData->vertexData.componentPortVertex.componentId, 
                    &componentType), 
                XME_STATUS_INTERNAL_ERROR);

            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createComponent(rtGraph,    
                    routeVertexData->vertexData.componentPortVertex,
                    componentType,
                    i),
                XME_STATUS_INTERNAL_ERROR);
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER)
        {
            XME_LOG(XME_LOG_DEBUG, "The %d vertex is a marshaler waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createMarshalerWaypoint(rtGraph, i),
                XME_STATUS_INTERNAL_ERROR);
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND)
        {
            XME_LOG(XME_LOG_DEBUG, "The %d vertex is a UPD send waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createUDPSendWaypoint(rtGraph, 
                    routeVertexData->vertexData.waypointUdpSendVertex.key,
                    routeVertexData->vertexData.waypointUdpSendVertex.destination,
                    i),
                XME_STATUS_INTERNAL_ERROR);
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE)
        {
            XME_LOG(XME_LOG_DEBUG, "The %d vertex is a UPD receive waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createUDPReceiveWaypoint(rtGraph, 
                    routeVertexData->vertexData.waypointUdpReceiveVertex.key,
                    routeVertexData->vertexData.waypointUdpReceiveVertex.host,
                    i),
                XME_STATUS_INTERNAL_ERROR);
        }
        else if (routeVertexData->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER)
        {
            XME_LOG(XME_LOG_DEBUG, "The %d vertex is a demarshaler waypoint\n", i);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createDemarshalerWaypoint(rtGraph, i),
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

        XME_CHECK(i < XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH, XME_STATUS_UNSUPPORTED);

        XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != vertexId); 
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initOutgoingEdgeIterator(physicalRouteGraph, vertexId), XME_STATUS_INTERNAL_ERROR);
        
        while(xme_hal_graph_hasNextOutgoingEdge(physicalRouteGraph, vertexId))
        {
            xme_core_pnp_dataLinkGraph_edgeData_t* routeEdgeData;
            xme_hal_graph_vertexId_t srcVertexId;
            xme_hal_graph_vertexId_t dstVertexId;
            xme_hal_graph_edgeId_t edgeId = xme_hal_graph_nextOutgoingEdge(physicalRouteGraph, vertexId);

            XME_CHECK(i < XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH, XME_STATUS_UNSUPPORTED);

            XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != edgeId);

            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(physicalRouteGraph, edgeId, (void**) &routeEdgeData), XME_STATUS_INTERNAL_ERROR);

            XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY == routeEdgeData->edgeType);

            srcVertexId = xme_hal_graph_getSourceVertex(physicalRouteGraph, edgeId);
            dstVertexId = xme_hal_graph_getSinkVertex(physicalRouteGraph, edgeId);
            XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != srcVertexId, XME_STATUS_INVALID_PARAMETER);
            XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != dstVertexId, XME_STATUS_INVALID_PARAMETER);

            XME_LOG(XME_LOG_DEBUG, "The %d edge is a mem copy: from %d to %d \n", i, srcVertexId, dstVertexId);
            XME_CHECK(
                XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_createMemCopyEdge(
                    rtGraph,
                    srcVertexId,
                    dstVertexId,
                    routeEdgeData->topicId,
                    (xme_hal_linkedList_descriptor_t*) &vertexIndex,
                    i),
                    XME_STATUS_INTERNAL_ERROR);
            i++;
        }

        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiOutgoingEdgeIterator(physicalRouteGraph, vertexId), XME_STATUS_INTERNAL_ERROR);
    }

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiVertexIterator(physicalRouteGraph), XME_STATUS_INTERNAL_ERROR);

    // 3. Remove the previous runtime graph associated to the node. 
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        rtGraphList,
        xme_core_pnp_pnpManager_rtGraphItem_t,
        rtGraphListItem
    );
    {
        if (rtGraphListItem->nodeId == nodeId) {
            XME_CHECK(0 != XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(rtGraphList, rtGraphListItem, false), XME_STATUS_INTERNAL_ERROR);
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
    XME_HAL_SINGLYLINKEDLIST_FINI(vertexIndex);
    
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

                XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_updateInstance(nodeId, componentId, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_SUBMITTED),
                    XME_STATUS_INTERNAL_ERROR);

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
        if (rtGraphItem->announced == false)
        {
            rtGraphItem->announced = true;

            if (NULL != rtGraphItem->rtGraph)
            {
                xme_core_component_t componentId;

                *outGraph = *rtGraphItem->rtGraph;
                componentId = xme_core_pnp_pnpManager_getComponentIdFromRTGraph(rtGraphItem->rtGraph);

                XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INTERNAL_ERROR);

                XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_updateInstance(rtGraphItem->nodeId, componentId, XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_SUBMITTED),
                    XME_STATUS_INTERNAL_ERROR);

                return XME_STATUS_SUCCESS;
            }

        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_NOT_FOUND;
}

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

xme_status_t
xme_core_pnp_pnpManager_announcePorts
(
    xme_core_componentManifest_t* componentManifest,
    xme_core_node_nodeId_t nodeId, 
    xme_core_component_t componentId
)
{
    uint16_t numPorts, i;

    XME_CHECK(NULL != componentManifest, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);

    // Gets the port information from the component manifest.
    numPorts = sizeof(componentManifest->portManifests) / sizeof(componentManifest->portManifests[0]);
    for (i = 0; i < numPorts; i++)
    {
        xme_core_componentPortManifest_t* portManifest = &componentManifest->portManifests[i];
        XME_ASSERT(NULL != portManifest);

        // Stop on first empty port descriptor
        if (portManifest->portType == XME_CORE_COMPONENT_PORTTYPE_INVALID)
        {
            break;
        }

        XME_CHECK
        (
            XME_STATUS_SUCCESS == 
            xme_core_pnp_lrm_announcePort(
                nodeId,
                componentId,
                portManifest->portType,
                i,
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

///////////////////////////////////////////////////////////////////////////////////////

xme_status_t 
xme_core_pnp_pnpManager_createInstance
(
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t componentId,
    xme_core_componentType_t componentType
)
{
    xme_core_pnp_pnpManager_nodeComponentInstance_t* nodeComponentInstance;
    xme_hal_table_rowHandle_t rowHandle;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentType, XME_STATUS_INVALID_PARAMETER);

    rowHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpManager_nodeComponentInstanceList);

    nodeComponentInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpManager_nodeComponentInstanceList, rowHandle);
    XME_CHECK(NULL != nodeComponentInstance, XME_STATUS_OUT_OF_RESOURCES);
    nodeComponentInstance->nodeId = nodeId;
    nodeComponentInstance->componentId = componentId;
    nodeComponentInstance->componentType = componentType;
    nodeComponentInstance->status = XME_CORE_PNP_PNPMANAGER_NODECOMPONENTINSTANCE_REQUESTED;

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_pnp_pnpManager_getComponentType
(
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t componentId,
    xme_core_componentType_t* componentType
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_pnp_pnpManager_nodeComponentInstance_t* instanceItem;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    instanceItem = NULL;

    XME_CHECK(nodeId != XME_CORE_NODE_INVALID_NODE_ID, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(componentId != XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != componentType, XME_STATUS_INVALID_PARAMETER);

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

    *componentType = instanceItem->componentType;

    return XME_STATUS_SUCCESS;
}

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

xme_status_t
xme_core_pnp_pnpManager_removeInstance
(
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t componentId
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_pnp_pnpManager_nodeComponentInstance_t* instanceItem;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    instanceItem = NULL;

    XME_CHECK(nodeId != XME_CORE_NODE_INVALID_NODE_ID, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(componentId != XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, XME_STATUS_INVALID_PARAMETER);

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

    return XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpManager_nodeComponentInstanceList, handle);
}

xme_status_t
xme_core_pnp_pnpManager_createComponent
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_core_pnp_dataLinkGraph_componentPortVertexData_t componentPortData,
    xme_core_componentType_t componentType,
    uint8_t internalIndex
)
{
    rtGraph->vertex[internalIndex].componentId = componentPortData.componentId;
    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;
    rtGraph->vertex[internalIndex].componentType = componentType;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createMarshalerWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t internalIndex
)
{
    rtGraph->vertex[internalIndex].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT; // waypoints does not have a component id. 
    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER;
    rtGraph->vertex[internalIndex].componentType = XME_CORE_COMPONENT_TYPE_INVALID;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_createDemarshalerWaypoint
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    uint8_t internalIndex
)
{
    rtGraph->vertex[internalIndex].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT; // waypoints does not have a component id. 
    rtGraph->vertex[internalIndex].vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER;
    rtGraph->vertex[internalIndex].componentType = XME_CORE_COMPONENT_TYPE_INVALID;

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
        userTopicSize = xme_core_topicUtil_getTopicSize(topicId);
    }

    XME_ASSERT(0 != userTopicSize);

    XME_CHECK
    (        
        0 < xme_hal_safeString_snprintf(rtGraph->edge[internalIndex].edgeData, 
            sizeof(rtGraph->edge[internalIndex].edgeData), 
            "%"PRIu32"|%"PRIu16"", 
            topicId, 
            userTopicSize),
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
            lastAssignedComponentIdPerNode->lastAssignedComponentId = (xme_core_component_t) (((uint32_t) nodeId) * 10u);
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
    xme_core_node_nodeId_t nodeId;
    xme_core_component_t componentId;
    xme_core_componentType_t componentType;
    uint8_t i;

    XME_CHECK(NULL != manifest, XME_STATUS_INVALID_PARAMETER);

    nodeId = manifest->nodeId;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(xme_core_pnp_pnpManager_isNodeRegistered(nodeId), XME_STATUS_NOT_FOUND);

    for(i = 0; i < 10; i++)
    {
        componentId = manifest->components[i].componentId;
        componentType = manifest->components[i].componentType;

        // if both are not theree 
        if (componentId == XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT &&
            componentType == XME_CORE_COMPONENT_TYPE_INVALID)
        {
            continue;
        }

        // This check is unnecessary, because it can include both running and not running components. 
        //XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
        
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == xme_core_pnp_pnpManager_instantiateComponentOnNode(componentType, nodeId, componentId), 
            XME_STATUS_INTERNAL_ERROR, 
            XME_LOG_ERROR,
            "[PnPManager] The component %d of component type %d in the manifest for node %d cannot be announced.\n", 
            componentId, componentType, nodeId
        );
    }

    return XME_STATUS_SUCCESS;

}

xme_status_t
xme_core_pnp_pnpManager_setEstablishedRoutes
(
    xme_core_pnp_lrm_logicalRoutes_t* logicalRoutes,
    xme_core_component_t componentId
)
{
    xme_hal_graph_vertexId_t vertexId;
    xme_hal_graph_edgeId_t edgeId;
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData = NULL;
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData = NULL;

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initVertexIterator(logicalRoutes), XME_STATUS_INTERNAL_ERROR);

    while (xme_hal_graph_hasNextVertex(logicalRoutes))
    {
        vertexId = xme_hal_graph_nextVertex(logicalRoutes);

        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(logicalRoutes, vertexId, (void**) &vertexData), XME_STATUS_INTERNAL_ERROR);

        XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vertexData->vertexType);

        if (vertexData->vertexData.componentPortVertex.componentId == componentId)
        {
            // Set as established outgoing edges
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initOutgoingEdgeIterator(logicalRoutes, vertexId), XME_STATUS_INTERNAL_ERROR);
            while (xme_hal_graph_hasNextOutgoingEdge(logicalRoutes, vertexId))
            {
                edgeId = xme_hal_graph_nextOutgoingEdge(logicalRoutes, vertexId);
                XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(logicalRoutes, edgeId, (void**) &edgeData), XME_STATUS_INTERNAL_ERROR);

                XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType);

                edgeData->edgeData.logicalRouteEdge.established = true;
            }
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiOutgoingEdgeIterator(logicalRoutes, vertexId), XME_STATUS_INTERNAL_ERROR);

            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_initIncomingEdgeIterator(logicalRoutes, vertexId), XME_STATUS_INTERNAL_ERROR);
            while (xme_hal_graph_hasNextIncomingEdge(logicalRoutes, vertexId))
            {
                edgeId = xme_hal_graph_nextIncomingEdge(logicalRoutes, vertexId);
                XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(logicalRoutes, edgeId, (void**) &edgeData), XME_STATUS_INTERNAL_ERROR);

                XME_ASSERT(XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType);

                edgeData->edgeData.logicalRouteEdge.established = true;
            }
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiIncomingEdgeIterator(logicalRoutes, vertexId), XME_STATUS_INTERNAL_ERROR);
        }
    }

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_finiVertexIterator(logicalRoutes), XME_STATUS_INTERNAL_ERROR);

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

/**
 * @}
 */
