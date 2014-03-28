/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: networkTopologyCalculator.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Network Topology Calculator.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/directory/include/networkTopologyCalculator.h"

#include "xme/hal/include/mem.h"
/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief The network graph.
 */
xme_hal_graph_graph_t networkGraph;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief The network graph.
 */
int xme_core_directory_networkTopologyCalculator_vertexCompareFilterNodeId(void* vertexData1, void* vertexData2);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
int
xme_core_directory_networkTopologyCalculator_vertexCompareFilterNodeId
(
    void* vertexData1,
    void* vertexData2
)
{
    xme_core_directory_networkTopologyCalculator_vertexData_t* vertexDataFirst;
    xme_core_directory_networkTopologyCalculator_vertexData_t* vertexDataSecond;

    vertexDataFirst = (xme_core_directory_networkTopologyCalculator_vertexData_t*) vertexData1;
    vertexDataSecond = (xme_core_directory_networkTopologyCalculator_vertexData_t*) vertexData2;

    return (vertexDataFirst->nodeId == vertexDataSecond->nodeId) ? 0 : -1;
}

xme_status_t
xme_core_directory_networkTopologyCalculator_init
(
    void* initStruct
)
{
    xme_status_t status;

    XME_UNUSED_PARAMETER(initStruct);

    //Initialize the network graph that is later modified and represents the network topology
    status = xme_hal_graph_init(&networkGraph);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);
    status = xme_hal_graph_setVertexCompareCallback(&networkGraph, xme_core_directory_networkTopologyCalculator_vertexCompareFilterNodeId);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    return XME_STATUS_SUCCESS;
}

void
xme_core_directory_networkTopologyCalculator_fini(void)
{
    xme_status_t status = xme_hal_graph_fini(&networkGraph);
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
}

xme_status_t
xme_core_directory_networkTopologyCalculator_updateNetwork
(
    const xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t* update
)
{
    xme_core_directory_networkTopologyCalculator_vertexData_t* currentNode;
    xme_core_directory_networkTopologyCalculator_vertexData_t* sendingNode;
    xme_core_directory_networkTopologyCalculator_vertexData_t* vertexData;
    xme_hal_graph_vertexId_t currentVertex;
    xme_hal_graph_vertexId_t sendingVertex;
    xme_core_directory_networkTopologyCalculator_edgeData_t* edgeProperties;
    xme_status_t status;
    xme_hal_graph_edgeId_t nextEdge;
    unsigned int i;
    xme_core_directory_networkTopologyCalculator_vertexData_t* data;
    xme_hal_graph_vertexId_t startVertex = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    //Check whether the was an error while processing the neighborhoodInformation
    /*if(update->overflow)
    {
        return XME_STATUS_INTERNAL_ERROR;
    }*/

    currentNode = (xme_core_directory_networkTopologyCalculator_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_vertexData_t));
    currentNode->nodeId = update->nodeId;

    currentVertex = xme_hal_graph_getNextVertexWithDataComparison(&networkGraph, currentNode, startVertex);

    if(currentVertex == XME_HAL_GRAPH_INVALID_VERTEX_ID) //The node is not yet in the graph and needs to be added first
    {
        currentNode->lastSeenBeforeTicks = NUMBER_OF_TICKS;
        currentVertex = xme_hal_graph_addVertex(&networkGraph, currentNode);
        XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentVertex, XME_STATUS_INTERNAL_ERROR);
    }
    else //The node already exists and we need to remove all its incoming edges
    {
        status = xme_hal_graph_getVertexData(&networkGraph, currentVertex, (void**) &vertexData);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        if(!update->overflow)
        {
            (vertexData->lastSeenBeforeTicks)++;
        }
        status = xme_hal_graph_initIncomingEdgeIterator(&networkGraph, currentVertex);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        while(xme_hal_graph_hasNextIncomingEdge(&networkGraph, currentVertex))
        {
            nextEdge = xme_hal_graph_nextIncomingEdge(&networkGraph, currentVertex);
            status = xme_hal_graph_removeEdge(&networkGraph, nextEdge);
            XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        }
        status = xme_hal_graph_finiIncomingEdgeIterator(&networkGraph, currentVertex);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    }

    for(i = 0; i < update->neighborsCount; i++)
    {
        //Add the sending node if it doesn't exist
        sendingNode = (xme_core_directory_networkTopologyCalculator_vertexData_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_vertexData_t));
        sendingNode->nodeId = update->neighbors[i].senderNodeId;
        sendingVertex = xme_hal_graph_getNextVertexWithDataComparison(&networkGraph, sendingNode, startVertex);
        if(sendingVertex == XME_HAL_GRAPH_INVALID_VERTEX_ID) //The node is not yet in the graph and needs to be added
        {
            sendingNode->lastSeenBeforeTicks = NUMBER_OF_TICKS;
            sendingVertex = xme_hal_graph_addVertex(&networkGraph, sendingNode);
            XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != sendingVertex, XME_STATUS_INTERNAL_ERROR);
        }
        edgeProperties = (xme_core_directory_networkTopologyCalculator_edgeData_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_networkTopologyCalculator_edgeData_t));
        edgeProperties->senderInterfaceId = update->neighbors[i].senderInterfaceId;
        edgeProperties->receiverInterfaceId = update->neighbors[i].receiverInterfaceId;
        nextEdge = xme_hal_graph_addEdge(&networkGraph, sendingVertex, currentVertex, edgeProperties);
        XME_CHECK(XME_HAL_GRAPH_INVALID_EDGE_ID != nextEdge, XME_STATUS_INTERNAL_ERROR);
    }

    //Remove nodes that weren't updated for a while
    status = xme_hal_graph_initVertexIterator(&networkGraph);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    while(xme_hal_graph_hasNextVertex(&networkGraph))
    {
        currentVertex = xme_hal_graph_nextVertex(&networkGraph);
        status = xme_hal_graph_getVertexData(&networkGraph, currentVertex, (void**) &vertexData);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        if(vertexData->nodeId != currentNode->nodeId)
        {
            if(--(vertexData->lastSeenBeforeTicks) <= 0)
            {
                //Remove the node from the graph
                XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(&networkGraph, currentVertex, (void**) &data), XME_STATUS_INTERNAL_ERROR);
                xme_hal_mem_free(data);
                status = xme_hal_graph_removeVertex(&networkGraph, currentVertex);
                XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
            }
        }
    }
    status = xme_hal_graph_finiVertexIterator(&networkGraph);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_hal_graph_graph_t
xme_core_directory_networkTopologyCalculator_getNetworkGraph(void)
{
    return networkGraph;
}
