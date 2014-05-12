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
 * $Id: graph_arch.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Graph abstraction (architecture specific part: generic OS based
 *         implementation).
 */

/**
 * \addtogroup hal_graph
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/graph.h"

#include "xme/core/log.h"
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_hal_graph_edge_t
 *
 * \brief  Internal representation of a directed edge in a graph.
 */
typedef struct xme_hal_graph_edge_t
{
    xme_hal_graph_edgeId_t edgeId;                   ///< Unique identifier of the this edge.
    void* edgeData;                                  ///< Data associated with this edge.
    struct xme_hal_graph_vertex_t* sourceVertex;     ///< Source vertex of this directed edge.
    struct xme_hal_graph_vertex_t* sinkVertex;       ///< Sink vertex of this directed edge.
    struct xme_hal_graph_edge_t* nextOutgoingEdge;   ///< Next outgoing edge.
    //struct xme_hal_graph_edge_t* nextIncomingEdge; ///< Next incoming edge.
} xme_hal_graph_edge_t;

/**
 * \typedef xme_hal_graph_vertex_t
 *
 * \brief  Internal representation of a vertex in a graph.
 */
typedef struct xme_hal_graph_vertex_t
{
    xme_hal_graph_vertexId_t vertexId;                 ///< Unique identifier of this vertex.
    void* vertexData;                                  ///< Data associated with this vertex.
    xme_hal_graph_edge_t* firstOutgoingEdge;           ///< First outgoing edge of this vertex.
    //xme_hal_graph_edge_t* firstIncomingEdge;         ///< First incoming edge of this vertex.
    struct xme_hal_graph_vertex_t* nextVertex;         ///< Next vertex in the data structure.
    struct xme_hal_graph_edge_t* outgoingEdgeIterator; ///< Outgoing edge iterator.
    bool outgoingEdgeIteratorInUse;                    ///< Whether the outgoing edge iterator is initialized.
    struct xme_hal_graph_edge_t* incomingEdgeIterator; ///< Incoming edge iterator.
} xme_hal_graph_vertex_t;

/**
 * \struct xme_hal_graph_graphInternal_t
 *
 * \brief  Internal representation of a graph.
 *
 * \warning Must be kept in sync with xme_hal_graph_graph_t!
 */
typedef struct xme_hal_graph_graphInternal_t
{
    xme_hal_graph_vertex_t* firstVertex;                         ///< First vertex added in the graph.
    uint16_t numVertices;                                        ///< Number of vertex in the graph.
    uint16_t numEdges;                                           ///< Number of edges in the graph.
    struct xme_hal_graph_vertex_t* vertexIterator;               ///< Vertex iterator.
    bool vertexIteratorInUse;                                    ///< Whether the vertex iterator is initialized.
    xme_hal_graph_vertexId_t lastAssignedVertex;                 ///< Last assigned vertex identifier.
    xme_hal_graph_edgeId_t lastAssignedEdge;                     ///< Last assigned edge identifier.
    xme_hal_graph_vertexCompareCallback_t vertexCompareCallback; ///< Vertex compare callback.
    xme_hal_graph_edgeCompareCallback_t edgeCompareCallback;     ///< Edge compare callback.
} xme_hal_graph_graphInternal_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
#if defined(DEBUG) && !defined(DOXYGEN)
static xme_hal_graph_graph_t _xme_hal_graph_externalGraphRepresentation;
static xme_hal_graph_graphInternal_t _xme_hal_graph_internalGraphRepresentation;
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Creates a new vertex.
 *
 * \param[in,out] graph Graph descriptor where the new vertex is to be created
 *                in.
 * \param[in,out] vertex Address of a vertex descriptor to fill with
 *                information about the newly created vertex.
 * \param[in] vertexData Data to associate with the newly created vertex.
 *
 * \return On success, returns a non-NULL pointer to the newly created vertex
 *         (i.e., *vertex). On error, returns NULL.
 */
xme_hal_graph_vertex_t*
xme_hal_graph_createVertex
(
    xme_hal_graph_graphInternal_t* graph,
    xme_hal_graph_vertex_t** vertex,
    void* vertexData
);

/**
 * \brief Removes the given vertex and all outgoing and incoming edges
 *        of that vertex from the graph.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] vertex Identifier of the vertex to remove.
 */
void
xme_hal_graph_deleteVertex
(
    xme_hal_graph_graphInternal_t* graph,
    xme_hal_graph_vertex_t *vertex
);

/**
 * \brief Creates a new edge.
 *
 * \param[in,out] graph Graph descriptor where the new edge is to be created
 *                in.
 * \param[in,out] edge Address of an edge descriptor to fill with
 *                information about the newly created edge.
 * \param[in] edgeData Data to associate with the newly created edge.
 * \param[in] sourceVertex Destination vertex descriptor of the newly
 *            created edge.
 * \param[in] sinkVertex Destination vertex descriptor of the newly
 *            created edge.
 *
 * \return On success, returns a non-NULL pointer to the newly created edge
 *         (i.e., *edge). On error, returns NULL.
 */
xme_hal_graph_edge_t*
xme_hal_graph_createEdge
(
    xme_hal_graph_graphInternal_t* graph,
    xme_hal_graph_edge_t** edge,
    void* edgeData,
    xme_hal_graph_vertex_t* sourceVertex,
    xme_hal_graph_vertex_t* sinkVertex
);

/**
 * \brief Gets the next edge identifier.
 * \note Apart from creating a new identifier, increments the corresponding
 *       variable that contains the most recently assigned edge identifier.
 *
 * \param[in,out] graph Graph descriptor.
 *
 * \return An unique edge identifier.
 */
xme_hal_graph_edgeId_t
xme_hal_graph_generateEdgeId
(
    xme_hal_graph_graphInternal_t* graph
);

/**
 * \brief Gets the next vertex identifier.
 *
 * \note  Apart from creating a new identifier, increments the corresponding
 *        variable that contains the most recently assigned vertex identifier.
 *
 * \param[in,out] graph Graph descriptor.
 *
 * \return An unique vertex identifier.
 */

xme_hal_graph_vertexId_t
xme_hal_graph_generateVertexId
(
    xme_hal_graph_graphInternal_t* graph
);

/**
 * \brief  Returns the vertex descriptor corresponding to the given vertex
 *         identifier.
 *
 * \details The provided vertex descriptor allows to explore all adjacent edges
 *          that are linked to the respective vertex.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId Vertex identifier to retrieve vertex descriptor for.
 * \param[out] outVertex Address of a vertex descriptor to fill in.
 *
 * \retval XME_CORE_STATUS_SUCCESS if a vertex with the given identifier
 *         has been found and the descriptor stored in outVertex.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NOT_FOUND if no vertex with the given identifier was found.
 */
xme_status_t
xme_hal_graph_getVertex
(
    const xme_hal_graph_graphInternal_t* graph,
    xme_hal_graph_vertexId_t vertexId,
    xme_hal_graph_vertex_t** outVertex
);

/**
 * \brief  Returns the edge descriptor corresponding to the given edge
 *         identifier.
 *
 * \details The provided edge descriptor allows to explore all edge fields and
 *          connected vertices.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] edgeId Edge identifier to retrieve edge descriptor for.
 * \param[out] outEdge Address of a edge descriptor to fill in.
 *
  * \retval XME_CORE_STATUS_SUCCESS if an edge with the given identifier
 *         has been found and the descriptor stored in outEdge.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NOT_FOUND if no edge with the given identifier was found.
 */
xme_status_t
xme_hal_graph_getEdge
(
    const xme_hal_graph_graphInternal_t* graph,
    xme_hal_graph_edgeId_t edgeId,
    xme_hal_graph_edge_t** outEdge
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_hal_graph_init
(
    xme_hal_graph_graph_t *graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);

#if defined(DEBUG) && !defined(DOXYGEN)
    // Ensure consistency between xme_hal_graph_graph_t and xme_hal_graph_graphInternal_t
    XME_ASSERT(sizeof(xme_hal_graph_graph_t) == sizeof(xme_hal_graph_graphInternal_t));
    XME_ASSERT(sizeof(_xme_hal_graph_externalGraphRepresentation) == sizeof(_xme_hal_graph_internalGraphRepresentation));
    XME_ASSERT(&((xme_hal_graph_graph_t*) 0)->internal1 == (void**) &((xme_hal_graph_graphInternal_t*) 0)->firstVertex);
    XME_ASSERT(&((xme_hal_graph_graph_t*) 0)->internal2 ==          &((xme_hal_graph_graphInternal_t*) 0)->numVertices);
    XME_ASSERT(&((xme_hal_graph_graph_t*) 0)->internal3 ==          &((xme_hal_graph_graphInternal_t*) 0)->numEdges);
    XME_ASSERT(&((xme_hal_graph_graph_t*) 0)->internal4 == (void**) &((xme_hal_graph_graphInternal_t*) 0)->vertexIterator);
    XME_ASSERT(&((xme_hal_graph_graph_t*) 0)->internal5 ==          &((xme_hal_graph_graphInternal_t*) 0)->vertexIteratorInUse);
    XME_ASSERT(&((xme_hal_graph_graph_t*) 0)->internal6 ==          &((xme_hal_graph_graphInternal_t*) 0)->lastAssignedVertex);
    XME_ASSERT(&((xme_hal_graph_graph_t*) 0)->internal7 ==          &((xme_hal_graph_graphInternal_t*) 0)->lastAssignedEdge);
    XME_ASSERT(&((xme_hal_graph_graph_t*) 0)->internal8 ==          &((xme_hal_graph_graphInternal_t*) 0)->vertexCompareCallback);
    XME_ASSERT(&((xme_hal_graph_graph_t*) 0)->internal9 ==          &((xme_hal_graph_graphInternal_t*) 0)->edgeCompareCallback);
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

    // Initialize graph fields
    graphInternal->numEdges = 0;
    graphInternal->numVertices = 0;
    graphInternal->firstVertex = NULL;
    graphInternal->vertexIterator = NULL;
    graphInternal->vertexIteratorInUse = false;
    graphInternal->lastAssignedEdge = XME_HAL_GRAPH_INVALID_EDGE_ID;
    graphInternal->lastAssignedVertex = XME_HAL_GRAPH_INVALID_VERTEX_ID;
    graphInternal->vertexCompareCallback = NULL;
    graphInternal->edgeCompareCallback = NULL;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_graph_fini
(
    xme_hal_graph_graph_t* graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);

    // Clear the graph
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_clear(graph), XME_STATUS_INTERNAL_ERROR);

    // Reset remaining members that may be non-zero
    graphInternal->vertexCompareCallback = NULL;
    graphInternal->edgeCompareCallback = NULL;

    return XME_STATUS_SUCCESS;
}

uint16_t
xme_hal_graph_getVertexCount
(
    xme_hal_graph_graph_t* graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(graphInternal != NULL, 0);

    return graphInternal->numVertices;
}

uint16_t
xme_hal_graph_getEdgeCount
(
    xme_hal_graph_graph_t *graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(graphInternal != NULL, 0);

    return graphInternal->numEdges;
}

xme_status_t
xme_hal_graph_clone
(
    xme_hal_graph_graph_t* templateGraph,
    xme_hal_graph_graph_t* outCopiedGraph
)
{
    // FIXME. Take into account the new field incomingEdges
    xme_hal_graph_graphInternal_t* templateGraphInternal = (xme_hal_graph_graphInternal_t*) templateGraph;
    xme_hal_graph_graphInternal_t* copiedGraphInternal = (xme_hal_graph_graphInternal_t*) outCopiedGraph;

    xme_hal_graph_vertex_t* currentVertex;
    xme_hal_graph_vertex_t* currentNewVertex;
    xme_hal_graph_vertex_t* prevNewVertex;

    XME_CHECK(NULL != templateGraphInternal, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != copiedGraphInternal, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(0 == copiedGraphInternal->numVertices, XME_STATUS_INVALID_CONFIGURATION);
    XME_CHECK(0 == copiedGraphInternal->numEdges, XME_STATUS_INVALID_CONFIGURATION);
    XME_ASSERT(NULL == copiedGraphInternal->firstVertex);
    XME_ASSERT(NULL == copiedGraphInternal->vertexIterator);

    // Clone vertices
    currentVertex = templateGraphInternal->firstVertex;
    prevNewVertex = NULL;
    while (currentVertex != NULL)
    {
        xme_hal_graph_vertex_t* newVertex = xme_hal_graph_createVertex(copiedGraphInternal, &newVertex, currentVertex->vertexData);

        XME_CHECK_REC
        (
            NULL != newVertex,
            XME_STATUS_OUT_OF_RESOURCES,
            (void) xme_hal_graph_fini((xme_hal_graph_graph_t*) copiedGraphInternal)
        );

        // Increment vertex count
        copiedGraphInternal->numVertices++;

        // Force identifier of new vertex to the same value
        newVertex->vertexId = currentVertex->vertexId;

        if (NULL == prevNewVertex)
        {
            copiedGraphInternal->firstVertex = newVertex;
        }
        else
        {
            prevNewVertex->nextVertex = newVertex;
        }

        prevNewVertex = newVertex;
        currentVertex = currentVertex->nextVertex;
    }

    // Clone outgoing edges
    currentVertex = templateGraphInternal->firstVertex;
    currentNewVertex = copiedGraphInternal->firstVertex;
    while (currentVertex != NULL)
    {
        xme_hal_graph_edge_t* currentEdge = currentVertex->firstOutgoingEdge;
        xme_hal_graph_edge_t* prevNewEdge = NULL;

        XME_ASSERT(NULL != currentNewVertex);

        while (NULL != currentEdge)
        {
            xme_hal_graph_vertex_t* dstVertex = NULL;
            xme_hal_graph_edge_t* newEdge;

            XME_ASSERT(NULL != currentEdge->sinkVertex);
            XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentEdge->sinkVertex->vertexId);

            XME_CHECK_REC
            (
                XME_STATUS_SUCCESS == xme_hal_graph_getVertex(copiedGraphInternal, currentEdge->sinkVertex->vertexId, &dstVertex),
                XME_STATUS_OUT_OF_RESOURCES,
                (void) xme_hal_graph_fini((xme_hal_graph_graph_t*) copiedGraphInternal)
            );

            newEdge = xme_hal_graph_createEdge(copiedGraphInternal, &newEdge, currentEdge->edgeData, currentVertex, dstVertex);

            XME_CHECK_REC
            (
                NULL != newEdge,
                XME_STATUS_OUT_OF_RESOURCES,
                (void) xme_hal_graph_fini((xme_hal_graph_graph_t*) copiedGraphInternal)
            );

            // Increment edge count
            copiedGraphInternal->numEdges++;

            // Force identifier of new edge to the same value
            newEdge->edgeId = currentEdge->edgeId;

            if (NULL == prevNewEdge)
            {
                currentNewVertex->firstOutgoingEdge = newEdge;
            }
            else
            {
                prevNewEdge->nextOutgoingEdge = newEdge;
            }

            prevNewEdge = newEdge;
            currentEdge = currentEdge->nextOutgoingEdge;
        }

        currentVertex = currentVertex->nextVertex;
        currentNewVertex = currentNewVertex->nextVertex;
    }

    // Checking of some assertions
    XME_ASSERT(templateGraphInternal->numVertices == copiedGraphInternal->numVertices);
    XME_ASSERT(templateGraphInternal->numEdges == copiedGraphInternal->numEdges);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_graph_clear
(
    xme_hal_graph_graph_t* graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* currentVertex;

    XME_CHECK(NULL != graphInternal, XME_STATUS_INVALID_PARAMETER);

    currentVertex = graphInternal->firstVertex;
    while (NULL != currentVertex)
    {
        xme_hal_graph_vertex_t* thisVertex;

        xme_hal_graph_edge_t* currentEdge = currentVertex->firstOutgoingEdge;
        while (NULL != currentEdge)
        {
            xme_hal_graph_edge_t* thisEdge = currentEdge;
            currentEdge = thisEdge->nextOutgoingEdge;
            xme_hal_mem_free(thisEdge);
        }

        thisVertex = currentVertex;
        currentVertex = thisVertex->nextVertex;
        xme_hal_mem_free(thisVertex);
    }

    // Initialize graph fields (partially)
    graphInternal->numEdges = 0;
    graphInternal->numVertices = 0;
    graphInternal->firstVertex = NULL;
    graphInternal->vertexIterator = NULL;
    graphInternal->vertexIteratorInUse = false;
    graphInternal->lastAssignedEdge = XME_HAL_GRAPH_INVALID_EDGE_ID;
    graphInternal->lastAssignedVertex = XME_HAL_GRAPH_INVALID_VERTEX_ID;

    return XME_STATUS_SUCCESS;
}

xme_hal_graph_vertexId_t
xme_hal_graph_addVertex
(
    xme_hal_graph_graph_t *graph,
    void* vertexData
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t *currentVertex;
    xme_hal_graph_vertex_t *newVertex;
    uint16_t i;

    XME_CHECK(graph != NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID);

    // Graph has no vertices. Unconditionally add a new vertex.
    if (graphInternal->numVertices == 0)
    {
        newVertex = xme_hal_graph_createVertex(graphInternal, &(graphInternal->firstVertex), vertexData);

        XME_CHECK(NULL != newVertex, XME_HAL_GRAPH_INVALID_VERTEX_ID);
        XME_CHECK(newVertex->vertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID);

        graphInternal->numVertices++;
        return newVertex->vertexId;
    }

    currentVertex = graphInternal->firstVertex;

    if (NULL != graphInternal->vertexCompareCallback)
    {
        // There is a comparison callback. Use it to avoid addition of nodes with duplicate data.
        for (i = 0; i < graphInternal->numVertices-1; i++)
        {
            if (0 == graphInternal->vertexCompareCallback(currentVertex->vertexData, vertexData))
            {
                return XME_HAL_GRAPH_INVALID_VERTEX_ID;
            }
            currentVertex = currentVertex->nextVertex;
        }

        // Handle last vertex
        if (0 == graphInternal->vertexCompareCallback(currentVertex->vertexData, vertexData))
        {
            return XME_HAL_GRAPH_INVALID_VERTEX_ID;
        }
    }
    else
    {
        for (i = 0; i < graphInternal->numVertices-1; i++)
        {
            currentVertex = currentVertex->nextVertex;
        }
    }

    XME_ASSERT_RVAL(NULL != currentVertex, XME_HAL_GRAPH_INVALID_VERTEX_ID);

    newVertex = xme_hal_graph_createVertex(graphInternal, &(newVertex), vertexData);

    XME_CHECK(newVertex != NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    XME_CHECK(newVertex->vertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID);

    if (NULL == graphInternal->vertexIterator && graphInternal->vertexIteratorInUse)
    {
        graphInternal->vertexIterator = newVertex;
    }

    // currentVertex is the last vertex of the graph
    currentVertex->nextVertex = newVertex;

    graphInternal->numVertices++;
    return newVertex->vertexId;
}

xme_hal_graph_vertexId_t
xme_hal_graph_getNextVertexWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    void* vertexData,
    xme_hal_graph_vertexId_t startAfterVertex
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* currentVertex;

    XME_CHECK(graphInternal != NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID);

    currentVertex = graphInternal->firstVertex;
    while (XME_HAL_GRAPH_INVALID_VERTEX_ID != startAfterVertex && NULL != currentVertex)
    {
        XME_ASSERT_RVAL(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentVertex->vertexId, XME_HAL_GRAPH_INVALID_VERTEX_ID);

        if (currentVertex->vertexId == startAfterVertex)
        {
            currentVertex = currentVertex->nextVertex;
            break;
        }
        currentVertex = currentVertex->nextVertex;
    }

    // Return invalid vertex identifier if either:
    // - The vertex with identifier startAfterVertex is not part of the graph
    // - The graph is empty
    XME_CHECK(NULL != currentVertex, XME_HAL_GRAPH_INVALID_VERTEX_ID);

    while (NULL != currentVertex)
    {
        if (NULL != graphInternal->vertexCompareCallback)
        {
            if (0 == graphInternal->vertexCompareCallback(currentVertex->vertexData, vertexData))
            {
                // Found a match
                return currentVertex->vertexId;
            }
        }
        else
        {
            if (currentVertex->vertexData == vertexData)
            {
                // Found a match
                return currentVertex->vertexId;
            }
        }

        currentVertex = currentVertex->nextVertex;
    }

    // No match found
    return XME_HAL_GRAPH_INVALID_VERTEX_ID;
}

xme_status_t
xme_hal_graph_removeVertex
(
    xme_hal_graph_graph_t *graph,
    xme_hal_graph_vertexId_t vertexId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t *currentVertex;
    xme_hal_graph_vertex_t *prevVertex;
    uint16_t i;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(graphInternal->numVertices != 0, XME_STATUS_NOT_FOUND);

    currentVertex = graphInternal->firstVertex;
    prevVertex = NULL;

    for(i = 0; i < graphInternal->numVertices; i++)
    {
        if (currentVertex->vertexId == vertexId)
        {
            if (prevVertex == NULL)
            {
                // The vertex to remove is the first vertex of the graph
                graphInternal->firstVertex = currentVertex->nextVertex;
            }
            else
            {
                // The vertex to remove is *not* the first vertex of the graph
                prevVertex->nextVertex = currentVertex->nextVertex;
            }

            graphInternal->numVertices--;
            xme_hal_graph_deleteVertex(graphInternal, currentVertex);

            return XME_STATUS_SUCCESS;
        }

        prevVertex = currentVertex;
        currentVertex = currentVertex->nextVertex;
    }

    return XME_STATUS_NOT_FOUND;
}

xme_status_t
xme_hal_graph_removeVertexWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    void* vertexData,
    bool all
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t *currentVertex;
    xme_hal_graph_vertex_t *prevVertex;
    uint16_t i;
    xme_status_t status = XME_STATUS_NOT_FOUND;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(graphInternal->numVertices != 0, XME_STATUS_NOT_FOUND);

    currentVertex = graphInternal->firstVertex;
    prevVertex = NULL;

    for(i = 0; i < graphInternal->numVertices; i++)
    {
        if ((NULL == graphInternal->vertexCompareCallback) ?
            (currentVertex->vertexData == vertexData) :
            (0 == graphInternal->vertexCompareCallback(currentVertex->vertexData, vertexData)))
        {
            if (prevVertex == NULL)
            {
                // The vertex to remove is the first vertex of the graph
                graphInternal->firstVertex = currentVertex->nextVertex;
            }
            else
            {
                // The vertex to remove is *not* the first vertex of the graph
                prevVertex->nextVertex = currentVertex->nextVertex;
            }

            graphInternal->numVertices--;
            xme_hal_graph_deleteVertex(graphInternal, currentVertex);

            status = XME_STATUS_SUCCESS;

            if (!all)
            {
                return status;
            }
        }

        prevVertex = currentVertex;
        currentVertex = currentVertex->nextVertex;
    }

    return status;
}

xme_hal_graph_edgeId_t
xme_hal_graph_addEdge
(
    xme_hal_graph_graph_t *graph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    void* edgeData
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t *currentVertex;
    xme_hal_graph_vertex_t *sourceVertex = NULL;
    xme_hal_graph_vertex_t *sinkVertex = NULL;
    uint16_t i;

    XME_CHECK(graphInternal != NULL, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_CHECK(graphInternal->numVertices != 0, XME_HAL_GRAPH_INVALID_EDGE_ID); // no vertices, no connections

    currentVertex = graphInternal->firstVertex;

    for (i = 0; i < graphInternal->numVertices; i++)
    {
        if (currentVertex->vertexId == sinkVertexId)
        {
            sinkVertex = currentVertex;
        }

        if (currentVertex->vertexId == sourceVertexId)
        {
            sourceVertex = currentVertex;
        }

        // We can insert the edge as soon as we know the source and destination vertex
        if (sourceVertex != NULL && sinkVertex != NULL)
        {
            xme_hal_graph_edge_t* edge;

            // First check whether the edge is already existing, in which case we return
            edge = sourceVertex->firstOutgoingEdge;
            while (NULL != edge)
            {
                if (edge->sinkVertex == sinkVertex)
                {
                    // This edge already exists. If an edge comparison callback is registered,
                    // we need to ensure that the edges are unequal before we insert the new edge.
                    if (NULL != graphInternal->edgeCompareCallback)
                    {
                        if (0 == graphInternal->edgeCompareCallback(edge->edgeData, edgeData))
                        {
                            return XME_HAL_GRAPH_INVALID_EDGE_ID;
                        }
                    }
                }

                edge = edge->nextOutgoingEdge;
            }

            // Add new edge
            edge = xme_hal_graph_createEdge(graphInternal, &(edge), edgeData, sourceVertex, sinkVertex);

            XME_CHECK(NULL != edge, XME_HAL_GRAPH_INVALID_EDGE_ID);

            // Set the corresponding outgoing edges for source vertex.
            if (sourceVertex->firstOutgoingEdge == NULL)
            {
                // Add the first edge originating from this node
                sourceVertex->firstOutgoingEdge = edge;
            }
            else
            {
                // Add to the end of the edge list
                xme_hal_graph_edge_t* currentEdge = sourceVertex->firstOutgoingEdge;
                while (currentEdge->nextOutgoingEdge != NULL)
                {
                    currentEdge = currentEdge->nextOutgoingEdge;
                }

                currentEdge->nextOutgoingEdge = edge;
            }

            // If the new edge can be part of the current vertex, add it to the list
            if (NULL == sourceVertex->outgoingEdgeIterator && sourceVertex->outgoingEdgeIteratorInUse)
            {
                sourceVertex->outgoingEdgeIterator = edge;
            }

            // Increment the number of edges in the graph
            graphInternal->numEdges++;

            return edge->edgeId;
        }

        currentVertex = currentVertex->nextVertex;
    }

    return XME_HAL_GRAPH_INVALID_EDGE_ID;
}

xme_hal_graph_edgeId_t
xme_hal_graph_getNextEdgeWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    void* edgeData,
    xme_hal_graph_edgeId_t startAfterEdge
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* currentVertex;
    bool startAfterEdgeFound;
    bool stopOnNextEdge = false;

    XME_CHECK(graphInternal != NULL, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_CHECK(graphInternal->numEdges != 0, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_ASSERT_RVAL(graphInternal->numVertices != 0, XME_HAL_GRAPH_INVALID_EDGE_ID);

    startAfterEdgeFound = (XME_HAL_GRAPH_INVALID_EDGE_ID == startAfterEdge);

    // Iterate over all vertices
    currentVertex = graphInternal->firstVertex;
    while (NULL != currentVertex)
    {
        xme_hal_graph_edge_t* currentEdge;

        currentEdge = currentVertex->firstOutgoingEdge;
        if (!startAfterEdgeFound)
        {
            while (NULL != currentEdge)
            {
                XME_ASSERT_RVAL(XME_HAL_GRAPH_INVALID_EDGE_ID != currentEdge->edgeId, XME_HAL_GRAPH_INVALID_EDGE_ID);

                if (stopOnNextEdge)
                {
                    break;
                }

                if (currentEdge->edgeId == startAfterEdge)
                {
                    currentEdge = currentEdge->nextOutgoingEdge;
                    if (NULL == currentEdge)
                    {
                        stopOnNextEdge = true;
                    }
                    break;
                }
                currentEdge = currentEdge->nextOutgoingEdge;
            }

            if (NULL != currentEdge)
            {
                // Found the "start after" edge
                startAfterEdgeFound = true;
            }
        }

        // Find the edge
        while (NULL != currentEdge)
        {
            XME_ASSERT_RVAL(XME_HAL_GRAPH_INVALID_EDGE_ID != currentEdge->edgeId, XME_HAL_GRAPH_INVALID_EDGE_ID);
            XME_ASSERT_RVAL(NULL != currentEdge->sinkVertex, XME_HAL_GRAPH_INVALID_EDGE_ID);
            XME_ASSERT_RVAL(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentEdge->sinkVertex->vertexId, XME_HAL_GRAPH_INVALID_EDGE_ID);

            if (NULL != graphInternal->edgeCompareCallback)
            {
                if (0 == graphInternal->edgeCompareCallback(currentEdge->edgeData, edgeData))
                {
                    // Found a match
                    return currentEdge->edgeId;
                }
            }
            else
            {
                if (currentEdge->edgeData == edgeData)
                {
                    // Found a match
                    return currentEdge->edgeId;
                }
            }

            currentEdge = currentEdge->nextOutgoingEdge;
        }

        currentVertex = currentVertex->nextVertex;
    }

    // No match found
    return XME_HAL_GRAPH_INVALID_EDGE_ID;
}

xme_hal_graph_edgeId_t
xme_hal_graph_getNextEdgeBetweenWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    void* edgeData,
    xme_hal_graph_edgeId_t startAfterEdge
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* currentVertex;

    XME_CHECK(graphInternal != NULL, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_CHECK(graphInternal->numEdges != 0, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_ASSERT_RVAL(graphInternal->numVertices != 0, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != sourceVertexId, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != sinkVertexId, XME_HAL_GRAPH_INVALID_EDGE_ID);

    // Find the vertex
    currentVertex = graphInternal->firstVertex;
    while (NULL != currentVertex)
    {
        XME_ASSERT_RVAL(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentVertex->vertexId, XME_HAL_GRAPH_INVALID_EDGE_ID);

        if (currentVertex->vertexId == sourceVertexId)
        {
            // Found the vertex
            xme_hal_graph_edge_t* currentEdge = currentVertex->firstOutgoingEdge;
            while (XME_HAL_GRAPH_INVALID_EDGE_ID != startAfterEdge && NULL != currentEdge)
            {
                XME_ASSERT_RVAL(XME_HAL_GRAPH_INVALID_EDGE_ID != currentEdge->edgeId, XME_HAL_GRAPH_INVALID_EDGE_ID);

                if (currentEdge->edgeId == startAfterEdge)
                {
                    XME_ASSERT_RVAL(NULL != currentEdge->sinkVertex, XME_HAL_GRAPH_INVALID_EDGE_ID);

                    if (currentEdge->sinkVertex->vertexId == sinkVertexId)
                    {
                        currentEdge = currentEdge->nextOutgoingEdge;
                        break;
                    }
                }
                currentEdge = currentEdge->nextOutgoingEdge;
            }

            // Return invalid edge identifier if either:
            // - The edge with identifier startAfterEdge is not part of the graph
            // - The source vertex does not have any outgoing edges
            XME_CHECK(NULL != currentEdge, XME_HAL_GRAPH_INVALID_EDGE_ID);

            // Find the edge
            while (NULL != currentEdge)
            {
                XME_ASSERT_RVAL(XME_HAL_GRAPH_INVALID_EDGE_ID != currentEdge->edgeId, XME_HAL_GRAPH_INVALID_EDGE_ID);
                XME_ASSERT_RVAL(NULL != currentEdge->sinkVertex, XME_HAL_GRAPH_INVALID_EDGE_ID);
                XME_ASSERT_RVAL(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentEdge->sinkVertex->vertexId, XME_HAL_GRAPH_INVALID_EDGE_ID);

                if (currentEdge->sinkVertex->vertexId == sinkVertexId)
                {
                    // Found an edge between source and sink vertex
                    if (NULL != graphInternal->edgeCompareCallback)
                    {
                        if (0 == graphInternal->edgeCompareCallback(currentEdge->edgeData, edgeData))
                        {
                            // Found a match
                            return currentEdge->edgeId;
                        }
                    }
                    else
                    {
                        if (currentEdge->edgeData == edgeData)
                        {
                            // Found a match
                            return currentEdge->edgeId;
                        }
                    }
                }

                currentEdge = currentEdge->nextOutgoingEdge;
            }

            // We can break here, because no other vertex in the graph can have
            // the same source vertex identifier than the one that we just processed
            break;
        }

        currentVertex = currentVertex->nextVertex;
    }

    // No match found
    return XME_HAL_GRAPH_INVALID_EDGE_ID;
}

xme_status_t
xme_hal_graph_removeEdge
(
    xme_hal_graph_graph_t *graph,
    xme_hal_graph_edgeId_t edgeId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* currentVertex;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_HAL_GRAPH_INVALID_EDGE_ID != edgeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(graphInternal->numEdges != 0, XME_STATUS_NOT_FOUND);
    XME_ASSERT(graphInternal->numVertices != 0);

    // Find the edge in outgoing edges
    currentVertex = graphInternal->firstVertex;
    while (NULL != currentVertex)
    {
        xme_hal_graph_edge_t* prevEdge = NULL;
        xme_hal_graph_edge_t* currentEdge = currentVertex->firstOutgoingEdge;

        XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentVertex->vertexId);

        while (NULL != currentEdge)
        {
            XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != currentEdge->edgeId);

            if (currentEdge->edgeId == edgeId)
            {
                // Found the edge to remove
                if (NULL == prevEdge)
                {
                    // The edge to remove is the first edge of the vertex
                    currentVertex->firstOutgoingEdge = currentEdge->nextOutgoingEdge;
                }
                else
                {
                    // The edge to remove is *not* the first edge of the vertex
                    prevEdge->nextOutgoingEdge = currentEdge->nextOutgoingEdge;
                }

                // It is just now, after removing both incoming and outgoing edges, when
                // we can remove the edge. 
                graphInternal->numEdges--;
                xme_hal_mem_free(currentEdge);

                return XME_STATUS_SUCCESS;
            }
            prevEdge = currentEdge;
            currentEdge = currentEdge->nextOutgoingEdge;
        }
        currentVertex = currentVertex->nextVertex;
    }

    return XME_STATUS_NOT_FOUND;
}

xme_status_t
xme_hal_graph_removeEdgeWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    void* edgeData,
    bool all
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* currentVertex;
    xme_status_t status = XME_STATUS_NOT_FOUND;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(graphInternal->numEdges != 0, XME_STATUS_NOT_FOUND);
    XME_ASSERT(graphInternal->numVertices != 0);

    // Iterate over all vertices
    currentVertex = graphInternal->firstVertex;
    while (NULL != currentVertex)
    {
        xme_hal_graph_edge_t* prevEdge = NULL;
        xme_hal_graph_edge_t* currentEdge = currentVertex->firstOutgoingEdge;

        XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentVertex->vertexId);

        while (NULL != currentEdge)
        {
            XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != currentEdge->edgeId);

            if ((NULL == graphInternal->edgeCompareCallback) ?
                (currentEdge->edgeData == edgeData) :
                (0 == graphInternal->edgeCompareCallback(currentEdge->edgeData, edgeData)))
            {
                // Found an edge to remove
                xme_hal_graph_edge_t* nextEdge;

                if (NULL == prevEdge)
                {
                    // The edge to remove is the first edge of the vertex
                    currentVertex->firstOutgoingEdge = currentEdge->nextOutgoingEdge;
                }
                else
                {
                    // The edge to remove is *not* the first edge of the vertex
                    prevEdge->nextOutgoingEdge = currentEdge->nextOutgoingEdge;
                }

                nextEdge = currentEdge->nextOutgoingEdge;

                graphInternal->numEdges--;
                xme_hal_mem_free(currentEdge);

                status = XME_STATUS_SUCCESS;

                if (!all)
                {
                    return status;
                }

                currentEdge = nextEdge;
                continue;
            }
            prevEdge = currentEdge;
            currentEdge = currentEdge->nextOutgoingEdge;
        }
        currentVertex = currentVertex->nextVertex;
    }

    return status;
}

xme_status_t
xme_hal_graph_removeEdgeBetween
(
    xme_hal_graph_graph_t *graph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    bool all
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* currentVertex;
    xme_status_t status = XME_STATUS_NOT_FOUND;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(graphInternal->numEdges != 0, XME_STATUS_NOT_FOUND);
    XME_ASSERT(graphInternal->numVertices != 0);
    XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != sourceVertexId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_HAL_GRAPH_INVALID_VERTEX_ID != sinkVertexId, XME_STATUS_INVALID_PARAMETER);

    // Find the vertex
    currentVertex = graphInternal->firstVertex;
    while (NULL != currentVertex)
    {
        XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentVertex->vertexId);

        if (currentVertex->vertexId == sourceVertexId)
        {
            // Found the vertex
            xme_hal_graph_edge_t* prevEdge = NULL;
            xme_hal_graph_edge_t* currentEdge = currentVertex->firstOutgoingEdge;

            // Find the edge
            while (NULL != currentEdge)
            {
                XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != currentEdge->edgeId);
                XME_ASSERT(NULL != currentEdge->sinkVertex);
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentEdge->sinkVertex->vertexId);

                if (currentEdge->sinkVertex->vertexId == sinkVertexId)
                {
                    xme_hal_graph_edge_t* nextEdge;

                    // Found the edge to remove
                    if (NULL == prevEdge)
                    {
                        // The edge to remove is the first edge of the vertex
                        currentVertex->firstOutgoingEdge = currentEdge->nextOutgoingEdge;
                    }
                    else
                    {
                        // The edge to remove is *not* the first edge of the vertex
                        prevEdge->nextOutgoingEdge = currentEdge->nextOutgoingEdge;
                    }

                    nextEdge = currentEdge->nextOutgoingEdge;

                    graphInternal->numEdges--;
                    xme_hal_mem_free(currentEdge);

                    status = XME_STATUS_SUCCESS;

                    if (!all)
                    {
                        return status;
                    }

                    currentEdge = nextEdge;
                    continue;
                }

                prevEdge = currentEdge;
                currentEdge = currentEdge->nextOutgoingEdge;
            }
        }

        currentVertex = currentVertex->nextVertex;
    }

    return status;
}

xme_status_t
xme_hal_graph_removeEdgeBetweenWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    void* edgeData,
    bool all
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_status_t status = XME_STATUS_NO_SUCH_VALUE;
    xme_hal_graph_vertex_t* currentVertex;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(graphInternal->numEdges != 0, XME_STATUS_NO_SUCH_VALUE);
    XME_ASSERT(graphInternal->numVertices != 0);

    // Find the source vertex
    currentVertex = graphInternal->firstVertex;
    while (NULL != currentVertex)
    {
        XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentVertex->vertexId);

        if (sourceVertexId == currentVertex->vertexId)
        {
            // Found the source vertex
            xme_hal_graph_edge_t* prevEdge = NULL;
            xme_hal_graph_edge_t* currentEdge = currentVertex->firstOutgoingEdge;

            // Find the edge
            while (NULL != currentEdge)
            {
                XME_ASSERT(XME_HAL_GRAPH_INVALID_EDGE_ID != currentEdge->edgeId);
                XME_ASSERT(NULL != currentEdge->sinkVertex);
                XME_ASSERT(XME_HAL_GRAPH_INVALID_VERTEX_ID != currentEdge->sinkVertex->vertexId);

                if (sinkVertexId == currentEdge->sinkVertex->vertexId)
                {
                    // Found a matching sink vertex and hence an edge candidate
                    if (XME_STATUS_NO_SUCH_VALUE == status)
                    {
                        // Update status: From now on, we are going to return
                        // XME_STATUS_NOT_FOUND if no match is found
                        status = XME_STATUS_NOT_FOUND;
                    }

                    if ((NULL == graphInternal->edgeCompareCallback) ?
                        (currentEdge->edgeData == edgeData) :
                        (0 == graphInternal->edgeCompareCallback(currentEdge->edgeData, edgeData)))
                    {
                        xme_hal_graph_edge_t* nextEdge;

                        // Found an edge to remove
                        if (NULL == prevEdge)
                        {
                            // The edge to remove is the first edge of the vertex
                            currentVertex->firstOutgoingEdge = currentEdge->nextOutgoingEdge;
                        }
                        else
                        {
                            // The edge to remove is *not* the first edge of the vertex
                            prevEdge->nextOutgoingEdge = currentEdge->nextOutgoingEdge;
                        }

                        nextEdge = currentEdge->nextOutgoingEdge;

                        graphInternal->numEdges--;
                        xme_hal_mem_free(currentEdge);

                        status = XME_STATUS_SUCCESS;

                        if (!all)
                        {
                            return status;
                        }

                        currentEdge = nextEdge;
                        continue;
                    }
                }

                prevEdge = currentEdge;
                currentEdge = currentEdge->nextOutgoingEdge;
            }

            return status;
        }

        currentVertex = currentVertex->nextVertex;
    }

    return status;
}

xme_status_t
xme_hal_graph_initVertexIterator
(
    xme_hal_graph_graph_t* graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(!graphInternal->vertexIteratorInUse, XME_STATUS_UNSUPPORTED);

    graphInternal->vertexIterator = graphInternal->firstVertex;
    graphInternal->vertexIteratorInUse = true;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_graph_finiVertexIterator
(
    xme_hal_graph_graph_t* graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(graphInternal->vertexIteratorInUse, XME_STATUS_INVALID_CONFIGURATION);

    graphInternal->vertexIterator = NULL;
    graphInternal->vertexIteratorInUse = false;

    return XME_STATUS_SUCCESS;
}

bool
xme_hal_graph_hasNextVertex
(
    xme_hal_graph_graph_t* graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(graphInternal != NULL, false);

    XME_CHECK(graphInternal->vertexIteratorInUse, false);

    return (NULL != graphInternal->vertexIterator);
}

xme_hal_graph_vertexId_t
xme_hal_graph_nextVertex 
(
    xme_hal_graph_graph_t* graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertexId_t vertexId;

    XME_CHECK(graphInternal != NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    XME_CHECK(graphInternal->vertexIterator != NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID);

    vertexId = graphInternal->vertexIterator->vertexId;
    graphInternal->vertexIterator = graphInternal->vertexIterator->nextVertex;

    return vertexId;
}

xme_status_t
xme_hal_graph_initOutgoingEdgeIterator
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* vertex;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(vertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_STATUS_INVALID_PARAMETER);
    
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getVertex(graphInternal, vertexId, &vertex),
        XME_STATUS_INVALID_PARAMETER
    );

    XME_ASSERT(NULL != vertex);

    XME_CHECK(!vertex->outgoingEdgeIteratorInUse, XME_STATUS_UNSUPPORTED);

    vertex->outgoingEdgeIterator = vertex->firstOutgoingEdge;
    vertex->outgoingEdgeIteratorInUse = true;

    return XME_STATUS_SUCCESS;
}

bool
xme_hal_graph_hasNextOutgoingEdge
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* vertex;

    XME_CHECK(graphInternal != NULL, false);

    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getVertex(graphInternal, vertexId, &vertex),
        false
    );

    return (NULL != vertex->outgoingEdgeIterator);
}

xme_hal_graph_edgeId_t
xme_hal_graph_nextOutgoingEdge
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_edgeId_t edgeId;
    xme_hal_graph_vertex_t* vertex;

    XME_CHECK(graphInternal != NULL, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getVertex(graphInternal, vertexId, &vertex),
        XME_HAL_GRAPH_INVALID_EDGE_ID
    );

    XME_ASSERT_RVAL(NULL != vertex, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_CHECK(NULL != vertex->outgoingEdgeIterator, XME_HAL_GRAPH_INVALID_EDGE_ID);

    edgeId = vertex->outgoingEdgeIterator->edgeId;
    vertex->outgoingEdgeIterator = vertex->outgoingEdgeIterator->nextOutgoingEdge;

    return edgeId;
}

xme_status_t
xme_hal_graph_finiOutgoingEdgeIterator
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* vertex;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(vertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getVertex(graphInternal, vertexId, &vertex),
        XME_STATUS_INVALID_PARAMETER
    );

    XME_ASSERT(NULL != vertex);
    XME_CHECK(vertex->outgoingEdgeIteratorInUse, XME_STATUS_INVALID_CONFIGURATION);

    vertex->outgoingEdgeIterator = NULL;
    vertex->outgoingEdgeIteratorInUse = false;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_graph_initIncomingEdgeIterator
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* currentVertex;
    xme_hal_graph_vertex_t* vertex;
    xme_hal_graph_edge_t* currentEdge;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(vertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getVertex(graphInternal, vertexId, &vertex),
        XME_STATUS_INVALID_PARAMETER
    );

    currentVertex = graphInternal->firstVertex;
    while (NULL != currentVertex)
    {
        currentEdge = currentVertex->firstOutgoingEdge;
        while(NULL != currentEdge) {
            if (currentEdge->sinkVertex->vertexId == vertexId)
            {
                vertex->incomingEdgeIterator = currentEdge;
                return XME_STATUS_SUCCESS;
            }

            currentEdge = currentEdge->nextOutgoingEdge;
        }

        currentVertex = currentVertex->nextVertex;
    }

    return XME_STATUS_SUCCESS;
}

bool
xme_hal_graph_hasNextIncomingEdge
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* vertex;

    XME_CHECK(graphInternal != NULL, false);

    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getVertex(graphInternal, vertexId, &vertex),
        false
    );

    return (NULL != vertex->incomingEdgeIterator);
}

xme_hal_graph_edgeId_t
xme_hal_graph_nextIncomingEdge
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_edgeId_t edgeId;
    xme_hal_graph_vertex_t* vertex;
    xme_hal_graph_vertex_t* currentVertex;
    xme_hal_graph_edge_t* currentEdge;
    bool next;
    bool iteratorEstablished;

    XME_CHECK(graphInternal != NULL, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getVertex(graphInternal, vertexId, &vertex),
        XME_HAL_GRAPH_INVALID_EDGE_ID
    );

    XME_ASSERT_RVAL(NULL != vertex, XME_HAL_GRAPH_INVALID_EDGE_ID);
    XME_CHECK(NULL != vertex->incomingEdgeIterator, XME_HAL_GRAPH_INVALID_EDGE_ID);

    edgeId = vertex->incomingEdgeIterator->edgeId;

    next = false;
    iteratorEstablished = false;
    currentVertex = graphInternal->firstVertex;
    while (NULL != currentVertex) 
    {
        currentEdge = currentVertex->firstOutgoingEdge;
        while(NULL != currentEdge) {
            if (currentEdge->edgeId == edgeId)
            {
                next = true;
                // establish the edge iterator to null. 
                vertex->incomingEdgeIterator = NULL;
            }
            else
            {
                if (next && 
                    currentEdge->sinkVertex->vertexId == vertexId
                    && !iteratorEstablished)
                {
                    vertex->incomingEdgeIterator = currentEdge;
                    iteratorEstablished = true;
                }
            }

            currentEdge = currentEdge->nextOutgoingEdge;
        }

        currentVertex = currentVertex->nextVertex;
    }

    return edgeId;
}

xme_status_t
xme_hal_graph_finiIncomingEdgeIterator
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* vertex;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(vertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getVertex(graphInternal, vertexId, &vertex),
        XME_STATUS_INVALID_PARAMETER
    );

    XME_ASSERT(NULL != vertex);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_graph_getVertexData
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId,
    void** outVertexData
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_vertex_t* vertex;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getVertex(graphInternal, vertexId, &vertex),
        XME_STATUS_NOT_FOUND
    );

    if (NULL != outVertexData)
    {
        *outVertexData = vertex->vertexData;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_graph_getEdgeData
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_edgeId_t edgeId,
    void** outEdgeData
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_edge_t* edge;

    XME_CHECK(graphInternal != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getEdge(graphInternal, edgeId, &edge),
        XME_STATUS_NOT_FOUND
    );

    if (NULL != outEdgeData)
    {
        *outEdgeData = edge->edgeData;
    }

    return XME_STATUS_SUCCESS;
}

xme_hal_graph_vertexId_t
xme_hal_graph_getSourceVertex
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_edgeId_t edgeId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_edge_t* edge;

    XME_CHECK(graphInternal != NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getEdge(graphInternal, edgeId, &edge),
        XME_HAL_GRAPH_INVALID_VERTEX_ID
    );

    XME_CHECK(NULL != edge->sourceVertex, XME_HAL_GRAPH_INVALID_VERTEX_ID);

    return edge->sourceVertex->vertexId;
}

xme_hal_graph_vertexId_t
xme_hal_graph_getSinkVertex
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_edgeId_t edgeId
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;
    xme_hal_graph_edge_t* edge;

    XME_CHECK(graphInternal != NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_graph_getEdge(graphInternal, edgeId, &edge),
        XME_HAL_GRAPH_INVALID_VERTEX_ID
    );

    XME_CHECK(NULL != edge->sinkVertex, XME_HAL_GRAPH_INVALID_VERTEX_ID); 

    return edge->sinkVertex->vertexId;
}

xme_hal_graph_vertexCompareCallback_t
xme_hal_graph_getVertexCompareCallback
(
    xme_hal_graph_graph_t* graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(NULL != graphInternal, NULL);

    return graphInternal->vertexCompareCallback;
}

xme_status_t
xme_hal_graph_setVertexCompareCallback
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexCompareCallback_t vertexCompareCallback
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(NULL != graphInternal, XME_STATUS_INVALID_PARAMETER);

    graphInternal->vertexCompareCallback = vertexCompareCallback;

    return XME_STATUS_SUCCESS;
}

xme_hal_graph_edgeCompareCallback_t
xme_hal_graph_getEdgeCompareCallback
(
    xme_hal_graph_graph_t* graph
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(NULL != graphInternal, NULL);

    return graphInternal->edgeCompareCallback;
}

xme_status_t
xme_hal_graph_setEdgeCompareCallback
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_edgeCompareCallback_t edgeCompareCallback
)
{
    xme_hal_graph_graphInternal_t* graphInternal = (xme_hal_graph_graphInternal_t*) graph;

    XME_CHECK(NULL != graphInternal, XME_STATUS_INVALID_PARAMETER);

    graphInternal->edgeCompareCallback = edgeCompareCallback;

    return XME_STATUS_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////

xme_hal_graph_vertex_t*
xme_hal_graph_createVertex
(
    xme_hal_graph_graphInternal_t* graph,
    xme_hal_graph_vertex_t **vertex,
    void* vertexData
)
{
    XME_ASSERT_RVAL(NULL != vertex, NULL);

    *vertex = (xme_hal_graph_vertex_t*) xme_hal_mem_alloc(sizeof(xme_hal_graph_vertex_t));
    XME_CHECK(NULL != (*vertex), NULL);

    (*vertex)->vertexId = xme_hal_graph_generateVertexId(graph); 
    (*vertex)->vertexData = vertexData;
    (*vertex)->firstOutgoingEdge = NULL;
    (*vertex)->nextVertex = NULL;
    (*vertex)->outgoingEdgeIterator = NULL;
    (*vertex)->outgoingEdgeIteratorInUse = false;
    (*vertex)->incomingEdgeIterator = NULL;

    return *vertex;
}

void
xme_hal_graph_deleteVertex
(
    xme_hal_graph_graphInternal_t* graph,
    xme_hal_graph_vertex_t* vertex
)
{
    xme_hal_graph_edge_t* currentEdge;
    xme_hal_graph_vertex_t* currentVertex;

    XME_ASSERT_NORVAL(NULL != graph);
    XME_ASSERT_NORVAL(NULL != vertex);

    // First remove all outgoing edges of the vertex
    currentEdge = vertex->firstOutgoingEdge;
    while (NULL != currentEdge)
    {
        xme_hal_graph_edge_t* thisEdge = currentEdge;
        currentEdge = currentEdge->nextOutgoingEdge;
        graph->numEdges--;
        xme_hal_mem_free(thisEdge);
    }
    vertex->firstOutgoingEdge = NULL;

    // TODO: Removed after changing the iterator.
    // Now remove all ingoing edges of the vertex
    currentVertex = graph->firstVertex;
    while (NULL != currentVertex)
    {
        xme_hal_graph_edge_t* prevEdge = NULL;
        currentEdge = currentVertex->firstOutgoingEdge;

        while (NULL != currentEdge)
        {
            if (currentEdge->sinkVertex == vertex)
            {
                if (NULL == prevEdge)
                {
                    // The edge to remove is the first edge of the vertex
                    currentVertex->firstOutgoingEdge = currentEdge->nextOutgoingEdge;
                    xme_hal_mem_free(currentEdge);
                    currentEdge = currentVertex->firstOutgoingEdge;
                    graph->numEdges--;
                }
                else
                {
                    // The edge to remove is *not* the first edge of the vertex
                    prevEdge->nextOutgoingEdge = currentEdge->nextOutgoingEdge;
                    xme_hal_mem_free(currentEdge);
                    currentEdge = prevEdge->nextOutgoingEdge;
                    graph->numEdges--;
                }
            }
            else
            {
                prevEdge = currentEdge;
                currentEdge = currentEdge->nextOutgoingEdge;
            }
        }

        currentVertex = currentVertex->nextVertex;
    }

    // Adapt vertex iterator in case it points to the vertex being removed
    if (graph->vertexIterator == vertex)
    {
        graph->vertexIterator = vertex->nextVertex;
    }

    // Free the vertex itself
    xme_hal_mem_free(vertex);
}

xme_hal_graph_edge_t*
xme_hal_graph_createEdge
(
    xme_hal_graph_graphInternal_t* graph,
    xme_hal_graph_edge_t **edge,
    void* edgeData,
    xme_hal_graph_vertex_t* sourceVertex,
    xme_hal_graph_vertex_t* sinkVertex
)
{
    XME_ASSERT_RVAL(NULL != graph, NULL);
    XME_ASSERT_RVAL(NULL != edge, NULL);
    XME_ASSERT_RVAL(NULL != sourceVertex, NULL);
    XME_ASSERT_RVAL(NULL != sinkVertex, NULL);

    *edge = (xme_hal_graph_edge_t*) xme_hal_mem_alloc(sizeof(xme_hal_graph_edge_t));
    XME_CHECK(NULL != (*edge), NULL);

    (*edge)->edgeId = xme_hal_graph_generateEdgeId(graph); 
    (*edge)->edgeData = edgeData;
    (*edge)->sourceVertex = sourceVertex;
    (*edge)->sinkVertex = sinkVertex;
    (*edge)->nextOutgoingEdge = NULL;
    //(*edge)->nextIncomingEdge = NULL;

    return *edge;
}

xme_hal_graph_edgeId_t
xme_hal_graph_generateEdgeId
(
    xme_hal_graph_graphInternal_t* graph
)
{
    graph->lastAssignedEdge = (xme_hal_graph_edgeId_t)(((int)graph->lastAssignedEdge) + 1);
    return graph->lastAssignedEdge;
}

xme_hal_graph_vertexId_t
xme_hal_graph_generateVertexId
(
    xme_hal_graph_graphInternal_t* graph
)
{
    graph->lastAssignedVertex = (xme_hal_graph_vertexId_t)(((int)graph->lastAssignedVertex) + 1);
    return graph->lastAssignedVertex;
}

xme_status_t
xme_hal_graph_getVertex
(
    const xme_hal_graph_graphInternal_t *graph,
    xme_hal_graph_vertexId_t vertexId,
    xme_hal_graph_vertex_t** outVertex
)
{
    xme_hal_graph_vertex_t *currentVertex;
    uint16_t i;

    XME_ASSERT(NULL != outVertex);

    XME_CHECK(graph != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(graph->numVertices != 0, XME_STATUS_NOT_FOUND);

    currentVertex = graph->firstVertex;

    for (i = 0; i < graph->numVertices; i++) 
    {
        if (currentVertex->vertexId == vertexId) 
        {
            *outVertex = currentVertex;
            return XME_STATUS_SUCCESS;
        }

        currentVertex = currentVertex->nextVertex;
    }

    return XME_STATUS_NOT_FOUND;
}

xme_status_t
xme_hal_graph_getEdge
(
    const xme_hal_graph_graphInternal_t* graph,
    xme_hal_graph_edgeId_t edgeId,
    xme_hal_graph_edge_t** outEdge
)
{
    xme_hal_graph_vertex_t *currentVertex;
    xme_hal_graph_edge_t *currentEdge;

    XME_ASSERT(NULL != outEdge);
    XME_CHECK(graph != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(graph->numVertices != 0, XME_STATUS_NOT_FOUND);

    currentVertex = graph->firstVertex;
    while(currentVertex != NULL)
    {
        currentEdge = currentVertex->firstOutgoingEdge;
        while(currentEdge != NULL)
        {
            if (currentEdge->edgeId == edgeId)
            {
                *outEdge = currentEdge;
                return XME_STATUS_SUCCESS;
            }

            currentEdge = currentEdge->nextOutgoingEdge;
        }

        currentVertex = currentVertex->nextVertex;
    }

    return XME_STATUS_NOT_FOUND;
}

/**
 * @}
 */
