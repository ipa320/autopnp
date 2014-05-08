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
 * $Id: graph.h 6509 2014-01-27 16:43:00Z geisinger $
 */

/**
 * \file
 * \brief Directed Graph abstraction.
 */

#ifndef XME_HAL_GRAPH_H
#define XME_HAL_GRAPH_H

/**
 * \defgroup hal_graph Directed Graph abstraction
 * @{
 *
 * \brief   General directed graph abstraction.
 *
 * \details The functions in this group allow creation of, inspection of and
 *          iteration over a directed (multi-)graph. The graph consists of
 *          a set of vertices and a set of edges, where each edge has a
 *          dedicated source and sink vertex. Vertices and edges are uniquely
 *          identified in the graph by the vertex respectively edge identifiers
 *          that are returned when edges and vertices are created in the graph.
 *          User-defined data can be attached to every vertex and edge
 *          independently. Iteration is supported over all vertices as well as
 *          over outgoing and incoming edges of a vertex.
 *          In order to avoid multiple insertion of vertices and edges that
 *          are considered equal by the respective algorithm using the graph
 *          data structure, vertex and edge comparison callback functions may
 *          be registered. If registered, attempts to insert new vertices or
 *          edges that are considered equal to already existing items fails
 *          (in case of edges, insertion only fails if an edge between the
 *          same pair of vertices already exists and the edge callback function
 *          indicates that the edges are the same).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \enum xme_hal_graph_vertexId_t
 *
 * \brief  Unique identifier for a vertex in a graph.
 *
 * \note   This value is only identifying in a given graph, it is not
 *         identifying over multiple graphs.
 */
typedef enum
{
    XME_HAL_GRAPH_INVALID_VERTEX_ID = 0, ///< Invalid vertex identifier.
    XME_HAL_GRAPH_MAX_VERTEX_ID = XME_MAX_SYSTEM_VALUE ///< Highest possible vertex identifier.
}
xme_hal_graph_vertexId_t;

/**
 * \enum xme_hal_graph_edgeId_t
 *
 * \brief  Unique identifier of an edge in a graph.
 *
 * \note   This value is only identifying in a given graph, it is not
 *         identifying over multiple graphs.
 */
typedef enum
{
    XME_HAL_GRAPH_INVALID_EDGE_ID = 0, ///< Invalid edge identifier.
    XME_HAL_GRAPH_MAX_EDGE_ID = XME_MAX_SYSTEM_VALUE ///< Highest possible edge identifier.
}
xme_hal_graph_edgeId_t;

/**
 * \typedef xme_hal_graph_vertexCompareCallback_t
 *
 * \brief  Signature of a callback function for vertex comparison.
 *
 * \param[in] vertexData1 Data associated to the first vertex.
 * \param[in] vertexData2 Data associated to the second vertex.
 *
 * \return Should return an integral value indicating the relationship between
 *         the vertices: A zero value indicates that both vertices are equal.
 *         A value greater than zero indicates that the first vertex is greater
 *         than the second vertex. A value less than zero indicates the opposite.
 */
typedef int (*xme_hal_graph_vertexCompareCallback_t)
(
    void* vertexData1,
    void* vertexData2
);

/**
 * \typedef xme_hal_graph_edgeCompareCallback_t
 *
 * \brief  Signature of a callback function for edge comparison.
 *
 * \param[in] edgeData1 Data associated to the first edge.
 * \param[in] edgeData2 Data associated to the second edge.
 *
 * \return Should return an integral value indicating the relationship between
 *         the edges: A zero value indicates that both edges are equal.
 *         A value greater than zero indicates that the first edge is greater
 *         than the second edge. A value less than zero indicates the opposite.
 */
typedef int (*xme_hal_graph_edgeCompareCallback_t)
(
    void* edgeData1,
    void* edgeData2
);

/**
 * \struct xme_hal_graph_graph_t
 *
 * \brief  Graph descriptor.
 */
typedef struct
{
    void* internal1; ///< internal data structure.
    uint16_t internal2; ///< internal data structure.
    uint16_t internal3; ///< internal data structure.
    void* internal4; ///< internal data structure.
    bool internal5; ///< internal data structure.
    xme_hal_graph_vertexId_t internal6; ///< internal data structure.
    xme_hal_graph_edgeId_t internal7; ///< internal data structure.
    xme_hal_graph_vertexCompareCallback_t internal8; ///< internal data structure.
    xme_hal_graph_edgeCompareCallback_t internal9; ///< internal data structure.
}
xme_hal_graph_graph_t;

#if 0
typedef xme_hal_graph_vertex_t xme_hal_graph_vertexIterator_t; ///< Vertex iterator descriptor.
typedef xme_hal_graph_edge_t xme_hal_graph_outgoingEdgeIterator_t; ///< Outgoing edge iterator descriptor.
typedef xme_hal_graph_edge_t xme_hal_graph_incomingEdgeIterator_t; ///< Incoming edge iterator descriptor.
#endif

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the directed graph.
 *
 * \param[in,out] graph Graph descriptor to initialize.
 *
 * \retval XME_STATUS_SUCCESS if the graph component has been properly initialized.
 * \retval XME_STATUS_INVALID_PARAMETER if the incoming parameter is not a valid graph.
 * \retval XME_STATUS_OUT_OF_RESOURCES if the graph component initialization failed.
 */
xme_status_t
xme_hal_graph_init
(
    xme_hal_graph_graph_t* graph
);

/**
 * \brief  Frees resources occupied by the graph.
 *
 * \param[in,out] graph Graph descriptor to finalize.
 *
 * \retval XME_STATUS_SUCCESS if the graph component has been properly freed.
 * \retval XME_STATUS_INVALID_PARAMETER if the incoming parameter is not a valid graph.
 * \retval XME_STATUS_OUT_OF_RESOURCES if the graph component initialization failed.
 */
xme_status_t
xme_hal_graph_fini
(
    xme_hal_graph_graph_t* graph
);

/**
 * \brief Gets the number of vertices contained in the graph.
 * \note If the graph is not initialized, the return value is zero.
 *
 * \param[in] graph Graph descriptor.
 *
 * \return Number of vertices of the graph.
 */
uint16_t
xme_hal_graph_getVertexCount
(
    xme_hal_graph_graph_t* graph
);

/**
 * \brief Gets the number of edges established in the graph.
 * \note If the graph is not initialized, the return value is zero.
 *
 * \param[in] graph Graph descriptor.
 *
 * \return Number of edges in the graph.
 */
uint16_t
xme_hal_graph_getEdgeCount
(
    xme_hal_graph_graph_t* graph
);

/**
 * \brief  Creates an exact copy of an existing graph.
 *
 * \note   The vertex and edge identifiers of the new graph are
 *         in general different from the ones in the template graph.
 *
 * \param[in] templateGraph Graph descriptor of the graph to clone.
 * \param[in,out] outCopiedGraph Graph descriptor to fill in.
 *                The functions expects this graph to be initialized
 *                and empty.
 *
 * \retval XME_STATUS_SUCCESS if the graph was successfully cloned.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the arguments was NULL.
 * \retval XME_STATUS_INVALID_CONFIGURATION if outCopiedGraph did not name an
 *         empty graph (i.e., an initialized graph with no vertices or edges).
 * \retval XME_STATUS_OUT_OF_RESOURCES if there were not enough resources
 *         available to to clone the graph.
 */
xme_status_t
xme_hal_graph_clone
(
    xme_hal_graph_graph_t* templateGraph,
    xme_hal_graph_graph_t* outCopiedGraph
);

/**
 * \brief  Removes all vertices and edges from the graph, leaving the graph empty.
 *
 * \note   Notice that the data contained in the vertices and edges is not
 *         freed by this function. In case the data is dynamically allocated,
 *         first iterate over the graph to deallocate it.
 *
 * \param  graph Graph descriptor of the graph to clear.
 *
 * \retval XME_STATUS_SUCCESS if the graph was successfully cleared.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 */
xme_status_t
xme_hal_graph_clear
(
    xme_hal_graph_graph_t* graph
);

/**
 * \brief  Adds a new vertex to the graph.
 *
 * \details The new vertex does not have any edges to or from other vertices.
 *          In order to link the newly created vertex to other vertices,
 *          use ::xme_hal_graph_addEdge().
 *          In case a vertex comparison callback has been registered, it is
 *          used to decide whether the new vertex is a duplicate of an
 *          already existing vertex, in which case this function returns
 *          XME_HAL_GRAPH_INVALID_VERTEX_ID.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] vertexData Data to associate with the newly created vertex.
 *
 * \return On success, returns a unique identifier for the newly created vertex.
 *         On error (e.g., compare callback indicated that the same vertex
 *         already exists), returns XME_HAL_GRAPH_INVALID_VERTEX_ID.
 */
/* TODO: We should probably implement the following return values instead:
 * \retval XME_CORE_STATUS_SUCCESS if the new vertex was created and its unique
 *         identifier stored in outVertexId.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_ALREADY_EXIST if a vertex compare callback has been
 *         registered and it indicated that the same vertex already exists.
 *         No new vertex has been added in this case.
 */
xme_hal_graph_vertexId_t
xme_hal_graph_addVertex
(
    xme_hal_graph_graph_t* graph,
    void* vertexData
);

/**
 * \brief   Finds a vertex in the graph whose data matches the given data.
 *
 * \details This function iterates over all vertices in the graph (starting
 *          at the vertex next to the one identified by startAfterVertex) and
 *          compares the data associated to every vertex with the given data.
 *          If no vertex comparison callback is registered, the address of the
 *          pointers are compared. Otherwise, the vertex comparison function is
 *          called for every vertex. As soon as a match is found, the respective
 *          vertex identifier is returned.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexData Data to compare with the data of the vertices in the
 *            graph.
 * \param[in] startAfterVertex Identifier of a vertex after which to start
 *            searching (i.e., the given vertex is not searched). If set to
 *            XME_HAL_GRAPH_INVALID_VERTEX_ID, search starts at the first
 *            vertex. You may incrementally call this function to obtain all
 *            vertices matching the data: start with this parameter set to
 *            XME_HAL_GRAPH_INVALID_VERTEX_ID and subsequently pass the value
 *            returned by the respective previous function call if it is not
 *            XME_HAL_GRAPH_INVALID_VERTEX_ID (indicating no more matches).
 *
 * \note If startAfterVertex is not equal to XME_HAL_GRAPH_INVALID_VERTEX_ID
 *       and does not name vertex in the graph, the return value is
 *       XME_HAL_GRAPH_INVALID_VERTEX_ID.
 *
 * \return On successful match, returns the vertex identifier of the next
 *         vertex matching the given data. If no match is found, returns
 *         XME_HAL_GRAPH_INVALID_VERTEX_ID.
 */
xme_hal_graph_vertexId_t
xme_hal_graph_getNextVertexWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    void* vertexData,
    xme_hal_graph_vertexId_t startAfterVertex
);

/**
 * \brief  Removes a vertex from the graph.
 *
 * \details When the vertex is removed from the graph, all outgoing and
 *          incoming edges of that vertex are removed as well.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] vertexId Unique identifier of the vertex to remove.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the vertex has been successfully removed.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NOT_FOUND if no vertex with the given identifier could
 *         be found.
 */
xme_status_t
xme_hal_graph_removeVertex
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief   Removes one or multiple vertices from the graph identified by comparing the
 *          data stored in the vertices.
 *
 * \details This function iterates over the vertices in the graph and compares
 *          the data associated to every vertex with the given data. If no vertex
 *          comparison callback is registered, the address of the pointers is compared.
 *          Otherwise, the vertex comparison function is called for every vertex.
 *          If a match is found, the vertex is removed from the graph.
 *          This also includes removing all outgoing and all incoming edges of that vertex.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] vertexData Data to compare with the data of the vertices in the graph.
 * \param[in] all Whether search should be continued for all other vertices after
 *            the first match.
 *
 * \retval XME_CORE_STATUS_SUCCESS if one or more vertices have been successfully removed.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NOT_FOUND if there were no vertices matching vertexData.
 */
xme_status_t
xme_hal_graph_removeVertexWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    void* vertexData,
    bool all
);

/**
 * \brief  Adds an edge between two vertices, identifier by their vertex identifiers.
 *
 * \details Both vertices must have been previously added using
 *          ::xme_hal_graph_addVertex(), for example.
 *          In case an edge comparison callback has been registered, it is
 *          used to decide whether the new edge is a duplicate of an
 *          already existing (directed) edge between the given source and sink, in
 *          which case this function returns XME_HAL_GRAPH_INVALID_EDGE_ID.
 *          Self-edges are allowed, i.e., the graph may be a multigraph.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] sourceVertexId Unique identifier of the source vertex of the newly
 *            created edge.
 * \param[in] sinkVertexId Unique identifier of the sink vertex of the newly
 *            created edge.
 * \param[in] edgeData Data to associate with the newly created edge.
 *
 * \return On success, returns a unique identifier for the newly created edge.
 *         On error (e.g., compare callback indicated that the same edge
 *         already exists), returns XME_HAL_GRAPH_INVALID_EDGE_ID.
 */
/* TODO: We should probably implement the following return values instead:
 * \retval XME_CORE_STATUS_SUCCESS if the new edge was created and its unique
 *         identifier stored in outEdgeId.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NOT_FOUND if either the source or the sink vertex could
 *         not be found.
 * \retval XME_STATUS_ALREADY_EXIST if an edge compare callback has been
 *         registered and it indicated that the same edge already exists.
 *         No new edge has been added in this case.
 */
xme_hal_graph_edgeId_t
xme_hal_graph_addEdge
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    void* edgeData
);

/**
 * \brief   Finds an edge whose data matches the given data.
 *
 * \details This function iterates over all edges in the graph (first iterating
 *          over all vertices and then for each vertex considering its outgoing
 *          edges) and compares the data associated to every edge with the given data.
 *          If no edge comparison callback is registered, the address of the pointers
 *          is compared. Otherwise, the edge comparison function is called for
 *          every edge. As soon as a match is found, the respective edge
 *          identifier is returned.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] edgeData Data to compare with the data of the edges in the graph.
 * \param[in] startAfterEdge Identifier of an edge after which to start
 *            searching (i.e., the given edge is not searched). If set to
 *            XME_HAL_GRAPH_INVALID_EDGE_ID, search starts at the first
 *            edge in the graph. You may incrementally call this function to
 *            obtain all edges matching the data: start with this parameter set
 *            to XME_HAL_GRAPH_INVALID_EDGE_ID and subsequently pass the value
 *            returned by the respective previous function call if it is not
 *            XME_HAL_GRAPH_INVALID_EDGE_ID (indicating no more matches).
 *
 * \note If startAfterEdge is not equal to XME_HAL_GRAPH_INVALID_EDGE_ID and
 *       does not name a (directed) edge in the graph, the return value is
 *       XME_HAL_GRAPH_INVALID_EDGE_ID.
 *
 * \return On successful match, returns the edge identifier of the next
 *         edge matching the given data. If no match is found, returns
 *         XME_HAL_GRAPH_INVALID_EDGE_ID.
 */
xme_hal_graph_edgeId_t
xme_hal_graph_getNextEdgeWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    void* edgeData,
    xme_hal_graph_edgeId_t startAfterEdge
);

/**
 * \brief   Finds an edge between two vertices whose data matches the given data.
 *
 * \details This function iterates over the (directed) edges between the
 *          given pair of vertices (starting at the edge next to the one
 *          identified by startAfterEdge) and compares the data associated
 *          to every edge with the given data. If no edge comparison callback
 *          is registered, the address of the pointers is compared.
 *          Otherwise, the edge comparison function is called for every
 *          edge between the given pair of vertices. As soon as a match is
 *          found, the respective edge identifier is returned.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] sourceVertexId Unique identifier of the source vertex of the edge.
 * \param[in] sinkVertexId Unique identifier of the sink vertex of the edge.
 * \param[in] edgeData Data to compare with the data of the edges between the
 *            given pair of vertices.
 * \param[in] startAfterEdge Identifier of an edge after which to start
 *            searching (i.e., the given edge is not searched). If set to
 *            XME_HAL_GRAPH_INVALID_EDGE_ID, search starts at the first
 *            edge between the given pair of vertices. You may incrementally
 *            call this function to obtain all edges matching the data:
 *            start with this parameter set to XME_HAL_GRAPH_INVALID_EDGE_ID
 *            and subsequently pass the value returned by the respective
 *            previous function call if it is not
 *            XME_HAL_GRAPH_INVALID_EDGE_ID (indicating no more matches).
 *
 * \note If startAfterEdge is not equal to XME_HAL_GRAPH_INVALID_EDGE_ID and
 *       does not name a (directed) edge between the given source and sink
 *       vertex, the return value is XME_HAL_GRAPH_INVALID_EDGE_ID.
 *
 * \note If one of sourceVertexId or sinkVertexId is set to
 *       XME_HAL_GRAPH_INVALID_VERTEX_ID, the return value is
 *       XME_HAL_GRAPH_INVALID_EDGE_ID.
 *
 * \return On successful match, returns the edge identifier of the next
 *         edge matching the given data. If no match is found, returns
 *         XME_HAL_GRAPH_INVALID_EDGE_ID.
 */
xme_hal_graph_edgeId_t
xme_hal_graph_getNextEdgeBetweenWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    void* edgeData,
    xme_hal_graph_edgeId_t startAfterEdge
);

/**
 * \brief  Removes the edge with the given unique identifier.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] edgeId Unique identifier of the edge to remove.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the edge has been successfully removed.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NOT_FOUND if no edge with the given identifier could
 *         be found.
 */
xme_status_t
xme_hal_graph_removeEdge
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_edgeId_t edgeId
);

/**
 * \brief   Removes one or multiple edges from the graph identified by comparing the
 *          data stored in the edges.
 *
 * \details This function iterates over all edges in the graph (first iterating
 *          over all vertices and then for each vertex considering its outgoing
 *          edges) and compares the data associated to every edge with the given data.
 *          If no edge comparison callback is registered, the address of the pointers
 *          is compared. Otherwise, the edge comparison function is called for every edge.
 *          If a match is found, the edge is removed from the graph.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] edgeData Data to compare with the data of the edges in the graph.
 * \param[in] all Whether search should be continued for all other edges in the graph
 *            after the first match.
 *
 * \retval XME_CORE_STATUS_SUCCESS if one or more edges have been successfully removed.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NOT_FOUND if none of the edges in the graph match edgeData.
 */
xme_status_t
xme_hal_graph_removeEdgeWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    void* edgeData,
    bool all
);

/**
 * \brief   Removes one or multiple edges between two vertices.
 *
 * \details Both vertices must have been previously added using
 *          ::xme_hal_graph_addVertex(), for example.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] sourceVertexId Unique identifier of the source vertex of the edge(s) to remove.
 * \param[in] sinkVertexId Unique identifier of the sink vertex of the edge(s) to remove.
 * \param[in] all Whether search should be continued for all other edges between the given
 *            source and sink vertex after the first match.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the edge between source and sink vertex is removed.
 * \retval XME_STATUS_NOT_FOUND if either source or sink vertices are not part of the graph.
 * \retval XME_STATUS_NO_SUCH_VALUE if there is not an edge between source vertex and sink vertex.
 * \retval XME_CORE_STATUS_OUT_OF_RESOURCES if cannot complete the operation.
 */
xme_status_t
xme_hal_graph_removeEdgeBetween
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    bool all
);

/**
 * \brief   Removes one or multiple edges from the graph identified by comparing the
 *          data stored in the edges.
 *
 * \details This function iterates over the (directed) edges originating from the given
 *          vertex and compares the data associated to every edge with the given data.
 *          If no edge comparison callback is registered, the address of the pointers
 *          is compared. Otherwise, the edge comparison function is called for every edge.
 *          If a match is found, the edge is removed from the graph.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] sourceVertexId Unique identifier of the source vertex of the edge to remove.
 * \param[in] sinkVertexId Unique identifier of the sink vertex of the edge to remove.
 * \param[in] edgeData Data to compare with the data of the edges between the given
 *            source and sink vertex.
 * \param[in] all Whether search should be continued for all other edges between the
 *            given source and sink vertex after the first match.
 *
 * \retval XME_CORE_STATUS_SUCCESS if one or more edges have been successfully removed.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NO_SUCH_VALUE if there is no edge between the given source vertex
 *         and sink vertex.
 * \retval XME_STATUS_NOT_FOUND if none of the edges between the given source and sink
 *         vertex match edgeData.
 */
xme_status_t
xme_hal_graph_removeEdgeBetweenWithDataComparison
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId,
    void* edgeData,
    bool all
);

/**
 * \brief  Creates a new vertex iterator for the graph.
 *
 * \details The graph establishes the iterator position at the first vertex of the graph.
 *          For accessing the next vertex, the ::xme_hal_graph_hasNextVertex() and
 *          ::xme_hal_graph_nextVertex() functions are provided.
 *
 * \note Iteration order for vertices is the order of insertion of vertices into the graph.
 *
 * \param[in] graph Graph descriptor.
 *
 * \retval XME_STATUS_SUCCESS if the vertex iterator has been successfully created.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_UNSUPPORTED if the initialization of a subsequent iterator
 *         was called before the finalization of the current iterator.
 *         Nested iteration is currently not supported.
 */
/*TODO:
 * \return On success, returns a non-NULL vertex iterator for the graph
 *         (even if the graph does not contain any vertices).
 *         On error (e.g., NULL graph argument), returns NULL.
 */
//xme_hal_graph_vertexIterator_t
xme_status_t
xme_hal_graph_initVertexIterator
(
    xme_hal_graph_graph_t* graph
);

/**
 * \brief  Finalizes a vertex iterator.
 *
 * \param[in] graph Graph descriptor.
 *
 * \retval XME_STATUS_SUCCESS if the iterator has been successfully finalized.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the input parameters was invalid.
 * \retval XME_STATUS_INVALID_CONFIGURATION if no iterator has been initialized before.
 */
// TODO: Update the docs above (especially the return values) when the API is changed!
xme_status_t
xme_hal_graph_finiVertexIterator
(
    xme_hal_graph_graph_t* graph
    //xme_hal_graph_vertexIterator_t* vertexIterator
);

/**
 * \brief  Determines if there are more vertices to iterate over.
 *
 * \details Vertex iteration is associated to the graph structure. The
 *          checking of next iterator is established during the 
 *          ::xme_hal_graph_nextVertex() function call. 
 *
 * \note If the iterator has not been initialized before, false is returned.
 *
 * \param[in] graph Graph descriptor.
 *
 * \retval true if there are more vertices to iterate over.
 * \retval false if there are no more vertices to iterate over.
 */
bool
xme_hal_graph_hasNextVertex
(
    xme_hal_graph_graph_t* graph
    //xme_hal_graph_vertexIterator_t* vertexIterator
);

/**
 * \brief  Retrieves the unique identifier of the next vertex in the iteration.
 *
 * \details When calling this function, the vertex iterator moves to the next
 *          item in the graph, if any.
 *          The vertices are obtained in the order that they have been added to
 *          the graph. 
 *
 * \note This method is used jointly with ::xme_hal_graph_hasNextVertex()
 *       and ::xme_hal_graph_initVertexIterator().
 *
 * \note If the vertex iterator is not initialized,
 *       XME_HAL_GRAPH_INVALID_VERTEX_ID is returned.
 *
 * \param[in] graph Graph descriptor.
 *
 * \return On success, returns the unique identifier of the next vertex in the
 *         iteration. On error, returns XME_HAL_GRAPH_INVALID_VERTEX_ID.
 */
xme_hal_graph_vertexId_t
xme_hal_graph_nextVertex
(
    xme_hal_graph_graph_t* graph
    //xme_hal_graph_vertexIterator_t* vertexIterator
);

/**
 * \brief  Creates a new outgoing edge iterator for the given vertex in the graph.
 *
 * \details The graph establishes the iterator position at the first outgoing edge
 *          of the vertex. For accessing the next outgoing edge, the
 *          ::xme_hal_graph_hasNextOutgoingEdge() and
 *          ::xme_hal_graph_hasNextOutgoingEdge() functions are provided.
 *
 * \details The outgoing edge iterator iterates over all outgoing edges for the
 *          given vertex. The iteration order is the insertion order of outgoing
 *          edges of that vertex.
 *
 * \note If the vertex id is a non existing vertex in the graph, XME_STATUS_INVALID_PARAMETER
 *       is returned. 
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId Unique identifier of the vertex for which the outgoing
 *            edge iterator is to be initialized.
 *
 * \retval XME_STATUS_SUCCESS if the edge iterator has been successfully created.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL or vertexId was invalid.
 * \retval XME_STATUS_UNSUPPORTED if the initialization of a subsequent iterator
 *         was called before the finalization of the current iterator.
 *         Nested iteration is currently not supported.
 */
/*TODO:
 * \return On success, returns a non-NULL edge iterator for the given vertex
 *         (even if the vertex does have any outgoing edges).
 *         On error (e.g., NULL graph argument), returns NULL.
 */
//xme_hal_graph_outgoingEdgeIterator_t
xme_status_t
xme_hal_graph_initOutgoingEdgeIterator
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief  Determines if there are more outgoing edges to iterate over.
 *
 * \details This function determines if there are more outgoing edges for the
 *          given vertex. 
 *
 * \note If the iterator has not been initialized before, false is returned.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId the vertex identifier.
 *
 * \retval true if there are more outgoing edges to iterate over.
 * \retval false if there are no more outgoing edges to iterate over.
 */
bool
xme_hal_graph_hasNextOutgoingEdge
(
    xme_hal_graph_graph_t* graph,
    //xme_hal_graph_outgoingEdgeIterator_t* outgoingEdgeIterator
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief  Retrieves the unique identifier of the next outgoing edge in the iteration.
 *
 * \details When calling this function, the edge iterator moves to the next
 *          outgoing edge of the vertex, if any. The order of the outgoing edge
 *          iteration is the order of edge addition to the graph.
 *
 * \note This method is used jointly with ::xme_hal_graph_hasNextOutgoingEdge()
 *       and ::xme_hal_graph_initOutgoingEdgeIterator().
 *
 * \note If the vertex iterator is not initialized,
 *       XME_HAL_GRAPH_INVALID_EDGE_ID is returned.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId the vertex identifier.
 *
 * \return On success, returns the unique identifier of the next outgoing edge
 *         in the iteration. On error, returns XME_HAL_GRAPH_INVALID_EDGE_ID.
 */
xme_hal_graph_edgeId_t
xme_hal_graph_nextOutgoingEdge
(
    xme_hal_graph_graph_t* graph,
    //xme_hal_graph_outgoingEdgeIterator_t* outgoingEdgeIterator
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief  Finalizes an outgoing edge iterator.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId the vertex identifier.
 *
 * \retval XME_STATUS_SUCCESS if the iterator has been successfully finalized.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the input parameters was invalid.
 * \retval XME_STATUS_INVALID_CONFIGURATION if no iterator has been initialized before.
 */
// TODO: Update the docs above (especially the return values) when the API is changed!
xme_status_t
xme_hal_graph_finiOutgoingEdgeIterator
(
    xme_hal_graph_graph_t* graph,
    //xme_hal_graph_outgoingEdgeIterator_t* outgoingEdgeIterator
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief  Creates a new incoming edge iterator for the given vertex in the graph.
 *
 * \details The graph establishes the iterator position at the first incoming edge
 *          of the vertex. For accessing the next outgoing edge, the
 *          ::xme_hal_graph_hasNextIncomingEdge() and
 *          ::xme_hal_graph_hasNextIncomingEdge() functions are provided.
 *
 * \details The incoming vertex iterator iterates over all edges which have incoming
 *          edges for the given vertex. The function explores the vertices in the
 *          graph sequentially in order of addition and for every vertex, checks
 *          for all outgoing edges whether they are incoming edges of the given
 *          vertex.
 *
 * \note If the vertex identifier specifies a non-existing vertex in the graph,
 *       XME_STATUS_INVALID_PARAMETER is returned. 
 * 
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId Unique identifier of the vertex for which the incoming
 *            edge iterator is to be initialized.
 *
 * \retval XME_STATUS_SUCCESS if the edge iterator has been successfully created.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL or the given vertex
 *         identifier was not found in the graph.
 * \retval XME_STATUS_UNSUPPORTED if the initialization of a subsequent iterator
 *         was called before the finalization of the current iterator.
 *         Nested iteration is currently not supported.
 */
/*TODO:
 * \return On success, returns a non-NULL edge iterator for the given vertex
 *         (even if the vertex does have any incoming edges).
 *         On error (e.g., NULL graph argument), returns NULL.
 */
//xme_hal_graph_incomingEdgeIterator_t
xme_status_t
xme_hal_graph_initIncomingEdgeIterator
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief  Determines if there are more incoming edges to iterate over.
 *
 * \note If the iterator has not been initialized before, false is returned.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId the vertex identifier.
 *
 * \retval true if there are more incoming edges to iterate over.
 * \retval false if there are no more incoming edges to iterate over.
 */
bool
xme_hal_graph_hasNextIncomingEdge
(
    xme_hal_graph_graph_t* graph,
    //xme_hal_graph_incomingEdgeIterator_t* incomingEdgeIterator
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief  Retrieves the unique identifier of the next incoming edge in the iteration.
 *
 * \details When calling this function, the edge iterator moves to the next
 *          incoming edge of the vertex, if any.
 *
 * \details The function explores the vertices in the graph sequentially in order of
 *          addition and for every vertex, checks for all outgoing edges whether they
 *          are incoming edges of the given vertex.
 *
 * \note This method is used jointly with ::xme_hal_graph_hasNextIncomingEdge()
 *       and ::xme_hal_graph_initIncomingEdgeIterator().
 *
 * \note If the vertex iterator is not initialized,
 *       XME_HAL_GRAPH_INVALID_EDGE_ID is returned.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId the vertex identifier.
 *
 * \return On success, returns the unique identifier of the next incoming edge
 *         in the iteration. On error, returns XME_HAL_GRAPH_INVALID_EDGE_ID.
 */
xme_hal_graph_edgeId_t
xme_hal_graph_nextIncomingEdge
(
    xme_hal_graph_graph_t* graph,
    //xme_hal_graph_incomingEdgeIterator_t* incomingEdgeIterator
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief  Finalizes an incoming edge iterator.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId the vertex identifier.
 *
 * \retval XME_STATUS_SUCCESS if the iterator has been successfully finalized.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the input parameters was invalid.
 * \retval XME_STATUS_INVALID_CONFIGURATION if no iterator has been initialized before.
 */
// TODO: Update the docs above (especially the return values) when the API is changed!
xme_status_t
xme_hal_graph_finiIncomingEdgeIterator
(
    xme_hal_graph_graph_t* graph,
    //xme_hal_graph_incomingEdgeIterator_t* incomingEdgeIterator
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief  Retrieves the data associated to a vertex.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] vertexId Unique identifier of the vertex to retrieve the data for.
 * \param[out] outVertexData Address of a pointer where to put the vertex data.
 *             This parameter may be NULL, in which case the result of this function
 *             can be used to determine whether or not the vertex is present in the graph.
 *
 * \retval XME_STATUS_SUCCESS if the vertex was found and the data was successfully stored
 *         at the address specified in outVertexData, provided outVertexData was non-NULL.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NOT_FOUND if the given vertex identifier was not found in the graph.
 */
xme_status_t
xme_hal_graph_getVertexData
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexId_t vertexId,
    void** outVertexData
);

/**
 * \brief  Retrieves the data associated to an edge.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] edgeId Unique identifier of the edge to retrieve the data for.
 * \param[out] outEdgeData Address of a pointer where to put the edge data.
 *             This parameter may be NULL, in which case the result of this function
 *             can be used to determine whether or not the edge is present in the graph.
 *
 * \retval XME_STATUS_SUCCESS if the edge was found and the data was successfully stored
 *         at the address specified in outEdgeData, provided outEdgeData was non-NULL.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 * \retval XME_STATUS_NOT_FOUND if the given edge identifier was not found in the graph.
 */
xme_status_t
xme_hal_graph_getEdgeData
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_edgeId_t edgeId,
    void** outEdgeData
);

/**
 * \brief  Retrieves the unique identifier of the source vertex of an edge.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] edgeId Unique identifier of the edge.
 *
 * \return On success, returns the unique vertex identifier of the source vertex
 *         of the given edge. On error, XME_HAL_GRAPH_INVALID_VERTEX_ID is returned.
 */
xme_hal_graph_vertexId_t
xme_hal_graph_getSourceVertex
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_edgeId_t edgeId
);

/**
 * \brief  Retrieves the unique identifier of the sink vertex of an edge.
 *
 * \param[in] graph Graph descriptor.
 * \param[in] edgeId Unique identifier of the edge.
 *
 * \return On success, returns the unique vertex identifier of the sink vertex
 *         of the given edge. On error, XME_HAL_GRAPH_INVALID_VERTEX_ID is returned.
 */
xme_hal_graph_vertexId_t
xme_hal_graph_getSinkVertex
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_edgeId_t edgeId
);

/**
 * \brief  Retrieves the callback function currently set up as vertex compare
 *         callback.
 *
 * \details See ::xme_hal_graph_setVertexCompareCallback for a description of
 *          the vertex compare callback.
 *
 * \param[in] graph the incoming graph.
 *
 * \return On success, returns a pointer to the currently registered vertex
 *         compare callback or NULL if no vertex comparison callback is
 *         currently registered. On error (e.g., NULL graph parameter),
 *         returns NULL.
 */
xme_hal_graph_vertexCompareCallback_t
xme_hal_graph_getVertexCompareCallback
(
    xme_hal_graph_graph_t* graph
);

/**
 * \brief  Registers a callback function to be invoked by the operations of the
 *         graph data structure when two vertices need to be compared with each
 *         other.
 *
 * \details The information passed to the callback function is the vertex data
 *          of the two vertices to be compared.
 *          The vertex comparison function is invoked under the following
 *          circumstances:
 *           - In ::xme_hal_graph_addVertex() in order to check whether a vertex
 *             with the same data is already present in the graph, in which case
 *             vertex insertion would fail.
 *           - In ::xme_hal_graph_removeVertexWithDataComparison() in order to
 *             check whether a vertex should be removed from the graph.
 *          The new callback function replaces an existing vertex comparison
 *          callback function.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] vertexCompareCallback Vertex comparation callback.
 *
 * \retval XME_STATUS_SUCCESS if the vertex compare callback has been successfully
 *         registered.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 */
xme_status_t
xme_hal_graph_setVertexCompareCallback
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_vertexCompareCallback_t vertexCompareCallback
);

/**
 * \brief  Retrieves the callback function currently set up as edge compare
 *         callback.
 *
 * \details See ::xme_hal_graph_setEdgeCompareCallback for a description of
 *          the edge compare callback.
 *
 * \param[in] graph the incoming graph.
 *
 * \return On success, returns a pointer to the currently registered edge
 *         compare callback or NULL if no edge comparison callback is
 *         currently registered. On error (e.g., NULL graph parameter),
 *         returns NULL.
 */
xme_hal_graph_edgeCompareCallback_t
xme_hal_graph_getEdgeCompareCallback
(
    xme_hal_graph_graph_t* graph
);

/**
 * \brief  Registers a callback function to be invoked by the operations of the
 *         graph data structure when two edges need to be compared with each
 *         other.
 *
 * \details The information passed to the callback function is the edge data of
 *          the two edges to be compared.
 *          The edge comparison function is invoked under the following
 *          circumstances:
 *           - In ::xme_hal_graph_addEdge() in order to check whether an edge
 *             between the same pair of vertices with the same data is already
 *             present in the graph, in which case edge insertion would fail.
 *           - In ::xme_hal_graph_removeEdgeBetweenWithDataComparison() in order
 *             to check whether an edge should be removed from the graph.
 *          The new callback function replaces an existing edge comparison
 *          callback function.
 *
 * \param[in,out] graph Graph descriptor.
 * \param[in] edgeCompareCallback Edge comparation callback.
 *
 * \retval XME_STATUS_SUCCESS if the vertex compare callback has been successfully
 *         registered.
 * \retval XME_STATUS_INVALID_PARAMETER if graph was NULL.
 */
xme_status_t
xme_hal_graph_setEdgeCompareCallback
(
    xme_hal_graph_graph_t* graph,
    xme_hal_graph_edgeCompareCallback_t edgeCompareCallback
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_HAL_GRAPH_H
