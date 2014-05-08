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
 * $Id: smokeTestGraph.cpp 4885 2013-08-30 14:30:07Z ruiz $
 */

/**
 * \file
 *         Graph smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/graph.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class GraphSmokeTest: public ::testing::Test
{
protected:
    GraphSmokeTest()
    {
    }

    virtual ~GraphSmokeTest()
    {
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     GraphSmokeTest                                                         //
//----------------------------------------------------------------------------//

// xme_hal_graph_init()

TEST_F(GraphSmokeTest, initNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_init(NULL));
}

TEST_F(GraphSmokeTest, initValidGraph)
{
    xme_hal_graph_graph_t g;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&g));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));
}

// xme_hal_graph_fini()

TEST_F(GraphSmokeTest, finiNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_fini(NULL));
}

// xme_hal_graph_getVertexCount()

TEST_F(GraphSmokeTest, countVerticesOfNullGraph)
{
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(NULL));
}

// xme_hal_graph_getEdgeCount()

TEST_F(GraphSmokeTest, countEdgesOfNullGraph)
{
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(NULL));
}

// xme_hal_graph_clone()

TEST_F(GraphSmokeTest, cloneNullGraphToNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_clone(NULL, NULL));
}

TEST_F(GraphSmokeTest, cloneNullGraphToValidGraph)
{
    xme_hal_graph_graph_t cloned;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&cloned));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_clone(NULL, &cloned));

    xme_hal_graph_fini(&cloned);
}

// xme_hal_graph_clear()

TEST_F(GraphSmokeTest, clearNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_clear(NULL));
}

// xme_hal_graph_addVertex()

TEST_F(GraphSmokeTest, addVertexWithNullDataToNullGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_addVertex(NULL, NULL));
}

TEST_F(GraphSmokeTest, addVertexWithNonNullDataToNullGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_addVertex(NULL, (void*)42));
}

// xme_hal_graph_getNextVertexWithDataComparison()

TEST_F(GraphSmokeTest, getNextVertexWithDataComparisonWithNullGraphAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(NULL, NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, getNextVertexWithDataComparisonWithNullGraphAndNullDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(NULL, NULL, (xme_hal_graph_vertexId_t)42));
}

TEST_F(GraphSmokeTest, getNextVertexWithDataComparisonWithNullGraphAndNonNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(NULL, (void*) 42, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, getNextVertexWithDataComparisonWithNullGraphAndNonNullDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(NULL, (void*) 42, (xme_hal_graph_vertexId_t)43));
}

// xme_hal_graph_removeVertex()

TEST_F(GraphSmokeTest, removeInvalidVertexFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeVertex(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, removeNonExistingVertexFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeVertex(NULL, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_removeVertexWithDataComparison()

TEST_F(GraphSmokeTest, removeVertexWithDataComparisonWithNullVertexDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeVertexWithDataComparison(NULL, NULL, true));
}

TEST_F(GraphSmokeTest, removeVertexWithDataComparisonWithNonExistingVertexDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeVertexWithDataComparison(NULL, (void*)42, true));
}

// xme_hal_graph_addEdge()

TEST_F(GraphSmokeTest, addEdgeBetweenInvalidVerticesToNullGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL));
}

TEST_F(GraphSmokeTest, addEdgeBetweenValidVertexAndInvalidVertexToNullGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(NULL, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL));
}

TEST_F(GraphSmokeTest, addEdgeBetweenInvalidVertexAndValidVertexToNullGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, NULL));
}

// xme_hal_graph_getNextEdgeWithDataComparison()

TEST_F(GraphSmokeTest, getNextEdgeWithDataComparisonWithNullGraphAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(NULL, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeWithDataComparisonWithNullGraphAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 42, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeWithDataComparisonWithNullGraphAndNullDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, (xme_hal_graph_edgeId_t)42));
}

TEST_F(GraphSmokeTest, getNextEdgeWithDataComparisonWithNullGraphAndArbitraryDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 42, (xme_hal_graph_edgeId_t)43));
}

// xme_hal_graph_getNextEdgeBetweenWithDataComparison()

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndInvalidSourceAndInvalidSinkAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndArbitrarySourceAndInvalidSinkAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndInvalidSourceAndArbitrarySinkAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndArbitrarySourceAndArbitrarySinkAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndInvalidSourceAndInvalidSinkAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 42, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndArbitrarySourceAndInvalidSinkAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 43, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndInvalidSourceAndArbitrarySinkAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, (void*) 43, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndArbitrarySourceAndArbitrarySinkAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, (void*) 44, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndInvalidSourceAndInvalidSinkAndNullDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, (xme_hal_graph_edgeId_t)42));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndArbitrarySourceAndInvalidSinkAndNullDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, (xme_hal_graph_edgeId_t)43));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndInvalidSourceAndArbitrarySinkAndNullDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, NULL, (xme_hal_graph_edgeId_t)43));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndArbitrarySourceAndArbitrarySinkAndNullDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, NULL, (xme_hal_graph_edgeId_t)44));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndInvalidSourceAndInvalidSinkAndArbitraryDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 42, (xme_hal_graph_edgeId_t)43));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndArbitrarySourceAndInvalidSinkAndArbitraryDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 43, (xme_hal_graph_edgeId_t)44));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndInvalidSourceAndArbitrarySinkAndArbitraryDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, (void*) 43, (xme_hal_graph_edgeId_t)44));
}

TEST_F(GraphSmokeTest, getNextEdgeBetweenWithDataComparisonWithNullGraphAndArbitrarySourceAndArbitrarySinkAndArbitraryDataAndArbitraryStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, (void*) 44, (xme_hal_graph_edgeId_t)45));
}

// xme_hal_graph_removeEdge()

TEST_F(GraphSmokeTest, removeEdgeWithInvalidEdgeFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdge(NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, removeEdgeWithNonExistingEdgeFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdge(NULL, (xme_hal_graph_edgeId_t)42));
}

// xme_hal_graph_removeEdgeWithDataComparison()

TEST_F(GraphSmokeTest, removeEdgeWithDataComparisonWithNullVertexDataAndNullEdgeDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeWithDataComparison(NULL, NULL, true));
}

TEST_F(GraphSmokeTest, removeEdgeWithDataComparisonWithNullVertexDataAndNonExistingEdgeDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeWithDataComparison(NULL, (void*)44, true));
}

// xme_hal_graph_removeEdgeBetween()

TEST_F(GraphSmokeTest, removeEdgeBetweenInvalidVerticesFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetween(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, true));
}

TEST_F(GraphSmokeTest, removeEdgeBetweenValidVertexAndInvalidVertexFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetween(NULL, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, true));
}

TEST_F(GraphSmokeTest, removeEdgeBetweenInvalidVertexAndValidVertexFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetween(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, true));
}

TEST_F(GraphSmokeTest, removeEdgeBetweenValidVerticesFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetween(NULL, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)42, true));
}

// xme_hal_graph_removeEdgeBetweenWithDataComparison()

TEST_F(GraphSmokeTest, removeEdgeBetweenWithDataComparisonWithNullVertexDataAndNullEdgeDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, true));
}

TEST_F(GraphSmokeTest, removeEdgeBetweenWithDataComparisonWithNonExistingSourceVertexDataAndNullEdgeDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, true));
}

TEST_F(GraphSmokeTest, removeEdgeBetweenWithDataComparisonWithNonExistingDestinationVertexDataAndNullEdgeDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, NULL, true));
}

TEST_F(GraphSmokeTest, removeEdgeBetweenWithDataComparisonWithNonExistingVertexDataAndNullEdgeDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, NULL, true));
}

TEST_F(GraphSmokeTest, removeEdgeBetweenWithDataComparisonWithNullVertexDataAndNonExistingEdgeDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetweenWithDataComparison(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*)44, true));
}

TEST_F(GraphSmokeTest, removeEdgeBetweenWithDataComparisonWithNonExistingVertexDataAndNonExistingEdgeDataFromNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdgeBetweenWithDataComparison(NULL, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, (void*)44, true));
}

// xme_hal_graph_initVertexIterator()

TEST_F(GraphSmokeTest, initVertexIteratorForNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initVertexIterator(NULL));
}

// xme_hal_graph_finiVertexIterator()

TEST_F(GraphSmokeTest, finiVertexIteratorForNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiVertexIterator(NULL));
}

// xme_hal_graph_hasNextVertex()

TEST_F(GraphSmokeTest, hasNextVertexForNullGraph)
{
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(NULL));
}

// xme_hal_graph_nextVertex()

TEST_F(GraphSmokeTest, nextVertexForNullGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(NULL));
}

// xme_hal_graph_initOutgoingEdgeIterator()

TEST_F(GraphSmokeTest, initEdgeIteratorForNullGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, initEdgeIteratorForNullGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(NULL, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_hasNextOutgoingEdge()

TEST_F(GraphSmokeTest, hasNextEdgeForNullGraphAndInvalidVertex)
{
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, hasNextEdgeForNullGraphAndNonExistingVertex)
{
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(NULL, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_nextOutgoingEdge()

TEST_F(GraphSmokeTest, nextEdgeForNullGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, nextEdgeForNullGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(NULL, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_finiOutgoingEdgeIterator()

TEST_F(GraphSmokeTest, finiOutgoingEdgeIteratorForNullGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, finiOutgoingEdgeIteratorForNullGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(NULL, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_initIncomingEdgeIterator()

TEST_F(GraphSmokeTest, initIncomingEdgeIteratorForNullGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, initIncomingEdgeIteratorForNullGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(NULL, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_hasNextIncomingEdge()

TEST_F(GraphSmokeTest, hasNextIncomingEdgeForNullGraphAndInvalidVertex)
{
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, hasNextIncomingEdgeForNullGraphAndNonExistingVertex)
{
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(NULL, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_nextIncomingEdge()

TEST_F(GraphSmokeTest, nextIncomingEdgeForNullGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, nextIncomingEdgeForNullGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(NULL, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_finiIncomingEdgeIterator()

TEST_F(GraphSmokeTest, finiIncomingEdgeIteratorForNullGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphSmokeTest, finiIncomingEdgeIteratorForNullGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(NULL, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_getVertexData()

TEST_F(GraphSmokeTest, getVertexDataForNullGraphAndInvalidVertexAndNullResult)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_getVertexData(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL));
}

TEST_F(GraphSmokeTest, getVertexDataForNullGraphAndNonExistingVertexAndNullResult)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_getVertexData(NULL, (xme_hal_graph_vertexId_t)42, NULL));
}

TEST_F(GraphSmokeTest, getVertexDataForNullGraphAndInvalidVertexAndNonNullResult)
{
    int i;
    void* p = &i;

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_getVertexData(NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID, &p));
    EXPECT_EQ(p, &i);
}

TEST_F(GraphSmokeTest, getVertexDataForNullGraphAndNonExistingVertexAndNonNullResult)
{
    int i;
    void* p = &i;

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_getVertexData(NULL, (xme_hal_graph_vertexId_t)42, &p));
    EXPECT_EQ(p, &i);
}

// xme_hal_graph_getEdgeData()

TEST_F(GraphSmokeTest, getEdgeDataForNullGraphAndInvalidVertexAndNullResult)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_getEdgeData(NULL, XME_HAL_GRAPH_INVALID_EDGE_ID, NULL));
}

TEST_F(GraphSmokeTest, getEdgeDataForNullGraphAndNonExistingVertexAndNullResult)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_getEdgeData(NULL, (xme_hal_graph_edgeId_t)42, NULL));
}

TEST_F(GraphSmokeTest, getEdgeDataForNullGraphAndInvalidVertexAndNonNullResult)
{
    int i;
    void* p = &i;

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_getEdgeData(NULL, XME_HAL_GRAPH_INVALID_EDGE_ID, &p));
    EXPECT_EQ(p, &i);
}

TEST_F(GraphSmokeTest, getEdgeDataForNullGraphAndNonExistingVertexAndNonNullResult)
{
    int i;
    void* p = &i;

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_getEdgeData(NULL, (xme_hal_graph_edgeId_t)42, &p));
    EXPECT_EQ(p, &i);
}

// xme_hal_graph_getSourceVertex()

TEST_F(GraphSmokeTest, getSourceVertexForNullGraphAndInvalidEdge)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getSourceVertexForNullGraphAndNonExistingEdge)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(NULL, (xme_hal_graph_edgeId_t)42));
}

// xme_hal_graph_getSinkVertex()

TEST_F(GraphSmokeTest, getSinkVertexForNullGraphAndInvalidEdge)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphSmokeTest, getSinkVertexForNullGraphAndNonExistingEdge)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(NULL, (xme_hal_graph_edgeId_t)42));
}

// xme_hal_graph_getVertexCompareCallback()

TEST_F(GraphSmokeTest, getVertexCompareCallbackWithNullGraph)
{
    EXPECT_EQ(NULL, xme_hal_graph_getVertexCompareCallback(NULL));
}

// xme_hal_graph_setVertexCompareCallback()

TEST_F(GraphSmokeTest, setVertexCompareCallbackWithNullGraphAndNullArgument)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_setVertexCompareCallback(NULL, NULL));
}

TEST_F(GraphSmokeTest, setVertexCompareCallbackWithNullGraphAndArbitraryArgument)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_setVertexCompareCallback(NULL, (xme_hal_graph_vertexCompareCallback_t)42));
}

// xme_hal_graph_getEdgeCompareCallback()

TEST_F(GraphSmokeTest, getEdgeCompareCallbackWithNullGraph)
{
    EXPECT_EQ(NULL, xme_hal_graph_getEdgeCompareCallback(NULL));
}

// xme_hal_graph_setEdgeCompareCallback()

TEST_F(GraphSmokeTest, setEdgeCompareCallbackWithNullGraphAndNullArgument)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_setEdgeCompareCallback(NULL, NULL));
}

TEST_F(GraphSmokeTest, setEdgeCompareCallbackWithNullGraphAndArbitraryArgument)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_setEdgeCompareCallback(NULL, (xme_hal_graph_edgeCompareCallback_t)42));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
