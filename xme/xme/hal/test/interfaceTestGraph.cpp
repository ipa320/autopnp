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
 * $Id: interfaceTestGraph.cpp 5393 2013-10-04 17:48:33Z geisinger $
 */

/**
 * \file
 *         Graph interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/graph.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class GraphInterfaceTest: public ::testing::Test
{
protected:
    GraphInterfaceTest()
    : vi1(XME_HAL_GRAPH_INVALID_VERTEX_ID)
    , vi2(XME_HAL_GRAPH_INVALID_VERTEX_ID)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&g));
    }

    virtual ~GraphInterfaceTest()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));
    }

    static int
    alwaysSameVertexCompareCallback
    (
        void* vertexData1,
        void* vertexData2
    )
    {
        XME_UNUSED_PARAMETER(vertexData1);
        XME_UNUSED_PARAMETER(vertexData2);

        return 0;
    }

    static int
    alwaysDifferentVertexCompareCallback
    (
        void* vertexData1,
        void* vertexData2
    )
    {
        XME_UNUSED_PARAMETER(vertexData1);
        XME_UNUSED_PARAMETER(vertexData2);

        return 1;
    }

    xme_hal_graph_graph_t g;
    xme_hal_graph_vertexId_t vi1;
    xme_hal_graph_vertexId_t vi2;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     GraphInterfaceTest                                                     //
//----------------------------------------------------------------------------//

// xme_hal_graph_init()

TEST_F(GraphInterfaceTest, initInitialized)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&g));
}

TEST_F(GraphInterfaceTest, initFinalized)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&g));
}

// xme_hal_graph_fini()

TEST_F(GraphInterfaceTest, finiInitialized)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));
}

TEST_F(GraphInterfaceTest, finiInitializedWithInUseVertexIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));
}

TEST_F(GraphInterfaceTest, finiInitializedWithInUseOutgoingEdgeIterator)
{
    xme_hal_graph_vertexId_t v = xme_hal_graph_addVertex(&g, NULL);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, v));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));
}

TEST_F(GraphInterfaceTest, finiInitializedWithInUseIncomingEdgeIterator)
{
    xme_hal_graph_vertexId_t v = xme_hal_graph_addVertex(&g, NULL);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, v);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, v));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));
}

// xme_hal_graph_getVertexCount()

TEST_F(GraphInterfaceTest, countVerticesInEmptyGraph)
{
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
}

// xme_hal_graph_getEdgeCount()

TEST_F(GraphInterfaceTest, countEdgesInEmptyGraph)
{
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_clone()

TEST_F(GraphInterfaceTest, cloneEmptyGraphToNullGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_clone(&g, NULL));
}

TEST_F(GraphInterfaceTest, cloneEmptyGraphToValidGraph)
{
    xme_hal_graph_graph_t cloned;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&cloned));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clone(&g, &cloned));

    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&cloned));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&cloned));
}

// xme_hal_graph_clear()

TEST_F(GraphInterfaceTest, clearEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&g));
}

// xme_hal_graph_addVertex()

TEST_F(GraphInterfaceTest, addOneVertexWithNullData)
{
    vi1 = xme_hal_graph_addVertex(&g, NULL);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, addTwoVerticesWithNullData)
{
    vi1 = xme_hal_graph_addVertex(&g, NULL);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    vi2 = xme_hal_graph_addVertex(&g, NULL);
    ASSERT_NE(vi2, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));

    EXPECT_NE(vi1, vi2);
}

TEST_F(GraphInterfaceTest, addTwoVerticesWithSomeSameData)
{
    vi1 = xme_hal_graph_addVertex(&g, (void*)42);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    vi2 = xme_hal_graph_addVertex(&g, (void*)42);
    ASSERT_NE(vi2, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));

    EXPECT_NE(vi1, vi2);
}

TEST_F(GraphInterfaceTest, addTwoVerticesWithSomeDifferentData)
{
    vi1 = xme_hal_graph_addVertex(&g, (void*)42);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    vi2 = xme_hal_graph_addVertex(&g, (void*)43);
    ASSERT_NE(vi2, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));

    EXPECT_NE(vi1, vi2);
}

// xme_hal_graph_getNextVertexWithDataComparison()

TEST_F(GraphInterfaceTest, getNextVertexWithDataComparisonWithNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, getNextVertexWithDataComparisonWithNullDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, NULL, (xme_hal_graph_vertexId_t) 42));
}

TEST_F(GraphInterfaceTest, getNextVertexWithDataComparisonWithNonNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, (void*) 42, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, getNextVertexWithDataComparisonWithNonNullDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, (void*) 42, (xme_hal_graph_vertexId_t) 43));
}

// xme_hal_graph_removeVertex()

TEST_F(GraphInterfaceTest, removeInvalidVertexFromEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertex(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, removeNonExistingVertexFromEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertex(&g, (xme_hal_graph_vertexId_t)42));
}

TEST_F(GraphInterfaceTest, removeInvalidVertexFromNonEmptyGraph)
{
    vi1 = xme_hal_graph_addVertex(&g, NULL);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertex(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeNonExistingVertexFromNonEmptyGraph)
{
    vi1 = xme_hal_graph_addVertex(&g, NULL);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertex(&g, (xme_hal_graph_vertexId_t)42));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeExistingVertexFromNonEmptyGraph)
{
    vi1 = xme_hal_graph_addVertex(&g, NULL);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
}

// xme_hal_graph_removeVertexWithDataComparison()

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithNullVertexDataFromEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, NULL, true));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithNonExistingVertexDataFromEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, (void*)42, true));
}

// -----

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithNullVertexDataFromNonEmptyGraph)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, NULL, true));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithNonExistingVertexDataFromNonEmptyGraph)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, (void*)42, true));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithExistingVertexFromNonEmptyGraph)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertexWithDataComparison(&g, &i, true));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, &i, true));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithExistingVertexFromNonEmptyGraphIncremental)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    vi2 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi2, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertexWithDataComparison(&g, &i, false));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertexWithDataComparison(&g, &i, false));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, &i, false));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
}

// -----

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithNullVertexDataFromNonEmptyGraphWithAlwaysSameComparisonCallback)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &alwaysSameVertexCompareCallback));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertexWithDataComparison(&g, NULL, true));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithNonExistingVertexDataFromNonEmptyGraphWithAlwaysSameComparisonCallback)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &alwaysSameVertexCompareCallback));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertexWithDataComparison(&g, (void*)42, true));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithExistingVertexFromNonEmptyGraphWithAlwaysSameComparisonCallback)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &alwaysSameVertexCompareCallback));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertexWithDataComparison(&g, &i, true));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, &i, true));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithExistingVertexFromNonEmptyGraphIncrementalWithAlwaysSameComparisonCallback)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    vi2 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi2, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &alwaysSameVertexCompareCallback));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertexWithDataComparison(&g, &i, false));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertexWithDataComparison(&g, &i, false));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, &i, false));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
}

// -----

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithNullVertexDataFromNonEmptyGraphWithAlwaysDifferentComparisonCallback)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &alwaysDifferentVertexCompareCallback));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, NULL, true));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithNonExistingVertexDataFromNonEmptyGraphWithAlwaysDifferentComparisonCallback)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &alwaysDifferentVertexCompareCallback));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, (void*)42, true));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithExistingVertexFromNonEmptyGraphWithAlwaysDifferentComparisonCallback)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &alwaysDifferentVertexCompareCallback));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, &i, true));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, &i, true));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
}

TEST_F(GraphInterfaceTest, removeVertexWithDataComparisonWithExistingVertexFromNonEmptyGraphIncrementalWithAlwaysDifferentComparisonCallback)
{
    int i = 0;

    vi1 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi1, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));

    vi2 = xme_hal_graph_addVertex(&g, &i);
    ASSERT_NE(vi2, XME_HAL_GRAPH_INVALID_VERTEX_ID);
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &alwaysDifferentVertexCompareCallback));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, &i, false));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, &i, false));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeVertexWithDataComparison(&g, &i, false));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
}

// xme_hal_graph_addEdge()

TEST_F(GraphInterfaceTest, addEdgeWithInvalidSameSourceAndDestinationToEmptyGraph)
{
    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, addEdgeWithNonExistingSameSourceAndDestinationToEmptyGraph)
{
    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)42, NULL));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, addEdgeWithInvalidSourceAndNonExistingDestinationToEmptyGraph)
{
    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, NULL));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, addEdgeWithNonExistingSourceAndInvalidDestinationToEmptyGraph)
{
    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, addEdgeWithNonExistingDifferentSourceAndDestinationToEmptyGraph)
{
    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, NULL));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_getNextEdgeWithDataComparison()

TEST_F(GraphInterfaceTest, getNextEdgeWithDataComparisonWithInvalidSourceAndInvalidSinkAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeWithDataComparisonWithInvalidSourceAndInvalidSinkAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, (void*) 42, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeWithDataComparisonWithInvalidSourceAndInvalidSinkAndNullDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, NULL, (xme_hal_graph_edgeId_t)42));
}

TEST_F(GraphInterfaceTest, getNextEdgeWithDataComparisonWithInvalidSourceAndInvalidSinkAndArbitraryDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, (void*) 42, (xme_hal_graph_edgeId_t)43));
}

// xme_hal_graph_getNextEdgeBetweenWithDataComparison()

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithInvalidSourceAndInvalidSinkAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithArbitrarySourceAndInvalidSinkAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithInvalidSourceAndArbitrarySinkAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithArbitrarySourceAndArbitrarySinkAndNullDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithInvalidSourceAndInvalidSinkAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 42, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithArbitrarySourceAndInvalidSinkAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 43, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithInvalidSourceAndArbitrarySinkAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, (void*) 43, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithArbitrarySourceAndArbitrarySinkAndArbitraryDataAndInvalidStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, (void*) 44, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithInvalidSourceAndInvalidSinkAndNullDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, (xme_hal_graph_edgeId_t)42));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithArbitrarySourceAndInvalidSinkAndNullDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL, (xme_hal_graph_edgeId_t)43));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithInvalidSourceAndArbitrarySinkAndNullDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, NULL, (xme_hal_graph_edgeId_t)43));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithArbitrarySourceAndArbitrarySinkAndNullDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, NULL, (xme_hal_graph_edgeId_t)44));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithInvalidSourceAndInvalidSinkAndArbitraryDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 42, (xme_hal_graph_edgeId_t)43));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithArbitrarySourceAndInvalidSinkAndArbitraryDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, (void*) 43, (xme_hal_graph_edgeId_t)44));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithInvalidSourceAndArbitrarySinkAndArbitraryDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, (void*) 43, (xme_hal_graph_edgeId_t)44));
}

TEST_F(GraphInterfaceTest, getNextEdgeBetweenWithDataComparisonWithArbitrarySourceAndArbitrarySinkAndArbitraryDataAndNonExistingStartAfter)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, (void*) 44, (xme_hal_graph_edgeId_t)45));
}

// xme_hal_graph_removeEdge()

TEST_F(GraphInterfaceTest, removeEdgeWithInvalidEdgeFromEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_removeEdge(&g, XME_HAL_GRAPH_INVALID_EDGE_ID));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, removeEdgeWithNonExistingEdgeFromEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdge(&g, (xme_hal_graph_edgeId_t)42));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_removeEdgeWithDataComparison()

TEST_F(GraphInterfaceTest, removeEdgeWithDataComparisonWithNullEdgeDataFromEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeWithDataComparison(&g, NULL, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, removeEdgeWithDataComparisonWithNullArbitraryDataFromEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeWithDataComparison(&g, (void*) 42, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_removeEdgeBetween()

TEST_F(GraphInterfaceTest, removeEdgeBetweenWithInvalidSameSourceAndDestinationFromEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_HAL_GRAPH_INVALID_VERTEX_ID, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, removeEdgeBetweenWithNonExistingSameSourceAndDestinationFromEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)42, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, removeEdgeBetweenWithInvalidSourceAndNonExistingDestinationFromEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, (xme_hal_graph_vertexId_t)42, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, removeEdgeBetweenWithNonExistingSourceAndInvalidDestinationFromEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, (xme_hal_graph_vertexId_t)42, XME_HAL_GRAPH_INVALID_VERTEX_ID, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphInterfaceTest, removeEdgeBetweenWithNonExistingDifferentSourceAndDestinationFromEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_initVertexIterator()

TEST_F(GraphInterfaceTest, initVertexIteratorWithEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&g));
}

// xme_hal_graph_finiVertexIterator()

TEST_F(GraphInterfaceTest, finiVertexIteratorWithEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiVertexIterator(&g));
}

// xme_hal_graph_hasNextVertex()

TEST_F(GraphInterfaceTest, hasNextVertexWithoutInitVertexIteratorAndEmptyGraph)
{
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphInterfaceTest, hasNextVertexWithInitVertexIteratorAndEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&g));
}

// xme_hal_graph_nextVertex()

TEST_F(GraphInterfaceTest, nextVertexWithoutInitVertexIteratorAndEmptyGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphInterfaceTest, nextVertexWithInitVertexIteratorAndEmptyGraph)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&g));
}

// xme_hal_graph_initOutgoingEdgeIterator()

TEST_F(GraphInterfaceTest, initEdgeIteratorWithEmptyGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, initEdgeIteratorWithEmptyGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_finiOutgoingEdgeIterator()

TEST_F(GraphInterfaceTest, xme_hal_graph_finiOutgoingEdgeIteratorWithEmptyGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, xme_hal_graph_finiOutgoingEdgeIteratorWithEmptyGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(&g, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_hasNextOutgoingEdge()

TEST_F(GraphInterfaceTest, hasNextEdgeWithoutInitEdgeIteratorAndInvalidVertexAndEmptyGraph)
{
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, hasNextEdgeWithoutInitEdgeIteratorAndNonExistingVertexAndEmptyGraph)
{
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, (xme_hal_graph_vertexId_t)42));
}

TEST_F(GraphInterfaceTest, hasNextEdgeWithInitEdgeIteratorAndInvalidVertexAndEmptyGraph)
{
    // TODO: What is supposed to happen if the two vertex identifiers specified to these two functions differ?
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, hasNextEdgeWithInitEdgeIteratorAndNonExistingVertexAndEmptyGraph)
{
    // TODO: What is supposed to happen if the two vertex identifiers specified to these two functions differ?
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, (xme_hal_graph_vertexId_t)42));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_nextOutgoingEdge()

TEST_F(GraphInterfaceTest, nextEdgeWithoutInitEdgeIteratorAndInvalidVertexAndEmptyGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, nextEdgeWithoutInitEdgeIteratorAndNonExistingVertexAndEmptyGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, (xme_hal_graph_vertexId_t)42));
}

TEST_F(GraphInterfaceTest, nextEdgeWithInitEdgeIteratorAndInvalidVertexAndEmptyGraph)
{
    // TODO: What is supposed to happen if the two vertex identifiers specified to these two functions differ?
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, nextEdgeWithInitEdgeIteratorAndNonExistingVertexAndEmptyGraph)
{
    // TODO: What is supposed to happen if the two vertex identifiers specified to these two functions differ?
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, (xme_hal_graph_vertexId_t)42));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_initIncomingEdgeIterator()

TEST_F(GraphInterfaceTest, initIncomingEdgeIteratorWithEmptyGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, initIncomingEdgeIteratorWithEmptyGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_finiIncomingEdgeIterator()

TEST_F(GraphInterfaceTest, finiIncomingEdgeIteratorWithEmptyGraphAndInvalidVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, finiIncomingEdgeIteratorWithEmptyGraphAndNonExistingVertex)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(&g, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_hasNextIncomingEdge()

TEST_F(GraphInterfaceTest, hasNextIncomingEdgeWithoutInitEdgeIteratorAndInvalidVertexAndEmptyGraph)
{
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, hasNextIncomingEdgeWithoutInitEdgeIteratorAndNonExistingVertexAndEmptyGraph)
{
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, (xme_hal_graph_vertexId_t)42));
}

TEST_F(GraphInterfaceTest, hasNextIncomingEdgeWithInitEdgeIteratorAndInvalidVertexAndEmptyGraph)
{
    // TODO: What is supposed to happen if the two vertex identifiers specified to these two functions differ?
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, hasNextIncomingEdgeWithInitEdgeIteratorAndNonExistingVertexAndEmptyGraph)
{
    // TODO: What is supposed to happen if the two vertex identifiers specified to these two functions differ?
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, (xme_hal_graph_vertexId_t)42));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_nextIncomingEdge()

TEST_F(GraphInterfaceTest, nextIncomingEdgeWithoutInitEdgeIteratorAndInvalidVertexAndEmptyGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, nextIncomingEdgeWithoutInitEdgeIteratorAndNonExistingVertexAndEmptyGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&g, (xme_hal_graph_vertexId_t)42));
}

TEST_F(GraphInterfaceTest, nextIncomingEdgeWithInitEdgeIteratorAndInvalidVertexAndEmptyGraph)
{
    // TODO: What is supposed to happen if the two vertex identifiers specified to these two functions differ?
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphInterfaceTest, nextIncomingEdgeWithInitEdgeIteratorAndNonExistingVertexAndEmptyGraph)
{
    // TODO: What is supposed to happen if the two vertex identifiers specified to these two functions differ?
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, (xme_hal_graph_vertexId_t)42));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&g, (xme_hal_graph_vertexId_t)42));
}

// xme_hal_graph_getVertexData()

TEST_F(GraphInterfaceTest, getVertexDataWithInvalidVertexAndNullResultAndEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getVertexData(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, NULL));
}

TEST_F(GraphInterfaceTest, getVertexDataWithInvalidVertexAndNonNullResultAndEmptyGraph)
{
    int i;
    void* p = &i;

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getVertexData(&g, XME_HAL_GRAPH_INVALID_VERTEX_ID, &p));
    EXPECT_EQ(p, &i);
}

TEST_F(GraphInterfaceTest, getVertexDataWithNonExistingVertexAndNullResultAndEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getVertexData(&g, (xme_hal_graph_vertexId_t)42, NULL));
}

TEST_F(GraphInterfaceTest, getVertexDataWithNonExistingVertexAndNonNullResultAndEmptyGraph)
{
    int i;
    void* p = &i;

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getVertexData(&g, (xme_hal_graph_vertexId_t)42, &p));
    EXPECT_EQ(p, &i);
}

// xme_hal_graph_getEdgeData()

TEST_F(GraphInterfaceTest, getVertexDataWithInvalidEdgeAndNullResultAndEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, XME_HAL_GRAPH_INVALID_EDGE_ID, NULL));
}

TEST_F(GraphInterfaceTest, getVertexDataWithInvalidEdgeAndNonNullResultAndEmptyGraph)
{
    int i;
    void* p = &i;

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, XME_HAL_GRAPH_INVALID_EDGE_ID, &p));
    EXPECT_EQ(p, &i);
}

TEST_F(GraphInterfaceTest, getVertexDataWithNonExistingEdgeAndNullResultAndEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, (xme_hal_graph_edgeId_t)42, NULL));
}

TEST_F(GraphInterfaceTest, getVertexDataWithNonExistingEdgeAndNonNullResultAndEmptyGraph)
{
    int i;
    void* p = &i;

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, (xme_hal_graph_edgeId_t)42, &p));
    EXPECT_EQ(p, &i);
}

// xme_hal_graph_getSourceVertex()

TEST_F(GraphInterfaceTest, getSourceVertexWithInvalidEdgeAndEmptyGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getSourceVertexWithNonExistingEdgeAndEmptyGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, (xme_hal_graph_edgeId_t)42));
}

// xme_hal_graph_getSinkVertex()

TEST_F(GraphInterfaceTest, getSinkVertexWithInvalidEdgeAndEmptyGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphInterfaceTest, getSinkVertexWithNonExistingEdgeAndEmptyGraph)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, (xme_hal_graph_edgeId_t)42));
}

// xme_hal_graph_getVertexCompareCallback()

TEST_F(GraphInterfaceTest, getVertexCompareCallback)
{
    EXPECT_EQ(NULL, xme_hal_graph_getVertexCompareCallback(NULL));
}

// xme_hal_graph_setVertexCompareCallback()

TEST_F(GraphInterfaceTest, setVertexCompareCallbackWithNullArgument)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, NULL));
}

TEST_F(GraphInterfaceTest, setVertexCompareCallbackWithArbitraryArgument)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, (xme_hal_graph_vertexCompareCallback_t)42));
}

// xme_hal_graph_getEdgeCompareCallback()

TEST_F(GraphInterfaceTest, getEdgeCompareCallback)
{
    EXPECT_EQ(NULL, xme_hal_graph_getEdgeCompareCallback(NULL));
}

// xme_hal_graph_setEdgeCompareCallback()

TEST_F(GraphInterfaceTest, setEdgeCompareCallbackWithNullArgument)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(&g, NULL));
}

TEST_F(GraphInterfaceTest, setEdgeCompareCallbackWithArbitraryArgument)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(&g, (xme_hal_graph_edgeCompareCallback_t)42));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
