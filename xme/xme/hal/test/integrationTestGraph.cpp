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
 * $Id: integrationTestGraph.cpp 6584 2014-01-31 16:50:28Z geisinger $
 */

/**
 * \file
 *         Graph integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
// Ensure that stdint.h defines limit macros
#define __STDC_LIMIT_MACROS

#include <gtest/gtest.h>

#include "xme/hal/include/graph.h"

#include <stdint.h>

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class GraphIntegrationTest: public ::testing::Test
{
protected:
    GraphIntegrationTest()
    : d1(1)
    , d2(2)
    , d3(3)
    , vd1(&d1)
    , vd2(&d2)
    , vd3(&d3)
    , ed1(&d1)
    , ed2(&d2)
    , ed3(&d3)
    , vi1(XME_HAL_GRAPH_INVALID_VERTEX_ID)
    , vi2(XME_HAL_GRAPH_INVALID_VERTEX_ID)
    , vi3(XME_HAL_GRAPH_INVALID_VERTEX_ID)
    , ei1(XME_HAL_GRAPH_INVALID_EDGE_ID)
    , ei2(XME_HAL_GRAPH_INVALID_EDGE_ID)
    , ei3(XME_HAL_GRAPH_INVALID_EDGE_ID)
    {
        // This constructor builds the following graph:
        //
        //       e3       e2
        //   v1 ----> v3 ----> v2
        //           / ^
        //           \_/ e1 (self-edge)

        // For the tests to work as expected,
        // the data need to be different
        EXPECT_NE(d1, d2);
        EXPECT_NE(d1, d3);
        EXPECT_NE(d2, d3);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&g));

        vi1 = xme_hal_graph_addVertex(&g, vd1);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        vi2 = xme_hal_graph_addVertex(&g, vd2);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        vi3 = xme_hal_graph_addVertex(&g, vd3);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);

        EXPECT_NE(vi1, vi2);
        EXPECT_NE(vi2, vi3);
        EXPECT_NE(vi3, vi1);

        ei1 = xme_hal_graph_addEdge(&g, vi3, vi3, ed1);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

        ei2 = xme_hal_graph_addEdge(&g, vi3, vi2, ed2);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);

        ei3 = xme_hal_graph_addEdge(&g, vi1, vi3, ed3);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei3);

        EXPECT_NE(ei1, ei2);
        EXPECT_NE(ei1, ei3);
        EXPECT_NE(ei2, ei3);
    }

    virtual ~GraphIntegrationTest()
    {
        // ensure all iterators are finalized
        xme_hal_graph_finiVertexIterator(&g);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));
    }

    static int
    testCompareVertex
    (
        void* vertexData1,
        void* vertexData2
    )
    {
        int* data1;
        int* data2;

        data1 = (int*) vertexData1;
        data2 = (int*) vertexData2;

        if (NULL == data1)
        {
            if (NULL == data2)
            {
                return 0;
            }
            else
            {
                return -1;
            }
        }
        else
        {
            if (NULL == data2)
            {
                return 1;
            }
            else
            {
                return (*data1 - *data2);
            }
        }
    }

    static int
    testCompareEdge
    (
        void* edgeData1,
        void* edgeData2
    )
    {
        int* data1;
        int* data2;

        data1 = (int*) edgeData1;
        data2 = (int*) edgeData2;

        if (NULL == data1)
        {
            if (NULL == data2)
            {
                return 0;
            }
            else
            {
                return -1;
            }
        }
        else
        {
            if (NULL == data2)
            {
                return 1;
            }
            else
            {
                return (*data1 - *data2);
            }
        }
    }

    xme_hal_graph_graph_t g;

    int d1;
    int d2;
    int d3;

    int* vd1;
    int* vd2;
    int* vd3;

    int* ed1;
    int* ed2;
    int* ed3;

    xme_hal_graph_vertexId_t vi1;
    xme_hal_graph_vertexId_t vi2;
    xme_hal_graph_vertexId_t vi3;

    xme_hal_graph_edgeId_t ei1;
    xme_hal_graph_edgeId_t ei2;
    xme_hal_graph_edgeId_t ei3;
};

class GraphIntegrationMultigraphTest: public GraphIntegrationTest
{
protected:
    GraphIntegrationMultigraphTest()
    : d4(4)
    , d5(5)
    , d6(6)
    , ed4(&d4)
    , ed5(&d5)
    , ed6(&d6)
    , ei4(XME_HAL_GRAPH_INVALID_EDGE_ID)
    , ei5(XME_HAL_GRAPH_INVALID_EDGE_ID)
    , ei6(XME_HAL_GRAPH_INVALID_EDGE_ID)
    {
        // This constructor extends the graph from base class
        // GraphIntegrationTest as follows:
        //
        //      e3, e6     e5
        //      ,--->    <---.
        //   v1 ----> v3 ----> v2
        //          // ^^  e2
        //         | \_/ \ e1, e4 (self-edges)
        //          \____/

        ei4 = xme_hal_graph_addEdge(&g, vi3, vi3, ed4);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);

        ei5 = xme_hal_graph_addEdge(&g, vi2, vi3, ed5);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei5);

        ei6 = xme_hal_graph_addEdge(&g, vi1, vi3, ed6);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei6);

        EXPECT_NE(ei1, ei4);
        EXPECT_NE(ei1, ei5);
        EXPECT_NE(ei1, ei6);
        EXPECT_NE(ei2, ei4);
        EXPECT_NE(ei2, ei5);
        EXPECT_NE(ei2, ei6);
        EXPECT_NE(ei3, ei4);
        EXPECT_NE(ei3, ei5);
        EXPECT_NE(ei3, ei6);
        EXPECT_NE(ei4, ei5);
        EXPECT_NE(ei4, ei6);
        EXPECT_NE(ei5, ei6);
    }

    int d4;
    int d5;
    int d6;

    int* ed4;
    int* ed5;
    int* ed6;

    xme_hal_graph_edgeId_t ei4;
    xme_hal_graph_edgeId_t ei5;
    xme_hal_graph_edgeId_t ei6;
};

class GraphIntegrationIncomingEdgesIteratorTest: public ::testing::Test
{
protected:
    GraphIntegrationIncomingEdgesIteratorTest()
    : d1(1)
    , d2(2)
    , d3(3)
    , d4(4)
    , vd1(&d1)
    , vd2(&d2)
    , vd3(&d3)
    , vd4(&d4)
    , ed1(&d1)
    , ed2(&d2)
    , ed3(&d3)
    , vi1(XME_HAL_GRAPH_INVALID_VERTEX_ID)
    , vi2(XME_HAL_GRAPH_INVALID_VERTEX_ID)
    , vi3(XME_HAL_GRAPH_INVALID_VERTEX_ID)
    , vi4(XME_HAL_GRAPH_INVALID_VERTEX_ID)
    , ei1(XME_HAL_GRAPH_INVALID_EDGE_ID)
    , ei2(XME_HAL_GRAPH_INVALID_EDGE_ID)
    , ei3(XME_HAL_GRAPH_INVALID_EDGE_ID)
    {
        // This constructor builds the following graph:
        //                  
        //   v4             
        //    | e3          
        //    v   e1        
        //   v1 <---- v2    
        //    ^             
        //    | e2          
        //   v3             
        //                  

        // For the tests to work as expected,
        // the data need to be different
        EXPECT_NE(d1, d2);
        EXPECT_NE(d1, d3);
        EXPECT_NE(d1, d4);
        EXPECT_NE(d2, d3);
        EXPECT_NE(d2, d4);
        EXPECT_NE(d3, d4);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&g));

        vi1 = xme_hal_graph_addVertex(&g, vd1);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi1);

        vi2 = xme_hal_graph_addVertex(&g, vd2);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi2);

        vi3 = xme_hal_graph_addVertex(&g, vd3);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi3);

        vi4 = xme_hal_graph_addVertex(&g, vd4);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

        EXPECT_NE(vi1, vi2);
        EXPECT_NE(vi2, vi3);
        EXPECT_NE(vi3, vi1);

        ei1 = xme_hal_graph_addEdge(&g, vi2, vi1, ed1);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);

        ei2 = xme_hal_graph_addEdge(&g, vi3, vi1, ed2);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);

        ei3 = xme_hal_graph_addEdge(&g, vi3, vi1, ed3);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei3);

        EXPECT_NE(ei1, ei2);
        EXPECT_NE(ei1, ei3);
        EXPECT_NE(ei2, ei3);
    }

    virtual ~GraphIntegrationIncomingEdgesIteratorTest()
    {
        // ensure all iterators are finalized
        xme_hal_graph_finiVertexIterator(&g);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));
    }

    static int
    testCompareVertex
    (
        void* vertexData1,
        void* vertexData2
    )
    {
        int* data1;
        int* data2;

        data1 = (int*) vertexData1;
        data2 = (int*) vertexData2;

        if (NULL == data1)
        {
            if (NULL == data2)
            {
                return 0;
            }
            else
            {
                return -1;
            }
        }
        else
        {
            if (NULL == data2)
            {
                return 1;
            }
            else
            {
                return (*data1 - *data2);
            }
        }
    }

    static int
    testCompareEdge
    (
        void* edgeData1,
        void* edgeData2
    )
    {
        int* data1;
        int* data2;

        data1 = (int*) edgeData1;
        data2 = (int*) edgeData2;

        if (NULL == data1)
        {
            if (NULL == data2)
            {
                return 0;
            }
            else
            {
                return -1;
            }
        }
        else
        {
            if (NULL == data2)
            {
                return 1;
            }
            else
            {
                return (*data1 - *data2);
            }
        }
    }

    xme_hal_graph_graph_t g;

    int d1;
    int d2;
    int d3;
    int d4;

    int* vd1;
    int* vd2;
    int* vd3;
    int* vd4;

    int* ed1;
    int* ed2;
    int* ed3;

    xme_hal_graph_vertexId_t vi1;
    xme_hal_graph_vertexId_t vi2;
    xme_hal_graph_vertexId_t vi3;
    xme_hal_graph_vertexId_t vi4;

    xme_hal_graph_edgeId_t ei1;
    xme_hal_graph_edgeId_t ei2;
    xme_hal_graph_edgeId_t ei3;
};


/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     GraphIntegrationTest                                                   //
//----------------------------------------------------------------------------//

// xme_hal_graph_init()

TEST_F(GraphIntegrationTest, initNonEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&g));

    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_fini()

TEST_F(GraphIntegrationTest, finiNonEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));

    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_getVertexCount()

TEST_F(GraphIntegrationTest, getVertexCount)
{
    EXPECT_EQ(3, xme_hal_graph_getVertexCount(&g));
}

// xme_hal_graph_getEdgeCount()

TEST_F(GraphIntegrationTest, getEdgeCount)
{
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_clone()

TEST_F(GraphIntegrationTest, cloneGraph)
{
    xme_hal_graph_graph_t cloned;
    xme_hal_graph_edgeId_t e;
    void* d = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&cloned));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clone(&g, &cloned));

    // Check counts

    EXPECT_EQ(3, xme_hal_graph_getVertexCount(&cloned));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&cloned));

    // Check vertices

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&cloned));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&cloned));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&cloned));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&cloned, vi1, &d));
    EXPECT_EQ(vd1, d);

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&cloned));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&cloned));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&cloned, vi2, &d));
    EXPECT_EQ(vd2, d);

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&cloned));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&cloned));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&cloned, vi3, &d));
    EXPECT_EQ(vd3, d);

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&cloned));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&cloned));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&cloned));

    // Check outgoing edges of V1

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&cloned, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&cloned, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei3, &d));
    EXPECT_EQ(ed3, d);

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&cloned, vi1));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&cloned, vi1));

    // Check outgoing edges of V2

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&cloned, vi2));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&cloned, vi2));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&cloned, vi2));

    // Check outgoing edges of V3

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&cloned, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei1, &d));
    EXPECT_EQ(ed1, d);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei2, &d));
    EXPECT_EQ(ed2, d);

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&cloned, vi3));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&cloned, vi3));

    // Check incoming edges of V1

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&cloned, vi1));

    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&cloned, vi1));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&cloned, vi1));

    // Check incoming edges of V2

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&cloned, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi2));
    EXPECT_EQ(ei2, xme_hal_graph_nextIncomingEdge(&cloned, vi2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei2, &d));
    EXPECT_EQ(ed2, d);

    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&cloned, vi2));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&cloned, vi2));

    // Check incoming edges of V3

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&cloned, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi3));
    e = xme_hal_graph_nextIncomingEdge(&cloned, vi3);
    ASSERT_TRUE(ei3 == e || ei1 == e);

    // Order of edge iteration is not defined. Hence, we consider both possible cases here.
    if (ei3 == e)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei3, &d));
        EXPECT_EQ(ed3, d);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi3));
        EXPECT_EQ(ei1, xme_hal_graph_nextIncomingEdge(&cloned, vi3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei1, &d));
        EXPECT_EQ(ed1, d);
    }
    else // if (ei1 == e)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei1, &d));
        EXPECT_EQ(ed1, d);

        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi3));
        EXPECT_EQ(ei1, xme_hal_graph_nextIncomingEdge(&cloned, vi3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei3, &d));
        EXPECT_EQ(ed3, d);
    }

    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&cloned, vi3));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&cloned, vi3));

    // Done

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&cloned));
}

// xme_hal_graph_clear()

TEST_F(GraphIntegrationTest, clearGraph)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&g));

    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&g));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi1));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi2));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi3));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(&g, vi1));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi2));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(&g, vi2));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi3));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(&g, vi3));
}

// xme_hal_graph_getNextVertexWithDataComparison()

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, NULL, XME_HAL_GRAPH_INVALID_VERTEX_ID));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithNullDataAndStartAfterV1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, NULL, vi1));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithNullDataAndStartAfterV2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, NULL, vi2));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithNullDataAndStartAfterV3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, NULL, vi3));
}

// -----

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV1DataAndStartAfterInvalid)
{
    ASSERT_EQ(vi1, xme_hal_graph_getNextVertexWithDataComparison(&g, vd1, XME_HAL_GRAPH_INVALID_VERTEX_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd1, vi1));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV1DataAndStartAfterV1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd1, vi1));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV1DataAndStartAfterV2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd1, vi2));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV1DataAndStartAfterV3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd1, vi3));
}

// -----

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV2DataAndStartAfterInvalid)
{
    ASSERT_EQ(vi2, xme_hal_graph_getNextVertexWithDataComparison(&g, vd2, XME_HAL_GRAPH_INVALID_VERTEX_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd2, vi2));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV2DataAndStartAfterV1)
{
    ASSERT_EQ(vi2, xme_hal_graph_getNextVertexWithDataComparison(&g, vd2, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd2, vi2));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV2DataAndStartAfterV2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd2, vi2));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV2DataAndStartAfterV3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd2, vi3));
}

// -----

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV3DataAndStartAfterInvalid)
{
    ASSERT_EQ(vi3, xme_hal_graph_getNextVertexWithDataComparison(&g, vd3, XME_HAL_GRAPH_INVALID_VERTEX_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd3, vi3));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV3DataAndStartAfterV1)
{
    ASSERT_EQ(vi3, xme_hal_graph_getNextVertexWithDataComparison(&g, vd3, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd3, vi3));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV3DataAndStartAfterV2)
{
    ASSERT_EQ(vi3, xme_hal_graph_getNextVertexWithDataComparison(&g, vd3, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd3, vi3));
}

TEST_F(GraphIntegrationTest, getNextVertexWithDataComparisonWithV3DataAndStartAfterV3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getNextVertexWithDataComparison(&g, vd3, vi3));
}

// xme_hal_graph_removeVertex()

TEST_F(GraphIntegrationTest, removeVertexThatIsSourceAndDestinationOfAnEdge)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    ei1 = xme_hal_graph_addEdge(&g, vi3, vi3, ed2);
    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllVerticesInOrderV1V2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v1->v3 (e3) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v2 (e2) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v3 (e1) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
}

TEST_F(GraphIntegrationTest, removeAllVerticesInOrderV1V3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v1->v3 (e3) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edges v3->v3 (e1) and v3->v2 (e2) are not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllVerticesInOrderV2V1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v2 (e2) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v1->v3 (e3) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v3 (e1) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
}

TEST_F(GraphIntegrationTest, removeAllVerticesInOrderV2V3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v2 (e2) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edges v3->v3 (e1) and v1->v3 (e3) are not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllVerticesInOrderV3V1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edges v3->v3 (e1), v3->v2 (e2) and v1->v3 (e3) are not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllVerticesInOrderV3V2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edges v3->v3 (e1), v3->v2 (e2) and v1->v3 (e3) are not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_addEdge()

TEST_F(GraphIntegrationTest, addEdgeWithNonExistingSameSourceAndDestination)
{
    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)42, ed1));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)42, ed2));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, addEdgeWithNonExistingDifferentSourceAndDestination)
{
    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, ed1));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, (xme_hal_graph_vertexId_t)42, (xme_hal_graph_vertexId_t)43, ed2));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, addEdgeWithExistingSameSourceAndDestination)
{
    xme_hal_graph_edgeId_t ei4, ei5;

    ei4 = xme_hal_graph_addEdge(&g, vi1, vi1, ed1);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ei5 = xme_hal_graph_addEdge(&g, vi1, vi1, ed1);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei5);
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    EXPECT_NE(ei1, ei4);
    EXPECT_NE(ei1, ei5);

    EXPECT_NE(ei2, ei4);
    EXPECT_NE(ei2, ei5);

    EXPECT_NE(ei3, ei4);
    EXPECT_NE(ei3, ei5);

    EXPECT_NE(ei4, ei5);
}

TEST_F(GraphIntegrationTest, addEdgeWithExistingDifferentSourceAndDestination)
{
    xme_hal_graph_edgeId_t ei4, ei5, ei6, ei7;

    ei4 = xme_hal_graph_addEdge(&g, vi1, vi2, ed1);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ei5 = xme_hal_graph_addEdge(&g, vi1, vi2, ed1);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei5);
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ei6 = xme_hal_graph_addEdge(&g, vi2, vi1, ed1);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei6);
    EXPECT_EQ(6, xme_hal_graph_getEdgeCount(&g));

    ei7 = xme_hal_graph_addEdge(&g, vi2, vi1, ed1);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);
    EXPECT_EQ(7, xme_hal_graph_getEdgeCount(&g));

    EXPECT_NE(ei1, ei4);
    EXPECT_NE(ei1, ei5);
    EXPECT_NE(ei1, ei6);
    EXPECT_NE(ei1, ei7);

    EXPECT_NE(ei2, ei4);
    EXPECT_NE(ei2, ei5);
    EXPECT_NE(ei2, ei6);
    EXPECT_NE(ei2, ei7);

    EXPECT_NE(ei3, ei4);
    EXPECT_NE(ei3, ei5);
    EXPECT_NE(ei3, ei6);
    EXPECT_NE(ei3, ei7);

    EXPECT_NE(ei4, ei5);
    EXPECT_NE(ei4, ei6);
    EXPECT_NE(ei4, ei7);

    EXPECT_NE(ei5, ei6);
    EXPECT_NE(ei5, ei7);

    EXPECT_NE(ei6, ei7);
}

// xme_hal_graph_getNextEdgeWithDataComparison()

// null data

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, NULL, ei3));
}

// ed1 data

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE1DataAndStartAfterInvalid)
{
    EXPECT_EQ(ei1, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE1DataAndStartAfterE3)
{
    EXPECT_EQ(ei1, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed1, ei3));
}

// ed2 data

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE2DataAndStartAfterInvalid)
{
    EXPECT_EQ(ei2, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE2DataAndStartAfterE1)
{
    EXPECT_EQ(ei2, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE2DataAndStartAfterE3)
{
    EXPECT_EQ(ei2, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed2, ei3));
}

// ed3 data

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE3DataAndStartAfterInvalid)
{
    EXPECT_EQ(ei3, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeWithDataComparisonWithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed3, ei3));
}

// xme_hal_graph_getNextEdgeBetweenWithDataComparison()

// null data

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, NULL, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, NULL, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, NULL, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, NULL, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, NULL, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, NULL, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, NULL, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, NULL, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithNullDataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithNullDataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, NULL, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithNullDataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, NULL, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithNullDataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, NULL, ei3));
}

// ed1 data

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE1DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE1DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed1, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE1DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE1DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed1, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE1DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE1DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed1, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE1DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE1DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed1, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE1DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE1DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed1, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE1DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE1DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed1, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE1DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE1DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed1, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE1DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE1DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed1, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE1DataAndStartAfterInvalid)
{
    ASSERT_EQ(ei1, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, XME_HAL_GRAPH_INVALID_EDGE_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE1DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE1DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE1DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, ei3));
}

// ed2 data

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE2DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE2DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE2DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed2, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE2DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE2DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE2DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed2, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE2DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE2DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE2DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed2, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE2DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE2DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE2DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed2, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE2DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE2DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE2DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed2, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE2DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE2DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE2DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed2, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE2DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE2DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE2DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed2, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE2DataAndStartAfterInvalid)
{
    ASSERT_EQ(ei2, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE2DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE2DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE2DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed2, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE2DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed2, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE2DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed2, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE2DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed2, ei3));
}

// ed3 data

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE3DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed3, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE3DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed3, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE3DataAndStartAfterInvalid)
{
    ASSERT_EQ(ei3, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, ei3));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE3DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed3, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE3DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed3, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE3DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed3, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE3DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed3, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE3DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed3, ei3));
}

// -----

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE3DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed3, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE3DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed3, ei1));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE3DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed3, ei2));
}

TEST_F(GraphIntegrationTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE3DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed3, ei3));
}

// xme_hal_graph_removeEdge()

TEST_F(GraphIntegrationTest, removeEdgeWithExistingSameSourceAndDestination)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeEdgeWithExistingDifferentSourceAndDestination)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeEdgeWithExistingDifferentSourceAndDestinationReverse)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesInOrderE1E2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesInOrderE1E3E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesInOrderE2E1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesInOrderE2E3E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesInOrderE3E1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesInOrderE3E2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_removeEdgeWithDataComparison()

TEST_F(GraphIntegrationTest, removeEdgeWithDataComparisonWithNullData)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeWithDataComparison(&g, NULL, true));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeEdgeWithDataComparisonWithArbitraryData)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeWithDataComparison(&g, (void*) 42, true));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeEdgeWithDataComparisonWithE1Data)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeWithDataComparison(&g, ed1, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeEdgeWithDataComparisonWithE2Data)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeWithDataComparison(&g, ed2, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeEdgeWithDataComparisonWithE3Data)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeWithDataComparison(&g, ed3, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_removeEdgeBetween()

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithExistingSameSourceAndDestination)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithExistingDifferentSourceAndDestination)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeEdgeWithReverseExistingSourceAndDestination)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi2, vi3, true));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesBetweenInOrderE1E2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesBetweenInOrderE1E3E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, true));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesBetweenInOrderE2E1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesBetweenInOrderE2E3E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, true));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesBetweenInOrderE3E1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeAllEdgesBetweenInOrderE3E2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_removeEdgeBetween() / xme_hal_graph_removeEdge() / xme_hal_graph_addEdge()

TEST_F(GraphIntegrationTest, removeEdgeBetweenAndAddSameEdgeAgain)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ei2 = xme_hal_graph_addEdge(&g, vi3, vi2, ed2);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, removeEdgeAndAddSameEdgeAgain)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    ei2 = xme_hal_graph_addEdge(&g, vi3, vi2, ed2);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei2);
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_removeEdgeBetweenWithDataComparison()

// null data

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, NULL, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, NULL, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithNullData)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, NULL, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, NULL, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, NULL, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, NULL, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, NULL, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithNullData)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, NULL, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithNullData)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, NULL, true));
}

// ed1 data

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, ed1, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, ed1, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, ed1, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, ed1, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, ed1, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, ed1, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, ed1, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, ed1, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithE1Data)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, true));
}

// ed2 data

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, ed2, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, ed2, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, ed2, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, ed2, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, ed2, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, ed2, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, ed2, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithE2Data)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, ed2, true));
}

// ed3 data

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, ed3, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, ed3, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithE3Data)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, ed3, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, ed3, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, ed3, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, ed3, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, ed3, true));
}

TEST_F(GraphIntegrationTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, ed3, true));
}

// xme_hal_graph_initVertexIterator()

TEST_F(GraphIntegrationTest, initVertexIteratorTwice)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_STATUS_UNSUPPORTED, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&g));
}

// xme_hal_graph_finiVertexIterator()

TEST_F(GraphIntegrationTest, finiVertexIteratorWithoutInitVertexIterator)
{
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiVertexIterator(&g));
}

TEST_F(GraphIntegrationTest, finiVertexIteratorWithInitVertexIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&g));
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiVertexIterator(&g));
}

// xme_hal_graph_hasNextVertex()

TEST_F(GraphIntegrationTest, hasNextVertexWithoutInitVertexIterator)
{
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexWithInitVertexIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV1V2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV1V3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV2V1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV2V3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV3V1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, hasNextVertexAfterRemovalOfV3V2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

// xme_hal_graph_nextVertex()

TEST_F(GraphIntegrationTest, nextVertexWithoutInitVertexIterator)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexWithInitVertexIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV1V2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV1V3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV2V1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV2V3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV31V1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, nextVertexAfterRemovalOfV3V2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

// xme_hal_graph_initVertexIterator() / xme_hal_graph_hasNextVertex() / xme_hal_graph_nextVertex()

TEST_F(GraphIntegrationTest, vertexIteration)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

// xme_hal_graph_initVertexIterator() / xme_hal_graph_hasNextVertex() / xme_hal_graph_nextVertex() / xme_hal_graph_addVertex()

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateAdditionOfVertexWhileAtV1)
{
    xme_hal_graph_vertexId_t vi4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    vi4 = xme_hal_graph_addVertex(&g, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi4, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateAdditionOfVertexWhileAtV2)
{
    xme_hal_graph_vertexId_t vi4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));

    vi4 = xme_hal_graph_addVertex(&g, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi4, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateAdditionOfVertexWhileAtV3)
{
    xme_hal_graph_vertexId_t vi4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));

    vi4 = xme_hal_graph_addVertex(&g, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi4, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateAdditionOfVertexWhileAtEnd)
{
    xme_hal_graph_vertexId_t vi4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));

    vi4 = xme_hal_graph_addVertex(&g, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi4);

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi4, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
}

// xme_hal_graph_initVertexIterator() / xme_hal_graph_hasNextVertex() / xme_hal_graph_nextVertex() / xme_hal_graph_removeVertex()

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateRemovalOfV1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateRemovalOfV2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateRemovalOfV3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateRemovalOfV1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateRemovalOfV1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateRemovalOfV2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateRemovalOfV2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateRemovalOfV3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

TEST_F(GraphIntegrationTest, vertexIterationWithIntermediateRemovalOfV3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&g));

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&g));
}

// xme_hal_graph_initOutgoingEdgeIterator()

TEST_F(GraphIntegrationTest, initOutgoingEdgeIteratorTwiceWithSameVertex)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_UNSUPPORTED, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi1));
}

TEST_F(GraphIntegrationTest, initOutgoingEdgeIteratorTwiceWithDifferentVertex)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi2));
}

// xme_hal_graph_finiOutgoingEdgeIterator()

TEST_F(GraphIntegrationTest, finiOutgoingEdgeIteratorWithoutInitOutgoingEdgeIterator)
{
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi3));
}

TEST_F(GraphIntegrationTest, finiOutgoingEdgeIteratorWithInitVertexIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi3));
}

// xme_hal_graph_hasNextOutgoingEdge()

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1WithoutInitOutgoingEdgeIterator)
{
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2WithoutInitOutgoingEdgeIterator)
{
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3WithoutInitOutgoingEdgeIterator)
{
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1WithInitOutgoingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2WithInitOutgoingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3WithInitOutgoingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

// xme_hal_graph_initOutgoingEdgeIterator() / xme_hal_graph_hasNextOutgoingEdge() / xme_hal_graph_addEdge()

TEST_F(GraphIntegrationTest, outgoingEdgeIterationAtV1WithIntermediateAdditionOfEdgeWhileAtE3)
{
    xme_hal_graph_edgeId_t ei4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));

    ei4 = xme_hal_graph_addEdge(&g, vi1, vi2, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, outgoingEdgeIterationAtV1WithIntermediateAdditionOfEdgeWhileAtEnd)
{
    xme_hal_graph_edgeId_t ei4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    ei4 = xme_hal_graph_addEdge(&g, vi1, vi2, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, outgoingEdgeIterationAtV2WithIntermediateAdditionOfEdgeWhileAtEnd)
{
    xme_hal_graph_edgeId_t ei4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));

    ei4 = xme_hal_graph_addEdge(&g, vi2, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi2));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, outgoingEdgeIterationAtV3WithIntermediateAdditionOfEdgeWhileAtE1)
{
    xme_hal_graph_edgeId_t ei4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));

    ei4 = xme_hal_graph_addEdge(&g, vi3, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, outgoingEdgeIterationAtV3WithIntermediateAdditionOfEdgeWhileAtE2)
{
    xme_hal_graph_edgeId_t ei4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    ei4 = xme_hal_graph_addEdge(&g, vi3, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, outgoingEdgeIterationAtV3WithIntermediateAdditionOfEdgeWhileAtEnd)
{
    xme_hal_graph_edgeId_t ei4;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    ei4 = xme_hal_graph_addEdge(&g, vi3, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

// xme_hal_graph_initOutgoingEdgeIterator() / xme_hal_graph_hasNextOutgoingEdge() / xme_hal_graph_removeEdge()

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE1E2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE1E3E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE2E1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE2E3E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE3E1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1AfterRemovalOfE3E2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

// -----

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE1E2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE1E3E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE2E1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE2E3E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE3E1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2AfterRemovalOfE3E2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

// -----

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE1E2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE1E3E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE2E1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE2E3E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE3E1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3AfterRemovalOfE3E2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

// xme_hal_graph_nextOutgoingEdge()

TEST_F(GraphIntegrationTest, nextEdgeOfV1WithoutInitEdgeIterator)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2WithoutInitEdgeIterator)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3WithoutInitEdgeIterator)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1WithInitEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2WithInitEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3WithInitEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

// -----

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE1E2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE1E3E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE2E1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE2E3E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE3E1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV1AfterRemovalOfE3E2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi1));
}

// -----

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE1E2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE1E3E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE2E1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE2E3E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE3E1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV2AfterRemovalOfE3E2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi2));
}

// -----

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE1E2E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE1E3E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE2E1E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE2E3E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE3E1E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, nextEdgeOfV3AfterRemovalOfE3E2E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&g, vi3));
}

// xme_hal_graph_initIncomingEdgeIterator

TEST_F(GraphIntegrationTest, initIncomingEdgeIteratorTwiceWithSameVertex)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi1));
}

TEST_F(GraphIntegrationTest, initEdgeIteratorTwiceWithDifferentVertex)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi2));
}

// xme_hal_graph_finiIncomingEdgeIterator()

TEST_F(GraphIntegrationTest, finiIncomingEdgeIteratorWithoutInitIncomingEdgeIterator)
{
    // TODO: Currently, the incoming edge iterator does not know whether init() has been called or not
#if 0
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiIncomingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiIncomingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiIncomingEdgeIterator(&g, vi3));
#else
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi3));
#endif
}

TEST_F(GraphIntegrationTest, finiIncomingEdgeIteratorWithInitIncomingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi1));
    // TODO: Currently, the incoming edge iterator does not know whether init() has been called or not
#if 0
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiIncomingEdgeIterator(&g, vi1));
#else
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi1));
#endif

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi2));
#if 0
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiIncomingEdgeIterator(&g, vi2));
#else
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi2));
#endif

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi3));
#if 0
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_graph_finiIncomingEdgeIterator(&g, vi3));
#else
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi3));
#endif
}

// xme_hal_graph_hasNextIncomingEdge()

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1WithoutInitIncomingEdgeIterator)
{
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2WithoutInitIncomingEdgeIterator)
{
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3WithoutInitIncomingEdgeIterator)
{
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi3));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV1WithInitIncomingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV2WithInitIncomingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi2));
    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&g, vi2));
}

TEST_F(GraphIntegrationTest, hasNextEdgeOfV3WithInitIncomingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&g, vi3));
}

// xme_hal_graph_getVertexData()

TEST_F(GraphIntegrationTest, getVertexDataOfV1)
{
    void* vd = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&g, vi1, &vd));
    EXPECT_EQ(vd1, vd);
}

TEST_F(GraphIntegrationTest, getVertexDataOfV2)
{
    void* vd = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&g, vi2, &vd));
    EXPECT_EQ(vd2, vd);
}

TEST_F(GraphIntegrationTest, getVertexDataOfV3)
{
    void* vd = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&g, vi3, &vd));
    EXPECT_EQ(vd3, vd);
}

TEST_F(GraphIntegrationTest, getVertexDataOfV1AfterRemoval)
{
    void* vd = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getVertexData(&g, vi1, &vd));
    EXPECT_EQ(NULL, vd);
}

TEST_F(GraphIntegrationTest, getVertexDataOfV2AfterRemoval)
{
    void* vd = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getVertexData(&g, vi2, &vd));
    EXPECT_EQ(NULL, vd);
}

TEST_F(GraphIntegrationTest, getVertexDataOfV3AfterRemoval)
{
    void* vd = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getVertexData(&g, vi3, &vd));
    EXPECT_EQ(NULL, vd);
}

// xme_hal_graph_getEdgeData()

TEST_F(GraphIntegrationTest, getVertexDataOfE1)
{
    void* ed = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&g, ei1, &ed));
    EXPECT_EQ(ed1, ed);
}

TEST_F(GraphIntegrationTest, getVertexDataOfE2)
{
    void* ed = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&g, ei2, &ed));
    EXPECT_EQ(ed2, ed);
}

TEST_F(GraphIntegrationTest, getVertexDataOfE3)
{
    void* ed = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&g, ei3, &ed));
    EXPECT_EQ(ed3, ed);
}

TEST_F(GraphIntegrationTest, getVertexDataOfE1AfterRemoval)
{
    void* ed = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, &ed));
    EXPECT_EQ(NULL, ed);
}

TEST_F(GraphIntegrationTest, getVertexDataOfE2AfterRemoval)
{
    void* ed = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, &ed));
    EXPECT_EQ(NULL, ed);
}

TEST_F(GraphIntegrationTest, getVertexDataOfE3AfterRemoval)
{
    void* ed = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, &ed));
    EXPECT_EQ(NULL, ed);
}

// xme_hal_graph_getSourceVertex() / xme_hal_graph_getSinkVertex()

TEST_F(GraphIntegrationTest, getSourceAndSinkVertices)
{
    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(vi2, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(vi1, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(vi2, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(vi1, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV1V2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV1V3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV2V1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV2V3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV3V1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

TEST_F(GraphIntegrationTest, getSourceAndSinkVerticesAfterRemovalOfV3V2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei3));
}

// xme_hal_graph_getVertexCompareCallback()

TEST_F(GraphIntegrationTest, getVertexCompareCallback)
{
    EXPECT_EQ(NULL, xme_hal_graph_getVertexCompareCallback(&g));
}

// xme_hal_graph_setVertexCompareCallback()

TEST_F(GraphIntegrationTest, setVertexCompareCallbackWithNullArgument)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, NULL));
    EXPECT_EQ(NULL, xme_hal_graph_getVertexCompareCallback(&g));
}

TEST_F(GraphIntegrationTest, setVertexCompareCallbackWithValidArgument)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &testCompareVertex));
    EXPECT_EQ((xme_hal_graph_vertexCompareCallback_t) &testCompareVertex, xme_hal_graph_getVertexCompareCallback(&g));
}

TEST_F(GraphIntegrationTest, addNonExistingVerticesWithComparisonCallback)
{
    int i = 42; // nonexisting number
    ASSERT_NE(i, d1);
    ASSERT_NE(i, d2);
    ASSERT_NE(i, d3);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &testCompareVertex));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_addVertex(&g, NULL));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_addVertex(&g, &i));
}

TEST_F(GraphIntegrationTest, addExistingVerticesWithComparisonCallback)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setVertexCompareCallback(&g, &testCompareVertex));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_addVertex(&g, vd1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_addVertex(&g, vd2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_addVertex(&g, vd3));
}

// xme_hal_graph_getEdgeCompareCallback()

TEST_F(GraphIntegrationTest, getEdgeCompareCallback)
{
    EXPECT_EQ(NULL, xme_hal_graph_getEdgeCompareCallback(&g));
}

// xme_hal_graph_setEdgeCompareCallback()

TEST_F(GraphIntegrationTest, setEdgeCompareCallbackWithNullArgument)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(&g, NULL));
    EXPECT_EQ(NULL, xme_hal_graph_getEdgeCompareCallback(&g));
}

TEST_F(GraphIntegrationTest, setEdgeCompareCallbackWithValidArgument)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(&g, &testCompareEdge));
    EXPECT_EQ((xme_hal_graph_edgeCompareCallback_t) &testCompareEdge, xme_hal_graph_getEdgeCompareCallback(&g));
}

TEST_F(GraphIntegrationTest, addNonExistingEdgesWithComparisonCallback)
{
    int i = 42; // nonexisting number
    ASSERT_NE(i, d1);
    ASSERT_NE(i, d2);
    ASSERT_NE(i, d3);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(&g, &testCompareEdge));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi3, NULL));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi2, &i));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationTest, addExistingEdgesWithComparisonCallbackAndSameVertices)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(&g, &testCompareEdge));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi3, ed1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi2, ed2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi1, vi3, ed3));
    EXPECT_EQ(3, xme_hal_graph_getEdgeCount(&g));
}

// Edge comparison callbacks are only allowed to be compared when the
// vertices are the same. Edge comparation itself does not make sense.
TEST_F(GraphIntegrationTest, addExistingEdgesWithComparisonCallbackAndDifferentVertices)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(&g, &testCompareEdge));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi3, ed2));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi2, ed3));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi1, vi3, ed1));
    EXPECT_EQ(6, xme_hal_graph_getEdgeCount(&g));
}

// Stress tests

TEST_F(GraphIntegrationTest, stressTest)
{
    uint32_t i;
    const uint16_t maxVertices = 16384; //UINT16_MAX;
    const uint16_t maxEdges = 16384; //UINT16_MAX;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&g));

    for (i = 0; i < maxVertices; i++)
    {
        xme_hal_graph_vertexId_t vi;

        vi = xme_hal_graph_addVertex(&g, (void*)(uintptr_t) i);
        EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

        if (0 == i % (maxVertices/10))
        {
            printf("Adding vertices: %.0f%%\n", 100.f * i / maxVertices);
        }
    }

    EXPECT_EQ(maxVertices, xme_hal_graph_getVertexCount(&g));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    {
        for (i = 0; i < maxVertices; i++)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));

            vi = xme_hal_graph_nextVertex(&g);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi));
            {
                EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi));

            if (0 == i % (maxVertices/10))
            {
                printf("Iterating through vertices without edges: %.0f%%\n", 100.f * i / maxVertices);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    }
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&g));

    for (i = 0; i < maxEdges; i++)
    {
        xme_hal_graph_edgeId_t ei;

        ei = xme_hal_graph_addEdge
        (
            &g,
            xme_hal_graph_getNextVertexWithDataComparison(&g, (void*)(uintptr_t) i, XME_HAL_GRAPH_INVALID_VERTEX_ID),
            xme_hal_graph_getNextVertexWithDataComparison(&g, (void*)(uintptr_t) ((i + 1) % maxVertices), XME_HAL_GRAPH_INVALID_VERTEX_ID),
            (void*)(uintptr_t) i
        );
        EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei);

        if (0 == i % (maxEdges/10))
        {
            printf("Adding edges: %.0f%%\n", 100.f * i / maxEdges);
        }
    }

    EXPECT_EQ(maxEdges, xme_hal_graph_getEdgeCount(&g));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    {
        for (i = 0; i < maxVertices; i++)
        {
            xme_hal_graph_vertexId_t vi;

            EXPECT_TRUE(xme_hal_graph_hasNextVertex(&g));

            vi = xme_hal_graph_nextVertex(&g);
            EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, vi);

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi));
            {
                xme_hal_graph_edgeId_t ei;
                void* ed = NULL;
                xme_hal_graph_vertexId_t soi;
                xme_hal_graph_vertexId_t sii;
                void* sod = NULL;
                void* sid = NULL;

                EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi));

                ei = xme_hal_graph_nextOutgoingEdge(&g, vi);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei);

                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&g, ei, &ed));
                EXPECT_TRUE(NULL != ed || 0 == i);

                soi = xme_hal_graph_getSourceVertex(&g, ei);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, soi);

                sii = xme_hal_graph_getSinkVertex(&g, ei);
                EXPECT_NE(XME_HAL_GRAPH_INVALID_VERTEX_ID, sii);

                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&g, soi, &sod));
                EXPECT_TRUE(NULL != sod || 0 == i);

                EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&g, sii, &sid));
                EXPECT_TRUE(NULL != sid || 0 == ((i + 1) % maxVertices));

                EXPECT_EQ((void*)(uintptr_t) i, sod);
                EXPECT_EQ((void*)(uintptr_t) ((i + 1) % maxVertices), sid);

                EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi));
            }
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi));

            if (0 == i % (maxVertices/10))
            {
                printf("Iterating through vertices with edges: %.0f%%\n", 100.f * i / maxVertices);
            }
        }

        EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    }
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&g));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&g));
}

// Various regression tests

TEST_F(GraphIntegrationTest, regressionTest2874)
{
    // Regression test for #2874
    // Reduce the graph to a simpler version with just two vertices and one edge
    // (v2 --> v1) and then iterate over the incoming edges of v1.
    xme_hal_graph_edgeId_t ei4, ei;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(2, xme_hal_graph_getVertexCount(&g));
    ASSERT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    ei4 = xme_hal_graph_addEdge(&g, vi2, vi1, NULL);
    ASSERT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei4);
    ASSERT_EQ(2, xme_hal_graph_getVertexCount(&g));
    ASSERT_EQ(1, xme_hal_graph_getEdgeCount(&g));

    xme_hal_graph_initIncomingEdgeIterator(&g, vi1);
    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));

    ei = xme_hal_graph_nextIncomingEdge(&g, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei); // Issue #2874
}

//----------------------------------------------------------------------------//
//     GraphIntegrationMultigraphTest                                         //
//----------------------------------------------------------------------------//

// xme_hal_graph_init()

TEST_F(GraphIntegrationMultigraphTest, initNonEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&g));

    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_fini()

TEST_F(GraphIntegrationMultigraphTest, finiNonEmptyGraph)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&g));

    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_getVertexCount()

TEST_F(GraphIntegrationMultigraphTest, getVertexCount)
{
    EXPECT_EQ(3, xme_hal_graph_getVertexCount(&g));
}

// xme_hal_graph_getEdgeCount()

TEST_F(GraphIntegrationMultigraphTest, getEdgeCount)
{
    EXPECT_EQ(6, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_clone()

TEST_F(GraphIntegrationMultigraphTest, cloneGraph)
{
    xme_hal_graph_graph_t cloned;
    xme_hal_graph_edgeId_t e;
    void* d = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_init(&cloned));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clone(&g, &cloned));

    // Check counts

    EXPECT_EQ(3, xme_hal_graph_getVertexCount(&cloned));
    EXPECT_EQ(6, xme_hal_graph_getEdgeCount(&cloned));

    // Check vertices

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&cloned));

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&cloned));
    EXPECT_EQ(vi1, xme_hal_graph_nextVertex(&cloned));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&cloned, vi1, &d));
    EXPECT_EQ(vd1, d);

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&cloned));
    EXPECT_EQ(vi2, xme_hal_graph_nextVertex(&cloned));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&cloned, vi2, &d));
    EXPECT_EQ(vd2, d);

    EXPECT_TRUE(xme_hal_graph_hasNextVertex(&cloned));
    EXPECT_EQ(vi3, xme_hal_graph_nextVertex(&cloned));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getVertexData(&cloned, vi3, &d));
    EXPECT_EQ(vd3, d);

    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&cloned));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_nextVertex(&cloned));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&cloned));

    // Check outgoing edges of V1

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&cloned, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&cloned, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei3, &d));
    EXPECT_EQ(ed3, d);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi1));
    EXPECT_EQ(ei6, xme_hal_graph_nextOutgoingEdge(&cloned, vi1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei6, &d));
    EXPECT_EQ(ed6, d);

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&cloned, vi1));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&cloned, vi1));

    // Check outgoing edges of V2

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&cloned, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi2));
    EXPECT_EQ(ei5, xme_hal_graph_nextOutgoingEdge(&cloned, vi2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei5, &d));
    EXPECT_EQ(ed5, d);

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&cloned, vi2));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&cloned, vi2));

    // Check outgoing edges of V3

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&cloned, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei1, &d));
    EXPECT_EQ(ed1, d);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei2, &d));
    EXPECT_EQ(ed2, d);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei4, &d));
    EXPECT_EQ(ed4, d);

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&cloned, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextOutgoingEdge(&cloned, vi3));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiOutgoingEdgeIterator(&cloned, vi3));

    // Check incoming edges of V1

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&cloned, vi1));

    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&cloned, vi1));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&cloned, vi1));

    // Check incoming edges of V2

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&cloned, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi2));
    EXPECT_EQ(ei2, xme_hal_graph_nextIncomingEdge(&cloned, vi2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei2, &d));
    EXPECT_EQ(ed2, d);

    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&cloned, vi2));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&cloned, vi2));

    // Check incoming edges of V3

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&cloned, vi3));

    // Order of edge iteration is not defined. Hence, we consider all four possible cases here.
    for (int i = 0; i < 5; i++)
    {
        EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi3));
        e = xme_hal_graph_nextIncomingEdge(&cloned, vi3);
        ASSERT_TRUE(ei1 == e || ei3 == e || ei4 == e || ei5 == e || ei6 == e);

        if (ei1 == e)
        {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei1, &d));
            EXPECT_EQ(ed1, d);
        }
        else if (ei3 == e)
        {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei3, &d));
            EXPECT_EQ(ed3, d);
        }
        else if (ei4 == e)
        {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei4, &d));
            EXPECT_EQ(ed4, d);
        }
        else if (ei5 == e)
        {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei5, &d));
            EXPECT_EQ(ed5, d);
        }
        else // if (ei6 == e)
        {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&cloned, ei6, &d));
            EXPECT_EQ(ed6, d);
        }
    }

    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&cloned, vi3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_nextIncomingEdge(&cloned, vi3));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&cloned, vi3));

    // Done

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_fini(&cloned));
}

// xme_hal_graph_clear()

TEST_F(GraphIntegrationMultigraphTest, clearGraph)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_clear(&g));

    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initVertexIterator(&g));
    EXPECT_FALSE(xme_hal_graph_hasNextVertex(&g));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiVertexIterator(&g));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi1));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi2));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiOutgoingEdgeIterator(&g, vi3));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(&g, vi1));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi2));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(&g, vi2));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_initIncomingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi3));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_graph_finiIncomingEdgeIterator(&g, vi3));
}

// xme_hal_graph_getNextVertexWithDataComparison()
// skipped, because already covered by base class

// xme_hal_graph_removeVertex()

TEST_F(GraphIntegrationMultigraphTest, removeVertexThatIsSourceAndDestinationOfAnEdge)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    ei1 = xme_hal_graph_addEdge(&g, vi3, vi3, ed2);
    ASSERT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, ei1);
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllVerticesInOrderV1V2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v1->v3 (e3) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v2 (e2) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v3 (e1) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllVerticesInOrderV1V3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v1->v3 (e3) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edges v3->v3 (e1) and v3->v2 (e2) are not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllVerticesInOrderV2V1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v2 (e2) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(2, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v1->v3 (e3) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v3 (e1) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllVerticesInOrderV2V3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edge v3->v2 (e2) is not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edges v3->v3 (e1) and v1->v3 (e3) are not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllVerticesInOrderV3V1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edges v3->v3 (e1), v3->v2 (e2) and v1->v3 (e3) are not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllVerticesInOrderV3V2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    EXPECT_EQ(2, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    // Ensure that the edges v3->v3 (e1), v3->v2 (e2) and v1->v3 (e3) are not present any more in the graph
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei1, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei2, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei3, NULL));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    EXPECT_EQ(1, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    EXPECT_EQ(0, xme_hal_graph_getVertexCount(&g));
    EXPECT_EQ(0, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_addEdge()
// skipped, because already covered by base class

// xme_hal_graph_getNextEdgeWithDataComparison()

// null data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, NULL, ei6));
}

// ed1 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE1DataAndStartAfterE5)
{
    EXPECT_EQ(ei1, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE1DataAndStartAfterE6)
{
    EXPECT_EQ(ei1, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed1, ei6));
}

// ed2 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed2, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE2DataAndStartAfterE5)
{
    EXPECT_EQ(ei2, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE2DataAndStartAfterE6)
{
    EXPECT_EQ(ei2, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed2, ei6));
}

// ed3 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed3, ei6));
}

// ed4 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE4DataAndStartAfterInvalid)
{
    EXPECT_EQ(ei4, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE4DataAndStartAfterE1)
{
    EXPECT_EQ(ei4, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed4, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE4DataAndStartAfterE2)
{
    EXPECT_EQ(ei4, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE4DataAndStartAfterE3)
{
    EXPECT_EQ(ei4, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE4DataAndStartAfterE5)
{
    EXPECT_EQ(ei4, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE4DataAndStartAfterE6)
{
    EXPECT_EQ(ei4, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed4, ei6));
}

// ed5 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE5DataAndStartAfterInvalid)
{
    EXPECT_EQ(ei5, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE5DataAndStartAfterE3)
{
    EXPECT_EQ(ei5, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE5DataAndStartAfterE6)
{
    EXPECT_EQ(ei5, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed5, ei6));
}

// ed6 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE6DataAndStartAfterInvalid)
{
    EXPECT_EQ(ei6, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE6DataAndStartAfterE3)
{
    EXPECT_EQ(ei6, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed6, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeWithDataComparisonWithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeWithDataComparison(&g, ed6, ei6));
}

// xme_hal_graph_getNextEdgeBetweenWithDataComparison()

// null data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, NULL, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, NULL, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, NULL, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, NULL, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, NULL, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, NULL, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, NULL, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, NULL, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithNullDataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, NULL, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithNullDataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, NULL, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithNullDataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, NULL, ei6));
}

// ed1 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE1DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE1DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed1, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE1DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE1DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed1, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE1DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE1DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed1, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE1DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE1DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed1, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE1DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE1DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed1, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE1DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE1DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed1, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE1DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE1DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed1, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE1DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE1DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed1, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE1DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE1DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE1DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, ei6));
}

// ed2 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed2, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE2DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE2DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed2, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed2, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE2DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE2DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed2, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed2, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE2DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE2DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed2, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed2, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE2DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE2DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed2, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed2, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE2DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE2DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed2, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed2, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE2DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE2DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed2, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed2, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE2DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE2DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed2, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE2DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE2DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE2DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed2, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE2DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed2, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE2DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed2, ei6));
}

// ed3 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed3, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed3, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed3, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed3, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed3, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed3, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed3, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE3DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed3, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE3DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed3, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE3DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed3, ei6));
}

// ed4 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE4DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE4DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed4, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE4DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE4DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE4DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE4DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed4, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE4DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE4DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed4, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE4DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE4DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE4DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE4DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed4, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE4DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE4DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed4, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE4DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE4DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE4DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE4DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed4, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE4DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE4DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed4, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE4DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE4DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE4DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE4DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed4, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE4DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE4DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed4, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE4DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE4DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE4DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE4DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed4, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE4DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE4DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed4, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE4DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE4DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE4DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE4DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed4, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE4DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE4DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed4, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE4DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE4DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE4DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE4DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed4, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE4DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE4DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed4, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE4DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE4DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE4DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE4DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed4, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE4DataAndStartAfterInvalid)
{
    ASSERT_EQ(ei4, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, XME_HAL_GRAPH_INVALID_EDGE_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE4DataAndStartAfterE1)
{
    ASSERT_EQ(ei4, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, ei1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE4DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE4DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE4DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE4DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE4DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, ei6));
}

// ed5 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE5DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE5DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE5DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed5, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE5DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE5DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE5DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed5, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE5DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE5DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE5DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed5, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE5DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE5DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE5DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed5, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE5DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE5DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE5DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed5, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE5DataAndStartAfterInvalid)
{
    ASSERT_EQ(ei5, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE5DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE5DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed5, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE5DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE5DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE5DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed5, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE5DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE5DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE5DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed5, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE5DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed5, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE5DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed5, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE5DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed5, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE5DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed5, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE5DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed5, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE5DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed5, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE5DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed5, ei6));
}

// ed6 data

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE6DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE6DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed6, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V1WithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi1, ed6, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE6DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE6DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed6, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V2WithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi2, ed6, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE6DataAndStartAfterInvalid)
{
    ASSERT_EQ(ei6, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE6DataAndStartAfterE3)
{
    ASSERT_EQ(ei6, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, ei3));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV1V3WithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE6DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE6DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed6, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V1WithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi1, ed6, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE6DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE6DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed6, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V2WithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi2, ed6, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE6DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE6DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed6, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV2V3WithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi2, vi3, ed6, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE6DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE6DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed6, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V1WithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi1, ed6, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE6DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE6DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed6, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V2WithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi2, ed6, ei6));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE6DataAndStartAfterInvalid)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed6, XME_HAL_GRAPH_INVALID_EDGE_ID));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE6DataAndStartAfterE1)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed6, ei1));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE6DataAndStartAfterE2)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed6, ei2));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE6DataAndStartAfterE3)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed6, ei3));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE6DataAndStartAfterE4)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed6, ei4));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE6DataAndStartAfterE5)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed6, ei5));
}

TEST_F(GraphIntegrationMultigraphTest, getNextEdgeBetweenWithDataComparisonBetweenV3V3WithE6DataAndStartAfterE6)
{
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_getNextEdgeBetweenWithDataComparison(&g, vi3, vi3, ed6, ei6));
}

// xme_hal_graph_removeEdge()

TEST_F(GraphIntegrationMultigraphTest, removeEdgeWithExistingSameSourceAndDestination)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdge(&g, ei4));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeWithExistingDifferentSourceAndDestination)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeWithExistingDifferentSourceAndDestinationReverse)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdge(&g, ei5));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesInOrderE1E4)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesInOrderE4E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesInOrderE2E5)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesInOrderE5E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesInOrderE3E6)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei6));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesInOrderE6E3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei6));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_removeEdgeWithDataComparison()

TEST_F(GraphIntegrationMultigraphTest, removeEdgeWithDataComparisonWithE4Data)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeWithDataComparison(&g, ed4, true));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeWithDataComparisonWithE5Data)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeWithDataComparison(&g, ed5, true));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeWithDataComparisonWithE6Data)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeWithDataComparison(&g, ed6, true));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_removeEdgeBetween()

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithExistingSameSourceAndDestinationOneAtATime)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, false));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithExistingSameSourceAndDestinationAllAtOnce)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, true));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithExistingDifferentSourceAndDestinationOneAtATime)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, false));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithExistingDifferentSourceAndDestinationAllAtOnce)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, true));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, true));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithExistingDifferentSourceAndDestinationWithEdgeInOppositeDirectionV3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, true));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithExistingDifferentSourceAndDestinationWithEdgeInOppositeDirectionV2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi2, vi3, true));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi2, vi3, true));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeWithReverseExistingSourceAndDestination)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi1, true));
    EXPECT_EQ(6, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesBetweenInOrderE1E4EqualsE4E1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, false));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesBetweenInOrderE2E5)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, false));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, false));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi2, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi2, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesBetweenInOrderE5E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi2, vi3, false));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi2, vi3, false));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi2, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesBetweenInOrderE3E6)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, false));
    EXPECT_EQ(5, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi1, vi3, false));
    EXPECT_EQ(4, xme_hal_graph_getEdgeCount(&g));
}

TEST_F(GraphIntegrationMultigraphTest, removeAllEdgesBetweenInOrderE6E3)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetween(&g, vi3, vi1, false));
    EXPECT_EQ(6, xme_hal_graph_getEdgeCount(&g));
}

// xme_hal_graph_removeEdgeBetween() / xme_hal_graph_removeEdge() / xme_hal_graph_addEdge()
// skipped, because already covered by base class

// xme_hal_graph_removeEdgeBetweenWithDataComparison()

// null data

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, NULL, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, NULL, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithNullData)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, NULL, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, NULL, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, NULL, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithNullData)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, NULL, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithNullData)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, NULL, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithNullData)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, NULL, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithNullData)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, NULL, true));
}

// ed1 data

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, ed1, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, ed1, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, ed1, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, ed1, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, ed1, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, ed1, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, ed1, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithE1Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, ed1, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithE1Data)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, ed1, true));
}

// ed2 data

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, ed2, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, ed2, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, ed2, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, ed2, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, ed2, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, ed2, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, ed2, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithE2Data)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, ed2, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithE2Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, ed2, true));
}

// ed3 data

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, ed3, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, ed3, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithE3Data)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, ed3, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, ed3, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, ed3, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, ed3, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, ed3, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, ed3, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithE3Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, ed3, true));
}

// ed4 data

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithE4Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, ed4, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithE4Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, ed4, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithE4Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, ed4, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithE4Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, ed4, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithE4Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, ed4, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithE4Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, ed4, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithE4Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, ed4, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithE4Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, ed4, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithE4Data)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, ed4, true));
}

// ed5 data

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithE5Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, ed5, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithE5Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, ed5, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithE5Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, ed5, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithE5Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, ed5, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithE5Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, ed5, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithE5Data)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, ed5, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithE5Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, ed5, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithE5Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, ed5, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithE5Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, ed5, true));
}

// ed6 data

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V1WithE6Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi1, ed6, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V2WithE6Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi2, ed6, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV1V3WithE6Data)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi1, vi3, ed6, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V1WithE6Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi1, ed6, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V2WithE6Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi2, ed6, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV2V3WithE6Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi2, vi3, ed6, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V1WithE6Data)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi1, ed6, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V2WithE6Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi2, ed6, true));
}

TEST_F(GraphIntegrationMultigraphTest, removeEdgeBetweenWithDataComparisonBetweenV3V3WithE6Data)
{
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_removeEdgeBetweenWithDataComparison(&g, vi3, vi3, ed6, true));
}

// xme_hal_graph_initVertexIterator()
// skipped, because already covered by base class

// xme_hal_graph_finiVertexIterator()
// skipped, because already covered by base class

// xme_hal_graph_hasNextVertex()
// skipped, because already covered by base class

// xme_hal_graph_nextVertex()
// skipped, because already covered by base class

// xme_hal_graph_initVertexIterator() / xme_hal_graph_hasNextVertex() / xme_hal_graph_nextVertex()
// skipped, because already covered by base class

// xme_hal_graph_initVertexIterator() / xme_hal_graph_hasNextVertex() / xme_hal_graph_nextVertex() / xme_hal_graph_addVertex()
// skipped, because already covered by base class

// xme_hal_graph_initVertexIterator() / xme_hal_graph_hasNextVertex() / xme_hal_graph_nextVertex() / xme_hal_graph_removeVertex()
// skipped, because already covered by base class

// xme_hal_graph_initOutgoingEdgeIterator()
// skipped, because already covered by base class

// xme_hal_graph_finiOutgoingEdgeIterator()
// skipped, because already covered by base class

// xme_hal_graph_hasNextOutgoingEdge()

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV1WithInitOutgoingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV2WithInitOutgoingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV3WithInitOutgoingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

// xme_hal_graph_initOutgoingEdgeIterator() / xme_hal_graph_hasNextOutgoingEdge() / xme_hal_graph_addEdge()

TEST_F(GraphIntegrationMultigraphTest, outgoingEdgeIterationAtV1WithIntermediateAdditionOfEdgeWhileAtE3)
{
    xme_hal_graph_edgeId_t ei7;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));

    ei7 = xme_hal_graph_addEdge(&g, vi1, vi2, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei6, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei7, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, outgoingEdgeIterationAtV1WithIntermediateAdditionOfEdgeWhileAtE6)
{
    xme_hal_graph_edgeId_t ei7;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    ei7 = xme_hal_graph_addEdge(&g, vi1, vi2, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei6, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei7, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, outgoingEdgeIterationAtV1WithIntermediateAdditionOfEdgeWhileAtEnd)
{
    xme_hal_graph_edgeId_t ei7;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei3, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei6, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    ei7 = xme_hal_graph_addEdge(&g, vi1, vi2, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
    EXPECT_EQ(ei7, xme_hal_graph_nextOutgoingEdge(&g, vi1));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, outgoingEdgeIterationAtV2WithIntermediateAdditionOfEdgeWhileAtE5)
{
    xme_hal_graph_edgeId_t ei7;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));

    ei7 = xme_hal_graph_addEdge(&g, vi2, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
    EXPECT_EQ(ei5, xme_hal_graph_nextOutgoingEdge(&g, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
    EXPECT_EQ(ei7, xme_hal_graph_nextOutgoingEdge(&g, vi2));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationMultigraphTest, outgoingEdgeIterationAtV2WithIntermediateAdditionOfEdgeWhileAtEnd)
{
    xme_hal_graph_edgeId_t ei7;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
    EXPECT_EQ(ei5, xme_hal_graph_nextOutgoingEdge(&g, vi2));

    ei7 = xme_hal_graph_addEdge(&g, vi2, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
    EXPECT_EQ(ei7, xme_hal_graph_nextOutgoingEdge(&g, vi2));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationMultigraphTest, outgoingEdgeIterationAtV3WithIntermediateAdditionOfEdgeWhileAtE1)
{
    xme_hal_graph_edgeId_t ei7;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));

    ei7 = xme_hal_graph_addEdge(&g, vi3, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei7, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationMultigraphTest, outgoingEdgeIterationAtV3WithIntermediateAdditionOfEdgeWhileAtE2)
{
    xme_hal_graph_edgeId_t ei7;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    ei7 = xme_hal_graph_addEdge(&g, vi3, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei7, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationMultigraphTest, outgoingEdgeIterationAtV3WithIntermediateAdditionOfEdgeWhileAtE4)
{
    xme_hal_graph_edgeId_t ei7;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    ei7 = xme_hal_graph_addEdge(&g, vi3, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei7, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationMultigraphTest, outgoingEdgeIterationAtV3WithIntermediateAdditionOfEdgeWhileAtEnd)
{
    xme_hal_graph_edgeId_t ei7;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei1, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei2, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei4, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    ei7 = xme_hal_graph_addEdge(&g, vi3, vi1, NULL);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, ei7);

    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
    EXPECT_EQ(ei7, xme_hal_graph_nextOutgoingEdge(&g, vi3));

    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

// xme_hal_graph_initOutgoingEdgeIterator() / xme_hal_graph_hasNextOutgoingEdge() / xme_hal_graph_removeEdge()

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV1AfterRemovalOfE4)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV1AfterRemovalOfE5)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV1AfterRemovalOfE6)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei6));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV1AfterRemovalOfE1E4)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV1AfterRemovalOfE2E5)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV1AfterRemovalOfE3E6)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei6));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi1));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV2AfterRemovalOfE4)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV2AfterRemovalOfE5)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV2AfterRemovalOfE6)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei6));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV2AfterRemovalOfE1E4)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV2AfterRemovalOfE2E5)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV2AfterRemovalOfE3E6)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei6));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi2));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi2));
}

// -----

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV3AfterRemovalOfE4)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV3AfterRemovalOfE5)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV3AfterRemovalOfE6)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei6));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV3AfterRemovalOfE1E4)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV3AfterRemovalOfE1E4E2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_FALSE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV3AfterRemovalOfE2E5)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV3AfterRemovalOfE3E6)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei6));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initOutgoingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextOutgoingEdge(&g, vi3));
}

// xme_hal_graph_nextOutgoingEdge()
// skipped, because already covered by base class

// xme_hal_graph_initIncomingEdgeIterator
// skipped, because already covered by base class

// xme_hal_graph_finiIncomingEdgeIterator()
// skipped, because already covered by base class

// xme_hal_graph_hasNextIncomingEdge()

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV1WithInitIncomingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi1));
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV2WithInitIncomingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi2));
    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&g, vi2));
}

TEST_F(GraphIntegrationMultigraphTest, hasNextEdgeOfV3WithInitIncomingEdgeIterator)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi3));
    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&g, vi3));
}

// xme_hal_graph_getVertexData()
// skipped, because already covered by base class

// xme_hal_graph_getEdgeData()

TEST_F(GraphIntegrationMultigraphTest, getVertexDataOfE4)
{
    void* ed = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&g, ei4, &ed));
    EXPECT_EQ(ed4, ed);
}

TEST_F(GraphIntegrationMultigraphTest, getVertexDataOfE5)
{
    void* ed = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&g, ei5, &ed));
    EXPECT_EQ(ed5, ed);
}

TEST_F(GraphIntegrationMultigraphTest, getVertexDataOfE6)
{
    void* ed = NULL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_getEdgeData(&g, ei6, &ed));
    EXPECT_EQ(ed6, ed);
}

TEST_F(GraphIntegrationMultigraphTest, getVertexDataOfE4AfterRemoval)
{
    void* ed = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei4));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei4, &ed));
    EXPECT_EQ(NULL, ed);
}

TEST_F(GraphIntegrationMultigraphTest, getVertexDataOfE5AfterRemoval)
{
    void* ed = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei5));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei5, &ed));
    EXPECT_EQ(NULL, ed);
}

TEST_F(GraphIntegrationMultigraphTest, getVertexDataOfE6AfterRemoval)
{
    void* ed = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeEdge(&g, ei6));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_hal_graph_getEdgeData(&g, ei6, &ed));
    EXPECT_EQ(NULL, ed);
}

// xme_hal_graph_getSourceVertex() / xme_hal_graph_getSinkVertex()

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVertices)
{
    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(vi2, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(vi1, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(vi2, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(vi1, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(vi3, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(vi3, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV1V2V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV1V3V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV2V1V3)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV2V3V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV3V1V2)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

TEST_F(GraphIntegrationMultigraphTest, getSourceAndSinkVerticesAfterRemovalOfV3V2V1)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_removeVertex(&g, vi1));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei4));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei4));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei5));

    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSourceVertex(&g, ei6));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_VERTEX_ID, xme_hal_graph_getSinkVertex(&g, ei6));
}

// xme_hal_graph_getVertexCompareCallback()
// skipped, because already covered by base class

// xme_hal_graph_setVertexCompareCallback()
// skipped, because already covered by base class

// xme_hal_graph_getEdgeCompareCallback()
// skipped, because already covered by base class

// xme_hal_graph_setEdgeCompareCallback()

TEST_F(GraphIntegrationMultigraphTest, addExistingEdgesWithComparisonCallbackAndSameVertices)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(&g, &testCompareEdge));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi3, ed1));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi2, ed2));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi2, vi3, ed5));
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi1, vi3, ed3));
    EXPECT_EQ(6, xme_hal_graph_getEdgeCount(&g));
}

// Edge comparison callbacks are only allowed to be compared when the
// vertices are the same. Edge comparation itself does not make sense.
TEST_F(GraphIntegrationMultigraphTest, addExistingEdgesWithComparisonCallbackAndDifferentVertices)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_setEdgeCompareCallback(&g, &testCompareEdge));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi3, ed2));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi3, vi2, ed3));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi2, vi3, ed3));
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, xme_hal_graph_addEdge(&g, vi1, vi3, ed1));
    EXPECT_EQ(10, xme_hal_graph_getEdgeCount(&g));
}

//----------------------------------------------------------------------------//
//     GraphIntegrationIncomingEdgesIteratorTest                              //
//----------------------------------------------------------------------------//

// xme_hal_graph_incomingEdgeIterator()

TEST_F(GraphIntegrationIncomingEdgesIteratorTest, TestIncomingEdgeIteratorWithMoreThanTwoIncomingEdges)
{
    xme_hal_graph_edgeId_t edgeId;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_initIncomingEdgeIterator(&g, vi1));

    // First edge
    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));
    edgeId = xme_hal_graph_nextIncomingEdge(&g, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, edgeId);
    EXPECT_EQ(ei1, edgeId);

    // Second edge
    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));
    edgeId = xme_hal_graph_nextIncomingEdge(&g, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, edgeId);
    EXPECT_EQ(ei2, edgeId);

    // Third edge
    EXPECT_TRUE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));
    edgeId = xme_hal_graph_nextIncomingEdge(&g, vi1);
    EXPECT_NE(XME_HAL_GRAPH_INVALID_EDGE_ID, edgeId);
    EXPECT_EQ(ei3, edgeId);

    // No more edges
    EXPECT_FALSE(xme_hal_graph_hasNextIncomingEdge(&g, vi1));
    edgeId = xme_hal_graph_nextIncomingEdge(&g, vi1);
    EXPECT_EQ(XME_HAL_GRAPH_INVALID_EDGE_ID, edgeId);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_graph_finiIncomingEdgeIterator(&g, vi1));

}


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
