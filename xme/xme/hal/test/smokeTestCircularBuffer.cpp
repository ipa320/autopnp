/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: smokeTestCircularBuffer.cpp 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Circular Buffer smoke test.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/circularBuffer.h"

#include <stdint.h>

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class CircularBufferSmokeTest : public ::testing::Test {
    protected:
        CircularBufferSmokeTest() 
        {
            // Set values to the items to include in the circular buffer.
            items[0] = 1;
            items[1] = 2;
            items[2] = 3;
            items[3] = 4;
            items[4] = 5;

            // Set maximum size of the circular buffer. 
            cbMaxSize = 4U;
        }

        virtual
        ~CircularBufferSmokeTest() {
            // Nothing to do
        }

    uint8_t items[5];
    uint8_t cbMaxSize;
    uint8_t item;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     CircularBufferSmokeTest                                                //
//----------------------------------------------------------------------------//

TEST_F(CircularBufferSmokeTest, initAndFini)
{
    xme_hal_circularBuffer_t circularBuffer;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_init(&circularBuffer, cbMaxSize, sizeof(uint8_t), false));
    xme_hal_circularBuffer_fini(&circularBuffer);
}

TEST_F(CircularBufferSmokeTest, invalidInit)
{
    xme_hal_circularBuffer_t circularBuffer;
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_circularBuffer_init(&circularBuffer, 0U, sizeof(uint8_t), false));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_circularBuffer_init(&circularBuffer, 0U, 0U, false));
}

TEST_F(CircularBufferSmokeTest, popJustInitialized)
{
    xme_hal_circularBuffer_t circularBuffer;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_init(&circularBuffer, cbMaxSize, sizeof(uint8_t), false));
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    xme_hal_circularBuffer_fini(&circularBuffer);
}

TEST_F(CircularBufferSmokeTest, pushAndPop)
{
    xme_hal_circularBuffer_t circularBuffer;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_init(&circularBuffer, cbMaxSize, sizeof(uint8_t), false));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[0]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[0], item);
    xme_hal_circularBuffer_fini(&circularBuffer);
}

TEST_F(CircularBufferSmokeTest, pushAllItemsWithOverwriteSetToFalse)
{
    xme_hal_circularBuffer_t circularBuffer;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_init(&circularBuffer, cbMaxSize, sizeof(uint8_t), false));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[0]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[1]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[2]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[3]));
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[4]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[0], item);
    xme_hal_circularBuffer_fini(&circularBuffer);
}

TEST_F(CircularBufferSmokeTest, pushAllItemsWithOverwriteSetToTrue)
{
    xme_hal_circularBuffer_t circularBuffer;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_init(&circularBuffer, cbMaxSize, sizeof(uint8_t), true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[0]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[1]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[2]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[3]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[4]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[1], item);
    xme_hal_circularBuffer_fini(&circularBuffer);
}

TEST_F(CircularBufferSmokeTest, popAllItemsAndOverwriteSetToFalse)
{
    xme_hal_circularBuffer_t circularBuffer;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_init(&circularBuffer, cbMaxSize, sizeof(uint8_t), false));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[0]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[1]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[2]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[3]));

    // Pop
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[0], item);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[1], item);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[2], item);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[3], item);
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    xme_hal_circularBuffer_fini(&circularBuffer);
}

TEST_F(CircularBufferSmokeTest, popAllItemsAndOverwriteSetToTrue)
{
    xme_hal_circularBuffer_t circularBuffer;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_init(&circularBuffer, cbMaxSize, sizeof(uint8_t), true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[0]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[1]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[2]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[3]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[4]));

    // Pop
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[1], item);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[2], item);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[3], item);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(items[4], item);
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    xme_hal_circularBuffer_fini(&circularBuffer);
}

TEST_F(CircularBufferSmokeTest, getRelativeTailAndHeadPositions)
{
    xme_hal_circularBuffer_t circularBuffer;
    uint32_t position;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_init(&circularBuffer, cbMaxSize, sizeof(uint8_t), true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getHeadPosition(&circularBuffer, &position));
    EXPECT_EQ(0U, position);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[0]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getHeadPosition(&circularBuffer, &position));
    EXPECT_EQ(1U, position);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[1]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getHeadPosition(&circularBuffer, &position));
    EXPECT_EQ(2U, position);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[2]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getHeadPosition(&circularBuffer, &position));
    EXPECT_EQ(3U, position);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[3]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getHeadPosition(&circularBuffer, &position));
    EXPECT_EQ(0U, position);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_pushBack(&circularBuffer, &items[4]));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getHeadPosition(&circularBuffer, &position));
    EXPECT_EQ(1U, position);

    // Pop
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getTailPosition(&circularBuffer, &position));
    EXPECT_EQ(1U, position);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getTailPosition(&circularBuffer, &position));
    EXPECT_EQ(2U, position);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getTailPosition(&circularBuffer, &position));
    EXPECT_EQ(3U, position);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getTailPosition(&circularBuffer, &position));
    EXPECT_EQ(0U, position);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_popFront(&circularBuffer, &item));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_circularBuffer_getTailPosition(&circularBuffer, &position));
    EXPECT_EQ(1U, position);
    xme_hal_circularBuffer_fini(&circularBuffer);
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
