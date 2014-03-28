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
 * $Id: smokeTestMem.cpp 2908 2013-04-15 11:27:16Z geisinger $
 */

/**
 * \file
 *         Memory smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class MemorySmokeTest: public ::testing::Test
{
protected:
    MemorySmokeTest()
    : letter('a')
    , memory(NULL)
    , destination(NULL)
    {
    }

    virtual ~MemorySmokeTest()
    {
    }

    const char letter;
    void* memory;
    void* destination;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     MemorySmokeTest                                                        //
//----------------------------------------------------------------------------//

// xme_hal_mem_alloc() tests

TEST_F(MemorySmokeTest, memoryAllocationWithZeroSize)
{
    memory = xme_hal_mem_alloc(0);
    EXPECT_TRUE(NULL != memory);
}

TEST_F(MemorySmokeTest, memoryAllocationWithSizeOfCharType)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_TRUE(NULL != memory);
}

// xme_hal_mem_realloc() tests

TEST_F(MemorySmokeTest, memoryReallocationWithZeroInitAndReallocWithSizeOfCharType)
{
    memory = xme_hal_mem_alloc(0);
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char));
    EXPECT_TRUE(NULL != memory);
}

TEST_F(MemorySmokeTest, memoryReallocationWithSizeOfCharTypeInitAndReallocWithSizeOfCharType)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char));
    EXPECT_TRUE(NULL != memory);
}

TEST_F(MemorySmokeTest, memoryReallocationWithSizeOfCharTypeInitAndReallocWithZero)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, 0);
    EXPECT_TRUE(NULL == memory);
}

TEST_F(MemorySmokeTest, memoryReallocationWithSizeOferoInitAndReallocWithSizeOfZero)
{
    memory = xme_hal_mem_alloc(0);
    memory = xme_hal_mem_realloc(memory, 0);
    EXPECT_TRUE(NULL == memory);
    memory = NULL;
}

// xme_hal_mem_free() tests

TEST_F(MemorySmokeTest, memoryFreeOfAllocatedMemoryWithSizeOfCharType)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_TRUE(NULL != memory);

    xme_hal_mem_free(memory);
    EXPECT_TRUE(NULL != memory);
    memory = NULL;
}

TEST_F(MemorySmokeTest, memoryFreeOfUnAllocatedMemory)
{
    xme_hal_mem_free(memory);
    EXPECT_TRUE(NULL == memory);
}

TEST_F(MemorySmokeTest, memoryFreeOfReallocatedMemoryWithSizeOfZero)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    memory = xme_hal_mem_realloc(memory, 0);
    EXPECT_TRUE(NULL == memory);
    xme_hal_mem_free(memory);
    EXPECT_TRUE(NULL == memory);
}

// xme_hal_mem_set() tests

TEST_F(MemorySmokeTest, memorySetWithSimpleLetter)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_set(memory, letter, sizeof(letter));
    EXPECT_TRUE(NULL != memory);
    EXPECT_EQ(letter, *(char*)memory);
}

TEST_F(MemorySmokeTest, memorySetWithSimpleString)
{
    char str[2] = { letter, 0 };

    memory = xme_hal_mem_alloc(sizeof(char)*2);
    EXPECT_NE((void*)NULL, memory);

    memory = xme_hal_mem_set(memory, letter, sizeof(letter));
    EXPECT_NE((void*)NULL, memory);

    xme_hal_mem_set(((char*)memory)+1, 0, sizeof(letter));
    EXPECT_STREQ(str, (char*)memory);
}

TEST_F(MemorySmokeTest, memorySetOfZeroWithNULLInNULLMemory)
{
    EXPECT_EQ(NULL, xme_hal_mem_set(NULL, 0, 0));
}

TEST_F(MemorySmokeTest, memorySetOfZeroWithOneLetterInNULLMemory)
{
    EXPECT_EQ(NULL, xme_hal_mem_set(NULL, 'a', 0));
}

TEST_F(MemorySmokeTest, memoryCopyOfZeroElementsToNULLFromNull)
{
    EXPECT_EQ(NULL, xme_hal_mem_copy(NULL, NULL, 0));
}

TEST_F(MemorySmokeTest, memoryCopyOfZeroElementsToNULLFromBuffer)
{
    memory = xme_hal_mem_alloc(8);
    xme_hal_mem_set(memory, letter, 8);
    EXPECT_EQ(NULL, xme_hal_mem_copy(NULL, memory, 0));
}

TEST_F(MemorySmokeTest, memoryCopyOfZeroElementsToBufferFromNULL)
{
    destination = xme_hal_mem_alloc(8);
    EXPECT_EQ(destination, xme_hal_mem_copy(destination, NULL, 0));
}

TEST_F(MemorySmokeTest, memoryCopyToBufferWithZeroSpace)
{
    memory = xme_hal_mem_alloc(8);
    EXPECT_EQ(memory, xme_hal_mem_copy(memory, NULL, 0));
}

// xme_hal_mem_compare() tests

TEST_F(MemorySmokeTest, memoryCompareOfZeroElementsInNULLWithNULL)
{
    EXPECT_EQ(0, xme_hal_mem_compare(NULL, NULL, 0));
}

TEST_F(MemorySmokeTest, memoryCompareOfZeroElementsInEmptyBufferWithNULL)
{
    memory = xme_hal_mem_alloc(0);
    EXPECT_EQ(0, xme_hal_mem_compare(memory, NULL, 0));
}

TEST_F(MemorySmokeTest, memoryCompareOfZeroElementsInNullWithEmptyBuffer)
{
    memory = xme_hal_mem_alloc(0);
    EXPECT_EQ(0, xme_hal_mem_compare(NULL, memory, 0));
}

TEST_F(MemorySmokeTest, memoryCompareOfZeroElementsInEmptyBufferWithItself)
{
    memory = xme_hal_mem_alloc(0);
    EXPECT_EQ(0, xme_hal_mem_compare(memory, memory, 0));
}

TEST_F(MemorySmokeTest, memoryCompareOfZeroElementsInEmptyBufferWithEmptyBuffer)
{
    memory = xme_hal_mem_alloc(0);
    destination = xme_hal_mem_alloc(0);
    EXPECT_EQ(0, xme_hal_mem_compare(memory, destination, 0));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
