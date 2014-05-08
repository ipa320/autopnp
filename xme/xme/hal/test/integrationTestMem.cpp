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
 * $Id: integrationTestMem.cpp 3574 2013-05-29 15:30:53Z ruiz $
 */

/**
 * \file
 *         Memory integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
// Ensure that stdint.h defines limit macros
#define __STDC_LIMIT_MACROS

#include <gtest/gtest.h>

#include "xme/hal/include/mem.h"

#include <stdint.h>

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class Person
{
public:
    Person()
    {
        // Ensure a consistent, non-zero state for testing

        firstName[ 0] = firstName[ 1] = firstName[ 2] = firstName[ 3] = 0x55;
        firstName[ 4] = firstName[ 5] = firstName[ 6] = firstName[ 7] = 0x55;
        firstName[ 8] = firstName[ 9] = firstName[10] = firstName[11] = 0x55;
        firstName[12] = firstName[13] = firstName[14] = firstName[15] = 0x55;

        lastName[ 0] = lastName[ 1] = lastName[ 2] = lastName[ 3] = 0x55;
        lastName[ 4] = lastName[ 5] = lastName[ 6] = lastName[ 7] = 0x55;
        lastName[ 8] = lastName[ 9] = lastName[10] = lastName[11] = 0x55;
        lastName[12] = lastName[13] = lastName[14] = lastName[15] = 0x55;
    }

    virtual ~Person()
    {
    }

    char firstName[16];
    char lastName[16];
};


class MemoryIntegrationTest: public ::testing::Test
{
protected:
    MemoryIntegrationTest()
    : letter('a')
    , memory(NULL)
    , destination(NULL)
    {
    }

    virtual ~MemoryIntegrationTest()
    {
        // Deleting NULL won't hurt
        xme_hal_mem_free(memory);
        xme_hal_mem_free(destination);
    }

    const char letter;
    void* memory;
    void* destination;
    Person person;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     MemoryIntegrationTest                                                  //
//----------------------------------------------------------------------------//

// xme_hal_mem_alloc() tests

/*!
 * \brief  Tests the function xme_hal_mem_alloc() with the goal of
 *         receiving a null pointer (failed allocate).
 */
TEST_F(MemoryIntegrationTest, memoryAllocationIncrementalSizeOfMemoryUpToUINT16Max)
{
    for (uint16_t i = 1u; i < UINT16_MAX; ++i)
    {
        memory = xme_hal_mem_alloc(i);
        EXPECT_TRUE(NULL != memory);

        // In case of failure, trace the value
        if (memory == NULL)
        {
            RecordProperty("maxMemoryAllocationBlockSize", i - 1);
            break;
        }

        // This is necessary, because we will run out of memory otherwise!
        // (otherwise this test will take ((UINT16_MAX-1)*(UINT16_MAX-2))/2
        // bytes of memory, which is roughly 2GB!)
        xme_hal_mem_free(memory);
        memory = NULL;
    }

    RecordProperty("maxMemoryAllocationBlockSize", UINT16_MAX);
}

// xme_hal_mem_realloc() tests

TEST_F(MemoryIntegrationTest, memoryReallocationWithSizeOfCharTypeInitAndReallocWithSizeOfFactorTwo)
{
    uint16_t size = 1024;
    memory = xme_hal_mem_realloc(NULL, sizeof(char));

    for (uint16_t i = 2u; i <= size; i <<= 1)
    {
        memory = xme_hal_mem_realloc(memory, i);
        EXPECT_TRUE(NULL != memory);
    }
}

TEST_F(MemoryIntegrationTest, memoryReallocationWithSizeOfCharType)
{
    // Several incremental reallocations, starting from NULL-valued
    // memory position using the OS-generated new mem position
    const uint16_t size = 1024;

    memory = xme_hal_mem_realloc(NULL, sizeof(char));
    EXPECT_TRUE(NULL != memory);

    for (uint16_t i = 2; i <= size; i <<= 1)
    {
        void* memNew = xme_hal_mem_realloc(memory, i);
        EXPECT_TRUE(NULL != memNew);
        memory = memNew;
    }
}

TEST_F(MemoryIntegrationTest, memoryReallocationWithSizeOfCharTypeInDecrementalOrder)
{
    // Several decremental reallocations, starting from a memory position
    // pointing to the biggest possible block using the initial memory position
    const uint16_t size = 1024;

    memory = xme_hal_mem_realloc(NULL, size);
    EXPECT_TRUE(NULL != memory);

    for (uint16_t i = (size/2); i >= 1; i >>= 1)
    {
        void* memNew = xme_hal_mem_realloc(memory, i);
        EXPECT_TRUE(NULL != memNew);
        memory = memNew;
    }
}

TEST_F(MemoryIntegrationTest, memoryReallocationExtensionAndReductionWithMultipleSizeOfCharType)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char) * 32);
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char) * 8);
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char) * 128);
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char) * 2);
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char) * 64);
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char) * 4);
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char) * 512);
    EXPECT_TRUE(NULL != memory);
}

TEST_F(MemoryIntegrationTest, memoryReallocationWithNullAllocationInBetween)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, 0);
    EXPECT_TRUE(NULL == memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char));
    EXPECT_TRUE(NULL != memory);
}

TEST_F(MemoryIntegrationTest, memoryReallocationWith16timesSizeOfCharAndAdditionalNullAllocationInBetween)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, 0);
    EXPECT_TRUE(NULL == memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char) * 16);
    EXPECT_TRUE(NULL != memory);
}

TEST_F(MemoryIntegrationTest, memoryReallocationAsReductionWithAdditionalNullAllocationInBetween)
{
    memory = xme_hal_mem_alloc(sizeof(char) * 16);
    EXPECT_TRUE(NULL != memory);

    memory = xme_hal_mem_realloc(memory, 0);
    EXPECT_TRUE(NULL == memory);

    memory = xme_hal_mem_realloc(memory, sizeof(char));
    EXPECT_TRUE(NULL != memory);
}

// xme_hal_mem_set() tests

TEST_F(MemoryIntegrationTest, memorySetOfAObjectElementWritingIntoAnotherElement)
{
    // Note: Care must be taken to avoid that this test yields undefined behavior.
    //       As the order of class members in memory is not defined and memory
    //       alignment could lead to the fields being padded, we distinguish here
    //       two typical cases by comparing the addresses directly.

    ASSERT_NE(&person.lastName, &person.firstName);

    if (&person.lastName > &person.firstName)
    {
        EXPECT_EQ(&person.firstName, xme_hal_mem_set(person.firstName, letter, 32));
    }
    else
    {
        EXPECT_EQ(&person.lastName, xme_hal_mem_set(person.lastName, letter, 32));
    }
}

TEST_F(MemoryIntegrationTest, memorySetOfAObjectElementInTheMiddlePosition)
{
    EXPECT_EQ(&person.lastName, xme_hal_mem_set(person.lastName, letter, 8));
}

TEST_F(MemoryIntegrationTest, memorySetOfAObjectElementAtLastPosition)
{
    EXPECT_EQ(&person.lastName, xme_hal_mem_set(person.lastName, letter, 16));
}

// xme_hal_mem_copy() tests

TEST_F(MemoryIntegrationTest, memoryCopyOfEightElementsToAnEightElementBufferFromAnEightElementBuffer)
{
    memory = xme_hal_mem_alloc(8);
    destination = xme_hal_mem_alloc(8);

    xme_hal_mem_set(memory, '@', 8);
    EXPECT_EQ(destination, xme_hal_mem_copy(destination, memory, 8));

    for (int i = 0; i < 8; ++i)
    {
        EXPECT_EQ(((char*)memory)[i], ((char*)destination)[i]);
    }
}

TEST_F(MemoryIntegrationTest, memoryCopyOfZeroElementsToBufferFromBuffer)
{
    memory = xme_hal_mem_alloc(8);
    destination = xme_hal_mem_alloc(8);

    xme_hal_mem_set(memory, 'x', 8);
    xme_hal_mem_set(destination, 'u', 8);
    EXPECT_EQ(destination, xme_hal_mem_copy(destination, memory, 0));

    for (int i = 0; i < 8; ++i)
    {
        EXPECT_NE(((char*)memory)[i], ((char*)destination)[i]);
    }
}

TEST_F(MemoryIntegrationTest, memoryCopyOfOneElementToBufferFromBuffer)
{
    memory = xme_hal_mem_alloc(8);
    destination = xme_hal_mem_alloc(8);

    xme_hal_mem_set(memory, 'x', 8);
    xme_hal_mem_set(destination, 'u', 8);
    EXPECT_EQ(destination, xme_hal_mem_copy(destination, memory, 1));
    EXPECT_EQ(((char*) memory)[0], ((char*) destination)[0]);

    for (int i = 1; i < 8; ++i)
    {
        EXPECT_NE(((char*) memory)[i], ((char*) destination)[i]);
    }
}

TEST_F(MemoryIntegrationTest, memoryCopyThreeWayCopyWithNULLToOneBufferAsIntermediate)
{
    memory = xme_hal_mem_alloc(8);
    destination = xme_hal_mem_alloc(8);

    xme_hal_mem_set(memory, '@', 8);
    xme_hal_mem_set(destination, '-', 8);

    EXPECT_EQ(destination, xme_hal_mem_set(destination, 0, 8));
    EXPECT_EQ(destination, xme_hal_mem_copy(destination, memory, 8));

    for (int i = 0; i < 8; ++i)
    {
        EXPECT_EQ(((char*) memory)[i], ((char*) destination)[i]);
    }
}

TEST_F(MemoryIntegrationTest, memoryCopyToObjecetWithSameSize)
{
    memory = xme_hal_mem_alloc(16);
    xme_hal_mem_set(memory, 'b', 16);
    xme_hal_mem_set(person.firstName, 'a', 8);
    EXPECT_EQ(person.firstName, xme_hal_mem_copy(person.firstName, memory, 16));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
