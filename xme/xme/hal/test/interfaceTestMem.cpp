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
 * $Id: interfaceTestMem.cpp 3267 2013-05-14 07:44:18Z geisinger $
 */

/**
 * \file
 *         Memory interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
// Ensure that stdint.h defines limit macros
#define __STDC_LIMIT_MACROS

#include <gtest/gtest.h>

#include "xme/hal/include/mem.h"

#include <stdint.h>

/*
 * From C89:
 *
 * 4.10.3 Memory management functions
 * The order and contiguity of storage allocated by successive calls to the
 * calloc, malloc, and realloc functions is unspecified. The pointer returned
 * if the allocation succeeds is suitably aligned so that it may be assigned
 * to a pointer to any type of object and then used to access such an object
 * in the space allocated (until the space is explicitly freed or reallocated).
 * Each such allocation shall yield a pointer to an object disjoint from any
 * other object. The pointer returned points to the start (lowest byte address)
 * of the allocated space. If the space cannot be allocated, a null pointer is
 * returned. If the size of the space requested is zero, the behavior is
 * implementation-defined; the value returned shall be either a null pointer or
 * a unique pointer. The value of a pointer that refers to freed space is
 * indeterminate.
 */

/*
 * From C99:
 *
 * 7.20.3 Memory management functions
 * The order and contiguity of storage allocated by successive calls to the
 * calloc, malloc, and realloc functions is unspecified. The pointer returned
 * if the allocation succeeds is suitably aligned so that it may be assigned to
 * a pointer to any type of object and then used to access such an object or an
 * array of such objects in the space allocated (until the space is explicitly
 * deallocated). The lifetime of an allocated object extends from the
 * allocation until the deallocation. Each such allocation shall yield a
 * pointer to an object disjoint from any other object. The pointer returned
 * points to the start (lowest byte address) of the allocated space. If the
 * space cannot be allocated, a null pointer is returned. If the size of the
 * space requested is zero, the behavior is implementationdefined: either a
 * null pointer is returned, or the behavior is as if the size were some
 * nonzero value, except that the returned pointer shall not be used to access
 * an object.
 */

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class MemoryInterfaceTest: public ::testing::Test
{
protected:
    MemoryInterfaceTest()
    : letter('a')
    , memory(NULL)
    , destination(NULL)
    {
        // "fortiss GmbH"
        stringData[ 0] = 'f'; stringData[ 1] = 'o'; stringData[ 2] = 'r'; stringData[ 3] = 't';
        stringData[ 4] = 'i'; stringData[ 5] = 's'; stringData[ 6] = 's'; stringData[ 7] = ' ';
        stringData[ 8] = 'G'; stringData[ 9] = 'm'; stringData[10] = 'b'; stringData[11] = 'H';
        stringData[12] = 0;
    }

    virtual ~MemoryInterfaceTest()
    {
        // Deleting NULL won't hurt
        xme_hal_mem_free(memory);
        xme_hal_mem_free(destination);
    }

    const char letter;
    char stringData[13];
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

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfCharType)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    ASSERT_TRUE(NULL != memory);

    char* m = (char*)memory;
    EXPECT_EQ(0, m[0]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfCharArray)
{
    memory = xme_hal_mem_alloc(sizeof(char[16]));
    ASSERT_TRUE(NULL != memory);

    char* m = (char*)memory;
    EXPECT_EQ(0, m[0]);
    EXPECT_EQ(0, m[1]);
    EXPECT_EQ(0, m[2]);
    EXPECT_EQ(0, m[3]);
    EXPECT_EQ(0, m[4]);
    EXPECT_EQ(0, m[5]);
    EXPECT_EQ(0, m[6]);
    EXPECT_EQ(0, m[7]);
    EXPECT_EQ(0, m[8]);
    EXPECT_EQ(0, m[9]);
    EXPECT_EQ(0, m[10]);
    EXPECT_EQ(0, m[11]);
    EXPECT_EQ(0, m[12]);
    EXPECT_EQ(0, m[13]);
    EXPECT_EQ(0, m[14]);
    EXPECT_EQ(0, m[15]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfShortType)
{
    memory = xme_hal_mem_alloc(sizeof(short));
    ASSERT_TRUE(NULL != memory);

    short* m = (short*)memory;
    EXPECT_EQ(0, m[0]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfIntegerType)
{
    memory = xme_hal_mem_alloc(sizeof(int));
    ASSERT_TRUE(NULL != memory);

    int* m = (int*)memory;
    EXPECT_EQ(0, m[0]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfLongType)
{
    memory = xme_hal_mem_alloc(sizeof(long));
    ASSERT_TRUE(NULL != memory);

    long* m = (long*)memory;
    EXPECT_EQ(0L, m[0]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfLongLongType)
{
    memory = xme_hal_mem_alloc(sizeof(long long));
    ASSERT_TRUE(NULL != memory);

    long long* m = (long long*)memory;
    EXPECT_EQ(0LL, m[0]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfFloatType)
{
    memory = xme_hal_mem_alloc(sizeof(float));
    ASSERT_TRUE(NULL != memory);

    float* m = (float*)memory;
    EXPECT_EQ(0.0f, m[0]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfDoubleType)
{
    memory = xme_hal_mem_alloc(sizeof(double));
    ASSERT_TRUE(NULL != memory);

    double* m = (double*)memory;
    EXPECT_EQ(0.0, m[0]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfLongDoubleType)
{
    memory = xme_hal_mem_alloc(sizeof(long double));
    ASSERT_TRUE(NULL != memory);

    long double* m = (long double*)memory;
    EXPECT_EQ(0.0L, m[0]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfStructType)
{
    typedef struct
    {
        char name[20];
        int index;
    } someStruct;

    memory = xme_hal_mem_alloc(sizeof(someStruct));
    ASSERT_TRUE(NULL != memory);

    someStruct* m = (someStruct*)memory;
    EXPECT_EQ(0, m->name[0]);
    EXPECT_EQ(0, m->name[1]);
    EXPECT_EQ(0, m->name[2]);
    EXPECT_EQ(0, m->name[3]);
    EXPECT_EQ(0, m->name[4]);
    EXPECT_EQ(0, m->name[5]);
    EXPECT_EQ(0, m->name[6]);
    EXPECT_EQ(0, m->name[7]);
    EXPECT_EQ(0, m->name[8]);
    EXPECT_EQ(0, m->name[9]);
    EXPECT_EQ(0, m->name[10]);
    EXPECT_EQ(0, m->name[11]);
    EXPECT_EQ(0, m->name[12]);
    EXPECT_EQ(0, m->name[13]);
    EXPECT_EQ(0, m->name[14]);
    EXPECT_EQ(0, m->name[15]);
    EXPECT_EQ(0, m->name[16]);
    EXPECT_EQ(0, m->name[17]);
    EXPECT_EQ(0, m->name[18]);
    EXPECT_EQ(0, m->name[19]);
    EXPECT_EQ(0, m->index);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfBoolType)
{
    memory = xme_hal_mem_alloc(sizeof(bool));
    ASSERT_TRUE(NULL != memory);

    bool* m = (bool*)memory;
    EXPECT_FALSE(m[0]);
}

TEST_F(MemoryInterfaceTest, memoryAllocationWithSizeOfUInt16Max)
{
    memory = xme_hal_mem_alloc(UINT16_MAX);
    ASSERT_TRUE(NULL != memory);

    uint16_t* m = (uint16_t*)memory;
    EXPECT_EQ(0U, m[0]);
}

// xme_hal_mem_realloc() tests

TEST_F(MemoryInterfaceTest, memoryReallocationWithSizeOfCharTypeInitAndReallocWithSizeOfIntType)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    ASSERT_TRUE(NULL != memory);

    char* m = (char*)memory;
    EXPECT_EQ(0, m[0]);

    memory = xme_hal_mem_realloc(memory, sizeof(int));
    ASSERT_TRUE(NULL != memory);

    m = (char*)memory;
    EXPECT_EQ(0, m[0]);
}

TEST_F(MemoryInterfaceTest, memoryReallocationWithNullTypeInitAndReallocWithSizeOfIntType)
{
    memory = xme_hal_mem_realloc(NULL, sizeof(int));
    EXPECT_TRUE(NULL != memory);
}

TEST_F(MemoryInterfaceTest, memoryReallocationWithSizeOfInitTypeInitAndReallocWithSizeOfZero)
{
    memory = xme_hal_mem_alloc(sizeof(int));
    EXPECT_TRUE(NULL == xme_hal_mem_realloc(memory, 0));
    memory = NULL;
}

// xme_hal_mem_set() tests

TEST_F(MemoryInterfaceTest, memorySetWithSimpleLetterAndMemoryGreaterThanNeeded)
{
    char str[5] = { letter, letter, letter, letter, 0 };

    memory = xme_hal_mem_alloc(sizeof(char)*8);
    EXPECT_NE((void*)NULL, memory);

    memory = xme_hal_mem_set(memory, letter, 4);
    EXPECT_NE((void*)NULL, memory);

    xme_hal_mem_set(((char*)memory)+4, 0, sizeof(letter));
    EXPECT_STREQ(str, (char*) memory);
}

TEST_F(MemoryInterfaceTest, memorySetWithOverwrittingAStringWithOneLetter)
{
    char str[13] = { letter, letter, letter, letter, letter, letter,
        letter, letter, letter, letter, letter, letter, 0 };

    xme_hal_mem_set(stringData, letter, strlen(stringData));
    EXPECT_STRNE("fortiss GmbH", stringData);
    EXPECT_STRNE("aortiss GmbH", stringData);
    EXPECT_STREQ(str, stringData);
}

TEST_F(MemoryInterfaceTest, memorySetWithOverwrittingFirstLetterOfAString)
{
    xme_hal_mem_set(stringData, 'F', 1);
    EXPECT_STRNE("aaaaaaaaaaaa", stringData);
    EXPECT_STRNE("bbbbbbbbbbbb", stringData);
    EXPECT_STREQ("Fortiss GmbH", stringData);
}

TEST_F(MemoryInterfaceTest, memorySetWithOverwrittingNothingOfAString)
{
    xme_hal_mem_set(stringData, 'c', 0);
    EXPECT_STREQ("fortiss GmbH", stringData);
}

TEST_F(MemoryInterfaceTest, memorySetOfZeroElementWithNULLInOneByteMemory)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_EQ(memory, xme_hal_mem_set(memory, 0, 0));
}

TEST_F(MemoryInterfaceTest, memorySetOfOneElementWithNULLInOneByteMemory)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_EQ(memory, xme_hal_mem_set(memory, 0, 1));
}

TEST_F(MemoryInterfaceTest, memorySetOfZeroElementWithOneLetterInOneByteMemory)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_EQ(memory, xme_hal_mem_set(memory, letter, 0));
}

TEST_F(MemoryInterfaceTest, memorySetOfOneElementWithOneLetterInOneByteMemory)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    EXPECT_EQ(memory, xme_hal_mem_set(memory, letter, 1));
}

TEST_F(MemoryInterfaceTest, memorySetOfZeroElementWithNULLInZeroByteMemory)
{
    memory = xme_hal_mem_alloc(0);
    EXPECT_EQ(memory, xme_hal_mem_set(memory, 0, 0));
}

TEST_F(MemoryInterfaceTest, memorySetOfZeroElementWithOneLetterInZeroByteMemory)
{
    memory = xme_hal_mem_alloc(0);
    EXPECT_EQ(memory, xme_hal_mem_set(memory, letter, 0));
}

// xme_hal_mem_copy() tests

TEST_F(MemoryInterfaceTest, memoryCopyToBufferWithZeroSpaceZeroElements)
{
    memory = xme_hal_mem_alloc(8);
    destination = xme_hal_mem_alloc(0);
    xme_hal_mem_set(memory, '-', 5);
    EXPECT_EQ(destination, xme_hal_mem_copy(destination, memory, 0));
}

TEST_F(MemoryInterfaceTest, memoryCopyToBufferWithZeroSpaceOneElement)
{
    memory = xme_hal_mem_alloc(8);
    destination = xme_hal_mem_alloc(0);
    xme_hal_mem_set(memory, '-', 5);
    EXPECT_EQ(destination, xme_hal_mem_copy(destination, memory, 1));
}

// xme_hal_mem_compare() tests

TEST_F(MemoryInterfaceTest, memoryCompareOfZeroElementsInOneElementBufferWithNULL)
{
    memory = xme_hal_mem_alloc(0);
    xme_hal_mem_set(memory, 's', 1);
    EXPECT_EQ(0, xme_hal_mem_compare(memory, NULL, 0));
}

TEST_F(MemoryInterfaceTest, memoryCompareOfZeroElementsInNULLWithOneElementBuffer)
{
    memory = xme_hal_mem_alloc(0);

    xme_hal_mem_set(memory, 's', 1);
    EXPECT_EQ(0, xme_hal_mem_compare(NULL, memory, 0));
}

TEST_F(MemoryInterfaceTest, memoryCompareOfZeroElementsInTwoBuffersWithOneElement)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    destination = xme_hal_mem_alloc(sizeof(char));

    xme_hal_mem_set(memory, 's', 1);
    xme_hal_mem_set(destination, 's', 1);
    EXPECT_EQ(0, xme_hal_mem_compare(memory, destination, 0));
}

TEST_F(MemoryInterfaceTest, memoryCompareOfOneElementInTwoBuffersWithOneElement)
{
    memory = xme_hal_mem_alloc(sizeof(char));
    destination = xme_hal_mem_alloc(sizeof(char));

    xme_hal_mem_set(memory, 's', 1);
    xme_hal_mem_set(destination, 's', 1);
    EXPECT_EQ(0, xme_hal_mem_compare(memory, destination, 1));
}

TEST_F(MemoryInterfaceTest, memoryCompareTwoElementsABiggerWithASmallerOne)
{
    char first[] = "good";
    char second[] = "goal";
    EXPECT_GT(xme_hal_mem_compare(first, second, 3), 0);
}

TEST_F(MemoryInterfaceTest, memoryCompareTwoElementsASmallerWithABiggerOne)
{
    char first[] = "good";
    char second[] = "goal";
    EXPECT_LT(xme_hal_mem_compare(second, first, 3), 0);
}

TEST_F(MemoryInterfaceTest, memoryCompareTwoElementsStartingWithSameTwoLetters)
{
    char first[] = "good";
    char second[] = "goal";
    EXPECT_EQ(xme_hal_mem_compare(first, second, 2), 0);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
