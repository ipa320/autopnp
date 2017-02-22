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
 * $Id: interfaceTestTable.cpp 6508 2014-01-27 14:35:07Z kainz $
 */

/**
 * \file
 *         Table interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/testUtils.h"

#include "xme/hal/include/table.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class TableInterfaceTest: public ::testing::Test
{
protected:
    TableInterfaceTest()
    : maxTableHandle((xme_hal_table_rowHandle_t)XME_MAX_SYSTEM_VALUE)
    {
        XME_HAL_TABLE_INIT(myTable);
    }

    virtual ~TableInterfaceTest()
    {
        XME_HAL_TABLE_FINI(myTable);
    }

    XME_HAL_TABLE(double, myTable, 3);

    const xme_hal_table_rowHandle_t maxTableHandle;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     TableInterfaceTest                                                     //
//----------------------------------------------------------------------------//

TEST_F(TableInterfaceTest, initializeInitialized)
{
    // TODO: This should return a status code
    XME_HAL_TABLE_INIT(myTable);
}

TEST_F(TableInterfaceTest, countEmptyTable)
{
    EXPECT_EQ(0, XME_HAL_TABLE_ITEM_COUNT(myTable));
}

TEST_F(TableInterfaceTest, clearEmptyTable)
{
    XME_HAL_TABLE_CLEAR(myTable);
    EXPECT_EQ(0, XME_HAL_TABLE_ITEM_COUNT(myTable));
}

TEST_F(TableInterfaceTest, removeFromEmptyTable)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, XME_HAL_TABLE_REMOVE_ITEM(myTable, (xme_hal_table_rowHandle_t)0));
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, XME_HAL_TABLE_REMOVE_ITEM(myTable, (xme_hal_table_rowHandle_t)1));

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, XME_HAL_TABLE_REMOVE_ITEM(myTable, (xme_hal_table_rowHandle_t)(maxTableHandle-1)));
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, XME_HAL_TABLE_REMOVE_ITEM(myTable, maxTableHandle));
}

TEST_F(TableInterfaceTest, addToEmptyTable)
{
    xme_hal_table_rowHandle_t h1, h2, h3, h4;
    double* d;
    char zeroes[sizeof(double)] = { 0 };

    h1 = XME_HAL_TABLE_ADD_ITEM(myTable);
    EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h1);
    EXPECT_EQ(1, XME_HAL_TABLE_ITEM_COUNT(myTable));

    d = XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h1);
    ASSERT_TRUE(NULL != d);
    EXPECT_EQ(0, xme_fallback_memcmp(zeroes, d, sizeof(zeroes)));

    h2 = XME_HAL_TABLE_ADD_ITEM(myTable);
    EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h2);
    EXPECT_NE(h1, h2);
    EXPECT_EQ(2, XME_HAL_TABLE_ITEM_COUNT(myTable));

    d = XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h2);
    ASSERT_TRUE(NULL != d);
    EXPECT_EQ(0, xme_fallback_memcmp(zeroes, d, sizeof(zeroes)));

    h3 = XME_HAL_TABLE_ADD_ITEM(myTable);
    EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h3);
    EXPECT_NE(h1, h3);
    EXPECT_NE(h2, h3);
    EXPECT_EQ(3, XME_HAL_TABLE_ITEM_COUNT(myTable));

    d = XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h3);
    ASSERT_TRUE(NULL != d);
    EXPECT_EQ(0, xme_fallback_memcmp(zeroes, d, sizeof(zeroes)));

#ifdef XME_HAL_TABLE_STATIC_ALLOC

    // For static tables, insertion of the 4th item fails...
    EXPECT_XME_ASSERTION_FAILURE(h4 = XME_HAL_TABLE_ADD_ITEM(myTable));
    EXPECT_EQ(XME_HAL_TABLE_INVALID_ROW_HANDLE, h4);
    EXPECT_EQ(3, XME_HAL_TABLE_ITEM_COUNT(myTable));

    d = XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h4);
    ASSERT_TRUE(NULL == d);

#else // #ifdef XME_HAL_TABLE_STATIC_ALLOC

    // ...while it works for dynamic tables
    EXPECT_NO_XME_ASSERTION_FAILURES(h4 = XME_HAL_TABLE_ADD_ITEM(myTable));
    EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h4);
    EXPECT_EQ(4, XME_HAL_TABLE_ITEM_COUNT(myTable));

    d = XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h4);
    ASSERT_TRUE(NULL != d);
    EXPECT_EQ(0, xme_fallback_memcmp(zeroes, d, sizeof(zeroes)));

#endif // #ifdef XME_HAL_TABLE_STATIC_ALLOC
}

TEST_F(TableInterfaceTest, addAndRemoveAndAddAndCheckZeroMemory)
{
    xme_hal_table_rowHandle_t h1, h2;
    double* d;
    char zeroes[sizeof(double)] = { 0 };

    h1 = XME_HAL_TABLE_ADD_ITEM(myTable);
    EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h1);
    EXPECT_EQ(1, XME_HAL_TABLE_ITEM_COUNT(myTable));

    d = XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h1);
    ASSERT_TRUE(NULL != d);
    EXPECT_EQ(0, xme_fallback_memcmp(zeroes, d, sizeof(zeroes)));

    *d = 1.2345678;

    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h1));
    EXPECT_EQ(0, XME_HAL_TABLE_ITEM_COUNT(myTable));

    h2 = XME_HAL_TABLE_ADD_ITEM(myTable);
    EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h2);
    EXPECT_EQ(1, XME_HAL_TABLE_ITEM_COUNT(myTable));

    EXPECT_EQ(h1, h2);

    d = XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h1);
    ASSERT_TRUE(NULL != d);
    EXPECT_EQ(0, xme_fallback_memcmp(zeroes, d, sizeof(zeroes)));
}

TEST_F(TableInterfaceTest, itemFromEmptyTable)
{
    EXPECT_EQ(NULL, XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, (xme_hal_table_rowHandle_t)0));
    EXPECT_EQ(NULL, XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, (xme_hal_table_rowHandle_t)1));

    EXPECT_EQ(NULL, XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, (xme_hal_table_rowHandle_t)XME_MAX_SYSTEM_VALUE-1));
    EXPECT_EQ(NULL, XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, (xme_hal_table_rowHandle_t)XME_MAX_SYSTEM_VALUE));
}

TEST_F(TableInterfaceTest, iterateOverEmptyTable)
{
    double sum = 0;
    int count = 0;

    XME_HAL_TABLE_ITERATE_BEGIN(myTable, xme_hal_table_rowHandle_t, h, double, d);
    {
        sum += *d;
        count++;
    }
    XME_HAL_TABLE_ITERATE_END();

    EXPECT_EQ(0, sum);
    EXPECT_EQ(0, count);
}

TEST_F(TableInterfaceTest, iterateReverseOverEmptyTable)
{
    double sum = 0;
    int count = 0;

    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(myTable, xme_hal_table_rowHandle_t, h, double, d);
    {
        sum += *d;
        count++;
    }
    XME_HAL_TABLE_ITERATE_REVERSE_END();

    EXPECT_EQ(0, sum);
    EXPECT_EQ(0, count);
}

TEST_F(TableInterfaceTest, getNextWithEmptyTableAndTrueCondition)
{
    xme_hal_table_rowHandle_t h = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    double* d = (double*)0x1234;

    XME_HAL_TABLE_GET_NEXT(myTable, xme_hal_table_rowHandle_t, h, double*, d, 1);
    EXPECT_EQ(XME_HAL_TABLE_INVALID_ROW_HANDLE, h);
    EXPECT_EQ(NULL, d);
}

TEST_F(TableInterfaceTest, getNextWithEmptyTableAndFalseCondition)
{
    xme_hal_table_rowHandle_t h = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    double* d = (double*)0x1234;

    XME_HAL_TABLE_GET_NEXT(myTable, xme_hal_table_rowHandle_t, h, double*, d, 0);
    EXPECT_EQ(XME_HAL_TABLE_INVALID_ROW_HANDLE, h);
    EXPECT_EQ(NULL, d);
}

TEST_F(TableInterfaceTest, getNextWithEmptyTableAndInvalidHandleAndTrueCondition)
{
    xme_hal_table_rowHandle_t h = (xme_hal_table_rowHandle_t)1; // invalid handle!
    double* d = (double*)0x1234;

    XME_HAL_TABLE_GET_NEXT(myTable, xme_hal_table_rowHandle_t, h, double*, d, 1);
    EXPECT_EQ(XME_HAL_TABLE_INVALID_ROW_HANDLE, h);
    EXPECT_EQ(NULL, d);
}

TEST_F(TableInterfaceTest, getNextWithEmptyTableAndInvalidHandleAndFalseCondition)
{
    xme_hal_table_rowHandle_t h = (xme_hal_table_rowHandle_t)1; // invalid handle!
    double* d = (double*)0x1234;

    XME_HAL_TABLE_GET_NEXT(myTable, xme_hal_table_rowHandle_t, h, double*, d, 0);
    EXPECT_EQ(XME_HAL_TABLE_INVALID_ROW_HANDLE, h);
    EXPECT_EQ(NULL, d);
}

int compareDouble(double* value1, double* value2)
{
	double result = *value1 - *value2;

	if (0 > result)
	{
		return -1;
	}
	else if (0 < result)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

TEST_F(TableInterfaceTest, DISABLED_bubbleSort)
{
    double* d1;
    double* d2;
    double* d3;

    d1 = (double*)XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, XME_HAL_TABLE_ADD_ITEM(myTable));
    d2 = (double*)XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, XME_HAL_TABLE_ADD_ITEM(myTable));
    d3 = (double*)XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, XME_HAL_TABLE_ADD_ITEM(myTable));

    *d1 = 20;
    *d2 = 0;
    *d3 = -100;

    XME_HAL_TABLE_BUBBLESORT(myTable, double, compareDouble);

    EXPECT_EQ(d3, XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, (xme_hal_table_rowHandle_t)1));
    EXPECT_EQ(d2, XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, (xme_hal_table_rowHandle_t)2));
    EXPECT_EQ(d1, XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, (xme_hal_table_rowHandle_t)3));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
