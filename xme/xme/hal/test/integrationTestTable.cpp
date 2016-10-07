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
 * $Id: integrationTestTable.cpp 5129 2013-09-19 16:16:32Z geisinger $
 */

/**
 * \file
 *         Table integration tests.
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

class TableIntegrationTest: public ::testing::Test
{
protected:
    TableIntegrationTest()
    : h1(XME_HAL_TABLE_INVALID_ROW_HANDLE)
    , h2(XME_HAL_TABLE_INVALID_ROW_HANDLE)
    , h3(XME_HAL_TABLE_INVALID_ROW_HANDLE)
    , d1(NULL)
    , d2(NULL)
    , d3(NULL)
    {
        XME_HAL_TABLE_INIT(myTable);

        h1 = XME_HAL_TABLE_ADD_ITEM(myTable);
        h2 = XME_HAL_TABLE_ADD_ITEM(myTable);
        h3 = XME_HAL_TABLE_ADD_ITEM(myTable);

        // Assign values to table rows
        d1 = (double*)XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h1);
        EXPECT_TRUE(NULL != d1);
        *d1 = 1.0;

        d2 = (double*)XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h2);
        EXPECT_TRUE(NULL != d2);
        *d2 = 2.0;

        d3 = (double*)XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h3);
        EXPECT_TRUE(NULL != d3);
        *d3 = 3.0;
    }

    virtual ~TableIntegrationTest()
    {
        XME_HAL_TABLE_FINI(myTable);
    }

    XME_HAL_TABLE(double, myTable, 3);

    xme_hal_table_rowHandle_t h1;
    xme_hal_table_rowHandle_t h2;
    xme_hal_table_rowHandle_t h3;
    double* d1;
    double* d2;
    double* d3;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     TableInterfaceTest                                                     //
//----------------------------------------------------------------------------//

TEST_F(TableIntegrationTest, iterateAndRemoveAndIterate)
{
    xme_hal_table_rowHandle_t h4, h5, h6;
    double sum;
    int count;

    // Sum up table, after updating the sum, add 5 to row values
    sum = 0.0;
    count = 0;
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, uint16_t, i, double, item);
    {
        sum += *item;
        *item = *item + 5.0;
        count++;
    }
    XME_HAL_TABLE_ITERATE_END();
    EXPECT_DOUBLE_EQ(6.0, sum); // 1 + 2 + 3
    EXPECT_EQ(3, count);

    // Check if updating worked
    sum=0.0;
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, uint16_t, i, double, item);
    {
        sum += *item;
    }
    XME_HAL_TABLE_ITERATE_END();
    EXPECT_DOUBLE_EQ(21.0, sum); // (1+5) + (2+5) + (3+5)

    // Remove 2nd item
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h2));
    EXPECT_EQ(2, XME_HAL_TABLE_ITEM_COUNT(myTable));

    // Check sum again
    sum = 0.0;
    count = 0;
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, uint16_t, i, double, item);
    {
        sum += *item;
        count++;
    }
    XME_HAL_TABLE_ITERATE_END();
    EXPECT_DOUBLE_EQ(14.0, sum); // (1+5) + (3+5)
    EXPECT_EQ(2, count);

    // Add new item, will go to 2nd row
    h4 = XME_HAL_TABLE_ADD_ITEM(myTable);
    EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h4);
    EXPECT_NE(h1, h4);
    EXPECT_NE(h3, h4);
    EXPECT_EQ(3, XME_HAL_TABLE_ITEM_COUNT(myTable));

#ifdef XME_HAL_TABLE_STATIC_ALLOC

    // For static tables, this fails
    EXPECT_XME_ASSERTION_FAILURE(h5 = XME_HAL_TABLE_ADD_ITEM(myTable));
    EXPECT_EQ(XME_HAL_TABLE_INVALID_ROW_HANDLE, h5);
    EXPECT_EQ(3, XME_HAL_TABLE_ITEM_COUNT(myTable));

#else // #ifdef XME_HAL_TABLE_STATIC_ALLOC

    // For dynamic tables, this works well, ...
    EXPECT_NO_XME_ASSERTION_FAILURES(h5 = XME_HAL_TABLE_ADD_ITEM(myTable));
    EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h5);
    EXPECT_EQ(4, XME_HAL_TABLE_ITEM_COUNT(myTable));

    // ... but we revert the change in order to simply further tests.
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h5));
    EXPECT_EQ(3, XME_HAL_TABLE_ITEM_COUNT(myTable));
    h5 = XME_HAL_TABLE_INVALID_ROW_HANDLE;

#endif // #ifdef XME_HAL_TABLE_STATIC_ALLOC

    // Delete last row
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h3));
    EXPECT_EQ(2, XME_HAL_TABLE_ITEM_COUNT(myTable));

    // And add it again
    h6 = XME_HAL_TABLE_ADD_ITEM(myTable);
    EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h6);
    EXPECT_NE(h1, h6);
    EXPECT_NE(h4, h6);
    EXPECT_EQ(3, XME_HAL_TABLE_ITEM_COUNT(myTable));

    // Check value of 2nd row: should be still (1+5)
    EXPECT_DOUBLE_EQ(6.0, *XME_HAL_TABLE_ITEM_FROM_HANDLE(myTable, h1));

    // Clear the table
    xme_hal_table_clear(&myTable);
    EXPECT_EQ(0, XME_HAL_TABLE_ITEM_COUNT(myTable));
}

// Insert while iterating

TEST_F(TableIntegrationTest, insertAtEndWhileIteratingInFirstSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h3));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            h3 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h3);
        }
        else if (2 == iteration)
        {
            // Do nothing
        }
        else if (3 == iteration)
        {
            // Do nothing
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    EXPECT_EQ(3, iteration);
}

TEST_F(TableIntegrationTest, insertAtEndWhileIteratingInSecondSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h3));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            // Do nothing
        }
        else if (2 == iteration)
        {
            h3 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h3);
        }
        else if (3 == iteration)
        {
            // Do nothing
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    EXPECT_EQ(3, iteration);
}

TEST_F(TableIntegrationTest, insertAtFrontWhileIteratingInFirstSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h1));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            h1 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h1);
        }
        else if (2 == iteration)
        {
            // Do nothing
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    EXPECT_EQ(2, iteration);
}

TEST_F(TableIntegrationTest, insertAtFrontWhileIteratingInSecondSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h1));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            // Do nothing
        }
        else if (2 == iteration)
        {
            h1 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h1);
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    EXPECT_EQ(2, iteration);
}

TEST_F(TableIntegrationTest, insertAtMiddleWhileIteratingInFirstSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h2));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            h2 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h2);
        }
        else if (2 == iteration)
        {
            // Do nothing
        }
        else if (3 == iteration)
        {
            // Do nothing
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    EXPECT_EQ(3, iteration);
}

TEST_F(TableIntegrationTest, insertAtMiddleWhileIteratingInSecondSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h2));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            // Do nothing
        }
        else if (2 == iteration)
        {
            h2 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h2);
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    EXPECT_EQ(2, iteration);
}

// Insert while reverse iterating

TEST_F(TableIntegrationTest, insertAtEndWhileReverseIteratingInFirstSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h3));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            h3 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h3);
        }
        else if (2 == iteration)
        {
            // Do nothing
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_REVERSE_END();

    EXPECT_EQ(2, iteration);
}

TEST_F(TableIntegrationTest, insertAtEndWhileReverseIteratingInSecondSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h3));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            // Do nothing
        }
        else if (2 == iteration)
        {
            h3 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h3);
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_REVERSE_END();

    EXPECT_EQ(2, iteration);
}

TEST_F(TableIntegrationTest, insertAtFrontWhileReverseIteratingInFirstSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h1));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            h1 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h1);
        }
        else if (2 == iteration)
        {
            // Do nothing
        }
        else if (3 == iteration)
        {
            // Do nothing
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_REVERSE_END();

    EXPECT_EQ(3, iteration);
}

TEST_F(TableIntegrationTest, insertAtFrontWhileReverseIteratingInSecondSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h1));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            // Do nothing
        }
        else if (2 == iteration)
        {
            h1 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h1);
        }
        else if (3 == iteration)
        {
            // Do nothing
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_REVERSE_END();

    EXPECT_EQ(3, iteration);
}

TEST_F(TableIntegrationTest, insertAtMiddleWhileReverseIteratingInFirstSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h2));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            h2 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h2);
        }
        else if (2 == iteration)
        {
            // Do nothing
        }
        else if (3 == iteration)
        {
            // Do nothing
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_REVERSE_END();

    EXPECT_EQ(3, iteration);
}

TEST_F(TableIntegrationTest, insertAtMiddleWhileReverseIteratingInSecondSlot)
{
    int iteration = 0;

    // Remove last item in order to be able to insert it later
    // even if the table is statically allocated
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, h2));

    // Add an item while iterating
    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        iteration++;

        if (1 == iteration)
        {
            // Do nothing
        }
        else if (2 == iteration)
        {
            h2 = XME_HAL_TABLE_ADD_ITEM(myTable);
            EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h2);
        }
        else
        {
            EXPECT_TRUE(false);
        }
    }
    XME_HAL_TABLE_ITERATE_REVERSE_END();

    EXPECT_EQ(2, iteration);
}

// Remove while iterating

TEST_F(TableIntegrationTest, removeWhileIterating)
{
    uint16_t count = XME_HAL_TABLE_ITEM_COUNT(myTable);

    // Remove items while iterating
    XME_HAL_TABLE_ITERATE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, handle));
        EXPECT_EQ(--count, XME_HAL_TABLE_ITEM_COUNT(myTable));
    }
    XME_HAL_TABLE_ITERATE_END();
}

// Remove while reverse iterating

TEST_F(TableIntegrationTest, removeWhileReverseIterating)
{
    uint16_t count = XME_HAL_TABLE_ITEM_COUNT(myTable);

    // Remove items while reverse iterating
    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(myTable, xme_hal_table_rowHandle_t, handle, double, item);
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_TABLE_REMOVE_ITEM(myTable, handle));
        EXPECT_EQ(--count, XME_HAL_TABLE_ITEM_COUNT(myTable));
    }
    XME_HAL_TABLE_ITERATE_REVERSE_END();
}

// Stress tests

TEST_F(TableIntegrationTest, stressTest)
{
    const uint16_t maxItems = 2048;
    uint32_t i;

    XME_HAL_TABLE(uint16_t, t, maxItems);

    XME_HAL_TABLE_INIT(t);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_table_clear(&t));

    for (i = 0U; i < maxItems; i++)
    {
        xme_hal_table_rowHandle_t h;
        uint16_t* n;

        h = XME_HAL_TABLE_ADD_ITEM(t);
        EXPECT_NE(XME_HAL_TABLE_INVALID_ROW_HANDLE, h);

        n = XME_HAL_TABLE_ITEM_FROM_HANDLE(t, h);
        ASSERT_TRUE(NULL != n);
        EXPECT_EQ(0U, *n);

        *n = i + 1;
    }

    EXPECT_EQ(maxItems, XME_HAL_TABLE_ITEM_COUNT(t));

    i = 0U;
    XME_HAL_TABLE_ITERATE_BEGIN(t, xme_hal_table_rowHandle_t, h, uint16_t, n);
    {
        ASSERT_TRUE(NULL != n);
        EXPECT_EQ(i + 1, *n);

        i++;
    }
    XME_HAL_TABLE_ITERATE_END();

    EXPECT_EQ(maxItems, i);

    i = maxItems;
    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(t, xme_hal_table_rowHandle_t, h, uint16_t, n);
    {
        ASSERT_TRUE(NULL != n);
        EXPECT_EQ(i, *n);

        i--;
    }
    XME_HAL_TABLE_ITERATE_REVERSE_END();

    EXPECT_EQ(0U, i);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_table_clear(&t));

    EXPECT_EQ(0, XME_HAL_TABLE_ITEM_COUNT(t));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
