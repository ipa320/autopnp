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
 * $Id: integrationTestLinkedList.cpp 7829 2014-03-14 10:29:33Z ruiz $
 */

/**
 * \file
 *         Linked list integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/defines.h"

#include "xme/core/testUtils.h"

#include "xme/hal/include/linkedList.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define LINKEDLIST_CAPACITY (6U)

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class LinkedListIntegrationTest: public ::testing::Test
{
protected:
    typedef struct data_t
    {
        uint16_t priority;
        uint32_t value;
        uint64_t utility;
    }
    data_t;

    LinkedListIntegrationTest()
    : d1(1.0)
    , d2(2.0)
    , d3(3.0)
    {
        data1.priority = 1U;
        data1.value = 10U;
        data1.utility = 100000000000ULL;

        data2.priority = 2U;
        data2.value = 15U;
        data2.utility = 200000000000ULL;

        data3.priority = 3U;
        data3.value = 5U;
        data3.utility = 300000000000ULL;

        data4.priority = 0U;
        data4.value = 25U;
        data4.utility = 0U;

        data5.priority = 2U;
        data5.value = 15U;
        data5.utility = 200000000000ULL;

        data6.priority = 2U;
        data6.value = 15U;
        data6.utility = 200000000000ULL;
    }

    virtual ~LinkedListIntegrationTest()
    {
    }

    static int
    sortByPriorityInsertionCallback
    (
        const void* const item,
        const void* const currentItem,
        const void* userData
    )
    {
        data_t* dataItem = (data_t*) item;
        data_t* currentDataItem = (data_t*) currentItem;

        numInsertionCallbackCalls++;

        EXPECT_TRUE(NULL != dataItem);
        EXPECT_EQ((void*) 0x1234, userData);

        // Semantics: insert just in front of an item with a higher priority
        // or at the end of the list if no higher priority item exists

        // 0 means "insert in front of currentItem"
        // 1 means "insert at a later position"
        return (dataItem->priority < currentDataItem->priority) ? 0 : 1;
    }

    static int
    sortByPriorityNoDuplicatesInsertionCallback
    (
        const void* const item,
        const void* const currentItem,
        const void* userData
    )
    {
        data_t* dataItem = (data_t*) item;
        data_t* currentDataItem = (data_t*) currentItem;

        numInsertionCallbackCalls++;

        EXPECT_TRUE(NULL != dataItem);
        EXPECT_EQ((void*) 0x1234, userData);

        // Semantics: insert just in front of an item with a higher priority,
        // or at the end of the list if no higher priority item exists,
        // but avoid duplicate insertion

        if (dataItem->priority == currentDataItem->priority) {
            // No duplicates
            return -1;
        }

        // 0 means "insert in front of currentItem"
        // 1 means "insert at a later position"
        return (dataItem->priority < currentDataItem->priority) ? 0 : 1;
    }

    static int
    sortByValueInsertionCallback
    (
        const void* const item,
        const void* const currentItem,
        const void* userData
    )
    {
        data_t* dataItem = (data_t*) item;
        data_t* currentDataItem = (data_t*) currentItem;

        numInsertionCallbackCalls++;

        EXPECT_TRUE(NULL != dataItem);
        EXPECT_EQ((void*) 0x1234, userData);

        // Semantics: insert just in front of an item with a higher value
        // or at the end of the list if no higher value item exists

        // 0 means "insert in front of currentItem"
        // 1 means "insert at a later position"
        return (dataItem->value < currentDataItem->value) ? 0 : 1;
    }

    static int
    sortByUtilityInsertionCallback
    (
        const void* const item,
        const void* const currentItem,
        const void* userData
    )
    {
        data_t* dataItem = (data_t*) item;
        data_t* currentDataItem = (data_t*) currentItem;

        numInsertionCallbackCalls++;

        EXPECT_TRUE(NULL != dataItem);
        EXPECT_EQ((void*) 0x1234, userData);

        // Semantics: insert just in front of an item with a higher utility
        // or at the end of the list if no higher utility item exists

        // 0 means "insert in front of currentItem"
        // 1 means "insert at a later position"
        return (dataItem->utility < currentDataItem->utility) ? 0 : 1;
    }

    void processSinglyLinkedList(xme_hal_linkedList_descriptor_t* singlyLinkedList)
    {
        uint16_t num;

        // Insert test items
        ASSERT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(singlyLinkedList));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(singlyLinkedList, &d1));
        ASSERT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(singlyLinkedList));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(singlyLinkedList, &d2));
        ASSERT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(singlyLinkedList));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(singlyLinkedList, &d3));
        ASSERT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(singlyLinkedList));

        // Iterate and remove the current item in iterator body
        num = 0U;
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*singlyLinkedList, double, item);
        {
            EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(singlyLinkedList, item, (bool) true));
            num++;
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

        EXPECT_EQ(3U, num);
        EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(singlyLinkedList));
    }

    void processDoublyLinkedList(xme_hal_linkedList_descriptor_t* doublyLinkedList)
    {
        uint16_t num;

        // Insert test items
        ASSERT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(doublyLinkedList));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(doublyLinkedList, &d1));
        ASSERT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(doublyLinkedList));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(doublyLinkedList, &d2));
        ASSERT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(doublyLinkedList));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(doublyLinkedList, &d3));
        ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(doublyLinkedList));

        // Iterate and remove the current item in iterator body
        num = 0U;
        XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(*doublyLinkedList, double, item);
        {
            EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(doublyLinkedList, item, true));
            num++;
        }
        XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();

        EXPECT_EQ(3U, num);
        EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(doublyLinkedList));
    }

    double d1;
    double d2;
    double d3;

    data_t data1;
    data_t data2;
    data_t data3;
    data_t data4;
    data_t data5;
    data_t data6;

    static uint16_t numInsertionCallbackCalls;
};

class SinglyLinkedListIntegrationTest: public LinkedListIntegrationTest
{
protected:
    SinglyLinkedListIntegrationTest()
    {
        XME_HAL_SINGLYLINKEDLIST_INIT(myList);
    }

    virtual ~SinglyLinkedListIntegrationTest()
    {
#if defined(DEBUG) && !defined(DOXYGEN)
        // Do not provoke a misleading assertion failure
        if (XME_HAL_LINKEDLIST_MAGIC == myList._isInitialized)
#endif // #if defined(DEBUG) && !defined(DOXYGEN)
        {
            xme_hal_singlyLinkedList_fini(&myList);
        }
    }

    xme_hal_singlyLinkedList_t(LINKEDLIST_CAPACITY) myList;
};

class DoublyLinkedListIntegrationTest: public LinkedListIntegrationTest
{
protected:
    DoublyLinkedListIntegrationTest()
    {
        XME_HAL_DOUBLYLINKEDLIST_INIT(myList);
    }

    virtual ~DoublyLinkedListIntegrationTest()
    {
#if defined(DEBUG) && !defined(DOXYGEN)
        // Do not provoke a misleading assertion failure
        if (XME_HAL_LINKEDLIST_MAGIC == myList._isInitialized)
#endif // #if defined(DEBUG) && !defined(DOXYGEN)
        {
            xme_hal_doublyLinkedList_fini(&myList);
        }
    }

    xme_hal_doublyLinkedList_t(LINKEDLIST_CAPACITY) myList;
};

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
uint16_t LinkedListIntegrationTest::numInsertionCallbackCalls = 0U;

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     LinkedListIntegrationTest                                              //
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
//     SinglyLinkedListIntegrationTest                                        //
//----------------------------------------------------------------------------//

TEST_F(SinglyLinkedListIntegrationTest, finalizeFinalized)
{
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_singlyLinkedList_fini(&myList));
    EXPECT_XME_ASSERTION_FAILURES(xme_hal_singlyLinkedList_fini(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, finalizeFinalizedDeprecated)
{
    ASSERT_NO_XME_ASSERTION_FAILURES(XME_HAL_SINGLYLINKEDLIST_FINI(myList));
    EXPECT_XME_ASSERTION_FAILURES(XME_HAL_SINGLYLINKEDLIST_FINI(myList));
}

TEST_F(SinglyLinkedListIntegrationTest, clearWithValidItems)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x12));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x34));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x56));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x78));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x9A));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0xBC));

    EXPECT_NO_XME_ASSERTION_FAILURES(ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_clear(&myList)));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, clearWithPartiallyNullItems)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x5678));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));

    EXPECT_NO_XME_ASSERTION_FAILURES(ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_clear(&myList)));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, insertOneNullItemAndRetrieveAndRemove)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));

    // Remove test items
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, insertTwoNullItemsAndRetrieveAndRemove)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));

    // Remove test items
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, insertTwoNullItemsAndRetrieveAndBatchRemove)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));

    // Remove test items
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, insertOneNonNullAndTwoNullItemsAndRetrieveAndRemoveNullItems)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ((void*) 0x1234, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));

    // Remove test items
    EXPECT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, insertOneNonNullAndTwoNullItemsAndRetrieveAndBatchRemoveNullItems)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ((void*) 0x1234, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));

    // Remove test items
    EXPECT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, insertManyAndRetrieveAndBatchRemove)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x5678));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) 0x1234, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) 0x5678, xme_hal_singlyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 5U));

    // Remove test items
    EXPECT_EQ(6U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(5U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, true));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, (void*) 0x5678, true));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, (void*) 0x1234, true));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, insertManyAndRetrieveAndBatchRemoveDeprecated)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, (void*) 0x5678));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
    EXPECT_EQ((void*) 0x1234, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
    EXPECT_EQ(NULL, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 2U));
    EXPECT_EQ(NULL, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 3U));
    EXPECT_EQ((void*) 0x5678, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 4U));
    EXPECT_EQ(NULL, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 5U));

    // Remove test items
    EXPECT_EQ(6U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) false));
    EXPECT_EQ(5U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) false));
    EXPECT_EQ(4U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) true));
    EXPECT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) false));
    EXPECT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) true));
    EXPECT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, (void*) 0x5678, (bool) true));
    EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, (void*) 0x1234, (bool) true));
    EXPECT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
}

TEST_F(SinglyLinkedListIntegrationTest, genericInsertAndIterateAndModifyAndIterate)
{
    const int add = 5;
    typedef struct
    {
       int i;
    }idStruct;
    idStruct id1, id2, id3;
    int sum;
    int expectedSum;
    int expectedSums[3];
    uint16_t num;
    id1.i = 1;
    id2.i = 2;
    id3.i = 3;

    // Insert test items
    ASSERT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &id1));
    ASSERT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &id2));
    ASSERT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &id3));
    ASSERT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Sum up linked list items, after updating the sum, add 'add' to items
    sum = 0U;
    num = 0U;
    expectedSum = id1.i + id2.i + id3.i;
    expectedSums[0] = id1.i;
    expectedSums[1] = expectedSums[0] + id2.i;
    expectedSums[2] = expectedSums[1] + id3.i;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, idStruct, item);
    {
        sum += (item->i);
        item->i = item->i + add;
        EXPECT_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Check if updating worked
    sum = 0U;
    num = 0U;
    expectedSum = id1.i + id2.i + id3.i;
    expectedSums[0] = id1.i;
    expectedSums[1] = expectedSums[0] + id2.i;
    expectedSums[2] = expectedSums[1] + id3.i;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, idStruct, item);
    {
        sum += (item->i);
        EXPECT_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Remove 2nd item
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, &id2, (bool) false));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Check sum again
    sum = 0U;
    num = 0U;
    expectedSum = id1.i + id3.i;
    expectedSums[0] = id1.i;
    expectedSums[1] = expectedSums[0] + id3.i;
    num = 0U;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, idStruct, item);
    {
        sum += (item->i);
        EXPECT_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_EQ(expectedSum, sum);
    EXPECT_EQ(2U, num);
}

TEST_F(SinglyLinkedListIntegrationTest, insertAndIterateAndModifyAndIterate)
{
    const double add = 5.0;

    double sum;
    double expectedSum;
    double expectedSums[3];
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Sum up linked list items, after updating the sum, add 'add' to items
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d3;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        *item = *item + add;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Check if updating worked
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d3;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Remove 2nd item
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, &d2, (bool) false));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Check sum again
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d3;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(2U, num);
}

TEST_F(SinglyLinkedListIntegrationTest, insertAndIterateAndModifyAndIterateDeprecated)
{
    const double add = 5.0;

    double sum;
    double expectedSum;
    double expectedSums[3];
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    // Sum up linked list items, after updating the sum, add 'add' to items
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d3;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        *item = *item + add;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Check if updating worked
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d3;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Remove 2nd item
    EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, &d2, (bool) false));
    EXPECT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    // Check sum again
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d3;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(2U, num);
}

TEST_F(SinglyLinkedListIntegrationTest, insertAndIterateWhileRemovingItems)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Iterate and remove the current item in iterator body
    num = 0U;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, item, (bool) true));
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    EXPECT_EQ(3U, num);
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListIntegrationTest, insertAndIterateWhileRemovingItemsDeprecated)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    // Iterate and remove the current item in iterator body
    num = 0U;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, item, (bool) true));
        num++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    EXPECT_EQ(3U, num);
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
}

TEST_F(SinglyLinkedListIntegrationTest, insertOrderedByPriority)
{
    // Insert items sorted by priority
    numInsertionCallbackCalls = 0U;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data1, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(0U, numInsertionCallbackCalls);
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data2, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(1U, numInsertionCallbackCalls);
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data3, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(3U, numInsertionCallbackCalls);
    EXPECT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data4, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(4U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data5, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(8U, numInsertionCallbackCalls);
    EXPECT_EQ(5U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data6, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(13U, numInsertionCallbackCalls);
    EXPECT_EQ(6U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_singlyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_singlyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data3, xme_hal_singlyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, insertSortedByPriority)
{
    // Insert items sorted by priority (16 bit data type)
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data1, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data2, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data3, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data4, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data5, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(5U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data6, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(6U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_singlyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_singlyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data3, xme_hal_singlyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, insertSortedByPriorityDeprecated)
{
    // Insert items sorted by priority (16 bit data type)
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data1, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data2, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data3, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data4, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(4U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data5, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(5U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data6, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(6U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
    EXPECT_EQ((void*) &data1, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
    EXPECT_EQ((void*) &data2, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 2U));
    EXPECT_EQ((void*) &data5, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 3U));
    EXPECT_EQ((void*) &data6, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 4U));
    EXPECT_EQ((void*) &data3, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, insertOrderedByPriorityNoDuplicates)
{
    // Insert items sorted by priority, but without duplicates
    numInsertionCallbackCalls = 0U;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data1, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(0U, numInsertionCallbackCalls);
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data2, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(1U, numInsertionCallbackCalls);
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data3, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(3U, numInsertionCallbackCalls);
    EXPECT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data4, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(4U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_ABORTED, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data5, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(7U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_ABORTED, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data6, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(10U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data3, xme_hal_singlyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ(        NULL,   xme_hal_singlyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ(        NULL,   xme_hal_singlyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, insertOrderedByValue)
{
    // Insert items sorted by value
    numInsertionCallbackCalls = 0U;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data1, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(0U, numInsertionCallbackCalls);
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data2, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(1U, numInsertionCallbackCalls);
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data3, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(2U, numInsertionCallbackCalls);
    EXPECT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data4, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(5U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data5, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(9U, numInsertionCallbackCalls);
    EXPECT_EQ(5U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data6, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(14U, numInsertionCallbackCalls);
    EXPECT_EQ(6U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data3, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_singlyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_singlyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data4, xme_hal_singlyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, insertSortedByValue)
{
    // Insert items sorted by value (32 bit data type)
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data1, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data2, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data3, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data4, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data5, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(5U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data6, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(6U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data3, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_singlyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_singlyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data4, xme_hal_singlyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, insertSortedByValueDeprecated)
{
    // Insert items sorted by value (32 bit data type)
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data1, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data2, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data3, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data4, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(4U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data5, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(5U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data6, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(6U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data3, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
    EXPECT_EQ((void*) &data1, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
    EXPECT_EQ((void*) &data2, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 2U));
    EXPECT_EQ((void*) &data5, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 3U));
    EXPECT_EQ((void*) &data6, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 4U));
    EXPECT_EQ((void*) &data4, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, insertOrderedByUtility)
{
    // Insert items sorted by utility
    numInsertionCallbackCalls = 0U;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data1, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(0U, numInsertionCallbackCalls);
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data2, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(1U, numInsertionCallbackCalls);
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data3, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(3U, numInsertionCallbackCalls);
    EXPECT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data4, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(4U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data5, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(8U, numInsertionCallbackCalls);
    EXPECT_EQ(5U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, &data6, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(13U, numInsertionCallbackCalls);
    EXPECT_EQ(6U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_singlyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_singlyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data3, xme_hal_singlyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, insertSortedByUtility)
{
    // Insert items sorted by utility (64 bit data type)
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data1, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(1U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data2, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data3, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data4, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(4U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data5, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(5U, xme_hal_singlyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, &data6, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(6U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_singlyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_singlyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_singlyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data3, xme_hal_singlyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, insertSortedByUtilityDeprecated)
{
    // Insert items sorted by utility (64 bit data type)
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data1, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(1U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data2, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data3, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data4, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(4U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data5, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(5U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data6, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(6U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
    EXPECT_EQ((void*) &data1, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
    EXPECT_EQ((void*) &data2, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 2U));
    EXPECT_EQ((void*) &data5, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 3U));
    EXPECT_EQ((void*) &data6, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 4U));
    EXPECT_EQ((void*) &data3, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 5U));
}

TEST_F(SinglyLinkedListIntegrationTest, exceedCapacity)
{
    for (uint16_t i=0; i<LINKEDLIST_CAPACITY; i++)
    {
        EXPECT_EQ(i, xme_hal_singlyLinkedList_getItemCount(&myList));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*)(uintptr_t)i));
    }
    EXPECT_EQ(LINKEDLIST_CAPACITY, xme_hal_singlyLinkedList_getItemCount(&myList));

#ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
    EXPECT_EQ(XME_STATUS_OUT_OF_RESOURCES, xme_hal_singlyLinkedList_addItem(&myList, (void*)(uintptr_t) 0xFFFF));
    EXPECT_EQ(LINKEDLIST_CAPACITY, xme_hal_singlyLinkedList_getItemCount(&myList));
#else // #ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*)(uintptr_t) 0xFFFF));
    EXPECT_EQ(LINKEDLIST_CAPACITY+1, xme_hal_singlyLinkedList_getItemCount(&myList));
#endif // #ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
}

TEST_F(SinglyLinkedListIntegrationTest, exceedCapacityDeprecated)
{
    for (uint16_t i=0; i<LINKEDLIST_CAPACITY; i++)
    {
        EXPECT_EQ(i, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
        EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, (void*)(uintptr_t)i));
    }
    EXPECT_EQ(LINKEDLIST_CAPACITY, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

#ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
    EXPECT_EQ(XME_STATUS_OUT_OF_RESOURCES, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, (void*)(uintptr_t) 0xFFFF));
    EXPECT_EQ(LINKEDLIST_CAPACITY, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
#else // #ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, (void*)(uintptr_t) 0xFFFF));
    EXPECT_EQ(LINKEDLIST_CAPACITY+1, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
#endif // #ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
}

// Tests passing a linked list to a function

TEST_F(SinglyLinkedListIntegrationTest, passListAsFunctionArgument)
{
    processSinglyLinkedList((xme_hal_linkedList_descriptor_t*) &myList);
}

// Various regression tests

TEST_F(SinglyLinkedListIntegrationTest, regressionTest1823)
{
    double newData = 5.0;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Iterate and remove + add
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item)
    {
        if (item == &d1)
        {
            EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, item, (bool) false));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &newData));
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    ASSERT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));
    xme_hal_singlyLinkedList_fini(&myList);
    XME_HAL_SINGLYLINKEDLIST_INIT(myList);
    ASSERT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Iterate and remove
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item)
    {
        if (item == &d1)
        {
            EXPECT_EQ(1U, xme_hal_singlyLinkedList_removeItem(&myList, item, (bool) false));
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    ASSERT_EQ(2U, xme_hal_singlyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &newData));
    ASSERT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));

}

TEST_F(SinglyLinkedListIntegrationTest, regressionTest1823Deprecated)
{
    double newData = 5.0;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    // Iterate and remove + add
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item)
    {
        if (item == &d1) 
        {
            XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, item, false);
            ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &newData));
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    ASSERT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    XME_HAL_SINGLYLINKEDLIST_FINI(myList);
    XME_HAL_SINGLYLINKEDLIST_INIT(myList);
    ASSERT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    // Iterate and remove
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item)
    {
        if (item == &d1) 
        {
            XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, item, (bool) false);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    ASSERT_EQ(2U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &newData));
    ASSERT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

}

TEST_F(SinglyLinkedListIntegrationTest, regressionTest2925)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_singlyLinkedList_getItemCount(&myList));

    // Count list iterations
    num = 0U;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        num++;
        item++;
        continue; // Regression test for Issue #2925
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_EQ(3U, num);
}

TEST_F(SinglyLinkedListIntegrationTest, regressionTest2925Deprecated)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));

    // Count list iterations
    num = 0U;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        num++;
        item++;
        continue; // Regression test for Issue #2925
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    EXPECT_EQ(3U, num);
}

//----------------------------------------------------------------------------//
//     DoublyLinkedListIntegrationTest                                        //
//----------------------------------------------------------------------------//

TEST_F(DoublyLinkedListIntegrationTest, finalizeFinalized)
{
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_doublyLinkedList_fini(&myList));
    EXPECT_XME_ASSERTION_FAILURES(xme_hal_doublyLinkedList_fini(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, finalizeFinalizedDeprecated)
{
    ASSERT_NO_XME_ASSERTION_FAILURES(XME_HAL_DOUBLYLINKEDLIST_FINI(myList));
    EXPECT_XME_ASSERTION_FAILURES(XME_HAL_DOUBLYLINKEDLIST_FINI(myList));
}

TEST_F(DoublyLinkedListIntegrationTest, clearWithValidItems)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x12));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x34));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x56));
/*    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x78));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x9A));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0xBC));*/

    EXPECT_NO_XME_ASSERTION_FAILURES(ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_clear(&myList)));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, clearWithPartiallyNullItems)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x5678));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));

    EXPECT_NO_XME_ASSERTION_FAILURES(ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_clear(&myList)));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertOneNullItemAndRetrieveAndRemove)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));

    // Remove test items
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertTwoNullItemsAndRetrieveAndRemove)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));

    // Remove test items
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertTwoNullItemsAndRetrieveAndBatchRemove)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));

    // Remove test items
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, true));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, false));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, true));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertOneNonNullAndTwoNullItemsAndRetrieveAndRemoveNullItems)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ((void*) 0x1234, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));

    // Remove test items
    EXPECT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertOneNonNullAndTwoNullItemsAndRetrieveAndBatchRemoveNullItems)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ((void*) 0x1234, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));

    // Remove test items
    EXPECT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertManyAndRetrieveAndBatchRemove)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x5678));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) 0x1234, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) 0x5678, xme_hal_doublyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 5U));

    // Remove test items
    EXPECT_EQ(6U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(5U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) false));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, (bool) true));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, (void*) 0x5678, (bool) true));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, (void*) 0x1234, (bool) true));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertManyAndRetrieveAndBatchRemoveDeprecated)
{
    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, (void*) 0x1234));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, (void*) 0x5678));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, NULL));

    // Retrieve test items
    EXPECT_EQ(NULL, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
    EXPECT_EQ((void*) 0x1234, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
    EXPECT_EQ(NULL, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 2U));
    EXPECT_EQ(NULL, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 3U));
    EXPECT_EQ((void*) 0x5678, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 4U));
    EXPECT_EQ(NULL, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 5U));

    // Remove test items
    EXPECT_EQ(6U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) false));
    EXPECT_EQ(5U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) false));
    EXPECT_EQ(4U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) true));
    EXPECT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) false));
    EXPECT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, NULL, (bool) true));
    EXPECT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, (void*) 0x5678, (bool) true));
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, (void*) 0x1234, (bool) true));
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
}

TEST_F(DoublyLinkedListIntegrationTest, genericInsertAndIterateAndModifyAndIterate)
{
    const int add = 5;
    typedef struct
    {
       int i;
    }idStruct;
    idStruct id1, id2, id3;
    int sum;
    int expectedSum;
    int expectedSums[3];
    uint16_t num;
    id1.i = 1;
    id2.i = 2;
    id3.i = 3;

    // Insert test items
    ASSERT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &id1));
    ASSERT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &id2));
    ASSERT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &id3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Sum up linked list items, after updating the sum, add 'add' to items
    sum = 0U;
    num = 0U;
    expectedSum = id1.i + id2.i + id3.i;
    expectedSums[0] = id1.i;
    expectedSums[1] = expectedSums[0] + id2.i;
    expectedSums[2] = expectedSums[1] + id3.i;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, idStruct, item);
    {
        sum += (item->i);
        item->i = item->i + add;
        EXPECT_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Check if updating worked
    sum = 0U;
    num = 0U;
    expectedSum = id1.i + id2.i + id3.i;
    expectedSums[0] = id1.i;
    expectedSums[1] = expectedSums[0] + id2.i;
    expectedSums[2] = expectedSums[1] + id3.i;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, idStruct, item);
    {
        sum += (item->i);
        EXPECT_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Remove 2nd item
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, &id2, (bool) false));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check sum again
    sum = 0U;
    num = 0U;
    expectedSum = id1.i + id3.i;
    expectedSums[0] = id1.i;
    expectedSums[1] = expectedSums[0] + id3.i;
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, idStruct, item);
    {
        sum += (item->i);
        EXPECT_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_EQ(expectedSum, sum);
    EXPECT_EQ(2U, num);
}

TEST_F(DoublyLinkedListIntegrationTest, insertAndIterateAndModifyAndIterate)
{
    const double add = 5.0;

    double sum;
    double expectedSum;
    double expectedSums[3];
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Sum up linked list items, after updating the sum, add 'add' to items
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d3;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        *item = *item + add;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Check if updating worked
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d3;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Remove 2nd item
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, &d2, (bool) false));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check sum again
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d3;
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(2U, num);
}

TEST_F(DoublyLinkedListIntegrationTest, insertAndIterateAndModifyAndIterateDeprecated)
{
    const double add = 5.0;

    double sum;
    double expectedSum;
    double expectedSums[3];
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Sum up linked list items, after updating the sum, add 'add' to items
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d3;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        *item = *item + add;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Check if updating worked
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d3;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Remove 2nd item
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, &d2, (bool) false));
    EXPECT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Check sum again
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d3;
    expectedSums[0] = d1;
    expectedSums[1] = expectedSums[0] + d3;
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(2U, num);
}

TEST_F(DoublyLinkedListIntegrationTest, insertAndIterateWhileRemovingItems)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Iterate and remove the current item in iterator body
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, item, (bool) true));
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();

    EXPECT_EQ(3U, num);
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertAndIterateWhileRemovingItemsDeprecated)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Iterate and remove the current item in iterator body
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, item, (bool) true));
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();

    EXPECT_EQ(3U, num);
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertOrderedByPriority)
{
    // Insert items sorted by priority
    numInsertionCallbackCalls = 0U;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data1, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(0U, numInsertionCallbackCalls);
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data2, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(1U, numInsertionCallbackCalls);
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data3, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(3U, numInsertionCallbackCalls);
    EXPECT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data4, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(4U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data5, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(8U, numInsertionCallbackCalls);
    EXPECT_EQ(5U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data6, &sortByPriorityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(13U, numInsertionCallbackCalls);
    EXPECT_EQ(6U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_doublyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_doublyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data3, xme_hal_doublyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, genericInsertAndIterateAndModifyAndIterateReverse)
{
    const int add = 5;
    typedef struct
    {
       int i;
    }idStruct;
    idStruct id1, id2, id3;
    int sum;
    int expectedSum;
    int expectedSums[3];
    uint16_t num;
    id1.i = 1;
    id2.i = 2;
    id3.i = 3;

    // Insert test items
    ASSERT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &id1));
    ASSERT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &id2));
    ASSERT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &id3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Sum up linked list items, after updating the sum, add 'add' to items
    sum = 0U;
    num = 0U;
    expectedSum = id1.i + id2.i + id3.i;
    expectedSums[0] = id3.i;
    expectedSums[1] = expectedSums[0] + id2.i;
    expectedSums[2] = expectedSums[1] + id1.i;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, idStruct, item);
    {
        sum += (item->i);
        item->i = item->i + add;
        EXPECT_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Check if updating worked
    sum = 0U;
    num = 0U;
    expectedSum = id1.i + id2.i + id3.i;
    expectedSums[0] = id3.i;
    expectedSums[1] = expectedSums[0] + id2.i;
    expectedSums[2] = expectedSums[1] + id1.i;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, idStruct, item);
    {
        sum += (item->i);
        EXPECT_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Remove 2nd item
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, &id2, (bool) false));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check sum again
    sum = 0U;
    num = 0U;
    expectedSum = id1.i + id3.i;
    expectedSums[0] = id3.i;
    expectedSums[1] = expectedSums[0] + id1.i;
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, idStruct, item);
    {
        sum += (item->i);
        EXPECT_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_EQ(expectedSum, sum);
    EXPECT_EQ(2U, num);
}
TEST_F(DoublyLinkedListIntegrationTest, insertAndIterateReverseAndModifyAndIterateReverse)
{
    const double add = 5.0;

    double sum;
    double expectedSum;
    double expectedSums[3];
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Sum up linked list items, after updating the sum, add 'add' to items
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d3;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d1;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        sum += *item;
        *item = *item + add;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Check if updating worked
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d3;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d1;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Remove 2nd item
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, &d2, (bool) false));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check sum again
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d3;
    expectedSums[0] = d3;
    expectedSums[1] = expectedSums[0] + d1;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(2U, num);
}

TEST_F(DoublyLinkedListIntegrationTest, insertAndIterateReverseAndModifyAndIterateReverseDeprecated)
{
    const double add = 5.0;

    double sum;
    double expectedSum;
    double expectedSums[3];
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Sum up linked list items, after updating the sum, add 'add' to items
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d3;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d1;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        sum += *item;
        *item = *item + add;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Check if updating worked
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d2 + d3;
    expectedSums[0] = d3;
    expectedSums[1] = expectedSums[0] + d2;
    expectedSums[2] = expectedSums[1] + d1;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(3U, num);

    // Remove 2nd item
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, &d2, (bool) false));
    EXPECT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Check sum again
    sum = 0.0;
    num = 0U;
    expectedSum = d1 + d3;
    expectedSums[0] = d3;
    expectedSums[1] = expectedSums[0] + d1;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        sum += *item;
        EXPECT_DOUBLE_EQ(expectedSums[num], sum);
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_DOUBLE_EQ(expectedSum, sum);
    EXPECT_EQ(2U, num);
}

TEST_F(DoublyLinkedListIntegrationTest, insertAndIterateReverseWhileRemovingItems)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Iterate and remove the current item in iterator body
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        EXPECT_EQ(1U, xme_hal_doublyLinkedList_removeItem(&myList, item, (bool) true));
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();

    EXPECT_EQ(3U, num);
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertAndIterateReverseWhileRemovingItemsDeprecated)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Iterate and remove the current item in iterator body
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, item, (bool) true));
        num++;
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();

    EXPECT_EQ(3U, num);
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
}

TEST_F(DoublyLinkedListIntegrationTest, insertSortedByPriority)
{
    // Insert items sorted by priority
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data1, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data2, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data3, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data4, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data5, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(5U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data6, offsetof(data_t, priority), sizeof(uint16_t)));
    EXPECT_EQ(6U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check position of list elements
   EXPECT_EQ((void*) &data4, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
   EXPECT_EQ((void*) &data1, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
   EXPECT_EQ((void*) &data2, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));
   EXPECT_EQ((void*) &data5, xme_hal_doublyLinkedList_itemFromIndex(&myList, 3U));
   EXPECT_EQ((void*) &data6, xme_hal_doublyLinkedList_itemFromIndex(&myList, 4U));
   EXPECT_EQ((void*) &data3, xme_hal_doublyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, insertSortedByPriorityDeprecated)
{
    // Insert items sorted by priority
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data1, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data2, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data3, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data4, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(4U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data5, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(5U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data6, offsetof(data_t, priority), uint16_t));
    EXPECT_EQ(6U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Check position of list elements
   EXPECT_EQ((void*) &data4, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
   EXPECT_EQ((void*) &data1, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
   EXPECT_EQ((void*) &data2, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 2U));
   EXPECT_EQ((void*) &data5, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 3U));
   EXPECT_EQ((void*) &data6, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 4U));
   EXPECT_EQ((void*) &data3, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, insertOrderedByPriorityNoDuplicates)
{
    // Insert items sorted by priority, but without duplicates
    numInsertionCallbackCalls = 0U;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data1, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(0U, numInsertionCallbackCalls);
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data2, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(1U, numInsertionCallbackCalls);
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data3, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(3U, numInsertionCallbackCalls);
    EXPECT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data4, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(4U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_ABORTED, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data5, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(7U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_ABORTED, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data6, &sortByPriorityNoDuplicatesInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(10U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data3, xme_hal_doublyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ(        NULL,   xme_hal_doublyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ(        NULL,   xme_hal_doublyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, insertOrderedByValue)
{
    // Insert items sorted by value
    numInsertionCallbackCalls = 0U;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data1, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(0U, numInsertionCallbackCalls);
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data2, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(1U, numInsertionCallbackCalls);
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data3, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(2U, numInsertionCallbackCalls);
    EXPECT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data4, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(5U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data5, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(9U, numInsertionCallbackCalls);
    EXPECT_EQ(5U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data6, &sortByValueInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(14U, numInsertionCallbackCalls);
    EXPECT_EQ(6U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data3, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_doublyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_doublyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data4, xme_hal_doublyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, insertSortedByValue)
{
    // Insert items sorted by value
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data1, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data2, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data3, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data4, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data5, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(5U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data6, offsetof(data_t, value), sizeof(uint32_t)));
    EXPECT_EQ(6U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check position of list elements
   EXPECT_EQ((void*) &data3, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
   EXPECT_EQ((void*) &data1, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
   EXPECT_EQ((void*) &data2, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));
   EXPECT_EQ((void*) &data5, xme_hal_doublyLinkedList_itemFromIndex(&myList, 3U));
   EXPECT_EQ((void*) &data6, xme_hal_doublyLinkedList_itemFromIndex(&myList, 4U));
   EXPECT_EQ((void*) &data4, xme_hal_doublyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, insertSortedByValueDeprecated)
{
    // Insert items sorted by value
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data1, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data2, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data3, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data4, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(4U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data5, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(5U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data6, offsetof(data_t, value), uint32_t));
    EXPECT_EQ(6U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Check position of list elements
   EXPECT_EQ((void*) &data3, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
   EXPECT_EQ((void*) &data1, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
   EXPECT_EQ((void*) &data2, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 2U));
   EXPECT_EQ((void*) &data5, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 3U));
   EXPECT_EQ((void*) &data6, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 4U));
   EXPECT_EQ((void*) &data4, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, insertOrderedByUtility)
{
    // Insert items sorted by utility
    numInsertionCallbackCalls = 0U;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data1, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(0U, numInsertionCallbackCalls);
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data2, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(1U, numInsertionCallbackCalls);
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data3, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(3U, numInsertionCallbackCalls);
    EXPECT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data4, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(4U, numInsertionCallbackCalls);
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data5, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(8U, numInsertionCallbackCalls);
    EXPECT_EQ(5U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, &data6, &sortByUtilityInsertionCallback, (void*) 0x1234));
    EXPECT_EQ(13U, numInsertionCallbackCalls);
    EXPECT_EQ(6U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_doublyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_doublyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data3, xme_hal_doublyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, insertSortedByUtility)
{
    // Insert items sorted by value
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data1, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(1U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data2, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data3, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data4, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(4U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data5, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(5U, xme_hal_doublyLinkedList_getItemCount(&myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, &data6, offsetof(data_t, utility), sizeof(uint64_t)));
    EXPECT_EQ(6U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U));
    EXPECT_EQ((void*) &data1, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U));
    EXPECT_EQ((void*) &data2, xme_hal_doublyLinkedList_itemFromIndex(&myList, 2U));
    EXPECT_EQ((void*) &data5, xme_hal_doublyLinkedList_itemFromIndex(&myList, 3U));
    EXPECT_EQ((void*) &data6, xme_hal_doublyLinkedList_itemFromIndex(&myList, 4U));
    EXPECT_EQ((void*) &data3, xme_hal_doublyLinkedList_itemFromIndex(&myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, insertSortedByUtilityDeprecated)
{
    // Insert items sorted by value
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data1, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(1U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data2, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data3, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data4, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(4U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data5, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(5U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(myList, &data6, offsetof(data_t, utility), uint64_t));
    EXPECT_EQ(6U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Check position of list elements
    EXPECT_EQ((void*) &data4, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
    EXPECT_EQ((void*) &data1, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
    EXPECT_EQ((void*) &data2, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 2U));
    EXPECT_EQ((void*) &data5, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 3U));
    EXPECT_EQ((void*) &data6, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 4U));
    EXPECT_EQ((void*) &data3, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 5U));
}

TEST_F(DoublyLinkedListIntegrationTest, exceedCapacity)
{
    for (uint16_t i=0; i<LINKEDLIST_CAPACITY; i++)
    {
        EXPECT_EQ(i, xme_hal_doublyLinkedList_getItemCount(&myList));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*)(uintptr_t)i));
    }
    EXPECT_EQ(LINKEDLIST_CAPACITY, xme_hal_doublyLinkedList_getItemCount(&myList));

#ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
    EXPECT_EQ(XME_STATUS_OUT_OF_RESOURCES, xme_hal_doublyLinkedList_addItem(&myList, (void*)(uintptr_t) 0xFFFF));
    EXPECT_EQ(LINKEDLIST_CAPACITY, xme_hal_doublyLinkedList_getItemCount(&myList));
#else // #ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*)(uintptr_t) 0xFFFF));
    EXPECT_EQ(LINKEDLIST_CAPACITY+1, xme_hal_doublyLinkedList_getItemCount(&myList));
#endif // #ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
}

TEST_F(DoublyLinkedListIntegrationTest, exceedCapacityDeprecated)
{
    for (uint16_t i=0; i<LINKEDLIST_CAPACITY; i++)
    {
        EXPECT_EQ(i, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
        EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, (void*)(uintptr_t)i));
    }
    EXPECT_EQ(LINKEDLIST_CAPACITY, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

#ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
    EXPECT_EQ(XME_STATUS_OUT_OF_RESOURCES, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, (void*)(uintptr_t) 0xFFFF));
    EXPECT_EQ(LINKEDLIST_CAPACITY, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
#else // #ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, (void*)(uintptr_t) 0xFFFF));
    EXPECT_EQ(LINKEDLIST_CAPACITY+1, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
#endif // #ifdef XME_HAL_LINKEDLIST_STATIC_ALLOC
}

// Tests passing a linked list to a function

TEST_F(DoublyLinkedListIntegrationTest, passListAsFunctionArgument)
{
    processDoublyLinkedList((xme_hal_linkedList_descriptor_t*) &myList);
}

// Various regression tests

TEST_F(DoublyLinkedListIntegrationTest, regressionTest1823)
{
    double newData = 5.0;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Iterate and remove + add
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item)
    {
        if (item == &d1) 
        {
            xme_hal_doublyLinkedList_removeItem(&myList, item, (bool) false);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &newData));
        }
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();

    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
    xme_hal_doublyLinkedList_fini(&myList);
    XME_HAL_DOUBLYLINKEDLIST_INIT(myList);
    ASSERT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Iterate and remove
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item)
    {
        if (item == &d1) 
        {
            xme_hal_doublyLinkedList_removeItem(&myList, item, (bool) false);
        }
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();

    ASSERT_EQ(2U, xme_hal_doublyLinkedList_getItemCount(&myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &newData));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListIntegrationTest, regressionTest1823Deprecated)
{
    double newData = 5.0;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Iterate and remove + add
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item)
    {
        if (item == &d1) 
        {
            XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, item, (bool) false);
            ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &newData));
        }
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();

    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    XME_HAL_DOUBLYLINKEDLIST_FINI(myList);
    XME_HAL_DOUBLYLINKEDLIST_INIT(myList);
    ASSERT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Iterate and remove
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item)
    {
        if (item == &d1) 
        {
            XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, item, (bool) false);
        }
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();

    ASSERT_EQ(2U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &newData));
    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
}

TEST_F(DoublyLinkedListIntegrationTest, regressionTest2925Forward)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Count list iterations
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        num++;
        item++;
        continue; // Regression test for Issue #2925
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_EQ(3U, num);
}

TEST_F(DoublyLinkedListIntegrationTest, regressionTest2925ForwardDeprecated)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Count list iterations
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, double, item);
    {
        num++;
        item++;
        continue; // Regression test for Issue #2925
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    EXPECT_EQ(3U, num);
}

TEST_F(DoublyLinkedListIntegrationTest, regressionTest2925Reverse)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, &d3));
    ASSERT_EQ(3U, xme_hal_doublyLinkedList_getItemCount(&myList));

    // Count list iterations
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        num++;
        item++;
        continue; // Regression test for Issue #2925
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_EQ(3U, num);
}

TEST_F(DoublyLinkedListIntegrationTest, regressionTest2925ReverseDeprecated)
{
    uint16_t num;

    // Insert test items
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d1));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d2));
    ASSERT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, &d3));
    ASSERT_EQ(3U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));

    // Count list iterations
    num = 0U;
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, double, item);
    {
        num++;
        item++;
        continue; // Regression test for Issue #2925
    }
    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    EXPECT_EQ(3U, num);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
