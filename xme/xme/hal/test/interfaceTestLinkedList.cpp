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
 * $Id: interfaceTestLinkedList.cpp 5572 2013-10-22 17:29:28Z geisinger $
 */

/**
 * \file
 *         Linked list interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/testUtils.h"

#include "xme/hal/include/linkedList.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class LinkedListInterfaceTest: public ::testing::Test
{
protected:
    LinkedListInterfaceTest()
    {
    }

    virtual ~LinkedListInterfaceTest()
    {
    }

    static int
    alwaysHereInsertionCallback
    (
        const void* const item,
        const void* const currentItem,
        const void* userData
    )
    {
        XME_UNUSED_PARAMETER(item);
        XME_UNUSED_PARAMETER(currentItem);

        EXPECT_EQ((void*) 0x1234, userData);

        // 0 means "insert in front of currentItem"
        return 0;
    }

    static int
    alwaysAfterInsertionCallback
    (
        const void* const item,
        const void* const currentItem,
        const void* userData
    )
    {
        XME_UNUSED_PARAMETER(item);
        XME_UNUSED_PARAMETER(currentItem);

        EXPECT_EQ((void*) 0x1234, userData);

        // 1 means "insert at a later position"
        return 1;
    }
};

class SinglyLinkedListInterfaceTest: public LinkedListInterfaceTest
{
protected:
    SinglyLinkedListInterfaceTest()
    {
        XME_HAL_SINGLYLINKEDLIST_INIT(myList);
    }

    virtual ~SinglyLinkedListInterfaceTest()
    {
#if defined(DEBUG) && !defined(DOXYGEN)
        // Do not provoke a misleading assertion failure
        if (XME_HAL_LINKEDLIST_MAGIC == myList._isInitialized)
#endif // #if defined(DEBUG) && !defined(DOXYGEN)
        {
            EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_singlyLinkedList_fini(&myList));
        }
    }

    xme_hal_singlyLinkedList_t(6U) myList;
};

class DoublyLinkedListInterfaceTest: public LinkedListInterfaceTest
{
protected:
    DoublyLinkedListInterfaceTest()
    {
        XME_HAL_DOUBLYLINKEDLIST_INIT(myList);
    }

    virtual ~DoublyLinkedListInterfaceTest()
    {
#if defined(DEBUG) && !defined(DOXYGEN)
        // Do not provoke a misleading assertion failure
        if (XME_HAL_LINKEDLIST_MAGIC == myList._isInitialized)
#endif // #if defined(DEBUG) && !defined(DOXYGEN)
        {
            EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_doublyLinkedList_fini(&myList));
        }
    }

    xme_hal_doublyLinkedList_t(6U) myList;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     LinkedListInterfaceTest                                                //
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
//     SinglyLinkedListInterfaceTest                                          //
//----------------------------------------------------------------------------//

TEST_F(SinglyLinkedListInterfaceTest, initializeInitialized)
{
    // TODO: Actually return a status code!
    //EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_INIT(myList)));

    EXPECT_NO_XME_ASSERTION_FAILURES(XME_HAL_SINGLYLINKEDLIST_INIT(myList));
}

TEST_F(SinglyLinkedListInterfaceTest, finalizeInitialized)
{
    // TODO: Actually return a status code!
    //EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_fini(myList));

    xme_hal_singlyLinkedList_fini(&myList);
}

TEST_F(SinglyLinkedListInterfaceTest, finalizeInitializedDeprecated)
{
    XME_HAL_SINGLYLINKEDLIST_FINI(myList);
}

TEST_F(SinglyLinkedListInterfaceTest, clearInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_clear(&myList)));
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListInterfaceTest, getItemCountInitialized)
{
    EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList));
}

TEST_F(SinglyLinkedListInterfaceTest, getItemCountInitializedDeprecated)
{
    EXPECT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList));
}

TEST_F(SinglyLinkedListInterfaceTest, itemFromIndexNonExistingFromInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 1U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0x1234U)));
}

TEST_F(SinglyLinkedListInterfaceTest, itemFromIndexNonExistingFromInitializedDeprecated)
{
    EXPECT_EQ(NULL, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
    EXPECT_EQ(NULL, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
    EXPECT_EQ(NULL, XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0x1234U));
}

TEST_F(SinglyLinkedListInterfaceTest, removeNonExistingFromInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, false)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, true)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, (void*) 0x1234, false)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, (void*) 0x1234, true)));
}

TEST_F(SinglyLinkedListInterfaceTest, removeNonExistingFromInitializedDeprecated)
{
    EXPECT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, NULL, false));
    EXPECT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, NULL, true));
    EXPECT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, (void*) 0x1234, false));
    EXPECT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(myList, (void*) 0x1234, true));
}

TEST_F(SinglyLinkedListInterfaceTest, addItemToInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, (void*) 0x5678)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&myList, NULL)));
}

TEST_F(SinglyLinkedListInterfaceTest, addItemToInitializedDeprecated)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, (void*) 0x1234));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, (void*) 0x5678));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(myList, NULL));
}

TEST_F(SinglyLinkedListInterfaceTest, addItemOrderedToInitializedWithInvalidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(&myList, (void*) 0x1234, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(&myList, (void*) 0x5678, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, NULL, (void*) 0x1234)));
}

TEST_F(SinglyLinkedListInterfaceTest, addItemOrderedToInitializedWithAlwaysAfterCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, (void*) 0x1234, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, (void*) 0x5678, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, &alwaysAfterInsertionCallback, (void*) 0x1234)));
}

TEST_F(SinglyLinkedListInterfaceTest, addItemOrderedToInitializedWithAlwaysHereCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, (void*) 0x1234, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, (void*) 0x5678, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemOrdered(&myList, NULL, &alwaysHereInsertionCallback, (void*) 0x1234)));
}

TEST_F(SinglyLinkedListInterfaceTest, addItemSortedToInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, (void*) 0x1234, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, (void*) 0x5678, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
}

TEST_F(SinglyLinkedListInterfaceTest, addItemSortedWithInvalidCompareSizeToInitialized)
{
    uint64_t item[2] = { 0ULL };
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemSorted(&myList, &item, 0U, 9U)));
}

TEST_F(SinglyLinkedListInterfaceTest, addItemSortedToInitializedDeprecated)
{
    // No test possible, because this list does not contain meaningful data
}

TEST_F(SinglyLinkedListInterfaceTest, finalizeFinalized)
{
    // TODO: Actually return a status code!
    //ASSERT_NO_XME_ASSERTION_FAILURESASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_fini(myList)));
    //EXPECT_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_fini(myList)));

    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_singlyLinkedList_fini(&myList));
    EXPECT_XME_ASSERTION_FAILURES(xme_hal_singlyLinkedList_fini(&myList));
}

TEST_F(SinglyLinkedListInterfaceTest, finalizeFinalizedDeprecated)
{
    ASSERT_NO_XME_ASSERTION_FAILURES(XME_HAL_SINGLYLINKEDLIST_FINI(myList));
    EXPECT_XME_ASSERTION_FAILURES(XME_HAL_SINGLYLINKEDLIST_FINI(myList));
}

//----------------------------------------------------------------------------//
//     DoublyLinkedListInterfaceTest                                          //
//----------------------------------------------------------------------------//

TEST_F(DoublyLinkedListInterfaceTest, initializeInitialized)
{
    // TODO: Actually return a status code!
    //EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_INIT(myList)));

    EXPECT_NO_XME_ASSERTION_FAILURES(XME_HAL_DOUBLYLINKEDLIST_INIT(myList));
}

TEST_F(DoublyLinkedListInterfaceTest, finalizeInitialized)
{
    // TODO: Actually return a status code!
    //EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_fini(myList));

    xme_hal_doublyLinkedList_fini(&myList);
}

TEST_F(DoublyLinkedListInterfaceTest, finalizeInitializedDeprecated)
{
    XME_HAL_DOUBLYLINKEDLIST_FINI(myList);
}

TEST_F(DoublyLinkedListInterfaceTest, clearInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_clear(&myList)));
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListInterfaceTest, getItemCountInitialized)
{
    EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList));
}

TEST_F(DoublyLinkedListInterfaceTest, getItemCountInitializedDeprecated)
{
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList));
}

TEST_F(DoublyLinkedListInterfaceTest, itemFromIndexNonExistingFromInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 1U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0x1234U)));
}

TEST_F(DoublyLinkedListInterfaceTest, itemFromIndexNonExistingFromInitializedDeprecated)
{
    EXPECT_EQ(NULL, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0U));
    EXPECT_EQ(NULL, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 1U));
    EXPECT_EQ(NULL, XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(myList, 0x1234U));
}

TEST_F(DoublyLinkedListInterfaceTest, removeNonExistingFromInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, false)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, true)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, (void*) 0x1234, false)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, (void*) 0x1234, true)));
}

TEST_F(DoublyLinkedListInterfaceTest, removeNonExistingFromInitializedDeprecated)
{
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, NULL, false));
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, NULL, true));
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, (void*) 0x1234, false));
    EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(myList, (void*) 0x1234, true));
}

TEST_F(DoublyLinkedListInterfaceTest, addItemToInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, (void*) 0x5678)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItem(&myList, NULL)));
}

TEST_F(DoublyLinkedListInterfaceTest, addItemToInitializedDeprecated)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, (void*) 0x1234));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, NULL));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, (void*) 0x5678));
    EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(myList, NULL));
}

TEST_F(DoublyLinkedListInterfaceTest, addItemOrderedToInitializedWithInvalidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(&myList, (void*) 0x1234, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(&myList, (void*) 0x5678, NULL, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, NULL, (void*) 0x1234)));
}

TEST_F(DoublyLinkedListInterfaceTest, addItemOrderedToInitializedWithAlwaysAfterCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, (void*) 0x1234, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, (void*) 0x5678, &alwaysAfterInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, &alwaysAfterInsertionCallback, (void*) 0x1234)));
}

TEST_F(DoublyLinkedListInterfaceTest, addItemOrderedToInitializedWithAlwaysHereCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, (void*) 0x1234, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, (void*) 0x5678, &alwaysHereInsertionCallback, (void*) 0x1234)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemOrdered(&myList, NULL, &alwaysHereInsertionCallback, (void*) 0x1234)));
}

TEST_F(DoublyLinkedListInterfaceTest, addItemSortedToInitialized)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, (void*) 0x1234, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, (void*) 0x5678, 0U, 0U)));
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
}

TEST_F(DoublyLinkedListInterfaceTest, addItemSortedWithInvalidCompareSizeToInitialized)
{
    uint64_t item[2] = { 0ULL };
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemSorted(&myList, &item, 0U, 9U)));
}

TEST_F(DoublyLinkedListInterfaceTest, addItemToInitializedSortedDeprecated)
{
    // No test possible, because this list does not contain meaningful data
}

TEST_F(DoublyLinkedListInterfaceTest, finalizeFinalized)
{
    // TODO: Actually return a status code!
    //ASSERT_NO_XME_ASSERTION_FAILURES(ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_fini(myList)));
    //EXPECT_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_doublyLinkedList_fini(myList)));

    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_doublyLinkedList_fini(&myList));
    EXPECT_XME_ASSERTION_FAILURES(xme_hal_doublyLinkedList_fini(&myList));
}

TEST_F(DoublyLinkedListInterfaceTest, finalizeFinalizedDeprecated)
{
    ASSERT_NO_XME_ASSERTION_FAILURES(XME_HAL_DOUBLYLINKEDLIST_FINI(myList));
    EXPECT_XME_ASSERTION_FAILURES(XME_HAL_DOUBLYLINKEDLIST_FINI(myList));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
