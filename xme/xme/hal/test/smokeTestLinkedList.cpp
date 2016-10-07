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
 * $Id: smokeTestLinkedList.cpp 7829 2014-03-14 10:29:33Z ruiz $
 */

/**
 * \file
 *         Linked list smoke tests.
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

class LinkedListSmokeTest: public ::testing::Test
{
protected:
    LinkedListSmokeTest()
    {
    }

    virtual ~LinkedListSmokeTest()
    {
    }

    static int
    insertionCallback
    (
        const void* const item,
        const void* const currentItem,
        const void* userData
    )
    {
        XME_UNUSED_PARAMETER(item);
        XME_UNUSED_PARAMETER(currentItem);
        XME_UNUSED_PARAMETER(userData);

        // This should never be called
        EXPECT_TRUE(false);

        // 1 means "insert at a later position"
        return 1;
    }
};

class SinglyLinkedListSmokeTest: public LinkedListSmokeTest
{
protected:
    SinglyLinkedListSmokeTest()
    {
#ifdef DEBUG
        // Ensure that magic number is not by chance set to the correct value,
        // in which case the tests would be nondeterministic
        if (XME_HAL_LINKEDLIST_MAGIC == myList._isInitialized)
        {
            myList._isInitialized++;
        }
#endif // #ifdef DEBUG
    }

    virtual ~SinglyLinkedListSmokeTest()
    {
    }

    xme_hal_singlyLinkedList_t(6U) myList;
};

class DoublyLinkedListSmokeTest: public LinkedListSmokeTest
{
protected:
    DoublyLinkedListSmokeTest()
    {
#ifdef DEBUG
        // Ensure that magic number is not by chance set to the correct value,
        // in which case the tests would be nondeterministic
        if (XME_HAL_LINKEDLIST_MAGIC == myList._isInitialized)
        {
            myList._isInitialized++;
        }
#endif // #ifdef DEBUG
    }

    virtual ~DoublyLinkedListSmokeTest()
    {
    }

    xme_hal_doublyLinkedList_t(6U) myList;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     LinkedListSmokeTest                                                    //
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
//     SinglyLinkedListSmokeTest                                              //
//----------------------------------------------------------------------------//

TEST_F(SinglyLinkedListSmokeTest, finiNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_singlyLinkedList_fini(NULL));
}

TEST_F(SinglyLinkedListSmokeTest, clearNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_clear(NULL)));
}

TEST_F(SinglyLinkedListSmokeTest, addNullItemToNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItem(NULL, NULL)));
}

TEST_F(SinglyLinkedListSmokeTest, addNonNullItemToNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItem(NULL, (void*) 0x1234)));
}

TEST_F(SinglyLinkedListSmokeTest, addNullItemToNullListOrderedWithInvalidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(NULL, NULL, NULL, (void*) 0x1234)));
}

TEST_F(SinglyLinkedListSmokeTest, addNullItemToNullListOrderedWithValidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(NULL, NULL, &insertionCallback, (void*) 0x1234)));
}

TEST_F(SinglyLinkedListSmokeTest, addNonNullItemToNullListOrderedWithInvalidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(NULL, (void*) 0x1234, NULL, (void*) 0x1234)));
}

TEST_F(SinglyLinkedListSmokeTest, addNonNullItemToNullListOrderedWithValidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemOrdered(NULL, (void*) 0x1234, &insertionCallback, (void*) 0x1234)));
}

TEST_F(SinglyLinkedListSmokeTest, addNullItemToNullListSortedWithZeroCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemSorted(NULL, NULL, 0U, 0U)));
}

TEST_F(SinglyLinkedListSmokeTest, addNullItemToNullListSortedWithOddCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemSorted(NULL, NULL, 0, 7U)));
}

TEST_F(SinglyLinkedListSmokeTest, addNullItemToNullListSortedWithTooLargeCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemSorted(NULL, NULL, 0U, 9U)));
}

TEST_F(SinglyLinkedListSmokeTest, addNonNullItemToNullListSortedWithZeroCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemSorted(NULL, (void*) 0x1234, 0U, 0U)));
}

TEST_F(SinglyLinkedListSmokeTest, addNonNullItemToNullListSortedWithOddCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemSorted(NULL, (void*) 0x1234, 0U, 7U)));
}

TEST_F(SinglyLinkedListSmokeTest, addNonNullItemToNullListSortedWithTooLargeCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_singlyLinkedList_addItemSorted(NULL, (void*) 0x1234, 0U, 9U)));
}

TEST_F(SinglyLinkedListSmokeTest, removeItemFromNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(NULL, (void*) 0x1234, (bool) false)));
}

TEST_F(SinglyLinkedListSmokeTest, removeAllItemsFromNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(NULL, (void*) 0x1234, (bool) true)));
}

TEST_F(SinglyLinkedListSmokeTest, itemFromInvalidIndexInNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(NULL, 0U)));
}

TEST_F(SinglyLinkedListSmokeTest, itemFromArbitraryIndexInNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(NULL, 42U)));
}

TEST_F(SinglyLinkedListSmokeTest, getItemCountFromNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(NULL)));
}

// -----

TEST_F(SinglyLinkedListSmokeTest, initializeUninitialized)
{
    // TODO: Actually return a status code!
    //EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_SINGLYLINKEDLIST_INIT(myList)));

    EXPECT_NO_XME_ASSERTION_FAILURES(XME_HAL_SINGLYLINKEDLIST_INIT(myList));
}

TEST_F(SinglyLinkedListSmokeTest, finiUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_singlyLinkedList_fini(&myList));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(SinglyLinkedListSmokeTest, addItemToUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(XME_STATUS_UNEXPECTED, xme_hal_singlyLinkedList_addItem(&myList, NULL)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(SinglyLinkedListSmokeTest, addItemSortedToUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(XME_STATUS_UNEXPECTED, xme_hal_singlyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(SinglyLinkedListSmokeTest, removeItemFromUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(0U, xme_hal_singlyLinkedList_removeItem(&myList, NULL, (bool) true)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(SinglyLinkedListSmokeTest, getItemCountUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(0U, xme_hal_singlyLinkedList_getItemCount(&myList)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(SinglyLinkedListSmokeTest, getItemCountUninitializedDeprecated)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(0U, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(myList)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(SinglyLinkedListSmokeTest, itemFromIndexUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(NULL, xme_hal_singlyLinkedList_itemFromIndex(&myList, 0U)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(SinglyLinkedListSmokeTest, iterateUninitialized)
{
#ifdef DEBUG
    EXPECT_XME_ASSERTION_FAILURE
    (
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(myList, void*, item);
        {
            // The test just ensures that iterating over an
            // uninitialized linked list does not lead to a crash
            XME_UNUSED_PARAMETER(item);
            break;
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    );
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

//----------------------------------------------------------------------------//
//     DoublyLinkedListSmokeTest                                              //
//----------------------------------------------------------------------------//

TEST_F(DoublyLinkedListSmokeTest, finiNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_doublyLinkedList_fini(NULL));
}

TEST_F(DoublyLinkedListSmokeTest, clearNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_clear(NULL)));
}

TEST_F(DoublyLinkedListSmokeTest, addNullItemToNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItem(NULL, NULL)));
}

TEST_F(DoublyLinkedListSmokeTest, addNonNullItemToNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItem(NULL, (void*) 0x1234)));
}

TEST_F(DoublyLinkedListSmokeTest, addNullItemToNullListOrderedWithInvalidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(NULL, NULL, NULL, NULL)));
}

TEST_F(DoublyLinkedListSmokeTest, addNullItemToNullListOrderedWithValidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(NULL, NULL, &insertionCallback, NULL)));
}

TEST_F(DoublyLinkedListSmokeTest, addNonNullItemToNullListOrderedWithInvalidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(NULL, (void*) 0x1234, NULL, NULL)));
}

TEST_F(DoublyLinkedListSmokeTest, addNonNullItemToNullListOrderedWithValidCallbackFunction)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemOrdered(NULL, (void*) 0x1234, &insertionCallback, NULL)));
}

TEST_F(DoublyLinkedListSmokeTest, addNullItemToNullListSortedWithZeroCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemSorted(NULL, NULL, 0U, 0U)));
}

TEST_F(DoublyLinkedListSmokeTest, addNullItemToNullListSortedWithOddCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemSorted(NULL, NULL, 0U, 7U)));
}

TEST_F(DoublyLinkedListSmokeTest, addNullItemToNullListSortedWithTooLargeCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemSorted(NULL, NULL, 0U, 9U)));
}

TEST_F(DoublyLinkedListSmokeTest, addNonNullItemToNullListSortedWithZeroCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemSorted(NULL, (void*) 0x1234, 0U, 0U)));
}

TEST_F(DoublyLinkedListSmokeTest, addNonNullItemToNullListSortedWithOddCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemSorted(NULL, (void*) 0x1234, 0U, 7U)));
}

TEST_F(DoublyLinkedListSmokeTest, addNonNullItemToNullListSortedWithTooLargeCompareSize)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_doublyLinkedList_addItemSorted(NULL, (void*) 0x1234, 0U, 9U)));
}

TEST_F(DoublyLinkedListSmokeTest, removeItemFromNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(NULL, (void*) 0x1234, false)));
}

TEST_F(DoublyLinkedListSmokeTest, removeAllItemsFromNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(NULL, (void*) 0x1234, true)));
}

TEST_F(DoublyLinkedListSmokeTest, itemFromInvalidIndexInNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(NULL, 0U)));
}

TEST_F(DoublyLinkedListSmokeTest, itemFromArbitraryIndexInNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(NULL, 42U)));
}

TEST_F(DoublyLinkedListSmokeTest, getItemCountFromNullList)
{
    EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(NULL)));
}

// -----

TEST_F(DoublyLinkedListSmokeTest, initializeUninitialized)
{
    // TODO: Actually return a status code!
    //EXPECT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, XME_HAL_DOUBLYLINKEDLIST_INIT(myList)));

    EXPECT_NO_XME_ASSERTION_FAILURES(XME_HAL_DOUBLYLINKEDLIST_INIT(myList));
}

TEST_F(DoublyLinkedListSmokeTest, finiUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_doublyLinkedList_fini(&myList));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(DoublyLinkedListSmokeTest, addItemToUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(XME_STATUS_UNEXPECTED, xme_hal_doublyLinkedList_addItem(&myList, NULL)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(DoublyLinkedListSmokeTest, addItemSortedToUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(XME_STATUS_UNEXPECTED, xme_hal_doublyLinkedList_addItemSorted(&myList, NULL, 0U, 0U)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(DoublyLinkedListSmokeTest, removeItemFromUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(0U, xme_hal_doublyLinkedList_removeItem(&myList, NULL, true)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(DoublyLinkedListSmokeTest, getItemCountUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(0U, xme_hal_doublyLinkedList_getItemCount(&myList)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(DoublyLinkedListSmokeTest, getItemCountUninitializedDeprecated)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(0U, XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(myList)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(DoublyLinkedListSmokeTest, itemFromIndexUninitialized)
{
#ifdef DEBUG
    // Assertion failure expected
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(NULL, xme_hal_doublyLinkedList_itemFromIndex(&myList, 0U)));
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(DoublyLinkedListSmokeTest, iterateUninitialized)
{
#ifdef DEBUG
    EXPECT_XME_ASSERTION_FAILURES
    (
        XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(myList, void*, item);
        {
            // The test just ensures that iterating over an
            // uninitialized linked list does not lead to a crash
            XME_UNUSED_PARAMETER(item);
            break;
        }
        XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
    );
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

TEST_F(DoublyLinkedListSmokeTest, iterateReverseUninitialized)
{
#ifdef DEBUG
    EXPECT_XME_ASSERTION_FAILURES
    (
        XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(myList, void*, item);
        {
            // The test just ensures that iterating over an
            // uninitialized linked list does not lead to a crash
            XME_UNUSED_PARAMETER(item);
            break;
        }
        XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
    );
#else // #ifdef DEBUG
    // No checks in release mode
#endif // #ifdef DEBUG
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
