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
 * $Id: integrationTestSync.cpp 6306 2014-01-14 10:40:31Z gulati $
 */

/**
 * \file
 *         Synchronization integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/testUtils.h"

#include "xme/hal/include/sync.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class SyncIntegrationTest: public ::testing::Test
{
protected:
    SyncIntegrationTest()
    : h1(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE)
    , h2(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    }

    virtual ~SyncIntegrationTest()
    {
        EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_fini());
    }

    xme_hal_sync_criticalSectionHandle_t h1;
    xme_hal_sync_criticalSectionHandle_t h2;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     SyncIntegrationTest                                                    //
//----------------------------------------------------------------------------//

TEST_F(SyncIntegrationTest, oneHandleInSameThread)
{
    h1 = xme_hal_sync_createCriticalSection();
    ASSERT_NE(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE, h1);

    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_enterCriticalSection(h1));

    // TODO: Add tests involving multiple threads
#if XME_HAL_SYNC_RECURSIVE_LOCKING
    // Locking from the same thread succeeds
    ASSERT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_tryEnterCriticalSection(h1)));
#else // #if XME_HAL_SYNC_RECURSIVE_LOCKING
    // Locking from the same thread blocks
    ASSERT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_WOULD_BLOCK, xme_hal_sync_tryEnterCriticalSection(h1)));
#endif // #if XME_HAL_SYNC_RECURSIVE_LOCKING

    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_leaveCriticalSection(h1));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_destroyCriticalSection(h1));
}

TEST_F(SyncIntegrationTest, twoHandlesInSameThread)
{
    // Create two handles
    h1 = xme_hal_sync_createCriticalSection();
    ASSERT_NE(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE, h1);

    h2 = xme_hal_sync_createCriticalSection();
    ASSERT_NE(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE, h2);

    ASSERT_NE(h1, h2);

    // Enter h1
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_enterCriticalSection(h1));

    // Enter h2
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_enterCriticalSection(h2));

    // TODO: Add tests involving multiple threads
#if XME_HAL_SYNC_RECURSIVE_LOCKING
    // h2 is already locked, but xme_hal_sync_tryEnterCriticalSection should still succeed
    ASSERT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_tryEnterCriticalSection(h2)));
#else // #if XME_HAL_SYNC_RECURSIVE_LOCKING
    // h2 is already locked, hence xme_hal_sync_tryEnterCriticalSection should not succeed
    ASSERT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_WOULD_BLOCK, xme_hal_sync_tryEnterCriticalSection(h2)));
#endif // #if XME_HAL_SYNC_RECURSIVE_LOCKING

    // TODO: Add tests involving multiple threads
#if XME_HAL_SYNC_RECURSIVE_LOCKING
    // h1 is already locked, but xme_hal_sync_tryEnterCriticalSection should still succeed
    ASSERT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_tryEnterCriticalSection(h1)));
#else // #if XME_HAL_SYNC_RECURSIVE_LOCKING
    // h1 is already locked, hence xme_hal_sync_tryEnterCriticalSection should not succeed
    ASSERT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_WOULD_BLOCK, xme_hal_sync_tryEnterCriticalSection(h1)));
#endif // #if XME_HAL_SYNC_RECURSIVE_LOCKING

    // Free h1
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_leaveCriticalSection(h1));

    // Enter h1
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_enterCriticalSection(h1));

    // Free h1
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_leaveCriticalSection(h1));

    // TODO: Add tests involving multiple threads
#if XME_HAL_SYNC_RECURSIVE_LOCKING
    // h2 is still locked, but xme_hal_sync_tryEnterCriticalSection should still succeed
    ASSERT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_tryEnterCriticalSection(h2)));
#else // #if XME_HAL_SYNC_RECURSIVE_LOCKING
    // h2 is still locked, hence xme_hal_sync_tryEnterCriticalSection should not succeed
    ASSERT_NO_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_WOULD_BLOCK, xme_hal_sync_tryEnterCriticalSection(h2)));
#endif // #if XME_HAL_SYNC_RECURSIVE_LOCKING

    // Free h2
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_leaveCriticalSection(h2));

    // Destroy h1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_destroyCriticalSection(h1));

    // Enter h2
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_enterCriticalSection(h2));

    // Leave h2
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_leaveCriticalSection(h2));

    // Free h2
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_destroyCriticalSection(h2));
}

TEST_F(SyncIntegrationTest, destroyWithoutLeaveInSameThread)
{
    h1 = xme_hal_sync_createCriticalSection();
    ASSERT_NE(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE, h1);

    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_enterCriticalSection(h1));

    // Enable this if mutices are not recursive
#ifndef XME_HAL_SYNC_RECURSIVE_LOCKING 
    // Destroying while locked by the same thread is OK
#ifdef DEBUG
    EXPECT_XME_ASSERTION_FAILURES(EXPECT_EQ(XME_STATUS_UNEXPECTED, xme_hal_sync_destroyCriticalSection(h1)));
#else // #ifdef DEBUG
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_destroyCriticalSection(h1));
#endif // #ifdef DEBUG
#endif // #ifndef XME_HAL_SYNC_RECURSIVE_LOCKING
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
