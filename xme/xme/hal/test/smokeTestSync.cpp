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
 * $Id: smokeTestSync.cpp 6282 2014-01-09 16:43:46Z geisinger $
 */

/**
 * \file
 *         Synchronization smoke tests.
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

class SyncSmokeTest: public ::testing::Test
{
protected:
    SyncSmokeTest()
    {
    }

    virtual ~SyncSmokeTest()
    {
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     SyncSmokeTest                                                          //
//----------------------------------------------------------------------------//

TEST_F(SyncSmokeTest, initUninitialized)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_fini());
}

TEST_F(SyncSmokeTest, initUninitializedTwice)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());

    ASSERT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_fini());
    EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_fini());
}

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, finiUninitialized)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_fini());
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, createCriticalSectionUninitialized)
{
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE, xme_hal_sync_createCriticalSection()));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, destroyCriticalSectionUninitializedWithInvalidHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(XME_STATUS_UNEXPECTED, xme_hal_sync_destroyCriticalSection(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE)));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, destroyCriticalSectionUninitializedWithArbitraryHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(EXPECT_EQ(XME_STATUS_UNEXPECTED, xme_hal_sync_destroyCriticalSection((xme_hal_sync_criticalSectionHandle_t) 42)));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, enterCriticalSectionUninitializedWithInvalidHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_enterCriticalSection(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, enterCriticalSectionUninitializedWithArbitraryHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_enterCriticalSection((xme_hal_sync_criticalSectionHandle_t) 42));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, tryEnterCriticalSectionUninitializedWithInvalidHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_tryEnterCriticalSection(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, tryEnterCriticalSectionUninitializedWithArbitraryHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_tryEnterCriticalSection((xme_hal_sync_criticalSectionHandle_t) 42));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, leaveCriticalSectionUninitializedWithInvalidHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_leaveCriticalSection(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncSmokeTest, leaveCriticalSectionUninitializedWithArbitraryHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_leaveCriticalSection((xme_hal_sync_criticalSectionHandle_t) 42));
}
#endif // #ifdef DEBUG

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
