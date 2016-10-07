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
 * $Id: interfaceTestSync.cpp 6282 2014-01-09 16:43:46Z geisinger $
 */

/**
 * \file
 *         Synchronization interface tests.
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

class SyncInterfaceTest: public ::testing::Test
{
protected:
    SyncInterfaceTest()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    }

    virtual ~SyncInterfaceTest()
    {
        EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_sync_fini());
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     SyncInterfaceTest                                                      //
//----------------------------------------------------------------------------//

TEST_F(SyncInterfaceTest, createCriticalSection)
{
    EXPECT_NE(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE, xme_hal_sync_createCriticalSection());
}

TEST_F(SyncInterfaceTest, destroyCriticalSectionWithInvalidHandle)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_sync_destroyCriticalSection(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE));
}

TEST_F(SyncInterfaceTest, destroyCriticalSectionWithArbitraryHandle)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_sync_destroyCriticalSection((xme_hal_sync_criticalSectionHandle_t) 42));
}

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncInterfaceTest, enterCriticalSectionWithInvalidHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_enterCriticalSection(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncInterfaceTest, enterCriticalSectionWithArbitraryHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_enterCriticalSection((xme_hal_sync_criticalSectionHandle_t) 42));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncInterfaceTest, tryEnterCriticalSectionWithInvalidHandle)
{
    EXPECT_XME_ASSERTION_FAILURES(xme_hal_sync_tryEnterCriticalSection(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncInterfaceTest, tryEnterCriticalSectionWithArbitraryHandle)
{
    EXPECT_XME_ASSERTION_FAILURES(xme_hal_sync_tryEnterCriticalSection((xme_hal_sync_criticalSectionHandle_t) 42));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncInterfaceTest, leaveCriticalSectionWithInvalidHandle)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_sync_leaveCriticalSection(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE));
}
#endif // #ifdef DEBUG

#ifdef DEBUG
// This test relies on the ASSERT macro so only to be executed in DEBUG mode
TEST_F(SyncInterfaceTest, leaveCriticalSectionWithArbitraryHandle)
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
