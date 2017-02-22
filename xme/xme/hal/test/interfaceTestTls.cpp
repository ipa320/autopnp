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
 * $Id: interfaceTestTls.cpp 3464 2013-05-23 12:21:49Z geisinger $
 */

/**
 * \file
 *         Thread-local storage (TLS) interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
// Ensure that stdint.h defines limit macros
#define __STDC_LIMIT_MACROS

#include <gtest/gtest.h>

#include "xme/hal/include/tls.h"

#include <stdint.h>

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class TlsInterfaceTest: public ::testing::Test
{
protected:
    TlsInterfaceTest()
    : tlsHandle(XME_HAL_TLS_INVALID_TLS_HANDLE)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_tls_init());
    }

    virtual ~TlsInterfaceTest()
    {
        if (tlsHandle != XME_HAL_TLS_INVALID_TLS_HANDLE)
        {
            xme_hal_tls_free(tlsHandle);
        }

        xme_hal_tls_fini();
    }

    xme_hal_tls_handle_t tlsHandle;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     TlsInterfaceTest                                                       //
//----------------------------------------------------------------------------//

// xme_hal_tls_alloc() and xme_hal_tls_free()

TEST_F(TlsInterfaceTest, allocWithSizeOfZeroAndFree)
{
    tlsHandle = xme_hal_tls_alloc(0);
    EXPECT_EQ(XME_HAL_TLS_INVALID_TLS_HANDLE, tlsHandle);
}

TEST_F(TlsInterfaceTest, allocWithSizeOfOneAndFree)
{
    tlsHandle = xme_hal_tls_alloc(1);
    ASSERT_NE(XME_HAL_TLS_INVALID_TLS_HANDLE, tlsHandle);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_tls_free(tlsHandle));
    tlsHandle = XME_HAL_TLS_INVALID_TLS_HANDLE;
}

TEST_F(TlsInterfaceTest, allocWithMaximumSizeAndFree)
{
    tlsHandle = xme_hal_tls_alloc(UINT16_MAX);
    ASSERT_NE(XME_HAL_TLS_INVALID_TLS_HANDLE, tlsHandle);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_tls_free(tlsHandle));
    tlsHandle = XME_HAL_TLS_INVALID_TLS_HANDLE;
}

// xme_hal_tls_get()

TEST_F(TlsInterfaceTest, getFromInvalidHandle)
{
    EXPECT_EQ(NULL, xme_hal_tls_get(XME_HAL_TLS_INVALID_TLS_HANDLE));
}

TEST_F(TlsInterfaceTest, getFromArbitraryHandle)
{
    EXPECT_EQ(NULL, xme_hal_tls_get((xme_hal_tls_handle_t)42));
}

TEST_F(TlsInterfaceTest, getFromMaxHandle)
{
    EXPECT_EQ(NULL, xme_hal_tls_get(XME_HAL_TLS_MAX_TLS_HANDLE));
}

TEST_F(TlsInterfaceTest, allocWithSizeOfOneAndGet)
{
    tlsHandle = xme_hal_tls_alloc(1);
    EXPECT_TRUE(NULL != xme_hal_tls_get(tlsHandle));
}

TEST_F(TlsInterfaceTest, allocWithMaximumSizeAndGet)
{
    tlsHandle = xme_hal_tls_alloc(UINT16_MAX);
    EXPECT_TRUE(NULL != xme_hal_tls_get(tlsHandle));
}

// xme_hal_tls_getSize()

TEST_F(TlsInterfaceTest, getSizeOfInvalidHandle)
{
    EXPECT_EQ(0, xme_hal_tls_getSize(XME_HAL_TLS_INVALID_TLS_HANDLE));
}

TEST_F(TlsInterfaceTest, getSizeOfArbitraryHandle)
{
    EXPECT_EQ(0, xme_hal_tls_getSize((xme_hal_tls_handle_t)42));
}

TEST_F(TlsInterfaceTest, getSizeOfMaxHandle)
{
    EXPECT_EQ(0, xme_hal_tls_getSize(XME_HAL_TLS_MAX_TLS_HANDLE));
}

TEST_F(TlsInterfaceTest, allocWithSizeOfZeroAndGetSize)
{
    tlsHandle = xme_hal_tls_alloc(0);
    EXPECT_EQ(0, xme_hal_tls_getSize(tlsHandle));
}

TEST_F(TlsInterfaceTest, allocWithSizeOfOneAndGetSize)
{
    tlsHandle = xme_hal_tls_alloc(1);
    EXPECT_EQ(1, xme_hal_tls_getSize(tlsHandle));
}

TEST_F(TlsInterfaceTest, allocWithMaximumSizeAndGetSize)
{
    tlsHandle = xme_hal_tls_alloc(UINT16_MAX);
    EXPECT_EQ(UINT16_MAX, xme_hal_tls_getSize(tlsHandle));
}

TEST_F(TlsInterfaceTest, allocWithSizeOfZeroAndGetAndGetSize)
{
    tlsHandle = xme_hal_tls_alloc(0);
    (void)xme_hal_tls_get(tlsHandle);
    EXPECT_EQ(0, xme_hal_tls_getSize(tlsHandle));
}

TEST_F(TlsInterfaceTest, allocWithSizeOfOneAndGetAndGetSize)
{
    tlsHandle = xme_hal_tls_alloc(1);
    (void)xme_hal_tls_get(tlsHandle);
    EXPECT_EQ(1, xme_hal_tls_getSize(tlsHandle));
}

TEST_F(TlsInterfaceTest, allocWithMaximumSizeAndGetAndGetSize)
{
    tlsHandle = xme_hal_tls_alloc(UINT16_MAX);
    (void)xme_hal_tls_get(tlsHandle);
    EXPECT_EQ(UINT16_MAX, xme_hal_tls_getSize(tlsHandle));
}

// xme_hal_tls_free()

TEST_F(TlsInterfaceTest, freeInvalidHandle)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_tls_free(XME_HAL_TLS_INVALID_TLS_HANDLE));
}

TEST_F(TlsInterfaceTest, freeArbitraryHandle)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_tls_free((xme_hal_tls_handle_t)42));
}

TEST_F(TlsInterfaceTest, freeMaxHandle)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_tls_free(XME_HAL_TLS_MAX_TLS_HANDLE));
}

TEST_F(TlsInterfaceTest, allocWithSizeOfOneAndDoubleFree)
{
    tlsHandle = xme_hal_tls_alloc(1);
    ASSERT_NE(XME_HAL_TLS_INVALID_TLS_HANDLE, tlsHandle);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_tls_free(tlsHandle));
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_tls_free(tlsHandle));
    tlsHandle = XME_HAL_TLS_INVALID_TLS_HANDLE;
}

// xme_hal_tls_registerThread() and xme_hal_tls_deregisterThread()

TEST_F(TlsInterfaceTest, registerThread)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_tls_registerThread());
    xme_hal_tls_deregisterThread();
}

TEST_F(TlsInterfaceTest, registerThreadAndAllocWithSizeOfOneAndDeregisterThreadAndGet)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_tls_registerThread());

    tlsHandle = xme_hal_tls_alloc(1);
    EXPECT_TRUE(NULL != xme_hal_tls_get(tlsHandle));
    EXPECT_EQ(1, xme_hal_tls_getSize(tlsHandle));

    xme_hal_tls_deregisterThread();

    EXPECT_TRUE(NULL != xme_hal_tls_get(tlsHandle));
    EXPECT_EQ(1, xme_hal_tls_getSize(tlsHandle));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
