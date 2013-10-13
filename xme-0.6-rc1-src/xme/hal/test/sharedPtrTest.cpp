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
 * $Id: sharedPtrTest.cpp 4254 2013-07-17 13:13:22Z geisinger $
 */

/**
 * \file
 *         Shared pointer testsuite.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/sharedPtr.h"
#include "xme/core/core.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define DATA_BUFFER_SIZE 256

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
xme_hal_sharedPtr_test_basic(void)
{
    xme_hal_sharedPtr_t dataHandle, dataHandle2;
    uint8_t test_buffer[DATA_BUFFER_SIZE];
    uint8_t test_buffer2[DATA_BUFFER_SIZE];
    test_buffer[0] = 0x01;
    test_buffer[DATA_BUFFER_SIZE-1] = 0x99;
    test_buffer2[0] = 0x02;
    test_buffer2[DATA_BUFFER_SIZE-1] = 0x98;

    dataHandle = xme_hal_sharedPtr_createFromPointer(DATA_BUFFER_SIZE, test_buffer);

    EXPECT_EQ(DATA_BUFFER_SIZE, xme_hal_sharedPtr_getSize(dataHandle));
    EXPECT_EQ(1, xme_hal_sharedPtr_getReferenceCount(dataHandle));
    EXPECT_EQ(DATA_BUFFER_SIZE, xme_hal_sharedPtr_getSize(dataHandle));

    dataHandle2 = xme_hal_sharedPtr_createFromPointer(DATA_BUFFER_SIZE, test_buffer2);
    EXPECT_NE(XME_HAL_SHAREDPTR_INVALID_POINTER, dataHandle2);
    EXPECT_EQ(1, xme_hal_sharedPtr_getReferenceCount(dataHandle2));

    EXPECT_EQ(DATA_BUFFER_SIZE, xme_hal_sharedPtr_getSize(dataHandle2));

    EXPECT_NE(XME_HAL_SHAREDPTR_INVALID_POINTER, dataHandle);
    EXPECT_EQ(1, xme_hal_sharedPtr_getReferenceCount(dataHandle));

    EXPECT_EQ(dataHandle, xme_hal_sharedPtr_retain(dataHandle));
    EXPECT_EQ(2, xme_hal_sharedPtr_getReferenceCount(dataHandle));

    EXPECT_EQ(dataHandle, xme_hal_sharedPtr_retain(dataHandle));
    EXPECT_EQ(3, xme_hal_sharedPtr_getReferenceCount(dataHandle));

    xme_hal_sharedPtr_destroy(dataHandle);
    EXPECT_EQ(2, xme_hal_sharedPtr_getReferenceCount(dataHandle));

    EXPECT_EQ(dataHandle, xme_hal_sharedPtr_retain(dataHandle));
    EXPECT_EQ(3, xme_hal_sharedPtr_getReferenceCount(dataHandle));

    xme_hal_sharedPtr_destroy(dataHandle);
    xme_hal_sharedPtr_destroy(dataHandle);
    xme_hal_sharedPtr_destroy(dataHandle);

    EXPECT_EQ(XME_HAL_SHAREDPTR_INVALID_POINTER, xme_hal_sharedPtr_getReferenceCount(dataHandle));
}

TEST(XMEHalShmTest, xme_tests_hal_shm)
{
	// Setup
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_init());

	xme_hal_sharedPtr_test_basic();

	// Teardown
	xme_core_fini();
}
