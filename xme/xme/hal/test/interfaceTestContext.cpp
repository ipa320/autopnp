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
 * $Id: interfaceTestContext.cpp 3465 2013-05-23 12:28:58Z geisinger $
 */

/**
 * \file
 *         Context interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/context.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class ContextInterfaceTest: public ::testing::Test
{
protected:
    ContextInterfaceTest()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_context_init());
    }

    virtual ~ContextInterfaceTest()
    {
        xme_hal_context_fini();
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     ContextInterfaceTest                                                   //
//----------------------------------------------------------------------------//

TEST_F(ContextInterfaceTest, setInvalidContextAndGetContext)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_context_setContext(XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE));
    EXPECT_EQ(XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE, xme_hal_context_getContext());
}

TEST_F(ContextInterfaceTest, setArbitratyContextAndGetContext)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_context_setContext((xme_hal_context_contextHandle_t)42));
    EXPECT_EQ((xme_hal_context_contextHandle_t)42, xme_hal_context_getContext());
}

TEST_F(ContextInterfaceTest, setMaxContextAndGetContext)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_context_setContext(XME_HAL_CONTEXT_MAX_CONTEXT_HANDLE));
    EXPECT_EQ(XME_HAL_CONTEXT_MAX_CONTEXT_HANDLE, xme_hal_context_getContext());
}

TEST_F(ContextInterfaceTest, setInvalidContextOverwriteWithArbitraryContextAndGetContext)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_context_setContext(XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_context_setContext((xme_hal_context_contextHandle_t)42));
    EXPECT_EQ((xme_hal_context_contextHandle_t)42, xme_hal_context_getContext());
}

TEST_F(ContextInterfaceTest, setArbitraryContextOverwriteWithInvalidContextAndGetContext)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_context_setContext((xme_hal_context_contextHandle_t)42));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_context_setContext(XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE));
    EXPECT_EQ(XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE, xme_hal_context_getContext());
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
