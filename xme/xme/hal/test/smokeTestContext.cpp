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
 * $Id: smokeTestContext.cpp 3469 2013-05-23 15:12:04Z geisinger $
 */

/**
 * \file
 *         Context smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/context.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class ContextSmokeTest: public ::testing::Test
{
protected:
    ContextSmokeTest()
    {
    }

    virtual ~ContextSmokeTest()
    {
        xme_hal_context_fini();
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     ContextSmokeTest                                                       //
//----------------------------------------------------------------------------//

TEST_F(ContextSmokeTest, init)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_context_init());
}

TEST_F(ContextSmokeTest, finiWithoutInit)
{
    xme_hal_context_fini();
}

TEST_F(ContextSmokeTest, setWithoutInitAndInvalidContext)
{
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_context_setContext(XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE));
}

TEST_F(ContextSmokeTest, setWithoutInitAndArbitraryContext)
{
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_context_setContext((xme_hal_context_contextHandle_t)42));
}

TEST_F(ContextSmokeTest, setWithoutInitAndMaxContext)
{
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_hal_context_setContext(XME_HAL_CONTEXT_MAX_CONTEXT_HANDLE));
}

TEST_F(ContextSmokeTest, getWithoutInit)
{
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
