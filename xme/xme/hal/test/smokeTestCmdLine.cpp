/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: smokeTestCmdLine.cpp 6284 2014-01-09 17:43:53Z geisinger $
 */

/**
 * \file
 *         Command line parsing abstraction smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/testUtils.h"

#include "xme/hal/include/cmdLine.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class CmdLineSmokeTest: public ::testing::Test
{
protected:
    CmdLineSmokeTest()
    : initialized(false)
    {
    }

    virtual ~CmdLineSmokeTest()
    {
        if (initialized)
        {
            xme_hal_cmdLine_fini();
        }
    }

    bool initialized; ///< Set to true to indicate that a test case has initialized the xme_hal_cmdLine component (needed to determine whether it should be finalized or not).
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     CmdLineSmokeTest                                                       //
//----------------------------------------------------------------------------//

TEST_F(CmdLineSmokeTest, initializeUninitialized)
{
    char* args[1] = { NULL };
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_cmdLine_init(1, args, NULL, 0));
    initialized = true;
}

TEST_F(CmdLineSmokeTest, initializeUninitializedWithZeroArgc)
{
    char* args[1] = { NULL };
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_cmdLine_init(0, args, NULL, 0));
#ifdef NDEBUG
    initialized = true;
#endif
}

TEST_F(CmdLineSmokeTest, initializeUninitializedWithNullArgv)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_cmdLine_init(1, NULL, NULL, 0));
#ifdef NDEBUG
    initialized = true;
#endif
}

TEST_F(CmdLineSmokeTest, getArgsUninitialized)
{
    char** argv = (char**) 0x1234;
    ASSERT_EQ(0, xme_hal_cmdLine_getArgs(&argv));
    EXPECT_EQ(NULL, argv);
}

TEST_F(CmdLineSmokeTest, getNextOptionUninitialized)
{
    ASSERT_EQ(-1, xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(-1, xme_hal_cmdLine_getNextOption());
}

TEST_F(CmdLineSmokeTest, getOptionArgumentUninitialized)
{
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
}

TEST_F(CmdLineSmokeTest, getIntegerOptionArgumentUninitialized)
{
    EXPECT_EQ(42, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 42));
}

TEST_F(CmdLineSmokeTest, finalizeUninitialized)
{
    EXPECT_XME_ASSERTION_FAILURE(xme_hal_cmdLine_fini());
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
