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
 * $Id: smokeTestEnv.cpp 7278 2014-02-12 09:59:23Z geisinger $
 */

/**
 * \file
 * \brief Generic environment abstraction smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/testUtils.h"

#include "xme/hal/include/env.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class EnvSmokeTest: public ::testing::Test
{
protected:
    EnvSmokeTest()
    : initialized(false)
    {
    }

    virtual ~EnvSmokeTest()
    {
        if (initialized)
        {
            xme_hal_env_fini();
        }
    }

    bool initialized; ///< Set to true to indicate that a test case has initialized the xme_hal_env component (needed to determine whether it should be finalized or not).
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     EnvSmokeTest                                                           //
//----------------------------------------------------------------------------//

TEST_F(EnvSmokeTest, initializeUninitialized)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_env_init());
    initialized = true;
}

TEST_F(EnvSmokeTest, finalizeInitialized)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_init());
    initialized = true;

    EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_env_fini());
    initialized = false;
}

TEST_F(EnvSmokeTest, finalizeUninitialized)
{
    // Currently doing this is uncritical
    EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_env_fini());
}

// xme_hal_env_getCurrentExecutablePath()

TEST_F(EnvSmokeTest, getCurrentExecutablePathWithNullBufferAndNonZeroSize)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_init());
    initialized = true;

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_hal_env_getCurrentExecutablePath(NULL, 42U));
}

TEST_F(EnvSmokeTest, getCurrentExecutablePathWithValidBufferAndZeroSize)
{
    char buf[256] = { 1, 2, 3, 4, 5, 0 };
    buf[255] = 0;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_init());
    initialized = true;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_getCurrentExecutablePath(buf, 0U));
    EXPECT_STREQ("\1\2\3\4\5", buf);
}

TEST_F(EnvSmokeTest, getCurrentExecutablePathWithValidBufferAndSmallSize)
{
    char buf[256] = { 1, 2, 3, 4, 5, 0 };
    buf[255] = 0;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_init());
    initialized = true;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_getCurrentExecutablePath(buf, 4U));
    EXPECT_NE(1, buf[0]);
    EXPECT_NE(1, buf[1]);
    EXPECT_NE(1, buf[2]);
    EXPECT_EQ(0, buf[3]);
    EXPECT_EQ(5, buf[4]);
    EXPECT_EQ(0, buf[5]);
}

TEST_F(EnvSmokeTest, getCurrentExecutablePathWithValidBufferAndFullSize)
{
    char buf[256] = { 1 };
    buf[255] = 0;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_init());
    initialized = true;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_getCurrentExecutablePath(buf, sizeof(buf)));

    // 4 should be a safe lower bound for the executable path length
    EXPECT_GE(strlen(buf), 4U);
}

// xme_hal_env_setConsoleTitle()

TEST_F(EnvSmokeTest, setConsoleTitleWithNullTitle)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_init());
    initialized = true;

    EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_env_setConsoleTitle(NULL));
}

TEST_F(EnvSmokeTest, setConsoleTitleWithEmptyTitle)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_init());
    initialized = true;

    EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_env_setConsoleTitle(""));
}

TEST_F(EnvSmokeTest, setConsoleTitleWithNormalTitle)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_env_init());
    initialized = true;

    EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_env_setConsoleTitle("This is a test!"));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
