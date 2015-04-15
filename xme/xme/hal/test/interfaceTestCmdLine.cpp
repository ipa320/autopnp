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
 * $Id: interfaceTestCmdLine.cpp 6269 2014-01-08 16:00:03Z geisinger $
 */

/**
 * \file
 *         Command line parsing abstraction interface tests.
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

class CmdLineInterfaceTest: public ::testing::Test
{
protected:
    CmdLineInterfaceTest()
    : options("ab:c")
    , argv((char**) 0x1234)
    {
        argE[0] = (char*) "moduleName";
        argE[1] = NULL;

        argA[0] = (char*) "moduleName";
        argA[1] = (char*) "-a";
        argA[2] = NULL;

        argAB[0] = (char*) "moduleName";
        argAB[1] = (char*) "-a";
        argAB[2] = (char*) "-b";
        argAB[3] = NULL;

        argABv[0] = (char*) "moduleName";
        argABv[1] = (char*) "-a";
        argABv[2] = (char*) "-b";
        argABv[3] = (char*) "21";
        argABv[4] = NULL;

        argABC[0] = (char*) "moduleName";
        argABC[1] = (char*) "-a";
        argABC[2] = (char*) "-b";
        argABC[3] = (char*) "-c";
        argABC[4] = NULL;

        argABvC[0] = (char*) "moduleName";
        argABvC[1] = (char*) "-a";
        argABvC[2] = (char*) "-b";
        argABvC[3] = (char*) "21";
        argABvC[4] = (char*) "-c";
        argABvC[5] = NULL;
    }

    virtual ~CmdLineInterfaceTest()
    {
        EXPECT_NO_XME_ASSERTION_FAILURES(xme_hal_cmdLine_fini());
    }

    const char* options;
    char** argv;
    char* argE[2];
    char* argA[3];
    char* argAB[4];
    char* argABv[5];
    char* argABC[5];
    char* argABvC[6];
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     CmdLineInterfaceTest                                                   //
//----------------------------------------------------------------------------//

TEST_F(CmdLineInterfaceTest, initializeWithArgE)
{
    int numArgs = sizeof(argE) / sizeof(argE[0]) - 1;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_cmdLine_init(numArgs, argE, options, 1));

    EXPECT_EQ(numArgs, xme_hal_cmdLine_getArgs(&argv));
    EXPECT_EQ(argE, argv);

    EXPECT_EQ(-1, xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(42, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 42));
}

TEST_F(CmdLineInterfaceTest, initializeWithArgA)
{
    int numArgs = sizeof(argA) / sizeof(argA[0]) - 1;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_cmdLine_init(numArgs, argA, options, 1));

    EXPECT_EQ(numArgs, xme_hal_cmdLine_getArgs(&argv));
    EXPECT_EQ(argA, argv);

    EXPECT_EQ('a', xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(42, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 42));

    EXPECT_EQ(-1, xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(43, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 43));
}

TEST_F(CmdLineInterfaceTest, initializeWithArgAB)
{
    int numArgs = sizeof(argAB) / sizeof(argAB[0]) - 1;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_cmdLine_init(numArgs, argAB, options, 1));

    EXPECT_EQ(numArgs, xme_hal_cmdLine_getArgs(&argv));
    EXPECT_EQ(argAB, argv);

    EXPECT_EQ('a', xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(42, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 42));

    // option requires an argument -- b
    EXPECT_EQ('?', xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(43, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 43));

    EXPECT_EQ(-1, xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(44, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 44));
}

TEST_F(CmdLineInterfaceTest, initializeWithArgABv)
{
    int numArgs = sizeof(argABv) / sizeof(argABv[0]) - 1;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_cmdLine_init(numArgs, argABv, options, 1));

    EXPECT_EQ(numArgs, xme_hal_cmdLine_getArgs(&argv));
    EXPECT_EQ(argABv, argv);

    EXPECT_EQ('a', xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(42, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 42));

    EXPECT_EQ('b', xme_hal_cmdLine_getNextOption());
    EXPECT_STREQ("21", xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(21, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 43));
    EXPECT_EQ(43, xme_hal_cmdLine_getIntegerOptionArgument(-100, 0, 43));
    EXPECT_EQ(43, xme_hal_cmdLine_getIntegerOptionArgument(100, 200, 43));

    EXPECT_EQ(-1, xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(44, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 44));
}

TEST_F(CmdLineInterfaceTest, initializeWithArgABC)
{
    int numArgs = sizeof(argABC) / sizeof(argABC[0]) - 1;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_cmdLine_init(numArgs, argABC, options, 1));

    EXPECT_EQ(numArgs, xme_hal_cmdLine_getArgs(&argv));
    EXPECT_EQ(argABC, argv);

    EXPECT_EQ('a', xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(42, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 42));

    // option requires an argument -- b
    EXPECT_EQ('?', xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(43, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 43));

    EXPECT_EQ('c', xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(44, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 44));

    EXPECT_EQ(-1, xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(45, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 45));
}

TEST_F(CmdLineInterfaceTest, initializeWithArgABvC)
{
    int numArgs = sizeof(argABvC) / sizeof(argABvC[0]) - 1;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_cmdLine_init(numArgs, argABvC, options, 1));

    EXPECT_EQ(numArgs, xme_hal_cmdLine_getArgs(&argv));
    EXPECT_EQ(argABvC, argv);

    EXPECT_EQ('a', xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(42, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 42));

    EXPECT_EQ('b', xme_hal_cmdLine_getNextOption());
    EXPECT_STREQ("21", xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(21, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 43));
    EXPECT_EQ(43, xme_hal_cmdLine_getIntegerOptionArgument(-100, 0, 43));
    EXPECT_EQ(43, xme_hal_cmdLine_getIntegerOptionArgument(100, 200, 43));

    EXPECT_EQ('c', xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(44, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 44));

    EXPECT_EQ(-1, xme_hal_cmdLine_getNextOption());
    EXPECT_EQ(NULL, xme_hal_cmdLine_getOptionArgument());
    EXPECT_EQ(45, xme_hal_cmdLine_getIntegerOptionArgument(-100, 100, 45));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
