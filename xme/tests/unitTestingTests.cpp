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
 * $Id: unitTestingTests.cpp 3348 2013-05-17 12:28:31Z geisinger $
 */

/**
 * \file
 *         Testsuite unit tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "tests/unitTestingTestsUtils.h"

#include <gtest/gtest.h>

#include "xme/defines.h"

#include <stdio.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#ifndef __cplusplus
#error "This translation unit must be compiled in C++ mode for the test to be effective!"
#endif // #ifndef __cplusplus

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

TEST(DISABLED_UnitTestingTest, disabledTestSuite)
{
    // This test shows how to temporarily disable a whole test suite

    printf("********** THIS MESSAGE SHOULD NEVER BE PRINTED! **********\n");
    ASSERT_TRUE(0);
}

TEST(UnitTestingTest, DISABLED_disabledTestCase)
{
    // This test shows how to temporarily disable an individual test case

    printf("********** THIS MESSAGE SHOULD NEVER BE PRINTED! **********\n");
    ASSERT_TRUE(0);
}

TEST(UnitTestingTest, assertionInCMode)
{
    // This test shows how an assertion failure is nonfatal in unit tests
    // compiled in C mode

    printf("This test will print an assertion failure message:\n");
    xme_raiseAssertionFailure(); // defined in C mode
}

TEST(UnitTestingTest, assertionInCPlusPlusMode)
{
    // This test shows how an assertion failure raises an exception in C++ mode

    EXPECT_NO_THROW(XME_ASSERT_NORVAL(1));
    EXPECT_ANY_THROW(XME_ASSERT_NORVAL(0));
}

int main (int argc, char* argv[])
{
    printf("******************************************************************\n");
    printf("********** This testsuite should have 2 disabled tests! **********\n");
    printf("******************************************************************\n\n");

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
