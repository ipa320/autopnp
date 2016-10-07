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
 * $Id: smokeTestDefines.cpp 5379 2013-10-02 16:47:06Z geisinger $
 */

/**
 * \file
 *         Defines smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/log.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define ENABLE_REGRESSION_TEST_3419 0

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class DefinesSmokeTest: public ::testing::Test
{
protected:
    DefinesSmokeTest()
    {
    }

    virtual ~DefinesSmokeTest()
    {
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     DefinesSmokeTest                                                       //
//----------------------------------------------------------------------------//

#if ENABLE_REGRESSION_TEST_3419

TEST_F(DefinesSmokeTest, abort)
{
    // Crash test to verify that the fix for Issue #3419 is working correctly,
    // which is related to the behavior of our build server in the presence of
    // aborting unit tests. Such situations might be caused by the invocation
    // of the abort() function, for example as a result of an assert()
    // statement.
    // If you enable this test (by setting the ENABLE_REGRESSION_TEST_3419
    // define to "1" above), it will provoke an abort during execution of this
    // test suite. In this case, googletest does not have a chance to write the
    // XML report files and hence the failing test will stay undetected. Even
    // worse, the test cases following the crashing test are not even executed.
    // In order to prevent this from happening, we create dummy XML report
    // files on our build server before executing the unit tests. Those files
    // contain a fake failing unit test that will subsequently be picked up by
    // the build analysis tools and hence not stay undetected. However, unit
    // tests following the crashing one are still not executed. Hence, such
    // problems should be resolved as early after detection as possible.

    // Abort the test suite
    XME_LOG(XME_LOG_ALWAYS, "Aborting test suite in order to provoke a missing test result!\n");
    abort();
}

TEST_F(DefinesSmokeTest, thisIsNeverExecuted)
{
    // Provoke a failing unit test to demonstrate that it will not be executed.
    // Notice that this requires the tests to be run in order of definition.
    EXPECT_TRUE(false);
}

#endif // #if ENABLE_REGRESSION_TEST_3419

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
