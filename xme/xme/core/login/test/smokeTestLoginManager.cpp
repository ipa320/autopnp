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
 * $Id: smokeTestLoginManager.cpp 4357 2013-07-25 08:49:48Z ruiz $
 */

/**
 * \file
 *        Login Manager smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/login/include/loginManager.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class LoginManagerSmokeTest: public ::testing::Test
{
protected:
    // constructor
    LoginManagerSmokeTest()
    {
    }

    virtual ~LoginManagerSmokeTest()
    {
    }
};

TEST_F(LoginManagerSmokeTest, InitLoginManagerWithNULLParameter)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_login_loginManager_init(NULL));
    xme_core_login_loginManager_fini();
}

TEST_F(LoginManagerSmokeTest, FiniLoginManagerWithoutInit)
{
    xme_core_login_loginManager_fini();
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
