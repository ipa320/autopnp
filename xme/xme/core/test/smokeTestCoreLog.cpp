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
 * $Id: smokeTestCoreLog.cpp 7794 2014-03-12 17:13:22Z geisinger $
 */

/**
 * \file
 *         XME core logging utility functions smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/logUtils.h"

#include "xme/core/testUtils.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class CoreLogSmokeTest: public ::testing::Test
{
protected:
    CoreLogSmokeTest()
    : componentID((xme_core_log_componentID_t) 42)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_log_logUtils_init());
    }

    virtual ~CoreLogSmokeTest()
    {
        EXPECT_NO_XME_ASSERTION_FAILURES(xme_core_log_logUtils_fini());
    }

    xme_core_log_componentID_t componentID;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     CoreLogSmokeTest                                                       //
//----------------------------------------------------------------------------//

TEST_F(CoreLogSmokeTest, registerComponent)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_log_registerComponent(componentID, "testComponent", XME_LOG_DEBUG));
}

TEST_F(CoreLogSmokeTest, getComponentLogLevelWithoutRegister)
{
    EXPECT_EQ(XME_LOG_NOTE, xme_core_log_getComponentLogLevel(componentID));
}

TEST_F(CoreLogSmokeTest, getComponentAcronymWithoutRegister)
{
    EXPECT_EQ(NULL, xme_core_log_getComponentAcronym(componentID));
}

TEST_F(CoreLogSmokeTest, setComponentLogLevelWithoutRegister)
{
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_log_setComponentLogLevel(componentID, XME_LOG_NOTE));
}

TEST_F(CoreLogSmokeTest, registerComponentAndRetrieveInformation)
{
    const char* acronym = "testComponent";

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_log_registerComponent(componentID, acronym, XME_LOG_WARNING));
    EXPECT_EQ(XME_LOG_WARNING, xme_core_log_getComponentLogLevel(componentID));
    EXPECT_EQ(acronym, xme_core_log_getComponentAcronym(componentID));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_log_setComponentLogLevel(componentID, XME_LOG_FATAL));
    EXPECT_EQ(XME_LOG_FATAL, xme_core_log_getComponentLogLevel(componentID));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_log_setComponentLogLevel(componentID, XME_LOG_DEBUG));
    EXPECT_EQ(XME_LOG_DEBUG, xme_core_log_getComponentLogLevel(componentID));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
