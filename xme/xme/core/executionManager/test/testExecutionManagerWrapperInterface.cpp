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
 * $Id: testExecutionManagerWrapperInterface.cpp 4254 2013-07-17 13:13:22Z geisinger $
 */

/**
 * \file
 *         Execution Manager wrapper interface tests.
 */

#include <gtest/gtest.h>

/* Integration test mocks */
#include "testHelper_executionManager.h"

/* Includes */
#include "xme/core/executionManager/include/executionManagerWrapperInterface.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"

class Exec_WrapperInterfaceIntegrationTest : public ::testing::Test {
public:
    Exec_WrapperInterfaceIntegrationTest(): schedHandle(XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID)
    {}
protected:
  virtual void SetUp(void);
  virtual void TearDown(void);

  xme_core_exec_schedule_handle_t schedHandle;

};

void
Exec_WrapperInterfaceIntegrationTest::SetUp(void)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init(&testConfig));
    testInitFunctionCounters();
    schedHandle = setupEmptySchedule(1000000);
    registerCounterFunction(CID1, FID1, false);
    registerCounterFunction(CID2, FID2, true);
    registerCounterFunction(CID3, FID3, false);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addFunctionToSchedule(schedHandle,CID1,FID1,10000));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addFunctionToSchedule(schedHandle,CID2,FID2,10000));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addFunctionToSchedule(schedHandle,CID3,FID3,10000));
}

void
Exec_WrapperInterfaceIntegrationTest::TearDown(void)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}

TEST_F(Exec_WrapperInterfaceIntegrationTest, testIfItJustWorks)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_run(1,false));
}
