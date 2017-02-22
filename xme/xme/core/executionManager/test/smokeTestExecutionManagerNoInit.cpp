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
 * $Id: smokeTestExecutionManagerNoInit.cpp 4254 2013-07-17 13:13:22Z geisinger $
 */

/**
 * \file
 *         Execution Manager smoke tests (no init).
 */


#include <gtest/gtest.h>
#include "testFunctionsExecutionManager.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"

#include "xme/core/component.h"
#include "xme/hal/include/sync.h"
#include "xme/hal/include/sched.h"

/*****************************************************************************/
/***        Wrapper interface                                              ***/
/*****************************************************************************/
#include "xme/core/executionManager/include/executionManagerWrapperInterface.h"
TEST(ExecutionManagerSmokeTestNoInit, initializeTask)
{
    ASSERT_EQ(XME_STATUS_UNEXPECTED, xme_core_exec_dispatcher_initializeTask(CID1, FID1));
}

TEST(ExecutionManagerSmokeTestNoInit, executionCompleted)
{
    ASSERT_EQ(XME_STATUS_UNEXPECTED, xme_core_exec_dispatcher_executionCompleted(CID1, FID1));
}

TEST(ExecutionManagerSmokeTestNoInit, waitForStart)
{
    ASSERT_EQ(XME_STATUS_UNEXPECTED, xme_core_exec_dispatcher_waitForStart(CID1, FID1, NULL));
}

/*****************************************************************************/
/***        Wrapper interface                                              ***/
/*****************************************************************************/
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
TEST(ExecutionManagerSmokeTestNoInit, registerSchedule)
{
	xme_core_exec_schedule_table_t* sched = NULL;
	xme_core_exec_schedule_handle_t id;

    ASSERT_EQ(XME_STATUS_UNEXPECTED,
    		xme_core_exec_scheduler_registerSchedule(
    				sched,
    				&id
    		)
    );
}

TEST(ExecutionManagerSmokeTestNoInit, getSchedule)
{
	xme_core_exec_schedule_table_t* sched = NULL;

    ASSERT_EQ(XME_STATUS_UNEXPECTED,
    		xme_core_exec_scheduler_getSchedule(
    				(xme_core_exec_schedule_handle_t)1,
    				&sched
    		)
    );
}

TEST(ExecutionManagerSmokeTestNoInit, activateSchedule)
{
    ASSERT_EQ(XME_STATUS_UNEXPECTED,
    		xme_core_exec_scheduler_activateSchedule(
    				(xme_core_exec_schedule_handle_t)1
    		)
    );
}
