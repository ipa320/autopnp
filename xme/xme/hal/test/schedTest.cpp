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
 * $Id: schedTest.cpp 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Scheduler abstraction testsuite.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/core.h"

#include "xme/hal/include/sched.h"
#include "xme/hal/include/sleep.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static int globalTaskCounter = 0;
static int globalTaskErrors = 0;
static void* globalExpectedUserData = NULL;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
static void
xme_hal_sched_test_taskCallback(void* userData)
{
	globalTaskCounter++;

	if (globalExpectedUserData != userData)
	{
		globalTaskErrors++;
	}
}

static void
xme_hal_sched_test_resetGlobals(void* expectedUserData)
{
	globalTaskCounter = 0;
	globalTaskErrors = 0;
	globalExpectedUserData = expectedUserData;
}

void
xme_hal_sched_test_basic(void)
{
	// Choose a number that is high enough to not be a system task!
	const xme_hal_sched_taskHandle_t randomTaskHandle = (xme_hal_sched_taskHandle_t)1234;

	// Remove nonexistent tasks
	EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_sched_removeTask(XME_HAL_SCHED_INVALID_TASK_HANDLE));
	EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_sched_removeTask(randomTaskHandle));

	// Change state of nonexistent tasks
	EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_sched_setTaskExecutionState(XME_HAL_SCHED_INVALID_TASK_HANDLE, true));
	EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_sched_setTaskExecutionState(XME_HAL_SCHED_INVALID_TASK_HANDLE, false));
	EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_sched_setTaskExecutionState(randomTaskHandle, true));
	EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_sched_setTaskExecutionState(randomTaskHandle, false));

	// Add tasks with invalid parameters
	EXPECT_EQ(XME_HAL_SCHED_INVALID_TASK_HANDLE, xme_hal_sched_addTask(0, 0, XME_HAL_SCHED_PRIORITY_NORMAL, NULL, NULL));
}

void
xme_hal_sched_test_run(void)
{
	typedef struct
	{
		xme_hal_time_timeInterval_t startTime;
		xme_hal_time_timeInterval_t period;
		uint8_t priority;
		xme_hal_time_timeInterval_t sleepTime;
		xme_hal_time_timeInterval_t suspendedTime;
		int expectedCount;
	}
	xme_tests_hal_sched_testcase_t;

	xme_tests_hal_sched_testcase_t testcases[] =
	{
		{ XME_HAL_SCHED_TASK_INITIALLY_SUSPENDED, 0, XME_HAL_SCHED_PRIORITY_NORMAL, 500, 0, 0 }, // do not fire at all
		{ 1000, 0, XME_HAL_SCHED_PRIORITY_NORMAL, 500, 0, 0 }, // one-shot task that gets cancelled before it starts
		{ 0, 0, XME_HAL_SCHED_PRIORITY_NORMAL, 500, 0, 1 }, // one-shot task
		{ 1000, 100, XME_HAL_SCHED_PRIORITY_NORMAL, 500, 0, 0 }, // periodic task that gets cancelled before it starts
		{ 0, 100, XME_HAL_SCHED_PRIORITY_NORMAL, 550, 0, 6 }, // periodic task
		{ 1000, 0, XME_HAL_SCHED_PRIORITY_NORMAL, 500, 1000, 0 }, // one-shot task that gets suspended and then cancelled before it starts
		{ 0, 0, XME_HAL_SCHED_PRIORITY_NORMAL, 500, 500, 1 }, // one-shot task that gets suspended
	};

	void* userData[] =
	{
		NULL,
		(void*)0x1234
	};

	for (unsigned int i = 0; i < sizeof(testcases)/sizeof(testcases[0]); i++)
	{
		xme_tests_hal_sched_testcase_t* tc = &testcases[i];
		xme_hal_sched_taskHandle_t taskHandle;
		xme_status_t status;

		for (unsigned int j = 0; j < sizeof(userData)/sizeof(userData[0]); j++)
		{
			printf("xme_hal_sched_addTask() test %ld of %ld\n", (sizeof(userData)/sizeof(userData[0]))*i+j+1, (sizeof(userData)/sizeof(userData[0]))*(sizeof(testcases)/sizeof(testcases[0])));

			xme_hal_sched_test_resetGlobals(userData[j]);

			taskHandle = xme_hal_sched_addTask(tc->startTime, tc->period, tc->priority, xme_hal_sched_test_taskCallback, userData[j]);
			EXPECT_NE(XME_HAL_SCHED_INVALID_TASK_HANDLE, taskHandle);

			xme_hal_sleep_sleep(tc->sleepTime);

			if (tc->suspendedTime > 0)
			{
				xme_hal_sched_setTaskExecutionState(taskHandle, false);
				xme_hal_sleep_sleep(tc->suspendedTime);
			}

			status = xme_hal_sched_removeTask(taskHandle);
			EXPECT_TRUE(XME_STATUS_SUCCESS == status || XME_STATUS_INVALID_HANDLE == status);

			EXPECT_EQ(tc->expectedCount, globalTaskCounter);
			EXPECT_EQ(0, globalTaskErrors);
		}
	}
}

TEST(XMEHalSchedTest, xme_tests_hal_sched)
{
	// Setup
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_init());

	xme_hal_sched_test_basic();
	xme_hal_sched_test_run();

	// Teardown
	xme_core_fini();
}
