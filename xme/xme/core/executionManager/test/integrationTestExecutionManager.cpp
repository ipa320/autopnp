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
 * $Id: integrationTestExecutionManager.cpp 5157 2013-09-24 16:29:43Z rupanov $
 */

/**
 * \file
 *         Execution Manager integration tests.
 */

#include <gtest/gtest.h>
#include "testHelper_executionManager.h"
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"

#include "xme/core/log.h"

#include "xme/defines.h"


const int TEST_FUNCTIONS_TOTAL = 10;
const int TEST_NSCHEDULES = 5;

xme_status_t customCallbackAsync(void* arg);
xme_status_t customCallbackSync(void* arg);

class ExecutionManagerIntegrationTest : public ::testing::Test {
public:
    ExecutionManagerIntegrationTest()
    {}
protected:
  virtual void SetUp(void)
  {
		int index = 0;
		/* Initialize execution manager */
		xme_core_exec_init(&testConfig);

		/* Create functions */
		for (index = 0; index < TEST_FUNCTIONS_TOTAL; index++) {
			cid[index] = (xme_core_component_t) (index + 1);
			fid[index] = (xme_core_component_functionId_t) 1;
			args[index] = NULL;
			createHelperInstance(cid[index], fid[index], 1);
		}

		/* Set up a schedule set using configuration functions */
		for (index = 0; index < 5; index++) {
			xme_core_exec_schedule_table_t* table;
			xme_core_exec_scheduler_createScheduleTable(&table,
					xme_hal_time_timeIntervalFromMilliseconds(10 * index + 10));
			xme_core_exec_scheduler_registerSchedule(table,
					&(schedules[index]));
		}

		/* Fill in configuration schedule sets */
		for (index = 0; index < TEST_FUNCTIONS_TOTAL; index++) {
			XME_HAL_SINGLYLINKEDLIST_INIT(scheduleSets[index]);

			/* schedules[0] has all the items */
			XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(scheduleSets[index],
					&(schedules[0]));

			if (index % 2) {
				XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(scheduleSets[index],
						&(schedules[1]));
				XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(scheduleSets[index],
						&(schedules[3]));
			} else {
				XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(scheduleSets[index],
						&(schedules[2]));
				XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(scheduleSets[index],
						&(schedules[4]));
			}
		}

		/* Schedule the functions */
		for (index = 0; index < TEST_FUNCTIONS_TOTAL; index++) {
			xme_hal_linkedList_descriptor_t* currentScheduleSet;

			xme_core_exec_scheduler_getScheduleSetPointer(&currentScheduleSet);
			xme_core_exec_scheduler_addFunctionToScheduleSet(currentScheduleSet,
					&(scheduleSets[index]), cid[index], fid[index], NULL, 0,
					1000, 0, 0);
		}
		clearExecutionCounters();
  }

  virtual void TearDown(void)
  {
	    xme_core_exec_fini();
  }

protected:
  xme_core_component_t cid[TEST_FUNCTIONS_TOTAL];
  xme_core_component_functionId_t fid[TEST_FUNCTIONS_TOTAL];
  void* args[TEST_FUNCTIONS_TOTAL];
  xme_core_exec_schedule_handle_t schedules[TEST_NSCHEDULES];
  xme_hal_linkedList_descriptor_t scheduleSets[TEST_FUNCTIONS_TOTAL];
};

xme_status_t customCallbackAsync(void* arg)
{
    XME_UNUSED_PARAMETER(arg);
    printf("AsyncStop\n");
    xme_core_exec_stop(false);
    return XME_STATUS_SUCCESS;
}

xme_status_t customCallbackSync(void* arg)
{
    XME_UNUSED_PARAMETER(arg);
    printf("SyncStop\n");
    xme_core_exec_stop(true);
    return XME_STATUS_SUCCESS;
}

TEST_F(ExecutionManagerIntegrationTest, SimpleRun)
{
    int index = 0;

    printf("-Schedule 0---\n");

	/* Activate one of the schedules */
	xme_core_exec_scheduler_activateSchedule(schedules[0]);

	/* Run for 10 cycles */
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_run(10, false));

	/* Check if every function has executed once */
	for(index=0; index<TEST_FUNCTIONS_TOTAL; index++)
		ASSERT_EQ(10U, getExecutionCount((xme_core_component_t)(index+1), (xme_core_component_functionId_t)1))<<"index "<<index;

	/* Activate one of the schedules */
	xme_core_exec_scheduler_activateSchedule(schedules[1]);
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_run(5, false));

	/* Activate one of the schedules */
	xme_core_exec_scheduler_activateSchedule(schedules[2]);
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_run(5, false));

	/* Check if every function has executed once */
	for(index=0; index<TEST_FUNCTIONS_TOTAL; index++)
		ASSERT_EQ(15U, getExecutionCount((xme_core_component_t)(index+1), (xme_core_component_functionId_t)1))<<"index "<<index;

    printf("-Schedule 3---\n");
    /* Activate one of the schedules */
    xme_core_exec_scheduler_activateSchedule(schedules[3]);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_run(5, false));

    printf("-Schedule 4---\n");
    /* Activate one of the schedules */
    xme_core_exec_scheduler_activateSchedule(schedules[4]);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_run(5, false));

    /* Check if every function has executed once */
	for(index=0; index<TEST_FUNCTIONS_TOTAL; index++)
		ASSERT_EQ(20U, getExecutionCount((xme_core_component_t)(index+1), (xme_core_component_functionId_t)1))<<"index "<<index;
}

TEST_F(ExecutionManagerIntegrationTest, RunWithAsynchronousStop)
{
	int index = 0;
	functionPointers[4+1] = customCallbackAsync;

	/* Scenario: function 4 will trigger async stop, so no more functions will be executed within the cycle*/

	/* Activate the schedule with all the functions */
	xme_core_exec_scheduler_activateSchedule(schedules[0]);

	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_run(5, false));

	/* Check if every function has executed once */
	for(index=0; index<4; index++)
		ASSERT_EQ(1U, getExecutionCount((xme_core_component_t)(index+1), (xme_core_component_functionId_t)1))<<"index "<<index;

	/* Check if every function has executed once */
	for(index=4; index<TEST_FUNCTIONS_TOTAL; index++)
		ASSERT_EQ(0U, getExecutionCount((xme_core_component_t)(index+1), (xme_core_component_functionId_t)1))<<"index "<<index;
}

TEST_F(ExecutionManagerIntegrationTest, RunWithSynchronousStop)
{
	int index = 0;
	functionPointers[4+1] = customCallbackSync;

	/* Scenario: function 4 will trigger sync stop, so each function will be executed once within the cycle*/

	/* Activate the schedule with all the functions */
	xme_core_exec_scheduler_activateSchedule(schedules[0]);

	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_run(5, false));

	/* Check if every function has executed once */
	for(index=0; index<4; index++)
		ASSERT_EQ(1U, getExecutionCount((xme_core_component_t)(index+1), (xme_core_component_functionId_t)1))<<"index "<<index;

	/* Check if every function has executed once */
	for(index=4; index<TEST_FUNCTIONS_TOTAL; index++)
		ASSERT_EQ(1U, getExecutionCount((xme_core_component_t)(index+1), (xme_core_component_functionId_t)1))<<"index "<<index;
}
