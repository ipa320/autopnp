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
 * $Id: smokeTestExecutionManager.cpp 4254 2013-07-17 13:13:22Z geisinger $
 */

/**
 * \file
 *         Execution Manager smoke tests.
 */

#include <gtest/gtest.h>
#include "testFunctionsExecutionManager.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"

#include "xme/core/component.h"
#include "xme/hal/include/sync.h"
#include "xme/hal/include/sched.h"

/*****************************************************************************/
/***        "int main()" interface                                         ***/
/*****************************************************************************/

// Currently not implemented: separate thread
TEST(ExecutionManagerSmokeTest, SeparateThread)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init(&testConfig));
    ASSERT_EQ(XME_STATUS_UNSUPPORTED, xme_core_exec_run(0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}

//Double Init / Fini
TEST(ExecutionManagerSmokeTest, DISABLED_DoubleInitFini)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init(&testConfig));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init(&testConfig));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}

//Fini before Init should theoretically fail
TEST(ExecutionManagerSmokeTest, DISABLED_FiniBeforeInit)
{
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_exec_fini());
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init(&testConfig));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}

//Run before Init
TEST(ExecutionManagerSmokeTest, DISABLED_RunBeforeInit)
{
    ASSERT_EQ(XME_STATUS_UNEXPECTED, xme_core_exec_run(1, false));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init(&testConfig));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}

#if 0
//Run after Fini
TEST(ExecutionManagerSmokeTest, SmokeTest_runAfterFini)
{
    xme_core_exec_schedule_handle_t schedule = XME_CORE_EM_SCHEDULE_HANDLE_INVALID;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init());
    schedule = setupEmptySchedule();
    registerCounterFunction(CID1, FID1);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_exec_run(1, false));
}

//Double Init
TEST(ExecutionManagerTest, DISABLE_SmokeTest_doubleInit)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init());
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_exec_init());
}

#endif

//Double Fini
TEST(ExecutionManagerSmokeTest, DISABLED_DoubleFini)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init(&testConfig));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_exec_fini());
}

#if 0
//Double Stop
TEST(ExecutionManagerTest, DISABLE_SmokeTest_doubleStop)
{
    xme_core_exec_schedule_handle_t schedule = XME_CORE_EM_SCHEDULE_HANDLE_INVALID;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init());
    schedule = setupEmptySchedule();
    registerCounterFunction(CID1, FID1);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_run(1, false));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_stop());
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_exec_stop());
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}
#endif


#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
/*****************************************************************************/
/***        Schedule manipulation interface                                ***/
/*****************************************************************************/


/*****************************************************************************/
/*****************************************************************************/
#ifdef TESTS_REFACTORED
//TESTS FOR SCHEDULER
//Test Scheduler::addElementToScheduleTable
TEST(ExecutionManagerTest, scheduler_addElementToScheduleTable)
{
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init());

	//creaing a new schedule table
	xme_core_exec_schedule_table_t* scheduleTable;
	xme_hal_time_timeInterval_t cycleLength = xme_hal_time_timeIntervalFromSeconds(10);
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_createScheduleTable(&scheduleTable,cycleLength));
	a
	//create function 1
	xme_core_component_t comp1 = (xme_core_component_t)1;
	xme_core_component_functionId_t comp1Func= (xme_core_component_functionId_t)11;
	xme_hal_time_timeInterval_t comp1FuncStart = 0;
	xme_hal_time_timeInterval_t comp1FuncEnd = xme_hal_time_timeIntervalFromSeconds(2);

	//create function 2
	xme_core_component_t comp2 = (xme_core_component_t)2;
	xme_core_component_functionId_t comp2Func= (xme_core_component_functionId_t)21;
	xme_hal_time_timeInterval_t comp2FuncStart = xme_hal_time_timeIntervalFromSeconds(2);
	xme_hal_time_timeInterval_t comp2FuncEnd = xme_hal_time_timeIntervalFromSeconds(3);

	//create function 3
	xme_core_component_t comp3 = (xme_core_component_t)3;
	xme_core_component_functionId_t comp3Func= (xme_core_component_functionId_t)31;
    xme_hal_time_timeInterval_t comp3FuncStart = xme_hal_time_timeIntervalFromMilliseconds(2500);
    xme_hal_time_timeInterval_t comp3FuncEnd = xme_hal_time_timeIntervalFromMilliseconds(3500);

	//create function 4
	xme_core_component_t comp4 = (xme_core_component_t)4;
	xme_core_component_functionId_t comp4Func= (xme_core_component_functionId_t)41;
	xme_hal_time_timeInterval_t comp4FuncStart = xme_hal_time_timeIntervalFromMilliseconds(9500);
	xme_hal_time_timeInterval_t comp4FuncEnd = xme_hal_time_timeIntervalFromMilliseconds(10500);

	//add function 11, EXPECT SUCCESS
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable, comp1, comp1Func, comp1FuncStart, comp1FuncEnd, false));
	//add function 21, EXPECT SUCCESS
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable, comp2, comp2Func, comp2FuncStart, comp2FuncEnd, false));
	//add function 31, EXPECT ERROR:SCHEDULE OVERLAP
	EXPECT_EQ(XME_STATUS_OUT_OF_RESOURCES, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable, comp3, comp3Func, comp3FuncStart, comp3FuncEnd, false));
	//add function 41, EXPECT ERROR:SCHEDULE OUT OF CYCLE
	EXPECT_EQ(XME_STATUS_OUT_OF_RESOURCES, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable, comp4, comp4Func, comp4FuncStart, comp4FuncEnd, false));

	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}
/*
//Test Scheduler::findWindow
TEST(ExecutionManagerTest, scheduler_findWindow)
{
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init());

	//creates a scheduleTable
	xme_core_exec_schedule_table_t* scheduleTable;
	xme_hal_time_timeInterval_t cycleLength = 10000000;
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_createScheduleTable(&scheduleTable,cycleLength));

	//create function 1
	xme_core_component_t comp1 = (xme_core_component_t)1;
	xme_core_component_functionId_t comp1Func = (xme_core_component_functionId_t)11;
	xme_hal_time_timeInterval_t comp1FuncStart = 0;
	xme_hal_time_timeInterval_t comp1FuncEnd = 5000000;

	//create function 2
	xme_core_component_t comp2 = (xme_core_component_t)2;
	xme_core_component_functionId_t comp2Func = (xme_core_component_functionId_t)21;
	xme_hal_time_timeInterval_t comp2FuncStart = 6000000;
	xme_hal_time_timeInterval_t comp2FuncEnd = 10000000;

	//too long WCET
	xme_hal_time_timeInterval_t tryWCET1 = 2000000;
	//resonable WCET
	xme_hal_time_timeInterval_t tryWCET2 = 800000;
	//window start time
	xme_hal_time_timeInterval_t windowStart = 0;

	//add function 11, EXPECT SUCCESS
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable, comp1, comp1Func, comp1FuncStart, comp1FuncEnd, false));
	//add function 21, EXPECT SUCCESS
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable, comp2, comp2Func, comp2FuncStart, comp2FuncEnd, false));

	//try WCET 1, EXPECT NOT FOUND
	EXPECT_EQ(XME_STATUS_NOT_FOUND, findWindow(scheduleTable, tryWCET1, windowStart));
	//try WCET 2, EXPECT SUCCESS
	EXPECT_EQ(XME_STATUS_SUCCESS, findWindow(scheduleTable, tryWCET2, windowStart));

	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}
*/
//Test Scheduler::registerSchedule
TEST(ExecutionManagerTest, scheduler_registerSchedule)
{
}

//Test Scheduler::getSchedule
TEST(ExecutionManagerTest, scheduler_getSchedule)
{


	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init());

	//create and register schedule table 1
	xme_core_exec_schedule_table_t* scheduleTable1;
	xme_hal_time_timeInterval_t cycleLength = 10000000;
	xme_core_exec_schedule_handle_t scheduleId1;
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_createScheduleTable(&scheduleTable1, cycleLength));
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(scheduleTable1, &scheduleId1));

	//create and register schedule table 2
	xme_core_exec_schedule_table_t* scheduleTable2;
	xme_core_exec_schedule_handle_t scheduleId2;
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_createScheduleTable(&scheduleTable2, cycleLength));
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(scheduleTable2, &scheduleId2));

	//create an empty scheduleId
	xme_core_exec_schedule_handle_t scheduleId3 = (xme_core_exec_schedule_handle_t)100;

	//new scheduleTable
	xme_core_exec_schedule_table_t* checkScheduleTable;

	//check schedule table 1, EXPECT SUCCESS
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_getSchedule(scheduleId1, &checkScheduleTable));
	//check schedule table 2, EXPECT SUCCESS
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_getSchedule(scheduleId2, &checkScheduleTable));
	//check schedule table empty, EXPECT NO FOUND
	EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_exec_scheduler_getSchedule(scheduleId3, &checkScheduleTable));

	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}


//Test Scheduler::activateSchedule
TEST(ExecutionManagerTest, scheduler_activateSchedule)
{
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init());

	//create and register schedule table 1
	xme_core_exec_schedule_table_t* scheduleTable1;
	xme_hal_time_timeInterval_t cycleLength = 10000000;
	xme_core_exec_schedule_handle_t scheduleId1;
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_createScheduleTable(&scheduleTable1, cycleLength));
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(scheduleTable1, &scheduleId1));

	//create and register schedule table 2
	xme_core_exec_schedule_table_t* scheduleTable2;
	xme_core_exec_schedule_handle_t scheduleId2;
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_createScheduleTable(&scheduleTable2, cycleLength));
	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(scheduleTable2, &scheduleId2));

	//popolate schedule table 1
	xme_core_component_t comp1 = (xme_core_component_t)1;
	xme_core_component_functionId_t comp1Func1= (xme_core_component_functionId_t)11;
	xme_hal_time_timeInterval_t comp1Func1Start = 0;
	xme_hal_time_timeInterval_t comp1Func1End = 1000000;
	xme_core_component_functionId_t comp1Func2= (xme_core_component_functionId_t)12;
	xme_hal_time_timeInterval_t comp1Func2Start = 1000000;
	xme_hal_time_timeInterval_t comp1Func2End = 2000000;
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable1, comp1, comp1Func1, comp1Func1Start, comp1Func1End, false));
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable1, comp1, comp1Func2, comp1Func2Start, comp1Func2End, false));

	//popolate schedule table 2
	xme_core_component_t comp2 = (xme_core_component_t)2;
	xme_core_component_functionId_t comp2Func1= (xme_core_component_functionId_t)21;
	xme_hal_time_timeInterval_t comp2Func1Start = 2000000;
	xme_hal_time_timeInterval_t comp2Func1End = 3000000;
	xme_core_component_functionId_t comp2Func2= (xme_core_component_functionId_t)22;
	xme_hal_time_timeInterval_t comp2Func2Start = 3000000;
	xme_hal_time_timeInterval_t comp2Func2End = 3000000;
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable2, comp2, comp2Func1, comp2Func1Start, comp2Func1End, false));
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_addElementToScheduleTable(scheduleTable2, comp2, comp2Func2, comp2Func2Start, comp2Func2End, false));

	//create unregistered schedule ID
	xme_core_exec_schedule_handle_t checkScheduleId = (xme_core_exec_schedule_handle_t)100;

	//check activate schedule registered, EXPECT SUCCESS
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_activateSchedule(scheduleId1));
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_activateSchedule(scheduleId1));
	//check activate schedule registered, EXPECT INVALID PARAMETER
	EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_exec_scheduler_activateSchedule(checkScheduleId));

	ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}

//Test Scheduler::initializeLoop
TEST(ExecutionManagerTest, scheduler_initializeLoop)
{}

//Test Scheduler::calculateNextComponent
TEST(ExecutionManagerTest, scheduler_calculateNextComponent)
{}
#endif
