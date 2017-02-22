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
 * $Id: testExecutionManagerScheduleManagementInterface.cpp 5157 2013-09-24 16:29:43Z rupanov $
 */

/**
 * \file
 *         Execution Manager schedule management interface tests.
 */

#include <gtest/gtest.h>
#include "xme/core/executionManager/test/testHelper_executionManager.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/hal/include/linkedList.h"
#include "xme/hal/include/mem.h"

TEST(ExecutionManagerScheduleTest, scheduler_replicateScheduleTable)
{
    xme_core_exec_schedule_table_t* scheduleTables[3];
    xme_core_exec_schedule_table_t* targetTables[3];
    int index = 0;

    xme_hal_time_timeInterval_t cycleLength = xme_hal_time_timeIntervalFromMilliseconds(10);

    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_createScheduleTable(&(scheduleTables[0]), cycleLength));
    ASSERT_EQ(XME_STATUS_SUCCESS,
                xme_core_exec_scheduler_createScheduleTable(&(scheduleTables[1]), cycleLength));
    ASSERT_EQ(XME_STATUS_SUCCESS,
                xme_core_exec_scheduler_createScheduleTable(&(scheduleTables[2]), cycleLength));

    ASSERT_EQ(XME_STATUS_SUCCESS,
    xme_core_exec_scheduler_addElementToScheduleTable(scheduleTables[0], (xme_core_component_t)1, (xme_core_component_functionId_t)1, NULL, 0, 100, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
    xme_core_exec_scheduler_addElementToScheduleTable(scheduleTables[0], (xme_core_component_t)2, (xme_core_component_functionId_t)2, NULL, 100, 200, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
    xme_core_exec_scheduler_addElementToScheduleTable(scheduleTables[0], (xme_core_component_t)3, (xme_core_component_functionId_t)3, NULL, 300, 400, 0, 0, true));

    ASSERT_EQ(XME_STATUS_SUCCESS,
    xme_core_exec_scheduler_replicateSchedule(&(targetTables[0]), scheduleTables[0]));
    ASSERT_EQ(XME_STATUS_SUCCESS,
    xme_core_exec_scheduler_replicateSchedule(&(targetTables[1]), scheduleTables[1]));

    /**/
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(targetTables[0]->entries,xme_core_exec_schedule_table_entry_t, item);
        xme_core_exec_schedule_table_entry_t* itemOld = (xme_core_exec_schedule_table_entry_t*)xme_hal_singlyLinkedList_itemFromIndex(&(scheduleTables[0]->entries), index);
        ASSERT_EQ(itemOld->componentId, item->componentId);
        ASSERT_EQ(itemOld->functionId, item->functionId);
        ASSERT_EQ(itemOld->slotStart_ns, item->slotStart_ns);
        ASSERT_EQ(itemOld->slotLength_ns, item->slotLength_ns);
        index++;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    ASSERT_EQ(index, 3);
}

TEST(ExecutionManagerScheduleTest, scheduler_replicateScheduleSet)
{
    // TODO: Move static size to Options.cmake
    xme_hal_singlyLinkedList_t(10) nodeSchedulesA;
    // TODO: Move static size to Options.cmake
    xme_hal_singlyLinkedList_t(10) nodeSchedulesB;
    xme_core_exec_schedule_table_t* table = NULL;

    /* Work with separate linked lists */
    XME_HAL_SINGLYLINKEDLIST_INIT(nodeSchedulesA);
    XME_HAL_SINGLYLINKEDLIST_INIT(nodeSchedulesB);

    /* Add 4 items to A-list */
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_createScheduleTable(&table, xme_hal_time_timeIntervalFromMilliseconds(1000)));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)1, (xme_core_component_functionId_t)3, NULL,0,100, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)2, (xme_core_component_functionId_t)2, NULL,100,200, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)3, (xme_core_component_functionId_t)1, NULL,300,400, 0, 0, true));

    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_hal_singlyLinkedList_addItem(&nodeSchedulesA, table));

    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_createScheduleTable(&table, xme_hal_time_timeIntervalFromMilliseconds(1000)));

    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)2, (xme_core_component_functionId_t)1, NULL,0,100, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)3, (xme_core_component_functionId_t)2, NULL,100,200, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)1, (xme_core_component_functionId_t)3, NULL,300,400, 0, 0, true));

    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_hal_singlyLinkedList_addItem(&nodeSchedulesA, table));

    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_createScheduleTable(&table, xme_hal_time_timeIntervalFromMilliseconds(1000)));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)3, (xme_core_component_functionId_t)3, NULL,0,100, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)1, (xme_core_component_functionId_t)2, NULL,100,200, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)2, (xme_core_component_functionId_t)1, NULL,300,400, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_hal_singlyLinkedList_addItem(&nodeSchedulesA, table));

    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_createScheduleTable(&table, xme_hal_time_timeIntervalFromMilliseconds(1000)));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)1, (xme_core_component_functionId_t)3, NULL,0,100, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)3, (xme_core_component_functionId_t)2, NULL,100,200, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(table, (xme_core_component_t)2, (xme_core_component_functionId_t)1, NULL,300,400, 0, 0, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_hal_singlyLinkedList_addItem(&nodeSchedulesA, table));


    /* Replicate to B-list */
    /* TODO: xme_hal_linkedList_descriptor_t is an internal type, it should not be used directly! */
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_replicateFullScheduleSet((xme_hal_linkedList_descriptor_t*) &nodeSchedulesA, (xme_hal_linkedList_descriptor_t*) &nodeSchedulesB));

    xme_hal_singlyLinkedList_fini(&nodeSchedulesA);

    XME_HAL_SINGLYLINKEDLIST_INIT(nodeSchedulesA);

    ASSERT_EQ(0, xme_hal_singlyLinkedList_getItemCount(&nodeSchedulesA));

    /* Replicate back to A-list */
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_replicateFullScheduleSet((xme_hal_linkedList_descriptor_t*) &nodeSchedulesB, (xme_hal_linkedList_descriptor_t*) &nodeSchedulesA));

    ASSERT_EQ(4, xme_hal_singlyLinkedList_getItemCount(&nodeSchedulesA));

    xme_hal_singlyLinkedList_fini(&nodeSchedulesA);
    xme_hal_singlyLinkedList_fini(&nodeSchedulesB);
}

//Test Scheduler::addFunctionToSchedule
TEST(ExecutionManagerScheduleTest, scheduler_addFunctionToSchedule)
{
    //create and register a schedule table
    xme_core_exec_schedule_table_t* scheduleTable;
    xme_hal_time_timeInterval_t cycleLength = xme_hal_time_timeIntervalFromMilliseconds(10);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_createScheduleTable(&scheduleTable, cycleLength));

    //create function 1
    xme_core_component_t comp1 = (xme_core_component_t)1;
    xme_core_component_functionId_t comp1Func= (xme_core_component_functionId_t)11;

    //create function 2
    xme_core_component_t comp2 = (xme_core_component_t)2;
    xme_core_component_functionId_t comp2Func= (xme_core_component_functionId_t)21;

    //create function 3
    xme_core_component_t comp3 = (xme_core_component_t)3;
    xme_core_component_functionId_t comp3Func= (xme_core_component_functionId_t)31;

    //create function 4
    xme_core_component_t comp4 = (xme_core_component_t)4;
    xme_core_component_functionId_t comp4Func= (xme_core_component_functionId_t)41;

    /* This will fail while it has start 9ms and length 2ms, which goes beyond 10ms*/
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION,
            xme_core_exec_scheduler_addElementToScheduleTable(
                    scheduleTable,
                    comp1,
                    comp1Func,
                    NULL,
                    xme_hal_time_timeIntervalFromMilliseconds(9),
                    xme_hal_time_timeIntervalFromMilliseconds(2),
                    0,0,
                    true)
        );

    /* OK: This adds a slot [1,3) */
    EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(
                    scheduleTable,
                    comp2,
                    comp2Func,
                    NULL,
                    xme_hal_time_timeIntervalFromMilliseconds(1),
                    xme_hal_time_timeIntervalFromMilliseconds(2),
                    0,0,
                    true)
        );

    /* Not OK: we attach at the end of slot [1,3):  */
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION,
            xme_core_exec_scheduler_addElementToScheduleTable(
                    scheduleTable,
                    comp1,
                    comp1Func,
                    NULL,
                    xme_hal_time_timeIntervalFromMilliseconds(2),
                    xme_hal_time_timeIntervalFromMilliseconds(2),
                    0,0,
                    true)
        );

    /* OK: This attaches to the end of [1,3):[3,5) */
    EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(
                    scheduleTable,
                    comp3,
                    comp3Func,
                    NULL,
                    xme_hal_time_timeIntervalFromMilliseconds(3),
                    xme_hal_time_timeIntervalFromMilliseconds(2),
                    0,0,
                    true)
        );

    /* OK: This attaches to the end of [1,3),[3,5): [5,10) */
    EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_exec_scheduler_addElementToScheduleTable(
                    scheduleTable,
                    comp4,
                    comp4Func,
                    NULL,
                    xme_hal_time_timeIntervalFromMilliseconds(5),
                    xme_hal_time_timeIntervalFromMilliseconds(5),
                    0,0,
                    true)
        );

    /* OK: This attaches to the beginning of [1,3),[3,5),[5,10): [0,1) */
    EXPECT_EQ(XME_STATUS_SUCCESS,
                xme_core_exec_scheduler_addElementToScheduleTable(
                        scheduleTable,
                        comp1,
                        comp1Func,
                        NULL,
                        xme_hal_time_timeIntervalFromMilliseconds(0),
                        xme_hal_time_timeIntervalFromMilliseconds(1),
                        0,0,
                        true)
            );

    EXPECT_EQ(4, XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(scheduleTable->entries));

    EXPECT_EQ(comp1,((xme_core_exec_schedule_table_entry_t*)(XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(scheduleTable->entries,0)))->componentId);
    EXPECT_EQ(comp2,((xme_core_exec_schedule_table_entry_t*)(XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(scheduleTable->entries,1)))->componentId);
    EXPECT_EQ(comp3,((xme_core_exec_schedule_table_entry_t*)(XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(scheduleTable->entries,2)))->componentId);
    EXPECT_EQ(comp4,((xme_core_exec_schedule_table_entry_t*)(XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(scheduleTable->entries,3)))->componentId);

    //ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
}


TEST(ExecutionManagerScheduleTest, scheduler_clearScheduleTable)
{
    xme_core_exec_schedule_table_t* scheduleTable;
    xme_hal_time_timeInterval_t cycleLength = xme_hal_time_timeIntervalFromMilliseconds(10);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_createScheduleTable(&scheduleTable, cycleLength));

    //create function 1
    xme_core_component_t comp1 = (xme_core_component_t)1;
    xme_core_component_functionId_t comp1Func= (xme_core_component_functionId_t)11;

    //create function 2
    xme_core_component_t comp2 = (xme_core_component_t)2;
    xme_core_component_functionId_t comp2Func= (xme_core_component_functionId_t)21;

    /* OK: This attaches to the beginning of [1,3),[3,5),[5,10): [0,1) */
    EXPECT_EQ(XME_STATUS_SUCCESS,
                xme_core_exec_scheduler_addElementToScheduleTable(
                        scheduleTable,
                        comp1,
                        comp1Func,
                        NULL,
                        xme_hal_time_timeIntervalFromMilliseconds(0),
                        xme_hal_time_timeIntervalFromMilliseconds(1),
                        0,0,
                        true)
            );

    /* OK: This attaches to the beginning of [1,3),[3,5),[5,10): [0,1) */
    EXPECT_EQ(XME_STATUS_SUCCESS,
                xme_core_exec_scheduler_addElementToScheduleTable(
                        scheduleTable,
                        comp2,
                        comp2Func,
                        NULL,
                        xme_hal_time_timeIntervalFromMilliseconds(1),
                        xme_hal_time_timeIntervalFromMilliseconds(2),
                        0,0,
                        true)
            );

    EXPECT_EQ(2, xme_hal_singlyLinkedList_getItemCount(&(scheduleTable->entries)));
    xme_core_exec_scheduler_clearScheduleTable(&scheduleTable, false);
    EXPECT_EQ(0, xme_hal_singlyLinkedList_getItemCount(&(scheduleTable->entries)));
    xme_core_exec_scheduler_clearScheduleTable(&scheduleTable, true);
    EXPECT_EQ(NULL, scheduleTable);

}
