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
 * $Id: testExecutionManagerConfigurationInterface.cpp 5157 2013-09-24 16:29:43Z rupanov $
 */

/**
 * \file
 *         Execution Manager configurator interface tests.
 */

#include <gtest/gtest.h>

#define MODULE_ACRONYM "TstIfEMCfg:"
#define DATAHANDLERMEMORY 15
#define BROKER_ID 80

#include "xme/core/executionManager/test/testHelper_executionManager.h"
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/broker/include/broker.h"
#include "xme/core/dataHandler/include/dataHandler.h"

#include "xme/core/log.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/sync.h"
#include "xme/hal/include/sched.h"
#include "xme/hal/include/context.h"
#include "xme/hal/include/tls.h"

/** \brief Enumeration of lane states. */
typedef enum lane_state_
{
    LS_OFF_OR_SLEEPING = 0,  /*!< lane computer is off or sleeping */
    LS_HYBERNATING = 1,      /*!< system is hypernating (transition to LS_OFF_OR_SLEEPING) */
    LS_SYSTEM_STARTUP = 2,   /*!< system is being started from scratch */
    LS_SYSTEM_RESUME = 3,    /*!< system is being restored from hybernation */
    LS_HW_IDENTIFICATION = 4,  /*!< IDs for lane and DCC are read from EEPROM */
    LS_SYNCHRONIZING = 5,    /*!< system is being synchronized */
    LS_ISOLATED = 6,         /*!< system is isolated (due to severe failure) */
    LS_ALLOCATING = 7,       /*!< system is being reconfigured (resource allocation) */
    LS_NORMAL_OPERATION_INITIALIZE = 8,  /*!< normal operation is being initialized */
    LS_DISPATCHABILITY_CHECKING = 9,     /*!< dispatchability is being checked */
    LS_MAINTENANCE = 10,      /*!< system is being maintained (plug&play may occur here) */
    LS_NORMAL_OPERATION = 11, /*!< normal operation is ongoing */
    LS_MAX = 12,              /*!< number of possible lane state values */
    LS_NO_STATE = 13          /*!< define for 'no state' */
} lane_state_t;

xme_status_t setUp(void);
void tearDown(void);

class ExecutionManagerConfiguratorInterfaceTest : public ::testing::Test {
 protected:
  virtual void SetUp(void);
  virtual void TearDown(void);
};

xme_core_exec_schedule_handle_t schedules[LS_MAX]; /* Global schedule handles */
xme_hal_linkedList_descriptor_t modeList;
//xme_core_dataManager_dataPacketId_t port_platformStatusOwnLane = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

/*****************************************************************************/

xme_status_t xme_core_loop_RegisterModulesCallback(void);

xme_status_t xme_core_loop_CreateChunksCallback(void);

xme_status_t xme_core_loop_ActivateScheduleCallback(void);


xme_status_t setUp(void)
{
    xme_core_broker_initStruct_t brokerParams;
    xme_core_exec_schedule_handle_t* schedule;

    printf("fixture:setup\n");

    /* Initialization of XME components */

        /* Init sync */
        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_hal_sync_init(),
                XME_STATUS_INTERNAL_ERROR,
                XME_LOG_FATAL,
                MODULE_ACRONYM "Initialization of XME sync Module failed!\n");

        /* Init tls */
        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_hal_tls_init(),
                XME_STATUS_INTERNAL_ERROR,
                XME_LOG_FATAL,
                MODULE_ACRONYM "Initialization of XME tls Module failed!\n");

        /* Init sched */
        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_hal_context_init(),
                XME_STATUS_INTERNAL_ERROR,
                XME_LOG_FATAL,
                MODULE_ACRONYM "Initialization of XME context Module failed!\n");

        /* Init sched */
        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_hal_sched_init(),
                XME_STATUS_INTERNAL_ERROR,
                XME_LOG_FATAL,
                MODULE_ACRONYM "Initialization of XME sched Module failed!\n");

        brokerParams.componentId = (xme_core_component_t)BROKER_ID;
        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_broker_init(&brokerParams),
                XME_STATUS_INTERNAL_ERROR,
                XME_LOG_FATAL,
                MODULE_ACRONYM "Initialization of Broker failed!\n");

        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_dataHandler_init(DATAHANDLERMEMORY),
                XME_STATUS_INTERNAL_ERROR,
                XME_LOG_FATAL,
                MODULE_ACRONYM "Initialization of DataHandler failed!\n");

        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_init(NULL),
                  XME_STATUS_INTERNAL_ERROR,
                  XME_LOG_FATAL,
                  MODULE_ACRONYM "Initialization of RTE failed!\n");

        XME_HAL_SINGLYLINKEDLIST_INIT(modeList);

        schedule = (xme_core_exec_schedule_handle_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_handle_t));
        *schedule = (xme_core_exec_schedule_handle_t)LS_NORMAL_OPERATION;
        XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(modeList, schedule);

        schedule = (xme_core_exec_schedule_handle_t*)xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_handle_t));
        *schedule = (xme_core_exec_schedule_handle_t)LS_NORMAL_OPERATION_INITIALIZE;
        XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(modeList, schedule);

        schedule = (xme_core_exec_schedule_handle_t*)xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_handle_t));
        *schedule = (xme_core_exec_schedule_handle_t)LS_OFF_OR_SLEEPING;
        XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(modeList, schedule);

        /* Instantiate components */
         createHelperInstance((xme_core_component_t)1, (xme_core_component_functionId_t)1, 5000000);
         createHelperInstance((xme_core_component_t)2, (xme_core_component_functionId_t)1, 6000000);
         createHelperInstance((xme_core_component_t)3, (xme_core_component_functionId_t)1, 4000000);
         createHelperInstance((xme_core_component_t)4, (xme_core_component_functionId_t)1, 1000000);
         createHelperInstance((xme_core_component_t)5, (xme_core_component_functionId_t)1, 2000000);

         /* Create up a schedule set */
         {
             xme_core_exec_schedule_table_t* table=NULL;
             int i;

             for(i=0; i<LS_MAX; i++)
             {
                  xme_core_exec_scheduler_createScheduleTable(&table, xme_hal_time_timeIntervalFromMicroseconds(5000));
                 xme_core_exec_scheduler_registerSchedule(table, &(schedules[i]));
             }
         }

    return XME_STATUS_SUCCESS;
}

/*-------------------------------------------------------------------------*/
void tearDown(void)
{
    printf("fixture:teardown\n");
    xme_core_exec_fini();
    xme_core_dataHandler_fini();
    xme_core_broker_fini();
}

/*-------------------------------------------------------------------------*/
void ExecutionManagerConfiguratorInterfaceTest::SetUp(void)
{
    int i;
    printf(":setup\n");

    /* Clean up the schedules*/
     for(i=0; i<LS_MAX; i++)
     {
         xme_core_exec_schedule_table_t* schedule = NULL;
         xme_core_exec_scheduler_getSchedule((schedules[i]), &schedule);
         xme_core_exec_scheduler_clearScheduleTable(&schedule, false);
     }

     /* We have now a fully configured instance of executionManager */
}

/*-------------------------------------------------------------------------*/
void ExecutionManagerConfiguratorInterfaceTest::TearDown(void)
{
    xme_core_exec_configurator_rollback((xme_core_exec_transactionId_t)1);
    clearExecutionCounters();
    printf(":teardown\n");
}

//*=========================================================================*/
TEST_F(ExecutionManagerConfiguratorInterfaceTest,addToScheduleLargerThan)
{
    xme_status_t status;

    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)1, (xme_core_component_t)2, (xme_core_component_functionId_t)1, NULL, 0, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION, status);
    xme_core_exec_configurator_rollback((xme_core_exec_transactionId_t)1);
}

void test_Configurator_addToScheduleWrongCommit(void)
{
    xme_status_t status;

    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)1, (xme_core_component_t)1, (xme_core_component_functionId_t)1, NULL, 0, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_SUCCESS,
            status);

    status = xme_core_exec_configurator_commit((xme_core_exec_transactionId_t)2);

    ASSERT_EQ(XME_STATUS_UNEXPECTED,
                status);

    /*Check number of executions*/
}

TEST_F(ExecutionManagerConfiguratorInterfaceTest,addToScheduleJustTheSize)
{
    xme_status_t status;

    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)1, (xme_core_component_t)1, (xme_core_component_functionId_t)1, NULL, 0, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_SUCCESS,
            status);
    status = xme_core_exec_configurator_commit((xme_core_exec_transactionId_t)1);

    ASSERT_EQ(XME_STATUS_SUCCESS,
                status);
    /* todo: for all schedules check the execution counter */
    xme_core_exec_scheduler_activateSchedule(schedules[LS_NORMAL_OPERATION]);

    status = xme_core_exec_run(1, false);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                status);

    /*Check number of executions*/
    ASSERT_EQ(1U, getExecutionCount((xme_core_component_t)1,(xme_core_component_functionId_t)1));

}


TEST_F(ExecutionManagerConfiguratorInterfaceTest, test_Configurator_addToScheduleJustTheSizeButFalseOffset)
{
    xme_status_t status;

    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)1, (xme_core_component_t)2, (xme_core_component_functionId_t)1, NULL, 1, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION,
            status);
    status = xme_core_exec_configurator_rollback((xme_core_exec_transactionId_t)1);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                    status);
}

TEST_F(ExecutionManagerConfiguratorInterfaceTest, addToScheduleJustTheSizeTwoDifferent)
{
    xme_status_t status;

    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)5, (xme_core_component_t)5, (xme_core_component_functionId_t)1, NULL, 0, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                status);
    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)5, (xme_core_component_t)4, (xme_core_component_functionId_t)1, NULL, 0, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                status);
    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)5, (xme_core_component_t)5, (xme_core_component_functionId_t)1, NULL, 0, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                status);

    status = xme_core_exec_configurator_commit((xme_core_exec_transactionId_t)5);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                       status);

    xme_core_exec_scheduler_activateSchedule(schedules[(int)LS_NORMAL_OPERATION]);

    status = xme_core_exec_run(1,false);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                    status);

    /*Check number of executions*/
    ASSERT_EQ(2U, getExecutionCount((xme_core_component_t)5,(xme_core_component_functionId_t)1));
    ASSERT_EQ(1U, getExecutionCount((xme_core_component_t)4,(xme_core_component_functionId_t)1));
}

TEST_F(ExecutionManagerConfiguratorInterfaceTest, addToScheduleJustTheSizeLastFirst)
{
    xme_status_t status;

    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)6, (xme_core_component_t)5, (xme_core_component_functionId_t)1, NULL, 5000000-2000000, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                status);

    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)6, (xme_core_component_t)4, (xme_core_component_functionId_t)1, NULL, 0, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                status);
    status = xme_core_exec_configurator_addComponentToSchedule((xme_core_exec_transactionId_t)6, (xme_core_component_t)5, (xme_core_component_functionId_t)1, NULL, 0, 5000000, 0, 0, &modeList);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                status);

    status = xme_core_exec_configurator_commit((xme_core_exec_transactionId_t)6);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                       status);

    xme_core_exec_scheduler_activateSchedule(schedules[(int)LS_NORMAL_OPERATION]);

    status = xme_core_exec_run(1,false);
    ASSERT_EQ(XME_STATUS_SUCCESS,
                    status);

    /*Check number of executions*/
    ASSERT_EQ(2U, getExecutionCount((xme_core_component_t)5,(xme_core_component_functionId_t)1));
    ASSERT_EQ(1U, getExecutionCount((xme_core_component_t)4,(xme_core_component_functionId_t)1));
}

xme_status_t xme_core_loop_RegisterModulesCallback(void)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t xme_core_loop_CreateChunksCallback(void)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t xme_core_loop_ActivateScheduleCallback(void)
{
    return XME_STATUS_SUCCESS;
}

int main(int argc, char **argv)
{
    int result = -1;
    ::testing::InitGoogleTest(&argc, argv);

    setUp();
    result = RUN_ALL_TESTS();
    tearDown();
    return result;
}
