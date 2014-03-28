
#include <gtest/gtest.h>
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include <stdio.h>

static void initStateMockup();
static void finiStateMockup();

static void initStateMockup()
{
	xme_core_exec_state.lock = xme_hal_sync_createCriticalSection();
	xme_core_exec_state.value = XME_CORE_EXEC_PAUSED;
}

static void finiStateMockup()
{
	xme_hal_sync_destroyCriticalSection(xme_core_exec_state.lock);
	xme_core_exec_state.lock = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;
	xme_core_exec_state.value = XME_CORE_EXEC_NOT_INITIALIZED;
}


const int NUM_TEST_TABLES = 10;
const int NUM_TEST_ENTRIES = 10;

int main(int argc, char **argv)
{
    int result = -1;
    ::testing::InitGoogleTest(&argc, argv);

    result = RUN_ALL_TESTS();
    return result;
}



class smokeTest_ExecutionManager_Scheduler_activateSchedule : public ::testing::Test {
 protected:
    virtual void SetUp(void)
    {
    	xme_hal_sync_init();
    	xme_core_exec_scheduler_init();

    	/* Fake-init EM */
    	initStateMockup();

    }

    virtual void TearDown(void)
    {
    	xme_core_exec_scheduler_fini();

    	/* Fake-fini EM */
		finiStateMockup();

    	xme_hal_sync_fini();
    }



    xme_core_exec_schedule_table_t scheduleTables[NUM_TEST_TABLES];
    xme_core_exec_schedule_table_entry_t scheduleEntries[NUM_TEST_ENTRIES];
    xme_core_exec_schedule_handle_t scheduleIds[NUM_TEST_TABLES];
};



TEST_F(smokeTest_ExecutionManager_Scheduler_activateSchedule, schedule_list_is_empty__1st_schedule_has_id_0)
{
    xme_core_exec_schedule_handle_t next_schedule_entry;


    scheduleTables[0].majorCycleDuration_ns = 10000;
    XME_HAL_SINGLYLINKEDLIST_INIT(scheduleTables[0].entries);

    scheduleEntries[0].componentId=(xme_core_component_t)10;
    scheduleEntries[0].functionId=(xme_core_component_functionId_t)5;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&(scheduleTables[0]), &(scheduleEntries[0])));
    xme_core_exec_scheduler_registerSchedule(&(scheduleTables[0]), &(scheduleIds[0]));


    scheduleTables[1].majorCycleDuration_ns = 10000;
    XME_HAL_SINGLYLINKEDLIST_INIT(scheduleTables[1].entries);

    scheduleEntries[1].componentId=(xme_core_component_t)11;
    scheduleEntries[1].functionId=(xme_core_component_functionId_t)4;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&(scheduleTables[1]), &(scheduleEntries[1])));
    xme_core_exec_scheduler_registerSchedule(&(scheduleTables[1]), &(scheduleIds[1]));

    xme_core_exec_scheduler_activateSchedule(scheduleIds[0]);

    next_schedule_entry = xme_core_exec_scheduler_getNextScheduleHandle();
    EXPECT_EQ(scheduleIds[0], next_schedule_entry);

    xme_core_exec_scheduler_activateSchedule(scheduleIds[1]);

    next_schedule_entry = xme_core_exec_scheduler_getNextScheduleHandle();
    EXPECT_EQ(scheduleIds[1], next_schedule_entry);
}

