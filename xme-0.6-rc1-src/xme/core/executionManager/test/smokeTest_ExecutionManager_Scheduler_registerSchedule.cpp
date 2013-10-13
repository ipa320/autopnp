
#include <gtest/gtest.h>
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include <stdio.h>

const int NUM_TEST_TABLES = 10;
const int NUM_TEST_ENTRIES = 10;

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


int main(int argc, char **argv)
{
    int result = -1;
    ::testing::InitGoogleTest(&argc, argv);

    result = RUN_ALL_TESTS();
    return result;
}

class smokeTest_ExecutionManager_Scheduler_registerSchedule : public ::testing::Test {
 protected:

    virtual void SetUp(void)
    {
    	ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
		ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_init());

	    /* Fake-init EM */
	    initStateMockup();
    }

    virtual void TearDown(void)
    {
    	/* Fake-init EM */
		finiStateMockup();
    	xme_core_exec_scheduler_fini();
    	xme_hal_sync_fini();
    }



    xme_core_exec_schedule_table_t scheduleTables[NUM_TEST_TABLES];
    xme_core_exec_schedule_table_entry_t scheduleEntries[NUM_TEST_ENTRIES];
    xme_core_exec_schedule_handle_t scheduleIds[NUM_TEST_TABLES];
};

TEST_F(smokeTest_ExecutionManager_Scheduler_registerSchedule, schedule_list_is_empty__1st_schedule_has_id_0)
{
    scheduleTables[0].majorCycleDuration_ns = 10000;
    XME_HAL_SINGLYLINKEDLIST_INIT(scheduleTables[0].entries);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(&(scheduleTables[0]), &(scheduleIds[0])));
    EXPECT_EQ(scheduleIds[0],(xme_core_exec_schedule_handle_t)0);
}

TEST_F(smokeTest_ExecutionManager_Scheduler_registerSchedule, schedule_list_is_not_empty___new_schedule_has_valid_id)
{
	scheduleTables[0].majorCycleDuration_ns = 10000;
	XME_HAL_SINGLYLINKEDLIST_INIT(scheduleTables[0].entries);

	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(&(scheduleTables[0]), &(scheduleIds[0])));
	EXPECT_EQ(scheduleIds[0],(xme_core_exec_schedule_handle_t)0);

    scheduleTables[1].majorCycleDuration_ns = 10000;
    XME_HAL_SINGLYLINKEDLIST_INIT(scheduleTables[1].entries);

    scheduleEntries[1].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT;
    scheduleEntries[1].functionId = XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&(scheduleTables[1].entries), &scheduleEntries[1]));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(&(scheduleTables[1]), &(scheduleIds[1])));
    EXPECT_EQ(scheduleIds[1],(xme_core_exec_schedule_handle_t)1);
}

TEST_F(smokeTest_ExecutionManager_Scheduler_registerSchedule, schedule_list_with_3_elements___schedule_identifiers_unique)
{
	scheduleTables[0].majorCycleDuration_ns = 10000;
	XME_HAL_SINGLYLINKEDLIST_INIT(scheduleTables[0].entries);

	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(&(scheduleTables[0]), &(scheduleIds[0])));
	EXPECT_EQ(scheduleIds[0],(xme_core_exec_schedule_handle_t)0);

	scheduleTables[1].majorCycleDuration_ns = 10000;
	XME_HAL_SINGLYLINKEDLIST_INIT(scheduleTables[1].entries);

	scheduleEntries[1].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT;
	scheduleEntries[1].functionId = XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT;

	EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&(scheduleTables[1].entries), &scheduleEntries[1]));

	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(&(scheduleTables[1]), &(scheduleIds[1])));
	EXPECT_EQ(scheduleIds[1],(xme_core_exec_schedule_handle_t)1);
    scheduleTables[2].majorCycleDuration_ns = 10000;
    XME_HAL_SINGLYLINKEDLIST_INIT(scheduleTables[2].entries);

    scheduleEntries[2].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT;
    scheduleEntries[2].functionId = XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&(scheduleTables[2].entries), &scheduleEntries[2]));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(&(scheduleTables[2]), &(scheduleIds[2])));
    EXPECT_EQ(scheduleIds[2],(xme_core_exec_schedule_handle_t)2);

    EXPECT_NE(scheduleIds[0],scheduleIds[1]);
    EXPECT_NE(scheduleIds[0],scheduleIds[2]);
    EXPECT_NE(scheduleIds[1],scheduleIds[2]);
}

#if 0
static bool outOfMemory = false;
void* malloc(size_t size)
{
    if(outOfMemory)
    {
        return NULL;
    }
    return local_malloc(size);
}

#include "malloc.h"
void* local_malloc(size_t size)
{
    return malloc(size);
}
#endif
