
#include <gtest/gtest.h>
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"

#include "xme/core/broker/include/broker.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"

#include "xme/hal/include/sync.h"
#include "xme/hal/include/tls.h"
#include "xme/hal/include/context.h"
#include "xme/hal/include/sched.h"

const uint32_t TEST_FUNCTIONS_IN_SCHEDULE = 3;

static void initStateMockup();
static void finiStateMockup();

static void initStateMockup()
{
	xme_core_exec_state.lock = xme_hal_sync_createCriticalSection();
	xme_core_exec_state.value = XME_CORE_EXEC_PAUSED;
	xme_core_exec_intConfig.useDbCycleCounter = true;
}

static void finiStateMockup()
{
	xme_hal_sync_destroyCriticalSection(xme_core_exec_state.lock);
	xme_core_exec_state.lock = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;
	xme_core_exec_state.value = XME_CORE_EXEC_NOT_INITIALIZED;
}


void CommonSetUp(void);
void CommonTearDown(void);

int main(int argc, char **argv)
{
    int result = -1;
    ::testing::InitGoogleTest(&argc, argv);

    result = RUN_ALL_TESTS();

    return result;
}

void CommonSetUp(void)
{
	xme_core_broker_initStruct_t brokerParams;

	ASSERT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
	xme_hal_tls_init();
	xme_hal_context_init();
	xme_hal_sched_init();

	xme_core_broker_init(&brokerParams);
    xme_core_dataHandler_init();
}

void CommonTearDown(void)
{
    xme_core_dataHandler_fini();
    xme_core_broker_fini();
    xme_hal_sched_fini();
    xme_hal_context_fini();
	xme_hal_tls_fini();
	xme_hal_sync_fini();
}

class smokeTest_ExecutionManager_Scheduler : public ::testing::Test {
 protected:
  virtual void SetUp(void)
  {
	  uint32_t i;

      CommonSetUp();

      xme_core_exec_initDbCycleCounter();

      scheduleTable.majorCycleDuration_ns = 10000;
      XME_HAL_SINGLYLINKEDLIST_INIT(scheduleTable.entries);

      /* Initialize fake function slots */
      for (i=0; i<TEST_FUNCTIONS_IN_SCHEDULE; i++)
      {
    	  scheduleEntries[i].componentId = (xme_core_component_t)(i+1);
    	  scheduleEntries[i].functionId = (xme_core_component_functionId_t)(i+1);
    	  scheduleEntries[i].slotStart_ns = (xme_hal_time_timeInterval_t)(i+1);
    	  scheduleEntries[i].periodDivider = 0U;
    	  scheduleEntries[i].periodDividerOffset = 0U;

    	  EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_singlyLinkedList_addItem(&(scheduleTable.entries), &scheduleEntries[i]));
      }

      /* Fake-init EM */
      initStateMockup();

      ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_init());
      ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_registerSchedule(&scheduleTable, &scheduleId));
      xme_core_exec_scheduler_activateSchedule(scheduleId);
      ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_initExecution());
  }

  virtual void TearDown(void)
  {
	  ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_fini());

	  /* Fake-deinit EM */
	  finiStateMockup();

	  XME_HAL_SINGLYLINKEDLIST_FINI(scheduleTable.entries);

      CommonTearDown();
  }

/*****************************************************************************/
    xme_core_exec_schedule_table_t scheduleTable;
    xme_core_exec_schedule_table_entry_t scheduleEntries[TEST_FUNCTIONS_IN_SCHEDULE];
    xme_core_exec_schedule_handle_t scheduleId;
};

TEST_F(smokeTest_ExecutionManager_Scheduler, trivial)
{
    EXPECT_EQ(true, true);
}

TEST_F(smokeTest_ExecutionManager_Scheduler, calculateNextComponent__threeFunctionsInSchedule_performOneStep__component_and_function_ids_are_valid)
{
    xme_core_exec_schedule_table_entry_t* entry;
    xme_hal_time_timeInterval_t    startDelay_ns;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_calculateNextComponent(&entry, &startDelay_ns));

    EXPECT_EQ(1U, entry->componentId);
    EXPECT_EQ(1, entry->functionId);
}

TEST_F(smokeTest_ExecutionManager_Scheduler, calculateNextComponent__threeFunctionsInSchedule_performFourSteps__component_and_function_ids_are_valid)
{
    xme_core_exec_schedule_table_entry_t* entry;
    xme_hal_time_timeInterval_t    startDelay_ns;

    for(uint32_t i=0; i<TEST_FUNCTIONS_IN_SCHEDULE; i++)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_calculateNextComponent(&entry, &startDelay_ns));

        EXPECT_EQ((xme_core_component_t) (i+1), entry->componentId);
        EXPECT_EQ((xme_core_component_functionId_t) (i+1), entry->functionId);
    }

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_calculateNextComponent(&entry, &startDelay_ns));

	EXPECT_EQ((xme_core_component_t) (1), entry->componentId);
	EXPECT_EQ((xme_core_component_functionId_t) (1), entry->functionId);
}

TEST_F(smokeTest_ExecutionManager_Scheduler, calculateNextComponent__threeFunctionsInSchedule_performFourSteps__cycleCounterIncrementsOnStartOfCycle)
{
	xme_core_exec_schedule_table_entry_t* entry;
	xme_hal_time_timeInterval_t    startDelay_ns;
	uint32_t cycleCounterValue;

	/* before the first cycle cycleCounter is 0 */
	cycleCounterValue = xme_core_exec_scheduler_getCycleCounter();
	EXPECT_EQ(0U, cycleCounterValue);

	/* within the first cycle cycleCounter is 1 */
	for(uint32_t i=0; i<TEST_FUNCTIONS_IN_SCHEDULE; i++)
	{
		EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_calculateNextComponent(&entry, &startDelay_ns));
		cycleCounterValue = xme_core_exec_scheduler_getCycleCounter();
		EXPECT_EQ(1U, cycleCounterValue);
	}

	/* after we enter the second cycle, cycleCounter is set to 2 */
	EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_calculateNextComponent(&entry, &startDelay_ns));
	cycleCounterValue = xme_core_exec_scheduler_getCycleCounter();
	EXPECT_EQ(2U, cycleCounterValue);
}

#define TEST_NSTEPS (uint32_t)9

TEST_F(smokeTest_ExecutionManager_Scheduler, calculateNextComponent__three_functions_in_schedule_with_various_period_dividers__execution_sequence_correct)
{
	uint32_t index;

	xme_core_exec_schedule_table_entry_t* entry;
    xme_hal_time_timeInterval_t    startDelay_ns;

    for (index = 0; index < TEST_FUNCTIONS_IN_SCHEDULE; index++)
	{
		scheduleEntries[index].periodDivider = index + 2;
	}

    uint32_t componentIdSequence[TEST_NSTEPS] = {
    											 /*C0*/ 1, 2, 3,
    											 /*C3*/1,
    											 /*C4*/2,
    											 /*C5*/1,3,
    											 /*C7*/1,2
    											};
    for(index = 0; index < TEST_NSTEPS; index++)
    {
		ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_calculateNextComponent(&entry, &startDelay_ns));

		ASSERT_EQ((xme_core_component_t)(componentIdSequence[index]), entry->componentId)<<"index="<<index;
		ASSERT_EQ((xme_core_component_functionId_t) (componentIdSequence[index]), entry->functionId)<<"index="<<index;
    }
}

TEST_F(smokeTest_ExecutionManager_Scheduler, calculateNextComponent__three_functions_in_schedule_with_various_period_dividers__cycle_counter_sequence_correct)
{
	uint32_t index;
	uint32_t cycleCounterValue;

	xme_core_exec_schedule_table_entry_t* entry;
    xme_hal_time_timeInterval_t    startDelay_ns;

    for (index = 0; index < TEST_FUNCTIONS_IN_SCHEDULE; index++)
	{
		scheduleEntries[index].periodDivider = index + 2;
	}

    uint32_t cycleCounterSequence[TEST_NSTEPS] = {1, 1, 1, 3, 4, 5, 5, 7, 7};
    for(index = 0; index < TEST_NSTEPS; index++)
    {
		ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_calculateNextComponent(&entry, &startDelay_ns));

		cycleCounterValue = xme_core_exec_scheduler_getCycleCounter();
		ASSERT_EQ(cycleCounterSequence[index], cycleCounterValue)<<"index="<<index;
    }
}

#undef TEST_NSTEPS
#define TEST_NSTEPS (uint32_t)7
TEST_F(smokeTest_ExecutionManager_Scheduler, calculateNextComponent__three_functions_in_schedule_with_various_period_divider_offsets__execution_sequence_correct)
{
	uint32_t index;

	xme_core_exec_schedule_table_entry_t* entry;
    xme_hal_time_timeInterval_t    startDelay_ns;

    for (index = 0; index < TEST_FUNCTIONS_IN_SCHEDULE; index++)
	{
    	scheduleEntries[index].periodDivider = 3;
		scheduleEntries[index].periodDividerOffset = index;
	}

    uint32_t componentIdSequence[TEST_NSTEPS] = {
    											 1, 2, 3,
    											 1, 2, 3,
    											 1
    											};

    for(index = 0; index < TEST_NSTEPS; index++)
    {
		ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_calculateNextComponent(&entry, &startDelay_ns));

		ASSERT_EQ((xme_core_component_t)(componentIdSequence[index]), entry->componentId)<<"index="<<index;
		ASSERT_EQ((xme_core_component_functionId_t) (componentIdSequence[index]), entry->functionId)<<"index="<<index;

    }
}

TEST_F(smokeTest_ExecutionManager_Scheduler, calculateNextComponent__three_functions_in_schedule_with_various_period_divider_offsets__cycle_counter_sequence_correct)
{
	uint32_t index;
	uint32_t cycleCounterValue;

	xme_core_exec_schedule_table_entry_t* entry;
    xme_hal_time_timeInterval_t    startDelay_ns;

    for (index = 0; index < TEST_FUNCTIONS_IN_SCHEDULE; index++)
	{
    	scheduleEntries[index].periodDivider = 3;
		scheduleEntries[index].periodDividerOffset = index;
	}

    uint32_t cycleCounterSequence[TEST_NSTEPS] = {1, 2, 3, 4, 5, 6, 7};
    for(index = 0; index < TEST_NSTEPS; index++)
    {
		ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_scheduler_calculateNextComponent(&entry, &startDelay_ns));

		cycleCounterValue = xme_core_exec_scheduler_getCycleCounter();
		ASSERT_EQ(cycleCounterSequence[index], cycleCounterValue)<<"index="<<index;
    }
}
