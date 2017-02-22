/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: scheduler.c 7696 2014-03-06 16:43:58Z rupanov $
 */

/**
 * \file
 *         Scheduler.
 */

#define MODULE_ACRONYM "ExecMgr   : "

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"

#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"

#include "xme/core/topicData.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/time.h"

#include <inttypes.h>


/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_core_exec_scheduler_internalStateStruct_t
 *
 * \brief  Defines the internal state of scheduler.
 */
typedef struct xme_core_exec_scheduler_internalState_
{
    xme_hal_sync_criticalSectionHandle_t readyQueueMutex;   ///< Mutex for shared access to the task list.
    xme_hal_linkedList_descriptor_t* nodeSchedules;         ///< All the schedules of local execution manager
    xme_hal_time_timeHandle_t startOfCycle_ns;              ///< Start of the current scheduling cycle
    xme_hal_time_timeHandle_t measuredStartOfCycle_ns;      ///< Start of the current scheduling cycle
    xme_core_exec_schedule_handle_t currentScheduleHandle;  ///< Handle to the current schedule
    xme_core_exec_schedule_table_t* currentSchedule;        ///< A local copy of the current schedule
    xme_core_exec_schedule_handle_t nextScheduleHandle;     ///< Handle to the schedule to be used starting from the next cycle
    xme_core_exec_schedule_taskHandle_t nextTaskHandle;         ///< Sequential number of the next task to be selected for execution
    xme_core_exec_schedule_table_entry_t* nextTask;         ///< Next task to be executed
//    uint64_t                      cycleCounter;
} xme_core_exec_scheduler_internalState_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/** \var xme_core_exec_scheduler_internalState
 *
 * \brief Internal state-structure of the scheduler
 */
static xme_core_exec_scheduler_internalState_t xme_core_exec_scheduler_internalState;

static bool componentInitialized = false;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/** \brief  Scheduler finalization. This function is typically not called
 *           directly, but is executed as a part of xme_core_exec_fini().
 */
extern xme_status_t
xme_core_exec_scheduler_fini( void );


xme_status_t
xme_core_exec_scheduler_incrementCycleCounter(void);

/**
 * \brief Prepare for cyclic execution
 * \returns XME status
 */
extern xme_status_t
xme_core_exec_scheduler_initExecution(void);


static void
calculateNewStartOfCycle(void);

static void
selectNewSchedule(void);

static xme_status_t
updateLocalScheduleCopy(void);

static xme_status_t
getNextTableEntry(xme_core_exec_schedule_table_entry_t** tableEntry);

static xme_status_t
invalidateTableEntryBasedOnPeriodOffset(xme_core_exec_schedule_table_entry_t** tableEntry);

static xme_hal_time_timeInterval_t
getStartDelay(xme_hal_time_timeHandle_t timeNow, xme_hal_time_timeHandle_t startTime);

static bool
xme_core_exec_scheduler_is_initialized(void);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_exec_scheduler_init(void)
{
    xme_core_exec_scheduler_internalState.readyQueueMutex = xme_hal_sync_createCriticalSection();
    xme_core_exec_scheduler_internalState.currentScheduleHandle = XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID;
    xme_core_exec_scheduler_internalState.nextScheduleHandle = XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID;
    xme_core_exec_scheduler_internalState.nextTaskHandle = XME_CORE_EXEC_SCHEDULE_TASKHANDLE_INVALID;

    xme_core_exec_scheduler_internalState.nodeSchedules = (xme_hal_linkedList_descriptor_t*) xme_hal_mem_alloc(sizeof(xme_hal_linkedList_descriptor_t));

    XME_HAL_SINGLYLINKEDLIST_INIT(*(xme_core_exec_scheduler_internalState.nodeSchedules));
    componentInitialized = true;

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_fini(void)
{
    componentInitialized = false;
    /* freeing all schedule tables is not the task of the scheduler
     * -- someone manages those externally */
    XME_HAL_SINGLYLINKEDLIST_FINI(*(xme_core_exec_scheduler_internalState.nodeSchedules));

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_sync_destroyCriticalSection(xme_core_exec_scheduler_internalState.readyQueueMutex),
        XME_STATUS_INTERNAL_ERROR);
    xme_core_exec_scheduler_internalState.currentScheduleHandle = XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID;
    xme_core_exec_scheduler_internalState.nextScheduleHandle = XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID;
    xme_core_exec_scheduler_internalState.nextTaskHandle = XME_CORE_EXEC_SCHEDULE_TASKHANDLE_INVALID;

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_getScheduleSetPointer
(
    xme_hal_linkedList_descriptor_t** target
)
{

    *target = xme_core_exec_scheduler_internalState.nodeSchedules;
    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_addFunctionToSchedule
(
    xme_core_exec_schedule_handle_t scheduleId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArgs,
    uint32_t periodDivider,
	uint32_t periodDividerOffset
)
{
    xme_core_exec_schedule_table_t* schedule = NULL;

    /* Get the schedule structure */
    XME_CHECK(
        XME_STATUS_SUCCESS == xme_core_exec_scheduler_getSchedule(scheduleId, &schedule),
        XME_STATUS_NOT_FOUND);

    /* Now the new function in use: in future NO changes on active schedule set */
    return xme_core_exec_scheduler_autoAllocateInScheduleTable(
                schedule,
                componentId,
                functionId,
                functionArgs,
                periodDivider,
            	periodDividerOffset
            );
}

/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_registerSchedule
(
     xme_core_exec_schedule_table_t* schedule,      ///<[in]
     xme_core_exec_schedule_handle_t* scheduleId    ///<[out]
)
{
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(xme_core_exec_scheduler_is_initialized()));

    if(XME_STATUS_SUCCESS != xme_hal_singlyLinkedList_addItem(
                    (void*)(xme_core_exec_scheduler_internalState.nodeSchedules),
                    (void*)schedule))
        return XME_STATUS_OUT_OF_RESOURCES;

    *scheduleId = (xme_core_exec_schedule_handle_t)
                    (xme_hal_singlyLinkedList_getItemCount(xme_core_exec_scheduler_internalState.nodeSchedules) - 1UL);

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_getSchedule
(
     xme_core_exec_schedule_handle_t scheduleId,
     xme_core_exec_schedule_table_t** schedule
)
{
    /* We only deal with an initialized component */
    if (!xme_core_exec_isInitialized())
        return XME_STATUS_UNEXPECTED;

    *schedule = (xme_core_exec_schedule_table_t*) XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(
                *(xme_core_exec_scheduler_internalState.nodeSchedules),
                (int)scheduleId);
    XME_CHECK(NULL != *schedule, XME_STATUS_NOT_FOUND);
    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_core_exec_schedule_handle_t
xme_core_exec_scheduler_getCurrentScheduleHandle( void )
{
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_core_exec_scheduler_is_initialized()), XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID);

    return xme_core_exec_scheduler_internalState.currentScheduleHandle;
}

/****************************************************************************/
// XXX change the signature NOT to return xme_status_t
xme_core_exec_schedule_handle_t
xme_core_exec_scheduler_getNextScheduleHandle( void )
{
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_core_exec_scheduler_is_initialized()), XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID);

    return xme_core_exec_scheduler_internalState.nextScheduleHandle;
}

/****************************************************************************/
void
xme_core_exec_scheduler_activateSchedule
(
    xme_core_exec_schedule_handle_t scheduleId
)
{
    XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_core_exec_scheduler_is_initialized()));
    XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(xme_hal_singlyLinkedList_getItemCount(xme_core_exec_scheduler_internalState.nodeSchedules) > (xme_maxSystemValue_t)scheduleId));

    if(xme_core_exec_scheduler_internalState.nextScheduleHandle==XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID)
    {
        /* If the next schedule ID is not defined, current schedule is also set to scheduleId */
        xme_core_exec_scheduler_internalState.currentScheduleHandle = scheduleId;
        xme_core_exec_scheduler_internalState.nextTaskHandle = XME_CORE_EXEC_SCHEDULE_TASKHANDLE_DEFAULT;
        xme_core_exec_scheduler_internalState.startOfCycle_ns = xme_hal_time_getCurrentTime();
    }

    xme_core_exec_scheduler_internalState.nextScheduleHandle = scheduleId;

    return;
}

/****************************************************************************/

xme_status_t
xme_core_exec_scheduler_initExecution( void )
{
    xme_hal_linkedList_index_t itemCount;

    /* We only deal with an initialized component */
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(xme_core_exec_isInitialized()));

    if(xme_core_exec_scheduler_internalState.currentScheduleHandle==XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID)
    {
        XME_CHECK(0 < xme_hal_singlyLinkedList_getItemCount(xme_core_exec_scheduler_internalState.nodeSchedules),
            XME_STATUS_INVALID_CONFIGURATION);

        xme_core_exec_scheduler_activateSchedule((xme_core_exec_schedule_handle_t)0);
    }
    xme_core_exec_scheduler_internalState.startOfCycle_ns = xme_hal_time_getCurrentTime();
    //xme_core_exec_scheduler_internalState.cycleCounter = 0;
    xme_core_exec_scheduler_internalState.currentSchedule = (xme_core_exec_schedule_table_t*)xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_table_t));
    XME_HAL_SINGLYLINKEDLIST_INIT((xme_core_exec_scheduler_internalState.currentSchedule->entries));

    XME_CHECK( XME_STATUS_SUCCESS == updateLocalScheduleCopy(), XME_STATUS_INTERNAL_ERROR );

    itemCount = xme_hal_singlyLinkedList_getItemCount(&(xme_core_exec_scheduler_internalState.currentSchedule->entries));
    xme_core_exec_scheduler_internalState.nextTaskHandle = (xme_core_exec_schedule_taskHandle_t)itemCount;

    /* TODO check return */
    xme_core_exec_resetDbCycleCounter();

    return XME_STATUS_SUCCESS;
}

/*
 * todo break it into reasonably small parts
 */

/****************************************************************************/

xme_status_t
xme_core_exec_scheduler_calculateNextComponent
(
    xme_core_exec_schedule_table_entry_t** entry,
    xme_hal_time_timeInterval_t*    startDelay_ns
)
{
    xme_status_t status;
    xme_core_exec_schedule_table_entry_t* tableEntry = NULL;
    xme_hal_time_timeHandle_t startTime;
    xme_hal_time_timeHandle_t timeNow;
    xme_hal_linkedList_index_t numTasksInSchedule;

    /* Can only be called for an initialized component */
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(xme_core_exec_isInitialized()));

    *startDelay_ns = 0;

    /* Check for schedule ID and task validity */
    XME_ASSERT( (XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID != xme_core_exec_scheduler_internalState.currentScheduleHandle)
      || (XME_CORE_EXEC_SCHEDULE_TASKHANDLE_INVALID != xme_core_exec_scheduler_internalState.nextTaskHandle));

	/* XXX: Now that we have some tasks as inactive, we have to iterate through the table till we find the new one...
	 * This could lead to endless loops if used incorrectly.
	 */
    do
    {
    	numTasksInSchedule = xme_hal_singlyLinkedList_getItemCount(&(xme_core_exec_scheduler_internalState.currentSchedule->entries));

    	// fixme: this is a too complex condition, move to a separate function
		if( ((numTasksInSchedule > 1) && ((int32_t) xme_core_exec_scheduler_internalState.nextTaskHandle == (int32_t) (numTasksInSchedule - 1)))
				|| (numTasksInSchedule == 1) )
		{
			if(XME_CORE_EXEC_TO_BE_SHUT_DOWN == xme_core_exec_getState())
				xme_core_exec_setState(XME_CORE_EXEC_SHUTDOWN);
		}

		if(xme_core_exec_scheduler_internalState.nextTaskHandle == ((xme_core_exec_schedule_taskHandle_t)(numTasksInSchedule)))
		{
			/* Wrap the next handle */
			xme_core_exec_scheduler_internalState.nextTaskHandle = (xme_core_exec_schedule_taskHandle_t) 0x0;

			XME_CHECK(XME_STATUS_SUCCESS ==  xme_core_exec_scheduler_incrementCycleCounter(),
										  XME_STATUS_INTERNAL_ERROR);

			/* We are at the end of previous cycle */
			if(NULL != xme_core_exec_intConfig.onEndOfCycle)
			{
				XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_intConfig.onEndOfCycle(NULL),
						  XME_STATUS_INTERNAL_ERROR);
			}

			calculateNewStartOfCycle();

			/* Is schedule changed? */
			selectNewSchedule();

			status = updateLocalScheduleCopy();
			XME_CHECK(XME_STATUS_SUCCESS == status,
					  status);
		}

		status = getNextTableEntry(&tableEntry);
		XME_CHECK(XME_STATUS_SUCCESS == status,
						status);

		/* Validate the next table entry wrt periodicity and offset */
		status = invalidateTableEntryBasedOnPeriodOffset(&tableEntry);
		XME_CHECK(XME_STATUS_SUCCESS == status,
						status);
    }
    while((NULL == tableEntry)&&(XME_CORE_EXEC_SHUTDOWN != xme_core_exec_getState()));

    timeNow = xme_hal_time_getCurrentTime();
    startTime = xme_hal_time_offsetTime(
        xme_core_exec_scheduler_internalState.startOfCycle_ns,
        XME_HAL_TIME_OFFSET_OPERATION_ADD,
        tableEntry->slotStart_ns);

    *entry = tableEntry;
    *startDelay_ns = getStartDelay(timeNow, startTime);

    return XME_STATUS_SUCCESS;
}

static void
calculateNewStartOfCycle(void)
{
    xme_hal_time_timeHandle_t currentTime;
    uint64_t cycle_time=0U, cycle_offset=0U;
    int64_t slack=0, globalSlack=0;

    /* Update startOfCycle based on old "current schedule":
          new startOfCycle is an offset of previous by majorCycleDuration */
    currentTime = xme_hal_time_getCurrentTime();
    cycle_time = xme_hal_time_getTimeIntervalBetween(xme_core_exec_scheduler_internalState.measuredStartOfCycle_ns, currentTime);
    cycle_offset = xme_hal_time_getTimeIntervalBetween(xme_core_exec_scheduler_internalState.startOfCycle_ns, currentTime);
    xme_core_exec_scheduler_internalState.measuredStartOfCycle_ns = currentTime;

    xme_core_exec_scheduler_internalState.startOfCycle_ns =
               xme_hal_time_offsetTime(
                   xme_core_exec_scheduler_internalState.startOfCycle_ns,
                   XME_HAL_TIME_OFFSET_OPERATION_ADD,
                   xme_core_exec_scheduler_internalState.currentSchedule->majorCycleDuration_ns);

    // XXX Code Review: remove this when CC=100%, smells
    slack = ( -1 * (int64_t)cycle_time  + (int64_t)xme_core_exec_scheduler_internalState.currentSchedule->majorCycleDuration_ns)/1000;
    globalSlack =  ( -1 * (int64_t)cycle_offset  + (int64_t)xme_core_exec_scheduler_internalState.currentSchedule->majorCycleDuration_ns)/1000;

    XME_LOG(XME_LOG_DEBUG,
         MODULE_ACRONYM"-- cycle length: %c %7"PRIu64"us, slack %7"PRId64"us, globalSlack%7"PRId64"us\n",
         (slack>=0)?' ':'!',
         cycle_time/1000,
         slack,
         globalSlack);
}

static void
selectNewSchedule(void)
{
    if (xme_core_exec_scheduler_internalState.nextScheduleHandle !=
                   xme_core_exec_scheduler_internalState.currentScheduleHandle)
    {
        XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "Switching to next schedule: %d\n",
            xme_core_exec_scheduler_internalState.nextScheduleHandle);
        xme_core_exec_scheduler_internalState.currentScheduleHandle =
            xme_core_exec_scheduler_internalState.nextScheduleHandle;
        XME_LOG(XME_LOG_VERBOSE,
            MODULE_ACRONYM "Schedule change performed\n");
    }
}

/**
 * \brief This function is executed at the beginning of a cycyle to make sure plug and play does
 *          not interfere with the schedule
 */
static xme_status_t
updateLocalScheduleCopy(void)
{
    xme_core_exec_schedule_table_t* newSchedule = NULL;

    XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_scheduler_getSchedule(xme_core_exec_scheduler_internalState.currentScheduleHandle, &newSchedule),
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_ERROR,
        MODULE_ACRONYM "non-existing schedule ID specified as next: %d!",
        xme_core_exec_scheduler_internalState.currentScheduleHandle);

    XME_CHECK(NULL != newSchedule,
              XME_STATUS_NOT_FOUND);

    XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_scheduler_clearScheduleTable(&(xme_core_exec_scheduler_internalState.currentSchedule), (bool)true),
        XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            MODULE_ACRONYM "could not clear the local schedule replica: %d!",
            xme_core_exec_scheduler_internalState.currentScheduleHandle);

    XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_scheduler_replicateSchedule(&(xme_core_exec_scheduler_internalState.currentSchedule), newSchedule),
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_ERROR,
        MODULE_ACRONYM "error replicating the schedule locally: %d!",
        xme_core_exec_scheduler_internalState.currentScheduleHandle);

    return XME_STATUS_SUCCESS;
}

static xme_status_t
getNextTableEntry(xme_core_exec_schedule_table_entry_t** tableEntry)
{
	uint32_t nEntries = (uint32_t) xme_hal_singlyLinkedList_getItemCount(
			&(xme_core_exec_scheduler_internalState.currentSchedule->entries));

	XME_ASSERT(	0 != nEntries );

	*tableEntry =
        (xme_core_exec_schedule_table_entry_t*)
            XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(
                xme_core_exec_scheduler_internalState.currentSchedule->entries,
                (int)xme_core_exec_scheduler_internalState.nextTaskHandle);

    XME_ASSERT(NULL != *tableEntry);

    xme_core_exec_scheduler_internalState.nextTaskHandle++;

    return XME_STATUS_SUCCESS;
}

static xme_status_t
invalidateTableEntryBasedOnPeriodOffset(xme_core_exec_schedule_table_entry_t** tableEntry)
{
	uint32_t periodDivider;
	uint32_t periodDividerOffset;
	uint32_t cycleCounter;

	if(0 == (*tableEntry)->periodDivider)
			return XME_STATUS_SUCCESS;

	periodDivider = (*tableEntry)->periodDivider;
	periodDividerOffset = (*tableEntry)->periodDividerOffset;
	cycleCounter = xme_core_exec_scheduler_getCycleCounter();

	/* if the divider could not be reached by design */
	if( periodDividerOffset > cycleCounter )
	{
		*tableEntry = NULL;
		return XME_STATUS_SUCCESS;
	}

	/* non-negative, as (periodDivider >= 1) && (cycleCounter >= periodDividerOffset) */
	if( 0U != ((cycleCounter+periodDivider-periodDividerOffset-1) % periodDivider) )
	{
		*tableEntry = NULL;
	}
	return XME_STATUS_SUCCESS;
}

static xme_hal_time_timeInterval_t
getStartDelay(xme_hal_time_timeHandle_t timeNow, xme_hal_time_timeHandle_t startTime)
{
    if(xme_hal_time_compareTime(timeNow, startTime) < 0)
    {
      return xme_hal_time_getTimeIntervalBetween(timeNow, startTime);
    }
    /* else */
    return 0;
}


static bool
xme_core_exec_scheduler_is_initialized(void)
{
    return componentInitialized;
}


