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
 * $Id: tdmaScheduler.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Time Division Multiple Access (TDMA) scheduler: implementation
 */


/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/prim/tdmaScheduler.h"

#include "xme/core/log.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/sleep.h"
#include "xme/hal/include/linkedList.h"

#include <stdbool.h>


/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \brief  List containing all tasks for one TDMA slot.
 */
typedef xme_hal_singlyLinkedList_t(XME_PRIM_TDMA_MAX_TASK_COUNT_IN_SLOT) xme_tdma_tasksList_t;


/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/


/**
 * \brief  Static array of xme_tdma_tasks_list_t.
 *         Stores the tasks for each slot.
 *         (the slot number is the index into the array)
 */
static xme_tdma_tasksList_t* tdma_tasks[XME_PRIM_TDMA_MAX_SLOT_COUNT];


/**
 * \brief  Static version of the xme_comp_tdma_configStruct_t.
 *         For TDMA functions to which the xme_comp_tdma_configStruct_t
 *         cannot be passed as a parameter.
 */
static xme_prim_tdma_configStruct_t* xme_prim_tdma_config = NULL;


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/


static void
xme_tdma_control_task(void* data);


static xme_status_t
xme_hal_sched_setTaskExecutionState_tdma
(
	xme_hal_sched_taskHandle_t taskHandle,
	bool running
);


static xme_hal_time_timeInterval_t
xme_tdma_delay_until
(
	xme_hal_time_timeHandle_t target_time,
	bool time_overflow
);


static xme_status_t
xme_tdma_resume_tasks
(
	xme_prim_tdma_configStruct_t* config,
	uint8_t slot_number
);


static xme_status_t
xme_tdma_suspend_tasks
(
	xme_prim_tdma_configStruct_t* config,
	uint8_t slot_number
);

static xme_status_t
xme_tdma_check_task_preconditions
(
	xme_hal_time_timeInterval_t startTime,
	xme_hal_time_timeInterval_t period,
	uint8_t priority,
	xme_hal_sched_taskCallback_t callback,
	uint8_t slot,
	void* userData
);

static xme_status_t
xme_tdma_register_task
(
	xme_hal_sched_taskHandle_t handle,
	uint8_t slot
);

static xme_status_t
xme_tdma_remove_task
(
	xme_hal_sched_taskHandle_t handle
);


/********************************************************************************************/
/***   xme_hal_sched and xme_core_resourcemanager function implementations for TDMA       ***/
/********************************************************************************************/


xme_core_resourceManager_taskHandle_t
xme_core_resourceManager_scheduleTask_tdma
(
	xme_hal_time_timeInterval_t startTime,
	xme_hal_time_timeInterval_t period,
	uint8_t priority,
	xme_hal_sched_taskCallback_t callback,
	uint8_t slot,
	void* userData
)
{
	// TODO: actually call xme_core_resourceManager
	/* redirect to hal_sched TDMA implementation */
	return xme_hal_sched_addTask_tdma(startTime, period, priority, callback, slot, userData);

	xme_status_t status;

	status = xme_tdma_check_task_preconditions(startTime, period, priority, callback, slot, userData);

	XME_CHECK(XME_STATUS_SUCCESS == status, XME_HAL_SCHED_INVALID_TASK_HANDLE);

	/* redirect to native scheduler implementation of addTask TDMA implementation */
	xme_core_resourceManager_taskHandle_t handle = xme_core_resourceManager_scheduleTask(startTime, period, priority, callback, userData);

	status = xme_tdma_register_task( (xme_hal_sched_taskHandle_t)handle, slot);

	XME_CHECK(XME_STATUS_SUCCESS == status, XME_HAL_SCHED_INVALID_TASK_HANDLE);
	return handle;
}


xme_status_t
xme_core_resourceManager_killTask_tdma(xme_core_resourceManager_taskHandle_t taskHandle)
{
	xme_status_t status;
	/* redirect to native implementation of kill task */
	xme_status_t result = xme_core_resourceManager_killTask(taskHandle);
	if(XME_STATUS_SUCCESS == result){
		status = xme_tdma_remove_task(taskHandle);
		XME_CHECK(XME_STATUS_SUCCESS == status, status);
	}
	return result;
}


xme_hal_sched_taskHandle_t
xme_hal_sched_addTask_tdma
(
	xme_hal_time_timeInterval_t startTime,
	xme_hal_time_timeInterval_t period,
	uint8_t priority,
	xme_hal_sched_taskCallback_t callback,
	uint8_t slot,
	void* userData
)
{
	xme_status_t status;

	status = xme_tdma_check_task_preconditions(startTime, period, priority, callback, slot, userData);

	XME_CHECK(XME_STATUS_SUCCESS == status, XME_HAL_SCHED_INVALID_TASK_HANDLE);

	/* redirect to native scheduler implementation of addTask TDMA implementation */
	xme_hal_sched_taskHandle_t handle = xme_hal_sched_addTask(startTime, period, priority, callback, userData);

	status = xme_tdma_register_task(handle, slot);

	XME_CHECK(XME_STATUS_SUCCESS == status, XME_HAL_SCHED_INVALID_TASK_HANDLE);
	return handle;
}


xme_status_t
xme_hal_sched_removeTask_tdma(xme_hal_sched_taskHandle_t taskHandle)
{
	xme_status_t status;
	/* redirect to native implementation of remove task */
	xme_status_t result = xme_hal_sched_removeTask(taskHandle);
	if(XME_STATUS_SUCCESS == result){
		status = xme_tdma_remove_task(taskHandle);
		XME_CHECK(XME_STATUS_SUCCESS == status, status);
	}
	return result;
}


xme_status_t
xme_hal_sched_setTaskExecutionState_tdma
(
	xme_hal_sched_taskHandle_t taskHandle,
	bool running
)
{
	/* TODO: do not override the hal_sched execution state, but suspend the task, iff TDMA_suspend == true AND hal_sched_suspend == true
	/ this implementation overwrites the suspend or resume state of the 'XME user application' */
	return xme_hal_sched_setTaskExecutionState(taskHandle, running);
}


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_tdma_check_task_preconditions
(
	xme_hal_time_timeInterval_t startTime,
	xme_hal_time_timeInterval_t period,
	uint8_t priority,
	xme_hal_sched_taskCallback_t callback,
	uint8_t slot,
	void* userData
)
{
	if(NULL != xme_prim_tdma_config)
	{
		if(priority >= xme_prim_tdma_config->tdma_priority)
		{
			/* the tdma control task must have the highest priority in the system,
			 * otherwise it cannot interrupt the other tasks and the slot change might be delayed
			 */
			XME_LOG(XME_LOG_FATAL, "xme_tdma: adding task with priority >= XME_TDMA_CONTROL_TASK_PRIORITY.\n");
			return XME_STATUS_INVALID_CONFIGURATION;
		}
	}
	if(NULL == tdma_tasks[slot])
	{
		/* addTask_tdma can be called before the xme_tdma component has been initialized
		 * If this is the case, we have to initialize the task list on our own.
		 */
		XME_LOG(XME_LOG_NOTE ,"task list for TDMA slot %i not initialized! initializing before starting the tdma scheduler \n", slot);
		tdma_tasks[slot] = xme_hal_mem_alloc(sizeof(xme_tdma_tasksList_t));
		XME_CHECK(NULL != tdma_tasks[slot], XME_STATUS_OUT_OF_RESOURCES);
		XME_HAL_SINGLYLINKEDLIST_INIT(*(tdma_tasks[slot]));
	}
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_tdma_register_task
(
	xme_hal_sched_taskHandle_t handle,
	uint8_t slot
)
{
	xme_status_t status;
	if (XME_HAL_SCHED_INVALID_TASK_HANDLE != handle)
	{
		/* if the task creation was successful, register the task in its TDMA slot */
		xme_hal_sched_taskHandle_t* handle_heap = (xme_hal_sched_taskHandle_t*) xme_hal_mem_alloc(sizeof(handle));
		XME_CHECK_REC
		(
			NULL != handle_heap,
			XME_STATUS_OUT_OF_RESOURCES,
			{}
		);
		*handle_heap = handle;
		status = xme_hal_singlyLinkedList_addItem((tdma_tasks[slot]), handle_heap);
		return status;
	}
	else
	{
		return XME_STATUS_INVALID_HANDLE;
	}
}

xme_status_t
xme_tdma_remove_task
(
	xme_hal_sched_taskHandle_t handle
)
{
	uint8_t slot;
	/* if the task removal was successful, remove the task from its TDMA slot */
	for(slot=0;slot<xme_prim_tdma_config->num_of_slots; ++slot){
		if(NULL != tdma_tasks[slot]){
			XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*(tdma_tasks[slot]), xme_hal_sched_taskHandle_t, task_item);
			{
				if(*task_item == handle){
					xme_hal_mem_free(task_item);
					xme_hal_singlyLinkedList_removeItem(tdma_tasks[slot], task_item, true);
					return XME_STATUS_SUCCESS;
				}
			 }
			 XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
		}
	}
	return XME_STATUS_NOT_FOUND;
}


/**
 * \brief  Puts the current task into sleep until the specified time.
 *
 * \note   If target_time is in the past, this function will not sleep and
 *         return immediately.
 *
 * \param  target_time The target sleep time. The currently active task
 *         will sleep until (current time == target_time).
 * \param  time_overflow A parameter indicating an overflow of the
 *         target_time.
 * \return The actual time this function was sleeping.
 */
xme_hal_time_timeInterval_t
xme_tdma_delay_until(
	xme_hal_time_timeHandle_t target_time,
	bool time_overflow
)
{
	xme_hal_time_timeHandle_t current = xme_hal_time_getCurrentTime();
	xme_hal_time_timeInterval_t sleep_time;
	if( (!time_overflow) && (0 <= xme_hal_time_compareTime(current, target_time)) )
	{
		/* we are too late, current time is >= target_time */
		sleep_time = xme_hal_time_getTimeIntervalBetween(current, current);
		XME_LOG(XME_LOG_WARNING, "delay until: current %u, target %u, NOT SLEEPING until next TDMA slot \n", current, target_time);
	}
	else
	{
		sleep_time = xme_hal_time_getTimeIntervalBetween(current, target_time);
		if(0 <= sleep_time)
		{
			xme_hal_sleep_sleep(sleep_time);
		}
	}
	return sleep_time;
}


/**
 * \brief  Resumes all tasks in a TDMA slot.
 *
 * \note   If resuming of a task fails, this function will still try
 *         to resume the other tasks.
 *
 * \param  config The config struct of the tdma component.
 * \param  slot_number The number of the slot, whose tasks shall
 *         be resumed. (zero based)
 * \return Returns XME_CORE_STATUS_SUCCESS if all tasks could be resumed.
 */
xme_status_t
xme_tdma_resume_tasks
(
	xme_prim_tdma_configStruct_t* config,
	uint8_t slot_number
)
{
	xme_status_t result = XME_STATUS_SUCCESS;
	xme_status_t status = XME_STATUS_SUCCESS;
	XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*(tdma_tasks[slot_number]), xme_hal_sched_taskHandle_t, task_item);
		status = xme_hal_sched_setTaskExecutionState_tdma(*task_item, true);
		if(XME_STATUS_SUCCESS != status)
		{
			XME_LOG(XME_LOG_VERBOSE, "error resuming task %i! \n", *task_item);
			result = status;
		}
	 XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
	 return result;
}


/**
 * \brief  Suspends all tasks in a TDMA slot.
 *
 * \note   If suspension of a task fails, this function will still try
 *         to suspend the other tasks.
 *
 * \param  config The config struct of the tdma component.
 * \param  slot_number The number of the slot, whose tasks shall
 *         be suspended. (zero based)
 * \return Returns XME_CORE_STATUS_SUCCESS if all tasks could be suspended
 */
xme_status_t
xme_tdma_suspend_tasks
(
	xme_prim_tdma_configStruct_t* config,
	uint8_t slot_number
)
{
	xme_status_t result = XME_STATUS_SUCCESS;
	xme_status_t status = XME_STATUS_SUCCESS;
	XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*(tdma_tasks[slot_number]), xme_hal_sched_taskHandle_t, task_item);
		status = xme_hal_sched_setTaskExecutionState_tdma(*task_item, false);
		if(XME_STATUS_SUCCESS != status)
		{
			XME_LOG(XME_LOG_WARNING, "error suspending task %i! \n", *task_item);
			result = status;
		}
	 XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
	 return result;
}


/* TODO: synchronize access to the TDMA_comp config struct and the tdma_tasks list */
/**
 * \brief  The TDMA control task containing the TDMA logic.
 *
 * \note   The TDMA control task is the highest task in the system.
 *         All XME tasks are registered in one of the TDMA slots.
 *         The tdma control task will enable all tasks in the slot, which
 *         shall be executed at the moment and disable all other tasks.
 *         This guarantees that only the tasks from the current slot are
 *         running.
 *
 * \param  data The config struct of the tdma component.
 */
void
xme_tdma_control_task(void* data)
{
	xme_prim_tdma_configStruct_t* config = (xme_prim_tdma_configStruct_t*) data;

	/* local variables */

	/* the number of the last configured TDMA slot */
	uint8_t last_slot_number = config->num_of_slots - 1;

	/* the actual sleep time, i.e. the actual execution time of all tasks in a slot */
	xme_hal_time_timeInterval_t actual_delay;

	/* a variable counting the number of full scheduler runs (full = all slots once) */
	uint64_t scheduler_run_counter = 0;

	/* the currently executing TDMA slot */
	uint8_t current_slot = last_slot_number;

	/* the TDMA slot executing next */
	uint8_t next_slot = 0;

	/* has an overflow of the xme_hal_time_timeHandle_t occured? */
	bool time_overflow = false;

	uint8_t i = 0;

	xme_hal_time_timeHandle_t slot_switch_time;
	xme_hal_time_timeHandle_t slot_switch_time_predecessor_slot;

	XME_LOG(XME_LOG_NOTE, "starting xme_tdma_xontrol_task. \n");


	/* blocks until xme_comp_tdma_activate has finished  */
	xme_hal_sync_enterCriticalSection(config->runtime_config->tdma_control_task_start_mutex);

	/* suspend all tasks in all TDMA slots before starting any slot */
	for( i=0; i<config->num_of_slots; i++ )
	{
		xme_tdma_suspend_tasks(config, i);
	}


	/* the absolute time, at which the next slot switch shall occur */
	slot_switch_time = xme_hal_time_getCurrentTime();

	/* the absolute time at which the last slot switch has occurred */
	slot_switch_time_predecessor_slot = slot_switch_time;

	/* scheduler starts here */
	for(;;)
	{
		/* loop over all TDMA slots */
		for(current_slot=0; current_slot<config->num_of_slots; ++current_slot)
		{
			xme_hal_time_timeInterval_t current_slot_length = config->slot_length_array[current_slot];
			/* the number of the next TDMA slot */
			next_slot = (current_slot >= last_slot_number) ? 0 : current_slot+1;
			XME_LOG(XME_LOG_DEBUG, "executing TDMA slot >%u< with length >%u< \n", current_slot, current_slot_length);

			slot_switch_time_predecessor_slot = slot_switch_time;
			/* the current slot shall end at (current_slot_length + last slot_switch_time) */
			slot_switch_time = xme_hal_time_offsetTime ( slot_switch_time, XME_HAL_TIME_OFFSET_OPERATION_ADD, (current_slot_length) );
			/* did the slow_switch_time xme_hal_time_timeHandle_t flow over? */
			time_overflow = ( 1 == xme_hal_time_compareTime( slot_switch_time_predecessor_slot, slot_switch_time) );
			/* before delaying the tdma_control_task, activate the scheduled tasks */
			xme_tdma_resume_tasks(config, current_slot);
			/* delaying the TDMA control task lets the tasks in the current slot start executing */
			actual_delay = xme_tdma_delay_until(slot_switch_time, time_overflow);
			if(current_slot_length != actual_delay)
			{
				/* the slot execution time is not as expected */
				XME_LOG(XME_LOG_WARNING, "slot %u was executing %i instead of %i in run %u \n", current_slot, actual_delay, current_slot_length, scheduler_run_counter);
			}
			/* suspend all tasks from the current tdma slot, i.e. end the current TDMA slot */
			xme_tdma_suspend_tasks(config, current_slot);
		}
		/* every slot has been executed once, increase the counter */
		scheduler_run_counter++;
		/* end tdma task */
		if(true == config->runtime_config->stop_task_flag)
		{
			break;
		}
	}
	/* the scheduler is ending its execution */

	/* resume all tasks from all slots after the end of the schedulers */
	for( i=0; i<config->num_of_slots; i++ )
	{
		xme_tdma_resume_tasks(config, i);
	}

	xme_hal_sync_leaveCriticalSection(config->runtime_config->tdma_control_task_start_mutex);
	/* kill the tdma_control_task */
	xme_core_resourceManager_killTask(0);
}


xme_status_t
xme_prim_tdma_activate ( xme_prim_tdma_configStruct_t* config )
{
	uint8_t prio = config->tdma_priority;
	xme_status_t result = XME_STATUS_INTERNAL_ERROR;

	XME_LOG(XME_LOG_VERBOSE, "adding tdma control task with prio %u \n", prio);

	/* take the tdma_control_task semaphore, causing the task to block before starting to execute */
	config->runtime_config->tdma_control_task_start_mutex = xme_hal_sync_createCriticalSection();
	/* take the mutex -> will cause the tdma_control_task to block until tdma_activate has finished */
	xme_hal_sync_enterCriticalSection(config->runtime_config->tdma_control_task_start_mutex);

	/* add the tdma_control_task to the native scheduler, without adding it to one of the tdma slots */
	xme_hal_sched_taskHandle_t handle = xme_hal_sched_addTask(0, 0, prio, xme_tdma_control_task, (void*)config);

	if (XME_HAL_SCHED_INVALID_TASK_HANDLE != handle)
	{
		/* store the tdma_control_task handle */
		config->runtime_config->tdma_control_task_handle = handle;
		result = XME_STATUS_SUCCESS;
	}
	else
	{
		result = XME_STATUS_INTERNAL_ERROR;
	}

	/* give back the semaphore and let the tdma control task start its execution */
	xme_hal_sync_leaveCriticalSection(config->runtime_config->tdma_control_task_start_mutex);

	return result;
}


void
xme_prim_tdma_deactivate ( xme_prim_tdma_configStruct_t* config )
{
	config->runtime_config->stop_task_flag = true;
	/* wait until the tdma_control_task has finished its last schedule */
	xme_hal_sync_enterCriticalSection(config->runtime_config->tdma_control_task_start_mutex);
	xme_hal_sync_leaveCriticalSection(config->runtime_config->tdma_control_task_start_mutex);
	xme_hal_sync_destroyCriticalSection(config->runtime_config->tdma_control_task_start_mutex);
	config->runtime_config->tdma_control_task_start_mutex = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;
}


xme_status_t
xme_prim_tdma_create ( xme_prim_tdma_configStruct_t* config )
{
	uint8_t slot = 0;

	xme_hal_time_timeHandle_t current = xme_hal_time_getCurrentTime();
	xme_hal_time_timeHandle_t current_offset = current;

	xme_tdma_runtimeConfig_t* runtime_config;

	/* prevent multiple creation, the TDMA scheduler is a Singleton */
	XME_CHECK(config->runtime_config == NULL, XME_STATUS_ALREADY_EXIST);

	runtime_config = xme_hal_mem_alloc(sizeof(xme_tdma_runtimeConfig_t));

	/* initialize the runtime config structure */
	config->runtime_config = runtime_config;
	config->runtime_config->stop_task_flag = false;
	config->runtime_config->tdma_control_task_handle = XME_HAL_SCHED_INVALID_TASK_HANDLE;
	config->runtime_config->tdma_control_task_start_mutex = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;
	config->runtime_config->total_scheduler_run_length = xme_hal_time_getTimeIntervalBetween(current, current); /* time interval of length 0 */
	XME_LOG(XME_LOG_DEBUG, "xme_tdma_create: config->num_of_slots: %u \n", config->num_of_slots);


	for(slot = 0; slot<config->num_of_slots; slot++)
	{
		XME_LOG(XME_LOG_DEBUG, "tdma_create: slot_length[%u]: %i \n ", slot, config->slot_length_array[slot]);
		if(NULL == tdma_tasks[slot])
		{
			/* initialize the task lists, if they have not been initialized earlier (e.g. by addTask_tdma) */
			tdma_tasks[slot] = xme_hal_mem_alloc(sizeof(xme_tdma_tasksList_t));
			XME_HAL_SINGLYLINKEDLIST_INIT(*(tdma_tasks[slot]));
		}
		current_offset = xme_hal_time_offsetTime(current_offset, XME_HAL_TIME_OFFSET_OPERATION_ADD, config->slot_length_array[slot]);
	}
	config->runtime_config->total_scheduler_run_length = xme_hal_time_getTimeIntervalBetween(current, current_offset);

	/* store the xme_comp_tdma_configStruct_t in a static variable */
	xme_prim_tdma_config = config;

	return XME_STATUS_SUCCESS;
}

/* TODO: secure access to config struct with semaphore */

void xme_prim_tdma_destroy ( xme_prim_tdma_configStruct_t* config){
	uint8_t slot;
	for(slot = 0; slot<config->num_of_slots; slot++)
	{
		XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*(tdma_tasks[slot]), xme_hal_sched_taskHandle_t, task_item);
			xme_hal_mem_free(task_item);
		XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

		xme_hal_mem_free(tdma_tasks[slot]);
		tdma_tasks[slot] = NULL;
	}
	xme_hal_mem_free(config->runtime_config);
	config->runtime_config = NULL;
	xme_prim_tdma_config = NULL;
}

