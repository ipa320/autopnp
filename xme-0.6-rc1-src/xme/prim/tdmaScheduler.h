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
 * $Id: tdmaScheduler.h 3445 2013-05-22 15:22:58Z ruiz $
 */

/**
 * \file
 *         Time Division Multiple Access (TDMA) scheduler
 */

#ifndef XME_PRIM_TDMASCHEDULER_H
#define XME_PRIM_TDMASCHEDULER_H

/* How to use the TDMA scheduler:
 * - design a slot assignment, set the number of TDMA slots and assign slot times to them
 * - rename all calls from xme_hal_sched_addTask to xme_hal_sched_addTask_tdma
 *   (this registers the task in the TDMA scheduler. Make sure to also change the calls in
 *   lower chromosome layers, otherwise tasks defined in lower layers are ignored by the TDMA
 *   scheduler - the TDMA scheduler can only handle tasks, which are created by using a
 *   function with the _tdma suffix. If a task is created with a non-TDMA function, this
 *   task might disturb the TDMA schedule cycle and steal CPU time for TDMA tasks.
 *
 *   original non-TDMA call                if using the TDMA-scheduler, call this function instead:
 *   xme_hal_sched_addTask                 xme_hal_sched_addTask_tdma
 *   xme_hal_sched_removeTask              xme_hal_sched_removeTask_tdma
 *   xme_core_resourceManager_scheduleTask xme_core_resourceManager_scheduleTask_tdma
 *   xme_core_resourceManager_killTask     xme_core_resourceManager_killTask_tdma
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/component.h"
#include "xme/core/resourceManager.h"

#include "xme/hal/include/sched.h"
#include "xme/hal/include/sync.h"
#include "xme/hal/include/time.h"

#include "xme/defines.h"


/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/


/**
 * \struct xme_tdma_runtimeConfig_t
 *
 * \brief  Sruct with runtime data for the tdma scheduler.
 */
typedef struct
{
	bool stop_task_flag; ///< set to true to stop the TDMA control task
	xme_hal_sched_taskHandle_t tdma_control_task_handle; ///< the task handle of the TDMA control task
	xme_hal_time_timeInterval_t total_scheduler_run_length; ///< the sum of all scheduler slots
	xme_hal_sync_criticalSectionHandle_t tdma_control_task_start_mutex; ///< a semaphore to delay the start of the TDMA control task
//	xme_hal_sync_criticalSectionHandle_t xme_tdma_config_struct_mutex; ///< a semaphore to protect the config structs from simultaneous access
}
xme_tdma_runtimeConfig_t;



/**
 * \struct xme_comp_tdma_configStruct_t
 *
 * \brief  Config struct for the TDMA component.
 */
XME_COMPONENT_CONFIG_STRUCT
(
	xme_prim_tdma,
	uint8_t num_of_slots; ///< total number of timeslots for the tdma scheduler.
	uint8_t tdma_priority; ///< the priority at which the tdma_control_task is executed. Recommended value: highest priority in the system.
	xme_hal_time_timeInterval_t slot_length_array[XME_PRIM_TDMA_MAX_SLOT_COUNT]; ///< the length for each TDMA slot.
	xme_tdma_runtimeConfig_t* runtime_config; ///< a pointer to the runtime information of the TDMA scheduler, the TDMA scheduler takes care of it, users should initialize it to NULL.
);


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN


/**
 * \brief  Schedules a task for execution and registers it in the TDMA scheduler.
 *
 * \note   On some platforms, a task's callback function might be run from
 *         a different execution context (e.g., a different thread), so
 *         synchronization mechanisms might be required if the task shares
 *         data with other tasks or the main program.
 *
 * \param  startTime If non-zero, specifies the time in nanoseconds from now
 *         when the task should be executed. A value of zero will cause the
 *         task to be started as soon as possible. A value of
 *         xme_HAL_SCHED_TASK_INITIALLY_SUSPENDED will cause the task to stay
 *         suspended until it is resumed using
 *         xme_hal_sched_setTaskExecutionState().
 * \param  period If non-zero, specifies the period in nanoseconds of the
 *         task. Otherwise, the task is a one-shot task and will be
 *         automatically removed after execution.
 * \param  priority Task priority (only meaningful for priority-based
 *         scheduler implementations). Higher values mean a higher priority.
 * \param  callback Callback function implementing the task to be executed.
 *         Note that on some platforms, this callback function might be run
 *         from a different execution context (e.g., a different thread),
 *         so synchronization mechanisms might be required if the task
 *         shares data with other tasks or the main program.
 * \param  slot TDMA slot in which the scheduled task will be executed.
 * \param  userData User-defined data to be passed to the callback function.
 * \return the task handle.
 */
xme_hal_sched_taskHandle_t
xme_hal_sched_addTask_tdma
(
	xme_hal_time_timeInterval_t startTime,
	xme_hal_time_timeInterval_t period,
	uint8_t priority,
	xme_hal_sched_taskCallback_t callback,
	uint8_t slot,
	void* userData
);


/**
 * \brief  Removes a task from the schedule and from the TDMA scheduler
 *
 * \details If the task is currently being executed, it is removed at the next
 *         possible point in time after finishing execution. If this function
 *         is called from the execution context of the respective task, the
 *         task is automatically removed after it returns control to the
 *         runtime system. If this function is called from a different
 *         execution context, the function will only return after the task has
 *         been successfully removed. This means that it is guaranteed that
 *         the task will not use any shared resources after this function
 *         returned. This function will also remove the task if it is
 *         currently suspended, so resuming the task is not required
 *         beforehand if it might be suspended.
 *
 * \note   If the respective task executes an infinite loop, a call to this
 *         function from a different execution context than that of the
 *         respective task will never return!
 *
 * \param  taskHandle Handle of the task to remove.
 * \retval XME_CORE_STATUS_SUCCESS if the task has been removed from schedule.
 * \retval XME_CORE_STATUS_INVALID_HANDLE if the given task handle was
 *            invalid.
 */
xme_status_t
xme_hal_sched_removeTask_tdma(xme_hal_sched_taskHandle_t taskHandle);


/**
 * \brief  Tries to schedule the task according to the given parameters.
 *         If a new task with the given parameters is not feasible, the
 *         function will fail. Also adds the task to the TDMA scheduler
 *
 * \param  startTime If non-zero, specifies the time in nanoseconds from
 *         now when the task should be executed. A value of zero will cause
 *         the task to be started as soon as possible. A value of
 *         XME_HAL_SCHED_TASK_INITIALLY_SUSPENDED will cause the task to stay
 *         suspended until it is resumed using
 *         xme_hal_sched_setTaskExecutionState().
 * \param  period If non-zero, specifies the period in nanoseconds of
 *         the task. Otherwise, the task is a one-shot task and will be
 *         automatically removed after execution.
 * \param  priority Task priority (only meaningful for priority-based
 *         scheduler implementations). Higher values mean a higher priority.
 * \param  callback Callback function implementing the task to be executed.
 *         Note that on some platforms, this callback function might be run
 *         from a different execution context (e.g., a different thread),
 *         so synchronization mechanisms might be required if the task
 *         shares data with other tasks or the main program.
 * \param  slot TDMA slot in which the scheduled task will be executed.
 * \param  userData User-defined data to be passed to the callback function.
 *
 * \return Returns a task handle on success. If the a task according to the
 *         given parameters is not feasible, the function fails and returns
 *         XME_CORE_RESOURCEMANAGER_TASK_INFEASIBLE.
 */
xme_core_resourceManager_taskHandle_t
xme_core_resourceManager_scheduleTask_tdma
(
	xme_hal_time_timeInterval_t startTime,
	xme_hal_time_timeInterval_t period,
	uint8_t priority,
	xme_hal_sched_taskCallback_t callback,
	uint8_t slot,
	void* userData
);


/**
 * \brief  Removes the given task from the schedule. However, if the task is
 *         currently running, it will not be stopped. In case of a one-shot
 *         task, this function can be used to prevent starting of the task
 *         before it has been started. In case of a periodic task, this
 *         function will prevent subsequent invocations of the task.
 *         The task is also removed from the TDMA schedule
 *
 * \param  taskHandle Handle of the task to remove.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if the task has been successfully
 *            removed.
 *          - XME_STATUS_INVALID_PARAMETER if the given task handle
 *            is invalid.
 */
xme_status_t
xme_core_resourceManager_killTask_tdma
(
	xme_core_resourceManager_taskHandle_t taskHandle
);

/**
 * \brief  Activates the TDMA scheduler component.
 *
 * \details Please note, that there should only be one TDMA scheduler component on each node.
 *         Activating the TDMA component will start the TDMA control task and activate
 *         the TDMA scheduler's slot based scheduling algorithm.
 * \param  config the configuration parameters.
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if the component has been successfully
 *            activated.
 *          - XME_STATUS_INTERNAL_ERROR if the component cannot be activated.
 */
xme_status_t
xme_prim_tdma_activate ( xme_prim_tdma_configStruct_t* config );

/**
 * \brief  Deactivates the TDMA scheduler component.
 * \param  config the configuration parameters.
 */
void
xme_prim_tdma_deactivate ( xme_prim_tdma_configStruct_t* config );

/**
 * \brief  Creates the TDMA scheduler component.
 *
 * \details Please note, that there should only be one TDMA scheduler component on each node.
 *         Creating the TDMA component will initialize its data structures.
 * \param  config the configuration parameters.
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if the component has been successfully
 *            created.
 *          - XME_STATUS_INTERNAL_ERROR if the component cannot be created.
 */
xme_status_t
xme_prim_tdma_create ( xme_prim_tdma_configStruct_t* config );

/**
 * \brief  Destroys the TDMA scheduler component.
 * \param  config the configuration parameters.
 */
void
xme_prim_tdma_destroy ( xme_prim_tdma_configStruct_t* config );


XME_EXTERN_C_END

#endif /* XME_PRIM_TDMASCHEDULER_H */
