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
 * $Id: sched.h 5190 2013-09-26 13:10:19Z camek $
 */

/**
 * \file
 * \brief Public scheduler API.
 */

#ifndef XME_HAL_SCHED_H
#define XME_HAL_SCHED_H

/**
 * \defgroup hal_sched  Scheduler Abstraction
 * @{
 *
 * \brief This component allows the scheduling of tasks.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/executionManagerCallback.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/** \def XME_HAL_SCHED_TASK_INITIALLY_SUSPENDED
 * \brief Used in conjunction with xme_hal_sched_addTask() to indicate that
 *        a task should be created suspended.
 */
#define XME_HAL_SCHED_TASK_INITIALLY_SUSPENDED ((xme_hal_time_timeInterval_t)0xFFFFFFFFFFFFFFFFULL)

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/** \enum xme_hal_sched_taskHandle_t
 *
 * \brief Task handle.
 */
typedef enum
{
    XME_HAL_SCHED_INVALID_TASK_HANDLE = 0, ///< Invalid task handle.
    XME_HAL_SCHED_MAX_TASK_HANDLE = XME_MAX_SYSTEM_VALUE ///< Largest possible task handle.
}
xme_hal_sched_taskHandle_t;

/**
 * \enum xme_hal_sched_taskState_t
 * \brief Task state.
 */
typedef enum
{
    XME_HAL_SCHED_TASK_STATE_RUNNING = 0, ///< Task is currently running. Note that this does not imply when the task will be executed, it just means that the task is scheduled for execution.
    XME_HAL_SCHED_TASK_STATE_SUSPENDED = 1, ///< Task is suspended.
    XME_HAL_SCHED_TASK_STATE_TERMINATING = 255 ///< Task is about to be removed. Since tasks can only be removed when they are not currently executed, this flag indicates that the task will be removed at the next possible point in time after it returns control to the runtime system.
}
xme_hal_sched_taskState_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the scheduler.
 *
 * \retval XME_CORE_STATUS_SUCCESS the scheduler component has been successfully initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if not enough resources were available.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR the scheduler cannot be initialized.
 */
xme_status_t
xme_hal_sched_init(void);

/**
 * \brief  Schedules a task for execution.
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
 * \param  userData User-defined data to be passed to the callback function.
 * \return the task handle.
 */
xme_hal_sched_taskHandle_t
xme_hal_sched_addTask
(
    xme_hal_time_timeInterval_t startTime,
    xme_hal_time_timeInterval_t period,
    uint8_t priority,
    xme_hal_sched_taskCallback_t callback,
    void* userData
);

/**
 * \brief  Sets task exection state of the task with the given handle.
 *
 *         If the task is currently being executed, its execution state will
 *         be updated at the next possible point in time after finishing
 *         execution. The function will block until the new state has been
 *         enforced.
 *
 * \note   The attempt to suspend or resume a task that is scheduled for
 *         termination (i.e., xme_hal_sched_removeTask() has been called
 *         on the task) will fail with a status code of
 *         XME_CORE_STATUS_PERMISSION_DENIED.
 *
 * \param  taskHandle Handle of the task to set execution state of.
 * \param  running Flag indicating whether the task should currently be
 *         executed. A value of true will resume the task, a value of false
 *         will suspend it.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the task's execution state has
 *            been successfully updated.
 * \retval XME_CORE_STATUS_INVALID_HANDLE if the given task handle was
 *            invalid.
 * \retval XME_CORE_STATUS_INVALID_CONFIGURATION if the task is
 *            scheduled for termination and was already running.
 *            Suspending a task scheduled for termination is not
 *            supported.
 * \retval XME_CORE_STATUS_PERMISSION_DENIED if the task is scheduled
 *            for termination. It is not allowed to suspend or resume a
 *            thread waiting for termination.
 *
 */
xme_status_t
xme_hal_sched_setTaskExecutionState
(
    xme_hal_sched_taskHandle_t taskHandle,
    bool running
);

/**
 * \brief  Removes a task from the schedule.
 *
 * \details If the task is currently being executed, it is removed at the next
 *          possible point in time after finishing execution. If this function
 *          is called from the execution context of the respective task, the
 *          task is automatically removed after it returns control to the
 *          runtime system. If this function is called from a different
 *          execution context, the function will only return after the task has
 *          been successfully removed. This means that it is guaranteed that
 *          the task will not use any shared resources after this function
 *          returned. This function will also remove the task if it is
 *          currently suspended, so resuming the task is not required
 *          beforehand if it might be suspended.
 *
 * \note   If the respective task executes an infinite loop, a call to this
 *         function from a different execution context than that of the
 *         respective task will never return!
 *
 * \param  taskHandle Handle of the task to remove.
 * \retval XME_CORE_STATUS_SUCCESS if the task has been removed from schedule.
 * \retval XME_CORE_STATUS_INVALID_HANDLE if the given task handle was
 *            invalid.
 * \retval XME_STATUS_INTERNAL_ERROR if an internal error occurred.
 */
xme_status_t
xme_hal_sched_removeTask(xme_hal_sched_taskHandle_t taskHandle);

/**
 * \brief  Frees resources occupied by the scheduler.
 */
void
xme_hal_sched_fini(void);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_HAL_SCHED_H
