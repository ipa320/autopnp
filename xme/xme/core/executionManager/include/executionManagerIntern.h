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
 * $Id: executionManagerIntern.h 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Internal API and TBD functionality.
 */

#ifndef XME_CORE_EXEC_INTERN_H
#define XME_CORE_EXEC_INTERN_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/core/component.h"
#include "xme/core/executionManagerCallback.h"
#include "xme/hal/include/linkedList.h"
#include "xme/hal/include/sync.h"
#include "xme/hal/include/sched.h"
#include "xme/defines.h"

#include "xme/core/dataManagerTypes.h"

typedef enum xme_core_exec_state_value_ {
    XME_CORE_EXEC_NOT_INITIALIZED,
    XME_CORE_EXEC_RUNNING,
    XME_CORE_EXEC_PAUSED,
    XME_CORE_EXEC_TO_BE_SHUT_DOWN,
    XME_CORE_EXEC_SHUTDOWN
} xme_core_exec_state_value_t;

typedef struct xme_core_exec_state_ {
	xme_core_exec_state_value_t 			value;
	xme_hal_sync_criticalSectionHandle_t 	lock;
}
xme_core_exec_state_t;
/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

extern xme_core_exec_state_t xme_core_exec_state;
extern xme_core_exec_configStruct_t xme_core_exec_intConfig;

XME_EXTERN_C_END

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Scheduler initialization. This function is typically not called
 *        directly, but is executed as a part of xme_core_exec_init().
 */
xme_status_t
xme_core_exec_scheduler_init(void);

/**
 * \brief Prepare for cyclic execution
 * \returns XME status
 */
extern xme_status_t
xme_core_exec_scheduler_initExecution(void);


/**
 * \brief Prepare for cyclic execution
 * \returns XME status
 */
extern xme_status_t
xme_core_exec_dispatcher_initExecution(void);

extern bool
xme_core_exec_isInitialized( void );

/* todo: refactor: document */
extern xme_status_t
createRunnable
(
    xme_core_exec_functionDescriptor_t* functionWrapper,
    xme_hal_sched_taskHandle_t* runnable
);

/* todo: refactor: document */
extern xme_status_t
destroyRunnable
(
    xme_hal_sched_taskHandle_t runnable
);

extern xme_status_t
xme_core_exec_resetDbCycleCounter(void);

extern uint32_t
xme_core_exec_dispatcher_incCycleCount( void );

extern uint32_t
xme_core_exec_dispatcher_getCycleCount( void );

extern bool
xme_core_exec_dispatcher_maximumCycleReached(uint32_t maximumCycle);

/**
 * \brief   Creates an execution unit for the function wrapper
 *
 * \note    A task descriptor is created and added to the dispatcher table.
 *          If any initialization happens in the function wrapper, it is also
 *          performed before the execution unit (thread) blocks.
 * \param   functionWrapper A structure containing details on the callback
 *                  function and its runtime behavior, relevant for dispatch.
 * \param   eventTriggeredBehavior A flag indicating that "executionReady" will
 *                  take part in decision, whether a function has to be started
 *                  during its slot, or not.
 */
extern xme_status_t
xme_core_exec_dispatcher_createFunctionExecutionUnit
(
    xme_core_exec_functionDescriptor_t* functionWrapper,
    bool eventTriggeredBehavior
);


/**
 * \brief Resumes execution of a paused function.
 *
 * \param   componentId Locally unique identifier of the component,
 *          to which the function belongs.
 * \param   functionId Identifier of the function within a component.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if function has been resumed;
 *          - XME_CORE_STATUS_NO_SUCH_VALUE if no such task exists.
 */
xme_status_t
xme_core_exec_dispatcher_resumeFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);


/**
 * \brief Pauses execution of a running function.
 *
 * \param   componentId Locally unique identifier of the component,
 *          to which the function belongs.
 * \param   functionId Identifier of the function within a component.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if task is now paused;
 *          - XME_CORE_STATUS_NO_SUCH_VALUE if no such task exists.
 */
xme_status_t
xme_core_exec_dispatcher_pauseFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

/** \brief  Returns a valid pointer to the task scheduler record to be executed
 *          in the next slot and delay to wait before starting this task.
 *
 * \note    Multiple CPU's are currently not supported.
 *
 * \param   entry A pointer to the scheduler record of the next function to be
 *                  executed.
 * \param   startDelay_ns Delay till execution of the record (with respect to
 *                  current time).
 *
 *
 *
 * /// TODO
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if writing data into the shadow database was successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any error, like invalid port, etc.
 *          - ...
 */
extern
xme_status_t
xme_core_exec_scheduler_calculateNextComponent
(
    xme_core_exec_schedule_table_entry_t** entry,
    xme_hal_time_timeInterval_t*    startDelay_ns
);

/** todo: document */
xme_status_t
xme_core_exec_scheduler_getScheduleSetPointer
(
    xme_hal_linkedList_descriptor_t** target
);

/** \brief
 * \param schedule
 * \param scheduleId
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if writing data into the shadow database was successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any error, like invalid port, etc.
 *          - ...
 */
extern xme_status_t
xme_core_exec_scheduler_init( void );

extern xme_status_t
xme_core_exec_scheduler_fini( void );

extern xme_status_t
xme_core_exec_dispatcher_init( void );

extern xme_status_t
xme_core_exec_dispatcher_fini( void );

/**
 * \brief Prints the schedule in a form of a table on the console.
 */
extern void
xme_core_exec_scheduler_printSchedule
(
    xme_core_exec_schedule_table_t* schedule,
    const char* message
);

/**
 * \brief Main dispatcher loop step
 * Called exclusively by xme_core_exec_executionManager_step(),
 * so is not listed under public API.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS in any case.
 */
extern xme_status_t
xme_core_exec_dispatcher_executeStep
(
    const xme_core_exec_schedule_table_entry_t* const nextSlot, ///< Schedule table entry for the next slot
    xme_hal_time_timeInterval_t slack               ///< Current slack
);

xme_core_exec_state_value_t
xme_core_exec_getState( void );

void
xme_core_exec_setState
(
	xme_core_exec_state_value_t newState
);

extern bool
xme_core_exec_isValidFunctionDescriptor
(
    xme_core_exec_functionDescriptor_t* desc
);

/*-------------------------------------------------------------------------*/

xme_core_exec_schedule_handle_t
xme_core_exec_scheduler_getCurrentScheduleHandle( void );

xme_core_exec_schedule_handle_t
xme_core_exec_scheduler_getNextScheduleHandle( void );

/*-------------------------------------------------------------------------*/

extern xme_status_t
xme_core_exec_configurator_init( void );

/*-------------------------------------------------------------------------*/

extern xme_status_t
xme_core_exec_configurator_fini( void );

/*-------------------------------------------------------------------------*/

extern xme_status_t
xme_core_exec_configurator_lock( void );

/*-------------------------------------------------------------------------*/

extern xme_status_t
xme_core_exec_configurator_unlock( void );

/*-------------------------------------------------------------------------*/
/**
 * \brief A critical section locking shortcut with logging capability
 */
extern void
xme_core_exec_lockMutex
(
    const char* ID,
    xme_hal_sync_criticalSectionHandle_t handle,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

/*-------------------------------------------------------------------------*/
/**
 * \brief A critical section ulocking shortcut with logging capability
 *
 * Unlocks the mutex and adds a custom log message with component/function
 *      references.
 */
extern void
xme_core_exec_unlockMutex
(
    const char* ID,
    xme_hal_sync_criticalSectionHandle_t handle,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_EXEC_INTERN_H
