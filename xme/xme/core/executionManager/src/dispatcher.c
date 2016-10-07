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
 * $Id: dispatcher.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Dispatcher implementation.
 */

#define MODULE_ACRONYM "ExecMgr   : "

#define TMP_FINI_SLEEP 1

#define TMP_FINI_NODESTRUCT 1

/****************************************************************************/
/**   Includes                                                             **/
/****************************************************************************/
#include "xme/defines.h"
#include "xme/core/executionManager/include/internDescriptorTable.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/executionManager/include/executionManagerWrapperInterface.h"
#include "xme/core/broker/include/brokerDataManagerInterface.h"
#include "xme/core/log.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/sleep.h"

#include <inttypes.h>

/****************************************************************************/
/**   Variables                                                            **/
/****************************************************************************/
/**
 * \var taskDescriptorTable
 * \brief A list of all tasks protected with a mutex.
 */
//static xme_hal_sync_criticalSectionHandle_t taskDescriptorsMutex; ///< Mutex for shared access to the task list.
//static XME_HAL_TABLE(xme_core_exec_taskDescriptor_t, taskDescriptorTable, XME_CORE_EXEC_TASKDESCRIPTORTABLE_SIZE); ///< Global list of tasks

uint32_t dispatcherCycleCounter;

// XXX these were statics, think of implications
xme_hal_sync_criticalSectionHandle_t cpuToken;
void* startData;


/****************************************************************************/
/**   Prototypes                                                           **/
/****************************************************************************/

/**
 * \brief Initialization of dispatcher.
 *
 * Called exclusively by xme_core_exec_executionManager_init(),
 * so is not listed under public API.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if initialization is successful.
 *          - XME_STATUS_INTERNAL_ERROR if initialization could
 *              not be completed.
 */
extern xme_status_t
xme_core_exec_dispatcher_init( void );

/*
 * \brief Finalize the dispatcher.
 *
 * Called exclusively by xme_core_exec_executionManager_init(),
 * so is not listed under public API.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS in any case.
 *
 *     odo enhance functionality to handle stop-errors and returns.
 */
extern xme_status_t
xme_core_exec_dispatcher_fini( void );

/**
 * \brief Dispatcher calls this function to activate the task
 */
static xme_status_t
xme_core_exec_dispatcher_grantExecutionToken
(
    xme_core_exec_taskDescriptor_t* task
);

/**
 * \brief Dispatcher calls this after activation of the task to get it locked
 *          again as soon as it completes
 */
static xme_status_t
xme_core_exec_dispatcher_recaptureExecutionToken
(
    xme_core_exec_taskDescriptor_t* taskDesc
);

/**
 * \brief Dispatcher calls this to activate a task.
 */
static xme_status_t
xme_core_exec_dispatcher_startFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArgs
);

/**
 * \brief Prepare for cyclic execution
 * \returns XME status
 */
extern xme_status_t
xme_core_exec_dispatcher_initExecution(void);

/** Statics **/

static xme_status_t
xme_core_exec_dispatcher_checkWCETConformance(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId);


/** \brief This function is executed during final iteration through the task
 *          descriptor table to stop the function execution correctly
 */
static void
terminationCallback
(
    xme_core_exec_taskDescriptor_t* task
);

//******************************************************************************//
//***   Implementation                                                       ***//
//******************************************************************************//

/****************************************************************************/
xme_status_t
xme_core_exec_dispatcher_init( void )
{
    /* Create CPU resource representation */
    cpuToken = xme_hal_sync_createCriticalSection();
    XME_CHECK( XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != cpuToken,
        XME_STATUS_INTERNAL_ERROR);

    /* Initially dispatcher owns the resource */
    xme_core_exec_lockMutex("CPU/m", cpuToken, (xme_core_component_t)0, (xme_core_component_functionId_t)0);

    /* Create the descriptor table */
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_descriptorTable_init(),
        XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_dispatcher_initExecution(void)
{
	/* Reset internal cycle counter */
	dispatcherCycleCounter = 0;

    /* Set to execution state */
    xme_core_exec_setState(XME_CORE_EXEC_RUNNING);

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_dispatcher_fini( void )
{
    /**
     *     odo should stop all tasks before completing; monitor this case better
     *          there are some stochastic problems in RACE
     */

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "terminating all tasks...\n");

    /* Terminate all registered tasks */
    if(XME_STATUS_SUCCESS != xme_core_exec_descriptorTable_forEach(terminationCallback))
        XME_LOG(XME_LOG_ERROR, MODULE_ACRONYM "error terminating functions\n");

#ifdef TMP_FINI_SLEEP
    xme_hal_sleep_sleep(((xme_hal_time_timeInterval_t)1000)*1000*10); // 10 ms
#endif

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "terminated.\n");
#ifndef TMP_FINI_NODESTRUCT
    /* Unlock the resource before destroying it */
    xme_core_exec_unlockMutex("CPU/m", cpuToken, (xme_core_component_t)0, (xme_core_component_functionId_t)0);

    /* Destroy the resource representation */
    if(XME_STATUS_SUCCESS != xme_hal_sync_destroyCriticalSection(cpuToken))
        XME_LOG(XME_LOG_ERROR, MODULE_ACRONYM "error destroying mutex representing CPU resource\n");
    cpuToken = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;

    /* Clean the descriptor table */
    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "finalizing descriptor table.\n");
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_descriptorTable_fini(),
                  XME_STATUS_INTERNAL_ERROR);
#endif
    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
static void
terminationCallback(xme_core_exec_taskDescriptor_t* task)
{
    xme_core_exec_functionDescriptor_t* function = NULL;

    XME_CHECK_MSG(task!=NULL,, XME_LOG_WARNING,
                  MODULE_ACRONYM "terminating tasks: task descriptor=NULL");

    function = task->wrapper;

    /* Initiate termination */
    if(XME_STATUS_SUCCESS != xme_core_exec_setTaskState(function->componentId, function->functionId, XME_CORE_EXEC_FUNCTION_STATE_TERMINATED))
        XME_LOG(XME_LOG_ERROR, MODULE_ACRONYM "could not set function state to TERMINATED\n");

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "Terminating task [%d|%d]", function->componentId,
            function->functionId);

    /* Trigger the task to enter the loop and return immediately */
    if(XME_STATUS_SUCCESS != xme_core_exec_dispatcher_startFunction(
            function->componentId,
            function->functionId,
            NULL))
    {
        XME_LOG(XME_LOG_WARNING, MODULE_ACRONYM "could not reactivate the task %d for normal completion\n",
                (int)(task->handle));
    }

    /* Upon completion get back the CPU resource */
    if(XME_STATUS_SUCCESS != xme_core_exec_dispatcher_recaptureExecutionToken(task))
    {
        XME_LOG(XME_LOG_WARNING, MODULE_ACRONYM "could not reactivate the task %d for normal completion\n",
                (int)(task->handle));
    }

#ifdef TMP_FINI_SLEEP
    xme_hal_sleep_sleep(((xme_hal_time_timeInterval_t)1000)*100); // 0.1 ms
#endif

    /* fixme: refactor: add join; this hangs currently under Linux */
    /* Mentioned in ticket #2878 */
#if 0
    if(XME_STATUS_SUCCESS != destroyRunnable(task->handle))
    {
        XME_LOG(XME_LOG_WARNING, MODULE_ACRONYM "could not destroy runnable for task %d!\n",
                        (int)(task->handle));
    }
#endif
}

/****************************************************************************/
xme_status_t
xme_core_exec_dispatcher_executeStep
(
    const xme_core_exec_schedule_table_entry_t* const nextSlot, ///< Schedule table entry for the next slot
    xme_hal_time_timeInterval_t slack               ///< Current slack
)
{
    /* Descriptor of the task to be executed */
    xme_core_exec_taskDescriptor_t* executedTask = NULL;

    /* Determines if the task will be executed at all */
    bool executeTask = true;

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(m): nextSlot = [%d|%d]\n",
        nextSlot->componentId, nextSlot->functionId);

    // XXX: Code review: maybe merge the branches???

    /* Scheduler tells us also the slack that we have; we either sleep or immediately execute */
    if (slack > 0)
    {
        XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "Schedule slack: [%" PRIu64 "] us\n",
            xme_hal_time_timeIntervalInMicroseconds(slack));
        xme_hal_sleep_sleep(slack);
    }
    else
    {
        // XXX: Code review : remove sleep(0)
        // XXX incorrect context for the comment (we don't see the negative slack)
        XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "[Negative slack in schedule]\n");
        xme_hal_sleep_sleep(0ULL);
    }

    /* Get descriptor of the next task that scheduler told us */
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_descriptorTable_getTaskDescriptor(nextSlot->componentId,
                                                                                    nextSlot->functionId,
                                                                                    &executedTask),
              XME_STATUS_NOT_FOUND);

    if (executedTask->eventTriggered)
    {

        // XXX Code Review: use descriptor of task instead of cid/fid in the internal calls

        /* Check if an ET task has all data ready */
        xme_status_t functionReady = XME_STATUS_UNEXPECTED;
        functionReady = xme_core_broker_isFunctionReady(nextSlot->componentId,
                                                        nextSlot->functionId,
                                                        nextSlot->functionArgs);
        if(XME_STATUS_SUCCESS != functionReady)
            //  XXX Code Review: RETURN from here
            executeTask = false;
    }

    if( executeTask )
    {
        XME_CHECK(XME_STATUS_SUCCESS ==
                   xme_core_exec_dispatcher_startFunction(nextSlot->componentId,
                                                          nextSlot->functionId,
                                                          nextSlot->functionArgs),
                   XME_STATUS_INTERNAL_ERROR);

        /* If we have started a task, we have to catch its S-lock before we exit, and check the runtime */
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_recaptureExecutionToken(executedTask),
                  XME_STATUS_INTERNAL_ERROR);

        if(NULL != xme_core_exec_intConfig.onFunctionReturned)
        {
            xme_core_exec_intConfig.onFunctionReturned(nextSlot->componentId, nextSlot->functionId);
        }

        if( nextSlot->completion )
            XME_CHECK(XME_STATUS_SUCCESS ==
                xme_core_exec_dispatcher_checkWCETConformance(nextSlot->componentId,
                                                              nextSlot->functionId),
                    XME_STATUS_TIMEOUT);
    }

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(m): < dispatcher_executeStep()\n");

    return XME_STATUS_SUCCESS;
}



/****************************************************************************/
/*          Dispatcher                                                      */
/****************************************************************************/

/****************************************************************************/
static xme_status_t
xme_core_exec_dispatcher_startFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArgs
)
{
    xme_core_exec_taskDescriptor_t* task = NULL;

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "(m): > [%d|%d] dispatcher_startFunction\n", componentId,
            functionId);

    XME_CHECK(XME_STATUS_SUCCESS
                  == xme_core_exec_descriptorTable_getTaskDescriptor(
                                  componentId,
                                  functionId,
                                  &task),
              XME_STATUS_NOT_FOUND);
    XME_LOG(XME_LOG_VERBOSE, MODULE_ACRONYM "(m): [%d|%d]: starting\n", componentId, functionId);

    /* XXX should the descriptor table not bbe locked? */
    task->running = true;
    task->startTime = xme_hal_time_getCurrentTime();
    startData = functionArgs;

    /*  */

    if(NULL != xme_core_exec_intConfig.onFunctionActivate)
    {
        xme_core_exec_intConfig.onFunctionActivate(task->wrapper->componentId, task->wrapper->functionId);
    }

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_grantExecutionToken(task),
              XME_STATUS_INTERNAL_ERROR);

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(m): < [%d|%d] dispatcher_startFunction()\n",
        componentId, functionId);

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
uint32_t
xme_core_exec_dispatcher_incCycleCount( void )
{
    /* We only deal with an initialized component */
    if (!xme_core_exec_isInitialized())
            return XME_STATUS_UNEXPECTED;

    return ++dispatcherCycleCounter;
}

/****************************************************************************/
uint32_t
xme_core_exec_dispatcher_getCycleCount( void )
{
    /* We only deal with an initialized component */
    if (!xme_core_exec_isInitialized())
            return XME_STATUS_UNEXPECTED;

    return dispatcherCycleCounter;
}

/****************************************************************************/
bool
xme_core_exec_dispatcher_maximumCycleReached(uint32_t maximumCycle)
{
	return (!(0ULL == maximumCycle)) && (0ULL != dispatcherCycleCounter) && (dispatcherCycleCounter > maximumCycle);
}

/****************************************************************************/
/**     TODO: refactor: implement
*/
static xme_status_t
xme_core_exec_dispatcher_checkWCETConformance(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId)
{
    xme_core_exec_taskDescriptor_t* thisTask = NULL;

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_descriptorTable_getTaskDescriptor(componentId, functionId, &thisTask),
        XME_STATUS_NOT_FOUND);

    // TODO: Monitor time to completion

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
static xme_status_t
xme_core_exec_dispatcher_grantExecutionToken
(
    xme_core_exec_taskDescriptor_t* task
)
{
    xme_core_exec_functionDescriptor_t* function = NULL;
    XME_CHECK(NULL != task,
                  XME_STATUS_INVALID_HANDLE);

    function = task->wrapper;

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "[%d|%d] dispatcher_grantExecutionToken()\n",
            function->componentId, function->functionId);

    xme_core_exec_unlockMutex("T/m", task->execLock, function->componentId, function->functionId);
    xme_core_exec_lockMutex("S/m", task->waitLock, function->componentId, function->functionId);
    xme_core_exec_unlockMutex("R/m", cpuToken, function->componentId, function->functionId);

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "[%d|%d] /dispatcher_grantExecutionToken()\n",
            function->componentId, function->functionId);

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
static xme_status_t
xme_core_exec_dispatcher_recaptureExecutionToken
(
    xme_core_exec_taskDescriptor_t* task
)
{
    xme_core_exec_functionDescriptor_t* function = NULL;
    XME_CHECK(NULL != task,
                  XME_STATUS_INVALID_HANDLE);

    function = task->wrapper;

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "[%d|%d] dispatcher_recaptureExecutionToken()\n",
            function->componentId, function->functionId);

    xme_core_exec_lockMutex("T/m", task->execLock, function->componentId, function->functionId);
    xme_core_exec_lockMutex("R/m", cpuToken, function->componentId, function->functionId);
    xme_core_exec_unlockMutex("S/m", task->waitLock, function->componentId, function->functionId);

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "[%d|%d] /dispatcher_recaptureExecutionToken()\n",
            function->componentId, function->functionId);

    return XME_STATUS_SUCCESS;
}


