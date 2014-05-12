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
 * $Id: wrapperInterface.c 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Dispatcher implementation: part responsible for functionWrapper interface
 */

#define MODULE_ACRONYM "ExecMgr   : "

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
#include "xme/hal/include/signal.h"

#include <inttypes.h>

/***********/
/* Statics */
extern xme_hal_sync_criticalSectionHandle_t cpuToken;
extern void* startData;

/** \brief This utility function is called by the task to get access to
 *          "cpu" resource
 */
static xme_status_t
xme_core_exec_dispatcher_requestExecutionToken
(
    xme_core_exec_taskDescriptor_t* task
);

static xme_status_t
xme_core_exec_dispatcher_returnExecutionToken
(
    xme_core_exec_taskDescriptor_t* task
);

/****************************************************************************/
/*          Wrapper                                                         */
/****************************************************************************/



/****************************************************************************/
#ifdef __PikeOS__disabled
#include "xme/hal/include/task.h"
#define SIGNAL_START 0x00
#define SIGNAL_COMPLETED 0x01
#define SIGNAL_READY 0x01

#define MEMNAME_SIZE 16

xme_core_component_t thisComponentId;
xme_core_component_functionId_t thisFunctionId;

xme_status_t
xme_hal_signal_set(uint32_t signalId)
{
    XME_UNUSED_PARAMETER(signalId);
}

xme_status_t
xme_hal_signal_wait(uint32_t signalId)
{
    XME_UNUSED_PARAMETER(signalId);
}

xme_status_t
xme_core_exec_dispatcher_initializeTask
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    char memName[MEMNAME_SIZE];
    /* do I need this? */
    thisComponentId = componentId;
    thisFunctionId = functionId;

    /* map shared memory with signals */
    snprintf(memName, MEMNAME_SIZE, "EM_%d", xme_hal_task_getCurrentProcessId());

    /* set the signal */
    xme_hal_signal_set(SIGNAL_READY);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_dispatcher_waitForStart
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void** functionArguments
)
{
    xme_hal_signal_wait(SIGNAL_START);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_dispatcher_executionCompleted
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    xme_hal_signal_set(SIGNAL_COMPLETED);
    return XME_STATUS_SUCCESS;
}
#else
xme_status_t
xme_core_exec_dispatcher_initializeTask
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    /* Look up for a task descriptor */
    xme_core_exec_taskDescriptor_t* thisTask = NULL;

    /* We only deal with an initialized component */
    if (!xme_core_exec_isInitialized())
            return XME_STATUS_UNEXPECTED;

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(t): > [%d|%d] dispatcher_initializeTask()\n",
            componentId,
            functionId);
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_descriptorTable_getTaskDescriptor(componentId, functionId, &thisTask),
        XME_STATUS_NOT_FOUND);

    /* Move the task to running state */
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_setTaskState(componentId, functionId, XME_CORE_EXEC_FUNCTION_STATE_RUNNING),
        XME_STATUS_INTERNAL_ERROR);

    /* Execution manager initially owns the mutex allowing the task to execute */
    xme_core_exec_lockMutex("S/t",thisTask->waitLock, componentId, functionId);

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(t): < [%d|%d] dispatcher_initializeTask()\n",
            componentId,
            functionId);

    return XME_STATUS_SUCCESS;
}


/****************************************************************************/
xme_status_t
xme_core_exec_dispatcher_waitForStart
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void** functionArguments
)
{
    xme_core_exec_taskDescriptor_t* thisTask = NULL;

    /* We only deal with an initialized component */
    if (!xme_core_exec_isInitialized())
            return XME_STATUS_UNEXPECTED;

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(t): > dispatcher_waitForStart()\n");

    XME_CHECK_MSG(XME_STATUS_SUCCESS ==
        xme_core_exec_descriptorTable_getTaskDescriptor(componentId, functionId, &thisTask),
        XME_STATUS_NOT_FOUND,
        XME_LOG_FATAL,
        MODULE_ACRONYM "task [%d|%d] not present in the task table!\n",
        componentId, functionId);

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(t): [%d|%d] waitForStart()\n",  componentId, functionId);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_requestExecutionToken(thisTask),
              XME_STATUS_INTERNAL_ERROR);

    XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "passing %" PRIu32 " to function [%d|%d]\n",
            (uint32_t)(uintptr_t)startData, componentId, functionId);

    if (NULL != functionArguments)
        *functionArguments = startData;

    /* XXX temporary workaround to allow smooth transition to xme_core_exec_getTaskState() */
    thisTask->wrapper->state = thisTask->state;

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(t): < [%d|%d] dispatcher_waitForStart()\n",
        componentId, functionId);
    return XME_STATUS_SUCCESS;
}


/****************************************************************************/
xme_status_t
xme_core_exec_dispatcher_executionCompleted
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    xme_core_exec_taskDescriptor_t* thisTask = NULL;

    /* We only deal with an initialized component */
    if (!xme_core_exec_isInitialized())
            return XME_STATUS_UNEXPECTED;

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(t): > [%d|%d] dispatcher_executionCompleted()\n",
            componentId,
            functionId);

    /* Look up for a task descriptor */
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_descriptorTable_getTaskDescriptor(componentId, functionId, &thisTask),
        XME_STATUS_NOT_FOUND);

    /* Stop monitoring the task */
    thisTask->running = false;

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "(t): [%d|%d] execution time total = %" PRIu64 "\n",
        componentId, functionId,
        xme_hal_time_getTimeInterval(&(thisTask->startTime), (bool)false));

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_returnExecutionToken(thisTask),
              XME_STATUS_INTERNAL_ERROR);

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "***************** /executionCompleted  [%d|%d] *********************\n",
        componentId,
        functionId);

    return XME_STATUS_SUCCESS;
}


// XXX: Code review: check the xme_hal_sync calls for return status
/****************************************************************************/
static xme_status_t
xme_core_exec_dispatcher_requestExecutionToken
(
    xme_core_exec_taskDescriptor_t* task
)
{
    xme_core_exec_functionDescriptor_t* function = NULL;
    XME_CHECK(NULL != task,
              XME_STATUS_INVALID_HANDLE);

    function = task->wrapper;

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "[%d|%d] dispatcher_requestExecutionToken()\n",
            function->componentId, function->functionId);

    xme_core_exec_lockMutex("T/t", task->execLock, function->componentId, function->functionId);
    xme_core_exec_unlockMutex("S/t", task->waitLock, function->componentId, function->functionId);
    xme_core_exec_lockMutex("R/t", cpuToken, function->componentId, function->functionId);

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "[%d|%d] /dispatcher_requestExecutionToken()\n",
            function->componentId, function->functionId);

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
static xme_status_t
xme_core_exec_dispatcher_returnExecutionToken
(
    xme_core_exec_taskDescriptor_t* task
)
{
    xme_core_exec_functionDescriptor_t* function = NULL;
    XME_CHECK(NULL != task,
                  XME_STATUS_INVALID_HANDLE);

    function = task->wrapper;

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "[%d|%d] dispatcher_returnExecutionToken()\n",
            function->componentId, function->functionId);

    xme_core_exec_unlockMutex("T/t", task->execLock, function->componentId, function->functionId);
    xme_core_exec_unlockMutex("R/t", cpuToken, function->componentId, function->functionId);
    xme_core_exec_lockMutex("S/t", task->waitLock, function->componentId, function->functionId);

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "[%d|%d] /dispatcher_returnExecutionToken()\n",
            function->componentId, function->functionId);

    return XME_STATUS_SUCCESS;
}
#endif
