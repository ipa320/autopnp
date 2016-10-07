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
 * $Id: descriptorTable.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Descriptor table implementation
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
/* Component-specific data */
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/core/executionManager/include/executionManagerWrapperInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/executionManager/include/internDescriptorTable.h"
/* Components that we use or interface */

/* Basic structures */
#include "xme/hal/include/table.h"

/* Chromosome logging */
#include "xme/core/log.h"

/****************************************************************************/
/**   Logging prefix                                                       **/
/****************************************************************************/
#define MODULE_ACRONYM "ExecMgrDTb: "

/****************************************************************************/
/**   Variables                                                            **/
/****************************************************************************/
static xme_hal_sync_criticalSectionHandle_t taskDescriptorsMutex; ///< Mutex for shared access to the task list.
static XME_HAL_TABLE(xme_core_exec_taskDescriptor_t, taskDescriptorsTable, XME_CORE_EXEC_TASKDESCRIPTORTABLE_SIZE); ///< Global list of tasks

/****************************************************************************/
/**   Prototypes                                                           **/
/****************************************************************************/

static xme_status_t
xme_core_exec_dispatcher_addNewTaskDescriptor
(
    xme_core_exec_taskDescriptor_t** record1
);

static xme_status_t
xme_core_exec_dispatcher_initializeTaskDescriptor
(
    xme_core_exec_taskDescriptor_t* taskDescriptor,
    xme_core_exec_functionDescriptor_t* functionDescriptor,
    bool eventTriggered
);

static xme_status_t
xme_core_exec_dispatcher_startRunnable
(
    xme_core_exec_functionDescriptor_t* functionDescriptor
);

/****************************************************************************/
/**   Implementation                                                       **/
/****************************************************************************/

/*--------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_descriptorTable_init( void )
{
    /* Initialize a critical section to control the access to the task list */
    taskDescriptorsMutex = xme_hal_sync_createCriticalSection();

    XME_CHECK( XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != taskDescriptorsMutex,
        XME_STATUS_INTERNAL_ERROR);

    xme_hal_sync_enterCriticalSection(taskDescriptorsMutex);

    XME_HAL_TABLE_INIT(taskDescriptorsTable);

    xme_hal_sync_leaveCriticalSection(taskDescriptorsMutex);
    return XME_STATUS_SUCCESS;
}

/*--------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_descriptorTable_fini( void )
{
    xme_hal_sync_enterCriticalSection(taskDescriptorsMutex);
    XME_HAL_TABLE_FINI(taskDescriptorsTable);
    xme_hal_sync_leaveCriticalSection(taskDescriptorsMutex);

    if(XME_STATUS_SUCCESS != xme_hal_sync_destroyCriticalSection(taskDescriptorsMutex))
        XME_LOG(XME_LOG_ERROR, MODULE_ACRONYM "could not destroy task descriptor table mutex!\n");
    return XME_STATUS_SUCCESS;

}

/*--------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_descriptorTable_forEach
(
    void (*fun)(xme_core_exec_taskDescriptor_t*)
)
{
    XME_CHECK_MSG( fun!=NULL,
        XME_STATUS_INVALID_PARAMETER,
        XME_LOG_WARNING,
        MODULE_ACRONYM "Callback function is NULL!\n");

    XME_HAL_TABLE_ITERATE_BEGIN(taskDescriptorsTable,
                                   int, rowHandle,
                                   xme_core_exec_taskDescriptor_t, curTask);
           fun(curTask);
       XME_HAL_TABLE_ITERATE_END();
       return XME_STATUS_SUCCESS;
}


/*--------------------------------------------------------------------------*/
/* TODO: refactor: Linear search in task table might be slow at some point in time!   */
/*--------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_descriptorTable_getTaskDescriptor
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_exec_taskDescriptor_t** taskDesc
)
{
        xme_core_exec_taskDescriptor_t* schedRecord = NULL;
        bool found=false;

        /* We only deal with an initialized component */
        if (!xme_core_exec_isInitialized())
                return XME_STATUS_UNEXPECTED;

        XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "getTaskDescriptor(): (%d:%d)\n", componentId, functionId);

        xme_hal_sync_enterCriticalSection(taskDescriptorsMutex);
        {
            XME_HAL_TABLE_ITERATE_BEGIN(taskDescriptorsTable, uint32_t, i, xme_core_exec_taskDescriptor_t, schedRecord1);
            {
                if((schedRecord1->wrapper->componentId == componentId) && (schedRecord1->wrapper->functionId == functionId))
                {
                    schedRecord=schedRecord1;
                    found=true;
                    break;
                }
            }
            XME_HAL_TABLE_ITERATE_END();
        }
        xme_hal_sync_leaveCriticalSection(taskDescriptorsMutex);

        XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "/getTaskDescriptor(): (%d:%d)\n", componentId, functionId);

        if(found)
        {
            *taskDesc = schedRecord;
            return XME_STATUS_SUCCESS;
        }
        else
        {
            XME_LOG(XME_LOG_VERBOSE,
                MODULE_ACRONYM "[%d|%d] not found in task table!\n", componentId, functionId);
            *taskDesc = NULL;
            return XME_STATUS_NOT_FOUND;
        }
}

/*--------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_dispatcher_setRunnable
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_hal_sched_taskHandle_t handle
)
{
    xme_core_exec_taskDescriptor_t* taskRecord;
    xme_status_t status;

    /* We only deal with an initialized component */
    if (!xme_core_exec_isInitialized())
            return XME_STATUS_UNEXPECTED;

    XME_CHECK(XME_STATUS_SUCCESS == (status=xme_core_exec_descriptorTable_getTaskDescriptor(componentId,
                                                                                    functionId,
                                                                                    &taskRecord)),
        status);

    xme_hal_sync_enterCriticalSection(taskDescriptorsMutex);

    taskRecord->handle = handle;

    xme_hal_sync_leaveCriticalSection(taskDescriptorsMutex);
    return XME_STATUS_SUCCESS;
}

/*--------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_dispatcher_getRunnable
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_hal_sched_taskHandle_t* taskHandle
)
{
    xme_core_exec_taskDescriptor_t* taskRecord = NULL;
    xme_status_t status;

    XME_CHECK(XME_STATUS_SUCCESS == (status = xme_core_exec_descriptorTable_getTaskDescriptor(componentId,
                                                                                    functionId,
                                                                                    &taskRecord)),
            status);

    *taskHandle = taskRecord->handle;
    return XME_STATUS_SUCCESS;
}

/*--------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_getTaskState
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_exec_functionState_t* taskState
)
{
    xme_core_exec_taskDescriptor_t* task = NULL;
    xme_status_t status;

    XME_CHECK_MSG(XME_STATUS_SUCCESS == (status = xme_core_exec_descriptorTable_getTaskDescriptor(componentId,
                                                                                                  functionId,
                                                                                                  &task)),
              status,
              XME_LOG_WARNING,
              MODULE_ACRONYM "error getting task descriptor for [%d|%d]\n",
              componentId, functionId);

    XME_CHECK(NULL != task,
              XME_STATUS_INTERNAL_ERROR);

    /* todo  Is locking while reading such a thing necessary? */
    xme_hal_sync_enterCriticalSection(taskDescriptorsMutex);
    *taskState = task->state;
    xme_hal_sync_leaveCriticalSection(taskDescriptorsMutex);

    return XME_STATUS_SUCCESS;
}

/*--------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_setTaskState
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_exec_functionState_t newTaskState
)
{
    xme_core_exec_taskDescriptor_t* task = NULL;

    XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_descriptorTable_getTaskDescriptor(componentId, functionId, &task),
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_WARNING,
        MODULE_ACRONYM "error getting task descriptor for [%d|%d]\n",
        componentId, functionId);

    XME_CHECK(NULL != task,
              XME_STATUS_INTERNAL_ERROR);

    xme_hal_sync_enterCriticalSection(taskDescriptorsMutex);
    task->state = newTaskState;
    xme_hal_sync_leaveCriticalSection(taskDescriptorsMutex);
    return XME_STATUS_SUCCESS;
}

/*--------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_dispatcher_createFunctionExecutionUnit
(
    xme_core_exec_functionDescriptor_t* functionDescriptor,
    bool eventTriggeredBehavior
)
{
    xme_core_exec_taskDescriptor_t* taskRecord = NULL;

    if (!xme_core_exec_isInitialized())
            return XME_STATUS_UNEXPECTED;

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "> dispatcher_createFunctionExecutionUnit()\n");

    XME_CHECK_MSG( true == xme_core_exec_isValidFunctionDescriptor(functionDescriptor),
        XME_STATUS_INVALID_PARAMETER,
        XME_LOG_WARNING,
        "error adding a new task descriptor to the descriptor table.\n");

    XME_CHECK_MSG( XME_STATUS_NOT_FOUND == xme_core_exec_descriptorTable_getTaskDescriptor(
                    functionDescriptor->componentId, functionDescriptor->functionId, &taskRecord),
               XME_STATUS_ALREADY_EXIST,
               XME_LOG_WARNING,
               MODULE_ACRONYM "error: the task descriptor exists for [%d|%d]\n",
               functionDescriptor->componentId, functionDescriptor->functionId);

    XME_CHECK_MSG( XME_STATUS_SUCCESS == xme_core_exec_dispatcher_addNewTaskDescriptor(&taskRecord),
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_WARNING,
        "error adding a new task descriptor to the descriptor table.\n");

    XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_initializeTaskDescriptor(taskRecord, functionDescriptor, eventTriggeredBehavior),
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_WARNING,
        "error adding a new task descriptor to the descriptor table.\n");

    XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_startRunnable(functionDescriptor),
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_WARNING,
        "error starting runnable.\n");


    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "< dispatcher_createFunctionExecutionUnit()\n");
    return XME_STATUS_SUCCESS;
}

/*--------------------------------------------------------------------------*/
/* TODO functionality to remove the task record is missing                 */
/*--------------------------------------------------------------------------*/
static xme_status_t
xme_core_exec_dispatcher_addNewTaskDescriptor
(
    xme_core_exec_taskDescriptor_t** record1
)
{
    xme_core_exec_taskDescriptor_t* rec1 = NULL;
    xme_hal_table_rowHandle_t handle;

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "> dispatcher_createTaskRecord()\n");

    xme_hal_sync_enterCriticalSection(taskDescriptorsMutex);
    {
        handle = XME_HAL_TABLE_ADD_ITEM(taskDescriptorsTable);
        XME_CHECK_REC(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle,
            XME_STATUS_INTERNAL_ERROR,
            {
                xme_hal_sync_leaveCriticalSection(taskDescriptorsMutex);
            });

        rec1 = XME_HAL_TABLE_ITEM_FROM_HANDLE(taskDescriptorsTable, (int)handle);
        XME_CHECK_REC(NULL != rec1,
            XME_STATUS_INVALID_HANDLE,
            {
                xme_hal_sync_leaveCriticalSection(taskDescriptorsMutex);
            });
    }
    xme_hal_sync_leaveCriticalSection(taskDescriptorsMutex);

    *record1 = rec1;

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "< dispatcher_createTaskRecord()\n");

    return XME_STATUS_SUCCESS;
}

/*--------------------------------------------------------------------------*/
static xme_status_t
xme_core_exec_dispatcher_initializeTaskDescriptor
(
    xme_core_exec_taskDescriptor_t* taskDescriptor,
    xme_core_exec_functionDescriptor_t* functionDescriptor,
    bool eventTriggered
)
{
    taskDescriptor->execLock = xme_hal_sync_createCriticalSection();
    XME_CHECK( XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != taskDescriptor->execLock,
       XME_STATUS_OUT_OF_RESOURCES);

    taskDescriptor->waitLock = xme_hal_sync_createCriticalSection();
    XME_CHECK_REC( XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != taskDescriptor->waitLock,
       XME_STATUS_OUT_OF_RESOURCES,
       {
           if(XME_STATUS_SUCCESS != xme_hal_sync_destroyCriticalSection(taskDescriptor->execLock))
               XME_LOG(XME_LOG_WARNING, MODULE_ACRONYM "could not destruct execution token while recovering\n");
       });

    taskDescriptor->eventTriggered = eventTriggered;
    taskDescriptor->running = false;
    taskDescriptor->wrapper = functionDescriptor;
    taskDescriptor->state = XME_CORE_EXEC_FUNCTION_STATE_INVALID_STATE;

    XME_LOG(XME_LOG_DEBUG,
       MODULE_ACRONYM "[%d|%d] received a dispatcher structure\n",
       functionDescriptor->componentId,
       functionDescriptor->functionId);

    xme_core_exec_lockMutex("T/m", taskDescriptor->execLock,
       functionDescriptor->componentId,
       functionDescriptor->functionId);

    return XME_STATUS_SUCCESS;
}

/*--------------------------------------------------------------------------*/
static xme_status_t
xme_core_exec_dispatcher_startRunnable
(
        xme_core_exec_functionDescriptor_t* functionDescriptor
)
{
    xme_hal_sched_taskHandle_t taskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;

    XME_CHECK_MSG(XME_STATUS_SUCCESS == createRunnable(functionDescriptor, &taskHandle),
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            MODULE_ACRONYM "could not create a runnable unit!\n");

    XME_CHECK_MSG( XME_STATUS_SUCCESS == xme_core_exec_dispatcher_setRunnable(functionDescriptor->componentId,
                                                                              functionDescriptor->functionId,
                                                                              taskHandle),
                   XME_STATUS_INTERNAL_ERROR,
                   XME_LOG_WARNING,
                   MODULE_ACRONYM "runnable handle could not be stored!\n");

    return XME_STATUS_SUCCESS;
}

