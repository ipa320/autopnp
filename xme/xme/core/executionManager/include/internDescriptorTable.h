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
 * $Id: internDescriptorTable.h 6250 2013-12-20 18:16:10Z ruiz $
 */

/**
 * \file
 *         Execution Manager descriptor table functions (internal).
 */

#ifndef INTERNDESCRIPTORTABLE_H_
#define INTERNDESCRIPTORTABLE_H_

/****************************************************************************/
/**   Includes                                                             **/
/****************************************************************************/
/* Component-specific data */
#include "xme/core/executionManager/include/executionManagerDataStructures.h"

/* Chromosome HAL */
#include "xme/hal/include/sync.h"
#include "xme/hal/include/sched.h"
#include "xme/hal/include/time.h"

/* Chromosome metalanguage */
#include "xme/core/component.h"
#include "xme/defines.h"

/*
 * If a doxygen group is required for internal functions, please create it. At the moment there is none.
 */

/****************************************************************************/
/**   Type definitions                                                     **/
/****************************************************************************/
/*-------------------------------------------------------------------------*/
/**
 * \struct xme_core_exec_taskDescriptor_t
 * \brief contains supporting information to keep a function running and controlled.
 */
typedef struct xme_core_exec_task_descriptor_
{
    xme_core_exec_functionDescriptor_t* wrapper; ///< Pointer to the function wrapper structure
    xme_hal_sched_taskHandle_t handle; ///< A handle to the thread/task object allocated for the function execution
    xme_hal_sync_criticalSectionHandle_t execLock; ///< The gate for enabling / locking function execution
    xme_hal_sync_criticalSectionHandle_t waitLock; ///< The gate for waiting / inverse locking function execution

    bool eventTriggered; ///< Indicates, whether a function will be "gated" by executionReady() calls
    bool running; ///< identifies if the function is being executed at the moment (execLock?)
    xme_core_exec_functionState_t state;
    xme_hal_time_timeHandle_t startTime; ///< Timestamp of start for monitoring
} xme_core_exec_taskDescriptor_t;


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN
/*-------------------------------------------------------------------------*/
/**
 * \brief Sets up the descriptor table
 *
 * \returns Returns XME standard status:
 * - XME_STATUS_INTERNAL_ERROR  if unable to initialize table mutex
 * - XME_STATUS_SUCCESS         if no error has occured
 */
xme_status_t
xme_core_exec_descriptorTable_init( void );

/*-------------------------------------------------------------------------*/
/**
 * \brief Finalizes and cleans up the descriptor table
 *
 * \returns Returns XME standard status:
  * - XME_STATUS_SUCCESS         if no error has occured
 */
xme_status_t
xme_core_exec_descriptorTable_fini( void );

/*-------------------------------------------------------------------------*/
/**
 * \brief Find a task in descriptor table and return it
 *
 * \param[in]   componentId     ID of the component
 * \param[in]   functionId      ID of the function
 * \param[out]  taskDesc        descriptor of the corresponding task
 *
 *     odo Usage of this function in this form could lead to concurrent modification
 * \returns Returns XME standard status:
  * - XME_STATUS_SUCCESS         if no error has occured
 */
// fixme: thread safety of usage
xme_status_t
xme_core_exec_descriptorTable_getTaskDescriptor
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_exec_taskDescriptor_t** taskDesc
);

/*-------------------------------------------------------------------------*/
/**
 * \brief Perform an operation for all descriptor table elements
 *
 * \param[in]   fun     callback function to execute on each element
 *
 * \returns Returns XME standard status:
 * - XME_STATUS_SUCCESS             if no error has occured
 * - XME_STATUS_INVALID_PARAMETER   if the callback function is NULL
 */
xme_status_t
xme_core_exec_descriptorTable_forEach
(
    void (*fun)(xme_core_exec_taskDescriptor_t*)
);

/*-------------------------------------------------------------------------*/
/**
 * \brief Sets the task handle
 *
 * \param[in]   componentId     ID of the component
 * \param[in]   functionId      ID of the function
 * \param[in]   handle          runnable handle to be set
 *
 * \returns Returns XME standard status:
 * - XME_STATUS_SUCCESS     if no error has occured
 * - XME_STATUS_NOT_FOUND   if no descriptor found for a function
 * - XME_STATUS_UNEXPECTED  if attempting to operate on an uninitialized component
 *
 */
extern xme_status_t
xme_core_exec_dispatcher_setRunnable
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_hal_sched_taskHandle_t handle
);

/*-------------------------------------------------------------------------*/
/**
 * \brief Retrieves the corresponding task handle for a function
 *
 * \param[in]   componentId     ID of the component
 * \param[in]   functionId      ID of the function
 * \param[out]  handle          runnable handle
 *
 * \returns Returns XME standard status:
 * - XME_STATUS_SUCCESS     if no error has occured
 * - XME_STATUS_NOT_FOUND   if no descriptor found for a function
 * - XME_STATUS_UNEXPECTED  if attempting to operate on an uninitialized component
 */
extern xme_status_t
xme_core_exec_dispatcher_getRunnable
(
        xme_core_component_t componentId,
        xme_core_component_functionId_t functionId,
        xme_hal_sched_taskHandle_t*
);

/*-------------------------------------------------------------------------*/
/**
 * \brief Updates the state of the function
 *
 * \param[in]   componentId     ID of the component
 * \param[in]   functionId      ID of the function
 * \param[in]   newTaskState    new state of the task to be set
 *
 * \returns Returns one of the following:
 *  - XME_STATUS_SUCCESS in case of success,
 *  - XME_STATUS_NOT_FOUND if no task exists with the specified IDs,
 *  - XME_STATUS_INTERNAL_ERROR if other errors occur.
 *
 */
extern xme_status_t
xme_core_exec_setTaskState
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_exec_functionState_t newTaskState
);

/*-------------------------------------------------------------------------*/
/**
 * \brief Retrieves the state of the function
 *
 * \param[in]   componentId     ID of the component
 * \param[in]   functionId      ID of the function
 * \param[out]   taskState      state of the task
 *
 * \returns Returns one of the following:
 *  - XME_STATUS_SUCCESS in case of success,
 *  - XME_STATUS_NOT_FOUND if no task exists with the specified IDs,
 *  - XME_STATUS_INTERNAL_ERROR if other errors occur.
 */
extern xme_status_t
xme_core_exec_getTaskState
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_exec_functionState_t* taskState
);
XME_EXTERN_C_END
/*
 *
 */

#endif /* INTERNDESCRIPTORTABLE_H_ */
