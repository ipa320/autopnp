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
 * $Id: executionManagerDataStructures.h 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Execution Manager data structures
 */

#ifndef XME_CORE_EXEC_DATA_STRUCTURES_H
#define XME_CORE_EXEC_DATA_STRUCTURES_H

/**
 * \ingroup core_em Execution Manager
 * @{
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/hal/include/linkedList.h"
#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/core/executionManagerCallback.h"

#include <stdbool.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/* EM specific callback */
typedef xme_status_t (*xme_core_exec_functionSpecificCallback_t)
                (xme_core_component_t cid, xme_core_component_functionId_t fid);

/*-------------------------------------------------------------------------*/
/**
 * \struct xme_core_exec_configStruct_t
 * \brief Config structure for executionManager
 */
typedef struct xme_core_exec_configStruct_
{
    xme_core_exec_initCallback_t onEndOfCycle;
    xme_core_exec_functionSpecificCallback_t onFunctionActivate;
    xme_core_exec_functionSpecificCallback_t onFunctionReturned;
    bool useDbCycleCounter;
}
xme_core_exec_configStruct_t;

/*-------------------------------------------------------------------------*/
/**
 * \typedef xme_core_exec_dataReadyCallback_t
 * \brief Callback function type for component wrapper functions that resolve
 *          data dependencies in IL1.
 */
typedef bool (*xme_core_exec_dataReadyCallback_t) (xme_core_dataManager_dataPacketId_t);

/*-------------------------------------------------------------------------*/
/**
 * \typedef xme_core_exec_functionState_t
 * \brief Status of the function execution.
 *
 *     odo this is a *dirty hack*, should be hidden and presented through a
 *          dedicated dispatcher interface
 */
typedef enum xme_core_exec_functionState_
{
    XME_CORE_EXEC_FUNCTION_STATE_INVALID_STATE = 0,   ///< Incorrect state, not initialized
    XME_CORE_EXEC_FUNCTION_STATE_TERMINATED = 1,      ///< Terminated/terminating
    XME_CORE_EXEC_FUNCTION_STATE_RUNNING = 2,         ///< Running - normal mode
    XME_CORE_EXEC_FUNCTION_STATE_PAUSED = 3           ///< Paused - execution unit exists, but does not run
}
xme_core_exec_functionState_t;

/**
 * \struct xme_core_exec_functionDescriptor_t
 * \brief Defines a function within a component with all function-specific parameters.
 *
 *     odo The story with state sounds like a hack.
 *     odo dataReady() is an IL1-only hack for Chromosome only.
 *     odo initialization / function start infrastructure in XME execution manager
 */
typedef struct xme_core_exec_functionDescriptor_s
{
    xme_core_component_t componentId; ///< Identifies the component
    xme_core_component_functionId_t functionId; ///< Locally unique identifier of function within component
    xme_hal_sched_taskCallback_t task;  ///< Task callback function to be executed by the dispatcher

    void* taskArgs; ///< For component functions, pointer to the configuration of the function's parent component instance. For waypoint functions, pointer to the function descriptor.
    xme_hal_time_timeInterval_t wcet_ns;
    xme_core_exec_functionState_t state;

    xme_core_exec_initCallback_t init;            ///< function initialization function callback
    xme_core_exec_finiCallback_t fini;            ///< function finalization function callback
    xme_hal_time_timeInterval_t initWcet_ns;    ///< Worst case execution time for the initialization function
}
xme_core_exec_functionDescriptor_t;

/*-------------------------------------------------------------------------*/
/**
 * \struct xme_core_exec_componentDescriptor_t
 * \brief Defines a component with all component-global parameters like initialization and function list.
 */
typedef struct xme_core_exec_componentDescriptor_s
{
    xme_core_component_t componentId;             ///< Identifies the component
    bool autoInit;                              ///< Auto-initialize during startup? typically true
    uint32_t initPriority;                      ///< Priority for initialization
    xme_core_exec_initCallback_t init;            ///< Component initialization function callback
    // TODO: Adapt initParam to new schema as specified in xme_core_exec_functionDescriptor_t above (it should not only be used for initialization, but also for finalization)!
    void* initParam;                            ///< Component initialization parameter
    xme_core_exec_finiCallback_t fini;            ///< Component finalization function callback
    xme_hal_time_timeInterval_t initWcet_ns;    ///< Worst case execution time for the initialization function
    xme_hal_linkedList_descriptor_t functions; ///< The set of functions for this component
}
xme_core_exec_componentDescriptor_t;

/**
 * @}
 */

#endif
