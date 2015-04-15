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
 * $Id: taskTimer.c 6092 2013-12-13 12:55:58Z gulati $
 */

/**
 * \file
 *         Runnable: an executable abstraction.
 */

// #define MODULE_ACRONYM "ExecMgrRbl: "

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/log.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
createRunnable
(
    xme_core_exec_functionDescriptor_t* functionWrapper,
    xme_hal_sched_taskHandle_t* newRunnable
)
{
    *newRunnable = xme_hal_sched_addTask
    (
        0ULL, // start time
        0ULL, // period
        0U, // priority
        (xme_hal_sched_taskCallback_t)functionWrapper->task,    // callback
        ((void *)functionWrapper)   // arguments
    );

    if(XME_HAL_SCHED_INVALID_TASK_HANDLE == *newRunnable)
    {
        return XME_STATUS_INTERNAL_ERROR;
    }

    return XME_STATUS_SUCCESS;
}
/* issue #2878 */
#if 0
/*****************************************************************************/
xme_status_t
destroyRunnable
(
    xme_hal_sched_taskHandle_t runnable
)
{
    return xme_hal_sched_removeTask(runnable);
}
#endif

