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
 * $Id: channelSelectorWayPointFunctionWrapper.c 6684 2014-02-07 14:34:26Z geisinger $
 */

/**
 * \file
 *         Function wrapper - a generic abstraction for one executable function
 *              scheduled by the execution manager.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/wp/channel/include/channelSelectorWayPointFunctionWrapper.h"

#include "xme/wp/channel/include/channelSelector.h"

#include "xme/core/executionManager/include/executionManagerWrapperInterface.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/core/log.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
xme_wp_channel_channelSelector_channelSelectorWayPointFunctionWrapper_execute
(
    void* userFunctionDescRaw
)
{
    xme_status_t status;
    xme_core_exec_functionDescriptor_t* funDesc;
    xme_core_component_functionVariantId_t functionVariantId;

    funDesc = (xme_core_exec_functionDescriptor_t*)userFunctionDescRaw;

    status = xme_core_exec_dispatcher_initializeTask
    (
        funDesc->componentId, 
        funDesc->functionId
    );
    XME_CHECK_MSG_REC
    (
        XME_STATUS_SUCCESS == status,
        XME_CHECK_RVAL_VOID,
        {
            funDesc->state = XME_CORE_EXEC_FUNCTION_STATE_INVALID_STATE;
        },
        XME_LOG_FATAL,
        "Dispatcher [%d|%d] initialization returned an error code %d\n",
        funDesc->componentId,
        funDesc->functionId,
        status
    );

    if (NULL != funDesc->init)
    {
        status = funDesc->init(funDesc->taskArgs);
        XME_CHECK_MSG_REC
        (
            XME_STATUS_SUCCESS == status,
            XME_CHECK_RVAL_VOID,
            {
                funDesc->state = XME_CORE_EXEC_FUNCTION_STATE_INVALID_STATE;
            },
            XME_LOG_FATAL,
            "Function [%d|%d] initialization returned an error code %d\n",
            funDesc->componentId,
            funDesc->functionId,
            status
        );
    }

    while(XME_CORE_EXEC_FUNCTION_STATE_TERMINATED != funDesc->state)
    {
        status = xme_core_exec_dispatcher_waitForStart
        (
            funDesc->componentId,
            funDesc->functionId,
            &functionVariantId
        );
        if (XME_STATUS_SUCCESS != status)
        {
            XME_LOG
            (
                XME_LOG_FATAL,
                "Dispatcher_waitForStart failed for function [%d|%d] returned an error code %d\n",
                funDesc->componentId,
                funDesc->functionId,
                status
            );
            funDesc->state = XME_CORE_EXEC_FUNCTION_STATE_INVALID_STATE;
            break;
        }

        if (XME_CORE_EXEC_FUNCTION_STATE_PAUSED != funDesc->state)
        {
            (void) xme_wp_channel_channelSelector_run((xme_wp_waypoint_instanceId_t)(uintptr_t)functionVariantId);
        }

        status = xme_core_exec_dispatcher_executionCompleted
        (
            funDesc->componentId,
            funDesc->functionId
        );
        if (XME_STATUS_SUCCESS != status)
        {
            XME_LOG
            (
                XME_LOG_FATAL,
                "Dispatcher_executionCompleted failed for function [%d|%d] returned an error code %d\n",
                funDesc->componentId,
                funDesc->functionId,
                status
            );
            funDesc->state = XME_CORE_EXEC_FUNCTION_STATE_INVALID_STATE;
            break;
        }
    }  

    if (NULL != funDesc->fini)
    {
        funDesc->fini(funDesc->taskArgs);
    }
}
