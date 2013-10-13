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
 * $Id: dispatcherMock.c 4565 2013-08-05 14:15:52Z wiesmueller $
 */

/**
 * \file
 *         A mock of xme_core_exec_dispatcher.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/component.h"

#include <stdbool.h>

XME_EXTERN_C_BEGIN

/******************************************************************************/
/***   Global Variables                                                     ***/
/******************************************************************************/
/**
 * \brief  When this is false the next xme_core_exec_dispatcher_waitForStart
 *         call will return XME_STATUS_SUCCESS. After that it will be set to
 *         true, so that subsequent calls will result in failure (until set
 *         to false again).
 */
bool xme_adv_wrapperTest_dispatcherMock_waitForStartRunOnce = false;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Mocked function. Does nothing.
 */
xme_status_t
xme_core_exec_dispatcher_executionCompleted
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

/**
 * \brief  Mocked function. See description of xme_adv_wrapperTest_dispatcherMock_waitForStartRunOnce.
 */
xme_status_t
xme_core_exec_dispatcher_waitForStart
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void** functionArguments
);

/**
 * \brief  Mocked function. Does nothing.
 */
xme_status_t
xme_core_exec_dispatcher_initializeTask
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_exec_dispatcher_executionCompleted
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);

    // Do nothing

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
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionArguments);

    if (!xme_adv_wrapperTest_dispatcherMock_waitForStartRunOnce) {
        xme_adv_wrapperTest_dispatcherMock_waitForStartRunOnce = true;
        return XME_STATUS_SUCCESS;
    }

    return XME_STATUS_INTERNAL_ERROR; // This will cause the function wrapper to stop its while loop
}

xme_status_t
xme_core_exec_dispatcher_initializeTask
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);

    // Do nothing

    return XME_STATUS_SUCCESS;
}
