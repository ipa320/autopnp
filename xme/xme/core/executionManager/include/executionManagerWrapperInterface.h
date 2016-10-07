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
 * $Id: executionManagerWrapperInterface.h 4731 2013-08-22 09:23:17Z rupanov $
 */

/**
 * \file
 *         Execution Manager wrapper interface.
 */

#ifndef XME_CORE_EXEC_WRAPPER_INTERFACE_H
#define XME_CORE_EXEC_WRAPPER_INTERFACE_H

/**
 * \ingroup core_em Execution Manager
 * @{
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/component.h"

#include "xme/hal/include/linkedList.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Prepare the runnable for execution.
 *
 * Called by the task-thread before function-specific initialization.
 *
 * \param   componentId Locally unique identifier of the component,
 *          to which the function belongs.
 * \param   functionId Identifier of the function within a component.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if task is now paused;
 *          - XME_STATUS_NO_SUCH_VALUE  if no such task exists.
 */
xme_status_t
xme_core_exec_dispatcher_initializeTask
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

/**
 * \brief   Called by the function wrapper to notify that execution has
 *          completed normally, and dispatcher has to start the next function.
 *
 * \note    All the data has to be written by the function or the wrapper
 *          before this call.
 * \param   componentId Locally unique identifier of the component,
 *          to which the function belongs.
 * \param   functionId Identifier of the function within a component.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if completion has been successful;
 *          - XME_STATUS_INTERNAL_ERROR if an error occured.
 */
extern xme_status_t
xme_core_exec_dispatcher_executionCompleted
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

/**
 * \brief   Called by the function wrapper to synchronize and start executing
 *          the task whenever leaving this call.
 *
 * \param   componentId Locally unique identifier of the component,
 *          to which the function belongs.
 * \param   functionId Identifier of the function within a component.
 * \param   functionArgs Address of a variable where to store the function
 *          arguments are passed when creating the schedule.
 *          This argument may be NULL, in which case no function arguments
 *          are returned.
 *
 * \retval  XME_STATUS_SUCCESS if start is allowed;
 * \retval  XME_STATUS_NO_SUCH_VALUE if no such task exists.
 */
extern xme_status_t
xme_core_exec_dispatcher_waitForStart
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void** functionArgs
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif /* #ifdef XME_CORE_EXEC_WRAPPER_INTERFACE_H */
