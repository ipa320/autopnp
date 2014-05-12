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
 * $Id: loginManagerExecutionManagerMock.c 4431 2013-07-31 08:24:55Z ruiz $
 */

/**
 * \file
 *
 * \brief  A mock of xme_core_dataHandler, to be used with the broker
 *         for testing xme_core_broker_notifyOutputDataPacketAvailability.
 *
 * \note This mock will use the following functions:
 *       * DataHandler's completeWriteOperation(). This function will call broker's 
 *         notifyOutputDataPacketAvailability.
 *       * DataHandler's transferData(). This is called directly from the broker
 *         notifyOutputDataPacketAvailability (currently static call). Several
 *         responses to implement. 
 *       * DataHandler completeReadOperation(). This function will "unblock" the port. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/executionManager/include/executionManagerWrapperInterface.h"
#include "xme/core/log.h"

#include "xme/core/topicData.h"

#include "xme/hal/include/mem.h"

#include <stdbool.h>

XME_EXTERN_C_BEGIN

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_exec_dispatcher_initializeTask
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_dispatcher_executionCompleted
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);

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

    return XME_STATUS_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////

XME_EXTERN_C_END
