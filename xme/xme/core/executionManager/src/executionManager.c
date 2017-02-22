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
 * $Id: executionManager.c 7808 2014-03-13 10:30:27Z geisinger $
 */

/**
 * \file
 *         Execution Manager: implementation.
 */
#define MODULE_ACRONYM "ExecMgr   : "
//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"

#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"

#include "xme/core/broker/include/broker.h"

#include "xme/core/log.h"

#include "xme/hal/include/mem.h"


//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

//******************************************************************************//
//***   Variables                                                            ***//
//******************************************************************************//
xme_core_exec_state_t xme_core_exec_state  = {XME_CORE_EXEC_NOT_INITIALIZED, XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE};
xme_core_exec_configStruct_t xme_core_exec_intConfig = {0};

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
static xme_status_t
xme_core_exec_step
(
    uint32_t maximumCycle     ///< maximum cycle number to run till; infinite if 0
);

static INLINE bool
xme_core_exec_isRunning( void );

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

// XXX: Code Review 6.08.2013: XME_CHECK_MSG promotion
//              + break down to improve readability
// XXX: Code Review 6.08.2013: ASSERT instead of CHECK in init() functions

xme_status_t
xme_core_exec_init
(
	xme_core_exec_configStruct_t* initConfig
)
{
	xme_core_exec_state_value_t state;

    /* Initialize with default behavior */
    xme_core_exec_intConfig.onEndOfCycle = NULL;
    xme_core_exec_intConfig.onFunctionActivate = NULL;
    xme_core_exec_intConfig.onFunctionReturned = NULL;

    if(NULL != initConfig)
    {
        XME_CHECK(NULL != xme_hal_mem_copy(
                    &xme_core_exec_intConfig,
                    initConfig,
                    sizeof(xme_core_exec_configStruct_t)),
                XME_STATUS_OUT_OF_RESOURCES
            );
    }

    state = xme_core_exec_getState();

    XME_CHECK( XME_CORE_EXEC_SHUTDOWN != state,
                XME_STATUS_UNEXPECTED);

    // XXX: Code Review 6.08.2013 hard to understand and read

    XME_CHECK( XME_CORE_EXEC_NOT_INITIALIZED == state,
            XME_STATUS_ALREADY_EXIST);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_scheduler_init(),
        XME_STATUS_INTERNAL_ERROR);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_init(),
        XME_STATUS_INTERNAL_ERROR);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_componentRepository_init(),
        XME_STATUS_INTERNAL_ERROR);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_init(),
        XME_STATUS_INTERNAL_ERROR);

    xme_core_exec_state.lock = xme_hal_sync_createCriticalSection();
    xme_core_exec_setState(XME_CORE_EXEC_PAUSED);

    return XME_STATUS_SUCCESS;
}

//******************************************************************************//

// XXX: Code review 6.8.13 should be ASSERTED instead of CHECK/if
// + possibly change the return type to void
xme_status_t
xme_core_exec_run
(
    uint32_t numCycles,
    bool separateThread
)
{
    if(false != separateThread)
        return XME_STATUS_UNSUPPORTED;

    /* Check if the component has been initialized */
    if (!xme_core_exec_isInitialized())
        return XME_STATUS_UNEXPECTED;

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_scheduler_initExecution(),
        XME_STATUS_INTERNAL_ERROR);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_initExecution(),
        XME_STATUS_INTERNAL_ERROR);

    while(xme_core_exec_isRunning())
    {
        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_step(numCycles),
                  XME_STATUS_INTERNAL_ERROR,
                  XME_LOG_ERROR,
                  MODULE_ACRONYM "error in dispatcher loop!\n");
    }

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
static bool
xme_core_exec_isRunning( void )
{
	xme_core_exec_state_value_t state;
	state = xme_core_exec_getState();
	if ((XME_CORE_EXEC_SHUTDOWN != state)
			&& (XME_CORE_EXEC_NOT_INITIALIZED != state))
	{
		return true;
	}
	return false;
}

/****************************************************************************/
static xme_status_t
xme_core_exec_step
(
    uint32_t maximumCycle     ///< maximum cycle number to run till; infinite if 0
)
{
    // XXX uniform default initialization

    /* Schedule table entry for the next slot */
    xme_core_exec_schedule_table_entry_t* nextSlot;

    /* Current slack */
    xme_hal_time_timeInterval_t slack = 0U;

    /* Ask the scheduler to provide the next component to be executed */
    XME_CHECK(
        XME_STATUS_SUCCESS == xme_core_exec_scheduler_calculateNextComponent(&nextSlot, &slack),
        XME_STATUS_INTERNAL_ERROR
    );

    /* Maximum cycle interface */
    if(xme_core_exec_dispatcher_maximumCycleReached(maximumCycle))
    {
        xme_core_exec_setState(XME_CORE_EXEC_SHUTDOWN);
        return XME_STATUS_SUCCESS;
    }

    return xme_core_exec_dispatcher_executeStep(nextSlot, slack);
}

//******************************************************************************//
//XXX: Code Review: MB change name to something different from onEndOfCycle (callback in config)
xme_status_t
xme_core_exec_stop
(
    bool atEndOfCycle
)
{
    xme_core_exec_state_value_t state;

    if (!xme_core_exec_isInitialized())
            return XME_STATUS_UNEXPECTED;

    // Do nothing if state is already XME_CORE_EXEC_SHUTDOWN
    state = xme_core_exec_getState();
    if (XME_CORE_EXEC_SHUTDOWN == state)
            return XME_STATUS_SUCCESS;

    if(atEndOfCycle)
        xme_core_exec_setState(XME_CORE_EXEC_TO_BE_SHUT_DOWN);
    else
        xme_core_exec_setState(XME_CORE_EXEC_SHUTDOWN);

    return XME_STATUS_SUCCESS;
}


//******************************************************************************//
// XXX change return type to void
xme_status_t
xme_core_exec_fini( void )
{
    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "finalizing execution manager\n");

    if (!xme_core_exec_isInitialized())
        return XME_STATUS_UNEXPECTED;

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_fini(),
            XME_STATUS_INTERNAL_ERROR);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_componentRepository_fini(),
            XME_STATUS_INTERNAL_ERROR);

    // finalization
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_fini(),
        XME_STATUS_INTERNAL_ERROR);


    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_scheduler_fini(),
        XME_STATUS_INTERNAL_ERROR);

    xme_core_exec_setState(XME_CORE_EXEC_NOT_INITIALIZED);
    xme_hal_sync_destroyCriticalSection(xme_core_exec_state.lock);
    xme_core_exec_state.lock = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;

    return XME_STATUS_SUCCESS;
}

//******************************************************************************//

// XXX: Code review: inline's in the whole EM code
bool
xme_core_exec_isInitialized( void )
{
	bool result;

	if(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE == xme_core_exec_state.lock)
		return false;

	xme_hal_sync_enterCriticalSection(xme_core_exec_state.lock);
	result = !(xme_core_exec_state.value == XME_CORE_EXEC_NOT_INITIALIZED);
	xme_hal_sync_leaveCriticalSection(xme_core_exec_state.lock);

    return result;
}


// XXX: Code review: inline's in the whole EM code
void
xme_core_exec_setState
(
	xme_core_exec_state_value_t newState
)
{
	XME_ASSERT_NORVAL(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != xme_core_exec_state.lock);

	xme_hal_sync_enterCriticalSection(xme_core_exec_state.lock);
	xme_core_exec_state.value = newState;
	xme_hal_sync_leaveCriticalSection(xme_core_exec_state.lock);
	XME_LOG(XME_LOG_VERBOSE, "setState %d\n", (uint32_t)newState);
}

xme_core_exec_state_value_t
xme_core_exec_getState( void )
{
	xme_core_exec_state_value_t result;
	if(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE == xme_core_exec_state.lock)
			return XME_CORE_EXEC_NOT_INITIALIZED;

	xme_hal_sync_enterCriticalSection(xme_core_exec_state.lock);
	result = xme_core_exec_state.value;
	xme_hal_sync_leaveCriticalSection(xme_core_exec_state.lock);
	return result;
}
