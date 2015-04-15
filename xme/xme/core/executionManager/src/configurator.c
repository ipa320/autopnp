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
 * $Id: configurator.c 5832 2013-11-15 16:16:38Z wiesmueller $
 */

/**
 * \file
 *         Implementation of Execution Manager configurator.
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"

#include "xme/hal/include/mem.h"
#include "xme/core/log.h"

#define MODULE_ACRONYM "ExecMgrCfg: "

//******************************************************************************//
//***   Local variables                                                      ***//
//******************************************************************************//

/* TODO at the moment one transaction only is supported. Could we have multiple? */
static struct
{
    xme_core_exec_transactionId_t transactionId;
    xme_hal_linkedList_descriptor_t* nodeSchedules;
} transactionDescriptor;

static bool initialized = false;

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//

static void
clearTransactionDescriptor( void );

/***************************************************************************/
static xme_status_t
enterTransaction
(
    xme_core_exec_transactionId_t transactionId
);

/***************************************************************************/
static bool
isInitialized( void );

//******************************************************************************//
//***   Implementation                                                       ***//
//******************************************************************************//

/****************************************************************************/
static bool
isInitialized( void )
{
    XME_CHECK_MSG(true == initialized,
        false,
        XME_LOG_WARNING,
        "attempting to execute a function of uninitialized xme_core_exec_configurator component!\n"
    );

    return true;
}

/****************************************************************************/
xme_status_t
xme_core_exec_configurator_init(void)
{
    XME_CHECK_MSG(true != initialized,
           XME_STATUS_ALREADY_EXIST,
           XME_LOG_WARNING,
           "attempting to initialize an already initialized xme_core_exec_configurator component!\n");

    transactionDescriptor.transactionId = XME_CORE_EXEC_TRANSACTION_ID_INVALID;
    transactionDescriptor.nodeSchedules = xme_hal_mem_alloc(sizeof(xme_hal_linkedList_descriptor_t));
    XME_HAL_SINGLYLINKEDLIST_INIT(*(transactionDescriptor.nodeSchedules));
    initialized = true;
    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_configurator_fini(void)
{
    XME_CHECK(true == isInitialized(), XME_STATUS_UNEXPECTED);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*(transactionDescriptor.nodeSchedules),
        xme_core_exec_schedule_table_t,
        schedule);
    {
        xme_hal_mem_free(schedule);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    initialized = false;
    XME_HAL_SINGLYLINKEDLIST_FINI(*(transactionDescriptor.nodeSchedules));
    xme_hal_mem_free(transactionDescriptor.nodeSchedules);
    transactionDescriptor.transactionId = XME_CORE_EXEC_TRANSACTION_ID_INVALID;

    return XME_STATUS_SUCCESS;
}


/****************************************************************************/
xme_status_t
xme_core_exec_configurator_commit
(
    xme_core_exec_transactionId_t transactionId
)
{
    XME_CHECK(true == isInitialized(), XME_STATUS_UNEXPECTED);

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "commit transaction %d\n", transactionId);

    XME_CHECK_MSG_REC( transactionDescriptor.transactionId == transactionId,
        XME_STATUS_INVALID_PARAMETER,
        {
            if(XME_STATUS_SUCCESS != xme_core_exec_configurator_rollback(transactionDescriptor.transactionId))
            {
                XME_LOG(XME_LOG_WARNING, MODULE_ACRONYM "could not roll back the transaction!\n");
            }
        },
        XME_LOG_WARNING,
        MODULE_ACRONYM "commitTransaction: Invalid transaction ID: %d, active transaction: %d. Force rollback.\n",
        transactionId, transactionDescriptor.transactionId
    );

    XME_CHECK_MSG( XME_STATUS_SUCCESS ==
            xme_core_exec_scheduler_overwriteFullScheduleSet(transactionDescriptor.nodeSchedules),
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            "commitTransaction: could not overwrite schedule list!\n"
    );

    clearTransactionDescriptor();

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_configurator_rollback
(
    xme_core_exec_transactionId_t transactionId
)
{
    XME_CHECK(true == isInitialized(), XME_STATUS_UNEXPECTED);

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "rollback transaction %d\n", transactionId);

    if(transactionId != transactionDescriptor.transactionId)
        XME_LOG(XME_LOG_WARNING,
                MODULE_ACRONYM "startTransaction: another transaction is active!\n");

    clearTransactionDescriptor();
    return XME_STATUS_SUCCESS;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
static void
clearTransactionDescriptor( void )
{
    transactionDescriptor.transactionId = XME_CORE_EXEC_TRANSACTION_ID_INVALID;

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*(transactionDescriptor.nodeSchedules),
        xme_core_exec_schedule_table_t,
        schedule);
    {
        xme_hal_mem_free(schedule);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_clear(transactionDescriptor.nodeSchedules),
                  XME_CHECK_RVAL_VOID,
                  XME_LOG_WARNING,
                  MODULE_ACRONYM "could not clear the transaction descriptor's schedule set!\n"
                  );
}

/****************************************************************************/
/* XXX VR: refactor */
xme_status_t
xme_core_exec_configurator_addComponentToSchedule
(
        xme_core_exec_transactionId_t transactionId,
        xme_core_component_t componentId,
        xme_core_component_functionId_t functionId,
        void* functionArgs,
        xme_hal_time_timeInterval_t afterTime_ns,
        xme_hal_time_timeInterval_t beforeTime_ns,
        uint32_t periodDivider,
    	uint32_t periodDividerOffset,
        xme_hal_linkedList_descriptor_t* affectedScheduleHandles
)
{
    xme_status_t status;
    bool singleScheduleMode = false;

    XME_CHECK(true == isInitialized(), XME_STATUS_UNEXPECTED);

    XME_CHECK_MSG( XME_STATUS_SUCCESS ==
            enterTransaction(transactionId),
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_WARNING,
            MODULE_ACRONYM "Could not enter a transaction!");

    if((xme_hal_time_timeInterval_t)0 == beforeTime_ns)
    {
        xme_core_exec_schedule_table_t* currentSchedule;
        xme_core_exec_schedule_handle_t currentScheduleHandle;
        currentScheduleHandle = xme_core_exec_scheduler_getCurrentScheduleHandle();
        XME_CHECK(XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID != currentScheduleHandle, XME_STATUS_NOT_FOUND);
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_scheduler_getSchedule(currentScheduleHandle, &currentSchedule),
            XME_STATUS_NOT_FOUND);
        beforeTime_ns = currentSchedule->majorCycleDuration_ns;
    }

    /* NULL is used to treat only the current schedule */
    if(NULL == affectedScheduleHandles)
    {
        xme_core_exec_schedule_handle_t* currentScheduleHandle = xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_handle_t));
        singleScheduleMode = true;
        XME_CHECK(NULL != currentScheduleHandle,
                  XME_STATUS_INTERNAL_ERROR);
        *currentScheduleHandle = xme_core_exec_scheduler_getCurrentScheduleHandle();

        affectedScheduleHandles = (xme_hal_linkedList_descriptor_t*)xme_hal_mem_alloc(sizeof(xme_hal_linkedList_descriptor_t));
        XME_HAL_SINGLYLINKEDLIST_INIT(*affectedScheduleHandles);

        XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(*affectedScheduleHandles, currentScheduleHandle),
                  XME_STATUS_INTERNAL_ERROR);
    }

    status = xme_core_exec_scheduler_addFunctionToScheduleSet(
                    transactionDescriptor.nodeSchedules,
                    affectedScheduleHandles,
                    componentId,
                    functionId,
                    functionArgs,
                    afterTime_ns,
                    beforeTime_ns,
                    periodDivider,
                	periodDividerOffset);

    if(XME_STATUS_SUCCESS != status)
    {
        if(singleScheduleMode)
        {
            xme_hal_mem_free((xme_core_exec_schedule_handle_t*)XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(*affectedScheduleHandles,0));
            XME_HAL_SINGLYLINKEDLIST_FINI(*affectedScheduleHandles);
            xme_hal_mem_free(affectedScheduleHandles);
        }
        XME_LOG(XME_LOG_WARNING, "addFunctionToScheduleSet() returned %d\n",
                status);
        return XME_STATUS_INVALID_CONFIGURATION;
    }

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "completed addToSchedule() with status = %d\n",
            status);

    return status;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
static xme_status_t
enterTransaction
(
    xme_core_exec_transactionId_t transactionId
)
{
    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "entering transaction %d\n", transactionId);
    XME_CHECK_MSG(
        (transactionId == transactionDescriptor.transactionId)
        ||
        (XME_CORE_EXEC_TRANSACTION_ID_INVALID == transactionDescriptor.transactionId),
        XME_STATUS_INVALID_PARAMETER,
        XME_LOG_ERROR,
        MODULE_ACRONYM "enterTransaction %d: another transaction %d is active!\n",
        transactionId,
        transactionDescriptor.transactionId);



    if(XME_CORE_EXEC_TRANSACTION_ID_INVALID == transactionDescriptor.transactionId)
    {
        /* get the schedule set */
        xme_hal_linkedList_descriptor_t* currentScheduleSet = NULL;

        clearTransactionDescriptor();

        XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_scheduler_getScheduleSetPointer(&currentScheduleSet),
                  XME_STATUS_INTERNAL_ERROR);

        XME_CHECK_MSG(XME_STATUS_SUCCESS ==
            xme_core_exec_scheduler_replicateFullScheduleSet(
                    currentScheduleSet,
                    transactionDescriptor.nodeSchedules),
            XME_STATUS_OUT_OF_RESOURCES,
            XME_LOG_ERROR,
            MODULE_ACRONYM "enterTransaction %d: Problem when making a schedule copy\n",
            transactionId);

        transactionDescriptor.transactionId = transactionId;
        XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "entered transaction %d\n", transactionId);
    }
    else
    {
        XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "in transaction %d\n", transactionId);
    }

    return XME_STATUS_SUCCESS;
}
