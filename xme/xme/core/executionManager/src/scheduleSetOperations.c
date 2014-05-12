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
 * $Id: scheduleSetOperations.c 5157 2013-09-24 16:29:43Z rupanov $
 */

/**
 * \file
 *         Execution Manager schedule set operations.
 */

#define MODULE_ACRONYM "ExecMgr   : "

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/log.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_exec_scheduler_replicateFullScheduleSet
(
    xme_hal_linkedList_descriptor_t* source,
    xme_hal_linkedList_descriptor_t* target
)
{
    int replicaNumber = 0;
    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "Replicating active schedule set\n");

    XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM"items in schedule set:%d\n",
            xme_hal_singlyLinkedList_getItemCount(source));

    /* Iterate through the initial list and push item copies to the new list */
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            *source,
            xme_core_exec_schedule_table_t,
            srcSchedule);
    {
        xme_core_exec_schedule_table_t* newSchedule = NULL;

        /*lint -save -e578 */
        XME_CHECK_MSG_REC( XME_STATUS_SUCCESS == xme_core_exec_scheduler_replicateSchedule(&newSchedule, srcSchedule),
            XME_STATUS_OUT_OF_RESOURCES,
            {
                    /* Leave target empty in case of problems */
                    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*target, xme_core_exec_schedule_table_t, scheduleX);
                        if(XME_STATUS_SUCCESS != xme_core_exec_scheduler_clearScheduleTable(&scheduleX, (bool) false))
                        {
                            XME_LOG(XME_LOG_WARNING, MODULE_ACRONYM "could not clear the table on error!\n");
                        }

                    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

                    if(XME_STATUS_SUCCESS != xme_hal_singlyLinkedList_clear(target))
                        XME_LOG(XME_LOG_ERROR, MODULE_ACRONYM "could not clear the target schedule set!");
            },
            XME_LOG_WARNING,
            "Problem replicating schedule %d\n",
            replicaNumber
        );
        /*lint -restore */

        xme_core_exec_scheduler_printSchedule(srcSchedule, "replicating");

        XME_LOG(XME_LOG_DEBUG, "replica number %d\n", replicaNumber);

        if(newSchedule != NULL)
        {
            /*lint -save -e578 */
            XME_CHECK_MSG_REC(XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem((void*)(target), newSchedule),
                    XME_STATUS_INTERNAL_ERROR,
                    {
                            /* Leave target empty in case of problems */
                            XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*target, xme_core_exec_schedule_table_t, scheduleX);
                            if(XME_STATUS_SUCCESS != xme_core_exec_scheduler_clearScheduleTable(&scheduleX, (bool) false))
                            {
                                XME_LOG(XME_LOG_WARNING, MODULE_ACRONYM "could not clear the table on error!\n");
                            }
                            XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
                            XME_HAL_SINGLYLINKEDLIST_FINI(*target);
                            XME_HAL_SINGLYLINKEDLIST_INIT(*target);
                    },
                    XME_LOG_WARNING,
                    "problem adding items!\n"
            );
            /*lint -restore */
        }
        else
        {
            XME_LOG(XME_LOG_ERROR, "replicateSchedule() returned NULL!\n");
            return XME_STATUS_INTERNAL_ERROR;
        }

        replicaNumber++;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "Completed replication of schedule set.\n");

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_overwriteFullScheduleSet
(
    xme_hal_linkedList_descriptor_t* source
)
{
    xme_hal_linkedList_descriptor_t* nodeScheduleSet;

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_scheduler_getScheduleSetPointer(&nodeScheduleSet),
              XME_STATUS_INTERNAL_ERROR);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*(nodeScheduleSet), xme_core_exec_schedule_table_t, schedule);
        xme_hal_mem_free(schedule);
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_clear((void*)(nodeScheduleSet)),
              XME_STATUS_INTERNAL_ERROR);

    return xme_core_exec_scheduler_replicateFullScheduleSet(source, nodeScheduleSet);
}

/*----------------------------------------------------------------------------*/
xme_status_t
xme_core_exec_scheduler_addFunctionToScheduleSet
(
        xme_hal_linkedList_descriptor_t* scheduleSet,
        xme_hal_linkedList_descriptor_t* scheduleIds,
        xme_core_component_t componentId,
        xme_core_component_functionId_t functionId,
        void* functionArgs,
        xme_hal_time_timeInterval_t fromTime_ns,
        xme_hal_time_timeInterval_t toTime_ns,
        uint32_t periodDivider,
    	uint32_t periodDividerOffset
)
{
    int index0 = 0;
    xme_hal_time_timeInterval_t wcet=0;
    xme_hal_time_timeInterval_t startTime_ns;

    xme_core_exec_schedule_table_t* localSchedule = NULL;
    xme_core_exec_functionDescriptor_t* fdesc = NULL;

    XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_componentRepository_getFunction(componentId, functionId, &fdesc),
            XME_STATUS_NO_SUCH_VALUE,
            XME_LOG_WARNING,
            MODULE_ACRONYM "no function exists: [%d|%d]\n",
            componentId, functionId);

    wcet = fdesc->wcet_ns;

    //todo: xme_check: scheduleIds is not empty

    index0 =  (int)(*((xme_core_exec_schedule_handle_t*)xme_hal_singlyLinkedList_itemFromIndex(scheduleIds, 0)));

    localSchedule = (xme_core_exec_schedule_table_t*) xme_hal_singlyLinkedList_itemFromIndex(scheduleSet, index0);

    XME_CHECK_MSG_REC( XME_STATUS_SUCCESS ==
        xme_core_exec_scheduler_findWindow(
            localSchedule,
            wcet,
            fromTime_ns,
            toTime_ns,
            &startTime_ns
        ),
        XME_STATUS_INTERNAL_ERROR,
        {xme_core_exec_scheduler_printSchedule(localSchedule, "failed schedule");},
        XME_LOG_WARNING,
        MODULE_ACRONYM "findWindow failed on schedule %d!\n",
        index0
    );

    /* First loop to think of validity of insert */
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(*scheduleIds, xme_core_exec_schedule_handle_t, schedIndex);

        localSchedule = (xme_core_exec_schedule_table_t*) xme_hal_singlyLinkedList_itemFromIndex(scheduleSet, (int)(*schedIndex));

        /* This should fail the test */
        XME_CHECK_MSG( XME_STATUS_SUCCESS ==
            xme_core_exec_scheduler_addElementToScheduleTable(
                localSchedule,
                componentId,
                functionId,
                functionArgs,
                startTime_ns,
                wcet,
                periodDivider,
                periodDividerOffset,
                (bool)true),
            XME_STATUS_OUT_OF_RESOURCES,
            XME_LOG_WARNING,
            MODULE_ACRONYM "Could not add to schedule #%d\n",
            *schedIndex
        );
        xme_core_exec_scheduler_printSchedule(localSchedule, "After adding function:");

    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

