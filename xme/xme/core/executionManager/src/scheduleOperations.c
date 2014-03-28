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
 * $Id: scheduleOperations.c 5572 2013-10-22 17:29:28Z geisinger $
 */

/**
 * \file
 *         Execution Manager schedule operations.
 */

#define MODULE_ACRONYM "ExecMgr   : "

#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/log.h"
#include "xme/hal/include/mem.h"
#include <inttypes.h>

/****************************************************************************/

/** \brief Local helper function to find a window for a newly started function
 *           in the existing schedule.
 */
xme_status_t
xme_core_exec_scheduler_findWindow
(
    xme_core_exec_schedule_table_t* schedule,
    xme_hal_time_timeInterval_t wcet_ns,
    xme_hal_time_timeInterval_t afterTime_ns,
    xme_hal_time_timeInterval_t beforeTime_ns,
    xme_hal_time_timeInterval_t* windowStart_ns
);

static INLINE xme_hal_time_timeInterval_t
max_time
(
    xme_hal_time_timeInterval_t t1,
    xme_hal_time_timeInterval_t t2
);

static INLINE xme_hal_time_timeInterval_t
min_time
(
    xme_hal_time_timeInterval_t t1,
    xme_hal_time_timeInterval_t t2
);

static INLINE int
sortBySlotStartInsertionCallback
(
    const void* const item,
    const void* const currentItem,
    const void* userData
)
{
    xme_core_exec_schedule_table_entry_t* entry = (xme_core_exec_schedule_table_entry_t*) item;
    xme_core_exec_schedule_table_entry_t* currentEntry = (xme_core_exec_schedule_table_entry_t*) currentItem;

    XME_UNUSED_PARAMETER(userData);

    // Semantics: insert just in front of an item with higher slot start time
    // or at the end of the list if no item with higher slot start time exists

    // 0 means "insert in front of currentItem"
    // 1 means "insert at a later position"
    return (currentEntry->slotStart_ns > entry->slotStart_ns) ? 0 : 1;
}

/** \brief Checks if two time-slots overlap.
 */
static bool
slotsIntersect
(
        uint64_t slot1Start,
        uint64_t slot1Length,
        uint64_t slot2Start,
        uint64_t slot2Length
);

/* TODO check if this function is useful some day */
#if 0
xme_status_t
xme_core_exec_scheduler_getFreeSlots
(
     xme_core_exec_schedule_table_t* schedule,
     xme_hal_linkedList_descriptor_t* freeSlotList
);
#endif


/****************************************************************************/
/** CREATE                                                                   **/
/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_createScheduleTable
(
    xme_core_exec_schedule_table_t** schedule,
    xme_hal_time_timeInterval_t majorCycleLength_ns
)
{
    /* Allocate the new schedule */
    *schedule =
            (xme_core_exec_schedule_table_t*)
            xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_table_t));
    XME_CHECK( NULL != *schedule,
        XME_STATUS_OUT_OF_RESOURCES);

    /* Initialize the linked list */
    XME_HAL_SINGLYLINKEDLIST_INIT((*schedule)->entries);

    /* Set cycle duration */
    (*schedule)->majorCycleDuration_ns = majorCycleLength_ns;

    /* By default, return success */
    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_replicateSchedule
(
        xme_core_exec_schedule_table_t** target,
        xme_core_exec_schedule_table_t* source
)
{
    XME_CHECK(NULL != source,
            XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(XME_STATUS_SUCCESS ==
            xme_core_exec_scheduler_createScheduleTable(
                    target, source->majorCycleDuration_ns),
            XME_STATUS_OUT_OF_RESOURCES
        );

    XME_CHECK_MSG((*target) != NULL,
            XME_STATUS_OUT_OF_RESOURCES,
            XME_LOG_WARNING,
            MODULE_ACRONYM "replicateSchedule: target = NULL!\n"
        );

    XME_HAL_SINGLYLINKEDLIST_INIT((*target)->entries);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            source->entries,
            xme_core_exec_schedule_table_entry_t,
            entry);
    {
        xme_core_exec_schedule_table_entry_t* newEntry;
        newEntry = xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_table_entry_t));
        (void) xme_hal_mem_copy(newEntry, entry, sizeof(xme_core_exec_schedule_table_entry_t));

        XME_CHECK_REC(newEntry != NULL,
            XME_STATUS_INTERNAL_ERROR,
            {
                if(XME_STATUS_SUCCESS != xme_core_exec_scheduler_clearScheduleTable(target,(bool) false))
                {
                    XME_LOG(XME_LOG_WARNING, MODULE_ACRONYM "could not clear table on error!\n");
                }
            }
        );

        XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM((*target)->entries, newEntry),
                  XME_STATUS_INTERNAL_ERROR);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
/** READ                                                                   **/
/****************************************************************************/
/**     odo in future we would like to store the schedule tables somewhere and
 * read them from that permanent location
 */

/****************************************************************************/
void
xme_core_exec_scheduler_printSchedule
(
    xme_core_exec_schedule_table_t* schedule,
    const char* message
)
{
    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "*************** %s  *****************\n", message);
    if(schedule==NULL)
    {
        XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "--- NULL ---\n");
        return;
    }

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM"majorCycleDuration=%"PRIu64"us\n",
            schedule->majorCycleDuration_ns/1000);

    if(0UL == (unsigned int)xme_hal_singlyLinkedList_getItemCount(&schedule->entries))
    {
       XME_LOG(XME_LOG_DEBUG, "Empty schedule\n");
       return;
    }

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN( schedule->entries, xme_core_exec_schedule_table_entry_t, entry);
    {
    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "%10d %10d [%10"PRIu64"us; %10"PRIu64"us]\n", entry->componentId,
        entry->functionId, entry->slotStart_ns/1000, (entry->slotStart_ns+entry->slotLength_ns)/1000);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "*************** /%s *****************\n", message);
}

/****************************************************************************/
/* TODO check if this function is useful some day */
#if 0
xme_status_t
xme_core_exec_scheduler_getFreeSlots
(
     xme_core_exec_schedule_table_t* schedule,
     xme_hal_linkedList_descriptor_t* freeSlotList
)
{
    uint32_t itemNum = 0,
            maxItem = 0;

    xme_core_exec_schedule_table_entry_t* prevEntry = NULL;

    XME_HAL_SINGLYLINKEDLIST_INIT(*freeSlotList);

    maxItem = XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(schedule->entries)- 1;

    if (0 == XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(schedule->entries)){
    xme_core_exec_schedule_table_entry_t* slot;
    slot = (xme_core_exec_schedule_table_entry_t*)xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_table_entry_t));
            slot->slotStart_ns=0;
            slot->slotLength_ns=schedule->majorCycleDuration_ns;
            XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(*freeSlotList, slot);
            return XME_STATUS_SUCCESS;
        }

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            schedule->entries,
            xme_core_exec_schedule_table_entry_t,
            entry);
        {
            xme_core_exec_schedule_table_entry_t* slot;
            slot = (xme_core_exec_schedule_table_entry_t*)xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_table_entry_t));

            if(0 == itemNum)
            {
                /* Start of schedule: first item allows precedence */
                if(entry->slotStart_ns > 0)
                {
                    slot->slotStart_ns = 0;
                    slot->slotLength_ns = entry->slotStart_ns;
                    XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(*freeSlotList, slot);
                }
            }

            if(itemNum == maxItem)
            {
                slot->slotStart_ns = entry->slotStart_ns + entry->slotLength_ns;
                slot->slotLength_ns = schedule->majorCycleDuration_ns - (entry->slotStart_ns + entry->slotLength_ns);

                XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(*freeSlotList, slot);
                return XME_STATUS_SUCCESS;
            }

            if(itemNum != 0)
            {
                slot->slotStart_ns = prevEntry->slotStart_ns + prevEntry->slotLength_ns;
                slot->slotLength_ns = entry->slotStart_ns - (prevEntry->slotStart_ns + prevEntry->slotLength_ns);

                XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(*freeSlotList, slot);
            }

            prevEntry = entry;
            itemNum++;
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

        return XME_STATUS_INTERNAL_ERROR;
}
#endif

/****************************************************************************/
/** UPDATE                                                                   **/
/****************************************************************************/
/* Insert to single schedule
 *     odo remove this function when group management is ready */
/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_autoAllocateInScheduleTable
(
    xme_core_exec_schedule_table_t*     schedule,
    xme_core_component_t                 componentId,
    xme_core_component_functionId_t     functionId,
    void*                              functionArgs,
    uint32_t periodDivider,
	uint32_t periodDividerOffset
)
{
    xme_hal_time_timeInterval_t startTime_ns = 0;
    xme_core_exec_functionDescriptor_t* fdesc = NULL;

    XME_CHECK( NULL != schedule,
        XME_STATUS_NOT_FOUND
        );

    XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_componentRepository_getFunction(componentId, functionId, &fdesc),
            XME_STATUS_NO_SUCH_VALUE,
            XME_LOG_WARNING,
            MODULE_ACRONYM "no such function: [%d|%d]\n",
            componentId,
            functionId);

    /* Check if the schedule has an available window of wcet_ns */
    XME_CHECK( XME_STATUS_SUCCESS ==
        xme_core_exec_scheduler_findWindow(schedule, fdesc->wcet_ns, 0ULL, schedule->majorCycleDuration_ns, &startTime_ns),
        XME_STATUS_OUT_OF_RESOURCES
    );

    /* Add the function to the schedule */
    XME_CHECK( XME_STATUS_SUCCESS ==
        xme_core_exec_scheduler_addElementToScheduleTable(
                schedule,
                componentId,
                functionId,
                functionArgs,
                startTime_ns,
                fdesc->wcet_ns,
                periodDivider,
				periodDividerOffset,
                (bool)true),
        XME_STATUS_INTERNAL_ERROR
    );

    /* Return success if nothing failed */
    return XME_STATUS_SUCCESS;
}



/* Insert */
/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_addElementToScheduleTable
(
    xme_core_exec_schedule_table_t* schedule,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArgs,
    xme_hal_time_timeInterval_t slotStart_ns,
    xme_hal_time_timeInterval_t slotLength_ns,
    uint32_t periodDivider,
	uint32_t periodDividerOffset,
    bool completion
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_exec_schedule_table_entry_t* item = NULL;

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "Adding function [%d|%d] to schedule\n",
            componentId, functionId);

    /* Verify the input parameters */
    XME_CHECK(componentId != XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT,
        XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(functionId != XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT,
        XME_STATUS_INVALID_PARAMETER);

    if( (slotStart_ns + slotLength_ns) > schedule->majorCycleDuration_ns)
    {
        XME_LOG(XME_LOG_WARNING, MODULE_ACRONYM "required slot does not fit schedule!\n");
        return XME_STATUS_INVALID_CONFIGURATION;
    }

    /* Check timing related to other items */
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(schedule->entries, xme_core_exec_schedule_table_entry_t, schedItem);
        if(slotsIntersect(
                schedItem->slotStart_ns,
                schedItem->slotLength_ns,
                slotStart_ns,
                slotLength_ns
                )
            )
        {
            XME_LOG(XME_LOG_WARNING,
                    MODULE_ACRONYM "found an intersection with existing slot:"
                                   " [%lu;%lu] while trying to insert [%lu; %lu]\n",
                    schedItem->slotStart_ns,
                    schedItem->slotStart_ns+schedItem->slotLength_ns,
                    slotStart_ns,
                    slotStart_ns+slotLength_ns);
            return XME_STATUS_INVALID_CONFIGURATION;
        }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    /* Allocate the new item */
    item = (xme_core_exec_schedule_table_entry_t *)
            xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_table_entry_t));
    XME_CHECK_MSG(item != NULL,
            XME_STATUS_OUT_OF_RESOURCES,
            XME_LOG_WARNING,
            MODULE_ACRONYM "could not allocate a new schedule entry!\n");

    /* Copy to the new structure */
    item->componentId = componentId;
    item->functionId = functionId;
    item->functionArgs = functionArgs;
    item->slotStart_ns  = slotStart_ns;
    item->slotLength_ns = slotLength_ns;
    item->periodDivider = periodDivider;
    item->periodDividerOffset = periodDividerOffset;

    item->completion = completion;

    /* Add the item to the table while keeping the table sorted by slotStart_ns */

    status = xme_hal_singlyLinkedList_addItemOrdered(&schedule->entries, item,
        &sortBySlotStartInsertionCallback, NULL);

    xme_core_exec_scheduler_printSchedule(schedule, "after insert");

    return status;
}


/****************************************************************************/
/** DELETE                                                                   **/
/****************************************************************************/
xme_status_t
xme_core_exec_scheduler_clearScheduleTable
(
    xme_core_exec_schedule_table_t** schedule,
    bool destroy
)
{
    XME_CHECK_MSG(schedule != NULL,
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            MODULE_ACRONYM "schedule is NULL\n");

    /* Delete all elements */
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN((*schedule)->entries,
        xme_core_exec_schedule_table_entry_t,
        scheduleEntry);
        xme_hal_mem_free(scheduleEntry);
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();


    /* Free some heap memory */
    if(destroy)
    {
        /* Finalize list */
        XME_HAL_SINGLYLINKEDLIST_FINI((*schedule)->entries);
        xme_hal_mem_free(*schedule);
        *schedule = NULL;
    }
    else
    {
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_clear(&((*schedule)->entries)),
                  XME_STATUS_INTERNAL_ERROR);
        //XME_LOG(XME_LOG_NOTE, "Cleared the schedule table\n");
    }

    /* Return success by default */
    return XME_STATUS_SUCCESS;
}


/****************************************************************************/
/****************************************************************************/
/*** LOCAL UTILITIES ***/
/****************************************************************************/
/****************************************************************************/





/** \brief Searches for a large enough window in the schedule
*/
xme_status_t
xme_core_exec_scheduler_findWindow
(
    xme_core_exec_schedule_table_t* schedule,
    xme_hal_time_timeInterval_t wcet_ns,
    xme_hal_time_timeInterval_t afterTime_ns,
    xme_hal_time_timeInterval_t beforeTime_ns,
    xme_hal_time_timeInterval_t* windowStart_ns
)
{
    //uint32_t itemNum = 0;
    //uint32_t maxItem = 0;

    xme_hal_time_timeInterval_t freeSlotStart;
    xme_hal_time_timeInterval_t freeSlotEnd;
    // xme_hal_time_timeInterval_t wStart = 0, wEnd = 0;
    xme_core_exec_schedule_table_entry_t* prevEntry = NULL;

    XME_CHECK(NULL != schedule,
              XME_STATUS_INVALID_PARAMETER);

    /* Minimum check if it fits */
    if(wcet_ns > schedule->majorCycleDuration_ns)
        return XME_STATUS_NOT_FOUND;

    /* Fits and the schedule is empty */
    if(0 == xme_hal_singlyLinkedList_getItemCount((void*)(&(schedule->entries))))
    {
        *windowStart_ns = afterTime_ns;
        return XME_STATUS_SUCCESS;
    }
    else
        XME_LOG(XME_LOG_DEBUG, "NITEMS %d\n", xme_hal_singlyLinkedList_getItemCount(&(schedule->entries)));


    // maxItem = xme_hal_singlyLinkedList_getItemCount(&(schedule->entries)) - 1;

    // last entry to be checked explicitely
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            schedule->entries,
            xme_core_exec_schedule_table_entry_t,
            entry);
    {
        freeSlotStart = (NULL == prevEntry)?0ULL:(prevEntry->slotStart_ns + prevEntry->slotLength_ns);
        freeSlotEnd = /*(0ULL == entry->slotStart_ns) ?(schedule->majorCycleDuration_ns):*/ entry->slotStart_ns;

        XME_LOG(XME_LOG_DEBUG, "a findWindow: fsStart= %10"PRIu64" fsEnd= %10"PRIu64"\n", freeSlotStart, freeSlotEnd);

        freeSlotStart = max_time(freeSlotStart, afterTime_ns);
        freeSlotEnd = min_time(freeSlotEnd, beforeTime_ns);

        XME_LOG(XME_LOG_DEBUG, "a findWindow::fsStart= %10"PRIu64" fsEnd= %10"PRIu64"\n", freeSlotStart, freeSlotEnd);

#if 0
        // XXX reduce these checks
        /* Start of free slot inside the [after; before] */
        if((freeSlotStart >= afterTime_ns) && (freeSlotStart <= beforeTime_ns))
        {
            wStart = freeSlotStart;
        }

        if((freeSlotEnd >= afterTime_ns) && (freeSlotEnd <= beforeTime_ns))
        {
            wEnd = freeSlotEnd;
        }

        if( (freeSlotStart > beforeTime_ns) || (freeSlotEnd < afterTime_ns) )
        {
            wStart=1;
            wEnd=0;
        }

        if(wStart<wEnd)
#endif
        if(freeSlotStart < freeSlotEnd)
        {
            if(freeSlotEnd - freeSlotStart >= wcet_ns)
            {
                *windowStart_ns = freeSlotStart;
                return XME_STATUS_SUCCESS;
            }
        }

        prevEntry = entry;
        //itemNum++;

        XME_LOG(XME_LOG_DEBUG, "findWindow: wStart = %10"PRIu64" wEnd = %10"PRIu64"\n",
                freeSlotStart, freeSlotEnd);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    /* last slot is special */

    {
        freeSlotStart = prevEntry->slotStart_ns + prevEntry->slotLength_ns;
        freeSlotEnd = schedule->majorCycleDuration_ns;

        XME_LOG(XME_LOG_DEBUG, "findWindow: fsStart= %10"PRIu64" fsEnd= %10"PRIu64"\n", freeSlotStart, freeSlotEnd);

        freeSlotStart = max_time(freeSlotStart, afterTime_ns);
        freeSlotEnd = min_time(freeSlotEnd, beforeTime_ns);

        XME_LOG(XME_LOG_DEBUG, "findWindow::fsStart= %10"PRIu64" fsEnd= %10"PRIu64"\n", freeSlotStart, freeSlotEnd);

        if(freeSlotStart < freeSlotEnd)
        {
            if(freeSlotEnd - freeSlotStart >= wcet_ns)
            {
                *windowStart_ns = freeSlotStart;
                return XME_STATUS_SUCCESS;
            }
        }


    }

    return XME_STATUS_NOT_FOUND;
}

static xme_hal_time_timeInterval_t
max_time(
    xme_hal_time_timeInterval_t t1,
    xme_hal_time_timeInterval_t t2)
{
    if(t1>t2)
        return t1;
    else
        return t2;
}

static xme_hal_time_timeInterval_t
min_time(
    xme_hal_time_timeInterval_t t1,
    xme_hal_time_timeInterval_t t2)
{
    if(t1<t2)
        return t1;
    else
        return t2;
}


/** \brief Searches for a large enough window in the schedule
*/

static bool
slotsIntersect
(
    uint64_t slot1Start,
    uint64_t slot1Length,
    uint64_t slot2Start,
    uint64_t slot2Length
)
{
    /* the comparision is not strict because the one slot can end at 'T' and the next can begin at 'T' */
    /* it is not mandatory to start at ('T'+1) */
    if( (slot2Start >= (slot1Start + slot1Length)) || ((slot2Start + slot2Length) <= slot1Start) )
        return false;

    return true;
}

