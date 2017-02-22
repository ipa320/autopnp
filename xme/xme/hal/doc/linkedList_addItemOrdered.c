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
 * $Id: linkedList_addItemOrdered.c 5641 2013-10-25 14:45:25Z geisinger $
 */

/**
 * \file linkedList_addItemOrdered.c
 */

/*
 * This example shows how to properly use the functions
 * xme_hal_singlyLinkedList_addItemOrdered() and
 * xme_hal_doublyLinkedList_addItemOrdered() from the xme_hal_linkedList
 * component. Make sure you link against xme_hal_linkedList and xme_core_log
 * in order to run this example.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/log.h"
#include "xme/hal/include/linkedList.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \brief Event type enumeration.
 */
typedef enum
{
    EVENTTYPE_REALTIME = 0, ///< High-priority interrupt.
    EVENTTYPE_TIMER = 1, ///< Timer event.
    EVENTTYPE_USER = 2 ///< User event (low priority).
} eventType_t;

/**
 * \brief Event information struct.
 */
typedef struct
{
    eventType_t eventType; ///< Type of event.
    const char* eventName; ///< Name of event.
} event_t;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

static int
sortByEventTypeInsertionCallback
(
    const void* const item,
    const void* const currentItem,
    const void* userData
)
{
    event_t* newEvent = (event_t*) item;
    event_t* existingEvent = (event_t*) currentItem;

    // Keeping the code clean
    XME_UNUSED_PARAMETER(userData);

    // Avoid duplicate insertion of realtime events
    if (EVENTTYPE_REALTIME == existingEvent->eventType &&
        EVENTTYPE_REALTIME == newEvent->eventType)
    {
        // -1 means "abort insertion".
        // The caller of xme_hal_singlyLinkedList_addItemOrdered()
        // will receive a result of XME_STATUS_ABORTED.
        return -1;
    }

    // Otherwise, insert the new event just in front of an existing
    // event with higher priority or at the end of the list if no
    // higher priority event exists.
    // 0 means "insert in front of currentItem".
    // 1 means "insert at a later position".
    return (newEvent->eventType < existingEvent->eventType) ? 0 : 1;
}

int
main(int argc, char **argv)
{
    // Define a linked list with a capacity of 10 elements.
    // The capacity will only be enforced on platforms with
    // static memory allocation. If a heap is available, the
    // capacity is only bounded by available heap memory.
    xme_hal_singlyLinkedList_t(10) queue;

    event_t realtimeEvent1 = { EVENTTYPE_REALTIME, "realtimeEvent1" };
    event_t realtimeEvent2 = { EVENTTYPE_REALTIME, "realtimeEvent2" };
    event_t timerEvent     = { EVENTTYPE_TIMER,    "timerEvent"     };
    event_t userEvent      = { EVENTTYPE_USER,     "userEvent"      };
    xme_status_t status;
    unsigned int count, i;

    // Keeping the code clean
    XME_UNUSED_PARAMETER(argc);
    XME_UNUSED_PARAMETER(argv);

    // Initialize the linked list before use
    XME_HAL_SINGLYLINKEDLIST_INIT(queue);

    status = xme_hal_singlyLinkedList_addItemOrdered(
        &queue, &realtimeEvent1, &sortByEventTypeInsertionCallback, NULL
    );
    XME_LOG(XME_LOG_ALWAYS, "Insertion of %s returned status %u (expecting %u).\n",
        realtimeEvent1.eventName, (unsigned int) status, (unsigned int) XME_STATUS_SUCCESS);

    status = xme_hal_singlyLinkedList_addItemOrdered(
        &queue, &userEvent, &sortByEventTypeInsertionCallback, NULL
    );
    XME_LOG(XME_LOG_ALWAYS, "Insertion of %s returned status %u (expecting %u).\n",
        userEvent.eventName, (unsigned int) status, (unsigned int) XME_STATUS_SUCCESS);

    status = xme_hal_singlyLinkedList_addItemOrdered(
        &queue, &timerEvent, &sortByEventTypeInsertionCallback, NULL
    );
    XME_LOG(XME_LOG_ALWAYS, "Insertion of %s returned status %u (expecting %u).\n",
        timerEvent.eventName, (unsigned int) status, (unsigned int) XME_STATUS_SUCCESS);

    status = xme_hal_singlyLinkedList_addItemOrdered(
        &queue, &realtimeEvent2, &sortByEventTypeInsertionCallback, NULL
    );
    XME_LOG(XME_LOG_ALWAYS, "Insertion of %s returned status %u (expecting %u).\n",
        realtimeEvent2.eventName, (unsigned int) status, (unsigned int) XME_STATUS_ABORTED);

    // Check correct insertion and sorting of items. Will print:
    //  Queue contains 3 items.
    //  Item at index 0 is is named "realtimeEvent1".
    //  Item at index 1 is is named "timerEvent".
    //  Item at index 2 is is named "userEvent".
    count = xme_hal_singlyLinkedList_getItemCount(&queue);
    XME_LOG(XME_LOG_ALWAYS, "Queue contains %u items.\n", count);
    for (i = 0U; i < count; i++)
    {
        event_t* event = (event_t*) xme_hal_singlyLinkedList_itemFromIndex(&queue, i);
        XME_LOG(XME_LOG_ALWAYS, "Item at index %u is named \"%s\".\n", i, event->eventName);
    }

    // Clean up (frees allocated memory for maintaining the queue)
    XME_HAL_SINGLYLINKEDLIST_FINI(queue);

    return 0;
}
