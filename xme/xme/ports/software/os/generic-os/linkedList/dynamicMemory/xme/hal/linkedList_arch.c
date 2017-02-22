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
 * $Id: linkedList_arch.c 7680 2014-03-05 14:36:50Z geisinger $
 */

/**
 * \file
 *         Linked list abstraction (architecture specific part: generic OS
 *         based implementation).
 *
 */

/**
 * \addtogroup hal_linkedList
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/linkedList.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Destroys the given linked list and frees the resources occupied by
 *        it.
 *
 * \note Note that this function will not free the memory occupied by the
 *       actual items, which is the responsibility of the code that allocated
 *       the respective memory.
 *
 * \note This is function called for singly and doubly linked lists.
 *
 * \param[in] linkedList The pointer to the list to finalize.
 */
static void
xme_hal_linkedList_fini
(
    xme_hal_linkedList_descriptor_t* const linkedList
);

/**
 * \brief Clears the given linked list, that is removes all elements.
 *        After calling this function, the linked list will be empty.
 *
 * \note Note that this function will not free the memory occupied by the
 *       actual items, which is the responsibility of the code that allocated
 *       the respective memory.
 *
 * \note This is function called for singly and doubly linked lists.
 *
 * \param[in] linkedList Linked list to clear.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_PARAMETER if linkedList was NULL.
 */
static xme_status_t
xme_hal_linkedList_clear
(
    xme_hal_linkedList_descriptor_t* const linkedList
);

/**
 * \brief Adds the given item to the linked list at a specific position.
 *
 * \details This function reserves memory for maintaining the new item
 *          within the linked list.
 *
 * \note This is function called for singly and doubly linked lists.
 *
 * \param[in] linkedList The pointer to the linked list descriptor.
 * \param[in] item Item to add to the linked list.
 * \param[in] doublyLinked Boolean value indicating whether the given linked
 *            list is doubly linked.
 * \param[in] insertionCallback Address of a callback function that is called
 *            once for every existing item in the linked list (starting with
 *            head) and determines whether the item should be inserted in front
 *            of the currently examined item.
 *            See the documentation of xme_hal_linkedList_insertionCallback_t.
 *            This parameter should not be NULL.
 * \param[in] userData User-defined data to pass unchanged to the insertion
 *            callback function's userData parameter in every call.
 *
 * \retval XME_STATUS_SUCCESS if the item has been added successfully.
 * \retval XME_STATUS_INVALID_PARAMETER if linkedList or insertionCallback
 *         were NULL.
 * \retval XME_STATUS_ABORTED if the callback function returned a negative
 *         value for any call and hence insertion of the item was aborted.
 * \retval XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *         platforms with dynamic memory mamanagement) or the linked list
 *         has reached its maximum capacity (on platforms without dynamic
 *         memory management).
 */
static xme_status_t
xme_hal_linkedList_addItem
(
    xme_hal_linkedList_descriptor_t* const linkedList,
    const void* const item,
    bool doublyLinked,
    xme_hal_linkedList_insertionCallback_t insertionCallback,
    const void* userData
);

/**
 * \brief Adds the given item to the linked list.
 *
 * \deprecated This function is deprecated, please use
 *             xme_hal_linkedList_addItem() instead!
 *
 * \details This function will reserve memory for maintaining the new item
 *          within the linked list.
 *
 * \note This is function called for singly and doubly linked lists.
 *
 * \param[in] linkedList The pointer to the linked list descriptor.
 * \param[in] item Item to add to the linked list.
 * \param[in] doublyLinked Boolean value determining double linked list.
 * \param[in] offset Offset of the compare value within the item data
 *            structure.
 * \param[in] compareSize Size of the attribute at the given offset within the
 *            item data structure.
 *
 * \retval XME_STATUS_SUCCESS if the item has been added successfully.
 * \retval XME_STATUS_INVALID_PARAMETER if linkedList was NULL or the given
 *         compareSize value is unsupported.
 * \retval XME_STATUS_OUT_OF_RESOURCES if the linked list has reached its
 *         maximum capacity.
 */
static xme_status_t
xme_hal_linkedList_addItemDeprecated
(
    xme_hal_linkedList_descriptor_t* const linkedList,
    const void* const item,
    bool doublyLinked,
    uint32_t offset,
    uint8_t compareSize
);

/**
 * \brief Returns a pointer to the item at the given index within the given
 *        linked list or NULL if no such item exists.
 *
 * \note This is function called for singly and doubly linked lists.
 *
 * \param[in] linkedList The pointer to the linked list.
 * \param[in] idx Zero-based index of the item within the doubly linked list
 *            to retrieve.
 * \param[in] doublyLinked Flag indicating whether is a double linked list.
 *
 * \return Returns a pointer to the item corresponding to the given index or
 *         NULL if no such item exists.
 */
const void*
xme_hal_linkedList_itemFromIndex
(
    const xme_hal_linkedList_descriptor_t* const linkedList,
    xme_hal_linkedList_index_t idx,
    bool doublyLinked
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
static void
xme_hal_linkedList_fini
(
    xme_hal_linkedList_descriptor_t* const linkedList
)
{
    XME_CHECK(NULL != linkedList, XME_CHECK_RVAL_VOID);
    XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedList)));

    (void) xme_hal_linkedList_clear(linkedList);

#if defined(DEBUG) && !defined(DOXYGEN)
    // Reset magic number
    linkedList->_isInitialized = (~XME_HAL_LINKEDLIST_MAGIC) & 0xFF;
#endif // #if defined(DEBUG) && !defined(DOXYGEN)
}

void
xme_hal_singlyLinkedList_fini
(
    void* const linkedList
)
{
    xme_hal_linkedList_fini((xme_hal_linkedList_descriptor_t*) linkedList);
}

void
xme_hal_doublyLinkedList_fini
(
    void* const linkedList
)
{
    xme_hal_linkedList_fini((xme_hal_linkedList_descriptor_t*) linkedList);
}

static xme_status_t
xme_hal_linkedList_clear
(
    xme_hal_linkedList_descriptor_t* const linkedList
)
{
    // We don't care whether the list is singly or doubly linked
    xme_hal_singlyLinkedList_genericElement_t* current;

    XME_CHECK(NULL != linkedList, XME_STATUS_INVALID_PARAMETER);
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedList)));

    current = (xme_hal_singlyLinkedList_genericElement_t*)linkedList->head;
    while (NULL != current)
    {
        xme_hal_singlyLinkedList_genericElement_t* thisElement = current;

        current = (xme_hal_singlyLinkedList_genericElement_t*)current->next;

        // Free the linked list element
        xme_hal_mem_free(thisElement);
    };

    linkedList->head = NULL;
    linkedList->end = NULL;
    linkedList->count[0] = 0;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_singlyLinkedList_clear
(
    void* const linkedList
)
{
    return xme_hal_linkedList_clear((xme_hal_linkedList_descriptor_t*) linkedList);
}

xme_status_t
xme_hal_doublyLinkedList_clear
(
    void* const linkedList
)
{
    return xme_hal_linkedList_clear((xme_hal_linkedList_descriptor_t*) linkedList);
}

static xme_status_t
xme_hal_linkedList_addItem
(
    xme_hal_linkedList_descriptor_t* const linkedList,
    const void* const item,
    bool doublyLinked,
    xme_hal_linkedList_insertionCallback_t insertionCallback,
    const void* userData
)
{
    xme_hal_doublyLinkedList_genericElement_t* element;

#if defined(DEBUG) && !defined(DOXYGEN)
    // Ensure consistency between xme_hal_singlyLinkedList_genericElement_t and xme_hal_doublyLinkedList_genericElement_t;
    // we rely here on the fact that the "next" and "item" members are located at the same offset in both structs
    XME_ASSERT((void*) &((xme_hal_singlyLinkedList_genericElement_t*) 0)->next == (void*) &((xme_hal_doublyLinkedList_genericElement_t*) 0)->next);
    XME_ASSERT((void*) &((xme_hal_singlyLinkedList_genericElement_t*) 0)->item == (void*) &((xme_hal_doublyLinkedList_genericElement_t*) 0)->item);
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

    XME_CHECK(NULL != linkedList, XME_STATUS_INVALID_PARAMETER);
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedList)));

    // Allocate memory for the new element
    XME_CHECK
    (
        NULL !=
        (
            element = (xme_hal_doublyLinkedList_genericElement_t*)xme_hal_mem_alloc(doublyLinked ? sizeof(xme_hal_doublyLinkedList_genericElement_t) : sizeof(xme_hal_singlyLinkedList_genericElement_t))
        ),
        XME_STATUS_OUT_OF_RESOURCES
    );

    element->item = item;

    if (NULL != insertionCallback)
    {
        xme_hal_doublyLinkedList_genericElement_t* loopElement = (xme_hal_doublyLinkedList_genericElement_t*) linkedList->head;
        xme_hal_doublyLinkedList_genericElement_t* prev = NULL;

        // Determine insertion position
        while (NULL != loopElement)
        {
            int insert;

            XME_ASSERT(NULL != linkedList->head);
            XME_ASSERT(NULL != linkedList->end);

            insert = insertionCallback(item, loopElement->item, userData);
            if (insert < 0)
            {
                // Abort insertion
                xme_hal_mem_free(element);

                return XME_STATUS_ABORTED;
            }

            if (0 == insert)
            {
                // Insertion point found
                if (NULL == prev)
                {
                    // Insert as head
                    XME_ASSERT(linkedList->head == loopElement);

                    if (doublyLinked)
                    {
                        // Set the old head's prev pointer to the new element
                        ((xme_hal_doublyLinkedList_genericElement_t*) linkedList->head)->prev = element;
                    }

                    linkedList->head = element;
                }
                else
                {
                    // Insert in between prev and loopElement
                    if (doublyLinked)
                    {
                        loopElement->prev = element;
                    }
                    prev->next = element;
                }

                if (doublyLinked)
                {
                    element->prev = prev;
                }
                element->next = loopElement;

                linkedList->count[0]++;

                return XME_STATUS_SUCCESS;
            }

            prev = loopElement;
            loopElement = (xme_hal_doublyLinkedList_genericElement_t*)loopElement->next;
        }
    }

    // If we get here, we need to insert the new item at the end

    // element must be initialized with all zeros
    XME_ASSERT(NULL == element->next);

    if (doublyLinked)
    {
        // Set the new element's prev to the previously last element in the list.
        // In case there was no previously last element (i.e., the list was empty),
        // the assigned NULL is fine.
        element->prev = (xme_hal_doublyLinkedList_genericElement_t*) linkedList->end;
    }

    // If this is not the first element, set the previously last element's
    // next pointer to new element
    if (NULL != linkedList->end)
    {
        ((xme_hal_doublyLinkedList_genericElement_t*) linkedList->end)->next = element;
    }

    // Set the list's end pointer to the new element
    linkedList->end = element;

    // If this is the first element, set the list's head pointer to the new element
    if (NULL == linkedList->head)
    {
        linkedList->head = element;
    }

    linkedList->count[0]++;

    return XME_STATUS_SUCCESS;
}

static xme_status_t
xme_hal_linkedList_addItemDeprecated
(
    xme_hal_linkedList_descriptor_t* const linkedList,
    const void* const item,
    bool doublyLinked,
    uint32_t offset,
    uint8_t compareSize
)
{
    xme_hal_doublyLinkedList_genericElement_t* element;
    xme_hal_doublyLinkedList_genericElement_t* prev = NULL;
    xme_hal_doublyLinkedList_genericElement_t* position = NULL;

    XME_CHECK(NULL != linkedList, XME_STATUS_INVALID_PARAMETER);
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedList)));
    XME_CHECK(compareSize <= 8, XME_STATUS_INVALID_PARAMETER);

    // Allocate memory for the new element
    XME_CHECK
    (
        NULL !=
        (
            element = (xme_hal_doublyLinkedList_genericElement_t*)xme_hal_mem_alloc(doublyLinked ? sizeof(xme_hal_doublyLinkedList_genericElement_t) : sizeof(xme_hal_singlyLinkedList_genericElement_t))
        ),
        XME_STATUS_OUT_OF_RESOURCES
    );

    if (0 == compareSize)
    {
        // Insert the new element at the end of the linked list.
        // In case the list is originally empty, position will be NULL.
        position = (xme_hal_doublyLinkedList_genericElement_t*)linkedList->end;
    }
    else
    {
        // TODO: It would be nice if we could check whether offset and compareSize are "valid".
        //       If we knew the item size (i.e., sizeof(itemType) as passed when declaring the
        //       linked list), then we could say:
        //        - XME_CHECK(offset + compareSize < itemSize, XME_CORE_STATUS_INVALID_PARAMETER);
        //       Which would also implicitly check (since compareSize >= 0):
        //        - XME_CHECK(offset < itemSize, XME_CORE_STATUS_INVALID_PARAMETER);
        //       However, since we don't know the item size and there is no way to receive it
        //       in a "safe" way (without the user specifying it as a explicit parameter, which
        //       may not match what was specified when creating the linked list), we can not
        //       perform these checks! It is not possible to remember the item size in a (global
        //       or static) variable, since the declaration of the linked list might be inside
        //       a struct definition!
        uint64_t* newValue = (uint64_t*)(((char*)item) + offset);

        // Determine insertion position
        xme_hal_doublyLinkedList_genericElement_t* loopElement = (xme_hal_doublyLinkedList_genericElement_t*)linkedList->head;
        while (loopElement)
        {
            //uint64_t* value = (uint64_t*)((uint64_t)loopElement->item + offset);
            uint64_t* value = (uint64_t*)(((char*)loopElement->item) + offset);
            uint64_t mask = (8 == compareSize) ? 0xFFFFFFFFFFFFFFFFULL : (1ULL << (compareSize*8)) - 1;

            if ((*value & mask) > (*newValue & mask))
            {
                // Insert after the previous element
                position = prev;
                break;
            }

            prev = loopElement;
            loopElement = (xme_hal_doublyLinkedList_genericElement_t*)loopElement->next;

            if (NULL == loopElement)
            {
                // Insert at the end
                position = (xme_hal_doublyLinkedList_genericElement_t*)linkedList->end;
            }
        }
    }

    if (NULL == position)
    {
        // Insert as first element
        element->next = (xme_hal_doublyLinkedList_genericElement_t*)linkedList->head;
        if (doublyLinked)
        {
            element->prev = NULL;

            if (linkedList->head) ((xme_hal_doublyLinkedList_genericElement_t*)linkedList->head)->prev = element;
        }
        linkedList->head = element;
        if (NULL == linkedList->end) linkedList->end = element;
    }
    else
    {
        // Insert as next element after position
        element->next = position->next;
        if (doublyLinked)
        {
            element->prev = (xme_hal_doublyLinkedList_genericElement_t*)position;
            if (position->next) ((xme_hal_doublyLinkedList_genericElement_t*)position->next)->prev = element;
        }
        position->next = (xme_hal_doublyLinkedList_genericElement_t*)element;
        if (linkedList->end == position) linkedList->end = element;
    }

    element->item = item;
    linkedList->count[0]++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_singlyLinkedList_addItem
(
    void* const linkedList,
    const void* const item
)
{
    return xme_hal_linkedList_addItem((xme_hal_linkedList_descriptor_t*) linkedList, item, (bool) false, NULL, NULL);
}

xme_status_t
xme_hal_singlyLinkedList_addItemOrdered
(
    void* const linkedList,
    const void* const item,
    xme_hal_linkedList_insertionCallback_t insertionCallback,
    const void* userData
)
{
    XME_CHECK(NULL != insertionCallback, XME_STATUS_INVALID_PARAMETER);

    return xme_hal_linkedList_addItem((xme_hal_linkedList_descriptor_t*) linkedList, item, (bool) false, insertionCallback, userData);
}

xme_status_t
xme_hal_singlyLinkedList_addItemSorted
(
    void* const linkedList,
    const void* const item,
    uint32_t offset,
    uint8_t compareSize
)
{
    return xme_hal_linkedList_addItemDeprecated ( (xme_hal_linkedList_descriptor_t *)linkedList, item, (bool) false, offset, compareSize);
}

xme_status_t
xme_hal_doublyLinkedList_addItem
(
    void* const linkedList,
    const void* const item
)
{
    return xme_hal_linkedList_addItem((xme_hal_linkedList_descriptor_t*) linkedList, item, (bool) true, NULL, NULL);
}

xme_status_t
xme_hal_doublyLinkedList_addItemOrdered
(
    void* const linkedList,
    const void* const item,
    xme_hal_linkedList_insertionCallback_t insertionCallback,
    const void* userData
)
{
    XME_CHECK(NULL != insertionCallback, XME_STATUS_INVALID_PARAMETER);

    return xme_hal_linkedList_addItem((xme_hal_linkedList_descriptor_t*) linkedList, item, (bool) true, insertionCallback, userData);
}

xme_status_t
xme_hal_doublyLinkedList_addItemSorted
(
    void* const linkedList,
    const void* const item,
    uint32_t offset,
    uint8_t compareSize
)
{
    return xme_hal_linkedList_addItemDeprecated ( (xme_hal_linkedList_descriptor_t *)linkedList, item, (bool) true, offset, compareSize);
}

xme_hal_linkedList_index_t
xme_hal_singlyLinkedList_removeItem
(
    void* const linkedList,
    const void* const item,
    bool all
)
{
    xme_hal_singlyLinkedList_genericElement_t* current;
    xme_hal_singlyLinkedList_genericElement_t* prev = NULL;
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t *)linkedList;
    xme_hal_linkedList_index_t origCount;

    XME_CHECK(NULL != linkedListDesc, 0U);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)), 0U);

    origCount = linkedListDesc->count[0];
    current = (xme_hal_singlyLinkedList_genericElement_t*)linkedListDesc->head;

    while (current)
    {
        // Check whether we should remove this item
        if (item == current->item)
        {
            void* next;

            if (NULL != prev)
            {
                // An element exists in front of current
                next = prev->next = current->next;
            }
            else
            {
                // Current was the first element
                next = linkedListDesc->head = current->next;
            }

            if (current == linkedListDesc->end) linkedListDesc->end = prev;

            // Free the memory allocated for maintaining the item
            xme_hal_mem_free(current);
            linkedListDesc->count[0]--;

            // Stop if we should only remove the first matching item
            if (!all)
            {
                return 1;
            }

            // Go to the next element
            current = (xme_hal_singlyLinkedList_genericElement_t*)next;
        }
        else
        {
            // Go to the next element
            prev = current;
            current = (xme_hal_singlyLinkedList_genericElement_t*)current->next;
        }
    };

    return origCount - linkedListDesc->count[0];
}

xme_hal_linkedList_index_t
xme_hal_doublyLinkedList_removeItem
(
    void* const linkedList,
    const void* const item,
    bool all
)
{
    xme_hal_doublyLinkedList_genericElement_t* current;
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t *)linkedList;
    xme_hal_linkedList_index_t origCount;

    XME_CHECK(NULL != linkedListDesc, 0U);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)), 0U);

    origCount = linkedListDesc->count[0];
    current = (xme_hal_doublyLinkedList_genericElement_t*)linkedListDesc->head;

    while (current)
    {
        // Check whether we should remove this item
        if (item == current->item)
        {
            void* next = NULL;

            if (NULL != current->prev)
            {
                ((xme_hal_doublyLinkedList_genericElement_t*)current->prev)->next = current->next;
            }
            if (NULL != current->next)
            {
                ((xme_hal_doublyLinkedList_genericElement_t*)current->next)->prev = current->prev;
                next = current->next;
            }
            if (linkedListDesc->head == current)
            {
                linkedListDesc->head = current->next;
            }
            if (linkedListDesc->end == current)
            {
                linkedListDesc->end = current->prev;
            }

            // Free the memory allocated for maintaining the item
            xme_hal_mem_free(current);
            linkedListDesc->count[0]--;

            // Stop if we should only remove the first matching item
            if (!all)
            {
                return 1;
            }

            // Go to the next element
            current = (xme_hal_doublyLinkedList_genericElement_t*)next;
        }
        else
        {
            // Go to the next element
            current = (xme_hal_doublyLinkedList_genericElement_t*)current->next;
        }
    };

    return origCount - linkedListDesc->count[0];
}

const void*
xme_hal_linkedList_itemFromIndex
(
    const xme_hal_linkedList_descriptor_t* const linkedList,
    xme_hal_linkedList_index_t idx,
    bool doublyLinked
)
{
    bool reverse = false;
    xme_hal_doublyLinkedList_genericElement_t* current;

#if defined(DEBUG) && !defined(DOXYGEN)
    // Ensure consistency between xme_hal_singlyLinkedList_genericElement_t and xme_hal_doublyLinkedList_genericElement_t;
    // we rely here on the fact that the "next" and "item" members are located at the same offset in both structs
    XME_ASSERT_RVAL((void*) &((xme_hal_singlyLinkedList_genericElement_t*) 0)->next == (void*) &((xme_hal_doublyLinkedList_genericElement_t*) 0)->next, NULL);
    XME_ASSERT_RVAL((void*) &((xme_hal_singlyLinkedList_genericElement_t*) 0)->item == (void*) &((xme_hal_doublyLinkedList_genericElement_t*) 0)->item, NULL);
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

    XME_CHECK(NULL != linkedList, NULL);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedList)), NULL);
    XME_CHECK(idx < linkedList->count[0], NULL);

    // If we want the last element, directly return it
    XME_ASSERT_RVAL(0 != linkedList->count[0], NULL);
    XME_ASSERT_RVAL(NULL != linkedList->head, NULL);
    XME_ASSERT_RVAL(NULL != linkedList->end, NULL);
    XME_CHECK(idx != linkedList->count[0] - 1, ((xme_hal_doublyLinkedList_genericElement_t*)linkedList->end)->item);

    // If the list is doubly linked and the item is more towards the end
    // of the list, we browse through the list in reverse direction
    if (doublyLinked && idx >= (linkedList->count[0]+1) >> 1)
    {
        reverse = true;
        idx = (linkedList->count[0] - idx) - 1;
        current = (xme_hal_doublyLinkedList_genericElement_t*)linkedList->end;
    }
    else
    {
        current = (xme_hal_doublyLinkedList_genericElement_t*)linkedList->head;
    }

    while (idx--)
    {
        current = (xme_hal_doublyLinkedList_genericElement_t*)(reverse ? current->prev : current->next);
    }

    return current->item;
}

/*const*/ void* // TODO: Issue #3388
xme_hal_singlyLinkedList_itemFromIndex
(
    const void* const linkedList,
    xme_hal_linkedList_index_t idx
)
{
    // TODO: Remove (void*), Issue #3388
    return (void*) xme_hal_linkedList_itemFromIndex ((xme_hal_linkedList_descriptor_t *)linkedList, idx, (bool) false);
}

/*const*/ void* // TODO: Issue #3388
xme_hal_doublyLinkedList_itemFromIndex
(
    const void* const linkedList,
    xme_hal_linkedList_index_t idx
)
{
    // TODO: Remove (void*), Issue #3388
    return (void*) xme_hal_linkedList_itemFromIndex ((xme_hal_linkedList_descriptor_t *)linkedList, idx, (bool) true);
}

xme_hal_linkedList_index_t
xme_hal_singlyLinkedList_getItemCount
(
    const void* const linkedList
)
{
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, 0U);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)), 0U);

    return linkedListDesc->count[0];
}

xme_hal_linkedList_index_t
xme_hal_doublyLinkedList_getItemCount
(
    const void* const linkedList
)
{
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, 0U);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)), 0U);

    return linkedListDesc->count[0];
}

/**
 * @}
 */
