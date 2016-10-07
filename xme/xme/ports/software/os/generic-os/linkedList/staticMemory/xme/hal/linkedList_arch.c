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
 * $Id: linkedList_arch.c 5640 2013-10-25 14:44:49Z geisinger $
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

#include "xme/defines.h"
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Adds the given item to the linked list at a specific position.
 *
 * \details The function does not allocate the memory required to store the
 *          new item. It will use one of the free slots, if available.
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
    void* const linkedList,
    const void* const item,
    bool doublyLinked,
    xme_hal_linkedList_insertionCallback_t insertionCallback,
    const void* userData
);

/**
 * \brief Adds the given item to the singly linked list.
 *
 * \details The function does not allocate the memory required to store the
 *          new item. It will use one of the free slots, if available.
 *
 * \note This is function called for singly and doubly linked lists.
 *
 * \param[in] linkedList The pointer to the linked list descriptor.
 * \param[in] item Item to add to the linked list.
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
xme_hal_singlyLinkedList_addItemDeprecated
(
    void* const linkedList,
    const void* const item,
    uint32_t offset,
    uint8_t compareSize
);

/**
 * \brief Adds the given item to the doubly linked list.
 *
 * \details The function does not allocate the memory required to store the
 *          new item. It will use one of the free slots, if available.
 *
 * \note This is function called for singly and doubly linked lists.
 *
 * \param[in] linkedList The pointer to the linked list descriptor.
 * \param[in] item Item to add to the linked list.
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
xme_hal_doublyLinkedList_addItemDeprecated
(
    void* const linkedList,
    const void* const item,
    uint32_t offset,
    uint8_t compareSize
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
xme_hal_singlyLinkedList_fini
(
    void* const linkedList
)
{
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;
    xme_status_t status;

    XME_CHECK(NULL != linkedList, XME_CHECK_RVAL_VOID);
    XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)));

    // Finalizing a statically allocated linked list is the same operation as clearing it
    status = xme_hal_singlyLinkedList_clear(linkedList);

#if defined(DEBUG) && !defined(DOXYGEN)
    // Reset magic number
    linkedListDesc->_isInitialized = (~XME_HAL_LINKEDLIST_MAGIC) & 0xFF;
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
}

void
xme_hal_doublyLinkedList_fini
(
    void* const linkedList
)
{
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;
    xme_status_t status;

    XME_CHECK(NULL != linkedList, XME_CHECK_RVAL_VOID);
    XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)));

    // Finalizing a statically allocated linked list is the same operation as clearing it
    status = xme_hal_doublyLinkedList_clear(linkedList);

#if defined(DEBUG) && !defined(DOXYGEN)
    // Reset magic number
    linkedListDesc->_isInitialized = (~XME_HAL_LINKEDLIST_MAGIC) & 0xFF;
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
}

xme_status_t
xme_hal_singlyLinkedList_clear
(
    void* const linkedList
)
{
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, XME_STATUS_INVALID_PARAMETER);
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)));

#ifdef DEBUG
    // In debug mode, we check here whether the data structure is consistent,
    // i.e., we check whether count corresponds to the actual number of items
    // and that all items point to valid next items.
    {
        xme_hal_singlyLinkedList_genericElement_t* memPool = (xme_hal_singlyLinkedList_genericElement_t*) &linkedListDesc->memPool;

        if (0U != linkedListDesc->head)
        {
            xme_hal_linkedList_index_t count;
            xme_hal_linkedList_index_t current = linkedListDesc->head;

            // Limit loop execution by maximum number of entries.
            // This probably also allows better loop unrolling optimizations.
            for (count = (xme_hal_linkedList_index_t) 1U; count <= linkedListDesc->maxCount; count++)
            {
                xme_hal_linkedList_index_t thisElement = current;
                current = memPool[current-1U].next;
                XME_ASSERT(current <= linkedListDesc->maxCount);

                // Reset the values of the array element of the memory pool in order
                // to avoid infinite loops in case of an inconsistent data structure
                memPool[thisElement-1U].next = 0U;
                memPool[thisElement-1U].item = NULL;

                // We have reached the end of the list as soon as we
                // see the element's own index as the next pointer
                if (current == thisElement)
                {
                    XME_ASSERT(linkedListDesc->end == current);
                    break;
                }
            };

            XME_ASSERT(linkedListDesc->count[0] == count);
            XME_ASSERT(count <= linkedListDesc->maxCount);
        }

        // When we get here, the memory must be all zeroes
        {
            xme_hal_linkedList_index_t i;

            for (i = 0; i < linkedListDesc->maxCount; i++)
            {
                XME_ASSERT(0U == memPool[i].next);
                XME_ASSERT(NULL == memPool[i].item);
            }
        }
    }
#endif

    // Clearing a statically allocated linked list equals filling its
    // memory pool with zeroes and resetting all other members accordingly.
    linkedListDesc->head = 0U;
    linkedListDesc->end = 0U;
    linkedListDesc->count[0] = 0U;
    xme_hal_mem_set(&linkedListDesc->memPool, 0, linkedListDesc->maxCount * sizeof(xme_hal_singlyLinkedList_genericElement_t));

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_doublyLinkedList_clear
(
    void* const linkedList
)
{
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, XME_STATUS_INVALID_PARAMETER);
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)));

#ifdef DEBUG
    // In debug mode, we check here whether the data structure is consistent,
    // i.e., we check whether count corresponds to the actual number of items
    // and that all items point to valid next and previous items.
    {
        xme_hal_doublyLinkedList_genericElement_t* memPool = (xme_hal_doublyLinkedList_genericElement_t*) &linkedListDesc->memPool;

        if (0U != linkedListDesc->head)
        {
            xme_hal_linkedList_index_t count;
            xme_hal_linkedList_index_t current = linkedListDesc->head;
            xme_hal_linkedList_index_t prev = 0U;

            // Limit loop execution by maximum number of entries.
            // This probably also allows better loop unrolling optimizations.
            for (count = (xme_hal_linkedList_index_t) 1U; count <= linkedListDesc->maxCount; count++)
            {
                xme_hal_linkedList_index_t thisElement = current;
                current = memPool[current-1U].next;
                XME_ASSERT(current <= linkedListDesc->maxCount);

                // If prev is zero, this means thisElement is the head and
                // current is the second element or thisElement if the list
                // only contains one item; in any case, prev must be equal to
                // thisElement. If prev is non-zero, thisElement is not the
                // first element.
                XME_ASSERT((0U == prev) ? (memPool[current-1U].prev == thisElement) : (memPool[thisElement-1U].prev == prev));

                // Reset the values of the array element of the memory pool in order
                // to avoid infinite loops in case of an inconsistent data structure
                memPool[thisElement-1U].next = 0U;
                memPool[thisElement-1U].item = NULL;
                memPool[thisElement-1U].prev = 0U;

                // We have reached the end of the list as soon as we
                // see the element's own index as the next pointer
                if (current == thisElement)
                {
                    XME_ASSERT(linkedListDesc->end == current);
                    break;
                }

                prev = thisElement;
            };

            XME_ASSERT(linkedListDesc->count[0] == count);
            XME_ASSERT(count <= linkedListDesc->maxCount);
        }

        // When we get here, the memory must be all zeroes
        {
            xme_hal_linkedList_index_t i;

            for (i = 0; i < linkedListDesc->maxCount; i++)
            {
                XME_ASSERT(0U == memPool[i].next);
                XME_ASSERT(NULL == memPool[i].item);
                XME_ASSERT(0U == memPool[i].prev);
            }
        }
    }
#endif

    // Clearing a statically allocated linked list equals filling its
    // memory pool with zeroes and resetting all other members accordingly.
    linkedListDesc->head = 0U;
    linkedListDesc->end = 0U;
    linkedListDesc->count[0] = 0U;
    xme_hal_mem_set(&linkedListDesc->memPool, 0, linkedListDesc->maxCount * sizeof(xme_hal_doublyLinkedList_genericElement_t));

    return XME_STATUS_SUCCESS;
}

static xme_status_t
xme_hal_linkedList_addItem
(
    void* const linkedList,
    const void* const item,
    bool doublyLinked,
    xme_hal_linkedList_insertionCallback_t insertionCallback,
    const void* userData
)
{
    xme_hal_linkedList_index_t element;
    xme_hal_linkedList_index_t prev = 0U;
    xme_hal_singlyLinkedList_genericElement_t* memPoolSingly;
    xme_hal_doublyLinkedList_genericElement_t* memPoolDoubly;
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, XME_STATUS_INVALID_PARAMETER);
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)));

    // Check if there exists memory in memPool
    XME_CHECK
    (
        linkedListDesc->count[0] < linkedListDesc->maxCount,
        XME_STATUS_OUT_OF_RESOURCES
    );

    memPoolSingly = (xme_hal_singlyLinkedList_genericElement_t*) &linkedListDesc->memPool;
    memPoolDoubly = (xme_hal_doublyLinkedList_genericElement_t*) &linkedListDesc->memPool;

    // Find a free element
    for (element = 0; element < linkedListDesc->maxCount; element++)
    {
        // A zero next pointer indicates an empty element.
        // Notice that the next pointer is set to the 1-based index of the
        // element itself in case it is the last item in the linked list.
        if ((doublyLinked && 0U == memPoolDoubly[element].next) || (!doublyLinked && 0U == memPoolSingly[element].next))
        {
            break;
        }
    }

    XME_ASSERT(element < linkedListDesc->maxCount);

    if (NULL != insertionCallback)
    {
        xme_hal_linkedList_index_t loopElement;

        // Determine insertion position
        loopElement = linkedListDesc->head;

        // TODO: Limit loop execution by maximum number of entries.
        // This probably also allows better loop unrolling optimizations.
        while (0U != loopElement)
        {
            int insert;

            XME_ASSERT(0U != linkedListDesc->head);
            XME_ASSERT(0U != linkedListDesc->end);

            insert = insertionCallback(item, doublyLinked ? memPoolDoubly[loopElement-1U].item : memPoolSingly[loopElement-1U].item, userData);
            if (insert < 0)
            {
                // Abort insertion
                return XME_STATUS_ABORTED;
            }

            if (0 == insert)
            {
                // Insertion point found
                if (0U == prev)
                {
                    // Insert as head
                    XME_ASSERT(linkedListDesc->head == loopElement);

                    if (doublyLinked)
                    {
                        // Set the old head's prev pointer to the 1-based
                        // index of the new element
                        memPoolDoubly[linkedListDesc->head-1U].prev = element+1U;
                    }

                    // Set the head pointer to the newly inserted item's 1-based index
                    linkedListDesc->head = element+1U;
                }
                else
                {
                    if (doublyLinked)
                    {
                        // Set the previous element's next pointer to the
                        // 1-based index of the new element
                        memPoolDoubly[prev-1U].next = element+1U;

                        // Set the current element's prev pointer to the
                        // 1-based index of the new element
                        memPoolDoubly[loopElement-1U].prev = element+1U;
                    }
                    else
                    {
                        // Set the previous element's next pointer to the
                        // 1-based index of the new element
                        memPoolSingly[prev-1U].next = element+1U;
                    }
                }

                if (doublyLinked)
                {
                    // Since the linked list contains at least one item, set the
                    // element's next pointer to the 1-based index of the old head
                    memPoolDoubly[element].next = loopElement;

                    // Set the prev pointer to the 1-based element of the
                    // previously seen element if such element exists or to the
                    // 1-based index of the current element otherwise to
                    // indicate that this is the first element
                    memPoolDoubly[element].prev = (0U == prev) ? (element+1U) : prev;

                    // Set the item pointer
                    memPoolDoubly[element].item = item;
                }
                else
                {
                    // Since the linked list contains at least one item, set the
                    // element's next pointer to the 1-based index of the old head
                    memPoolSingly[element].next = loopElement;

                    // Set the item pointer
                    memPoolSingly[element].item = item;
                }

                linkedListDesc->count[0]++;

                return XME_STATUS_SUCCESS;
            }

            prev = loopElement;
            loopElement = doublyLinked ? memPoolDoubly[loopElement-1U].next : memPoolSingly[loopElement-1U].next;

            // If prev equals loopElement, this means that the next pointer of
            // the original loop elements equals its index, hence marking the
            // end of the linked list
            if (prev == loopElement)
            {
                break;
            }
        }
    }

    // If we get here, we need to insert the new item at the end

    // The new element's next pointer should still be NULL
    XME_ASSERT(0U == (doublyLinked ? memPoolDoubly[element].next : memPoolSingly[element].next));

    if (doublyLinked)
    {
        // Since we are inserting at the end, set the next pointer of the new
        // element to its own 1-based index to indicate that this is the end
        // of the list
        memPoolDoubly[element].next = element+1U;

        // If this is the first element being inserted, indicated by
        // linkedList->end being zero, set the prev pointer of the new element
        // to its own index to indicate that it is the last element. Otherwise,
        // set it to the one-based index of the previously last element.
        memPoolDoubly[element].prev = (0U == linkedListDesc->end) ? (element+1U) : linkedListDesc->end;

        // Set the item pointer
        memPoolDoubly[element].item = item;
    }
    else
    {
        // Since we are inserting at the end, set the next pointer of the new
        // element to its own 1-based index to indicate that this is the end
        // of the list
        memPoolSingly[element].next = element+1U;

        // Set the item pointer
        memPoolSingly[element].item = item;
    }

    // If this is not the first element, set the previously last element's
    // next pointer to the 1-based index of the new element
    if (0U != linkedListDesc->end)
    {
        if (doublyLinked)
        {
            memPoolDoubly[linkedListDesc->end-1U].next = element+1U;
        }
        else
        {
            memPoolSingly[linkedListDesc->end-1U].next = element+1U;
        }
    }

    // Set the list's end pointer to the 1-based index of the new element
    linkedListDesc->end = element+1U;

    // If this is the first element, set the list's head pointer to the
    // 1-based index of the new element
    if (0U == linkedListDesc->head)
    {
        linkedListDesc->head = element+1U;
    }

    linkedListDesc->count[0]++;

    return XME_STATUS_SUCCESS;
}

static xme_status_t
xme_hal_singlyLinkedList_addItemDeprecated
(
    void* const linkedList,
    const void* const item,
    uint32_t offset,
    uint8_t compareSize
)
{
    xme_hal_linkedList_index_t element;
    xme_hal_linkedList_index_t prev = 0U;
    xme_hal_linkedList_index_t position = 0U;
    xme_hal_singlyLinkedList_genericElement_t* memPool;
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, XME_STATUS_INVALID_PARAMETER);
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)));
    XME_CHECK(compareSize <= 8, XME_STATUS_INVALID_PARAMETER);

    // Check if there exists memory in memPool
    XME_CHECK
    (
        linkedListDesc->count[0] < linkedListDesc->maxCount,
        XME_STATUS_OUT_OF_RESOURCES
    );

    memPool = (xme_hal_singlyLinkedList_genericElement_t*) &linkedListDesc->memPool;

    // Find a free element
    for (element = 0; element < linkedListDesc->maxCount; element++)
    {
        // A zero next pointer indicates an empty element.
        // Notice that the next pointer is set to the 1-based index of the
        // element itself in case it is the last item in the linked list.
        if (0U == memPool[element].next)
        {
            break;
        }
    }

    XME_ASSERT(element < linkedListDesc->maxCount);

    if (0U == compareSize)
    {
        // Insert the new element at the end of the linked list.
        // In case the list is originally empty, position will be NULL.
        position = linkedListDesc->end;
    }
    else
    {
        // TODO: It would be nice if we could check whether offset and compareSize are "valid".
        //       If we knew the item size (i.e., sizeof(itemType) as passed when declaring the
        //       linked list), then we could say:
        //        - XME_CHECK(offset + compareSize < itemSize, XME_STATUS_INVALID_PARAMETER);
        //       Which would also implicitly check (since compareSize >= 0):
        //        - XME_CHECK(offset < itemSize, XME_STATUS_INVALID_PARAMETER);
        //       However, since we don't know the item size and there is no way to receive it
        //       in a "safe" way (without the user specifying it as a explicit parameter, which
        //       may not match what was specified when creating the linked list), we can not
        //       perform these checks! It is not possible to remember the item size in a (global
        //       or static) variable, since the declaration of the linked list might be inside
        //       a struct definition!
        uint64_t* newValue = (uint64_t*)(((char*)item) + offset);

        // Determine insertion position
        xme_hal_linkedList_index_t loopElement = linkedListDesc->head;

        // TODO: Limit loop execution by maximum number of entries.
        // This probably also allows better loop unrolling optimizations.
        while (0U != loopElement)
        {
            //uint64_t* value = (uint64_t*)((uint64_t)loopElement->item + offset);
            uint64_t* value = (uint64_t*)(((char*)(memPool[loopElement-1U].item)) + offset);
            uint64_t mask = (8U == compareSize) ? 0xFFFFFFFFFFFFFFFFULL : (1ULL << (compareSize*8)) - 1;

            if ((*value & mask) > (*newValue & mask))
            {
                // Insert after the previous element
                position = prev;
                break;
            }

            prev = loopElement;
            loopElement = memPool[loopElement-1U].next;

            // If prev equals loopElement, this means that the next pointer of
            // the original loop elements equals its index, hence marking the
            // end of the linked list
            if (prev == loopElement)
            {
                // Insert at the end
                position = linkedListDesc->end;
                break;
            }
        }
    }

    if (0U == position)
    {
        // Insert as first element

        // If there is already a first element, set the next pointer of the new
        // element to the first element. If this is the first element being
        // inserted, set the next pointer to the 1-based index of the element
        // itself to indicate that this is the last element. Setting it to
        // zero would cause the item to be interpreted as unused.
        memPool[element].next = (0U == linkedListDesc->head) ? (element+1U) : linkedListDesc->head;

        // Set the head pointer to the newly inserted item's 1-based index
        linkedListDesc->head = element+1U;

        // If this is the first element being inserted, indicated by the fact
        // that the end pointer is still zero, set the end pointer also to the
        // newly inserted item's 1-based index
        if (0U == linkedListDesc->end)
        {
            linkedListDesc->end = linkedListDesc->head; // equals element+1U
        }
    }
    else
    {
        // Insert as next element after position

        // If position indicates the last item in the list, then its next
        // pointer must be the 1-based index of the respective item
        XME_ASSERT((linkedListDesc->end == position) ^ !(memPool[position-1U].next == position));

        // If position indicates the last item in the list, set the next
        // pointer to the 1-based index of the newly inserted element.
        // Otherwise, the next pointer of position indicates a valid next
        // item pointer that becomes the next item pointer of the newly
        // inserted element.
        memPool[element].next = (linkedListDesc->end == position) ? (element+1U) : memPool[position-1U].next;

        // Update the next pointer of the item preceding the newly added item
        memPool[position-1U].next = element+1U;

        // If position was the last element before insertion of the new item,
        // indicated by the fact that the end pointer points to position,
        // set the end pointer to the 1-based index of the newly inserted
        // item
        if (linkedListDesc->end == position)
        {
            linkedListDesc->end = element+1U;
        }
    }

    memPool[element].item = item;
    linkedListDesc->count[0]++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_singlyLinkedList_addItem
(
    void* const linkedList,
    const void* const item
)
{
    return xme_hal_linkedList_addItem(linkedList, item, (bool) false, NULL, NULL);
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

    return xme_hal_linkedList_addItem(linkedList, item, (bool) false, insertionCallback, userData);
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
    return xme_hal_singlyLinkedList_addItemDeprecated(linkedList, item, offset, compareSize);
}

static xme_status_t
xme_hal_doublyLinkedList_addItemDeprecated
(
    void* const linkedList,
    const void* const item,
    uint32_t offset,
    uint8_t compareSize
)
{
    xme_hal_linkedList_index_t element;
    xme_hal_linkedList_index_t prev = 0U;
    xme_hal_linkedList_index_t position = 0U;
    xme_hal_doublyLinkedList_genericElement_t* memPool;
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, XME_STATUS_INVALID_PARAMETER);
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)));
    XME_CHECK(compareSize <= 8, XME_STATUS_INVALID_PARAMETER);

    // Check if there exists memory in memPool
    XME_CHECK
    (
        linkedListDesc->count[0] < linkedListDesc->maxCount,
        XME_STATUS_OUT_OF_RESOURCES
    );

    memPool = (xme_hal_doublyLinkedList_genericElement_t*) &linkedListDesc->memPool;

    // Find a free element
    for (element = 0; element < linkedListDesc->maxCount; element++)
    {
        // A zero next pointer indicates an empty element.
        // Notice that the next pointer is set to the 1-based index of the
        // element itself in case it is the last item in the linked list.
        if (0U == memPool[element].next)
        {
            // In this case, the prev pointer also needs to be zero
            XME_ASSERT(0U == memPool[element].prev);
            break;
        }
    }

    XME_ASSERT(element < linkedListDesc->maxCount);

    if (0U == compareSize)
    {
        // Insert the new element at the end of the linked list.
        // In case the list is originally empty, position will be NULL.
        position = linkedListDesc->end;
    }
    else
    {
        // TODO: It would be nice if we could check whether offset and compareSize are "valid".
        //       If we knew the item size (i.e., sizeof(itemType) as passed when declaring the
        //       linked list), then we could say:
        //        - XME_CHECK(offset + compareSize < itemSize, XME_STATUS_INVALID_PARAMETER);
        //       Which would also implicitly check (since compareSize >= 0):
        //        - XME_CHECK(offset < itemSize, XME_STATUS_INVALID_PARAMETER);
        //       However, since we don't know the item size and there is no way to receive it
        //       in a "safe" way (without the user specifying it as a explicit parameter, which
        //       may not match what was specified when creating the linked list), we can not
        //       perform these checks! It is not possible to remember the item size in a (global
        //       or static) variable, since the declaration of the linked list might be inside
        //       a struct definition!
        uint64_t* newValue = (uint64_t*)(((char*)item) + offset);

        // Determine insertion position
        xme_hal_linkedList_index_t loopElement = linkedListDesc->head;

        // TODO: Limit loop execution by maximum number of entries.
        // This probably also allows better loop unrolling optimizations.
        while (0U != loopElement)
        {
            //uint64_t* value = (uint64_t*)((uint64_t)loopElement->item + offset);
            uint64_t* value = (uint64_t*)(((char*)(memPool[loopElement-1U].item)) + offset);
            uint64_t mask = (8U == compareSize) ? 0xFFFFFFFFFFFFFFFFULL : (1ULL << (compareSize*8)) - 1;

            if ((*value & mask) > (*newValue & mask))
            {
                // Insert after the previous element
                position = prev;
                break;
            }

            prev = loopElement;
            loopElement = memPool[loopElement-1U].next;

            // If prev equals loopElement, this means that the next pointer of
            // the original loop elements equals its index, hence marking the
            // end of the linked list
            if (prev == loopElement)
            {
                // Insert at the end
                position = linkedListDesc->end;
                break;
            }
        }
    }

    if (0U == position)
    {
        // Insert as first element

        // If there is already a first element, set the next pointer of the new
        // element to the first element. If this is the first element being
        // inserted, set the next pointer to the 1-based index of the element
        // itself to indicate that this is the last element. Setting it to
        // zero would cause the item to be interpreted as unused.
        memPool[element].next = (0U == linkedListDesc->head) ? (element+1U) : linkedListDesc->head;

        // Set the previous pointer to the 1-based index of the element itself,
        // because there is no element in from of this item
        memPool[element].prev = element+1U;

        // If there was already an element in the linked list, set that
        // element's prev pointer to the 1-based index of the newly inserted
        // item
        if (0U != linkedListDesc->head)
        {
            memPool[linkedListDesc->head-1U].prev = element+1U;
        }

        // Set the head pointer to the newly inserted item's 1-based index
        linkedListDesc->head = element+1U;

        // If this is the first element being inserted, indicated by the fact
        // that the end pointer is still zero, set the end pointer also to the
        // newly inserted item's 1-based index
        if (0U == linkedListDesc->end)
        {
            linkedListDesc->end = linkedListDesc->head; // equals element+1U
        }
    }
    else
    {
        // Insert as next element after position

        // If position indicates the last item in the list, then its next
        // pointer must be the 1-based index of the respective item
        XME_ASSERT((linkedListDesc->end == position) ^ !(memPool[position-1U].next == position));

        // If position indicates the first item in the list, then its prev
        // pointer must be the 1-based index of the respective item
        XME_ASSERT((linkedListDesc->head == position) ^ !(memPool[position-1U].prev == position));

        // If position indicates the last item in the list, set the next
        // pointer to the 1-based index of the newly inserted element.
        // Otherwise, the next pointer of position indicates a valid next
        // item pointer that becomes the next item pointer of the newly
        // inserted element.
        memPool[element].next = (linkedListDesc->end == position) ? (element+1U) : memPool[position-1U].next;

        // Since the item is inserted after position, the item's prev pointer
        // is the 1-based index of position
        memPool[element].prev = position;

        // If there was an element after position in the linked list, set that
        // element's prev pointer to the 1-based index of the newly inserted
        // item
        if (position != memPool[position-1U].next)
        {
            memPool[memPool[position-1U].next-1U].prev = element+1U;
        }

        // Update the next pointer of the item preceding the newly added item
        memPool[position-1U].next = element+1U;

        // If position was the last element before insertion of the new item,
        // indicated by the fact that the end pointer points to position,
        // set the end pointer to the 1-based index of the newly inserted
        // item
        if (linkedListDesc->end == position)
        {
            linkedListDesc->end = element+1U;
        }
    }

    memPool[element].item = item;
    linkedListDesc->count[0]++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_doublyLinkedList_addItem
(
    void* const linkedList,
    const void* const item
)
{
    return xme_hal_linkedList_addItem(linkedList, item, (bool) true, NULL, NULL);
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

    return xme_hal_linkedList_addItem(linkedList, item, (bool) true, insertionCallback, userData);
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
    return xme_hal_doublyLinkedList_addItemDeprecated(linkedList, item, offset, compareSize);
}

xme_hal_linkedList_index_t
xme_hal_singlyLinkedList_removeItem
(
    void* const linkedList,
    const void* const item,
    bool all
)
{
    xme_hal_linkedList_index_t count;
    xme_hal_linkedList_index_t current;
    xme_hal_linkedList_index_t prev = 0;
    xme_hal_linkedList_index_t origCount;
    xme_hal_singlyLinkedList_genericElement_t* memPool;
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, 0U);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)), 0U);

    origCount = linkedListDesc->count[0];
    memPool = (xme_hal_singlyLinkedList_genericElement_t*) &linkedListDesc->memPool;

    current = linkedListDesc->head;

    // Limit loop execution by maximum number of entries.
    // This probably also allows better loop unrolling optimizations.
    for (count = (xme_hal_linkedList_index_t) 0U; count < linkedListDesc->maxCount; count++)
    {
        if (0U == current)
        {
            break;
        }

        // Check whether we should remove this item
        if (item == memPool[current-1U].item)
        {
            xme_hal_linkedList_index_t next;

            if (0U != prev)
            {
                // An element exists in front of current

                // If the next pointer equals current, current is the last
                // item in the list. In this case, set next of the previous
                // item to the index of that item to indicate that that
                // element is now the new last element and set next to zero
                // in order to break the processing loop.
                memPool[prev-1U].next = (memPool[current-1U].next == current) ? prev : memPool[current-1U].next;
                next = (memPool[current-1U].next == current) ? 0U : memPool[current-1U].next;
            }
            else
            {
                // Current was the first element

                // If the next pointer equals current, current is the only
                // item in the list. In this case, set next and the list's
                // head pointer to zero to indicate that the list is now
                // empty. Otherwise, set it to the value of the next pointer
                // of the element being removed.
                next = linkedListDesc->head = (memPool[current-1U].next == current) ? 0U : memPool[current-1U].next;
            }

            if (current == linkedListDesc->end)
            {
                linkedListDesc->end = prev;
            }

            // Free the memory allocated for maintaining the item
            // But nothing to free here, just reset the values of the array element of the memPool
            memPool[current-1U].next = 0;
            memPool[current-1U].item = NULL;
            linkedListDesc->count[0]--;

            // Stop if we should only remove the first matching item
            if (!all)
            {
                return 1;
            }

            // Go to the next element
            current = next;
        }
        else
        {
            // If the next pointer of current equals current,
            // we are at the end of the linked list
            if (memPool[current-1U].next == current)
            {
                break;
            }

            // Go to the next element
            prev = current;
            current = memPool[current-1U].next;
        }
    };

    XME_ASSERT(count < linkedListDesc->maxCount);

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
    xme_hal_linkedList_index_t count;
    xme_hal_linkedList_index_t current;
    xme_hal_linkedList_index_t origCount;
    xme_hal_doublyLinkedList_genericElement_t* memPool;
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, 0U);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)), 0U);

    origCount = linkedListDesc->count[0];
    memPool = (xme_hal_doublyLinkedList_genericElement_t*) &linkedListDesc->memPool;

    current = linkedListDesc->head;

    // Limit loop execution by maximum number of entries.
    // This probably also allows better loop unrolling optimizations.
    for (count = (xme_hal_linkedList_index_t) 0U; count < linkedListDesc->maxCount; count++)
    {
        if (0U == current)
        {
            break;
        }

        // Check whether we should remove this item
        if (item == memPool[current-1U].item)
        {
            xme_hal_linkedList_index_t next = 0U;

            if (current != memPool[current-1U].prev)
            {
                // An element exists in front of current

                // If the next pointer equals current, current is the last
                // item in the list. In this case, set next of the previous
                // item to the index of that item to indicate that that
                // element is now the new last element and set next to zero
                // in order to break the processing loop.
                memPool[(memPool[current-1U].prev)-1U].next = (current == memPool[current-1U].next) ? memPool[current-1U].prev : memPool[current-1U].next;
            }
            if (current != memPool[current-1U].next)
            {
                // Current is not the last element in the list
                next = memPool[(memPool[current-1U].next)-1U].prev = (current == memPool[current-1U].prev) ? memPool[current-1U].next : memPool[current-1U].prev;
            }
            if (linkedListDesc->head == current)
            {
                // Find the new head: if the next pointer equals the current
                // index, then the list is going to be empty
                linkedListDesc->head = (current == memPool[current-1U].next) ? 0U : memPool[current-1U].next;
            }
            if (linkedListDesc->end == current)
            {
                // Find the new end: if the prev pointer equals the current
                // index, then the list is going to be empty
                linkedListDesc->end = (current == memPool[current-1U].prev) ? 0U : memPool[current-1U].prev;
            }

            // Free the memory allocated for maintaining the item
            // But nothing to free here, just reset the values of the array element of the memPool
            memPool[current-1U].next = 0U;
            memPool[current-1U].item = NULL;
            memPool[current-1U].prev = 0U;
            linkedListDesc->count[0]--;

            // Stop if we should only remove the first matching item
            if (!all)
            {
                return 1U;
            }

            // Go to the next element
            current = next;
        }
        else
        {
            // If the next pointer of current equals current,
            // we are at the end of the linked list
            if (memPool[current-1U].next == current)
            {
                break;
            }

            // Go to the next element
            current = memPool[current-1U].next;
        }
    };

    XME_ASSERT(count < linkedListDesc->maxCount);

    return origCount - linkedListDesc->count[0];
}

/*const*/ void* // TODO: Issue #3388
xme_hal_singlyLinkedList_itemFromIndex
(
    const void* const linkedList,
    xme_hal_linkedList_index_t idx
)
{
    xme_hal_linkedList_index_t current;
    xme_hal_singlyLinkedList_genericElement_t* memPool;
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, NULL);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)), NULL);
    XME_CHECK(idx < linkedListDesc->count[0], NULL);

    // If we want the last element, directly return it
    XME_ASSERT_RVAL(0U != linkedListDesc->count, NULL);
    XME_ASSERT_RVAL(0U != linkedListDesc->head, NULL);
    XME_ASSERT_RVAL(0U != linkedListDesc->end, NULL);

    current = linkedListDesc->head;

    memPool = (xme_hal_singlyLinkedList_genericElement_t*) &linkedListDesc->memPool;

    // Directly return the last item if index is count-1
    // TODO: Remove (void*), Issue #3388
    XME_CHECK(idx != linkedListDesc->count[0] - 1U, (void*) memPool[linkedListDesc->end-1U].item);

    while (0U != idx--)
    {
        current = memPool[current-1U].next;
    }

    // TODO: Remove (void*), Issue #3388
    return (void*) memPool[current-1U].item;
}

/*const*/ void* // TODO: Issue #3388
xme_hal_doublyLinkedList_itemFromIndex
(
    const void* const linkedList,
    xme_hal_linkedList_index_t idx
)
{
    bool reverse = false;
    xme_hal_linkedList_index_t current;
    xme_hal_doublyLinkedList_genericElement_t* memPool;
    xme_hal_linkedList_descriptor_t* linkedListDesc = (xme_hal_linkedList_descriptor_t*) linkedList;

    XME_CHECK(NULL != linkedList, NULL);
    XME_ASSERT_RVAL(XME_ASSERT_NO_SIDE_EFFECTS(XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedListDesc)), NULL);
    XME_CHECK(idx < linkedListDesc->count[0], NULL);


    // If we want the last element, directly return it
    XME_ASSERT_RVAL(0 != linkedListDesc->count, NULL);
    XME_ASSERT_RVAL(0 != linkedListDesc->head, NULL);
    XME_ASSERT_RVAL(0 != linkedListDesc->end, NULL);

    current = linkedListDesc->head;

    memPool = (xme_hal_doublyLinkedList_genericElement_t*) &linkedListDesc->memPool;

    // Directly return the last item if index is count-1
    // TODO: Remove (void*), Issue #3388
    XME_CHECK(idx != linkedListDesc->count[0] - 1U, (void*) memPool[linkedListDesc->end-1U].item);

    // If the list is doubly linked and the item is more towards the end
    // of the list, we browse through the list in reverse direction
    if(idx >= (linkedListDesc->count[0] + 1) >> 1)
    {
        reverse = true;
        idx = linkedListDesc->count[0] - idx - 1;
        current = linkedListDesc->end;
    }

    while (0U != idx--)
    {
        current = reverse ? memPool[current-1U].prev : memPool[current-1U].next;
    }

    // TODO: Remove (void*), Issue #3388
    return (void*) memPool[current-1U].item;
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
