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
 * $Id: linkedList.h 5641 2013-10-25 14:45:25Z geisinger $
 */

/**
 * \file
 *         Linked list abstraction.
 */

#ifndef XME_HAL_LINKEDLIST_H
#define XME_HAL_LINKEDLIST_H

/**
 * \defgroup hal_linkedList Linked list
 *
 * \brief  Linked list abstraction.
 *
 *         A linked list is a data structure in which each item consists of a
 *         pointer to the next and/or pevious item as well as a payload.
 *         Items can be added to and removed from the list by adjusting the
 *         next/previous pointers.
 *
 * \note   Linked lists are implementated differently depending on the
 *         architecture. This is why linked lists are covered by the hardware
 *         abstraction library. On some target platforms, linked list items
 *         are dynamically allocated and the number of items is only limited
 *         by the available system memory.
 *         On resource constrained platforms without efficient memory
 *         management, a static maximum size (in terms of number of items) is
 *         given for each linked list.
 */

/**
 * \example linkedList_addItemOrdered.c
 *
 * This example shows how to properly use the functions
 * xme_hal_singlyLinkedList_addItemOrdered() and
 * xme_hal_doublyLinkedList_addItemOrdered() from the
 * xme_hal_linkedList component.
 *
 * The scenario is as follows: a queue (implemented as a singly linked list)
 * is used to store events for later processing. There are different event
 * types available with different execution priority as defined in the
 * eventType_t type. In addition, the queue needs to keep the invariant that
 * at most one event of type EVENTTYPE_REALTIME can be contained in the queue
 * at any time.
 * This example illustrates the usage of xme_hal_singlyLinkedList_addItemOrdered()
 * only. However, since single and doubly linked lists have the same semantics
 * with respect to ordered insertion, it can be applied in the same way to
 * doubly linked lists using xme_hal_doublyLinkedList_addItemOrdered().
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdbool.h>
#include <stddef.h> // Ensure that offsetof() is available
#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def XME_HAL_LINKEDLIST_MAGIC
 *
 * \brief  Magic number used to determine whether a linked list is initialized or not.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define XME_HAL_LINKEDLIST_MAGIC (0x97) // Bit pattern 10010111
#else
    // Release mode
    #define XME_HAL_LINKEDLIST_MAGIC
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/**
 * \def XME_HAL_LINKEDLIST_MAGIC_CHECK
 *
 * \brief  Checks whether magic number is correct or not.
 *
 * \param[in] linkedList Address of the descriptor of the linked list to test.
 *
 * \return True if the magic number of the linked list is set correctly or when
 *         building in release mode (where not magic number check is performed).
 *         False otherwise.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedList) ((linkedList)->_isInitialized == XME_HAL_LINKEDLIST_MAGIC)
#else // #if defined(DEBUG) && !defined(DOXYGEN)
    // Release mode
    #define XME_HAL_LINKEDLIST_MAGIC_CHECK(linkedList) (!0)
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_hal_linkedList_index_t
 *
 * \brief  Linked list index type.
 */
typedef xme_maxSystemValue_t xme_hal_linkedList_index_t;

/**
 * \typedef xme_hal_linkedList_insertionCallback_t
 *
 * \brief Insertion callback function.
 *
 * \details Callback function that is used in conjunction with
 *          xme_hal_singlyLinkedList_addItemOrdered() and
 *          xme_hal_doublyLinkedList_addItemOrdered() to determine the point
 *          of insertion of a new item.
 *          The function is called for every existing item in a linked list.
 *          In case the callback function returns zero, the new item is
 *          inserted just in front of the currently examined item and the
 *          insertion callback is not called any more. If the callback
 *          function returns a positive value (e.g., 1), examination continues
 *          with the next item, if any. When the end of the list is reached and
 *          the callback function still returns a positive value, the item is
 *          inserted at the end of the list.
 *          If the callback function returns a negative value (e.g., -1) for
 *          any call, insertion is aborted and the caller of
 *          xme_hal_singlyLinkedList_addItemOrdered() or
 *          xme_hal_doublyLinkedList_addItemOrdered() will receive a status
 *          code of XME_STATUS_ABORTED. This can for example be used to
 *          prevent the insertion of duplicate items into the list.
 *
 * \param[in] item Item to be inserted into the linked list.
 * \param[in] currentItem Pointer to the existing item in the linked list
 *            currently being examined (starting with head).
 * \param[in] userData Pointer to user-defined data as passed in the userData
 *            parameter by the caller of the
 *            xme_hal_singlyLinkedList_addItemOrdered() or
 *            xme_hal_doublyLinkedList_addItemOrdered() function.
 *
 * \retval 0 should be returned to indicate that item should be inserted
 *         in front of the item pointed to by currentItem.
 * \retval 1 (or a positive value in general) should be returned to indicate
 *         that examination should continue with the next item.
 * \retval -1 (or a negative value in general) should be returned to abort
 *         insertion immediately.
 */
typedef int (*xme_hal_linkedList_insertionCallback_t)
(
    const void* const item,
    const void* const currentItem,
    const void* userData
);

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Destroys the given singly linked list and frees the resources
 *        occupied by it.
 *
 * \note Note that this function will not free the memory occupied by the
 *       actual items, which is the responsibility of the code that allocated
 *       the respective memory.
 *
 * \param[in] linkedList The pointer to the list to finalize.
 */
void
xme_hal_singlyLinkedList_fini
(
    void* const linkedList
);

/**
 * \brief Destroys the given doubly linked list and frees the resources
 *        occupied by it.
 *
 * \note Note that this function will not free the memory occupied by the
 *       actual items, which is the responsibility of the code that allocated
 *       the respective memory.
 *
 * \param[in] linkedList The pointer to the list to finalize.
 */
void
xme_hal_doublyLinkedList_fini
(
    void* const linkedList
);

/**
 * \brief Clears the given singly linked list, that is removes all elements.
 *        After calling this function, the linked list will be empty.
 *
 * \note Note that this function will not free the memory occupied by the
 *       actual items, which is the responsibility of the code that allocated
 *       the respective memory.
 *
 * \param[in] linkedList Linked list to clear.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_PARAMETER if linkedList was NULL.
 */
xme_status_t
xme_hal_singlyLinkedList_clear
(
    void* const linkedList
);

/**
 * \brief Clears the given doubly linked list, that is removes all elements.
 *        After calling this function, the linked list will be empty.
 *
 * \note Note that this function will not free the memory occupied by the
 *       actual items, which is the responsibility of the code that allocated
 *       the respective memory.
 *
 * \param[in] linkedList Linked list to clear.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_PARAMETER if linkedList was NULL.
 */
xme_status_t
xme_hal_doublyLinkedList_clear
(
    void* const linkedList
);

/**
 * \brief Adds the given item to the singly linked list.
 *
 * \details On platforms with dynamic memory allocation, this function will
 *          reserve memory for maintaining the new item within the linked list.
 *          On platforms with static memory allocation, it will use one of the
 *          free slots, if available.
 *
 * \param[in] linkedList The pointer to the linked list descriptor.
 * \param[in] item Item to add to the linked list.
 *
 * \retval XME_STATUS_SUCCESS if the item has been added successfully.
 * \retval XME_STATUS_INVALID_PARAMETER if linkedList was NULL.
 * \retval XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *         platforms with dynamic memory mamanagement) or the linked list
 *         has reached its maximum capacity (on platforms without dynamic
 *         memory management).
 */
xme_status_t
xme_hal_singlyLinkedList_addItem
(
    void* const linkedList,
    const void* const item
);

/**
 * \brief Adds the given item to the singly linked list at a specific position.
 *
 * \details On platforms with dynamic memory allocation, this function will
 *          reserve memory for maintaining the new item within the linked list.
 *          On platforms with static memory allocation, it will use one of the
 *          free slots, if available.
 *
 * \param[in] linkedList The pointer to the linked list descriptor.
 * \param[in] item Item to add to the linked list.
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
xme_status_t
xme_hal_singlyLinkedList_addItemOrdered
(
    void* const linkedList,
    const void* const item,
    xme_hal_linkedList_insertionCallback_t insertionCallback,
    const void* userData
);

/**
 * \brief Adds the given item to the singly linked list in sorted order.
 *
 * \deprecated This function is deprecated, please use
 *             xme_hal_singlyLinkedList_addItemOrdered() instead!
 *
 * \details On platforms with dynamic memory allocation, this function will
 *          reserve memory for maintaining the new item within the linked list.
 *          On platforms with static memory allocation, it will use one of the
 *          free slots, if available.
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
 * \retval XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *         platforms with dynamic memory mamanagement) or the linked list
 *         has reached its maximum capacity (on platforms without dynamic
 *         memory management).
 */
xme_status_t
xme_hal_singlyLinkedList_addItemSorted
(
    void* const linkedList,
    const void* const item,
    uint32_t offset,
    uint8_t compareSize
);

/**
 * \brief Adds the given item to the doubly linked list.
 *
 * \details On platforms with dynamic memory allocation, this function will
 *          reserve memory for maintaining the new item within the linked list.
 *          On platforms with static memory allocation, it will use one of the
 *          free slots, if available.
 *
 * \param[in] linkedList The pointer to the linked list descriptor.
 * \param[in] item Item to add to the linked list.
 *
 * \retval XME_STATUS_SUCCESS if the item has been added successfully.
 * \retval XME_STATUS_INVALID_PARAMETER if linkedList was NULL.
 * \retval XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *         platforms with dynamic memory mamanagement) or the linked list
 *         has reached its maximum capacity (on platforms without dynamic
 *         memory management).
 */
xme_status_t
xme_hal_doublyLinkedList_addItem
(
    void* const linkedList,
    const void* const item
);

/**
 * \brief Adds the given item to the doubly linked list at a specific position.
 *
 * \details On platforms with dynamic memory allocation, this function will
 *          reserve memory for maintaining the new item within the linked list.
 *          On platforms with static memory allocation, it will use one of the
 *          free slots, if available.
 *
 * \param[in] linkedList The pointer to the linked list descriptor.
 * \param[in] item Item to add to the linked list.
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
xme_status_t
xme_hal_doublyLinkedList_addItemOrdered
(
    void* const linkedList,
    const void* const item,
    xme_hal_linkedList_insertionCallback_t insertionCallback,
    const void* userData
);

/**
 * \brief Adds the given item to the doubly linked list in sorted order.
 *
 * \deprecated This function is deprecated, please use
 *             xme_hal_doublyLinkedList_addItemOrdered() instead!
 *
 * \details On platforms with dynamic memory allocation, this function will
 *          reserve memory for maintaining the new item within the linked list.
 *          On platforms with static memory allocation, it will use one of the
 *          free slots, if available.
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
 * \retval XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *         platforms with dynamic memory mamanagement) or the linked list
 *         has reached its maximum capacity (on platforms without dynamic
 *         memory management).
 */
xme_status_t
xme_hal_doublyLinkedList_addItemSorted
(
    void* const linkedList,
    const void* const item,
    uint32_t offset,
    uint8_t compareSize
);

/**
 * \brief Removes the given item from the given single linked list.
 *
 * \note The function does not free the memory associated with the item.
 *
 * \param[in] linkedList The pointer to the single linked list structure.
 * \param[in] item The pointer to the item to be removed from the list.
 * \param[in] all Flag indicating whether all matching items should be removed.
 *            If set to false, only the first occurence is removed. If set to
 *            true, all occurrences in the linked list are removed.
 *
 * \return Returns the number of items removed.
 */
xme_hal_linkedList_index_t
xme_hal_singlyLinkedList_removeItem
(
    void* const linkedList,
    const void* const item,
    bool all
);

/**
 * \brief Removes the given item from the given double linked list.
 *
 * \note The function does not free the memory associated with the item.
 *
 * \param[in] linkedList The pointer to the double linked list structure.
 * \param[in] item The pointer to the item to be removed from the list.
 * \param[in] all Flag indicating whether all matching items should be removed.
 *            If set to false, only the first occurence is removed. If set to
 *            true, all occurrences in the linked list are removed.
 *
 * \return Returns the number of items removed.
 */
xme_hal_linkedList_index_t
xme_hal_doublyLinkedList_removeItem
(
    void* const linkedList,
    const void* const item,
    bool all
);

/**
 * \brief Returns a pointer to the item at the given index within the given
 *        linked list or NULL if no such item exists.
 *
 * \param[in] linkedList The pointer to the linked list.
 * \param[in] idx Zero-based index of the item within the doubly linked list
 *            to retrieve.
 *
 * \return Returns a pointer to the item corresponding to the given index or
 *         NULL if no such item exists.
 */
/*const*/ void* // TODO: Issue #3388
xme_hal_singlyLinkedList_itemFromIndex
(
    const void* const linkedList,
    xme_hal_linkedList_index_t idx
);

/**
 * \brief Returns a pointer to the item at the given index within the given
 *        linked list or NULL if no such item exists.
 *
 * \param[in] linkedList The pointer to the linked list.
 * \param[in] idx Zero-based index of the item within the doubly linked list
 *            to retrieve.
 *
 * \return Returns a pointer to the item corresponding to the given index or
 *         NULL if no such item exists.
 */
/*const*/ void* // TODO: Issue #3388
xme_hal_doublyLinkedList_itemFromIndex
(
    const void* const linkedList,
    xme_hal_linkedList_index_t idx
);

/**
 * \brief Returns the number of elements in a singly linked list.
 *
 * \details The runtime of this function is guaranteed to be O(1).
 *
 * \param[in] linkedList Pointer to the single linked list.
 * 
 * \return Returns the number of elements in the singly linked list.
 */
xme_hal_linkedList_index_t
xme_hal_singlyLinkedList_getItemCount
(
    const void* const linkedList
);

/**
 * \brief Returns the number of elements in a doubly linked list.
 *
 * \details The runtime of this function is guaranteed to be O(1).
 *
 * \param[in] linkedList Pointer to the doubly linked list.
 *
 * \return Returns the number of elements in the doubly linked list.
 */
xme_hal_linkedList_index_t
xme_hal_doublyLinkedList_getItemCount
(
    const void* const linkedList
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/linkedList_arch.h"

#endif // #ifndef XME_HAL_LINKEDLIST_H
