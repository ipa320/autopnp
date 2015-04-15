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
 * $Id: linkedList_arch.h 5028 2013-09-10 13:27:32Z geisinger $
 */

/**
 * \file
 *         Linked list abstraction (architecture specific part: generic OS
 *         based implementation).
 *
 */

#ifndef XME_HAL_LINKEDLIST_ARCH_H
#define XME_HAL_LINKEDLIST_ARCH_H

/**
 * \addtogroup hal_linkedList
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/*
 * \typedef xme_hal_linkedList_descriptor_t
 *
 * \brief  Linked list descriptor.
 *
 *         Defines the internal representation of a linked list.
 */
typedef struct
{
    xme_hal_linkedList_index_t head; ///< Link to the first element in the list or zero if the list is empty.
    xme_hal_linkedList_index_t end; ///< Link to the last element in the list or zero if the list is empty.
    xme_hal_linkedList_index_t count[1]; ///< Number of elements in the list (for reasons of efficiency).
    xme_hal_linkedList_index_t maxCount; ///< Maximum number of elements allowed in the list (for it is implemented using an array).
#if defined(DEBUG) && !defined(DOXYGEN)
    uint8_t _isInitialized; ///< Magic number used to heuristically determine whether the linked list has been properly initialized.
#endif // #if defined(DEBUG) && !defined(DOXYGEN)
    void* memPool; ///< Static memory pool for array elements reserved during declaration of the linked list.
}
xme_hal_linkedList_descriptor_t;

/*
 * \struct xme_hal_singlyLinkedList_genericElement_t
 *
 * \brief  Singly Linked list descriptor.
 *
 * \details Defines the internal representation of a singly linked list.
 */
typedef struct xme_hal_singlyLinkedList_genericElement_s
{
    xme_hal_linkedList_index_t next; ///< Index of the next list element.
    const void* item; ///< Item stored in the linked list.
}
xme_hal_singlyLinkedList_genericElement_t;

/*
 * \struct xme_hal_doublyLinkedList_genericElement_t
 *
 * \brief  Doubly Linked list descriptor.
 *
 *\details Defines the internal representation of a doubly linked list.
 */
typedef struct xme_hal_doublyLinkedList_genericElement_s
{
    xme_hal_linkedList_index_t next; ///< Index of the next list element.
    const void* item; ///< Item stored in the linked list.
    xme_hal_linkedList_index_t prev; ///< Index of the previous list element. This element must come after the elements defined in xme_hal_singlyLinkedList_genericElement_s.
}
xme_hal_doublyLinkedList_genericElement_t;

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def XME_HAL_LINKEDLIST_STATIC_ALLOC
 *
 * \brief  Indicates that memory for linked lists is statically allocated.
 */
#define XME_HAL_LINKEDLIST_STATIC_ALLOC

/**
 * \def xme_hal_singlyLinkedList_t
 *
 * \brief  Defines a singly linked list with a given maximum size.
 *
 * \note   Note that the above definition of this macro must match the
 *         definition of the xme_hal_linkedList_descriptor_t type.
 *
 * \param  maxItems Maximum number of items in the linked list.
 *         On platforms without dynamic memory allocation, this parameter
 *         defines the maximum number of items in the linked list. The
 *         memory required to hold these items will then be statically
 *         allocated.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define xme_hal_singlyLinkedList_t(maxItems) \
    struct \
    { \
        xme_hal_linkedList_index_t head; \
        xme_hal_linkedList_index_t end; \
        xme_hal_linkedList_index_t count[1]; \
        xme_hal_linkedList_index_t maxCount; \
        uint8_t _isInitialized; \
        xme_hal_singlyLinkedList_genericElement_t memPool[(xme_hal_linkedList_index_t)((maxItems)+0U)]; \
    }
#else // #if defined(DEBUG) && !defined(DOXYGEN)
    // Release mode
    #define xme_hal_singlyLinkedList_t(maxItems) \
    struct \
    { \
        xme_hal_linkedList_index_t head; \
        xme_hal_linkedList_index_t end; \
        xme_hal_linkedList_index_t count[1]; \
        xme_hal_linkedList_index_t maxCount; \
        xme_hal_singlyLinkedList_genericElement_t memPool[(xme_hal_linkedList_index_t)((maxItems)+0U)]; \
    }
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/**
 * \def xme_hal_doublyLinkedList_t
 *
 * \brief  Defines a doubly linked list with a given maximum size.
 *
 * \note   Note that the above definition of this macro must match the
 *         definition of the xme_hal_linkedList_descriptor_t type.
 *
 * \param  maxItems Maximum number of items in the linked list.
 *         On platforms without dynamic memory allocation, this parameter
 *         defines the maximum number of items in the linked list. The
 *         memory required to hold these items will then be statically
 *         allocated.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define xme_hal_doublyLinkedList_t(maxItems) \
    struct \
    { \
        xme_hal_linkedList_index_t head; \
        xme_hal_linkedList_index_t end; \
        xme_hal_linkedList_index_t count[1]; \
        xme_hal_linkedList_index_t maxCount; \
        uint8_t _isInitialized; \
        xme_hal_doublyLinkedList_genericElement_t memPool[(xme_hal_linkedList_index_t)((maxItems)+0U)]; \
    }
#else // #if defined(DEBUG) && !defined(DOXYGEN)
    // Release mode
    #define xme_hal_doublyLinkedList_t(maxItems) \
    struct \
    { \
        xme_hal_linkedList_index_t head; \
        xme_hal_linkedList_index_t end; \
        xme_hal_linkedList_index_t count[1]; \
        xme_hal_linkedList_index_t maxCount; \
        xme_hal_doublyLinkedList_genericElement_t memPool[(xme_hal_linkedList_index_t)((maxItems)+0U)]; \
    }
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/**
 * \def XME_HAL_SINGLYLINKEDLIST_INIT
 *
 * \brief  Initializes the given singly linked list.
 *         This macro must be called for a linked list before it can be used.
 *
 * \param  name Name of the linked list to initialize.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define XME_HAL_SINGLYLINKEDLIST_INIT(name) \
    do \
    { \
        (name).head = (name).end = (name).count[0] = (xme_hal_linkedList_index_t) 0U; \
        (name).maxCount = (xme_hal_linkedList_index_t) (sizeof((name).memPool) / sizeof(xme_hal_singlyLinkedList_genericElement_t)); \
        (name)._isInitialized = XME_HAL_LINKEDLIST_MAGIC; \
        xme_hal_mem_set( &((name).memPool), 0, sizeof((name).memPool)); \
    } while (0)
#else // #if defined(DEBUG) && !defined(DOXYGEN)
    // Release mode
    #define XME_HAL_SINGLYLINKEDLIST_INIT(name) \
    do \
    { \
        (name).head = (name).end = (name).count[0] = (xme_hal_linkedList_index_t) 0U; \
        (name).maxCount = (xme_hal_linkedList_index_t) (sizeof((name).memPool) / sizeof(xme_hal_singlyLinkedList_genericElement_t)); \
        xme_hal_mem_set( &((name).memPool), 0, sizeof((name).memPool)); \
    } while (0)
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_INIT
 *
 * \brief  Initializes the given doubly linked list.
 *         This macro must be called for a linked list before it can be used.
 *
 * \param  name Name of the linked list to initialize.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define XME_HAL_DOUBLYLINKEDLIST_INIT(name) \
    do \
    { \
        (name).head = (name).end = (name).count[0] = (xme_hal_linkedList_index_t) 0U; \
        (name).maxCount = (xme_hal_linkedList_index_t) (sizeof((name).memPool) / sizeof(xme_hal_doublyLinkedList_genericElement_t)); \
        (name)._isInitialized = XME_HAL_LINKEDLIST_MAGIC; \
        xme_hal_mem_set( &((name).memPool), 0, sizeof((name).memPool)); \
    } while (0)
#else // #if defined(DEBUG) && !defined(DOXYGEN)
    // Release mode
    #define XME_HAL_DOUBLYLINKEDLIST_INIT(name) \
    do \
    { \
        (name).head = (name).end = (name).count[0] = (xme_hal_linkedList_index_t) 0U; \
        (name).maxCount = (xme_hal_linkedList_index_t) (sizeof((name).memPool) / sizeof(xme_hal_doublyLinkedList_genericElement_t)); \
        xme_hal_mem_set( &((name).memPool), 0, sizeof((name).memPool)); \
    } while (0)
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/**
 * \def XME_HAL_SINGLYLINKEDLIST_FINI
 *
 * \brief  Destroys the given linked list and frees the resources occupied
 *         by it.
 *
 * \deprecated This macro is deprecated, please use xme_hal_singlyLinkedList_fini() instead!
 *
 * \note   Note that this macro will not free the memory occupied by the
 *         actual items, which is the responsibility of the code that
 *         allocated the respective memory.
 *
 * \param  name Name of the linked list to finalize.
 */
#define XME_HAL_SINGLYLINKEDLIST_FINI(name) \
    xme_hal_singlyLinkedList_fini((xme_hal_linkedList_descriptor_t*)&(name))

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_FINI
 *
 * \brief  Destroys the given linked list and frees the resources occupied
 *         by it.
 *
 * \deprecated This macro is deprecated, please use xme_hal_doublyLinkedList_fini() instead!
 *
 * \note   Note that this macro will not free the memory occupied by the
 *         actual items, which is the responsibility of the code that
 *         allocated the respective memory.
 *
 * \param  name Name of the linked list to finalize.
 */
#define XME_HAL_DOUBLYLINKEDLIST_FINI(name) \
    xme_hal_doublyLinkedList_fini((xme_hal_linkedList_descriptor_t*)&(name))

/**
 * \def XME_HAL_SINGLYLINKEDLIST_ADD_ITEM
 *
 * \brief  Adds the given item to the given singly linked list.
 *
 *         The function does not allocate the memory required to store the
 *         new item.
 *
 *         On platforms with dynamic memory allocation, this function will
 *         reserve memory for maintaining the new item within the linked list.
 *         On platforms with static memory allocation, it will use one of the
 *         free slots, if available.
 *
 * \param  name Name of the singly linked list to add the item to.
 * \param  item Item to add to the singly linked list.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if the item has been added successfully.
 *          - XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *            platforms with dynamic memory mamanagement) or the linked list
 *            has reached its maximum capacity (on platforms without dynamic
 *            memory management).
 */
#define XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(name, item) \
    xme_hal_singlyLinkedList_addItem((xme_hal_linkedList_descriptor_t*)&(name), item)

/**
 * \def XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED
 *
 * \brief  Adds the given item to the given singly linked list, respecting
 *         sort order.
 *
 *         Sort order is determined by the value of type compareType at
 *         the given offset within the structure. Items are always sorted
 *         starting from the smallest value at the head of the list.
 *         In case multiple items have the same compare value, new items
 *         with the same compare value are appended after the last existing
 *         item with the same compare value.
 *
 *         The function does not allocate the memory required to store the
 *         new item.
 *
 *         On platforms with dynamic memory allocation, this function will
 *         reserve memory for maintaining the new item within the linked list.
 *         On platforms with static memory allocation, it will use one of the
 *         free slots, if available.
 *
 * \param  name Name of the singly linked list to add the item to.
 * \param  item Item to add to the singly linked list.
 * \param  offset Offset of the compare value within the item data structure.
 * \param  compareType Type of the attribute at the given offset within the
 *         item data structure.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if the item has been added successfully.
 *          - XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *            platforms with dynamic memory mamanagement) or the linked list
 *            has reached its maximum capacity (on platforms without dynamic
 *            memory management).
 */
#define XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(name, item, offset, compareType) \
    xme_hal_singlyLinkedList_addItemSorted((xme_hal_linkedList_descriptor_t*)&(name), item, offset, sizeof(compareType))

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM
 *
 * \brief  Adds the given item to the given doubly linked list.
 *
 *         The function does not allocate the memory required to store the
 *         new item.
 *
 *         On platforms with dynamic memory allocation, this function will
 *         reserve memory for maintaining the new item within the linked list.
 *         On platforms with static memory allocation, it will use one of the
 *         free slots, if available.
 *
 * \param  name Name of the doubly linked list to add the item to.
 * \param  item Item to add to the doubly linked list.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if the item has been added successfully.
 *          - XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *            platforms with dynamic memory mamanagement) or the linked list
 *            has reached its maximum capacity (on platforms without dynamic
 *            memory management).
 */
#define XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(name, item) \
    \
    /* Make compilation fail in case the list is not doubly linked.    */ \
    /* Since (name).end could be NULL, it is unsafe to dereference it. */ \
    /* Hence the false evaluation is added, which avoids the left      */ \
    /* side from being evaluated.                                      */ \
    ( \
        false ? (xme_status_t)((name).memPool[(name).end].prev) : \
        \
        xme_hal_doublyLinkedList_addItem((xme_hal_linkedList_descriptor_t*)&(name), item) \
    )

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED
 *
 * \brief  Adds the given item to the given doubly linked list, respecting
 *         sort order.
 *
 *         Sort order is determined by the value of type compareType at
 *         the given offset within the structure. Items are always sorted
 *         starting from the smallest value at the head of the list.
 *
 *         The function does not allocate the memory required to store the
 *         new item.
 *
 *         On platforms with dynamic memory allocation, this function will
 *         reserve memory for maintaining the new item within the linked list.
 *         On platforms with static memory allocation, it will use one of the
 *         free slots, if available.
 *
 * \param  name Name of the doubly linked list to add the item to.
 * \param  item Item to add to the doubly linked list.
 * \param  offset Offset of the compare value within the item data structure.
 * \param  compareType Type of the attribute at the given offset within the
 *         item data structure.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if the item has been added successfully.
 *          - XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *            platforms with dynamic memory mamanagement) or the linked list
 *            has reached its maximum capacity (on platforms without dynamic
 *            memory management).
 */
#define XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(name, item, offset, compareType) \
    \
    /* Make compilation fail in case the list is not doubly linked.    */ \
    /* Since (name).end could be NULL, it is unsafe to dereference it. */ \
    /* Hence the false evaluation is added, which avoids the left      */ \
    /* side from being evaluated.                                      */ \
    ( \
        false ? (xme_status_t)((name).memPool[(name).end].prev) : \
        \
        xme_hal_doublyLinkedList_addItemSorted((xme_hal_linkedList_descriptor_t*)&(name), item, offset, sizeof(compareType)) \
    )

/**
 * \def XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM
 *
 * \brief  Removes the given item from the singly linked list.
 *
 *         The function does not free the memory associated with the item.
 *
 * \param  name Name of the singly linked list.
 * \param  item Address of the item to remove.
 * \param  all Flag indicating whether all matching items should be removed.
 *         If set to false, only the first one will be removed.
 * \return Returns the number of items removed.
 */
#define XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(name, item, all) \
    xme_hal_singlyLinkedList_removeItem((xme_hal_linkedList_descriptor_t*)&(name), item, (bool) all)

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM
 *
 * \brief  Removes the given item from the doubly linked list.
 *
 *         The function does not free the memory associated with the item.
 *
 * \param  name Name of the doubly linked list.
 * \param  item Address of the item to remove.
 * \param  all Flag indicating whether all matching items should be removed.
 *         If set to false, only the first one will be removed.
 * \return Returns the number of items removed.
 */
#define XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(name, item, all) \
    \
    /* Make compilation fail in case the list is not doubly linked.    */ \
    /* Since (name).end could be NULL, it is unsafe to dereference it. */ \
    /* Hence the false evaluation is added, which avoids the left      */ \
    /* side from being evaluated.                                      */ \
    ( \
        false ? (xme_hal_linkedList_index_t)((name).memPool[(name).end].prev) : \
        \
        xme_hal_doublyLinkedList_removeItem((xme_hal_linkedList_descriptor_t*)&(name), item, (bool) all) \
    )

/**
 * \def XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT
 *
 * \brief  Returns the number of items in the given linked list.
 *
 * \param  name Name of the linked list.
 *
 * \return Number of items in the given linked list.
 */
#define XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(name) \
    ((xme_hal_linkedList_index_t) ((void) _XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name))), (name).count[0]))

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT
 *
 * \brief  Returns the number of items in the given linked list.
 *
 * \param  name Name of the linked list.
 *
 * \return Number of items in the given linked list.
 */
#define XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(name) \
    ((xme_hal_linkedList_index_t) ((void) _XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name))), (name).count[0]))

/**
 * \def XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX
 *
 * \brief  Returns a pointer to the item at the given index within the given
 *         singly linked list or NULL if no such item exists.
 *
 * \param  name Name of the singly linked list.
 * \param  index Zero-based index of the item within the singly linked list to
 *         retrieve.
 *
 * \return Returns a pointer to the item corresponding to the given index or
 *         NULL if no such item exists.
 */
#define XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(name, index) \
    xme_hal_singlyLinkedList_itemFromIndex((xme_hal_linkedList_descriptor_t*)&(name), index)

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX
 *
 * \brief  Returns a pointer to the item at the given index within the given
 *         doubly linked list or NULL if no such item exists.
 *
 * \param  name Name of the singly doubly list.
 * \param  index Zero-based index of the item within the doubly linked list to
 *         retrieve.
 *
 * \return Returns a pointer to the item corresponding to the given index or
 *         NULL if no such item exists.
 */
#define XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX(name, index) \
    \
    /* Make compilation fail in case the list is not doubly linked.    */ \
    /* Since (name).end could be NULL, it is unsafe to dereference it. */ \
    /* Hence the false evaluation is added, which avoids the left      */ \
    /* side from being evaluated.                                      */ \
    ( \
        false ? (void *)(uintptr_t )((name).memPool[(name).end].prev) : \
        \
        xme_hal_doublyLinkedList_itemFromIndex((xme_hal_linkedList_descriptor_t*)&(name), index) \
    )

/**
 * \def    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
 *
 * \brief  Begins a block for iterating over all items of the given linked list.
 *
 * \note   A block started with XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN() must be ended
 *         with XME_HAL_SINGLYLINKEDLIST_ITERATE_END() at the same scope.
 *
 * \param  name Name of the singly linked list to iterate over.
 * \param  loopItemType Base type of the items that are iterated over.
 *         This parameter should be the type of the linked list items, not a
 *         pointer to them.
 * \param  loopItem Name of the loop item variable. This variable can be used
 *         from within the iterator body.
 */
#define XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(name, loopItemType, loopItem) \
    do \
    { \
        xme_hal_linkedList_index_t __nextLoopElement; \
        xme_hal_singlyLinkedList_genericElement_t* __memPool = (xme_hal_singlyLinkedList_genericElement_t*) &(name).memPool; \
        xme_hal_linkedList_index_t __loopElement = (name).head; \
        if (!_XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name)))) { break; } \
        for ( ; 0U != __loopElement ; __loopElement = __nextLoopElement) \
        { \
            loopItemType* loopItem; \
            __nextLoopElement = __memPool[__loopElement-1U].next; \
            loopItem = (loopItemType*) __memPool[__loopElement-1U].item; \
            if (__memPool[__loopElement-1U].next == __loopElement) { __nextLoopElement = 0U; } \
            {

/**
 * \def    XME_HAL_SINGLYLINKEDLIST_ITERATE_END
 *
 * \brief  Ends a block for iterating over all items of the given linked list.
 *
 * \note   A block ended with XME_HAL_LINKEDLIST_ITERATE_END() must be started
 *         with XME_HAL_LINKEDLIST_ITERATE_BEGIN() at the same scope.
 */
#define XME_HAL_SINGLYLINKEDLIST_ITERATE_END() \
            } \
        } \
    } while (0)

/**
 * \def    XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN
 *
 * \brief  Begins a block for iterating over all items of the given linked list.
 *
 * \note   A block started with XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN() must be ended
 *         with XME_HAL_DOUBLYLINKEDLIST_ITERATE_END() at the same scope.
 *
 * \param  name Name of the doubly linked list to iterate over.
 * \param  loopItemType Base type of the items that are iterated over.
 *         This parameter should be the type of the linked list items, not a
 *         pointer to them.
 * \param  loopItem Name of the loop item variable. This variable can be used
 *         from within the iterator body.
 */
#define XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(name, loopItemType, loopItem) \
    do \
    { \
        xme_hal_linkedList_index_t __nextLoopElement; \
        xme_hal_doublyLinkedList_genericElement_t* __memPool = (xme_hal_doublyLinkedList_genericElement_t*) &(name).memPool; \
        xme_hal_linkedList_index_t __loopElement = (name).head; \
        if (!_XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name)))) { break; } \
        for ( ; 0U != __loopElement ; __loopElement = __nextLoopElement) \
        { \
            loopItemType* loopItem; \
            __nextLoopElement = __memPool[__loopElement-1U].next; \
            loopItem = (loopItemType*) __memPool[__loopElement-1U].item; \
            if (__memPool[__loopElement-1U].next == __loopElement) { __nextLoopElement = 0U; } \
            {

/**
 * \def    XME_HAL_DOUBLYLINKEDLIST_ITERATE_END
 *
 * \brief  Ends a block for iterating over all items of the given linked list.
 *
 * \note   A block ended with XME_HAL_DOUBLYLINKEDLIST_ITERATE_END() must be started
 *         with XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN() at the same scope.
 */
#define XME_HAL_DOUBLYLINKEDLIST_ITERATE_END() \
            } \
        } \
    } while (0)

/**
 * \def    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN
 *
 * \brief  Begins a block for iterating over all items of the given doubly
 *         linked list in reverse order.
 *
 * \note   A block started with XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN()
 *         must be ended with XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END() at
 *         the same scope.
 *
 * \param  name Name of the doubly linked list to iterate over.
 * \param  loopItemType Base type of the items that are iterated over.
 *         This parameter should be the type of the linked list items, not a
 *         pointer to them.
 * \param  loopItem Name of the loop item variable. This variable can be used
 *         from within the iterator body.
 */
#define XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(name, loopItemType, loopItem) \
    do \
    { \
        xme_hal_linkedList_index_t __prevLoopElement; \
        xme_hal_doublyLinkedList_genericElement_t* __memPool = (xme_hal_doublyLinkedList_genericElement_t*) &(name).memPool; \
        xme_hal_linkedList_index_t __loopElement = (name).end; \
        if (!_XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name)))) { break; } \
        for ( ; 0U != __loopElement ; __loopElement = __prevLoopElement) \
        { \
            loopItemType* loopItem; \
            __prevLoopElement = __memPool[__loopElement-1U].prev; \
            loopItem = (loopItemType*) __memPool[__loopElement-1U].item; \
            if (__memPool[__loopElement-1U].prev == __loopElement) { __prevLoopElement = 0U; } \
            {

/**
 * \def    XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END
 *
 * \brief  Ends a block for iterating over all items of the given doubly
 *         linked list in reverse order.
 *
 * \note   A block started with XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN()
 *         must be ended with XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END() at
 *         the same scope.
 */
#define XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END() \
            } \
        } \
    } while (0)

/**
 * @}
 */

#endif // #ifndef XME_HAL_LINKEDLIST_ARCH_H
