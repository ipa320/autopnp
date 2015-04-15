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
 * $Id: linkedList_arch.h 5062 2013-09-16 08:47:39Z gulati $
 */

/**
 * \file
 *         Linked list abstraction (architecture specific part: generic OS
 *         based implementation).
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

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \struct xme_hal_linkedList_descriptor_t
 *
 * \brief  Linked list descriptor.
 *
 * \details Defines the internal representation of a linked list.
 */
typedef struct
{
    void* head;                          ///< Link to the first element in the list or NULL if the list is empty.
    void* end;                           ///< Link to the last element in the list or NULL if the list is empty.
    xme_hal_linkedList_index_t count[1]; ///< Number of elements in the list (for reasons of efficiency).
    /* Declaring the 'count' member as an array is a "hack"   */
    /* to ensure that a valid number is passed in maxElements */
    /* and that the maxElements argument is not unused.       */
#if defined(DEBUG) && !defined(DOXYGEN)
    uint8_t _isInitialized; ///< Magic number used to heuristically determine whether the linked list has been properly initialized.
#endif // #if defined(DEBUG) && !defined(DOXYGEN)
}
xme_hal_linkedList_descriptor_t;

/**
 * \struct xme_hal_singlyLinkedList_genericElement_s
 *
 * \brief  Singly Linked list descriptor.
 *
 * \details Defines the internal representation of a singly linked list.
 */
struct xme_hal_singlyLinkedList_genericElement_s
{
    struct xme_hal_singlyLinkedList_genericElement_s* next; ///< the pointer to the next element. 
    const void* item; ///< the element stored in the linked list. 
};

/**
 * \typedef xme_hal_singlyLinkedList_genericElement_t
 *
 * \details Defines the typedef of internal representation of a singly linked list.
 */
typedef struct xme_hal_singlyLinkedList_genericElement_s xme_hal_singlyLinkedList_genericElement_t;

/**
 * \struct xme_hal_doublyLinkedList_genericElement_s
 *
 * \brief  Doubly Linked list descriptor.
 *
 *\details Defines the internal representation of a doubly linked list.
 */
struct xme_hal_doublyLinkedList_genericElement_s
{
    struct xme_hal_doublyLinkedList_genericElement_s* next; ///< the next element in the list. 
    const void* item; ///< the item stored in the linked list. 
    struct xme_hal_doublyLinkedList_genericElement_s* prev; ///< the previous element in the list. 
};

/**
 * \typedef xme_hal_doublyLinkedList_genericElement_t
 *
 * \details Defines the typedef of internal representation of a doubly linked list.
 */
typedef struct xme_hal_doublyLinkedList_genericElement_s xme_hal_doublyLinkedList_genericElement_t;

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def xme_hal_singlyLinkedList_t
 *
 * \brief  Defines a singly linked list with a given maximum size.
 *
 * \note   Note that the above definition of this macro must match the
 *         definition of the xme_hal_linkedList_descriptor_t type (except for
 *         the type of the head element).
 *
 * \param[in] maxItems Maximum number of items in the linked list.
 *            On platforms without dynamic memory allocation, this parameter
 *            defines the maximum number of items in the linked list. The
 *            memory required to hold these items will then be statically
 *            allocated.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define xme_hal_singlyLinkedList_t(maxItems) \
    struct \
    { \
        xme_hal_singlyLinkedList_genericElement_t* head; \
        xme_hal_singlyLinkedList_genericElement_t* end; \
        /* Declaring the 'count' member as an array is a "hack"   */ \
        /* to ensure that a valid number is passed in maxElements */ \
        /* and that the maxElements argument is not unused.       */ \
        xme_hal_linkedList_index_t count[(1 == maxItems) ? 1 : 1]; \
        uint8_t _isInitialized; \
    }
#else // #if defined(DEBUG) && !defined(DOXYGEN)
    // Release mode
    #define xme_hal_singlyLinkedList_t(maxItems) \
    struct \
    { \
        xme_hal_singlyLinkedList_genericElement_t* head; \
        xme_hal_singlyLinkedList_genericElement_t* end; \
        /* Declaring the 'count' member as an array is a "hack"   */ \
        /* to ensure that a valid number is passed in maxElements */ \
        /* and that the maxElements argument is not unused.       */ \
        xme_hal_linkedList_index_t count[(1 == maxItems) ? 1 : 1]; \
    }
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/**
 * \def xme_hal_doublyLinkedList_t
 *
 * \brief  Defines a doubly linked list with a given maximum size.
 *
 * \note   Note that the above definition of this macro must match the
 *         definition of the xme_hal_linkedList_descriptor_t type (except for
 *         the type of the head element).
 *
 * \param[in] maxItems Maximum number of items in the linked list.
 *            On platforms without dynamic memory allocation, this parameter
 *            defines the maximum number of items in the linked list. The
 *            memory required to hold these items will then be statically
 *            allocated.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define xme_hal_doublyLinkedList_t(maxItems) \
    struct \
    { \
        xme_hal_doublyLinkedList_genericElement_t* head; \
        xme_hal_doublyLinkedList_genericElement_t* end; \
        /* Declaring the 'count' member as an array is a "hack"   */ \
        /* to ensure that a valid number is passed in maxElements */ \
        /* and that the maxElements argument is not unused.       */ \
        xme_hal_linkedList_index_t count[(1 == maxItems) ? 1 : 1]; \
        uint8_t _isInitialized; \
    }
#else // #if defined(DEBUG) && !defined(DOXYGEN)
    // Release mode
    #define xme_hal_doublyLinkedList_t(maxItems) \
    struct \
    { \
        xme_hal_doublyLinkedList_genericElement_t* head; \
        xme_hal_doublyLinkedList_genericElement_t* end; \
        /* Declaring the 'count' member as an array is a "hack"   */ \
        /* to ensure that a valid number is passed in maxElements */ \
        /* and that the maxElements argument is not unused.       */ \
        xme_hal_linkedList_index_t count[(1 == maxItems) ? 1 : 1]; \
    }
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/**
 * \def XME_HAL_SINGLYLINKEDLIST_INIT
 *
 * \brief  Initializes the given singly linked list.
 *         This macro must be called for a linked list before it can be used.
 *
 * \param[in] name Name of the linked list to initialize.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define XME_HAL_SINGLYLINKEDLIST_INIT(name) \
    do { (name).head = (name).end = 0; (name).count[0] = 0; (name)._isInitialized = XME_HAL_LINKEDLIST_MAGIC; } while (0)
#else // #if defined(DEBUG) && !defined(DOXYGEN)
    // Release mode
    #define XME_HAL_SINGLYLINKEDLIST_INIT(name) \
    do { (name).head = (name).end = 0; (name).count[0] = 0; } while (0)
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_INIT
 *
 * \brief  Initializes the given doubly singly linked list.
 *         This macro must be called for a linked list before it can be used.
 *
 * \param[in] name Name of the linked list to initialize.
 */
#if defined(DEBUG) && !defined(DOXYGEN)
    // Debug mode
    #define XME_HAL_DOUBLYLINKEDLIST_INIT(name) \
    do { (name).head = (name).end = 0; (name).count[0] = 0; (name)._isInitialized = XME_HAL_LINKEDLIST_MAGIC; } while (0)
#else // #if defined(DEBUG) && !defined(DOXYGEN)
    // Release mode
    #define XME_HAL_DOUBLYLINKEDLIST_INIT(name) \
    do { (name).head = (name).end = 0; (name).count[0] = 0; } while (0)
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

/**
 * \def XME_HAL_SINGLYLINKEDLIST_FINI
 *
 * \brief  Destroys the given singly linked list and frees the resources
 *         occupied by it.
 *
 * \deprecated This macro is deprecated, please use xme_hal_singlyLinkedList_fini() instead!
 *
 * \note   Note that this macro will not free the memory occupied by the
 *         actual items, which is the responsibility of the code that
 *         allocated the respective memory.
 *
 * \param[in] name Name of the linked list to finalize.
 */
#define XME_HAL_SINGLYLINKEDLIST_FINI(name) \
    xme_hal_singlyLinkedList_fini((void*)&(name))

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_FINI
 *
 * \brief  Destroys the given doubly linked list and frees the resources occupied
 *         by it.
 *
 * \deprecated This macro is deprecated, please use xme_hal_doublyLinkedList_fini() instead!
 *
 * \note   Note that this macro will not free the memory occupied by the
 *         actual items, which is the responsibility of the code that
 *         allocated the respective memory.
 *
 * \param[in] name Name of the linked list to finalize.
 */
#define XME_HAL_DOUBLYLINKEDLIST_FINI(name) \
    xme_hal_doublyLinkedList_fini((void*)&(name))

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
 * \deprecated This macro is deprecated, please use xme_hal_singlyLinkedList_addItem() instead!
 *
 * \param[in] name Name of the singly linked list to add the item to.
 * \param[in] item Item to add to the singly linked list.
 *
 * \retval  XME_STATUS_SUCCESS if the item has been added successfully.
 * \retval  XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *          platforms with dynamic memory mamanagement) or the linked list
 *          has reached its maximum capacity (on platforms without dynamic
 *          memory management).
 */
#define XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(name, item) \
    xme_hal_singlyLinkedList_addItem((void*)&(name), item)

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
 * \deprecated This macro is deprecated, please use
 *             xme_hal_singlyLinkedList_addItemOrdered() instead!
 *
 * \param[in] name Name of the singly linked list to add the item to.
 * \param[in] item Item to add to the singly linked list.
 * \param[in] offset Offset of the compare value within the item data
 *            structure.
 * \param[in] compareType Type of the attribute at the given offset within the
 *            item data structure.
 *
 * \retval  XME_STATUS_SUCCESS if the item has been added successfully.
 * \retval  XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *          platforms with dynamic memory mamanagement) or the linked list
 *          has reached its maximum capacity (on platforms without dynamic
 *          memory management).
 */
#define XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(name, item, offset, compareType) \
    xme_hal_singlyLinkedList_addItemSorted((void*)&(name), item, offset, sizeof(compareType))

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
 * \deprecated This macro is deprecated, please use
 *             xme_hal_doublyLinkedList_addItem() instead!
 *
 * \param[in] name Name of the doubly linked list to add the item to.
 * \param[in] item Item to add to the doubly linked list.
 *
 * \retval  XME_STATUS_SUCCESS if the item has been added successfully.
 * \retval  XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *          platforms with dynamic memory mamanagement) or the linked list
 *          has reached its maximum capacity (on platforms without dynamic
 *          memory management).
 */
#define XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(name, item) \
    \
    /* Make compilation fail in case the list is not doubly linked.    */ \
    /* Since (name).end could be NULL, it is unsafe to dereference it. */ \
    /* Hence the false evaluation is added, which avoids the left      */ \
    /* side from being evaluated.                                      */ \
    ( \
        0 ? *(xme_status_t*)((name).end->prev) : \
        \
        xme_hal_doublyLinkedList_addItem((void*)&(name), item) \
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
 * \deprecated This macro is deprecated, please use
 *             xme_hal_doublyLinkedList_addItemOrdered() instead!
 *
 * \param[in] name Name of the doubly linked list to add the item to.
 * \param[in] item Item to add to the doubly linked list.
 * \param[in] offset Offset of the compare value within the item data
 *            structure.
 * \param[in] compareType Type of the attribute at the given offset within the
 *            item data structure.
 *
 * \retval  XME_STATUS_SUCCESS if the item has been added successfully.
 * \retval  XME_STATUS_OUT_OF_RESOURCES if memory allocation failed (on
 *          platforms with dynamic memory mamanagement) or the linked list
 *          has reached its maximum capacity (on platforms without dynamic
 *          memory management).
 */
#define XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM_SORTED(name, item, offset, compareType) \
    \
    /* Make compilation fail in case the list is not doubly linked.    */ \
    /* Since (name).end could be NULL, it is unsafe to dereference it. */ \
    /* Hence the false evaluation is added, which avoids the left      */ \
    /* side from being evaluated.                                      */ \
    ( \
        0 ? *(xme_status_t*)((name).end->prev) : \
        \
        xme_hal_doublyLinkedList_addItemSorted((void*)&(name), item, offset, sizeof(compareType)) \
    )

/**
 * \def XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM
 *
 * \brief  Removes the given item from the singly linked list.
 *
 *         The function does not free the memory associated with the item.
 *
 * \deprecated This macro is deprecated, please use xme_hal_singlyLinkedList_removeItem() instead!
 *
 * \param[in] name Name of the singly linked list.
 * \param[in] item Address of the item to remove.
 * \param[in] all Flag indicating whether all matching items should be removed.
 *            If set to false, only the first one will be removed.
 *
 * \return Returns the number of items removed.
 */
#define XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(name, item, all) \
    xme_hal_singlyLinkedList_removeItem((void*)&(name), item, all)

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM
 *
 * \brief  Removes the given item from the doubly linked list.
 *
 *         The function does not free the memory associated with the item.
 *
 * \deprecated This macro is deprecated, please use xme_hal_doublyLinkedList_removeItem() instead!
 *
 * \param[in] name Name of the doubly linked list.
 * \param[in] item Address of the item to remove.
 * \param[in] all Flag indicating whether all matching items should be removed.
 *            If set to false, only the first one will be removed.
 *
 * \return Returns the number of items removed.
 */
#define XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(name, item, all) \
    \
    /* Make compilation fail in case the list is not doubly linked.    */ \
    /* Since (name).end could be NULL, it is unsafe to dereference it. */ \
    /* Hence the false evaluation is added, which avoids the left      */ \
    /* side from being evaluated.                                      */ \
    ( \
        0 ? *(uint16_t*)((name).end->prev) : \
        \
        xme_hal_doublyLinkedList_removeItem((void*)&(name), item, all) \
    )

/**
 * \def XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT
 *
 * \brief  Returns the number of items in a singly linked list.
 *
 * \details The runtime of this macro is guaranteed to be O(1).
 *
 * \deprecated This macro is deprecated, please use xme_hal_singlyLinkedList_getItemCount() instead!
 *
 * \param[in] name Name of the linked list.
 *
 * \return Returns the number of elements in the singly linked list.
 */
#define XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(name) \
    ((xme_hal_linkedList_index_t) ((void) _XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name))), (name).count[0]))

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT
 *
 * \brief  Returns the number of items a doubly linked list.
 *
 * \details The runtime of this macro is guaranteed to be O(1).
 *
 * \deprecated This macro is deprecated, please use xme_hal_doublyLinkedList_getItemCount() instead!
 *
 * \param[in] name Name of the linked list.
 *
 * \return Returns the number of elements in the doubly linked list.
 */
#define XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(name) \
    ((xme_hal_linkedList_index_t) ((void) _XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name))), (name).count[0]))

/**
 * \def XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX
 *
 * \brief  Returns a pointer to the item at the given index within the given
 *         singly linked list or NULL if no such item exists.
 *
 * \param[in] name Name of the singly linked list.
 * \param[in] index Zero-based index of the item within the singly linked list
 *            to retrieve.
 *
 * \return Returns a pointer to the item corresponding to the given index or
 *         NULL if no such item exists.
 */
#define XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(name, index) \
    xme_hal_singlyLinkedList_itemFromIndex((void*)&(name), index)

/**
 * \def XME_HAL_DOUBLYLINKEDLIST_ITEM_FROM_INDEX
 *
 * \brief  Returns a pointer to the item at the given index within the given
 *         doubly linked list or NULL if no such item exists.
 *
 * \param[in] name Name of the singly doubly list.
 * \param[in] index Zero-based index of the item within the doubly linked list
 *            to retrieve.
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
        0 ? (void*)((name).end->prev) : \
        \
        xme_hal_doublyLinkedList_itemFromIndex((void*)&(name), index) \
    )

/**
 * \def    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
 *
 * \brief  Begins a block for iterating over all items of the given linked list.
 *
 * \note   A block started with XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN() must be ended
 *         with XME_HAL_SINGLYLINKEDLIST_ITERATE_END() at the same scope.
 *
 * \param[in] name Name of the singly or doubly linked list to iterate over.
 * \param[in] loopItemType Base type of the items that are iterated over.
 *            This parameter should be the type of the linked list items, not a
 *            pointer to them.
 * \param[in] loopItem Name of the loop item variable. This variable can be
 *            used from within the iterator body.
 */
#define XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(name, loopItemType, loopItem) \
    do \
    { \
        xme_hal_singlyLinkedList_genericElement_t* __nextLoopElement; \
        xme_hal_singlyLinkedList_genericElement_t* __loopElement; \
        if (!_XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name)))) { break; } \
        __loopElement = (xme_hal_singlyLinkedList_genericElement_t *)(name).head; \
        for(; NULL != __loopElement; __loopElement = __nextLoopElement) \
        { \
            loopItemType* loopItem = (loopItemType *)(__loopElement->item); \
            __nextLoopElement = __loopElement->next; \
            {

/**
 * \def    XME_HAL_SINGLYLINKEDLIST_ITERATE_END
 *
 * \brief  Ends a block for iterating over all items of the given linked list.
 *
 * \note   A block ended with XME_HAL_SINGLYLINKEDLIST_ITERATE_END() must be started
 *         with XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN() at the same scope.
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
 * \param[in] name Name of the singly or doubly linked list to iterate over.
 * \param[in] loopItemType Base type of the items that are iterated over.
 *            This parameter should be the type of the linked list items, not a
 *            pointer to them.
 * \param[in] loopItem Name of the loop item variable. This variable can be
 *            used from within the iterator body.
 */
#define XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN(name, loopItemType, loopItem) \
    do \
    { \
        xme_hal_doublyLinkedList_genericElement_t* __nextLoopElement; \
        xme_hal_doublyLinkedList_genericElement_t* __loopElement; \
        if (!_XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name)))) { break; } \
        __loopElement = (xme_hal_doublyLinkedList_genericElement_t *)(name).head; \
        for (; NULL != __loopElement; __loopElement = __nextLoopElement) \
        { \
            loopItemType* loopItem = (loopItemType *)(__loopElement->item); \
            __nextLoopElement = __loopElement->next; \
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
 * \param[in] name Name of the doubly linked list to iterate over.
 * \param[in] loopItemType Base type of the items that are iterated over.
 *            This parameter should be the type of the linked list items, not a
 *            pointer to them.
 * \param[in] loopItem Name of the loop item variable. This variable can be
 *            used from within the iterator body.
 */
#define XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN(name, loopItemType, loopItem) \
    do \
    { \
        xme_hal_doublyLinkedList_genericElement_t* __prevLoopElement; \
        xme_hal_doublyLinkedList_genericElement_t* __loopElement; \
        if (!_XME_ASSERT_HANDLER(XME_HAL_LINKEDLIST_MAGIC_CHECK(&(name)))) { break; } \
        __loopElement = (xme_hal_doublyLinkedList_genericElement_t *)(name).end; \
        for (; NULL != __loopElement; __loopElement = __prevLoopElement) \
        { \
            loopItemType* loopItem = (loopItemType *)(__loopElement->item); \
            __prevLoopElement = __loopElement->prev; \
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
