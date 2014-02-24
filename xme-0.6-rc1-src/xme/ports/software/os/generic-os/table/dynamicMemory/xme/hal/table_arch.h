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
 * $Id: table_arch.h 5129 2013-09-19 16:16:32Z geisinger $
 */

/**
 * \file
 *         Table abstraction (architecture specific part: generic OS based
 *         implementation).
 *
 */

#ifndef XME_HAL_TABLE_ARCH_H
#define XME_HAL_TABLE_ARCH_H

#ifndef XME_HAL_TABLE_H
    #error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_HAL_TABLE_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_hal_table_rowHandleBaseType_t
 *
 * \brief  Numeric base type used for table handle arithmetic.
 *
 *         It must be asserted that this type is able to express at least the
 *         values XME_HAL_TABLE_INVALID_ROW_HANDLE and XME_HAL_TABLE_MAX_ROW_HANDLE.
 */
typedef uint16_t xme_hal_table_rowHandleBaseType_t;

/**
 * \struct xme_hal_table_arrayStruct_t
 *
 * \brief  Table structure.
 *
 *         The table structure is the internal representation of a table.
 *
 * \note   Note that the definition of xme_hal_table_arrayStruct_t must match
 *         the definition of the XME_HAL_TABLE() macro (except for the type of
 *         the array element).
 */
typedef struct
{
    // Declaring the 'count' member as an array is a "hack"
    // to ensure that a valid number is passed in maxElements
    // in the XME_HAL_TABLE() macro and that the maxElements
    // argument is not unused.
    uint16_t count[1];                   ///< Number of items in the table.
    xme_hal_table_rowHandle_t maxHandle; ///< Highest valid handle value of an item in the table.
    uint8_t power;                       ///< Current dynamically allocated size of the table, in powers of two plus 1 (i.e., size = 2^(power-1)).
    void** array;                        ///< Table rows.
}
xme_hal_table_arrayStruct_t;

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def XME_HAL_TABLE
 *
 * \brief  Defines an table with a given maximum size.
 *
 * \note   Note that the above definition of this macro must match the
 *         definition of the xme_hal_table_arrayStruct_t type (except for the
 *         type of the array element).
 *
 * \param  itemType Type of the items in the table.
 * \param  name Name of the table.
 * \param  maxElements Maximum number of elements in the table.
 *         On platforms without dynamic memory allocation, this parameter
 *         defines the maximum number of items in the table. The memory
 *         required to hold these items will then be statically allocated.
 */
#define XME_HAL_TABLE(itemType, name, maxElements) \
    struct \
    { \
        /* Declaring the 'count' member as an array is a "hack"   */ \
        /* to ensure that a valid number is passed in maxElements */ \
        /* and that the maxElements argument is not unused.       */ \
        uint16_t count[(1 == maxElements) ? 1 : 1]; \
        xme_hal_table_rowHandle_t maxHandle; \
        uint8_t power; \
        itemType** array; \
    } name

/**
 * \def XME_HAL_TABLE_INIT
 *
 * \brief  Initializes the given table.
 *         This macro must be called for a table before it can be used.
 *
 * \param  name Name of the table to initialize.
 */
#define XME_HAL_TABLE_INIT(name) \
    do \
    { \
        (name).count[0] = 0; \
        (name).maxHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE; \
        (name).power = 0; \
        (name).array = NULL; \
    } while (0)

/**
 * \def XME_HAL_TABLE_FINI
 *
 * \brief  Destroys the given table and frees the resources occupied by it.
 *
 * \note   Since the table manages the memory occupied by the table items,
 *         this macro will free all associated memory. Any existing pointer
 *         to a table item may be invalid after a call to this macro.
 *
 * \param  name Name of the table to finalize.
 */
#define XME_HAL_TABLE_FINI(name) \
    do \
    { \
        /* Testing for existence of count[0] is a heuristic type check of the input parameter */ \
        (void)(name).count[0]; \
        xme_hal_table_fini((xme_hal_table_arrayStruct_t*)&(name)); \
    } \
    while (0)

/**
 * \def XME_HAL_TABLE_CLEAR
 *
 * \brief  Frees the elements of the given table.
 *
 * \warning This macro is deprecated, use ::xme_hal_table_clear() instead.
 *
 * \param  name Name of the table to clear.
 */
#define XME_HAL_TABLE_CLEAR(name) \
    XME_HAL_TABLE_FINI(name)

/**
 * \def XME_HAL_TABLE_ADD_ITEM
 *
 * \brief  Adds a new item to the given table, returning the handle to that
 *         item.
 *
 * \details On platforms with dynamic memory allocation, this function will
 *          reserve the memory required to hold the new item and resize the
 *          table to fit the new item. On systems with static memory
 *          allocation, it will use the next free slot and return a handle
 *          to that slot. In any case, the memory used for the new item will
 *          be initialized with all zeros.
 *
 * \param  name Name of the table to add an item to.
 *
 * \return Returns the table row handle for the newly created item or
 *         XME_HAL_TABLE_INVALID_ROW_HANDLE if memory allocation failed (on
 *         platforms with dynamic memory mamanagement) or the table has
 *         reached its maximum capacity (on platforms without dynamic memory
 *         management).
 */
#define XME_HAL_TABLE_ADD_ITEM(name) \
    xme_hal_table_addItem((xme_hal_table_arrayStruct_t*)&(name), sizeof(**(name).array))

/**
 * \def XME_HAL_TABLE_REMOVE_ITEM
 *
 * \brief  Removes the item corresponding to the given handle from the table.
 *
 * \param  name Name of the table.
 * \param  handle Table row handle of the item to remove.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the item has been successfully removed.
 *          - XME_CORE_STATUS_INVALID_HANDLE if the given handle was invalid.
 */
#define XME_HAL_TABLE_REMOVE_ITEM(name, handle) \
    xme_hal_table_removeItem((xme_hal_table_arrayStruct_t*)&(name), handle, sizeof(**(name).array))

/**
 * \def XME_HAL_TABLE_ITEM_COUNT
 *
 * \brief  Returns the number of items in the given table.
 *
 * \param  name Name of the table.
 *
 * \return Number of items in the given table.
 */
#define XME_HAL_TABLE_ITEM_COUNT(name) \
    ((name).count[0])

/**
 * \def XME_HAL_TABLE_ITEM_FROM_HANDLE
 *
 * \brief  Returns a pointer to the table item for the given table row handle
 *         or NULL if no such item exists.
 *
 * \param  name Name of the table.
 * \param  handle Table row handle of the item to retrieve.
 *
 * \return Returns a pointer to the item corresponding to the given table row
 *         handle or NULL if no such item exists.
 */
#define XME_HAL_TABLE_ITEM_FROM_HANDLE(name, handle) \
    (((xme_hal_table_rowHandle_t)(handle) > XME_HAL_TABLE_INVALID_ROW_HANDLE && (xme_hal_table_rowHandle_t)(handle) <= (name).maxHandle) ? (name).array[((xme_hal_table_rowHandleBaseType_t)handle)-1] : NULL)

/**
 * \def    XME_HAL_TABLE_ITERATE_BEGIN
 *
 * \brief  Begins a block for iterating over all items of the given table.
 *
 * \note   A block started with XME_HAL_TABLE_ITERATE_BEGIN() must be ended
 *         with XME_HAL_TABLE_ITERATE_END() at the same scope.
 *
 * \warning Addition and removal of table items is possible from within the
 *          iterator body. However, it is undefined whether or not newly added
 *          items will appear in subsequent iterations.
 *
 * \param  name Name of the table to iterate over.
 * \param  loopHandleType Type of the loop handle variable, usually
 *         xme_hal_table_rowHandleBaseType_t or a type defined to it.
 * \param  loopHandle Name of the loop handle variable. This variable can be
 *         used from within the iterator body.
 * \param  loopItemType Base type of the items that are iterated over.
 *         This parameter should be the type of the table items, not a pointer
 *         to them.
 * \param  loopItem Name of the loop item variable. This variable can be used
 *         from within the iterator body.
 */
#define XME_HAL_TABLE_ITERATE_BEGIN(name, loopHandleType, loopHandle, loopItemType, loopItem) \
    do \
    { \
        loopHandleType loopHandle; \
        for (loopHandle=(loopHandleType)1; loopHandle<=(loopHandleType)(name).maxHandle; loopHandle = (loopHandleType)(((xme_hal_table_rowHandleBaseType_t)loopHandle)+1)) \
        { \
            loopItemType* loopItem; \
            if (NULL != (loopItem = (name).array[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1])) \
            {

/**
 * \def    XME_HAL_TABLE_ITERATE_END
 *
 * \brief  Ends a block for iterating over all items of the given table.
 *
 * \note   A block ended with XME_HAL_TABLE_ITERATE_END() must be started
 *         with XME_HAL_TABLE_ITERATE_BEGIN() at the same scope.
 */
#define XME_HAL_TABLE_ITERATE_END() \
            } \
        } \
    } while (0)

/**
 * \def    XME_HAL_TABLE_ITERATE_REVERSE_BEGIN
 *
 * \brief  Begins a block for iterating over all items of the given table in
 *         reverse order.
 *
 * \note   A block started with XME_HAL_TABLE_ITERATE_REVERSE_BEGIN() must be
 *         ended with XME_HAL_TABLE_ITERATE_REVERSE_END() at the same scope.
 *
 * \warning Addition and removal of table items is possible from within the
 *          iterator body. However, it is undefined whether or not newly added
 *          items will appear in subsequent iterations.
 *
 * \param  name Name of the table to iterate over.
 * \param  loopHandleType Type of the loop handle variable, usually
 *         xme_hal_table_rowHandleBaseType_t or a type defined to it.
 * \param  loopHandle Name of the loop handle variable. This variable can be
 *         used from within the iterator body.
 * \param  loopItemType Base type of the items that are iterated over.
 *         This parameter should be the type of the table items, not a pointer
 *         to them.
 * \param  loopItem Name of the loop item variable. This variable can be used
 *         from within the iterator body.
 */
#define XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(name, loopHandleType, loopHandle, loopItemType, loopItem) \
    do \
    { \
        loopHandleType loopHandle; \
        for (loopHandle=(loopHandleType)(name).maxHandle; loopHandle>=(loopHandleType)1; loopHandle = (loopHandleType)(((xme_hal_table_rowHandleBaseType_t)loopHandle)-1)) \
        { \
            loopItemType* loopItem; \
            if (NULL != (loopItem = (name).array[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1])) \
            { \

/**
 * \def    XME_HAL_TABLE_ITERATE_REVERSE_END
 *
 * \brief  Ends a block for iterating over all items of the given table in
 *         reverse order.
 *
 * \note   A block ended with XME_HAL_TABLE_ITERATE_REVERSE_END() must be
 *         started with XME_HAL_TABLE_ITERATE_REVERSE_BEGIN() at the same
 *         scope.
 */
#define XME_HAL_TABLE_ITERATE_REVERSE_END() \
            } \
        } \
    } while (0)

/**
 * \def    XME_HAL_TABLE_GET_NEXT
 *
 * \brief  Search for the next element of the given table matching the given
 *         condition, starting at the element specified by loopHandle.
 *
 * \param  name Name of the table to iterate over.
 * \param  loopHandleType Type of the loop handle variable, usually
 *         xme_hal_table_rowHandleBaseType_t or a type defined to it.
 * \param  loopHandle Name of the loop handle variable. This variable has to
 *         be defined and initialized by the user and can be used from within
 *         the condition. The value of this variable determines at which
 *         element the scan over the table starts. To start at the beginning
 *         of the table, set the variable to XME_HAL_TABLE_INVALID_ROW_HANDLE.
 * \param  loopItemType Base type of the items that are iterated over.
 *         This parameter should be the type of the table items, not a pointer
 *         to them.
 * \param  loopItem Name of the loop item variable. This variable can be used
 *         from within the condition.
 * \param  condition Condition that the element shall satisfy. Must return a
 *         Boolean value. The variables specified in the loopHandle and
 *         loopItem parameters are available from within the condition.
 */
#define XME_HAL_TABLE_GET_NEXT(name, loopHandleType, loopHandle, loopItemType, loopItem, condition) \
    do \
    { \
        while (loopHandle <= (loopHandleType)(name).maxHandle) \
        { \
            loopHandle = (loopHandleType)(((xme_hal_table_rowHandleBaseType_t)loopHandle) + 1); \
            if (loopHandle <= (loopHandleType)(name).maxHandle && NULL != (loopItem = (name).array[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1])) \
            { \
                if (condition) \
                { \
                    break; \
                } \
            } \
        } \
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == loopHandle || loopHandle > (loopHandleType)(name).maxHandle) \
        { \
            loopHandle = (loopHandleType)XME_HAL_TABLE_INVALID_ROW_HANDLE; \
            loopItem = NULL; \
        } \
    } while (0)

/**
 * \def    XME_HAL_TABLE_BUBBLESORT
 *
 * \brief  Sorts all items of the given table.
 *
 * \param  name Name of the table to sort.
 * \param  loopItemType Base type of the items that are sorted.
 *         This parameter should be the type of the table items, not a pointer
 *         to them.
 * \param  offset Offset of the struct/array-member that should be used for the comparison.
 * \param  compareType Type of the struct/array-member that should be used for the comparison.
 */
#define XME_HAL_TABLE_BUBBLESORT(name, loopItemType, offset, compareType) \
    do \
    { \
        xme_hal_table_rowHandleBaseType_t outerLoopHandle, innerLoopHandle; \
        loopItemType* loopItem1; \
        loopItemType* loopItem2; \
        loopItemType tempItem; \
        compareType* compareItem1; \
        compareType* compareItem2; \
        for (outerLoopHandle=(xme_hal_table_rowHandleBaseType_t)1; outerLoopHandle<=(xme_hal_table_rowHandleBaseType_t)(name).maxHandle; outerLoopHandle++) \
        { \
            if (NULL != (loopItem1 = (name).array[outerLoopHandle-1])) \
            { \
                for (innerLoopHandle=outerLoopHandle + 1; innerLoopHandle<=(xme_hal_table_rowHandleBaseType_t)(name).maxHandle; innerLoopHandle++) \
                { \
                    if (NULL != (loopItem2 = (name).array[innerLoopHandle-1])) \
                    { \
                        compareItem1 = (compareType*)(((uintptr_t)loopItem1) + (offset)); \
                        compareItem2 = (compareType*)(((uintptr_t)loopItem2) + (offset)); \
                        if(*compareItem1 > *compareItem2) \
                        { \
                            tempItem = *loopItem1; \
                            *loopItem1 = *loopItem2; \
                            *loopItem2 = tempItem; \
                        } \
                    } \
                } \
            } \
        } \
    } while (0)

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Finalizes the given table. 
 *
 * \param table the array struct pointing to the table. 
 */
void
xme_hal_table_fini
(
    xme_hal_table_arrayStruct_t* table
);

/**
 * \brief Allocates an space in the table for a new element. 
 *
 * \param table the array struct pointing to the table. 
 * \param itemSize the size of allocated element.
 *
 * \return the row handle where the new element can be stored. 
 */
xme_hal_table_rowHandle_t
xme_hal_table_addItem
(
    xme_hal_table_arrayStruct_t* table, 
    uint16_t itemSize
);

/**
 * \brief Removes an item from the table. 
 *
 * \param table the array struct pointing to the table. 
 * \param handle the row handle to remove. 
 * \param itemSize the size of the item to remove. 
 * 
 * \return the result status from the item removal. 
 */
xme_status_t
xme_hal_table_removeItem
(
    xme_hal_table_arrayStruct_t* table, 
    xme_hal_table_rowHandle_t handle, 
    uint16_t itemSize
);

XME_EXTERN_C_END

#endif // #ifndef XME_HAL_TABLE_ARCH_H
