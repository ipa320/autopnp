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
 * $Id: table.h 6511 2014-01-27 16:55:21Z geisinger $
 */

/**
 * \file
 * \brief Table abstraction.
 */

#ifndef XME_HAL_TABLE_H
#define XME_HAL_TABLE_H

/**
 * \defgroup hal_table Table abstraction
 * @{
 *
 * \brief  Table data structure abstraction.
 *
 * \details A table is a data structure that allows to store collections of
 *          items with equal structure. An item is uniquely identified by its
 *          index (i.e., row) in the table. The structure of a row (i.e., the
 *          columns of the table) are defined by a C struct.
 *          Functionality is provided for adding new items, retrieving the
 *          address of the struct for a given index, removing items as well
 *          as iterating over the items.
 *
 * \note   Tables are implementated differently depending on the architecture.
 *         This is why tables are covered by the hardware abstraction library.
 *         On some target platforms, table items are dynamically allocated and
 *         the number of items is only limited by the available system memory.
 *         On resource constrained platforms without efficient memory
 *         management, a static maximum size (in terms of number of items) is
 *         given for each table.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def XME_HAL_TABLE
 *
 * \brief Defines an table with a given maximum size.
 *
 * \param[in] itemType Type of the items in the table.
 * \param[in] name Name of the table.
 * \param[in] maxElements Maximum number of elements in the table.
 *            On platforms without dynamic memory allocation, this parameter
 *            defines the maximum number of items in the table. The memory
 *            required to hold these items will then be statically allocated.
 */

/**
 * \def XME_HAL_TABLE_INIT
 *
 * \brief Initializes the given table.
 *        This macro must be called for a table before it can be used.
 *
 * \param[in] name Name of the table to initialize.
 */

/**
 * \def XME_HAL_TABLE_FINI
 *
 * \brief  Destroys the given table and frees the resources occupied by it.
 *
 * \note Since the table manages the memory occupied by the table items,
 *       this macro will free all associated memory. Any existing pointer
 *       to a table item may be invalid after a call to this macro.
 *
 * \param[in] name Name of the table to finalize.
 */

/**
 * \def XME_HAL_TABLE_CLEAR
 *
 * \brief Frees the elements of the given table.
 *
 * \deprecated This macro is deprecated, use ::xme_hal_table_clear() instead.
 *
 * \param[in] name Name of the table to clear.
 */

/**
 * \def XME_HAL_TABLE_ADD_ITEM
 *
 * \brief Adds a new item to the given table, returning the handle to that
 *        item.
 *
 * \details On platforms with dynamic memory allocation, this function will
 *          reserve the memory required to hold the new item and resize the
 *          table to fit the new item. On systems with static memory
 *          allocation, it will use the next free slot and return a handle
 *          to that slot. In any case, the memory used for the new item will
 *          be initialized with all zeros.
 *
 * \param[in] name Name of the table to add an item to.
 *
 * \return Returns the table row handle for the newly created item or
 *         XME_HAL_TABLE_INVALID_ROW_HANDLE if memory allocation failed (on
 *         platforms with dynamic memory management) or the table has
 *         reached its maximum capacity (on platforms without dynamic memory
 *         management).
 */

/**
 * \def XME_HAL_TABLE_REMOVE_ITEM
 *
 * \brief Removes the item corresponding to the given handle from the table.
 *
 * \param[in] name Name of the table.
 * \param[in] handle Table row handle of the item to remove.
 *
 * \retval XME_STATUS_SUCCESS if the item has been successfully removed.
 * \retval XME_STATUS_INVALID_HANDLE if the given handle was invalid.
 */

/**
 * \def XME_HAL_TABLE_ITEM_COUNT
 *
 * \brief Returns the number of items in the given table.
 *
 * \param[in] name Name of the table.
 *
 * \return Number of items in the given table.
 */

/**
 * \def XME_HAL_TABLE_ITEM_FROM_HANDLE
 *
 * \brief Returns a pointer to the table item for the given table row handle
 *        or NULL if no such item exists.
 *
 * \param[in] name Name of the table.
 * \param[in] handle Table row handle of the item to retrieve.
 *
 * \return Returns a pointer to the item corresponding to the given table row
 *         handle or NULL if no such item exists.
 */

/**
 * \def XME_HAL_TABLE_ITERATE_BEGIN
 *
 * \brief Begins a block for iterating over all items of the given table.
 *
 * \note A block started with XME_HAL_TABLE_ITERATE_BEGIN() must be ended
 *       with XME_HAL_TABLE_ITERATE_END() at the same scope.
 *
 * \warning Addition and removal of table items is possible from within the
 *          iterator body. However, it is undefined whether or not newly added
 *          items will appear in subsequent iterations.
 *
 * \param[in] name Name of the table to iterate over.
 * \param[in] loopHandleType Type of the loop handle variable, usually
 *            xme_hal_table_rowHandleBaseType_t or a type defined to it.
 * \param[in] loopHandle Name of the loop handle variable.
 *            This variable can be used from within the iterator body.
 * \param[in] loopItemType Base type of the items that are iterated over.
 *            This parameter should be the type of the table items, not a
 *            pointer to them.
 * \param[in] loopItem Name of the loop item variable.
 *            This variable can be used from within the iterator body.
 */

/**
 * \def XME_HAL_TABLE_ITERATE_END
 *
 * \brief Ends a block for iterating over all items of the given table.
 *
 * \note A block ended with XME_HAL_TABLE_ITERATE_END() must be started
 *       with XME_HAL_TABLE_ITERATE_BEGIN() at the same scope.
 */

/**
 * \def XME_HAL_TABLE_ITERATE_REVERSE_BEGIN
 *
 * \brief Begins a block for iterating over all items of the given table in
 *        reverse order.
 *
 * \note A block started with XME_HAL_TABLE_ITERATE_REVERSE_BEGIN() must be
 *       ended with XME_HAL_TABLE_ITERATE_REVERSE_END() at the same scope.
 *
 * \warning Addition and removal of table items is possible from within the
 *          iterator body. However, it is undefined whether or not newly added
 *          items will appear in subsequent iterations.
 *
 * \param[in] name Name of the table to iterate over.
 * \param[in] loopHandleType Type of the loop handle variable, usually
 *            xme_hal_table_rowHandleBaseType_t or a type defined to it.
 * \param[in] loopHandle Name of the loop handle variable.
 *            This variable can be used from within the iterator body.
 * \param[in] loopItemType Base type of the items that are iterated over.
 *            This parameter should be the type of the table items, not a
 *            pointer to them.
 * \param[in] loopItem Name of the loop item variable.
 *            This variable can be used from within the iterator body.
 */

/**
 * \def XME_HAL_TABLE_ITERATE_REVERSE_END
 *
 * \brief Ends a block for iterating over all items of the given table in
 *        reverse order.
 *
 * \note A block ended with XME_HAL_TABLE_ITERATE_REVERSE_END() must be
 *       started with XME_HAL_TABLE_ITERATE_REVERSE_BEGIN() at the same
 *       scope.
 */

/**
 * \def XME_HAL_TABLE_GET_NEXT
 *
 * \brief Search for the next element of the given table matching the given
 *        condition, starting at the element specified by loopHandle.
 *
 * \param[in] name Name of the table to iterate over.
 * \param[in] loopHandleType Type of the loop handle variable, usually
 *            xme_hal_table_rowHandleBaseType_t or a type defined to it.
 * \param[in,out] loopHandle Name of the loop handle variable. This variable has
 *                to be defined and initialized by the user and can be used from
 *                within the condition. The value of this variable determines at
 *                which element the scan over the table starts. To start at the
 *                beginning of the table, set the variable to
 *                XME_HAL_TABLE_INVALID_ROW_HANDLE.
 * \param[in] loopItemType Base type of the items that are iterated over.
 *            This parameter should be the type of the table items, not a
 *            pointer to them.
 * \param[in,out] loopItem Name of the loop item variable.
 *                This variable can be used from within the condition.
 * \param[in] condition Condition that the element shall satisfy. Must return
 *            a Boolean value. The variables specified in the loopHandle and
 *            loopItem parameters are available from within the condition.
 */

/**
 * \def XME_HAL_TABLE_BUBBLESORT
 *
 * \brief Sorts all items of the given table.
 *
 * \warning Not implemented for static tables!
 * \warning Usage destroys property of stable table handles!
 *
 * \deprecated This macro is deprecated. There is no replacement for it.
 *
 * \param[in] name Name of the table to sort.
 * \param[in] loopItemType Base type of the items that are sorted.
 *            This parameter should be the type of the table items, not a
 *            pointer to them.
 * \param[in] compareFunction Function comparing two struct/array-member and
 *            returning their relative order. The function receives two
 *            elements in each call and should return a value larger than zero
 *            if element1 is "larger" than element2, a value of zero if they
 *            are to be equally sorted and a value less than zero if element1
 *            is "smaller" than element2.
 */

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum xme_hal_table_rowHandle_t
 *
 * \brief  Table row handle.
 *
 * \details A table row handle is the index of a row in a table.
 *          Valid handles are non-zero and smaller or equal to
 *          XME_HAL_TABLE_MAX_ROW_HANDLE.
 */
typedef enum
{
    XME_HAL_TABLE_INVALID_ROW_HANDLE = 0, ///< Invalid table row handle.
    XME_HAL_TABLE_MAX_ROW_HANDLE = XME_MAX_SYSTEM_VALUE ///< Largest possible table row handle.
}
xme_hal_table_rowHandle_t;

/**
 * \typedef xme_hal_table_rowHandleBaseType_t
 *
 * \brief  Numeric base type used for table handle arithmetic.
 *
 *         It must be asserted that this type is able to express at least the
 *         values XME_HAL_TABLE_INVALID_ROW_HANDLE and XME_HAL_TABLE_MAX_ROW_HANDLE.
 */
// TODO: This type will vanish after we replace the above enum by #define statements (compare Issue #3455)
typedef uint16_t xme_hal_table_rowHandleBaseType_t;

/**
 * \typedef xme_hal_table_iterator_t
 *
 * \brief  Table iterator.
 *
 * \details Table iterators store the current progress of iterating over a
 *          table.
 *
 * \warning This NEW table iteration API is not yet fully implemented.
 *
 * \see xme_hal_table_initIterator
 * \see xme_hal_table_hasNext
 * \see xme_hal_table_next
 * \see xme_hal_table_finiIterator
 */
// TODO: This NEW table iteration API is not yet fully implemented.
// TODO: Support for reverse iteration!
// TODO: Clear documentation of iteration order and limitations!
typedef xme_hal_table_rowHandle_t xme_hal_table_iterator_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Clears the given table, that is removes all elements.
 *         After calling this function, the table will be empty.
 *
 * \note   Note that this function will free the memory occupied by the
 *         table items. This means that any remaining pointers to the
 *         respective items will become invalid.
 *
 * \param[in,out] table Table to clear.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_PARAMETER if table was NULL.
 */
xme_status_t
xme_hal_table_clear
(
    void* table
);

/**
 * \brief  Determines if there are more table items to iterate over.
 *
 * \details This function can be used as a loop invariant. Use
 *          xme_hal_table_next() to retrieve the actual item.
 *
 * \note If the iterator has not been initialized before, the return value is
 *       undefined.
 *
 * \param[in] table Table to iterate.
 * \param[in] iterator Table iterator.
 *
 * \retval true if there are more table items to iterate over.
 * \retval false if there are no more table items to iterate over or table was
 *         NULL.
 */
bool
xme_hal_table_hasNext
(
    void* table,
    xme_hal_table_iterator_t iterator
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/table_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_TABLE_H
