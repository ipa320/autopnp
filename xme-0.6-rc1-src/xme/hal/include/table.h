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
 * $Id: table.h 4908 2013-09-02 13:54:17Z ruiz $
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
