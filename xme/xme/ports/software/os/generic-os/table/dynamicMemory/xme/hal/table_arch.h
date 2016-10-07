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
 * $Id: table_arch.h 7829 2014-03-14 10:29:33Z ruiz $
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

#include <stdlib.h>
#include <string.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
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
// Documented in table.h
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

// Documented in table.h
#define XME_HAL_TABLE_INIT(name) \
    do \
    { \
        (name).count[0] = 0; \
        (name).maxHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE; \
        (name).power = 0; \
        (name).array = NULL; \
    } while (0)

// Documented in table.h
#define XME_HAL_TABLE_FINI(name) \
    do \
    { \
        /* Testing for existence of count[0] is a heuristic type check of the input parameter */ \
        (void)(name).count[0]; \
        xme_hal_table_fini((xme_hal_table_arrayStruct_t*)&(name)); \
    } \
    while (0)

// Documented in table.h
#define XME_HAL_TABLE_CLEAR(name) \
    xme_hal_table_clear((xme_hal_table_arrayStruct_t*)&(name))

// Documented in table.h
#define XME_HAL_TABLE_ADD_ITEM(name) \
    xme_hal_table_addItem((xme_hal_table_arrayStruct_t*)&(name), sizeof(**(name).array))

// Documented in table.h
#define XME_HAL_TABLE_REMOVE_ITEM(name, handle) \
    xme_hal_table_removeItem((xme_hal_table_arrayStruct_t*)&(name), handle, sizeof(**(name).array))

// Documented in table.h
#define XME_HAL_TABLE_ITEM_COUNT(name) \
    ((name).count[0])

// Documented in table.h
#define XME_HAL_TABLE_ITEM_FROM_HANDLE(name, handle) \
    (((xme_hal_table_rowHandle_t)(handle) > XME_HAL_TABLE_INVALID_ROW_HANDLE && (xme_hal_table_rowHandle_t)(handle) <= (name).maxHandle) ? (name).array[((xme_hal_table_rowHandleBaseType_t)handle)-1] : NULL)

// Documented in table.h
#define XME_HAL_TABLE_ITERATE_BEGIN(name, loopHandleType, loopHandle, loopItemType, loopItem) \
    do \
    { \
        loopHandleType loopHandle; \
        for (loopHandle=(loopHandleType)1; loopHandle<=(loopHandleType)(name).maxHandle; loopHandle = (loopHandleType)(((xme_hal_table_rowHandleBaseType_t)loopHandle)+1)) \
        { \
            loopItemType* loopItem; \
            if (NULL != (loopItem = (name).array[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1])) \
            {

// Documented in table.h
#define XME_HAL_TABLE_ITERATE_END() \
            } \
        } \
    } while (0)

// Documented in table.h
#define XME_HAL_TABLE_ITERATE_REVERSE_BEGIN(name, loopHandleType, loopHandle, loopItemType, loopItem) \
    do \
    { \
        loopHandleType loopHandle; \
        for (loopHandle=(loopHandleType)(name).maxHandle; loopHandle>=(loopHandleType)1; loopHandle = (loopHandleType)(((xme_hal_table_rowHandleBaseType_t)loopHandle)-1)) \
        { \
            loopItemType* loopItem; \
            if (NULL != (loopItem = (name).array[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1])) \
            { \

// Documented in table.h
#define XME_HAL_TABLE_ITERATE_REVERSE_END() \
            } \
        } \
    } while (0)

// Documented in table.h
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

// Documented in table.h
#define XME_HAL_TABLE_BUBBLESORT(name, loopItemType, compareFunction) \
    do \
    { \
        xme_hal_table_rowHandleBaseType_t outerLoopHandle, innerLoopHandle; \
        loopItemType* loopItem1; \
        loopItemType* loopItem2; \
        loopItemType tempItem; \
        for (outerLoopHandle=(xme_hal_table_rowHandleBaseType_t)1; outerLoopHandle<=(xme_hal_table_rowHandleBaseType_t)(name).maxHandle; outerLoopHandle++) \
        { \
            if (NULL != (loopItem1 = (name).array[outerLoopHandle-1])) \
            { \
                for (innerLoopHandle=outerLoopHandle + 1; innerLoopHandle<=(xme_hal_table_rowHandleBaseType_t)(name).maxHandle; innerLoopHandle++) \
                { \
                    if (NULL != (loopItem2 = (name).array[innerLoopHandle-1])) \
                    { \
                        if(0 < compareFunction(loopItem1, loopItem2)) \
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
 * \param[in] table Array structure representing the table.
 */
void
xme_hal_table_fini
(
    xme_hal_table_arrayStruct_t* table
);

/**
 * \brief Allocates an space in the table for a new element.
 *
 * \param[in] table Array structure representing the table.
 * \param[in] itemSize Size of a row in the table.
 *
 * \return Row handle where the new element can be stored.
 */
xme_hal_table_rowHandle_t
xme_hal_table_addItem
(
    xme_hal_table_arrayStruct_t* table, 
    size_t itemSize
);

/**
 * \brief Removes an item from the table.
 *
 * \param[in] table Array structure representing the table.
 * \param[in] handle Row handle of the item to remove.
 * \param[in] itemSize Size of a row in the table.
 * 
 * \retval XME_STATUS_SUCCESS if the item has been successfully removed.
 * \retval XME_STATUS_INVALID_HANDLE if the given handle was invalid.
 */
xme_status_t
xme_hal_table_removeItem
(
    xme_hal_table_arrayStruct_t* table, 
    xme_hal_table_rowHandle_t handle, 
    size_t itemSize
);

XME_EXTERN_C_END

#endif // #ifndef XME_HAL_TABLE_ARCH_H
