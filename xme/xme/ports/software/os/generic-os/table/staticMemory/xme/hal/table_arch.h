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
 *         Table abstraction (architecture specific part: generic embedded
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
#include <stdlib.h>
#include <string.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
typedef uint8_t xme_hal_table_bitmap_t;

#define XME_HAL_TABLE_ARCH_ROW_OCCUPIED (1)
#define XME_HAL_TABLE_ARCH_ROW_AVAILABLE (0)

#define XME_HAL_TABLE_STATIC_ALLOC

#ifdef _MSC_VER
    #pragma warning( push )
    // Silence warning about array length of member bitmap of struct xme_hal_table_arrayStruct_t
    #pragma warning( disable : 4200 )
#endif // _MSC_VER

/*
 * \typedef xme_hal_table_arrayStruct_t
 *
 * \brief  Table structure.
 *
 *         The table structure is the internal representation of a table.
 *
 * \note   Note that the definition of xme_hal_table_arrayStruct_t must match
 *         the definition of the XME_HAL_TABLE() macro (except for the array
 *         element).
 */
typedef struct
{
    uint16_t count;                      ///< Number of items in the table.
    xme_hal_table_rowHandle_t maxHandle; ///< Highest valid handle value of an item in the table.
    size_t itemSize;                     ///< Size of one item in the table.
    void* arrayPtr;                      ///< Pointer to the array member of the structure (located after the unknown-size bitmap member).
    xme_hal_table_bitmap_t bitmap[];     ///< Bitmap indicating which fields in the table are currently
                                         ///< occupied or available.
}
xme_hal_table_arrayStruct_t;

#ifdef _MSC_VER
    #pragma warning( pop )
#endif // _MSC_VER

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
// Documented in table.h
#define XME_HAL_TABLE(itemType, name, maxElements) \
    struct \
    { \
        uint16_t count; \
        xme_hal_table_rowHandle_t maxHandle; \
        size_t itemSize; \
        void* arrayPtr; \
        xme_hal_table_bitmap_t bitmap[maxElements]; \
        itemType array[maxElements]; \
    } \
    name

// Documented in table.h
#define XME_HAL_TABLE_INIT(name) \
    do \
    { \
        (name).count = 0U; \
        (name).maxHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE; \
        (name).itemSize = sizeof((name).array[0]); \
        (name).arrayPtr = &(name).array[0]; \
        xme_fallback_memset((name).bitmap, XME_HAL_TABLE_ARCH_ROW_AVAILABLE, sizeof((name).bitmap)); \
        xme_fallback_memset((name).array, 0, sizeof((name).array)); \
    } while (0)

// Documented in table.h
#define XME_HAL_TABLE_FINI(name) \
    do \
    { \
        (name).count = 0U; \
        (name).maxHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE; \
        (name).itemSize = 0U; \
        (name).arrayPtr = NULL; \
    } \
    while (0)

// Documented in table.h
#define XME_HAL_TABLE_CLEAR(name) \
    xme_hal_table_clear((xme_hal_table_arrayStruct_t*)&(name))

// Documented in table.h
#define XME_HAL_TABLE_ADD_ITEM(name) \
    xme_hal_table_addItem((xme_hal_table_arrayStruct_t*)&(name), sizeof((name).bitmap)/sizeof(xme_hal_table_bitmap_t))

// Documented in table.h
#define XME_HAL_TABLE_REMOVE_ITEM(name, handle) \
    xme_hal_table_removeItem((xme_hal_table_arrayStruct_t*)&(name), handle, sizeof((name).bitmap)/sizeof((name).bitmap[0]), sizeof((name).array[0]))

// Documented in table.h
#define XME_HAL_TABLE_ITEM_COUNT(name) \
    ((name).count)

// Documented in table.h
#define XME_HAL_TABLE_ITEM_FROM_HANDLE(name, handle) \
    ((handle > 0 && handle <= (name).maxHandle && (name).bitmap[((xme_hal_table_rowHandleBaseType_t)handle)-1] == XME_HAL_TABLE_ARCH_ROW_OCCUPIED) ? &(name).array[((xme_hal_table_rowHandleBaseType_t)handle)-1] : NULL)

// Documented in table.h
#define XME_HAL_TABLE_ITERATE_BEGIN(name, loopHandleType, loopHandle, loopItemType, loopItem) \
    do \
    { \
        loopHandleType loopHandle; \
        for (loopHandle=(loopHandleType)1; loopHandle<=(loopHandleType)(name).maxHandle; loopHandle = (loopHandleType)(((xme_hal_table_rowHandleBaseType_t)loopHandle)+1)) \
        { \
            if (XME_HAL_TABLE_ARCH_ROW_OCCUPIED == (name).bitmap[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1]) \
            { \
                loopItemType* loopItem = &(name).array[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1]; \
                { \

// Documented in table.h
#define XME_HAL_TABLE_ITERATE_END() \
                } \
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
            if (XME_HAL_TABLE_ARCH_ROW_OCCUPIED == (name).bitmap[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1]) \
            { \
                loopItemType* loopItem = &(name).array[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1]; \
                { \

// Documented in table.h
#define XME_HAL_TABLE_ITERATE_REVERSE_END() \
                } \
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
            if (loopHandle <= (loopHandleType)(name).maxHandle && XME_HAL_TABLE_ARCH_ROW_OCCUPIED == (name).bitmap[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1]) \
            { \
                loopItem = &(name).array[((xme_hal_table_rowHandleBaseType_t)loopHandle)-1]; \
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
        XME_UNUSED_PARAMETER(name); \
        XME_UNUSED_PARAMETER(compareFunction); \
    } \
    while (0)

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Allocates an space in the table for a new element.
 *
 * \param[in] table Array structure representing the table.
 * \param[in] capacity Capacity of the statically allocated table.
 *
 * \return Row handle where the new element can be stored.
 */
xme_hal_table_rowHandle_t
xme_hal_table_addItem
(
    xme_hal_table_arrayStruct_t* table,
    size_t capacity
);

/**
 * \brief Removes an item from the table.
 *
 * \param[in] table Array structure representing the table.
 * \param[in] handle Row handle of the item to remove.
 * \param[in] capacity Capacity of the statically allocated table.
 * \param[in] itemSize Size of a row in the table.
 * 
 * \retval XME_CORE_STATUS_SUCCESS if the item has been successfully removed.
 * \retval XME_CORE_STATUS_INVALID_HANDLE if the given handle was invalid.
 */
xme_status_t
xme_hal_table_removeItem
(
    xme_hal_table_arrayStruct_t* table,
    xme_hal_table_rowHandle_t handle,
    size_t capacity,
    size_t itemSize
);

XME_EXTERN_C_END

#endif // #ifndef XME_HAL_TABLE_ARCH_H
