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
 * $Id: table_arch.c 7829 2014-03-14 10:29:33Z ruiz $
 */

/**
 * \file
 *         Table abstraction (architecture specific part: generic embedded
 *         implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/table.h"

#include "xme/defines.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_hal_table_clear
(
    void* table
)
{
    xme_hal_table_arrayStruct_t* tableDesc = (xme_hal_table_arrayStruct_t*) table;

    XME_CHECK(NULL != table, XME_STATUS_INVALID_PARAMETER);

    XME_ASSERT(tableDesc->itemSize > 0U);
    XME_ASSERT(NULL != tableDesc->arrayPtr);
    XME_ASSERT(NULL != tableDesc->bitmap);

    // It is enough to zero out the first maxHandle items, because
    // all remaining items should be zeroed and available anyway
    if (tableDesc->maxHandle > 0U)
    {
        xme_fallback_memset(tableDesc->bitmap, XME_HAL_TABLE_ARCH_ROW_AVAILABLE, sizeof(tableDesc->bitmap[0]) * tableDesc->maxHandle);
        xme_fallback_memset(tableDesc->arrayPtr, 0, tableDesc->itemSize * tableDesc->maxHandle);
    }

    tableDesc->count = 0U;
    tableDesc->maxHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    return XME_STATUS_SUCCESS;
}

xme_hal_table_rowHandle_t
xme_hal_table_addItem(xme_hal_table_arrayStruct_t* table, size_t capacity)
{
	xme_hal_table_rowHandle_t handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

	XME_ASSERT_RVAL(NULL != table, XME_HAL_TABLE_INVALID_ROW_HANDLE);
	XME_ASSERT_RVAL(table->count <= table->maxHandle, XME_HAL_TABLE_INVALID_ROW_HANDLE);
	XME_ASSERT_RVAL(table->maxHandle <= capacity, XME_HAL_TABLE_INVALID_ROW_HANDLE);
	XME_ASSERT_RVAL(table->bitmap != NULL, XME_HAL_TABLE_INVALID_ROW_HANDLE);

	// This will halt the system on embedded targets
	XME_ASSERT_RVAL(table->count < capacity, XME_HAL_TABLE_INVALID_ROW_HANDLE);

	// Table is allocated statically, so return an error if it's full.
	XME_CHECK(table->count < capacity, XME_HAL_TABLE_INVALID_ROW_HANDLE);

	// If the memory is fragmented, we search for the first empty item
	// and return a handle to that item. The memory is fragmented when
	// the number of items in the table is less than the highest valid
	// handle (which is a one-based index into the array of table items).
	if (table->count != table->maxHandle)
	{
		// Search the fragmented array for the next free item.
		// Actually, the "-1" should be added in the bounds check of the
		// for loop, but it only complicates the calculation and doesn't
		// change behavior because the above assertion ensures that the
		// last table item is never unused when we reach this code, so we
		// can safely omit it.
		xme_hal_table_rowHandleBaseType_t i;
		for (i=0; i<table->maxHandle/*-1*/; i++)
		{
			// If the table item is unused...
			if (XME_HAL_TABLE_ARCH_ROW_AVAILABLE == table->bitmap[i])
			{
				// ...use its index as base for the handle and return
				table->bitmap[i] = XME_HAL_TABLE_ARCH_ROW_OCCUPIED;
				table->count++;
				return (xme_hal_table_rowHandle_t)(i+1);
			}
		}

		// We should never reach this location, because that would indicate
		// an inconsistent table state
		XME_ASSERT_RVAL(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_HAL_TABLE_INVALID_ROW_HANDLE);
	}
	else
	{
		// Table is contiguously filled: Use the next available item
		// at the back of the table.
		handle = (xme_hal_table_rowHandle_t)(++table->count);

		// Table is allocated statically, so respect it's capacity.
		if (table->maxHandle < capacity)
		{
			table->maxHandle = handle;
		}

		// Mark item as occupied
		table->bitmap[handle-1] = XME_HAL_TABLE_ARCH_ROW_OCCUPIED;
	}

	return handle;
}

xme_status_t
xme_hal_table_removeItem(xme_hal_table_arrayStruct_t* table, xme_hal_table_rowHandle_t handle, size_t capacity, size_t itemSize)
{
	XME_ASSERT(NULL != table);

	XME_CHECK(0 < table->count, XME_STATUS_INVALID_HANDLE);

	XME_ASSERT(0 < table->maxHandle);
	XME_ASSERT(table->count <= table->maxHandle);
	XME_ASSERT(table->maxHandle <= capacity);
	XME_ASSERT(table->bitmap != NULL);

	XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_INVALID_HANDLE);
	XME_CHECK(handle <= table->maxHandle, XME_STATUS_INVALID_HANDLE);

    // Free the table item and zero out the memory (the API guarantees initially zeroed memory)
    table->bitmap[handle-1] = XME_HAL_TABLE_ARCH_ROW_AVAILABLE;
    xme_fallback_memset((void*) (((uintptr_t) table->arrayPtr) + itemSize * (handle-1)), 0, itemSize);
    --table->count;

	// Check whether the highest handle value has been removed.
	// If this is the case, we can update maxHandle to reflect
	// the status of contiguously used space.
	//
	// "Holes" in the table will be filled by subsequent calls
	// of xme_hal_table_removeItem() in O(n).
	//
	// We do not try to detect the fact that the table has
	// become fully contiguous by multiple calls of xme_hal_table_removeItem()
	// in order to keep this function O(1).
	if (table->maxHandle == handle)
	{
		--table->maxHandle;
	}

	return XME_STATUS_SUCCESS;
}

bool
xme_hal_table_hasNext
(
    void* table,
    xme_hal_table_iterator_t iterator
)
{
    xme_hal_table_arrayStruct_t* tableDesc = (xme_hal_table_arrayStruct_t*) table;

    XME_CHECK(NULL != tableDesc, false);

    return (iterator < tableDesc->maxHandle);
}
