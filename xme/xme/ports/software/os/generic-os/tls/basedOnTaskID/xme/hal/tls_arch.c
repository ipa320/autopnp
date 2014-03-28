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
 * $Id: tls_arch.c 7269 2014-02-11 14:34:12Z geisinger $
 */

/**
 * \file
 *         Thread-local storage abstraction (architecture specific part:
 *         FreeRTOS implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/hal/include/tls.h"
#include "xme/hal/include/sharedPtr.h"
#include "xme/hal/include/table.h"
#include "xme/hal/include/task.h"


/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_hal_tls_allocationTable_t
 *
 * \brief Table for storing the sizes of the TLS's.
 */
typedef struct
{
	uint16_t tlsSize; ///< Size of tls
}
xme_hal_tls_allocationTableEntry_t;

/**
 * \typedef xme_hal_tls_allocationTable_t
 *
 * \brief Table for referencing the memory of the TLS. One entry corresponds to one combination of TLS and task.
 */
typedef struct
{
	xme_hal_tls_handle_t tlsHandle; ///< Handle of TLS
	xTaskHandle taskHandle; ///< Handle of task
	xme_hal_sharedPtr_t refToMemory; ///< The allocated memory for one particular task and TLS
}
xme_hal_tls_memoryRefTableEntry_t;

XME_HAL_TABLE(
		xme_hal_tls_allocationTableEntry_t,
		allocationTable,
		XME_HAL_TLS_MAX_ALLOCATABLE_TLS
);

XME_HAL_TABLE(
		xme_hal_tls_memoryRefTableEntry_t,
		memoryTable,
		XME_HAL_TLS_MAX_ALLOCATABLE_TLS * XME_HAL_TLS_MAX_TASKS_USING_TLS
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_tls_init()
{
    // TODO: Tolerate multiple initialization

	XME_HAL_TABLE_INIT( allocationTable );
	XME_HAL_TABLE_INIT( memoryTable );

	// Register the calling thread, which should be the "main thread",
	// if such concept exists on this platform
	{
		xme_status_t result = xme_hal_tls_registerThread();
		XME_ASSERT(XME_STATUS_SUCCESS == result);
	}

	return XME_STATUS_SUCCESS;
}

void
xme_hal_tls_fini()
{
    // TODO: Tolerate multiple finalization

	// Deregister the calling thread, which should be the "main thread",
	// if such concept exists on this platform
	xme_hal_tls_deregisterThread();

	XME_HAL_TABLE_FINI( allocationTable );
	XME_HAL_TABLE_FINI( memoryTable );
}

xme_hal_tls_handle_t
xme_hal_tls_alloc
(
	uint16_t size
)
{
	xme_hal_table_rowHandle_t handle;
	xme_hal_tls_allocationTableEntry_t* item;

	// When allocating a TLS, the size is stored
	// in the allocation table. Nothing more happens.

	// Later, when a task accesses the TLS,
	// the memory area is allocated.

	handle = XME_HAL_TABLE_ADD_ITEM ( allocationTable );
	if ( handle == XME_HAL_TABLE_INVALID_ROW_HANDLE )
	{
		// Out of memory
		return XME_HAL_TLS_INVALID_TLS_HANDLE;
	}
	item = XME_HAL_TABLE_ITEM_FROM_HANDLE( allocationTable, handle );
	item->tlsSize = size;

	// Return tls handle
	return (xme_hal_tls_handle_t)handle;
}

void*
xme_hal_tls_get
(
	xme_hal_tls_handle_t tlsHandle
)
{
	xme_hal_tls_allocationTableEntry_t* allocTableItem;
	xme_hal_tls_memoryRefTableEntry_t* memoryRefItem;
	xme_hal_table_rowHandle_t tableRowHandle;
	xme_hal_taskId_t currentTask = xme_hal_task_getCurrentTaskId();

	allocTableItem = XME_HAL_TABLE_ITEM_FROM_HANDLE( allocationTable, tlsHandle );

	// Check if tls is valid
	if ( allocTableItem == NULL )
	{
		return NULL;
	}

	// Search for entry in memoryTable
	XME_HAL_TABLE_ITERATE_BEGIN
	(
		memoryTable,
		xme_hal_table_rowHandle_t, tableHandle,
		xme_hal_tls_memoryRefTableEntry_t, item
	);
	{
		if ( item->taskHandle == currentTask &&
				item->tlsHandle == tlsHandle )
		{
			// Found memory reference for task
			// Return pointer to memory
			return xme_hal_sharedPtr_getPointer(item->refToMemory);
		}
	}
	XME_HAL_TABLE_ITERATE_END();

	// No entry found, so one is created.
	tableRowHandle = XME_HAL_TABLE_ADD_ITEM ( memoryTable );
	if ( tableRowHandle == XME_HAL_TABLE_INVALID_ROW_HANDLE )
	{
		// Out of memory
		return NULL;
	}

	memoryRefItem = XME_HAL_TABLE_ITEM_FROM_HANDLE( memoryTable, tableRowHandle );
	memoryRefItem->tlsHandle = tlsHandle;
	memoryRefItem->taskHandle = currentTask;
	memoryRefItem->refToMemory = xme_hal_sharedPtr_create(allocTableItem->tlsSize);

	if ( memoryRefItem->refToMemory == XME_HAL_SHAREDPTR_INVALID_POINTER )
	{
		// Out of memory
		XME_HAL_TABLE_REMOVE_ITEM( memoryTable, tableRowHandle );
		return NULL;
	}

	memset(
			xme_hal_sharedPtr_getPointer(memoryRefItem->refToMemory),
			0,
			allocTableItem->tlsSize
	);

	return xme_hal_sharedPtr_getPointer(memoryRefItem->refToMemory);
}

xme_status_t
xme_hal_tls_free
(
	xme_hal_tls_handle_t tlsHandle
)
{
	xme_hal_tls_allocationTableEntry_t* allocTableItem;

	// This function destroys all references to a specific TLS handle.
	// That means that the size is removed from the TLS table and all
	// memory references are destroyed and removed from the memory table.

	allocTableItem = XME_HAL_TABLE_ITEM_FROM_HANDLE( allocationTable, tlsHandle );

	// Check if tls is valid
	if ( allocTableItem == NULL )
	{
		return XME_STATUS_INVALID_HANDLE;
	}

	// Delete all entries in memory reference table that
	// are related to the given TLS handle.
	XME_HAL_TABLE_ITERATE_BEGIN
	(
		memoryTable,
		xme_hal_table_rowHandle_t, tableHandle,
		xme_hal_tls_memoryRefTableEntry_t, item
	);
	{
		if ( item->tlsHandle == tlsHandle )
		{
			xme_hal_sharedPtr_destroy( item->refToMemory );
			XME_HAL_TABLE_REMOVE_ITEM( memoryTable, tableHandle );
		}
	}
	XME_HAL_TABLE_ITERATE_END();

	// Delete entry in tls table
	XME_HAL_TABLE_REMOVE_ITEM( allocationTable, tlsHandle );

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_tls_registerThread()
{
	// Nothing to do.
	// The memory for a thread will be created on-the-fly.
	// This also means that a thread might access a TLS without
	// registering itself before.

	return XME_STATUS_SUCCESS;
}

void
xme_hal_tls_deregisterThread()
{
	xme_hal_taskId_t currentTask = xme_hal_task_getCurrentTaskId();

	// When a task deregisters itself, all memory references
	// hold by that task are destroyed.

	XME_HAL_TABLE_ITERATE_BEGIN
	(
		memoryTable,
		xme_hal_table_rowHandle_t, tableHandle,
		xme_hal_tls_memoryRefTableEntry_t, item
	);
	{
		if ( item->taskHandle == currentTask )
		{
			xme_hal_sharedPtr_destroy( item->refToMemory );
			XME_HAL_TABLE_REMOVE_ITEM( memoryTable, tableHandle );
		}
	}
	XME_HAL_TABLE_ITERATE_END();
}

