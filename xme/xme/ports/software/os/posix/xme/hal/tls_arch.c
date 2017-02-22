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
 * $Id: tls_arch.c 6684 2014-02-07 14:34:26Z geisinger $
 */

/**
 * \file
 *         Thread-local storage abstraction (platform specific part: Posix).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/tls.h"
#include "xme/hal/tls_util.h"

#include "xme/hal/include/mem.h"

#include <pthread.h>
#include <stdlib.h>

/******************************************************************************/
/***   Helper functions                                                     ***/
/******************************************************************************/
static
void
_xme_hal_tls_keyDestructor
(
	void* value
)
{
	xme_hal_mem_free(value);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_tls_init(void)
{
	return _xme_hal_tls_init();
}

void
xme_hal_tls_fini(void)
{
	_xme_hal_tls_fini();
}

xme_hal_tls_handle_t
xme_hal_tls_alloc
(
	uint16_t size
)
{
	xme_hal_tls_handle_t tlsHandle;
	xme_hal_tls_item_t* item;

	XME_CHECK
	(
		0 != size,
		XME_HAL_TLS_INVALID_TLS_HANDLE
	);

	item = _xme_hal_tls_addItem(&tlsHandle);
	XME_CHECK(NULL != item, XME_HAL_TLS_INVALID_TLS_HANDLE);

	XME_CHECK_REC
	(
		0 == pthread_key_create(&item->index, &_xme_hal_tls_keyDestructor),
		XME_HAL_TLS_INVALID_TLS_HANDLE,
		{
			xme_status_t rval = _xme_hal_tls_removeItem(tlsHandle);
			XME_ASSERT_RVAL(XME_STATUS_SUCCESS == rval, XME_HAL_TLS_INVALID_TLS_HANDLE);
		}
	);

	item->size = size;

	return tlsHandle;
}

void*
xme_hal_tls_get
(
	xme_hal_tls_handle_t tlsHandle
)
{
	xme_hal_tls_item_t* item;
	void* data;

	item = _xme_hal_tls_getItem(tlsHandle);
	XME_CHECK(NULL != item, NULL); // XME_STATUS_INVALID_PARAMETER

	data = pthread_getspecific(item->index);
	if (NULL == data)
	{
		// Memory has not yet been allocated
		XME_CHECK
		(
			NULL != (data = xme_hal_mem_alloc(item->size)),
			NULL // XME_STATUS_OUT_OF_RESOURCES
		);

		XME_CHECK_REC
		(
			(0 == pthread_setspecific(item->index, data)),
			NULL, // ENOMEM: XME_STATUS_OUT_OF_RESOURCES, EINVAL: XME_STATUS_INVALID_PARAMETER
			{
				xme_hal_mem_free(data);
			}
		);
	}

	XME_ASSERT_RVAL(NULL != data, NULL);

	return data;
}

uint16_t
xme_hal_tls_getSize
(
	xme_hal_tls_handle_t tlsHandle
)
{
	return _xme_hal_tls_getSize(tlsHandle);
}

xme_status_t
xme_hal_tls_free
(
	xme_hal_tls_handle_t tlsHandle
)
{
	// TODO (See ticket #807): pthread_key_delete() does not take care of freeing all the allocated memory occupied
	//                         by any thread! Assert here that the memory has been freed before!

	xme_hal_tls_item_t* item;
	xme_status_t rval;

	item = _xme_hal_tls_getItem(tlsHandle);
	XME_CHECK(NULL != item, XME_STATUS_INVALID_HANDLE);

	XME_CHECK
	(
		0 == pthread_key_delete(item->index),
		XME_STATUS_INVALID_PARAMETER
	);

	rval = _xme_hal_tls_removeItem(tlsHandle);
	XME_ASSERT(XME_STATUS_SUCCESS == rval);

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_tls_registerThread(void)
{
	// Nothing to do, memory will be allocated on demand as soon as data need to be stored
	return XME_STATUS_SUCCESS;
}

void
xme_hal_tls_deregisterThread(void)
{
	// Thread resources should have been destroyed using _xme_hal_tls_keyDestructor().
	// Hence, nothing to do here.

	// Free the memory occupied by the local storage of this thread
	/*XME_HAL_TABLE_ITERATE_BEGIN
	(
		xme_hal_tls_config.items,
		xme_hal_tls_handle_t, handle,
		xme_hal_tls_item_t, item
	);
	{
		void* data = pthread_getspecific(item->index);
		if (NULL != data)
		{
			free(data);
		}
	}
	XME_HAL_TABLE_ITERATE_END();
	*/
}
