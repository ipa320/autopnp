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
 * $Id: tls_arch.c 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 *         Thread-local storage abstraction (platform specific part: Windows).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/tls.h"
#include "xme/hal/tls_util.h"

#include <Windows.h>

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
        TLS_OUT_OF_INDEXES != (item->index = TlsAlloc()),
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
    LPVOID data;

    item = _xme_hal_tls_getItem(tlsHandle);
    XME_CHECK(NULL != item, NULL); // XME_STATUS_INVALID_PARAMETER

    data = TlsGetValue(item->index);
    if (NULL == data)
    {
        XME_CHECK
        (
            ERROR_SUCCESS == GetLastError(),
            NULL // XME_STATUS_INVALID_PARAMETER
        );

        // Memory has not yet been allocated
        XME_CHECK
        (
             NULL != (data = (LPVOID)LocalAlloc(LPTR, item->size)),
            NULL // XME_STATUS_OUT_OF_RESOURCES
        );

        XME_CHECK_REC
        (
            (0 != TlsSetValue(item->index, data)),
            NULL, // XME_STATUS_OUT_OF_RESOURCES
            {
                LocalFree((HLOCAL)data);
            }
        );
    }

    XME_ASSERT_RVAL(NULL != data, NULL);

    return (void*)data;
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
    // TODO (See ticket #807): TlsFree() does not take care of freeing all the allocated memory occupied
    //                         by other threads! Assert here that the memory has been freed before!

    xme_hal_tls_item_t* item;
    xme_status_t rval;

    item = _xme_hal_tls_getItem(tlsHandle);
    XME_CHECK(NULL != item, XME_STATUS_INVALID_HANDLE);

    XME_CHECK
    (
        0 != TlsFree(item->index),
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
    // Free the memory occupied by the local storage of this thread
    XME_HAL_TABLE_ITERATE_BEGIN
    (
        xme_hal_tls_config.items,
        xme_hal_tls_handle_t, handle,
        xme_hal_tls_item_t, item
    );
    {
        LPVOID data = TlsGetValue(item->index);
        if (NULL != data)
        {
            LocalFree((HLOCAL)data);
        }
    }
    XME_HAL_TABLE_ITERATE_END();
}
