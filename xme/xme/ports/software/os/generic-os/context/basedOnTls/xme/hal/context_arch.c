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
 * $Id: context_arch.c 4872 2013-08-30 12:06:54Z camek $
 */

/**
 * \file
 *         Context abstraction (architecture specific part:
 *         generic OS-based implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/context.h"

#include "xme/hal/include/tls.h"

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
/**
 * \var xme_hal_context_tlsHandle
 *
 * \brief  Thread-local storage handle used to implement context abstraction.
 */
static xme_hal_tls_handle_t xme_hal_context_tlsHandle = XME_HAL_TLS_INVALID_TLS_HANDLE;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_hal_context_init(void)
{
    XME_ASSERT(XME_HAL_TLS_INVALID_TLS_HANDLE == xme_hal_context_tlsHandle);

    XME_CHECK
    (
        XME_HAL_TLS_INVALID_TLS_HANDLE !=
            (xme_hal_context_tlsHandle = xme_hal_tls_alloc(sizeof(xme_hal_context_contextHandle_t))),
        XME_STATUS_OUT_OF_RESOURCES
    );

    return XME_STATUS_SUCCESS;
}

void
xme_hal_context_fini(void)
{
    if (xme_hal_context_tlsHandle != XME_HAL_TLS_INVALID_TLS_HANDLE)
    {
        (void) xme_hal_tls_free(xme_hal_context_tlsHandle);
        xme_hal_context_tlsHandle = XME_HAL_TLS_INVALID_TLS_HANDLE;
    }
}

xme_status_t
xme_hal_context_setContext
(
    xme_hal_context_contextHandle_t context
)
{
    void* data;

    XME_CHECK(XME_HAL_TLS_INVALID_TLS_HANDLE != xme_hal_context_tlsHandle, XME_STATUS_INVALID_CONFIGURATION);

    data = xme_hal_tls_get(xme_hal_context_tlsHandle);
    
    if (NULL != data || XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE != context)
    {
        XME_CHECK(NULL != data, XME_STATUS_OUT_OF_RESOURCES);
        *((xme_hal_context_contextHandle_t*)data) = context;
    }

    return XME_STATUS_SUCCESS;
}

xme_hal_context_contextHandle_t
xme_hal_context_getContext(void)
{
    void* data;

    XME_CHECK(XME_HAL_TLS_INVALID_TLS_HANDLE != xme_hal_context_tlsHandle, XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE);
    
    data = xme_hal_tls_get(xme_hal_context_tlsHandle);
    
    return (NULL != data) ? *((xme_hal_context_contextHandle_t*)data) : XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE;
}
