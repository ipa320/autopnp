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
 * $Id: contextTlsMock.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         A mock of xme_hal_tls, to be used with the xme_hal_context tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/tls.h"

#include <stdint.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Used instead of the original xme_hal_tls function.
 *
 * \note   This mock only supports one thread-local storage handle.
 *
 * \param  size Size to allocate thread-local storage with.
 *
 * \retval 1 for the first allocated handle.
 * \retval XME_HAL_TLS_INVALID_HANDLE otherwise.
 */
xme_hal_tls_handle_t
xme_hal_tls_alloc
(
    uint16_t size
);

/**
 * \brief  Used instead of the original xme_hal_tls function.
 *
 * \param  tlsHandle Thread-local storage handle.
 *
 * \return Always a pointer to allocated memory of the size
 *         specified by a previous call to xme_hal_tls_alloc().
 */
void*
xme_hal_tls_get
(
    xme_hal_tls_handle_t tlsHandle
);

/**
 * \brief  Used instead of the original xme_hal_tls function.
 *
 * \param  tlsHandle Thread-local storage handle.
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_hal_tls_free
(
    xme_hal_tls_handle_t tlsHandle
);

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static uint16_t tlsSize = 0; ///< Size of thread-local storage object.
static void* tlsMem = NULL; ///< Memory allocated for thread-local storage.

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_hal_tls_handle_t
xme_hal_tls_alloc
(
    uint16_t size
)
{
    // Only one handle supported by this mock
    XME_ASSERT_RVAL(0 == tlsSize, XME_HAL_TLS_INVALID_TLS_HANDLE);

    // Remember the size
    tlsSize = size;

    // Always return 1
    return (xme_hal_tls_handle_t)1;
}

void*
xme_hal_tls_get
(
    xme_hal_tls_handle_t tlsHandle
)
{
    // Expect the handle to be 1
    XME_ASSERT_RVAL((xme_hal_tls_handle_t)1 == tlsHandle, NULL);

    // Allocate the memory if necessary
    if (NULL == tlsMem)
    {
        tlsMem = xme_fallback_malloc(tlsSize);
    }

    // Return a pointer to the memory
    return tlsMem;
}

xme_status_t
xme_hal_tls_free
(
    xme_hal_tls_handle_t tlsHandle
)
{
    // Expect the handle to be 1
    XME_ASSERT((xme_hal_tls_handle_t)1 == tlsHandle);

    // Free the memory
    if (NULL != tlsMem)
    {
        xme_fallback_free(tlsMem);
        tlsMem = NULL;
        tlsSize = 0;
    }

    // Return success
    return XME_STATUS_SUCCESS;
}
