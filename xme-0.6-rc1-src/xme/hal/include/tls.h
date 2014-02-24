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
 * $Id: tls.h 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 * \brief Thread-local storage abstraction.
 */

#ifndef XME_HAL_TLS_H
#define XME_HAL_TLS_H

/**
 * \defgroup hal_tls Thread-local storage
 * @{
 *
 * \brief  Thread-local storage abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum xme_hal_tls_handle_t
 *
 * \brief  Thread-local storage handle.
 */
typedef enum
{
    XME_HAL_TLS_INVALID_TLS_HANDLE = 0, ///< Invalid thread-local storage handle.
    XME_HAL_TLS_MAX_TLS_HANDLE = XME_MAX_SYSTEM_VALUE ///< Largest possible thread-local storage handle.
}
xme_hal_tls_handle_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the thread-local storage abstraction.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the thread-local storage abstraction
 *            has been properly initialized.
 * \retval XME_CORE_STATUS_OUT_OF_RESOURCES if thread-local storage
 *            abstraction initialization failed.
 */
xme_status_t
xme_hal_tls_init(void);

/**
 * \brief  Frees resources occupied by the thread-local storage abstraction.
 */
void
xme_hal_tls_fini(void);

/**
 * \brief  Allocates a block of thread-local storage with the given size and
 *         returns a handle to the memory block.
 *
 * A distinguish memory block of the given size will be immediately
 * available for all threads that have been registered to the thread
 * local storage abstraction by calling xme_hal_tls_registerThread()
 * before. Threads created after calling this function will receive
 * the same block of memory as soon as they call the
 * ::xme_hal_tls_registerThread() function. The handle has to be shared
 * among the threads to access the storage.
 *
 * When a thread first accesses the memory block using
 * ::xme_hal_tls_get(), its memory block is guaranteed to be initialized
 * to all zeroes.
 *
 * \param  size Size of the thread-local storage memory block to allocate for
 *         each registered thread. This parameter must not be zero.
 *
 * \return Return a non-zero thread-local storage handle on success and
 *         ::XME_HAL_TLS_INVALID_TLS_HANDLE on error.
 */
xme_hal_tls_handle_t
xme_hal_tls_alloc
(
    uint16_t size
);

/**
 * \brief  Retrieves a pointer to the thread-local storage block corresponding
 *         to the given handle for the calling thread.
 *
 * The size of the memory block corresponds to the value passed to
 * ::xme_hal_tls_alloc() that returned the given thread-local storage
 * handle. If this is the first time the memory pointer is retrieved
 * by a specific thread, the memory content is guaranteed to be
 * initialized to all zeroes.
 *
 * \param  tlsHandle Thread-local storage handle to retrieve the memory pointer
 *         for.
 *
 * \return Returns a pointer to the thread-local storage block corresponding
 *         to the given handle for the calling thread. In case of an error,
 *         the function might return a NULL pointer (e.g., if the calling
 *         thread has not been registered by using ::xme_hal_tls_registerThread()).
 */
void*
xme_hal_tls_get
(
    xme_hal_tls_handle_t tlsHandle
);

/**
 * \brief  Returns the size of the thread-local storage block corresponding
 *         to the given handle.
 *
 * If the given thread-local storage handle is valid, the returned
 * value is the same for all registered threads calling this function.
 *
 * \param  tlsHandle Thread-local storage handle to retrieve the memory size
 *         for.
 *
 * \return Returns a non-zero size of the thread-local storage block
 *         corresponding to the given handle. If the given handle is invalid,
 *         zero is returned.
 */
uint16_t
xme_hal_tls_getSize
(
    xme_hal_tls_handle_t tlsHandle
);

/**
 * \brief  Frees the thread-local storage block corresponding to the given
 *         handle for all registered threads.
 *
 * After calling this function, no thread can use the specified
 * thread-local storage handle any more or read from or write to the
 * memory block assicated to it.
 *
 * \param  tlsHandle Thread-local storage handle for which the corresponding
 *         memory should be freed.
 * \retval XME_CORE_STATUS_SUCCESS if the thread-local storage was successfully freed.
 * \retval XME_CORE_STATUS_INVALID_HANDLE if the given TLS handle is invalid.
 */
xme_status_t
xme_hal_tls_free
(
    xme_hal_tls_handle_t tlsHandle
);

/**
 * \brief  Registers the calling thread for use of the thread-local storage
 *         abstraction.
 *
 * Every thread that will use the thread-local storage abstraction
 * has to be registered with a call to this function.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the calling thread has been
 *            successfully registered.
 * \retval XME_CORE_STATUS_OUT_OF_RESOURCES if thread registration has
 *            failed.
 */
xme_status_t
xme_hal_tls_registerThread(void);

/**
 * \brief  Deregisters the calling thread from use of the thread-local storage
 *         abstraction and frees the allocated memory.
 *
 * To prevent memory leaks, this function needs to be called from each
 * thread that called xme_hal_tls_registerThread() before it
 * terminates.
 */
void
xme_hal_tls_deregisterThread(void);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/tls_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_TLS_H
