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
 * $Id: sharedPtr.h 4664 2013-08-13 09:06:08Z ruiz $
 */

/**
 * \file
 * \brief Shared memory abstraction.
 */

#ifndef XME_HAL_SHAREDPTR_H
#define XME_HAL_SHAREDPTR_H

/**
 * \defgroup hal_sharedPtr Shared memory abstraction
 * @{
 *
 * \brief This component is for shared memory use.
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
 * \enum xme_hal_sharedPtr_t
 * \brief  Buffer handle.
 */
typedef enum
{
    XME_HAL_SHAREDPTR_INVALID_POINTER = 0, ///< Invalid buffer handle.
    XME_HAL_SHAREDPTR_MAX_POINTER = XME_MAX_SYSTEM_VALUE ///< Largest possible buffer handle.
}
xme_hal_sharedPtr_t;

/**
 * \typedef xme_hal_sharedPtr_referenceCount_t
 * \brief Type of reference count.
 */
typedef uint16_t xme_hal_sharedPtr_referenceCount_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the shared memory component.
 *         At most one component of this type can be present on every node.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the shared memory component has been
 *            properly initialized.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the shared memory initialization failed.
 */
xme_status_t
xme_hal_sharedPtr_init(void);

/**
 * \brief  Frees all resources occupied by the shared memory component.
 *         At most one component of this type can be present on every node.
 */
void
xme_hal_sharedPtr_fini(void);

/**
 * \brief  Returns the size of the buffer corresponding to the given handle.
 *
 * \param  sharedPtr Buffer handle.
 *
 * \return Size of the buffer corresponding to the given handle.
 */
uint16_t
xme_hal_sharedPtr_getSize
(
    xme_hal_sharedPtr_t sharedPtr
);

/**
 * \brief  Allocates memory for storing the given data, copies the data to the
 *         memory and returns a handle for referencing the memory.
 *
 * \param  size Size of buffer to be allocated.
 * \param  *data Pointer to the data to be copied to the memory.
 *
 * \return Returns a handle to the data buffer on success and
 *         xme_HAL_SHAREDPTR_INVALID_POINTER otherwise.
 */
xme_hal_sharedPtr_t
xme_hal_sharedPtr_createFromPointer
(
    uint16_t size,
    void *data
);

/**
 * \brief  Allocates reference counted memory for storing data of the given
 *         size and returns a handle for referencing the memory.
 *
 * \param  size Size of memory to be allocated.
 *
 * \return Returns a handle to the data buffer on success and
 *         xme_HAL_SHAREDPTR_INVALID_POINTER otherwise.
 */
xme_hal_sharedPtr_t
xme_hal_sharedPtr_create
(
    uint16_t size
);

/**
 * \brief  Reallocates the given reference counted memory with a new size.
 *         The content of the memory block is preserved up to the smaller
 *         of the current and the new size.
 *
 * \param  sharedPtr Handle of the memory buffer to reallocate.
 * \param  size New size for the memory block, in bytes. It is illegal to
 *         pass zero for this parameter.
 *
 * \return Returns a handle to the reallocated data buffer on success and
 *         xme_HAL_SHAREDPTR_INVALID_POINTER otherwise.
 */
xme_hal_sharedPtr_t
xme_hal_sharedPtr_realloc
(
    xme_hal_sharedPtr_t sharedPtr,
    uint16_t size
);

/**
 * \brief  Increments the reference counter of the reference-counted memory
 *         referred to by the given buffer handle.
 *
 *         This function is typically called by a software component that
 *         wants to store the data for private use.
 *
 * \note   A component calling this function has to call
 *         xme_hal_sharedPtr_destroy() at a later point in time to free the
 *         reference-counted memory.
 *
 * \param  dataHandle Handle to the data buffer whose reference count should
 *         be incremented.
 *
 * \return Returns a handle to the data buffer on success and
 *         xme_HAL_SHAREDPTR_INVALID_POINTER otherwise.
 */
xme_hal_sharedPtr_t
xme_hal_sharedPtr_retain
(
    xme_hal_sharedPtr_t dataHandle
);

/**
 * \brief  Decrements the reference counter of the reference-counted memory
 *         referred to by the given buffer handle.
 *
 *         If the reference count reaches zero, the memory is freed.
 *         This function is typically called by a software component that
 *         has store data for private use and wants to discard the data.
 *
 * \param  sharedPtr Handle to the data buffer whose reference count should
 *         be decremented.
 *
 * \return Returns a handle to the data buffer on success and
 *         xme_HAL_SHAREDPTR_INVALID_POINTER otherwise.
 */
void
xme_hal_sharedPtr_destroy
(
    xme_hal_sharedPtr_t sharedPtr
);

/**
 * \brief  Returns the reference count of the memory associated with the given
 *         buffer handle.
 *
 * \param  sharedPtr Handle to the data buffer.
 *
 * \return Number of references to the memory associated with the given buffer
 *         handle.
 */
xme_hal_sharedPtr_referenceCount_t
xme_hal_sharedPtr_getReferenceCount
(
    xme_hal_sharedPtr_t sharedPtr
);


/**
 * \brief  Returns a pointer to the memory associated with the given buffer
 *         handle.
 *
 * \param  sharedPtr Handle to the data buffer.
 * \return Pointer to the memory associated with the given buffer handle.
 */
void*
xme_hal_sharedPtr_getPointer
(
    xme_hal_sharedPtr_t sharedPtr
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/sharedPtr_arch.h"

/**
 * @}
 */


#endif // #ifndef XME_HAL_SHAREDPTR_H
