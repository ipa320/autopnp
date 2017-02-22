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
 * $Id: sync.h 6316 2014-01-14 14:43:20Z geisinger $
 */

/**
 * \file Synchronization abstraction.
 */

#ifndef XME_HAL_SYNC_H
#define XME_HAL_SYNC_H

/**
 * \defgroup hal_sync Synchronization
 * @{
 *
 * \brief  Synchronization abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def XME_HAL_SYNC_RECURSIVE_LOCKING
 *
 * \brief Determines whether or not recursive locking is used.
 *
 * \details Recursive locking (i.e., XME_HAL_SYNC_RECURSIVE_LOCKING set to 1)
 *          means that a thread which acquired a critical section handle may
 *          subsequently acquire it. In this case, an equal number of unlocking
 *          operations are needed to eventually free the critical section.
 *          Non-recursive locking (i.e., XME_HAL_SYNC_RECURSIVE_LOCKING = 0)
 *          means that once any thread has acquired a critical section, all
 *          subsequent locking requests (including from the locking thread)
 *          will block until the critical section is unlocked by the locking
 *          thread.
 */
// Defined in sync_arch.h

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum xme_hal_sync_criticalSectionHandle_t
 *
 * \brief  Critical section handle.
 */
typedef enum
{
    XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE = 0, ///< Invalid critical section handle.
    XME_HAL_SYNC_MAX_CRITICAL_SECTION_HANDLE = XME_MAX_SYSTEM_VALUE ///< Largest possible critical section handle.
}
xme_hal_sync_criticalSectionHandle_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the synchronization abstraction.
 *
 * \retval XME_STATUS_SUCCESS the synchronization abstraction has been properly
 *         initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if synchronization abstraction
 *         initialization failed.
 */
// FIXME: Some implementations might return XME_STATUS_INTERNAL_ERROR?
xme_status_t
xme_hal_sync_init(void);

/**
 * \brief  Frees resources occupied by the synchronization abstraction.
 */
void
xme_hal_sync_fini(void);

/**
 * \brief  Allocates a new critical section handle.
 *
 * \note   Critical section handles allocated by a call to this function need
 *         to be deleted once they are not used any more with a call to
 *         xme_hal_sync_destroyCriticalSection().
 *
 * \return Returns a non-zero critical section handle on success and
 *         XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE on error.
 */
xme_hal_sync_criticalSectionHandle_t
xme_hal_sync_createCriticalSection(void);

/**
 * \brief  Destroys the given critical section object.
 *
 * \note   Do not use the critical section handle in any function
 *         after the critical section has been destroyed.
 *
 * \param  criticalSectionHandle Handle of the critical section to destroy.
 *
 * \retval XME_STATUS_SUCCESS if the critical section has been successfully
 *         destroyed.
 * \retval XME_STATUS_INVALID_HANDLE if the given critical section handle was
 *         invalid.
 */
xme_status_t
xme_hal_sync_destroyCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
);

/**
 * \brief  Waits for ownership of the specified critical section.
 *         The function returns when the calling task is granted ownership.
 *
 * \param  criticalSectionHandle Handle of the critical section to enter.
 *         Must specify a valid critical section handle.
 */
void
xme_hal_sync_enterCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
);

/**
 * \brief  Attempts to enter a critical section without blocking. If the call
 *         is successful, the calling task takes ownership of the critical
 *         section.
 *
 * \param  criticalSectionHandle Handle of the critical section to enter.
 *         Must specify a valid critical section handle.
 *
 * \retval XME_STATUS_SUCCESS if the calling task has successfully taken
 *         ownership of the specified critical section.
 * \retval XME_STATUS_WOULD_BLOCK if the critical section is currently
 *         in use by another task. Ownership of the calling task has not
 *         been taken.
 */
xme_status_t
xme_hal_sync_tryEnterCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
);

/**
 * \brief  Releases ownership of the specified critical section.
 *
 * \param  criticalSectionHandle Handle of the critical section to leave.
 *         Must specify a valid critical section handle.
 */
void
xme_hal_sync_leaveCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/sync_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_SYNC_H

