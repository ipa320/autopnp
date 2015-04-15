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
 * $Id: tlsSyncMock.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         A mock of xme_hal_sync, to be used with the xme_hal_tls tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/sync.h"

#include <stdbool.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Used instead of the original xme_hal_sync function.
 *
 * \note   This mock only supports one critical section handle.
 *
 * \retval 1 for the first allocated handle.
 * \retval XME_HAL_TLS_INVALID_HANDLE otherwise.
 */
xme_hal_sync_criticalSectionHandle_t
xme_hal_sync_createCriticalSection(void);

/**
 * \brief  Used instead of the original xme_hal_tls function.
 *
 * \param  criticalSectionHandle Critical section handle.
 *
 * \retval XME_STATUS_SUCCESS if criticalSectionHandle is 1.
 * \retval XME_STATUS_INVALID_HANDLE otherwise.
 */
xme_status_t
xme_hal_sync_destroyCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
);

/**
 * \brief  Used instead of the original xme_hal_tls function.
 *
 * \param  criticalSectionHandle Critical section handle.
 */
void
xme_hal_sync_enterCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
);

/**
 * \brief  Used instead of the original xme_hal_tls function.
 *
 * \param  criticalSectionHandle Critical section handle.
 *
 * \retval XME_STATUS_SUCCESS if criticalSectionHandle is 1.
 * \retval XME_STATUS_INVALID_HANDLE otherwise.
 */
xme_status_t
xme_hal_sync_tryEnterCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
);

/**
 * \brief  Used instead of the original xme_hal_tls function.
 *
 * \param  criticalSectionHandle Critical section handle.
 */
void
xme_hal_sync_leaveCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
);

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static bool isHandleValid = false; ///< Whether the first critical section handle has been created.
static bool isSectionLocked = false; ///< Whether the critical section is currently locked.

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_hal_sync_criticalSectionHandle_t
xme_hal_sync_createCriticalSection(void)
{
    // Check whether this is the first handle being created
    XME_ASSERT_RVAL(!isHandleValid, XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE);

    // Mark the first handle as created
    isHandleValid = true;

    // Always return 1
    return (xme_hal_sync_criticalSectionHandle_t)1;
}

xme_status_t
xme_hal_sync_destroyCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
)
{
    // Expect the handle to be allocated
    XME_ASSERT_RVAL(isHandleValid, XME_STATUS_INVALID_HANDLE);

    // Expect the critical section not to be locked
    XME_ASSERT_RVAL(!isSectionLocked, XME_STATUS_INVALID_HANDLE);

    // Expect the handle to be 1
    XME_ASSERT_RVAL((xme_hal_sync_criticalSectionHandle_t)1 == criticalSectionHandle, XME_STATUS_INVALID_HANDLE);

    // Mark the first handle as destroyed
    isHandleValid = false;

    // Always return 1
    return XME_STATUS_SUCCESS;
}

void
xme_hal_sync_enterCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
)
{
    // Expect the handle to be allocated
    XME_ASSERT_NORVAL(isHandleValid);

    // Expect the critical section not to be locked
    XME_ASSERT_NORVAL(!isSectionLocked);

    // Expect the handle to be 1
    XME_ASSERT_NORVAL((xme_hal_sync_criticalSectionHandle_t)1 == criticalSectionHandle);

    // Mark the section as locked
    isSectionLocked = true;
}

xme_status_t
xme_hal_sync_tryEnterCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
)
{
    // Expect the handle to be allocated
    XME_ASSERT_RVAL(isHandleValid, XME_STATUS_INVALID_HANDLE);

    // Expect the handle to be 1
    XME_ASSERT_RVAL((xme_hal_sync_criticalSectionHandle_t)1 == criticalSectionHandle, XME_STATUS_INVALID_HANDLE);

    if (isSectionLocked)
    {
        // Indicate that the operation would block
        return XME_STATUS_WOULD_BLOCK;
    }
    else
    {
        // Mark the section as locked
        isSectionLocked = true;

        // Return success
        return XME_STATUS_SUCCESS;
    }
    
    // Expect the critical section not to be locked
    XME_ASSERT_RVAL(!isSectionLocked, XME_STATUS_INVALID_HANDLE);
}

void
xme_hal_sync_leaveCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
)
{
    // Expect the handle to be allocated
    XME_ASSERT_NORVAL(isHandleValid);

    // Expect the critical section to be locked
    XME_ASSERT_NORVAL(isSectionLocked);

    // Expect the handle to be 1
    XME_ASSERT_NORVAL((xme_hal_sync_criticalSectionHandle_t)1 == criticalSectionHandle);

    // Mark the section as unlocked
    isSectionLocked = false;
}
