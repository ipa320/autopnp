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
 * $Id: sync_arch.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Synchronization abstraction (platform specific part: Windows).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/sync.h"

#include "xme/core/component.h"

#include "xme/hal/include/table.h"

#include <Windows.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_hal_sync_criticalSectionDescriptor_t
 *
 * \brief  Synchronization descriptor.
 */
typedef struct
{
    CRITICAL_SECTION mutex; ///< Mutex object for implementation of critical section.
}
xme_hal_sync_criticalSectionDescriptor_t;

/**
 * \struct xme_hal_sync_configStruct_t
 *
 * \brief  Synchronization configuration structure.
 */
XME_COMPONENT_CONFIG_STRUCT
(
    xme_hal_sync,
    // private
    unsigned int initializationCount; ///< Number of times this component has been initialized.
    CRITICAL_SECTION criticalSectionMutex; ///< Mutex for shared access to the list of critical sections.
    XME_HAL_TABLE(xme_hal_sync_criticalSectionDescriptor_t, criticalSections, XME_HAL_DEFINES_MAX_CRITICAL_SECTION_DESCRIPTORS); // criticalSectionHandle is an index into this table ///< List of critical sections.
);

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static xme_hal_sync_configStruct_t xme_hal_sync_config = { 0U }; ///< Configuration structure of this component.

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_sync_init(void)
{
    if (0U == xme_hal_sync_config.initializationCount)
    {
        XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(0 == XME_HAL_TABLE_ITEM_COUNT(xme_hal_sync_config.criticalSections)));

        XME_HAL_TABLE_INIT(xme_hal_sync_config.criticalSections);

        XME_CHECK
        (
            0 != InitializeCriticalSectionAndSpinCount(&xme_hal_sync_config.criticalSectionMutex, 0x00000400),
            XME_STATUS_OUT_OF_RESOURCES
        );
    }

    XME_ASSERT(xme_hal_sync_config.initializationCount < (unsigned int) -1);
    xme_hal_sync_config.initializationCount++;

    return XME_STATUS_SUCCESS;
}

void
xme_hal_sync_fini(void)
{
    XME_ASSERT_NORVAL(xme_hal_sync_config.initializationCount > 0U);

    xme_hal_sync_config.initializationCount--;

    if (0U == xme_hal_sync_config.initializationCount)
    {
        // Synchronize access to the critical sections list mutex
        EnterCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
        {
            XME_HAL_TABLE_FINI(xme_hal_sync_config.criticalSections);
        }
        LeaveCriticalSection(&xme_hal_sync_config.criticalSectionMutex);

        // Destroy the critical sections list mutex
        DeleteCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
    }
}

xme_hal_sync_criticalSectionHandle_t
xme_hal_sync_createCriticalSection(void)
{
    xme_hal_sync_criticalSectionHandle_t newCriticalSectionHandle;
    xme_hal_sync_criticalSectionDescriptor_t* criticalSectionDesc;

    XME_ASSERT_RVAL(xme_hal_sync_config.initializationCount > 0U, XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE);

    // Synchronize access to the critical sections list mutex
    EnterCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
    {
        // Allocate a unique critical section handle
        newCriticalSectionHandle = (xme_hal_sync_criticalSectionHandle_t) XME_HAL_TABLE_ADD_ITEM(xme_hal_sync_config.criticalSections);
    }
    LeaveCriticalSection(&xme_hal_sync_config.criticalSectionMutex);

    // Ensure that the handle is valid
    XME_CHECK
    (
        XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != newCriticalSectionHandle,
        XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE
    );

    // Initialize the critical section descriptor
    criticalSectionDesc = (xme_hal_sync_criticalSectionDescriptor_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_sync_config.criticalSections, newCriticalSectionHandle);

    XME_ASSERT_RVAL(NULL != criticalSectionDesc, XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE);

    XME_CHECK_REC
    (
        0 != InitializeCriticalSectionAndSpinCount(&criticalSectionDesc->mutex, 0x00000400),
        XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE,
        {
            EnterCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
            {
                XME_HAL_TABLE_REMOVE_ITEM(xme_hal_sync_config.criticalSections, (xme_hal_table_rowHandle_t)newCriticalSectionHandle);
            }
            LeaveCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
        }
    );

    return newCriticalSectionHandle;
}

xme_status_t
xme_hal_sync_destroyCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
)
{
    xme_hal_sync_criticalSectionDescriptor_t* criticalSectionDesc;
    BOOL wasUnlocked = true;

    XME_ASSERT(xme_hal_sync_config.initializationCount > 0U);

    // Synchronize access to the critical sections list mutex
    EnterCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
    {
        // Retrieve the critical section descriptor
        criticalSectionDesc = (xme_hal_sync_criticalSectionDescriptor_t*)XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_sync_config.criticalSections, criticalSectionHandle);

        XME_CHECK_REC
        (
            NULL != criticalSectionDesc,
            XME_STATUS_INVALID_HANDLE,
            {
                LeaveCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
            }
        );

#if defined(DEBUG) && !defined(DOXYGEN)
        // In debug mode, verify that the critical section is actually unlocked
        wasUnlocked = TryEnterCriticalSection(&criticalSectionDesc->mutex);
        if (wasUnlocked)
        {
            // Clean up
            LeaveCriticalSection(&criticalSectionDesc->mutex);
        }
#endif // #if defined(DEBUG) && !defined(DOXYGEN)

        DeleteCriticalSection(&criticalSectionDesc->mutex);

        XME_HAL_TABLE_REMOVE_ITEM(xme_hal_sync_config.criticalSections, (xme_hal_table_rowHandle_t)criticalSectionHandle);
    }
    LeaveCriticalSection(&xme_hal_sync_config.criticalSectionMutex);

    XME_ASSERT(wasUnlocked);

    return XME_STATUS_SUCCESS;
}

void
xme_hal_sync_enterCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
)
{
    xme_hal_sync_criticalSectionDescriptor_t* criticalSectionDesc;

    XME_ASSERT_NORVAL(xme_hal_sync_config.initializationCount > 0U);

    // Synchronize access to the critical sections list mutex
    EnterCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
    {
        // Retrieve the critical section descriptor
        criticalSectionDesc = (xme_hal_sync_criticalSectionDescriptor_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_sync_config.criticalSections, criticalSectionHandle);
    }
    LeaveCriticalSection(&xme_hal_sync_config.criticalSectionMutex);

    XME_ASSERT_NORVAL(NULL != criticalSectionDesc);

    EnterCriticalSection(&criticalSectionDesc->mutex);
}

xme_status_t
xme_hal_sync_tryEnterCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
)
{
    xme_hal_sync_criticalSectionDescriptor_t* criticalSectionDesc;

    XME_ASSERT(xme_hal_sync_config.initializationCount > 0U);

    // Synchronize access to the critical sections list mutex
    EnterCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
    {
        // Retrieve the critical section descriptor
        criticalSectionDesc = (xme_hal_sync_criticalSectionDescriptor_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_sync_config.criticalSections, criticalSectionHandle);
    }
    LeaveCriticalSection(&xme_hal_sync_config.criticalSectionMutex);

    XME_ASSERT(NULL != criticalSectionDesc);

    return TryEnterCriticalSection(&criticalSectionDesc->mutex) ? XME_STATUS_SUCCESS : XME_STATUS_WOULD_BLOCK;
}

void
xme_hal_sync_leaveCriticalSection
(
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle
)
{
    xme_hal_sync_criticalSectionDescriptor_t* criticalSectionDesc;

    XME_ASSERT_NORVAL(xme_hal_sync_config.initializationCount > 0U);

    // Synchronize access to the critical sections list mutex
    EnterCriticalSection(&xme_hal_sync_config.criticalSectionMutex);
    {
        // Retrieve the critical section descriptor
        criticalSectionDesc = (xme_hal_sync_criticalSectionDescriptor_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_sync_config.criticalSections, criticalSectionHandle);
    }
    LeaveCriticalSection(&xme_hal_sync_config.criticalSectionMutex);

    XME_ASSERT_NORVAL(NULL != criticalSectionDesc);

    LeaveCriticalSection(&criticalSectionDesc->mutex);
}
