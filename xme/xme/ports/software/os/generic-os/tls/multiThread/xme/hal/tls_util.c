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
 * $Id: tls_util.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Thread-local storage abstraction (platform specific part: Windows).
 *
 */

/**
 * \addtogroup hal_tls
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/tls_util.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
xme_hal_tls_configStruct_t xme_hal_tls_config; ///< TLS configuration struct. 

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
_xme_hal_tls_init(void)
{
    if (0U == xme_hal_tls_config.initializationCount)
    {
        XME_ASSERT(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE == xme_hal_tls_config.criticalSectionHandle);
        XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(0 == XME_HAL_TABLE_ITEM_COUNT(xme_hal_tls_config.items)));

        XME_HAL_TABLE_INIT(xme_hal_tls_config.items);

        XME_CHECK(
            XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != (
                xme_hal_tls_config.criticalSectionHandle = xme_hal_sync_createCriticalSection()
            ),
            XME_STATUS_OUT_OF_RESOURCES
        );

        // Register the calling thread, which should be the "main thread",
        // if such concept exists on this platform
        {
            xme_status_t result = xme_hal_tls_registerThread();
            XME_ASSERT(XME_STATUS_SUCCESS == result);
        }
    }

    XME_ASSERT(xme_hal_tls_config.initializationCount < (unsigned int) -1);
    xme_hal_tls_config.initializationCount++;

    return XME_STATUS_SUCCESS;
}

void
_xme_hal_tls_fini(void)
{
    XME_ASSERT_NORVAL(xme_hal_tls_config.initializationCount > 0U);

    xme_hal_tls_config.initializationCount--;

    if (0U == xme_hal_tls_config.initializationCount)
    {
        // Deregister the calling thread, which should be the "main thread",
        // if such concept exists on this platform
        xme_hal_tls_deregisterThread();

        XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(0 == XME_HAL_TABLE_ITEM_COUNT(xme_hal_tls_config.items)));
        XME_HAL_TABLE_FINI(xme_hal_tls_config.items);

        (void) xme_hal_sync_destroyCriticalSection(xme_hal_tls_config.criticalSectionHandle);
        xme_hal_tls_config.criticalSectionHandle = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;
    }
}

xme_hal_tls_item_t*
_xme_hal_tls_addItem
(
    xme_hal_tls_handle_t* tlsHandle
)
{
    xme_hal_tls_item_t* item;

    XME_ASSERT_RVAL(NULL != tlsHandle, NULL);

    xme_hal_sync_enterCriticalSection(xme_hal_tls_config.criticalSectionHandle);
    {
        XME_CHECK_REC
        (
            0 != (*tlsHandle = (xme_hal_tls_handle_t)XME_HAL_TABLE_ADD_ITEM(xme_hal_tls_config.items)),
            NULL,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_tls_config.criticalSectionHandle);
            }
        );

        item = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_tls_config.items, *tlsHandle);
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_tls_config.criticalSectionHandle);

    return item;
}

xme_hal_tls_item_t*
_xme_hal_tls_getItem
(
    xme_hal_tls_handle_t tlsHandle
)
{
    xme_hal_tls_item_t* item;

    xme_hal_sync_enterCriticalSection(xme_hal_tls_config.criticalSectionHandle);
    {
        item = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_tls_config.items, tlsHandle);
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_tls_config.criticalSectionHandle);

    return item;
}

xme_status_t
_xme_hal_tls_removeItem
(
    xme_hal_tls_handle_t tlsHandle
)
{
    xme_status_t rval;

    xme_hal_sync_enterCriticalSection(xme_hal_tls_config.criticalSectionHandle);
    {
        rval = XME_HAL_TABLE_REMOVE_ITEM(xme_hal_tls_config.items, (xme_hal_table_rowHandle_t)tlsHandle);
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_tls_config.criticalSectionHandle);

    return rval;
}

uint16_t
_xme_hal_tls_getSize
(
    xme_hal_tls_handle_t tlsHandle
)
{
    xme_hal_tls_item_t* item;

    item = _xme_hal_tls_getItem(tlsHandle);
    XME_CHECK(NULL != item, 0U);

    return item->size;
}

/**
 * @}
 */
