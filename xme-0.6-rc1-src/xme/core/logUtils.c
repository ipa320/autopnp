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
 * $Id: logUtils.c 5044 2013-09-11 16:57:31Z geisinger $
 */

/**
 * \file
 *         Logging utility functions.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/logUtils.h"

#include "xme/hal/include/sync.h"
#include "xme/hal/include/table.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_core_log_componentRegistryItem_t
 *
 * \brief  Component registry item.
 */
typedef struct
{
    xme_core_log_componentID component; ///< Identifier of the component (key of the data structure).
    const char* acronym; ///< Human-readable (short) name of the component.
    xme_log_severity_t minSeverity; ///< Current log level for this component.
} xme_core_log_componentRegistryItem_t;

/******************************************************************************/
/***   Static variables                                                     ***/
/******************************************************************************/
static bool xme_core_log_componentRegistryInitialized = false;

static xme_hal_sync_criticalSectionHandle_t xme_core_log_componentRegistryCriticalSectionHandle = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;

// TODO: A hashmap with key "component" would be a better data structure for this
// TODO: Move maximum static size to Options.cmake
static XME_HAL_TABLE(xme_core_log_componentRegistryItem_t, xme_core_log_componentRegistry, 32);

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
void
xme_core_log_initComponentRegistry(void);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
xme_core_log_initComponentRegistry(void)
{
    xme_status_t status;

    // Return immediately if we are already initialized
    XME_CHECK(!xme_core_log_componentRegistryInitialized, XME_CHECK_RVAL_VOID);

    // FIXME: xme_hal_sync_fini() is never called on this handle!
    status = xme_hal_sync_init();
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

    // FIXME: xme_hal_sync_destroyCriticalSection() is never called on this handle!
    xme_core_log_componentRegistryCriticalSectionHandle = xme_hal_sync_createCriticalSection();
    XME_ASSERT_NORVAL(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != xme_core_log_componentRegistryCriticalSectionHandle);

    // FIXME: _FINI() is never called on this table!
    XME_HAL_TABLE_INIT(xme_core_log_componentRegistry);

    xme_core_log_componentRegistryInitialized = true;
}

xme_status_t
xme_core_log_registerComponent
(
    xme_core_log_componentID component,
    const char* componentAcronym,
    xme_log_severity_t defaultComponentMinSeverity
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_log_componentRegistryItem_t* item;

    xme_core_log_initComponentRegistry();

    xme_hal_sync_enterCriticalSection(xme_core_log_componentRegistryCriticalSectionHandle);
    {
        handle = XME_HAL_TABLE_ADD_ITEM(xme_core_log_componentRegistry);
    }
    xme_hal_sync_leaveCriticalSection(xme_core_log_componentRegistryCriticalSectionHandle);

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_OUT_OF_RESOURCES);

    item = (xme_core_log_componentRegistryItem_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_log_componentRegistry, handle);
    XME_ASSERT(NULL != item);

    item->component = component;
    item->acronym = componentAcronym;
    item->minSeverity = defaultComponentMinSeverity;

    return XME_STATUS_SUCCESS;
}

xme_log_severity_t
xme_core_log_getComponentLogLevel
(
    xme_core_log_componentID component
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_log_componentRegistryItem_t* item;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    item = NULL;

    xme_hal_sync_enterCriticalSection(xme_core_log_componentRegistryCriticalSectionHandle);
    {
        XME_HAL_TABLE_GET_NEXT(xme_core_log_componentRegistry, xme_hal_table_rowHandle_t,
            handle, xme_core_log_componentRegistryItem_t, item, item->component == component);
    }
    xme_hal_sync_leaveCriticalSection(xme_core_log_componentRegistryCriticalSectionHandle);

    // For unregistered components, we return the "default console log level"
    return (NULL != item) ? item->minSeverity : XME_LOG_CONSOLE_MIN_SEVERITY;
}

const char*
xme_core_log_getComponentAcronym
(
    xme_core_log_componentID component
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_log_componentRegistryItem_t* item;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    item = NULL;

    xme_hal_sync_enterCriticalSection(xme_core_log_componentRegistryCriticalSectionHandle);
    {
        XME_HAL_TABLE_GET_NEXT(xme_core_log_componentRegistry, xme_hal_table_rowHandle_t,
            handle, xme_core_log_componentRegistryItem_t, item, item->component == component);
    }
    xme_hal_sync_leaveCriticalSection(xme_core_log_componentRegistryCriticalSectionHandle);

    // For unregistered components, we return NULL
    return (NULL != item) ? item->acronym : NULL;
}

xme_status_t
xme_core_log_setComponentLogLevel
(
    xme_core_log_componentID component,
    xme_log_severity_t componentMinSeverity
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_log_componentRegistryItem_t* item;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    item = NULL;

    xme_hal_sync_enterCriticalSection(xme_core_log_componentRegistryCriticalSectionHandle);
    {
        XME_HAL_TABLE_GET_NEXT(xme_core_log_componentRegistry, xme_hal_table_rowHandle_t,
            handle, xme_core_log_componentRegistryItem_t, item, item->component == component);
    }
    xme_hal_sync_leaveCriticalSection(xme_core_log_componentRegistryCriticalSectionHandle);

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NO_SUCH_VALUE);

    XME_ASSERT(NULL != item);
    item->minSeverity = componentMinSeverity;

    return XME_STATUS_SUCCESS;
}
