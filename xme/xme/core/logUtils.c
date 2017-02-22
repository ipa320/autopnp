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
 * $Id: logUtils.c 7794 2014-03-12 17:13:22Z geisinger $
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
 * \struct xme_core_log_logUtils_componentRegistryItem_t
 *
 * \brief  Component registry item.
 */
typedef struct
{
    xme_core_log_componentID_t component; ///< Identifier of the component (key of the data structure).
    const char* acronym; ///< Human-readable (short) name of the component.
    xme_log_severity_t minSeverity; ///< Current log level for this component.
} xme_core_log_logUtils_componentRegistryItem_t;

/**
 * \typedef xme_core_log_logUtils_configStruct_t
 *
 * \brief  Logging utility functions configuration structure.
 */
typedef struct
{
    unsigned int initializationCount; ///< Number of times this component has been initialized.
    xme_hal_sync_criticalSectionHandle_t componentRegistryMutex;
    // FIXME: A hashmap with key "component" would be a better data structure for this
    XME_HAL_TABLE(xme_core_log_logUtils_componentRegistryItem_t, componentRegistry, XME_CORE_LOG_MAX_COMPONENT_REGISTRY_ITEMS);
} xme_core_log_logUtils_configStruct_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static xme_core_log_logUtils_configStruct_t xme_core_log_logUtils_configStruct;

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
// Documented in log.c
extern xme_log_severity_t xme_core_log_minimumLogSeverityLevel;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief Returns the component registry item corresponding to the given
 *        component type or instance identifier.
 *
 * \param[in] component Component type or instance identifier to retrieve item
 *            for.
 *
 * \return Returns the corresponding component registry item.
 *         If the component type or instance identifier is unknown,
 *         returns NULL.
 */
static xme_core_log_logUtils_componentRegistryItem_t*
_xme_core_log_getComponentRegistryItem
(
    xme_core_log_componentID_t component
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

static xme_core_log_logUtils_componentRegistryItem_t*
_xme_core_log_getComponentRegistryItem
(
    xme_core_log_componentID_t component
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_log_logUtils_componentRegistryItem_t* item;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    item = NULL;

    xme_hal_sync_enterCriticalSection(xme_core_log_logUtils_configStruct.componentRegistryMutex);
    {
        XME_HAL_TABLE_GET_NEXT(xme_core_log_logUtils_configStruct.componentRegistry, xme_hal_table_rowHandle_t,
            handle, xme_core_log_logUtils_componentRegistryItem_t, item, item->component == component);
    }
    xme_hal_sync_leaveCriticalSection(xme_core_log_logUtils_configStruct.componentRegistryMutex);

    return item;
}

xme_status_t
xme_core_log_logUtils_init(void)
{
    xme_status_t status;

    // Return immediately if we are already initialized
    xme_core_log_logUtils_configStruct.initializationCount++;
    XME_CHECK(1U == xme_core_log_logUtils_configStruct.initializationCount, XME_STATUS_SUCCESS);

    // Initialize depending HAL components
    status = xme_hal_sync_init();
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    // Initialize data structures
    xme_core_log_logUtils_configStruct.componentRegistryMutex = xme_hal_sync_createCriticalSection();
    XME_ASSERT(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != xme_core_log_logUtils_configStruct.componentRegistryMutex);

    XME_HAL_TABLE_INIT(xme_core_log_logUtils_configStruct.componentRegistry);

    return XME_STATUS_SUCCESS;
}

void
xme_core_log_logUtils_fini(void)
{
    xme_status_t status;

    XME_ASSERT_NORVAL(xme_core_log_logUtils_configStruct.initializationCount > 0U);

    // Return immediately if we should not yet finalize
    xme_core_log_logUtils_configStruct.initializationCount--;
    XME_CHECK(0U == xme_core_log_logUtils_configStruct.initializationCount, XME_CHECK_RVAL_VOID);

    // Finalize data structures
    XME_HAL_TABLE_FINI(xme_core_log_logUtils_configStruct.componentRegistry);

    status = xme_hal_sync_destroyCriticalSection(xme_core_log_logUtils_configStruct.componentRegistryMutex);
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

    // Finalize depending HAL components
    xme_hal_sync_fini();
}

xme_status_t
xme_core_log_registerComponent
(
    xme_core_log_componentID_t component,
    const char* componentAcronym,
    xme_log_severity_t defaultComponentMinSeverity
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_log_logUtils_componentRegistryItem_t* item;

    item = _xme_core_log_getComponentRegistryItem(component);
    XME_CHECK(NULL == item, XME_STATUS_ALREADY_EXIST);

    xme_hal_sync_enterCriticalSection(xme_core_log_logUtils_configStruct.componentRegistryMutex);
    {
        handle = XME_HAL_TABLE_ADD_ITEM(xme_core_log_logUtils_configStruct.componentRegistry);
    }
    xme_hal_sync_leaveCriticalSection(xme_core_log_logUtils_configStruct.componentRegistryMutex);

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_OUT_OF_RESOURCES);

    item = (xme_core_log_logUtils_componentRegistryItem_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_log_logUtils_configStruct.componentRegistry, handle);
    XME_ASSERT(NULL != item);

    item->component = component;
    item->acronym = componentAcronym;
    item->minSeverity = defaultComponentMinSeverity;

    return XME_STATUS_SUCCESS;
}

xme_log_severity_t
xme_core_log_getComponentLogLevel
(
    xme_core_log_componentID_t component
)
{
    xme_core_log_logUtils_componentRegistryItem_t* item;

    item = _xme_core_log_getComponentRegistryItem(component);

    // For unregistered components, we return the "default console log level"
    return (NULL != item) ? item->minSeverity : xme_core_log_minimumLogSeverityLevel;
}

const char*
xme_core_log_getComponentAcronym
(
    xme_core_log_componentID_t component
)
{
    xme_core_log_logUtils_componentRegistryItem_t* item;

    item = _xme_core_log_getComponentRegistryItem(component);

    // For unregistered components, we return NULL
    return (NULL != item) ? item->acronym : NULL;
}

xme_status_t
xme_core_log_setComponentLogLevel
(
    xme_core_log_componentID_t component,
    xme_log_severity_t componentMinSeverity
)
{
    xme_core_log_logUtils_componentRegistryItem_t* item;

    item = _xme_core_log_getComponentRegistryItem(component);
    XME_CHECK(NULL != item, XME_STATUS_NO_SUCH_VALUE);

    item->minSeverity = componentMinSeverity;

    return XME_STATUS_SUCCESS;
}
