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
 * $Id: manifestRepository.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Manifest Repository.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/manifestRepository/include/manifestRepository.h"

#include "xme/core/log.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \brief manifestTable
 * \details the table with all component manifests.
 */
static XME_HAL_TABLE
(
    xme_core_componentManifest_t,
    manifestTable,
    XME_CORE_MANIFESTREPOSITORY_MAX_MANIFESTS
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_manifestRepository_init(void)
{
    XME_HAL_TABLE_INIT(manifestTable);

    return XME_STATUS_SUCCESS;
}

void
xme_core_manifestRepository_fini(void)
{
    XME_HAL_TABLE_FINI(manifestTable);
}

xme_status_t
xme_core_manifestRepository_findComponentManifest
(
    xme_core_componentType_t componentType,
    xme_core_componentManifest_t* outComponentManifest
)
{
    xme_hal_table_rowHandle_t rowHandle;
    xme_core_componentManifest_t* componentManifestItem;

    rowHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    componentManifestItem = NULL;

    XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentType, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        manifestTable,
        xme_hal_table_rowHandle_t,
        rowHandle,
        xme_core_componentManifest_t,
        componentManifestItem,
        componentManifestItem->componentType == componentType
    );

    if (NULL == componentManifestItem)
    {
        return XME_STATUS_NOT_FOUND;
    }

    (void) xme_hal_mem_copy(outComponentManifest, componentManifestItem, sizeof(xme_core_componentManifest_t));

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_manifestRepository_addComponentManifest
(
    const xme_core_componentManifest_t* componentManifest,
    bool replace
)
{
    xme_hal_table_rowHandle_t rowHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_componentManifest_t* componentManifestEntry = NULL;

    XME_CHECK(NULL != componentManifest, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentManifest->componentType, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        manifestTable,
        xme_hal_table_rowHandle_t,
        rowHandle,
        xme_core_componentManifest_t,
        componentManifestEntry,
        componentManifest->componentType == componentManifestEntry->componentType
    );

    if (NULL == componentManifestEntry)
    {
        xme_hal_table_rowHandle_t row;

        // add to the manifest entry
        row = XME_HAL_TABLE_ADD_ITEM(manifestTable); // add a new element
        componentManifestEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(manifestTable, row);

        (void) xme_hal_mem_copy(componentManifestEntry, componentManifest, sizeof(xme_core_componentManifest_t));

        return XME_STATUS_SUCCESS;
    }
    else
    {
        XME_CHECK(true == replace, XME_STATUS_ALREADY_EXIST);

        // just set the new manifest
        (void) xme_hal_mem_copy(componentManifestEntry, componentManifest, sizeof(xme_core_componentManifest_t));

        return XME_STATUS_SUCCESS;
    }
}

uint16_t
xme_core_manifestRepository_getFunctionCount
(
    const xme_core_componentManifest_t* const componentManifest
)
{
    uint16_t i = 0, totalFunctions = 0;

    XME_CHECK(NULL != componentManifest, 0);
    XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentManifest->componentType, 0);

    totalFunctions = sizeof(componentManifest->functionManifests) / sizeof(xme_core_functionManifest_t);
    for (i = 0; i < totalFunctions; i++)
    {
        if (XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT == componentManifest->functionManifests[i].functionId)
        {
            break;
        }
    }
    return i;
}

uint16_t
xme_core_manifestRepository_getPortCount
(
    const xme_core_componentManifest_t* const componentManifest
)
{
    uint16_t i = 0, totalPorts = 0;

    XME_CHECK(NULL != componentManifest, 0);
    XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentManifest->componentType, 0);

    totalPorts = sizeof(componentManifest->portManifests) / sizeof(xme_core_componentPortManifest_t);
    for (i = 0; i < totalPorts; i++)
    {
        if (XME_CORE_COMPONENT_PORTTYPE_INVALID == componentManifest->portManifests[i].portType)
        {
            break;
        }
    }
    return i;
}

xme_status_t
xme_core_manifestRepository_removeComponentManifest
(
    xme_core_componentType_t componentType
)
{
    xme_hal_table_rowHandle_t rowHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_componentManifest_t* componentManifestEntry = NULL;

    XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentType, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        manifestTable,
        xme_hal_table_rowHandle_t,
        rowHandle,
        xme_core_componentManifest_t,
        componentManifestEntry,
        componentType == componentManifestEntry->componentType
    );

    XME_CHECK(NULL != componentManifestEntry, XME_STATUS_NOT_FOUND);

    return XME_HAL_TABLE_REMOVE_ITEM(manifestTable, rowHandle);
}

xme_core_manifestRepository_iterator_t
xme_core_manifestRepository_initIterator(void)
{
    return XME_HAL_TABLE_INVALID_ROW_HANDLE;
}

bool
xme_core_manifestRepository_hasNext
(
    xme_core_manifestRepository_iterator_t iterator
)
{
    return xme_hal_table_hasNext(&manifestTable, iterator);
}

xme_core_componentManifest_t*
xme_core_manifestRepository_next
(
    xme_core_manifestRepository_iterator_t* const iterator
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_componentManifest_t* manifest = NULL;

    XME_CHECK(NULL != iterator, NULL);

    handle = *iterator;

    XME_HAL_TABLE_GET_NEXT(manifestTable, xme_hal_table_rowHandle_t, handle, xme_core_componentManifest_t, manifest, true);

    *iterator = handle;

    // manifest will be NULL in case there is no more items
    return manifest;
}

void
xme_core_manifestRepository_finiIterator
(
    xme_core_manifestRepository_iterator_t iterator
)
{
    // Nothing to do
    XME_UNUSED_PARAMETER(iterator);
}

