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
 * $Id: topicRegistry.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Topic registry.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/directory/include/topicRegistry.h"

#include "xme/hal/include/table.h"

#include <limits.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \brief Data structure for an entry in the topicSizeRegistry.
 */
typedef struct
{
    xme_core_topic_t topic; ///< Topic identifier.
    uint16_t size; ///< Maximum data size of the topic in bytes.
} topicSizeRegistryEntry_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief Table for storing topic and topic data size pairs.
 */
static XME_HAL_TABLE
(
    topicSizeRegistryEntry_t,
    topicSizeRegistry,
    (USHRT_MAX - 1) // Maximum number of possible topics (xme_core_topic_t is uint16_t, -1 because 0 is not a valid topic)
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_directory_topicRegistry_init(void)
{
    XME_HAL_TABLE_INIT(topicSizeRegistry);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_topicRegistry_getTopicSize
(
    xme_core_topic_t topic,
    uint16_t* const size
)
{
    xme_hal_table_rowHandle_t handle;
    topicSizeRegistryEntry_t* item;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    item = NULL;

    XME_CHECK(XME_CORE_TOPIC_INVALID_TOPIC != topic, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != size, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        topicSizeRegistry,
        xme_hal_table_rowHandle_t,
        handle,
        topicSizeRegistryEntry_t,
        item,
        (topic == item->topic)
    );

    XME_CHECK(NULL != item, XME_STATUS_NOT_FOUND);

    *size = item->size;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_topicRegistry_registerTopicSize
(
    xme_core_topic_t topic,
    uint16_t size,
    bool overwrite
)
{
    xme_hal_table_rowHandle_t handle;
    topicSizeRegistryEntry_t* item;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    item = NULL;

    XME_CHECK(XME_CORE_TOPIC_INVALID_TOPIC != topic, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        topicSizeRegistry,
        xme_hal_table_rowHandle_t,
        handle,
        topicSizeRegistryEntry_t,
        item,
        (topic == item->topic)
    );

    XME_CHECK(NULL == item || overwrite, XME_STATUS_ALREADY_EXIST);

    // When item is NULL then add it to the registry
    if (NULL == item)
    {
        handle = XME_HAL_TABLE_ADD_ITEM(topicSizeRegistry);
        XME_CHECK(handle != XME_HAL_TABLE_INVALID_ROW_HANDLE, XME_STATUS_OUT_OF_RESOURCES);
 
        item = XME_HAL_TABLE_ITEM_FROM_HANDLE(topicSizeRegistry, handle);
        XME_ASSERT(NULL != item);
    }

    // Item is not NULL here
    item->topic = topic;
    item->size = size;

    return XME_STATUS_SUCCESS;
}

void
xme_core_directory_topicRegistry_fini(void)
{
    XME_HAL_TABLE_FINI(topicSizeRegistry);
}
