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
 * $Id: demarshaler.c 6684 2014-02-07 14:34:26Z geisinger $
 */

/**
 * \file
 *
 * \brief  Waypoint that demarshals topic data.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/wp/marshal/include/demarshaler.h"
#include "xme/wp/marshal/include/demarshalerInternalTypes.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/net.h"
#include "xme/hal/include/table.h"

#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/topic.h"
#include "xme/core/topicData.h"

#include "xme/wp/waypoint.h"

#include <inttypes.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_wp_marshal_demarshaler_init(void)
{
    XME_HAL_TABLE_INIT(xme_wp_marshal_demarshaler_configurationTable);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_wp_marshal_demarshaler_getConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t* topic,
    xme_core_channelId_t* channelID,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort
)
{
    xme_hal_table_rowHandle_t rowHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_wp_marshal_demarshaler_configurationItem_t* item = NULL;
    
    // Check parameters in debug build only (doc specifies they must not be null)
    XME_ASSERT(NULL != instanceId);
    XME_ASSERT(NULL != topic);
    XME_ASSERT(NULL != channelID);
    XME_ASSERT(NULL != inputPort);
    XME_ASSERT(NULL != outputPort);
    
    XME_HAL_TABLE_GET_NEXT
    (
        xme_wp_marshal_demarshaler_configurationTable,
        xme_hal_table_rowHandle_t,
        rowHandle,
        xme_wp_marshal_demarshaler_configurationItem_t,
        item,
        (
            (XME_WP_WAYPOINT_INSTANCEID_INVALID == *instanceId  || rowHandle == (xme_hal_table_rowHandle_t)(*instanceId))
            && (XME_CORE_TOPIC_INVALID_TOPIC == *topic  || item->topic == *topic)
            && (XME_CORE_INVALID_CHANNEL_ID == *channelID  || item->channelID == *channelID)
            && (XME_CORE_DATAMANAGER_DATAPACKETID_INVALID == *inputPort || item->inputPort == *inputPort)
            && (XME_CORE_DATAMANAGER_DATAPACKETID_INVALID == *outputPort || item->outputPort == *outputPort)
        )
    );
    
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE != rowHandle)
    {
        *instanceId = (xme_wp_waypoint_instanceId_t)rowHandle;
        *topic = item->topic;
        *channelID = item->channelID;
        *inputPort = item->inputPort;
        *outputPort = item->outputPort;
        
        return XME_STATUS_SUCCESS;
    }
    
    return XME_STATUS_NOT_FOUND;
}

xme_status_t
xme_wp_marshal_demarshaler_removeConfig
(
    xme_wp_waypoint_instanceId_t instanceId
)
{
    xme_status_t status;

    XME_CHECK(XME_WP_WAYPOINT_INSTANCEID_INVALID != instanceId, XME_STATUS_INVALID_HANDLE);

    status = XME_HAL_TABLE_REMOVE_ITEM(xme_wp_marshal_demarshaler_configurationTable, (xme_hal_table_rowHandle_t)(instanceId));
    XME_ASSERT(XME_STATUS_SUCCESS == status || XME_STATUS_INVALID_HANDLE == status);

    return status;
}

xme_status_t
xme_wp_marshal_demarshaler_addConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t topic,
    xme_core_channelId_t channelID,
    xme_core_dataManager_dataPacketId_t inputPort,
    xme_core_dataManager_dataPacketId_t outputPort
)
{
    xme_hal_table_rowHandle_t configurationItemHandle;
    xme_wp_marshal_demarshaler_configurationItem_t* configurationItem;

    // TODO: Commented out until issue #3315 is resolved
//    XME_CHECK
//    (
//        xme_wp_marshal_demarshaler_isSupported(topic),
//        XME_STATUS_INVALID_PARAMETER
//    );

    configurationItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_wp_marshal_demarshaler_configurationTable);

    XME_CHECK
    (
        XME_HAL_TABLE_INVALID_ROW_HANDLE != configurationItemHandle,
        XME_STATUS_OUT_OF_RESOURCES
    );

    configurationItem = XME_HAL_TABLE_ITEM_FROM_HANDLE
    (
        xme_wp_marshal_demarshaler_configurationTable, 
        configurationItemHandle
    );
    
    XME_ASSERT(NULL != configurationItem);

    configurationItem->topic = topic;
    configurationItem->channelID = channelID;
    configurationItem->inputPort = inputPort;
    configurationItem->outputPort = outputPort;

    // We use the row handle to identify this configuration
    *instanceId = (xme_wp_waypoint_instanceId_t)configurationItemHandle;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_wp_marshal_demarshaler_fini(void)
{
    XME_HAL_TABLE_FINI(xme_wp_marshal_demarshaler_configurationTable);

    return XME_STATUS_SUCCESS;
}
