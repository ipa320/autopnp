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
 * $Id: channelInjector.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Waypoint that injects an attribute representing the channel identifier
 *         into a data packet.
 */

/**
 * \addtogroup wp_channel_channelInjector
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/wp/channel/include/channelInjector.h"

#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/directory/include/attribute.h"
#include "xme/core/log.h"

#include "xme/hal/include/table.h"

#include <inttypes.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_wp_channel_channelInjector_configItem_t
 *
 * \brief  Structure for storing a configuration for this waypoint.
 */
typedef struct
{
    xme_core_dataManager_dataPacketId_t inputPort; ///< Port to read data from.
    xme_core_dataManager_dataPacketId_t outputPort; ///< Port to write data to.
    xme_core_channelId_t injectedChannelID; ///< Injected channel Identifier.
    xme_core_topic_t topic; ///< Topic associated to this port.
    uint16_t sizeOfTopic; ///< Size of the topic data (attributes not considered).
    uint16_t sizeOfTopicAndAttributes; ///< Size of topic data plus size for all attributes.
    void* buffer; ///< Buffer passed for writing the data to the network.
} xme_wp_channel_channelInjector_configItem_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief xme_wp_channel_channelInjector_configItemTable
 *
 * \details Table for storing this waypoint configurations.
 */
XME_HAL_TABLE(xme_wp_channel_channelInjector_configItem_t, xme_wp_channel_channelInjector_configItemTable, XME_WP_CHANNEL_CHANNELINJECTOR_CONFIGURATIONTABLE_SIZE);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_wp_channel_channelInjector_init(void)
{
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(0U == XME_HAL_TABLE_ITEM_COUNT(xme_wp_channel_channelInjector_configItemTable)));
    XME_HAL_TABLE_INIT(xme_wp_channel_channelInjector_configItemTable);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_wp_channel_channelInjector_run
(
    xme_wp_waypoint_instanceId_t instanceId
)
{
    xme_status_t status;
    xme_wp_channel_channelInjector_configItem_t* configurationItem;
    uint32_t bytesRead = 0U;
    
    configurationItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_wp_channel_channelInjector_configItemTable, (xme_hal_table_rowHandle_t) instanceId);

    XME_CHECK(NULL != configurationItem, XME_STATUS_INVALID_HANDLE);

    // Indicate we are going to start the read operation.
    status = xme_core_dataHandler_startReadOperation(configurationItem->inputPort);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    // read from the dataId into the buffer
    status = xme_core_dataHandler_readData
    (
        configurationItem->inputPort,
        configurationItem->buffer,
        configurationItem->sizeOfTopic,
        &bytesRead
    );

    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    XME_CHECK
    (
        bytesRead == configurationItem->sizeOfTopic,
        XME_STATUS_INTERNAL_ERROR
    );

    // Indicate we are going to start the write operation.
    status = xme_core_dataHandler_startWriteOperation(configurationItem->outputPort);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    
    // Write the output data
    status = xme_core_dataHandler_writeData
    (
        configurationItem->outputPort,
        configurationItem->buffer,
        configurationItem->sizeOfTopic
    );

    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    // Read all attributes and copy them to the output port
    {
        xme_core_attribute_descriptor_list_t attributeDescriptorList;
        uint8_t i;
        uint8_t* payloadPtr = ((uint8_t*)configurationItem->buffer) + configurationItem->sizeOfTopic;

        status = xme_core_directory_attribute_getAttributeDescriptorList(configurationItem->topic, &attributeDescriptorList);
        
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR,
            XME_LOG_WARNING, "[CIW] No attribute descriptor list found\n"
        );

        // If there is no attribute descriptor list for the given topic, then simply do nothing
        if (XME_STATUS_SUCCESS == status)
        {
            for (i = 0U; i < attributeDescriptorList.length; i++)
            {
                if (XME_CORE_ATTRIBUTE_KEY_CHANNELID == attributeDescriptorList.element[i].key)
                {
                    status = xme_core_dataHandler_writeAttribute
                    (
                        configurationItem->outputPort,
                        attributeDescriptorList.element[i].key,
                        (void*) &(configurationItem->injectedChannelID),
                        attributeDescriptorList.element[i].size
                    );
                }
                else
                {
                    status = xme_core_dataHandler_readAttribute
                    (
                        configurationItem->inputPort,
                        attributeDescriptorList.element[i].key,
                        payloadPtr,
                        attributeDescriptorList.element[i].size,
                        &bytesRead
                    );

                    if (status != XME_STATUS_SUCCESS &&
                        status != XME_STATUS_NO_SUCH_VALUE &&
                        attributeDescriptorList.element[i].size != bytesRead)
                    {
                        XME_LOG
                        (
                            XME_LOG_WARNING, "Reading of attribute '%" PRIu32 "' failed with status %" PRIu32 ".\n",
                            (uint32_t) attributeDescriptorList.element[i].key, (uint32_t) status
                        );
                    }
                    else
                    {
                        status = xme_core_dataHandler_writeAttribute
                        (
                            configurationItem->outputPort,
                            attributeDescriptorList.element[i].key,
                            payloadPtr,
                            attributeDescriptorList.element[i].size
                        );
                    }
                }
                
                payloadPtr += attributeDescriptorList.element[i].size;
                
                XME_ASSERT(status == XME_STATUS_SUCCESS);

                if (status != XME_STATUS_SUCCESS)
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING, "Writing of attribute '%" PRIu32 "' failed with status %" PRIu32 ".\n",
                        (uint32_t) attributeDescriptorList.element[i].key, (uint32_t) status
                    );
                }
            }
        }
    }

    status = xme_core_dataHandler_completeReadOperation(configurationItem->inputPort);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    status = xme_core_dataHandler_completeWriteOperation(configurationItem->outputPort);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_wp_channel_channelInjector_addConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_dataManager_dataPacketId_t inputPort,
    xme_core_dataManager_dataPacketId_t outputPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    void* buffer,
    uint16_t sizeOfBuffer,
    xme_core_channelId_t injectedChannelID
)
{
    xme_hal_table_rowHandle_t configurationItemHandle;
    xme_wp_channel_channelInjector_configItem_t* configurationItem;
    uint32_t sizeOfTopicAndAttributes;
    uint8_t i;
    xme_core_attribute_descriptor_list_t attributeDescriptorList;
    xme_status_t status;

    status = xme_core_directory_attribute_getAttributeDescriptorList(topic, &attributeDescriptorList);

    sizeOfTopicAndAttributes = sizeOfTopic;

    // If there is no attribute descriptor list for the given topic, then simply do nothing
    if (XME_STATUS_SUCCESS == status)
    {
        for (i = 0U; i < attributeDescriptorList.length; i++)
        {
            sizeOfTopicAndAttributes += attributeDescriptorList.element[i].size;
        }
    }

    XME_CHECK
    (
        sizeOfBuffer >= (sizeOfTopicAndAttributes), 
        XME_STATUS_INVALID_PARAMETER
    );

    {
        xme_hal_table_rowHandle_t exists = XME_HAL_TABLE_INVALID_ROW_HANDLE;
        xme_wp_channel_channelInjector_configItem_t* existsItem = NULL;
        XME_HAL_TABLE_GET_NEXT
        (
            xme_wp_channel_channelInjector_configItemTable,
            xme_hal_table_rowHandle_t, exists,
            xme_wp_channel_channelInjector_configItem_t, existsItem,
            (existsItem->topic==topic && existsItem->injectedChannelID == injectedChannelID)
        );
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE != exists)
        {
            // So his already exists, we don't have to add another entry we can return the same instance ID
            *instanceId = (xme_wp_waypoint_instanceId_t) exists;
            return XME_STATUS_SUCCESS;
        }
    }

    configurationItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_wp_channel_channelInjector_configItemTable);

    XME_CHECK
    (
        XME_HAL_TABLE_INVALID_ROW_HANDLE != configurationItemHandle, 
        XME_STATUS_OUT_OF_RESOURCES
    );
    *instanceId = (xme_wp_waypoint_instanceId_t) configurationItemHandle;

    configurationItem = XME_HAL_TABLE_ITEM_FROM_HANDLE
    (
        xme_wp_channel_channelInjector_configItemTable,
        configurationItemHandle
    );
    XME_ASSERT(NULL != configurationItem);

    configurationItem->inputPort = inputPort;
    configurationItem->outputPort = outputPort;
    configurationItem->sizeOfTopic = sizeOfTopic;
    configurationItem->sizeOfTopicAndAttributes = sizeOfTopicAndAttributes;
    configurationItem->topic = topic;
    configurationItem->buffer = buffer;
    configurationItem->injectedChannelID = injectedChannelID;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_wp_channel_channelInjector_removeConfig
(
    xme_wp_waypoint_instanceId_t instanceId
)
{
    xme_status_t status;

    XME_CHECK(XME_WP_WAYPOINT_INSTANCEID_INVALID != instanceId, XME_STATUS_INVALID_HANDLE);

    status = XME_HAL_TABLE_REMOVE_ITEM(xme_wp_channel_channelInjector_configItemTable, (xme_hal_table_rowHandle_t)(instanceId));
    XME_ASSERT(XME_STATUS_SUCCESS == status || XME_STATUS_INVALID_HANDLE == status);

    return status;
}

xme_status_t
xme_wp_channel_channelInjector_fini(void)
{
    XME_HAL_TABLE_FINI(xme_wp_channel_channelInjector_configItemTable);

    return XME_STATUS_SUCCESS;
}

/** @} */
