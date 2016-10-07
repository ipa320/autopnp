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
 * $Id: channelSelector.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *
 * \brief Waypoint that selects one of a set of output ports to copy the
 *        received data to depending on the channel identifier.
 *
 */

/**
 * \addtogroup wp_channel_channelSelector
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/wp/channel/include/channelSelector.h"

#include "xme/core/log.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/directory/include/attribute.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/table.h"

#include <inttypes.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_wp_channel_channelSelector_inputPortsConfigItem_t
 *
 * \brief  Structure for storing Input Port configuration for this waypoint.
 * \note 1. The architecture followed using these two structs is m:n
 *       2. For one input port there can be many srcChannelID->dstChannelID mappings
 *       3. Mathematically/logically we can have many inputPorts using the same 
 *          srcChannelID->dstChannelID mapping. But practically I am not sure. So to be
 *          safe I have added a count on the channelIDMappingConfigItem struct as well.
 *       4. count ist uint8_t which means 255 configurations are allowed.
 */
typedef struct
{
    xme_core_dataManager_dataPacketId_t inputPort; ///< Port to read the data from.
    uint16_t sizeOfTopicAndAttributes; ///< Size of topic data plus size for all attributes.
    uint16_t sizeOfTopic; ///< Size of the topic.
    xme_core_topic_t topic; ///< Topic associated to this port.
    void *buffer; ///< Buffer passed for reading/writing the data.
    uint8_t count; ///< Count to keep the number of times this entry is used with different source ChannelID/destination Channel ID pairs.
} xme_wp_channel_channelSelector_inputPortsConfigItem_t;

/**
 * \struct xme_wp_channel_channelSelector_channelIDMappingConfigItem_t
 *
 * \brief  Structure for storing a Channel ID  mapping configuration for this waypoint.
 * \note 1. The architecture followed using these two structs is m:n
 *       2. For one input port there can be many srcChannelID->dstChannelID mappings
 *       3. Mathematically/logically we can have many inputPorts using the same 
 *          srcChannelID->dstChannelID mapping. But practically I am not sure. So to be
 *          safe I have added a count on the channelIDMappingConfigItem struct as well.
 *       4. count ist uint8_t which means 255 configurations are allowed.
 */
typedef struct
{
    xme_core_channelId_t sourceChannelID; ///< Source channel identifier.
    xme_core_channelId_t destinationChannelID; ///< Destination Channel identifier.
    xme_core_dataManager_dataPacketId_t outputPort; ///< Port to write the data to for the destination channel identifier.
    uint8_t count; ///< Count to keep the number of times this entry is used with different Instance IDs
} xme_wp_channel_channelSelector_channelIDMappingConfigItem_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief xme_wp_channel_channelSelector_inputPortsConfigItemTable
 *
 * \details Table for storing Input Ports for CSW configurations.
 */
XME_HAL_TABLE(xme_wp_channel_channelSelector_inputPortsConfigItem_t, xme_wp_channel_channelSelector_inputPortsConfigItemTable, XME_WP_CHANNEL_CHANNELSELECTOR_INPUTPORTS_CONFIGURATIONTABLE_SIZE);

/**
 * \brief xme_wp_channel_channelSelector_channelIDMappingConfigItemTable
 *
 * \details Table to store the channel mappings of CSW configurations.
 */
XME_HAL_TABLE(xme_wp_channel_channelSelector_channelIDMappingConfigItem_t, xme_wp_channel_channelSelector_channelIDMappingConfigItemTable, XME_WP_CHANNEL_CHANNELSELECTOR_CHANNELMAPPINGS_CONFIGURATIONTABLE_SIZE);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_wp_channel_channelSelector_init(void)
{
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(0U == XME_HAL_TABLE_ITEM_COUNT(xme_wp_channel_channelSelector_inputPortsConfigItemTable)));
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(0U == XME_HAL_TABLE_ITEM_COUNT(xme_wp_channel_channelSelector_channelIDMappingConfigItemTable)));

    XME_HAL_TABLE_INIT(xme_wp_channel_channelSelector_inputPortsConfigItemTable);
    XME_HAL_TABLE_INIT(xme_wp_channel_channelSelector_channelIDMappingConfigItemTable);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_wp_channel_channelSelector_run
(
    xme_wp_waypoint_instanceId_t instanceId
)
{
    xme_status_t status;
    xme_wp_channel_channelSelector_inputPortsConfigItem_t* configurationItem = NULL;
    xme_wp_channel_channelSelector_channelIDMappingConfigItem_t* channelIDconfigItem = NULL;
    unsigned int bytesRead = 0U;

    configurationItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_wp_channel_channelSelector_inputPortsConfigItemTable, (xme_hal_table_rowHandle_t) instanceId);
    
    XME_CHECK
    (
        NULL != configurationItem,
        XME_STATUS_INVALID_HANDLE
    );

    // Indicate we are going to start the read operation.
    status = xme_core_dataHandler_startReadOperation(configurationItem->inputPort);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    // Read from the inputPort
    status = xme_core_dataHandler_readData
    (
        configurationItem->inputPort,
        configurationItem->buffer,
        configurationItem->sizeOfTopic,
        &bytesRead
    );
    XME_CHECK(bytesRead > 0U, XME_STATUS_INTERNAL_ERROR);

    //Get the attributes from the Port
    {
        xme_core_attribute_descriptor_list_t attributeDescriptorList;
        uint8_t i;
        uint8_t* payloadPtr = ((uint8_t*) configurationItem->buffer) + configurationItem->sizeOfTopic;

        status = xme_core_directory_attribute_getAttributeDescriptorList(configurationItem->topic, &attributeDescriptorList);

        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR,
            XME_LOG_WARNING, "[CSW] No attribute descriptor list found\n"
        );

        for (i = 0U; i < attributeDescriptorList.length; i++)
        {
            status = xme_core_dataHandler_readAttribute
            (
                configurationItem->inputPort,
                attributeDescriptorList.element[i].key,
                payloadPtr,
                attributeDescriptorList.element[i].size,
                &bytesRead
            );

            XME_ASSERT(XME_CORE_ATTRIBUTE_KEY_CHANNELID != attributeDescriptorList.element[i].key || XME_STATUS_NO_SUCH_VALUE != status);

            if ((XME_STATUS_NO_SUCH_VALUE != status) && (XME_STATUS_SUCCESS != status || attributeDescriptorList.element[i].size != bytesRead))
            {
                XME_LOG
                (
                    XME_LOG_WARNING, "Reading of attribute '%" PRIu32 "' failed with status %" PRIu32 ".",
                    (uint32_t) attributeDescriptorList.element[i].key, (uint32_t) status
                );
            }

            if (XME_CORE_ATTRIBUTE_KEY_CHANNELID == attributeDescriptorList.element[i].key)
            {
                xme_hal_table_rowHandle_t handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
                xme_core_channelId_t srcTemp = *(xme_core_channelId_t*) (payloadPtr);
                XME_HAL_TABLE_GET_NEXT
                (
                    xme_wp_channel_channelSelector_channelIDMappingConfigItemTable, 
                    xme_hal_table_rowHandle_t, handle,
                    xme_wp_channel_channelSelector_channelIDMappingConfigItem_t, channelIDconfigItem,
                    (srcTemp == channelIDconfigItem->sourceChannelID)
                );
                XME_CHECK_MSG
                (
                    NULL != channelIDconfigItem, XME_STATUS_INTERNAL_ERROR,
                    XME_LOG_WARNING, 
                    "[CSW] No channel mapping found for channel %d. A possible "
                    "cause might be that a wrong requestDataHandle was passed "
                    "in the writeResponseHandlerPort() call from the component "
                    "wrapper.\n", srcTemp
                );

                // Replace the input channel id with the one of the output
                (void) xme_hal_mem_copy(payloadPtr, (void*) &(channelIDconfigItem->destinationChannelID), attributeDescriptorList.element[i].size);
            }
            payloadPtr += attributeDescriptorList.element[i].size;


        }
    }

    // Indicate we are going to start the write operation.
    status = xme_core_dataHandler_startWriteOperation(channelIDconfigItem->outputPort);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    // Copy it out to the outputPort of the found configuration
    status = xme_core_dataHandler_writeData
    (
        channelIDconfigItem->outputPort,
        configurationItem->buffer,
        configurationItem->sizeOfTopic
    );
    XME_CHECK
    (
        XME_STATUS_SUCCESS == status,
        XME_STATUS_INTERNAL_ERROR
    );

    // Get all attributes from payload and write them to the output port
    {
        xme_core_attribute_descriptor_list_t attributeDescriptorList;
        uint8_t i;
        uint8_t* payloadPtr = ((uint8_t*) configurationItem->buffer) + configurationItem->sizeOfTopic;

        status = xme_core_directory_attribute_getAttributeDescriptorList(configurationItem->topic, &attributeDescriptorList);

        // If there is no attribute descriptor list for the given topic, then simply do nothing
        if (XME_STATUS_SUCCESS == status)
        {
            for (i = 0U; i < attributeDescriptorList.length; i++)
            {
                status = xme_core_dataHandler_writeAttribute
                (
                    channelIDconfigItem->outputPort,
                    attributeDescriptorList.element[i].key,
                    payloadPtr,
                    attributeDescriptorList.element[i].size
                );

                payloadPtr += attributeDescriptorList.element[i].size;

                XME_ASSERT(status == XME_STATUS_SUCCESS);

                if (status != XME_STATUS_SUCCESS)
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING, "Writing of attribute '%" PRIu32 "' failed with status %" PRIu32 ".",
                        (uint32_t) attributeDescriptorList.element[i].key, (uint32_t) status
                    );
                }
            }
        }
    }

    status = xme_core_dataHandler_completeReadOperation(configurationItem->inputPort);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    status = xme_core_dataHandler_completeWriteOperation(channelIDconfigItem->outputPort);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    return XME_STATUS_SUCCESS;
}

bool
xme_wp_channel_channelSelector_hasConfig
(
    xme_core_topic_t topic,
    xme_core_dataManager_dataPacketId_t* inputPort
)
{
    xme_wp_channel_channelSelector_inputPortsConfigItem_t* inputPortConfigItem = NULL;
    xme_hal_table_rowHandle_t inputPortConfigItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    XME_CHECK(XME_CORE_TOPIC_INVALID_TOPIC != topic, false);
    XME_HAL_TABLE_GET_NEXT
    (
        xme_wp_channel_channelSelector_inputPortsConfigItemTable,
        xme_hal_table_rowHandle_t, inputPortConfigItemHandle,
        xme_wp_channel_channelSelector_inputPortsConfigItem_t, inputPortConfigItem,
        inputPortConfigItem->topic == topic
    );
    *inputPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != inputPortConfigItemHandle && NULL != inputPortConfigItem, false);
    *inputPort = inputPortConfigItem->inputPort;
    return true;
}

xme_status_t
xme_wp_channel_channelSelector_addConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_dataManager_dataPacketId_t inputPort,
    xme_core_dataManager_dataPacketId_t outputPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    void* buffer,
    uint16_t sizeOfBuffer,
    xme_core_channelId_t sourceChannelID,
    xme_core_channelId_t destinationChannelID
)
{
    uint32_t sizeOfTopicAndAttributes;
    uint8_t i;
    xme_core_attribute_descriptor_list_t attributeDescriptorList;
    xme_status_t status;
    xme_wp_channel_channelSelector_inputPortsConfigItem_t* inputPortConfigItem = NULL;
    xme_hal_table_rowHandle_t inputPortConfigItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    XME_CHECK
    (
        XME_CORE_TOPIC_INVALID_TOPIC != topic &&
        XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != outputPort &&
        XME_CORE_INVALID_CHANNEL_ID != sourceChannelID && 
        XME_CORE_INVALID_CHANNEL_ID != destinationChannelID,
        XME_STATUS_INVALID_PARAMETER
    );

    // Check if the inputPort configuration already exists
    {
        XME_HAL_TABLE_GET_NEXT
        (
            xme_wp_channel_channelSelector_inputPortsConfigItemTable,
            xme_hal_table_rowHandle_t, inputPortConfigItemHandle,
            xme_wp_channel_channelSelector_inputPortsConfigItem_t, inputPortConfigItem,
            inputPortConfigItem->topic == topic
        );
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == inputPortConfigItemHandle)
        {   //Add new entry

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

            XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != inputPort, XME_STATUS_INVALID_PARAMETER);
            
            inputPortConfigItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_wp_channel_channelSelector_inputPortsConfigItemTable);
            XME_CHECK
            (
                XME_HAL_TABLE_INVALID_ROW_HANDLE != inputPortConfigItemHandle, 
                XME_STATUS_OUT_OF_RESOURCES
            );
            
            inputPortConfigItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_wp_channel_channelSelector_inputPortsConfigItemTable, inputPortConfigItemHandle);
            XME_ASSERT(NULL != inputPortConfigItem);
            
            inputPortConfigItem->inputPort = inputPort;
            inputPortConfigItem->sizeOfTopicAndAttributes = sizeOfTopicAndAttributes;
            inputPortConfigItem->sizeOfTopic = sizeOfTopic;
            inputPortConfigItem->topic = topic;
            inputPortConfigItem->buffer = buffer;
            inputPortConfigItem->count = 0;
        }
        XME_CHECK(255u > inputPortConfigItem->count, XME_STATUS_OUT_OF_RESOURCES);
        *instanceId = (xme_wp_waypoint_instanceId_t) inputPortConfigItemHandle;
        inputPortConfigItem->count++;
    }
    
    // Check if the source channel ID -> destination channel ID mapping already exists
    {
        xme_hal_table_rowHandle_t mappingHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
        xme_wp_channel_channelSelector_channelIDMappingConfigItem_t* mappingConfigItem = NULL;
        XME_HAL_TABLE_GET_NEXT
        (
            xme_wp_channel_channelSelector_channelIDMappingConfigItemTable,
            xme_hal_table_rowHandle_t, mappingHandle,
            xme_wp_channel_channelSelector_channelIDMappingConfigItem_t, mappingConfigItem,
            (mappingConfigItem->sourceChannelID == sourceChannelID && mappingConfigItem->destinationChannelID == destinationChannelID)
        );
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == mappingHandle)
        {
            // Add a new entry
            mappingHandle = XME_HAL_TABLE_ADD_ITEM(xme_wp_channel_channelSelector_channelIDMappingConfigItemTable);
            XME_CHECK
            (
                XME_HAL_TABLE_INVALID_ROW_HANDLE != mappingHandle, 
                XME_STATUS_OUT_OF_RESOURCES
            );
            
            mappingConfigItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_wp_channel_channelSelector_channelIDMappingConfigItemTable, mappingHandle);
            XME_ASSERT(NULL != mappingConfigItem);
            
            mappingConfigItem->sourceChannelID = sourceChannelID;
            mappingConfigItem->destinationChannelID = destinationChannelID;
            mappingConfigItem->outputPort = outputPort;
            mappingConfigItem->count = 0;
        }
        XME_CHECK_REC
        (
            255u > mappingConfigItem->count,
            XME_STATUS_OUT_OF_RESOURCES,
            {
                if (1u == inputPortConfigItem->count)
                {
                    //We had just created an entry, need to delete it. This should always be successful
                    status = XME_HAL_TABLE_REMOVE_ITEM(xme_wp_channel_channelSelector_inputPortsConfigItemTable, (xme_hal_table_rowHandle_t)(instanceId));
                    XME_ASSERT(XME_STATUS_SUCCESS == status);
                }
                else
                {
                    inputPortConfigItem->count--;
                }
            }
        );
        mappingConfigItem->count++;
    }
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_wp_channel_channelSelector_removeConfig
(
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_channelId_t sourceChannelID,
    xme_core_channelId_t destinationChannelID
)
{
    xme_status_t status;
    xme_wp_channel_channelSelector_inputPortsConfigItem_t* inputPortConfigItem = NULL;
    xme_hal_table_rowHandle_t mappingHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_wp_channel_channelSelector_channelIDMappingConfigItem_t* mappingConfigItem = NULL;

    XME_CHECK(XME_WP_WAYPOINT_INSTANCEID_INVALID != instanceId && XME_CORE_INVALID_CHANNEL_ID != sourceChannelID && XME_CORE_INVALID_CHANNEL_ID != destinationChannelID, XME_STATUS_INVALID_PARAMETER);

    //Get the inputPortConfigItem from the table
    inputPortConfigItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_wp_channel_channelSelector_inputPortsConfigItemTable, (xme_hal_table_rowHandle_t)(instanceId));

    //Get the source_channelId-destination_channelID mapping
    XME_HAL_TABLE_GET_NEXT
    (
        xme_wp_channel_channelSelector_channelIDMappingConfigItemTable,
        xme_hal_table_rowHandle_t, mappingHandle,
        xme_wp_channel_channelSelector_channelIDMappingConfigItem_t, mappingConfigItem,
        (mappingConfigItem->sourceChannelID == sourceChannelID && mappingConfigItem->destinationChannelID == destinationChannelID)
    );
    XME_CHECK(NULL != inputPortConfigItem && NULL != mappingConfigItem, XME_STATUS_INVALID_CONFIGURATION);

    XME_ASSERT(0 < inputPortConfigItem->count && 0 < mappingConfigItem->count);

    inputPortConfigItem->count--;
    mappingConfigItem->count--;
    if (0 == inputPortConfigItem->count)
    {
        status = XME_HAL_TABLE_REMOVE_ITEM(xme_wp_channel_channelSelector_inputPortsConfigItemTable, (xme_hal_table_rowHandle_t)(instanceId));
        XME_ASSERT(XME_STATUS_SUCCESS == status);
    }
    if (0 == mappingConfigItem->count)
    {
        status = XME_HAL_TABLE_REMOVE_ITEM(xme_wp_channel_channelSelector_channelIDMappingConfigItemTable, mappingHandle);
        XME_ASSERT(XME_STATUS_SUCCESS == status);
    }

    status = XME_STATUS_SUCCESS;

    return status;
}

xme_status_t
xme_wp_channel_channelSelector_fini(void)
{
    XME_HAL_TABLE_FINI(xme_wp_channel_channelSelector_inputPortsConfigItemTable);
    XME_HAL_TABLE_FINI(xme_wp_channel_channelSelector_channelIDMappingConfigItemTable);

    return XME_STATUS_SUCCESS;
}

/** @} */
