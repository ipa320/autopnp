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
 * $Id: plugAndPlayClientConfiguration.c 7838 2014-03-14 12:38:35Z geisinger $
 */

/**
 * \file
 *         Plug and Play Client Add Config.
 *         This file contains the function calls to generate the configiration
 *         for each of the vertex/edge nodes present in the received RunTime
 *         Graph from the PnPManager. One of the main function which calls
 *         the functions of the this file is xme_core_pnp_pnpClient_processGraph
 *         This file also has functions which register the broker mappings of
 *         the ports.The configuration data structures for each of the componets
 *         or waypoints are maintained in respective tables.
 *
 */

/**
 * \addtogroup core_pnp_pnpClient
 * @{
 */
/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/plugAndPlayClientConfiguration.h"

#include "xme/core/executionManager/include/executionManagerIntern.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#ifdef WIN32
#define sscanf sscanf_s
#endif

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
/**
 * \brief Array of ports for various components.
 */
static xme_core_dataManager_dataPacketId_t ports[XME_CORE_PNP_PNPCLIENT_PORTS_MAX];

/**
 * \brief Index into the array of ports
 */
static uint16_t gPortIndex = 0;

/**
 * \brief Tracks the highest used componentID.
 */
// TODO: we are using 16 as the begining componentId but this actually should come from a central repository on the node.
//xme_core_component_t componentID = (xme_core_component_t) 31;

/**
 * \brief Table to store configuration for components.
 */
XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_CPV_config_t, xme_core_pnp_pnpClientConfiguration_CPV_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief Table to store configuration for masrhaler waypoint.
 */
XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_marshaler_config_t, xme_core_pnp_pnpClientConfiguration_marshaler_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief Table to store configuration for demarshaler waypoint.
 */
XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_demarshaler_config_t, xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for udpSend waypoint.
 */
XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_udpSend_config_t, xme_core_pnp_pnpClientConfiguration_udpSend_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for udpReceive waypoint.
 */
XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_udpRecv_config_t, xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store port mappings.
 */
XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_portMapping_t, xme_core_pnp_pnpClientConfiguration_portMapping_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for channel injector waypoint.
 */
XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_channelInjector_config_t, xme_core_pnp_pnpClientConfiguration_channelInjector_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for channel selector waypoint.
 */
XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_channelSelector_config_t, xme_core_pnp_pnpClientConfiguration_channelSelector_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

#define PNPCLIENT_CONFIG_ADD (uint8_t)1
#define PNPCLIENT_CONFIG_GET (uint8_t)2

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
// TODO (Issue #4311): Add prototype with documentation
static xme_status_t
xme_core_pnp_pnpClientConfiguration_configComponentPortVertex
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    xme_core_componentType_t componentType,
    xme_core_component_t componentId,
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    const xme_core_topic_pnpManagerRuntimeGraphModelPortData_t* const portData,
    const xme_core_topic_pnpManagerRuntimeGraphModelFunctionData_t* const functionData,
    uint8_t operation,
    xme_core_pnp_pnpClientConfiguration_CPV_config_t** outConfig
)
{
    xme_core_pnp_pnpClientConfiguration_CPV_config_t* configItem;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize;
    xme_core_component_portType_t componentPortType = (xme_core_component_portType_t)0;
    const char* initStrPos;
    size_t initStrLen;
    char* initializationString = NULL;

    XME_UNUSED_PARAMETER(portType);
    
    *outConfig = NULL;
    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;

    //vertexdata has type of vertex in it XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION
    // or XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER or XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER
    // or XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION
    // or XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER or XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER.

    // vertexData contains a string of format "%u|<...>", where "<...>" is a string that is either empty
    // (i.e., just a nul character) in case no initializationString has been specified or it is a string
    // that should be used for initialization of the new component.
    sscanf(vertexData, "%u", &componentPortType);
    // TODO: Why do we need to parse the port type here, when we already have it as a parameter?
    //       The type passed as parameter is determined from wheter its the source or target
    //       of an edge in the runtime graph model. For source it is always set to PUBLICATION,
    //       even if its a REQUEST_SENDER, this is confusing

    initStrPos = strchr(vertexData, '|');
    XME_ASSERT(NULL != initStrPos); // TODO: Check instead of ASSERT. Think of corrupted/wrong data in the runtime graph!
    initStrPos++;

    initStrLen = strlen(initStrPos);;
    if (initStrLen > 0U)
    {
        // Account for terminating nul character
        initStrLen++;
        initializationString = (char*) xme_hal_mem_alloc(initStrLen);
        XME_CHECK(NULL != initializationString, XME_STATUS_OUT_OF_RESOURCES);
        xme_hal_safeString_strncpy(&initializationString[0], initStrPos, initStrLen);
    }

    //Extract topic and size from edgeData.
    sscanf(edgeData, "%" SCNu16 "%*c%" SCNu16, &topic, &topicSize);
    //Check if it already exists
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_pnp_pnpClientConfiguration_CPV_config_table, 
        xme_hal_table_rowHandle_t,
        configItemHandle,
        xme_core_pnp_pnpClientConfiguration_CPV_config_t,
        configItem,
        (
            configItem->portType == componentPortType &&
            configItem->topic == topic &&
            configItem->componentId == componentId
        )
        // TODO: We allow multiple subscriptions/publications of the same topic on the same component!
        //       This is not taken into account here!
    );
    *outConfig = configItem;

    if (PNPCLIENT_CONFIG_ADD == operation)
    {
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            XME_CHECK_REC
            (
                (gPortIndex-1) < XME_CORE_PNP_PNPCLIENT_PORTS_MAX, 
                XME_STATUS_OUT_OF_RESOURCES, 
                {
                    XME_LOG(XME_LOG_ERROR, "[PnPClient]: Exceeded maximum number of ports supported by plug and play client. Modify build option XME_CORE_PNP_PNPCLIENT_PORTS_MAX to increase the limit.\n");
                }
            );
            configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_CPV_config_table);
            configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_CPV_config_table, configItemHandle);
            XME_CHECK
            (
                NULL != configItem,
                XME_STATUS_OUT_OF_RESOURCES
            );
            configItem->port = ports[gPortIndex++];
            configItem->portType = componentPortType;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->countfDescriptor = 0;
            configItem->componentType = componentType;
            configItem->initializationString = initializationString;
            configItem->componentId = componentId;
            configItem->componentHandle = componentHandle;
            configItem->totalToSchedule = 1;
            configItem->toDelete = false;
            configItem->scheduleEarly = 0;
            configItem->totalUsageCount = 1;
            xme_hal_mem_copy(&(configItem->portData), portData, sizeof(configItem->portData));
            xme_hal_mem_copy(&(configItem->functionData), functionData, sizeof(configItem->functionData));

            *outConfig = configItem;
        }
        else
        {
            //This is a hack because we cannot get topicSize till now using the function
            //This is needed because if we happen to add an entry in initComponentPortVertexHelperFunction
            //we dont have way to fill in the size. So we just blindly refill
            //Either it updates the missing size of overwrites the same previous value
            //Nothing to lose.
            configItem->totalUsageCount++;
            configItem->topicSize = topicSize;
        }
        return XME_STATUS_SUCCESS;
    }
    else if (PNPCLIENT_CONFIG_GET == operation)
    {
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            return XME_STATUS_NOT_FOUND;
        }
        else
        {
            return XME_STATUS_SUCCESS;
        }
    }
    else
    {
        return XME_STATUS_INVALID_PARAMETER;
    }
}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigComponentPortVertex
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    xme_core_component_t componentID,
    xme_core_componentType_t componentType,
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    const xme_core_topic_pnpManagerRuntimeGraphModelPortData_t* const portData,
    const xme_core_topic_pnpManagerRuntimeGraphModelFunctionData_t* const functionData,
    bool early
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_CPV_config_t* configItem;

    XME_UNUSED_PARAMETER(early);

    status = xme_core_pnp_pnpClientConfiguration_configComponentPortVertex
    (
        portType,
        vertexData,
        edgeData,
        componentType,
        componentID,
        componentHandle,
        portData,
        functionData,
        PNPCLIENT_CONFIG_ADD,
        &configItem
    );
    if (XME_STATUS_SUCCESS == status)
    {
        // TODO: When is this needed?
        //       This causes statically configured components to be scheduled again if they have a subscription
        /*if (XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION == portType)
        {
            configItem->totalToSchedule++;
        }*/
        if (early)
        {
            //configItem->scheduleEarly++;
            if (configItem->totalUsageCount == 1)
            {
                configItem->scheduleEarly = 1;
            }
        }
        return &configItem->port;
    }
    return NULL;
}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigComponentPortVertex
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    xme_core_componentType_t componentType,
    xme_core_component_t componentId
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_CPV_config_t* configItem;
    status = xme_core_pnp_pnpClientConfiguration_configComponentPortVertex
    (
        portType,
        vertexData,
        edgeData,
        componentType,
        componentId,
        XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE,
        NULL,
        NULL,
        PNPCLIENT_CONFIG_GET,
        &configItem
    );
    if (XME_STATUS_SUCCESS == status)
    {
        configItem->totalUsageCount--;
        if (0 == configItem->totalUsageCount)
        {
            configItem->toDelete = true;
        }
        return &configItem->port;
    }
    return NULL;
}

static xme_status_t
xme_core_pnp_pnpClientConfiguration_configMarshaler
(
    const char *vertexData,
    const char *edgeData,
    uint8_t operation,
    xme_core_pnp_pnpClientConfiguration_marshaler_config_t** config_t
)
{
    xme_core_pnp_pnpClientConfiguration_marshaler_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize;
    uint32_t inputPortQueueSize;
    xme_core_channelId_t channelID = XME_CORE_INVALID_CHANNEL_ID;

    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;

    //vertexdata has the input queue size
    sscanf(vertexData, "%" SCNu32, &inputPortQueueSize);
    //edgeData should have the topicid and size
    sscanf(edgeData, "%" SCNu16 "%*c%" SCNu16 "%*c%" SCNu32, &topic, &topicSize, &channelID);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_marshaler_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_marshaler_config_t, configItem,
                               (configItem->channelID == channelID));

    if (PNPCLIENT_CONFIG_ADD == operation)
    {
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            XME_CHECK_REC
            (
                (gPortIndex - 2) < XME_CORE_PNP_PNPCLIENT_PORTS_MAX, 
                XME_STATUS_OUT_OF_RESOURCES, 
                {
                    XME_LOG(XME_LOG_ERROR, "[PnPClient]: Exceeded maximum number of ports supported by plug and play client. Modify build option XME_CORE_PNP_PNPCLIENT_PORTS_MAX to increase the limit.\n");
                }
            );
            configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_marshaler_config_table);
            configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_marshaler_config_table, configItemHandle);
            XME_CHECK_REC
            (
                NULL != configItem,
                XME_STATUS_OUT_OF_RESOURCES,
                {
                    (*config_t) = NULL;
                }
            );
            configItem->outputPort = &ports[gPortIndex++];
            configItem->inputPort = &ports[gPortIndex++];
            configItem->inputPortQueueSize = (uint8_t)inputPortQueueSize;
            configItem->topic = topic;
            configItem->channelID = channelID;
            configItem->topicSize = topicSize;
            configItem->fDescriptor = NULL;
            configItem->totalToSchedule = 1;
            configItem->toDelete = false;
        }
        (*config_t) = configItem;
        return XME_STATUS_SUCCESS;
    }
    else if (PNPCLIENT_CONFIG_GET == operation)
    {
        (*config_t) = configItem;
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            return XME_STATUS_NOT_FOUND;
        }
        else
        {
            return XME_STATUS_SUCCESS;
        }
    }
    else
    {
        (*config_t) = configItem;
        return XME_STATUS_INVALID_PARAMETER;
    }

}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigMarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_marshaler_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configMarshaler
    (
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_ADD,
        &configItem
    );

    if (XME_STATUS_SUCCESS == status)
    {
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == portType)
        {
            return configItem->outputPort;
        }
        else
        {
            return configItem->inputPort;
        }
    }
    return NULL;
}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigMarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_marshaler_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configMarshaler
    (
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_GET,
        &configItem
    );

    if (XME_STATUS_SUCCESS == status)
    {
        configItem->toDelete = true;
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == portType)
        {
            return configItem->outputPort;
        }
        else
        {
            return configItem->inputPort;
        }
    }
    return NULL;
}

static xme_status_t
xme_core_pnp_pnpClientConfiguration_configDemarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    uint8_t operation,
    xme_core_pnp_pnpClientConfiguration_demarshaler_config_t** config_t
)
{
    xme_core_pnp_pnpClientConfiguration_demarshaler_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize;
    xme_core_channelId_t channelID = XME_CORE_INVALID_CHANNEL_ID;
    uint32_t inputPortQueueSize;

    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;

    XME_UNUSED_PARAMETER(portType);
    //vertexdata need not have anything here
    sscanf(vertexData, "%" SCNu32, &inputPortQueueSize);
    //edgeData should have the topicid and size
    sscanf(edgeData, "%" SCNu16 "%*c%" SCNu16 "%*c%" SCNu32, &topic, &topicSize, &channelID);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_demarshaler_config_t, configItem,
                               (configItem->channelID == channelID));

    if (PNPCLIENT_CONFIG_ADD == operation)
    {
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            XME_CHECK_REC
            (
                (gPortIndex - 2) < XME_CORE_PNP_PNPCLIENT_PORTS_MAX, 
                XME_STATUS_OUT_OF_RESOURCES, 
                {
                    XME_LOG(XME_LOG_ERROR, "[PnPClient]: Exceeded maximum number of ports supported by plug and play client. Modify build option XME_CORE_PNP_PNPCLIENT_PORTS_MAX to increase the limit.\n");
                }
            );
            configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table);
            configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, configItemHandle);
            XME_CHECK_REC
            (
                NULL != configItem,
                XME_STATUS_OUT_OF_RESOURCES,
                {
                    (*config_t) = NULL;
                }
            );
            configItem->outputPort = &ports[gPortIndex++];
            configItem->inputPort = &ports[gPortIndex++];
            configItem->inputPortQueueSize = (uint8_t)inputPortQueueSize;
            configItem->topic = topic;
            configItem->channelID = channelID;
            configItem->topicSize = topicSize;
            configItem->fDescriptor = NULL;
            configItem->totalToSchedule = 0;
            configItem->toDelete = false;
        }
        (*config_t) = configItem;
        return XME_STATUS_SUCCESS;
    }
    else if (PNPCLIENT_CONFIG_GET == operation)
    {
        (*config_t) = configItem;
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            return XME_STATUS_NOT_FOUND;
        }
        else
        {
            return XME_STATUS_SUCCESS;
        }
    }
    else
    {
        (*config_t) = configItem;
        return XME_STATUS_INVALID_PARAMETER;
    }
}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigDemarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_demarshaler_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configDemarshaler
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_ADD,
        &configItem
    );

    if (XME_STATUS_SUCCESS == status)
    {
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == portType)
        {
            configItem->totalToSchedule++;
            return configItem->outputPort;
        }
        else
        {
            return configItem->inputPort;
        }
    }
    return NULL;
}    

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigDemarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_demarshaler_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configDemarshaler
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_GET,
        &configItem
    );

    if (XME_STATUS_SUCCESS == status)
    {
        configItem->toDelete = true;
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == portType)
        {
            return configItem->outputPort;
        }
        else
        {
            return configItem->inputPort;
        }
    }
    return NULL;
}    

static xme_status_t
xme_core_pnp_pnpClientConfiguration_configChannelInjector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    uint8_t operation,
    xme_core_pnp_pnpClientConfiguration_channelInjector_config_t** config_t
)
{
    xme_core_pnp_pnpClientConfiguration_channelInjector_config_t *configItem = NULL;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize;
    xme_core_channelId_t srcChID = (xme_core_channelId_t)0;
    uint32_t inputPortQueueSize = 0u;

    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;

    XME_UNUSED_PARAMETER(portType);
    //vertexdata has the src and destination channel ID
    sscanf(vertexData, "%u|%u", &srcChID, &inputPortQueueSize);
    //edgeData should have the topicid and size
    sscanf(edgeData, "%" SCNu16 "%*c%" SCNu16, &topic, &topicSize);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_channelInjector_config_table, 
                              xme_hal_table_rowHandle_t, configItemHandle,
                              xme_core_pnp_pnpClientConfiguration_channelInjector_config_t, configItem,
                              ((configItem->topic == topic) && (configItem->srcChID == srcChID)));


    if (PNPCLIENT_CONFIG_ADD == operation)
    {
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            XME_CHECK_REC
            (
                (gPortIndex - 2) < XME_CORE_PNP_PNPCLIENT_PORTS_MAX, 
                XME_STATUS_OUT_OF_RESOURCES, 
                {
                    XME_LOG(XME_LOG_ERROR, "[PnPClient]: Exceeded maximum number of ports supported by plug and play client. Modify build option XME_CORE_PNP_PNPCLIENT_PORTS_MAX to increase the limit.\n");
                }
            );
            configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_channelInjector_config_table);
            configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_channelInjector_config_table, configItemHandle);
            XME_CHECK_REC
            (
                NULL != configItem,
                XME_STATUS_OUT_OF_RESOURCES,
                {
                    (*config_t) = NULL;
                }
            );
            configItem->outputPort = &ports[gPortIndex++];
            configItem->inputPort = &ports[gPortIndex++];
            configItem->inputPortQueueSize = (uint8_t)inputPortQueueSize;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->fDescriptor = NULL;
            configItem->totalToSchedule = 0;
            configItem->toDelete = false;
            configItem->srcChID = srcChID;
        }
        (*config_t) = configItem;
        return XME_STATUS_SUCCESS;
    }
    else if (PNPCLIENT_CONFIG_GET == operation)
    {
        (*config_t) = configItem;
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            return XME_STATUS_NOT_FOUND;
        }
        else
        {
            return XME_STATUS_SUCCESS;
        }
    }
    else
    {
        (*config_t) = configItem;
        return XME_STATUS_INVALID_PARAMETER;
    }
}    

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigChannelInjector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_channelInjector_config_t *configItem = NULL;

    status = xme_core_pnp_pnpClientConfiguration_configChannelInjector
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_ADD,
        &configItem
    );
    if (XME_STATUS_SUCCESS == status)
    {
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == portType)
        {
            configItem->totalToSchedule++;
            return configItem->outputPort;
        }
        else
        {
            return configItem->inputPort;
        }
    }
    return NULL;
}    

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigChannelInjector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_channelInjector_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configChannelInjector
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_GET,
        &configItem
    );
    if (XME_STATUS_SUCCESS == status)
    {
        configItem->toDelete = true;
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == portType)
        {
            return configItem->outputPort;
        }
        else
        {
            return configItem->inputPort;
        }
    }
    return NULL;
}    

static xme_status_t
xme_core_pnp_pnpClientConfiguration_configChannelSelector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    uint8_t operation,
    xme_core_pnp_pnpClientConfiguration_channelSelector_config_t** config_t
)
{
    xme_core_pnp_pnpClientConfiguration_channelSelector_config_t *configItem = NULL;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize;
    xme_core_channelId_t srcChID = (xme_core_channelId_t)0, dstChID = (xme_core_channelId_t)0;
    uint32_t inputPortQueueSize = 0u;

    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;

    XME_UNUSED_PARAMETER(portType);

    //vertexdata has the src and destination channel ID
    sscanf(vertexData, "%u|%u|%u", &srcChID, &dstChID, &inputPortQueueSize);
    //edgeData should have the topicid and size
    sscanf(edgeData, "%" SCNu16 "%*c%" SCNu16, &topic, &topicSize);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_channelSelector_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_channelSelector_config_t, configItem,
                               ((configItem->topic == topic) && (configItem->srcChID == srcChID)
                               && (configItem->dstChID == dstChID)));

    if (PNPCLIENT_CONFIG_ADD == operation)
    {
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            XME_CHECK_REC
            (
                (gPortIndex - 2) < XME_CORE_PNP_PNPCLIENT_PORTS_MAX, 
                XME_STATUS_OUT_OF_RESOURCES, 
                {
                    XME_LOG(XME_LOG_ERROR, "[PnPClient]: Exceeded maximum number of ports supported by plug and play client. Modify build option XME_CORE_PNP_PNPCLIENT_PORTS_MAX to increase the limit.\n");
                }
            );
            configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_channelSelector_config_table);
            configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_channelSelector_config_table, configItemHandle);
            XME_CHECK_REC
            (
                NULL != configItem,
                XME_STATUS_OUT_OF_RESOURCES,
                {
                    (*config_t) = NULL;
                }
            );
            configItem->outputPort = &ports[gPortIndex++];
            configItem->inputPort = &ports[gPortIndex++];
            configItem->inputPortQueueSize = (uint8_t)inputPortQueueSize;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->fDescriptor = NULL;
            configItem->totalToSchedule = 0;
            configItem->toDelete = false;
            configItem->srcChID = srcChID;
            configItem->dstChID = dstChID;
        }
        (*config_t) = configItem;
        return XME_STATUS_SUCCESS;
    }
    else if (PNPCLIENT_CONFIG_GET == operation)
    {
        (*config_t) = configItem;
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            return XME_STATUS_NOT_FOUND;
        }
        else
        {
            return XME_STATUS_SUCCESS;
        }
    }
    else
    {
        (*config_t) = configItem;
        return XME_STATUS_INVALID_PARAMETER;
    }
}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigChannelSelector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_channelSelector_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configChannelSelector
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_ADD,
        &configItem
    );

    if (XME_STATUS_SUCCESS == status)
    {
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == portType)
        {
            configItem->totalToSchedule++;
            return configItem->outputPort;
        }
        else
        {
            return configItem->inputPort;
        }
    }
    return NULL;
}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigChannelSelector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_channelSelector_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configChannelSelector
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_GET,
        &configItem
    );

    if (XME_STATUS_SUCCESS == status)
    {
        configItem->toDelete = true;
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == portType)
        {
            return configItem->outputPort;
        }
        else
        {
            return configItem->inputPort;
        }
    }
    return NULL;
}

static xme_status_t
xme_core_pnp_pnpClientConfiguration_configUDPSend
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    uint8_t operation,
    xme_core_pnp_pnpClientConfiguration_udpSend_config_t** config_t
)
{
    xme_core_pnp_pnpClientConfiguration_udpSend_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize, offset, i;

    char destIP[24]; //TODO this is IPV6 size but we need to implement the interface abstraction ASAP
    unsigned int intDestIP[4];
    unsigned int tempKey[XME_WP_UDP_HEADER_KEY_LENGTH];
    uint8_t key[XME_WP_UDP_HEADER_KEY_LENGTH];
    uint16_t port = 0;

    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;
    offset = 0;
    i = 0;

    XME_UNUSED_PARAMETER(portType);

    (void) xme_hal_mem_set(&destIP, 0x0, sizeof(destIP));
    //vertexdata has information for destination ip/port and the key
    //get the key first
    for (i = 0; i < XME_WP_UDP_HEADER_KEY_LENGTH; i++)
    {
        sscanf((char *)&(vertexData[offset]), "%u", &tempKey[i]);
        if (tempKey[i] < 10)
        {
           offset += 2; //add the increment for the pipe character too
        }
        else if (tempKey[i] < 100)
        {
           offset += 3; //add the increment for the pipe character too
        }
        else
        {
           offset += 4; //add the increment for the pipe character too
        }

        // Copy key data (required, because SCNu8 is not natively supported on all platforms
        // and may fall back to %u, for which the C standard demands the argument to be unsigned int*).
        key[i] = (uint8_t) tempKey[i];
    }
    //now get the port and destip
    sscanf((char *)&(vertexData[offset]), "%hu|%u.%u.%u.%u", &port, &intDestIP[0], &intDestIP[1], &intDestIP[2], &intDestIP[3]);
    (void) xme_hal_safeString_snprintf(destIP,24,"%d.%d.%d.%d",intDestIP[0], intDestIP[1], intDestIP[2], intDestIP[3]);
    //edgeData should have the topicid and size
    sscanf(edgeData, "%" SCNu16 "|%" SCNu16, &topic, &topicSize);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_udpSend_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_udpSend_config_t, configItem,
                               ((xme_hal_mem_compare(configItem->destIP,destIP,strlen(destIP)) == 0) 
                               && configItem->port == port 
                               && xme_hal_mem_compare(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH) == 0));

    if (PNPCLIENT_CONFIG_ADD == operation)
    {
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
        //it does not exist
            XME_CHECK_REC
            (
                (gPortIndex - 1) < XME_CORE_PNP_PNPCLIENT_PORTS_MAX, 
                XME_STATUS_OUT_OF_RESOURCES, 
                {
                    XME_LOG(XME_LOG_ERROR, "[PnPClient]: Exceeded maximum number of ports supported by plug and play client. Modify build option XME_CORE_PNP_PNPCLIENT_PORTS_MAX to increase the limit.\n");
                }
            );
            configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_udpSend_config_table);
            configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_udpSend_config_table, configItemHandle);
            XME_CHECK_REC
            (
                NULL != configItem,
                XME_STATUS_OUT_OF_RESOURCES,
                {
                    (*config_t) = NULL;
                }
            );
            configItem->inputPort = &ports[gPortIndex++];
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            (void) xme_hal_mem_copy(configItem->destIP,destIP,strlen(destIP));
            (void) xme_hal_mem_copy(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH);
            configItem->port = (uint16_t)port;
            configItem->fDescriptor = NULL;
            configItem->totalToSchedule = 1;
            configItem->toDelete = false;
        }
        (*config_t) = configItem;
        return XME_STATUS_SUCCESS;
    }
    else if (PNPCLIENT_CONFIG_GET == operation)
    {
        (*config_t) = configItem;
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            return XME_STATUS_NOT_FOUND;
        }
        else
        {
            return XME_STATUS_SUCCESS;
        }
    }
    else
    {
        (*config_t) = configItem;
        return XME_STATUS_INVALID_PARAMETER;
    }
}    

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigUDPSend
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_udpSend_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configUDPSend
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_ADD,
        &configItem
    );
    if (XME_STATUS_SUCCESS == status)
    {
        return configItem->inputPort;
    }
    return NULL;
}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigUDPSend
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_udpSend_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configUDPSend
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_GET,
        &configItem
    );
    if (XME_STATUS_SUCCESS == status)
    {
        configItem->toDelete = true;
        return configItem->inputPort;
    }
    return NULL;
}

static xme_status_t
xme_core_pnp_pnpClientConfiguration_configUDPRecv
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    uint8_t operation,
    xme_core_pnp_pnpClientConfiguration_udpRecv_config_t** config_t
)
{
    xme_core_pnp_pnpClientConfiguration_udpRecv_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize, offset, i;

    unsigned int tempKey[XME_WP_UDP_HEADER_KEY_LENGTH];
    uint8_t key[XME_WP_UDP_HEADER_KEY_LENGTH];
    uint16_t port = 0;


    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;
    offset = 0;
    i = 0;

    XME_UNUSED_PARAMETER(portType);

    //vertexdata has information for ip/port and the key 
    //get the key first
    for (i = 0; i < XME_WP_UDP_HEADER_KEY_LENGTH; i++)
    {
        sscanf((char *)&(vertexData[offset]), "%u", &tempKey[i]);
        if (tempKey[i] < 10)
        {
           offset += 2; //add the increment for the pipe character too
        }
        else if (tempKey[i] < 100)
        {
           offset += 3; //add the increment for the pipe character too
        }
        else
        {
           offset += 4; //add the increment for the pipe character too
        }

        // Copy key data (required, because SCNu8 is not natively supported on all platforms
        // and may fall back to %u, for which the C standard demands the argument to be unsigned int*).
        key[i] = (uint8_t) tempKey[i];
    }
    //now get the port and destip
    //but for we are recv we dont need the ip but just the port
    sscanf((char *)&(vertexData[offset]), "%hu", &port);
    //edgeData should have the topicid and size
    sscanf(edgeData, "%" SCNu16 "|%" SCNu16 , &topic, &topicSize);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_udpRecv_config_t, configItem,
                               (configItem->port==port && 
                               xme_hal_mem_compare(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH) == 0));

    if (PNPCLIENT_CONFIG_ADD == operation)
    {
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            XME_CHECK_REC
            (
                (gPortIndex - 1) < XME_CORE_PNP_PNPCLIENT_PORTS_MAX, 
                XME_STATUS_OUT_OF_RESOURCES, 
                {
                    XME_LOG(XME_LOG_ERROR, "[PnPClient]: Exceeded maximum number of ports supported by plug and play client. Modify build option XME_CORE_PNP_PNPCLIENT_PORTS_MAX to increase the limit.\n");
                }
            );
            configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table);
            configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, configItemHandle);
            XME_CHECK_REC
            (
                NULL != configItem,
                XME_STATUS_OUT_OF_RESOURCES,
                {
                    (*config_t) = NULL;
                }
            );
            configItem->outputPort = &ports[gPortIndex++];
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            (void) xme_hal_mem_copy(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH);
            configItem->port = (uint16_t) port;
            configItem->fDescriptor = NULL;
            configItem->totalToSchedule = 0;
            configItem->toDelete = false;
        }
        (*config_t) = configItem;
        return XME_STATUS_SUCCESS;
    }
    else if (PNPCLIENT_CONFIG_GET == operation)
    {
        (*config_t) = configItem;
        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
        {
            //it does not exist
            return XME_STATUS_NOT_FOUND;
        }
        else
        {
            return XME_STATUS_SUCCESS;
        }
    }
    else
    {
        (*config_t) = configItem;
        return XME_STATUS_INVALID_PARAMETER;
    }
}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigUDPRecv
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_udpRecv_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configUDPRecv
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_ADD,
        &configItem
    );
    if (XME_STATUS_SUCCESS == status)
    {
        configItem->totalToSchedule++;
        return configItem->outputPort;
    }
    return NULL;
}

xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigUDPRecv
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    xme_status_t status;
    xme_core_pnp_pnpClientConfiguration_udpRecv_config_t *configItem;
    status = xme_core_pnp_pnpClientConfiguration_configUDPRecv
    (
        portType,
        vertexData,
        edgeData,
        PNPCLIENT_CONFIG_GET,
        &configItem
    );
    if (XME_STATUS_SUCCESS == status)
    {
        configItem->toDelete = true;
        return configItem->outputPort;
    }
    return NULL;
}

void
xme_core_pnp_pnpClientConfiguration_createPortMappingArray
(
    xme_core_dataManager_dataPacketId_t *srcPort,
    xme_core_dataManager_dataPacketId_t *dstPort
)
{
    xme_core_pnp_pnpClientConfiguration_portMapping_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_portMapping_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_portMapping_t, configItem,
                               (configItem->srcPort == srcPort && configItem->dstPort == dstPort));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_portMapping_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_portMapping_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->srcPort = srcPort;
            configItem->dstPort = dstPort;
        }
        // TODO: Error case configItem == NULL silently ignored
    }
}

xme_status_t
xme_core_pnp_pnpClientConfiguration_addPortMappingToBroker(void)
{
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_portMapping_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_portMapping_t, configItem);
    {
        xme_status_t status = xme_core_broker_addDataPacketTransferEntry(*(configItem->srcPort), *(configItem->dstPort));
        if (XME_STATUS_SUCCESS != status && XME_STATUS_ALREADY_EXIST != status)
        {
            return XME_STATUS_INTERNAL_ERROR;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
