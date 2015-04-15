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
 * $Id: mockLayerForLoginClient.c 7694 2014-03-06 15:56:24Z wiesmueller $
 */

/**
 * \file
 *         Mock layer for login client tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/manifestTypes.h"
#include "xme/hal/include/mem.h"
#include "xme/core/coreTypes.h"
#include "xme/wp/waypoint.h"
#include <stdbool.h>

XME_EXTERN_C_BEGIN
/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
xme_status_t
xme_core_dataHandler_createPort
(
    xme_core_component_t componentID,
    xme_core_component_portType_t type,
    xme_core_topic_t topic,
    int bufferSize,
    xme_core_attribute_descriptor_list_t metadata,
    int queueSize,
    bool overwrite,
    bool persistent,
    int historyDepth,
    xme_core_dataManager_dataPacketId_t * const dataPacketId
);

xme_status_t
xme_core_dataHandler_writeData
(
        xme_core_dataManager_dataPacketId_t port,
        void const * const buffer,
        unsigned int bufferSize
);

xme_status_t
xme_core_dataHandler_completeWriteOperation
(
        xme_core_dataManager_dataPacketId_t port
);

xme_status_t
xme_core_dataHandler_readData
(
        xme_core_dataManager_dataPacketId_t port,
        void * const buffer,
        unsigned int bufferSize,
        unsigned int * const bytesRead
);

xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t port
);

xme_status_t
xme_core_broker_addDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
);

xme_status_t
xme_core_exec_configurator_rollback
(
    xme_core_exec_transactionId_t transactionId
);

xme_status_t
xme_core_exec_configurator_commit
(
    xme_core_exec_transactionId_t transactionId
);

xme_status_t
xme_core_exec_dispatcher_createFunctionExecutionUnit
(
    xme_core_exec_functionDescriptor_t* functionWrapper,
    bool eventTriggeredBehavior
);

xme_status_t
xme_core_exec_configurator_addComponentToSchedule
(
    xme_core_exec_transactionId_t transactionId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArgs,
    xme_hal_time_timeInterval_t afterTime_ns,
    xme_hal_time_timeInterval_t beforeTime_ns,
    uint32_t period,
    uint32_t periodOffset,
    xme_hal_linkedList_descriptor_t* schedules
);

void
xme_core_exec_configurator_addComponentToScheduleSetStatus
(
    xme_status_t status
);

xme_status_t
xme_core_exec_componentRepository_registerComponent
(
    xme_core_exec_componentDescriptor_t* componentDescriptor
);

xme_status_t
xme_wp_marshal_marshaler_getConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t* topic,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort
);

void
xme_wp_marshal_marshaler_getConfigSetStatus
(
    xme_status_t status
);

xme_status_t
xme_wp_udp_udpSend_getConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_dataManager_dataPacketId_t* dataId,
    void *key,
    const char* hostname,
    uint16_t port
);

void
xme_wp_udp_udpSend_getConfigSetStatus
(
    xme_status_t status
);

xme_status_t
createUdpSendWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);
xme_status_t
udpSendWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* dataPort,
    uint8_t* key,
    const char* destIP,
    uint32_t ipPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** sendBuffer,
    bool isBroadcast
);
void
udpSendWaypointAddConfigSetStatus
(
    xme_status_t status
);
xme_status_t
createMarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);
xme_status_t
marshalerWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputDataPort,
    uint8_t inputPortQueueSize,
    xme_core_dataManager_dataPacketId_t* outputDataPort,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_core_channelId_t channelID
);
xme_status_t
createDemarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);
xme_status_t
demarshalerWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputDataPort,
    uint8_t inputPortQueueSize,
    xme_core_dataManager_dataPacketId_t* outputDataPort,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_core_channelId_t channelID
);
xme_status_t
createUdpReceiveWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);
xme_status_t
udpReceiveWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* dataPort,
    uint8_t* key,
    uint32_t ipPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** recvBuffer
);
xme_status_t
xme_wp_wci_addConfig
(
    uint32_t ipAddress,
    uint16_t portAddress,
    xme_core_channelId_t channelID,
    xme_core_topic_t topic
);

xme_status_t
xme_core_pnp_pnpClient_instantiateComponentOnThisNode
(
    xme_core_componentType_t component
);
/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static char portArray[5][100] = { {'\0'}, {'\0'}, {'\0'}, {'\0'}, {'\0'} };

xme_status_t statusXmeWpMarshalMarshalerGetConfig = XME_STATUS_SUCCESS;
xme_status_t status_xme_wp_wci_addConfig = XME_STATUS_SUCCESS;
xme_status_t statusDemarshalerWaypointAddConfig;// = XME_STATUS_SUCCESS;
xme_status_t statusUdpSendWaypointAddConfig = XME_STATUS_SUCCESS;
xme_status_t statusXmeCoreExecConfiguratorAddComponentToSchedule = XME_STATUS_SUCCESS;
xme_status_t statusXmeWpUdpUdpSendGetConfig = XME_STATUS_SUCCESS;
/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_dataHandler_createPort
(
    xme_core_component_t componentID,
    xme_core_component_portType_t type,
    xme_core_topic_t topic,
    int bufferSize,
    xme_core_attribute_descriptor_list_t metadata,
    int queueSize,
    bool overwrite,
    bool persistent,
    int historyDepth,
    xme_core_dataManager_dataPacketId_t * const dataPacketId
)
{
    XME_UNUSED_PARAMETER(componentID);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(bufferSize);
    XME_UNUSED_PARAMETER(metadata);
    XME_UNUSED_PARAMETER(queueSize);
    XME_UNUSED_PARAMETER(overwrite);
    XME_UNUSED_PARAMETER(persistent);
    XME_UNUSED_PARAMETER(historyDepth);

    if (type == XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION)
    {
        *dataPacketId = (xme_core_dataManager_dataPacketId_t) 1;
    }
    else
    {
        *dataPacketId = (xme_core_dataManager_dataPacketId_t) 2;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    unsigned int bufferSize
)
{
    xme_hal_mem_copy(portArray[port], buffer, bufferSize);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t port
)
{
    XME_UNUSED_PARAMETER(port);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t port,
    void * const buffer,
    unsigned int bufferSize,
    unsigned int * const bytesRead
)
{
    xme_hal_mem_copy(buffer, portArray[port], bufferSize);
    *bytesRead = bufferSize;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t port
)
{
    XME_UNUSED_PARAMETER(port);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_wp_marshal_marshaler_getConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t* topic,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort
)
{
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(topic);
    *instanceId = (xme_wp_waypoint_instanceId_t) 1;
    return statusXmeWpMarshalMarshalerGetConfig;
}

void
xme_wp_marshal_marshaler_getConfigSetStatus
(
    xme_status_t status
)
{
    statusXmeWpMarshalMarshalerGetConfig = status;
}

xme_status_t
xme_wp_udp_udpSend_getConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_dataManager_dataPacketId_t* dataId,
    void *key,
    const char* hostname,
    uint16_t port
)
{
    XME_UNUSED_PARAMETER(key);
    XME_UNUSED_PARAMETER(port);
    XME_UNUSED_PARAMETER(hostname);

    *instanceId = (xme_wp_waypoint_instanceId_t) 2;
    *dataId = (xme_core_dataManager_dataPacketId_t) 5;
    return statusXmeWpUdpUdpSendGetConfig;
}

void
xme_wp_udp_udpSend_getConfigSetStatus
(
    xme_status_t status
)
{
   statusXmeWpUdpUdpSendGetConfig = status;
}

xme_status_t
createUdpSendWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    static xme_core_exec_functionDescriptor_t* desc = NULL;
    XME_UNUSED_PARAMETER(componentId);
    if (NULL == desc)
    {
        desc = (xme_core_exec_functionDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
        xme_hal_mem_set(desc, 0x0, sizeof(xme_core_exec_functionDescriptor_t));
    }
    *descriptor = desc;
    return XME_STATUS_SUCCESS;
}

xme_status_t
udpSendWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* dataPort,
    uint8_t* key,
    const char* destIP,
    uint32_t ipPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** sendBuffer,
    bool isBroadcast
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(dataPort);
    XME_UNUSED_PARAMETER(key);
    XME_UNUSED_PARAMETER(destIP);
    XME_UNUSED_PARAMETER(ipPort);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(sizeOfTopic);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(sendBuffer);
    XME_UNUSED_PARAMETER(isBroadcast);
    return statusUdpSendWaypointAddConfig;
}

void
udpSendWaypointAddConfigSetStatus
(
    xme_status_t status
)
{
    statusUdpSendWaypointAddConfig = status;
}

xme_status_t
xme_core_exec_componentRepository_registerComponent
(
    xme_core_exec_componentDescriptor_t* componentDescriptor
)
{
    XME_UNUSED_PARAMETER(componentDescriptor);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_configurator_addComponentToSchedule
(
    xme_core_exec_transactionId_t transactionId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArgs,
    xme_hal_time_timeInterval_t afterTime_ns,
    xme_hal_time_timeInterval_t beforeTime_ns,
    uint32_t period,
    uint32_t periodOffset,
    xme_hal_linkedList_descriptor_t* schedules
)
{
    XME_UNUSED_PARAMETER(transactionId);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionArgs);
    XME_UNUSED_PARAMETER(afterTime_ns);
    XME_UNUSED_PARAMETER(beforeTime_ns);
    XME_UNUSED_PARAMETER(schedules);
    XME_UNUSED_PARAMETER(period);
    XME_UNUSED_PARAMETER(periodOffset);
    return statusXmeCoreExecConfiguratorAddComponentToSchedule;
}

void
xme_core_exec_configurator_addComponentToScheduleSetStatus
(
    xme_status_t status
)
{
    statusXmeCoreExecConfiguratorAddComponentToSchedule = status;
}

xme_status_t
xme_core_broker_addDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{
    XME_UNUSED_PARAMETER(srcDataPacketId);
    XME_UNUSED_PARAMETER(dstDataPacketId);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_configurator_rollback
(
    xme_core_exec_transactionId_t transactionId
)
{
    XME_UNUSED_PARAMETER(transactionId);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_configurator_commit
(
    xme_core_exec_transactionId_t transactionId
)
{
    XME_UNUSED_PARAMETER(transactionId);
    return XME_STATUS_SUCCESS;
}

xme_status_t
createMarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    static xme_core_exec_functionDescriptor_t* desc = NULL;
    XME_UNUSED_PARAMETER(componentId);
    if (NULL == desc)
    {
        desc = (xme_core_exec_functionDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
        xme_hal_mem_set(desc, 0x0, sizeof(xme_core_exec_functionDescriptor_t));
    }
    *descriptor = desc;
    return XME_STATUS_SUCCESS;
}

xme_status_t
marshalerWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputDataPort,
    uint8_t inputPortQueueSize,
    xme_core_dataManager_dataPacketId_t* outputDataPort,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_core_channelId_t channelID
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(inputDataPort);
    XME_UNUSED_PARAMETER(inputPortQueueSize);
    XME_UNUSED_PARAMETER(outputDataPort);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(topicSize);
    XME_UNUSED_PARAMETER(channelID);
    return XME_STATUS_SUCCESS;
}

xme_status_t
createDemarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    static xme_core_exec_functionDescriptor_t* desc = NULL;
    XME_UNUSED_PARAMETER(componentId);
    if (NULL == desc)
    {
        desc = (xme_core_exec_functionDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
        xme_hal_mem_set(desc, 0x0, sizeof(xme_core_exec_functionDescriptor_t));
    }
    *descriptor = desc;
    return XME_STATUS_SUCCESS;
}

xme_status_t
demarshalerWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputDataPort,
    uint8_t inputPortQueueSize,
    xme_core_dataManager_dataPacketId_t* outputDataPort,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_core_channelId_t channelID
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(inputDataPort);
    XME_UNUSED_PARAMETER(inputPortQueueSize);
    XME_UNUSED_PARAMETER(outputDataPort);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(topicSize);
    XME_UNUSED_PARAMETER(channelID);
    //return statusDemarshalerWaypointAddConfig;
    return XME_STATUS_INTERNAL_ERROR;
}

xme_status_t
createUdpReceiveWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    static xme_core_exec_functionDescriptor_t* desc = NULL;
    XME_UNUSED_PARAMETER(componentId);
    if (NULL == desc)
    {
        desc = (xme_core_exec_functionDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
        xme_hal_mem_set(desc, 0x0, sizeof(xme_core_exec_functionDescriptor_t));
    }
    *descriptor = desc;
    return XME_STATUS_SUCCESS;
}

xme_status_t
udpReceiveWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* dataPort,
    uint8_t* key,
    uint32_t ipPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** recvBuffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(dataPort);
    XME_UNUSED_PARAMETER(key);
    XME_UNUSED_PARAMETER(ipPort);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(sizeOfTopic);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(recvBuffer);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_wp_wci_addConfig
(
    uint32_t ipAddress,
    uint16_t portAddress,
    xme_core_channelId_t channelID,
    xme_core_topic_t topic
)
{
    XME_UNUSED_PARAMETER(ipAddress);
    XME_UNUSED_PARAMETER(portAddress);
    XME_UNUSED_PARAMETER(channelID);
    XME_UNUSED_PARAMETER(topic);
    return status_xme_wp_wci_addConfig;
}
xme_status_t
xme_core_pnp_pnpClient_instantiateComponentOnThisNode
(
    xme_core_componentType_t component
)
{
    XME_UNUSED_PARAMETER(component);
    return XME_STATUS_SUCCESS;
}
XME_EXTERN_C_END
