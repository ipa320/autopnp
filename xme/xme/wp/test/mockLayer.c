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
 * $Id: mockLayer.c 7694 2014-03-06 15:56:24Z wiesmueller $
 */

/**
 * \file
 *         Mock layer for login client tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/broker/include/broker.h"
#include "xme/core/broker/include/brokerPnpManagerInterface.h"
#include "xme/core/component.h"
#include "xme/core/coreTypes.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/hal/include/mem.h"

#include "xme/wp/waypoint.h"

#include <stdbool.h>

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
xme_status_t status_createDemarshalerWaypointInstance = XME_STATUS_SUCCESS;
xme_status_t status_demarshalerWaypointAddConfig = XME_STATUS_SUCCESS;
xme_status_t status_xme_core_exec_componentRepository_registerComponent = XME_STATUS_SUCCESS;
xme_status_t status_xme_core_exec_configurator_addComponentToSchedule = XME_STATUS_SUCCESS;
xme_status_t status_xme_core_exec_configurator_rollback = XME_STATUS_SUCCESS;
xme_status_t status_xme_core_exec_configurator_commit = XME_STATUS_SUCCESS;
xme_status_t status_xme_core_broker_addDataPacketTransferEntry = XME_STATUS_SUCCESS;
xme_status_t status_createMarshalerWaypointInstance = XME_STATUS_SUCCESS;
xme_status_t status_marshalerWaypointAddConfig = XME_STATUS_SUCCESS;
xme_status_t status_createUdpSendWaypointInstance = XME_STATUS_SUCCESS;
xme_status_t status_udpSendWaypointAddConfig = XME_STATUS_SUCCESS;
xme_status_t status_createUdpReceiveWaypointInstance = XME_STATUS_SUCCESS;
xme_status_t status_udpReceiveWaypointAddConfig = XME_STATUS_SUCCESS;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
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

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     Application Mock (Functions from XMT-generated main node file)         //
//----------------------------------------------------------------------------//
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
    return status_createUdpSendWaypointInstance;
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
    return status_udpSendWaypointAddConfig;
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
    return status_createMarshalerWaypointInstance;
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
    return status_marshalerWaypointAddConfig;
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
    return status_createDemarshalerWaypointInstance;
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
    return status_demarshalerWaypointAddConfig;
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
    return status_createUdpReceiveWaypointInstance;
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
    return status_udpReceiveWaypointAddConfig;
}

//----------------------------------------------------------------------------//
//     Execution Manager Mock                                                 //
//----------------------------------------------------------------------------//
extern xme_status_t
xme_core_exec_componentRepository_registerComponent
(
    const xme_core_exec_componentDescriptor_t* const componentDescriptor
)
{
    XME_UNUSED_PARAMETER(componentDescriptor);
    return status_xme_core_exec_componentRepository_registerComponent;
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
    return status_xme_core_exec_configurator_addComponentToSchedule;
}

xme_status_t
xme_core_exec_configurator_rollback
(
    xme_core_exec_transactionId_t transactionId
)
{
    XME_UNUSED_PARAMETER(transactionId);
    return status_xme_core_exec_configurator_rollback;
}

xme_status_t
xme_core_exec_configurator_commit
(
    xme_core_exec_transactionId_t transactionId
)
{
    XME_UNUSED_PARAMETER(transactionId);
    return status_xme_core_exec_configurator_commit;
}

//----------------------------------------------------------------------------//
//     Broker Mock                                                            //
//----------------------------------------------------------------------------//
xme_status_t
xme_core_broker_addDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{
    XME_UNUSED_PARAMETER(srcDataPacketId);
    XME_UNUSED_PARAMETER(dstDataPacketId);
    return status_xme_core_broker_addDataPacketTransferEntry;
}

xme_status_t
xme_core_broker_init 
(
    void* params
)
{
    XME_UNUSED_PARAMETER(params);
    return XME_STATUS_SUCCESS;
}

void
xme_core_broker_fini (void)
{
    
}

xme_status_t
xme_core_broker_removeDataPacketTransferEntry
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
xme_core_broker_setTransferCallback
(
    xme_core_transferDataCallback_t transferCallback
)
{
    XME_UNUSED_PARAMETER(transferCallback);
    return XME_STATUS_SUCCESS;
}

bool
xme_core_broker_isDataPacketAvailable
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    return true;
}

bool
xme_core_broker_isDataPacketRegistered
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    return true;
}

xme_status_t
xme_core_broker_registerFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_addDataPacketToFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    bool mandatory
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);
    XME_UNUSED_PARAMETER(mandatory);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeDataPacketFromFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeFunctionVariant
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    return XME_STATUS_SUCCESS;
}

bool
xme_core_broker_isFunctionRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    return true;
}

bool
xme_core_broker_isFunctionVariantRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);
    return true;
}

xme_status_t
xme_core_broker_getFunctionDataPackets
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    xme_hal_linkedList_descriptor_t* dataPacketsList
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);
    XME_UNUSED_PARAMETER(dataPacketsList);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_getDataPacketFunctions
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_hal_linkedList_descriptor_t* functionsList
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(functionsList);
    return XME_STATUS_SUCCESS;
}

//----------------------------------------------------------------------------//
//     Component Repository Mock                                              //
//----------------------------------------------------------------------------//
xme_status_t
xme_core_nodeMgr_compRep_getComponentInstance
(
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID,
    xme_core_nodeMgr_compRep_componentHandle_t* const outComponentHandle
)
{
    XME_UNUSED_PARAMETER(nodeID);
    XME_UNUSED_PARAMETER(componentID);
    XME_UNUSED_PARAMETER(outComponentHandle);
    return XME_STATUS_SUCCESS;
}

xme_core_nodeMgr_compRep_portHandle_t
xme_core_nodeMgr_compRep_getPort
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    uint16_t portTypeID
)
{
    XME_UNUSED_PARAMETER(componentHandle);
    XME_UNUSED_PARAMETER(portTypeID);
    return 0;
}

xme_core_dataManager_dataPacketId_t
xme_core_nodeMgr_compRep_getDataPacketID
(
    xme_core_nodeMgr_compRep_portHandle_t portHandle
)
{
    XME_UNUSED_PARAMETER(portHandle);
    return 0;
}
