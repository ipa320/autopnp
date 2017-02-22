/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: pnpClientApplicationMock.c 7694 2014-03-06 15:56:24Z wiesmueller $
 */

/**
 * \file pnpClienApplicationMock.c
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
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
demarshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_wp_waypoint_instanceId_t instanceId
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
marshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_wp_waypoint_instanceId_t instanceId
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
    xme_core_dataManager_dataPacketId_t *dataPort,
    uint8_t *key,
    uint32_t ipPort,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** buffer
);

xme_status_t
udpReceiveWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_wp_waypoint_instanceId_t instanceId,
    void* buffer
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
    xme_core_dataManager_dataPacketId_t *dataPort,
    uint8_t *key,
    const char *destIP,
    uint32_t ipPort,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** buffer,
    bool isBroadcast
);

xme_status_t
udpSendWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_wp_waypoint_instanceId_t instanceId,
    void* buffer
);

xme_status_t
createChannelSelectorWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

xme_status_t
channelSelectorWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_core_channelId_t sourceChannelID,
    xme_core_channelId_t destinationChannelID,
    void** recvBuffer
);

xme_status_t
channelSelectorWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_topic_t topic,
    xme_core_channelId_t srcChID,
    xme_core_channelId_t dstChID,
    void* buffer
);

xme_status_t
createChannelInjectorWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

xme_status_t
channelInjectorWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_channelId_t injectedChannelID,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** recvBuffer
);

xme_status_t
channelInjectorWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    void* buffer
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
createDemarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(componentId);

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

    return XME_STATUS_SUCCESS;
}

xme_status_t
demarshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_wp_waypoint_instanceId_t instanceId
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(instanceId);

    return XME_STATUS_SUCCESS;
}

xme_status_t
createMarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(componentId);

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
marshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_wp_waypoint_instanceId_t instanceId
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(instanceId);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
createUdpReceiveWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(componentId);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
udpReceiveWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t *dataPort,
    uint8_t *key,
    uint32_t ipPort, 
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** buffer
)
{   XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(dataPort);
    XME_UNUSED_PARAMETER(key);
    XME_UNUSED_PARAMETER(ipPort);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(topicSize);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(buffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
udpReceiveWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_wp_waypoint_instanceId_t instanceId,
    void* buffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(buffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
createUdpSendWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(componentId);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
udpSendWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t *dataPort,
    uint8_t *key,
    const char *destIP,
    uint32_t ipPort, 
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** buffer,
    bool isBroadcast
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(dataPort);
    XME_UNUSED_PARAMETER(key);
    XME_UNUSED_PARAMETER(destIP);
    XME_UNUSED_PARAMETER(ipPort);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(topicSize);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(buffer);
    XME_UNUSED_PARAMETER(isBroadcast);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
udpSendWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_wp_waypoint_instanceId_t instanceId,
    void* buffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(buffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
createChannelSelectorWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(componentId);

    return XME_STATUS_SUCCESS;
}

xme_status_t
channelSelectorWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_core_channelId_t sourceChannelID,
    xme_core_channelId_t destinationChannelID,
    void** recvBuffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(sizeOfTopic);
    XME_UNUSED_PARAMETER(sourceChannelID);
    XME_UNUSED_PARAMETER(destinationChannelID);
    XME_UNUSED_PARAMETER(recvBuffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
channelSelectorWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_topic_t topic,
    xme_core_channelId_t srcChID,
    xme_core_channelId_t dstChID,
    void* buffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(srcChID);
    XME_UNUSED_PARAMETER(dstChID);
    XME_UNUSED_PARAMETER(buffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
createChannelInjectorWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(componentId);

    return XME_STATUS_SUCCESS;
}

xme_status_t
channelInjectorWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_channelId_t injectedChannelID,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** recvBuffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(injectedChannelID);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(sizeOfTopic);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(recvBuffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
channelInjectorWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    void* buffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(buffer);

    return XME_STATUS_SUCCESS;
}
