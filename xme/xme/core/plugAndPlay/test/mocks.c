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
 * $Id: mocks.c 7802 2014-03-13 09:04:01Z geisinger $
 */

/**
 * \file mocks.c
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/test/mocks.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
uint32_t xme_core_pnp_test_brokerRegisterFunctionCallCount = 0U;
uint32_t xme_core_pnp_test_brokerRemoveFunctionCallCount = 0U;
uint32_t xme_core_pnp_test_dataHandlerCreatePortCallCount = 0U;
uint32_t xme_core_pnp_test_componentWrapperReceivePortCallCount = 0U;
uint32_t xme_core_pnp_test_componentInitCallCount = 0;

xme_hal_time_timeInterval_t xme_core_pnp_test_majorCycleLength;

xme_status_t xme_core_pnp_test_componentInitStatus = XME_STATUS_SUCCESS;

struct xme_core_pnp_test_periodDividers_s xme_core_pnp_test_periodDividers[10];

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     Application Mock (Functions from XMT-generated main node file)         //
//----------------------------------------------------------------------------//
void
mockCallback(void)
{
    // Nothing to do
}

xme_status_t
mockCallbackInit
(
    void* const componentConfig
)
{
    XME_UNUSED_PARAMETER(componentConfig);

    return XME_STATUS_SUCCESS;
}

void
mockCallbackFini
(
    void* const componentConfig
)
{
    // Nothing to do
    XME_UNUSED_PARAMETER(componentConfig);
}

xme_status_t
createMarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(componentId);

    *descriptor = (xme_core_exec_functionDescriptor_t*)
            xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
    xme_hal_mem_set(*descriptor, 0, sizeof(xme_core_exec_functionDescriptor_t));
    (*descriptor)->task = (xme_hal_sched_taskCallback_t)
            (&(mockCallback));
    (*descriptor)->taskArgs = *descriptor;
    (*descriptor)->componentId = componentId;
    (*descriptor)->functionId = (xme_core_component_functionId_t) 1;
    (*descriptor)->init = &mockCallbackInit;
    (*descriptor)->fini = &mockCallbackFini;
    (*descriptor)->wcet_ns = xme_hal_time_timeIntervalFromMilliseconds(100);

    return XME_STATUS_SUCCESS;
}

xme_status_t
marshalerWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t** descriptor,
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

void
destroyMarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t* descriptor
)
{
    XME_UNUSED_PARAMETER(descriptor);

    // DO NOTHING
}

xme_status_t
createUdpSendWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(componentId);

    *descriptor = (xme_core_exec_functionDescriptor_t*)
            xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
    xme_hal_mem_set(*descriptor, 0, sizeof(xme_core_exec_functionDescriptor_t));
    (*descriptor)->task = (xme_hal_sched_taskCallback_t)
            (&(mockCallback));
    (*descriptor)->taskArgs = *descriptor;
    (*descriptor)->componentId = componentId;
    (*descriptor)->functionId = (xme_core_component_functionId_t) 1;
    (*descriptor)->init = &mockCallbackInit;
    (*descriptor)->fini = &mockCallbackFini;
    (*descriptor)->wcet_ns = xme_hal_time_timeIntervalFromMilliseconds(100);

    return XME_STATUS_SUCCESS;
}

xme_status_t
udpSendWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t** descriptor,
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

    return XME_STATUS_SUCCESS;
}

void 
destroyUdpSendWaypointInstance
(
    xme_core_exec_functionDescriptor_t* descriptor
)
{
    XME_UNUSED_PARAMETER(descriptor);

    // DO NOTHING
}

xme_status_t
createDemarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(componentId);

    *descriptor = (xme_core_exec_functionDescriptor_t*)
            xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
    xme_hal_mem_set(*descriptor, 0, sizeof(xme_core_exec_functionDescriptor_t));
    (*descriptor)->task = (xme_hal_sched_taskCallback_t)
            (&(mockCallback));
    (*descriptor)->taskArgs = *descriptor;
    (*descriptor)->componentId = componentId;
    (*descriptor)->functionId = (xme_core_component_functionId_t) 1;
    (*descriptor)->init = &mockCallbackInit;
    (*descriptor)->fini = &mockCallbackFini;
    (*descriptor)->wcet_ns = xme_hal_time_timeIntervalFromMilliseconds(100);

    return XME_STATUS_SUCCESS;
}

xme_status_t
demarshalerWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t** descriptor,
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

void
destroyDemarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t* descriptor
)
{
    XME_UNUSED_PARAMETER(descriptor);

    // DO NOTHING
}

xme_status_t 
createUdpReceiveWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    XME_UNUSED_PARAMETER(componentId);

    *descriptor = (xme_core_exec_functionDescriptor_t*)
            xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
    xme_hal_mem_set(*descriptor, 0, sizeof(xme_core_exec_functionDescriptor_t));
    (*descriptor)->task = (xme_hal_sched_taskCallback_t)
            (&(mockCallback));
    (*descriptor)->taskArgs = *descriptor;
    (*descriptor)->componentId = componentId;
    (*descriptor)->functionId = (xme_core_component_functionId_t) 1;
    (*descriptor)->init = &mockCallbackInit;
    (*descriptor)->fini = &mockCallbackFini;
    (*descriptor)->wcet_ns = xme_hal_time_timeIntervalFromMilliseconds(100);

    return XME_STATUS_SUCCESS;
}

xme_status_t
udpReceiveWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t** descriptor,
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

void
destroyUdpReceiveWaypointInstance
(
    xme_core_exec_functionDescriptor_t* descriptor
)
{
    XME_UNUSED_PARAMETER(descriptor);

    // DO NOTHING
}

xme_status_t
createChannelSelectorWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    *descriptor = (xme_core_exec_functionDescriptor_t*)
            xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
    xme_hal_mem_set(*descriptor, 0, sizeof(xme_core_exec_functionDescriptor_t));
    (*descriptor)->task = (xme_hal_sched_taskCallback_t)
            (&(mockCallback));
    (*descriptor)->taskArgs = *descriptor;
    (*descriptor)->componentId = componentId;
    (*descriptor)->functionId = (xme_core_component_functionId_t) 1;
    (*descriptor)->init = &mockCallbackInit;
    (*descriptor)->fini = &mockCallbackFini;
    (*descriptor)->wcet_ns = xme_hal_time_timeIntervalFromMilliseconds(100);

    return XME_STATUS_SUCCESS;
}

xme_status_t
channelSelectorWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_channelId_t sourceChannelID,
    xme_core_channelId_t destinationChannelID,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void* recvBuffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(sourceChannelID);
    XME_UNUSED_PARAMETER(destinationChannelID);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(sizeOfTopic);
    XME_UNUSED_PARAMETER(recvBuffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
createChannelInjectorWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
)
{
    *descriptor = (xme_core_exec_functionDescriptor_t*)
            xme_hal_mem_alloc(sizeof(xme_core_exec_functionDescriptor_t));
    xme_hal_mem_set(*descriptor, 0, sizeof(xme_core_exec_functionDescriptor_t));
    (*descriptor)->task = (xme_hal_sched_taskCallback_t)
            (&(mockCallback));
    (*descriptor)->taskArgs = *descriptor;
    (*descriptor)->componentId = componentId;
    (*descriptor)->functionId = (xme_core_component_functionId_t) 1;
    (*descriptor)->init = &mockCallbackInit;
    (*descriptor)->fini = &mockCallbackFini;
    (*descriptor)->wcet_ns = xme_hal_time_timeIntervalFromMilliseconds(100);

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
    void* recvBuffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(injectedChannelID);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(sizeOfTopic);
    XME_UNUSED_PARAMETER(recvBuffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
marshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);

    return XME_STATUS_SUCCESS;
}

xme_status_t
demarshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);

    return XME_STATUS_SUCCESS;
}

xme_status_t
udpReceiveWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* outputPort,
    void* buffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(buffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
udpSendWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    void* buffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(buffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
channelSelectorWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_channelId_t srcChID,
    xme_core_channelId_t dstChID,
    void* buffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(srcChID);
    XME_UNUSED_PARAMETER(dstChID);
    XME_UNUSED_PARAMETER(buffer);

    return XME_STATUS_SUCCESS;
}

xme_status_t
channelInjectorWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    void *buffer
)
{
    XME_UNUSED_PARAMETER(descriptor);
    XME_UNUSED_PARAMETER(instanceId);
    XME_UNUSED_PARAMETER(inputPort);
    XME_UNUSED_PARAMETER(outputPort);
    XME_UNUSED_PARAMETER(buffer);

    return XME_STATUS_SUCCESS;
}

//----------------------------------------------------------------------------//
//     Execution Manager Mock                                                 //
//----------------------------------------------------------------------------//
xme_status_t
xme_core_exec_init
(
    xme_core_exec_configStruct_t* initConfig
)
{
    XME_UNUSED_PARAMETER(initConfig);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_stop
(
    bool atEndOfCycle
)
{
    XME_UNUSED_PARAMETER(atEndOfCycle);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_fini(void)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_componentRepository_init(void)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_componentRepository_fini(void)
{
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
xme_core_exec_dispatcher_createFunctionExecutionUnit
(
    xme_core_exec_functionDescriptor_t* functionWrapper,
    bool eventTriggeredBehavior
)
{
    XME_UNUSED_PARAMETER(functionWrapper);
    XME_UNUSED_PARAMETER(eventTriggeredBehavior);

    return XME_STATUS_SUCCESS;
}

void
xme_core_exec_scheduler_printSchedule
(
    xme_core_exec_schedule_table_t* schedule,
    const char* message
)
{
    XME_UNUSED_PARAMETER(schedule);
    XME_UNUSED_PARAMETER(message);
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

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_componentRepository_registerComponent
(
    const xme_core_exec_componentDescriptor_t* const componentDescriptor
)
{
    XME_UNUSED_PARAMETER(componentDescriptor);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_componentRepository_getComponent
(
    xme_core_component_t componentId,
    xme_core_exec_componentDescriptor_t** component
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(component);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_componentRepository_getFunction(
        xme_core_component_t componentId,
        xme_core_component_functionId_t functionId,
        xme_core_exec_functionDescriptor_t** function
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(function);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_componentRepository_getComponentFunction
(
        xme_core_exec_componentDescriptor_t* component,
        xme_core_component_functionId_t functionId,
        xme_core_exec_functionDescriptor_t** function
)
{
    XME_UNUSED_PARAMETER(component);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(function);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_createScheduleTable
(
    xme_core_exec_schedule_table_t** schedule,
    xme_hal_time_timeInterval_t majorCycleLength_ns
)
{
    /* Allocate the new schedule */
    *schedule =
            (xme_core_exec_schedule_table_t*)
            xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_table_t));
    XME_UNUSED_PARAMETER(majorCycleLength_ns);
    XME_CHECK(NULL != *schedule,
        XME_STATUS_OUT_OF_RESOURCES);

    /* Initialize the linked list */
    //XME_HAL_SINGLYLINKEDLIST_INIT((*schedule)->entries);

    /* Set cycle duration */
    //(*schedule)->majorCycleDuration_ns = majorCycleLength_ns;

    /* By default, return success */
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_addElementToScheduleTable
(
    xme_core_exec_schedule_table_t* schedule,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArgs,
    xme_hal_time_timeInterval_t slotStart_ns,
    xme_hal_time_timeInterval_t slotLength_ns,
    uint32_t periodDivider,
    uint32_t periodDividerOffset,
    bool completion
)
{
    XME_UNUSED_PARAMETER(schedule);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionArgs);
    XME_UNUSED_PARAMETER(slotStart_ns);
    XME_UNUSED_PARAMETER(slotLength_ns);
    XME_UNUSED_PARAMETER(periodDivider);
    XME_UNUSED_PARAMETER(periodDividerOffset);
    XME_UNUSED_PARAMETER(completion);

    {
        uint16_t i = 0u;

        for (i = 0u; i < sizeof(xme_core_pnp_test_periodDividers) / sizeof(xme_core_pnp_test_periodDividers[0]); i++)
        {
            if ((
                    componentId == xme_core_pnp_test_periodDividers[i].componentID &&
                    functionId == xme_core_pnp_test_periodDividers[i].functionID
                ) ||
                (
                    0u == xme_core_pnp_test_periodDividers[i].componentID &&
                    0u == xme_core_pnp_test_periodDividers[i].functionID
                ))
            {
                xme_core_pnp_test_periodDividers[i].componentID = componentId;
                xme_core_pnp_test_periodDividers[i].functionID = functionId;
                xme_core_pnp_test_periodDividers[i].periodDivider = periodDivider;
                break;
            }
        }
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_overwriteFullScheduleSet
(
    xme_hal_linkedList_descriptor_t* source
)
{
    XME_UNUSED_PARAMETER(source);

    return XME_STATUS_SUCCESS;
}

xme_core_exec_schedule_handle_t
xme_core_exec_scheduler_getCurrentScheduleHandle(void)
{
    return (xme_core_exec_schedule_handle_t) 1;
}

xme_status_t
xme_core_exec_scheduler_getSchedule
(
     xme_core_exec_schedule_handle_t scheduleId,
     xme_core_exec_schedule_table_t** schedule
)
{
    XME_UNUSED_PARAMETER(scheduleId);
    *schedule =
            (xme_core_exec_schedule_table_t*)
            xme_hal_mem_alloc(sizeof(xme_core_exec_schedule_table_t));
    (*schedule)->majorCycleDuration_ns = xme_core_pnp_test_majorCycleLength;
    XME_HAL_SINGLYLINKEDLIST_INIT((*schedule)->entries);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_setTaskState
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_exec_functionState_t newTaskState
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(newTaskState);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_replicateFullScheduleSet
(

    xme_hal_linkedList_descriptor_t* source,
    xme_hal_linkedList_descriptor_t* target
)
{
    XME_UNUSED_PARAMETER(source);
    XME_UNUSED_PARAMETER(target);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_addFunctionToScheduleSet
(
        xme_hal_linkedList_descriptor_t* scheduleSet,
        xme_hal_linkedList_descriptor_t* scheduleIds,
        xme_core_component_t componentId,
        xme_core_component_functionId_t functionId,
        void* functionArgs,
        xme_hal_time_timeInterval_t fromTime_ns,
        xme_hal_time_timeInterval_t toTime_ns,
        uint32_t periodDivider,
        uint32_t periodDividerOffset
)
{
    XME_UNUSED_PARAMETER(scheduleSet);
    XME_UNUSED_PARAMETER(scheduleIds);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionArgs);
    XME_UNUSED_PARAMETER(fromTime_ns);
    XME_UNUSED_PARAMETER(toTime_ns);
    XME_UNUSED_PARAMETER(periodDivider);
    XME_UNUSED_PARAMETER(periodDividerOffset);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_findWindow
(
    xme_core_exec_schedule_table_t* schedule,
    xme_hal_time_timeInterval_t wcet_ns,
    xme_hal_time_timeInterval_t fromTime_ns,
    xme_hal_time_timeInterval_t toTime_ns,
    xme_hal_time_timeInterval_t* windowStart_ns
)
{
    XME_UNUSED_PARAMETER(schedule);
    XME_UNUSED_PARAMETER(wcet_ns);
    XME_UNUSED_PARAMETER(fromTime_ns);
    XME_UNUSED_PARAMETER(toTime_ns);
    XME_UNUSED_PARAMETER(windowStart_ns);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_replicateSchedule(
    xme_core_exec_schedule_table_t**     target,
    xme_core_exec_schedule_table_t*     source
)
{
    XME_UNUSED_PARAMETER(target);
    XME_UNUSED_PARAMETER(source);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_autoAllocateInScheduleTable
(
    xme_core_exec_schedule_table_t*     schedule,
    xme_core_component_t                 componentId,
    xme_core_component_functionId_t     functionId,
    void*                              functionArgs,
    uint32_t periodDivider,
    uint32_t periodDividerOffset
)
{
    XME_UNUSED_PARAMETER(schedule);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionArgs);
    XME_UNUSED_PARAMETER(periodDivider);
    XME_UNUSED_PARAMETER(periodDividerOffset);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_clearScheduleTable
(
    xme_core_exec_schedule_table_t** schedule,
    bool destroy
)
{
    XME_UNUSED_PARAMETER(schedule);
    XME_UNUSED_PARAMETER(destroy);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_registerSchedule
(
     xme_core_exec_schedule_table_t* schedule,
     xme_core_exec_schedule_handle_t* scheduleId
)
{
    XME_UNUSED_PARAMETER(schedule);
    XME_UNUSED_PARAMETER(scheduleId);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_addFunctionToSchedule
(
    xme_core_exec_schedule_handle_t schedule,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArguments,
    uint32_t periodDivider,
    uint32_t periodDividerOffset
)
{
    XME_UNUSED_PARAMETER(schedule);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionArguments);
    XME_UNUSED_PARAMETER(periodDivider);
    XME_UNUSED_PARAMETER(periodDividerOffset);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_exec_scheduler_addFunctionToScheduleAt
(
    xme_core_exec_schedule_handle_t schedule,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArguments,
    xme_hal_time_timeInterval_t startTime_ns
)
{
    XME_UNUSED_PARAMETER(schedule);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionArguments);
    XME_UNUSED_PARAMETER(startTime_ns);

    return XME_STATUS_SUCCESS;
}

xme_core_exec_schedule_table_entry_t*
xme_core_exec_scheduler_findEntryInTable
(
    xme_core_exec_schedule_table_t* table,
    xme_core_component_t cid,
    xme_core_component_functionId_t fid
)
{
    XME_UNUSED_PARAMETER(table);
    XME_UNUSED_PARAMETER(cid);
    XME_UNUSED_PARAMETER(fid);

    return NULL;
}

void
xme_core_exec_scheduler_activateSchedule
(
    xme_core_exec_schedule_handle_t scheduleId
)
{
    XME_UNUSED_PARAMETER(scheduleId);
}

uint32_t
xme_core_exec_scheduler_getCycleCounter(void)
{
    return 0u;
}

//----------------------------------------------------------------------------//
//     Broker Mock                                                            //
//----------------------------------------------------------------------------//
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

    xme_core_pnp_test_brokerRegisterFunctionCallCount++;

    return XME_STATUS_SUCCESS;
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
xme_core_broker_removeFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);

    xme_core_pnp_test_brokerRemoveFunctionCallCount++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_dataAvailabilityChange
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint8_t size
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(size);

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
//     Data Handler Mock                                                      //
//----------------------------------------------------------------------------//
xme_status_t
xme_core_dataHandler_createDataPacket
(
    size_t dataPacketSizeInBytes,
    xme_core_dataManager_dataPacketId_t* dataPacketID
)
{
    XME_UNUSED_PARAMETER(dataPacketSizeInBytes);

    xme_core_pnp_test_dataHandlerCreatePortCallCount++;

    *dataPacketID = (xme_core_dataManager_dataPacketId_t)2;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_createAttribute
(
    size_t attributeSizeInBytes,
    uint32_t attributeKey,
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    XME_UNUSED_PARAMETER(attributeSizeInBytes);
    XME_UNUSED_PARAMETER(attributeKey);
    XME_UNUSED_PARAMETER(dataPacketID);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_configure(void)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_init(void)
{
    return XME_STATUS_SUCCESS;
}

void
xme_core_dataHandler_fini(void)
{
    
}

xme_status_t
xme_core_dataHandler_setDataPacketQueueSize
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t queueSize
)
{
    XME_UNUSED_PARAMETER(dataPacketID);
    XME_UNUSED_PARAMETER(queueSize);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setDataPacketPersistent
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    XME_UNUSED_PARAMETER(dataPacketID);

    return XME_STATUS_SUCCESS;
}

bool
xme_core_dataHandler_isDataPacketCreationFeasible
(
    size_t sizeInBytes
)
{
    XME_UNUSED_PARAMETER(sizeInBytes);

    return true;
}

xme_status_t
xme_core_dataHandler_setNumberOfDatabases
(
    uint16_t numberOfDatabases
)
{
    XME_UNUSED_PARAMETER(numberOfDatabases);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setDatabaseSize
(
    size_t sizeInBytes
)
{
    XME_UNUSED_PARAMETER(sizeInBytes);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setDataPacketOverwrite
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    bool overwrite,
    xme_status_t statusOnOverwrite,
    bool displayWarning
)
{
    XME_UNUSED_PARAMETER(dataPacketID);
    XME_UNUSED_PARAMETER(overwrite);
    XME_UNUSED_PARAMETER(statusOnOverwrite);
    XME_UNUSED_PARAMETER(displayWarning);

    return XME_STATUS_SUCCESS;
}
