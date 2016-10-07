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
 * $Id: mocks.h 7802 2014-03-13 09:04:01Z geisinger $
 */

/**
 * \file mocks.h
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"

#include "xme/core/dataHandler/include/dataHandler.h"

#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"

#include "xme/wp/waypoint.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

struct xme_core_pnp_test_periodDividers_s
{
    xme_core_component_t componentID;
    xme_core_component_functionId_t functionID;
    uint32_t periodDivider;
};

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

// The following variables count the number of calls of the associated function
// in applicationClientMock.c
extern uint32_t xme_core_pnp_test_brokerRegisterFunctionCallCount;
extern uint32_t xme_core_pnp_test_brokerRemoveFunctionCallCount;
extern uint32_t xme_core_pnp_test_dataHandlerCreatePortCallCount;
extern uint32_t xme_core_pnp_test_componentWrapperReceivePortCallCount;
extern uint32_t xme_core_pnp_test_componentInitCallCount;

extern xme_hal_time_timeInterval_t xme_core_pnp_test_majorCycleLength;

// Used as return value in componentInit
extern xme_status_t xme_core_pnp_test_componentInitStatus;

// Array for storing period divider values receive xme_core_exec_scheduler_addElementToScheduleTable() in mocks.c
extern struct xme_core_pnp_test_periodDividers_s xme_core_pnp_test_periodDividers[10];

XME_EXTERN_C_END

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

//----------------------------------------------------------------------------//
//     Application Mock (Functions from XMT-generated main node file)         //
//----------------------------------------------------------------------------//

/**
 * \brief Mocked callback for waypoints.
 */
void 
mockCallback(void);

/**
 * \brief Mocked callback for waypoints.
 */
xme_status_t
mockCallbackInit
(
    void* const componentConfig
);

/**
 * \brief Mocked callback for waypoints.
 */
void
mockCallbackFini
(
    void* const componentConfig
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
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_dataManager_dataPacketId_t* inputDataPort,
    uint8_t inputPortQueueSize,
    xme_core_dataManager_dataPacketId_t* outputDataPort,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_core_channelId_t channelID
);

void
destroyMarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t* descriptor
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
);

void
destroyUdpSendWaypointInstance
(
    xme_core_exec_functionDescriptor_t* descriptor
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
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_dataManager_dataPacketId_t* inputDataPort,
    uint8_t inputPortQueueSize,
    xme_core_dataManager_dataPacketId_t* outputDataPort,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_core_channelId_t channelID
);

void
destroyDemarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t* descriptor
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
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_dataManager_dataPacketId_t* dataPort,
    uint8_t* key,
    uint32_t ipPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** recvBuffer
);

void
destroyUdpReceiveWaypointInstance
(
    xme_core_exec_functionDescriptor_t* descriptor
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
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_channelId_t sourceChannelID,
    xme_core_channelId_t destinationChannelID,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void* recvBuffer
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
    void* recvBuffer
);

xme_status_t
marshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort
);

xme_status_t
demarshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort
);

xme_status_t
udpReceiveWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* outputPort,
    void* buffer
);

xme_status_t
udpSendWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    void* buffer
);

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
);

xme_status_t
channelInjectorWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    void *buffer
);

//----------------------------------------------------------------------------//
//     Execution Manager Mock                                                 //
//----------------------------------------------------------------------------//
xme_status_t
xme_core_exec_init
(
    xme_core_exec_configStruct_t* initConfig
);

xme_status_t
xme_core_exec_stop
(
    bool atEndOfCycle
);

xme_status_t
xme_core_exec_fini(void);

xme_status_t
xme_core_exec_componentRepository_init(void);

xme_status_t
xme_core_exec_componentRepository_fini(void);

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

void
xme_core_exec_scheduler_printSchedule
(
    xme_core_exec_schedule_table_t* schedule,
    const char* message
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

xme_status_t
xme_core_exec_componentRepository_registerComponent
(
    const xme_core_exec_componentDescriptor_t* const componentDescriptor
);

xme_status_t
xme_core_exec_componentRepository_getComponent
(
    xme_core_component_t componentId,
    xme_core_exec_componentDescriptor_t** component
);

xme_status_t
xme_core_exec_componentRepository_getFunction(
        xme_core_component_t componentId,
        xme_core_component_functionId_t functionId,
        xme_core_exec_functionDescriptor_t** function
);

xme_status_t
xme_core_exec_componentRepository_getComponentFunction
(
        xme_core_exec_componentDescriptor_t* component,
        xme_core_component_functionId_t functionId,
        xme_core_exec_functionDescriptor_t** function
);

xme_status_t
xme_core_exec_scheduler_createScheduleTable
(
    xme_core_exec_schedule_table_t** schedule,
    xme_hal_time_timeInterval_t majorCycleLength_ns
);

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
);

xme_status_t
xme_core_exec_scheduler_overwriteFullScheduleSet
(
    xme_hal_linkedList_descriptor_t* source
);

xme_core_exec_schedule_handle_t
xme_core_exec_scheduler_getCurrentScheduleHandle(void);

xme_status_t
xme_core_exec_scheduler_getSchedule
(
     xme_core_exec_schedule_handle_t scheduleId,
     xme_core_exec_schedule_table_t** schedule
);

xme_status_t
xme_core_exec_setTaskState
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_exec_functionState_t newTaskState
);

xme_status_t
xme_core_exec_scheduler_replicateFullScheduleSet
(

    xme_hal_linkedList_descriptor_t* source,
    xme_hal_linkedList_descriptor_t* target
);

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
);

xme_status_t
xme_core_exec_scheduler_findWindow
(
    xme_core_exec_schedule_table_t* schedule,
    xme_hal_time_timeInterval_t wcet_ns,
    xme_hal_time_timeInterval_t fromTime_ns,
    xme_hal_time_timeInterval_t toTime_ns,
    xme_hal_time_timeInterval_t* windowStart_ns
);

xme_status_t
xme_core_exec_scheduler_replicateSchedule(
    xme_core_exec_schedule_table_t**     target,
    xme_core_exec_schedule_table_t*     source
);

xme_status_t
xme_core_exec_scheduler_autoAllocateInScheduleTable
(
    xme_core_exec_schedule_table_t*     schedule,
    xme_core_component_t                 componentId,
    xme_core_component_functionId_t     functionId,
    void*                              functionArgs,
    uint32_t periodDivider,
    uint32_t periodDividerOffset
);

xme_status_t
xme_core_exec_scheduler_clearScheduleTable
(
    xme_core_exec_schedule_table_t** schedule,
    bool destroy
);

xme_status_t
xme_core_exec_scheduler_registerSchedule
(
     xme_core_exec_schedule_table_t* schedule,
     xme_core_exec_schedule_handle_t* scheduleId
);

xme_status_t
xme_core_exec_scheduler_addFunctionToSchedule
(
    xme_core_exec_schedule_handle_t schedule,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArguments,
    uint32_t periodDivider,
    uint32_t periodDividerOffset
);

xme_status_t
xme_core_exec_scheduler_addFunctionToScheduleAt
(
    xme_core_exec_schedule_handle_t schedule,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArguments,
    xme_hal_time_timeInterval_t startTime_ns
);

xme_core_exec_schedule_table_entry_t*
xme_core_exec_scheduler_findEntryInTable
(
    xme_core_exec_schedule_table_t* table,
    xme_core_component_t cid,
    xme_core_component_functionId_t fid
);

void
xme_core_exec_scheduler_activateSchedule
(
    xme_core_exec_schedule_handle_t scheduleId
);

uint32_t
xme_core_exec_scheduler_getCycleCounter(void);

//----------------------------------------------------------------------------//
//     Broker Mock                                                            //
//----------------------------------------------------------------------------//
xme_status_t
xme_core_broker_registerFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
);

xme_status_t
xme_core_broker_addDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
);

xme_status_t
xme_core_broker_removeDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
);

xme_status_t
xme_core_broker_removeFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

xme_status_t
xme_core_broker_dataAvailabilityChange
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint8_t size
);

xme_status_t
xme_core_broker_addDataPacketToFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    bool mandatory
);

xme_status_t
xme_core_broker_removeDataPacketFromFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
);

xme_status_t
xme_core_broker_removeDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
);

bool
xme_core_broker_isFunctionRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

bool
xme_core_broker_isFunctionVariantRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
);

xme_status_t
xme_core_broker_getFunctionDataPackets
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    xme_hal_linkedList_descriptor_t* dataPacketsList
);

xme_status_t
xme_core_broker_getDataPacketFunctions
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_hal_linkedList_descriptor_t* functionsList
);

//----------------------------------------------------------------------------//
//     Data Handler Mock                                                      //
//----------------------------------------------------------------------------//
xme_status_t
xme_core_dataHandler_createDataPacket
(
    size_t dataPacketSizeInBytes,
    xme_core_dataManager_dataPacketId_t* dataPacketID
);

xme_status_t
xme_core_dataHandler_createAttribute
(
    size_t attributeSizeInBytes,
    uint32_t attributeKey,
    xme_core_dataManager_dataPacketId_t dataPacketID
);

xme_status_t
xme_core_dataHandler_configure(void);

xme_status_t 
xme_core_dataHandler_init(void);

void
xme_core_dataHandler_fini(void);

xme_status_t
xme_core_dataHandler_setDataPacketQueueSize
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t queueSize
);

xme_status_t
xme_core_dataHandler_setDataPacketPersistent
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

bool
xme_core_dataHandler_isDataPacketCreationFeasible
(
    size_t sizeInBytes
);

xme_status_t
xme_core_dataHandler_setNumberOfDatabases
(
    uint16_t numberOfDatabases
);

xme_status_t
xme_core_dataHandler_setDatabaseSize
(
    size_t sizeInBytes
);

xme_status_t
xme_core_dataHandler_setDataPacketOverwrite
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    bool overwrite,
    xme_status_t statusOnOverwrite,
    bool displayWarning
);

XME_EXTERN_C_END
