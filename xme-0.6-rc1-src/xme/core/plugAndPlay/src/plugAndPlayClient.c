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
 * $Id: plugAndPlayClient.c 5245 2013-10-01 07:43:48Z wiesmueller $
 */

/**
 * \file
 *         Plug and Play Client.
 *
 */

/**
 * \addtogroup core_pnp_pnpClient
 * @{
 */
/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"

#include "xme/hal/include/mem.h"

#include "xme/core/broker/include/broker.h"

#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/executionManager/include/executionManager.h"

#include "xme/core/manifestRepository/include/manifestRepository.h"

#include <inttypes.h>
#ifdef WIN32
#define sscanf sscanf_s
#endif

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief  Creates the configuration table for the Component.
 *
 * \param portType Type of port, Subscription or Publication.
 * \param vertexData Pointer to data assocoated with the corresponding Component.
 * \param edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 * \param componentType Type of the component to be scheduled.
 * \param componentId Component Id of the to be scheduled component.
 * \param early If this components needs to be sceheduled early.
 *              This is true for memcopy operations.
 *
 * \return Pointer to the data packet created for that port.
 */
static xme_core_dataManager_dataPacketId_t*
addConfigComponentPortVertex
( 
    xme_core_component_portType_t portType, 
    const char *vertexData, 
    const char *edgeConfData,
    xme_core_componentType_t componentType,
    xme_core_component_t componentId,
    bool early
);

/**
 * \brief  Creates the configuration table for the Marshaler Waypoint.
 *
 * \param portType Type of port, Subscription or Publication.
 * \param vertexData Pointer to data assocoated with the corresponding Waypoint.
 * \param edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
static xme_core_dataManager_dataPacketId_t*
addConfigMarshaler
(
    xme_core_component_portType_t portType, 
    const char *vertexData, 
    const char *edgeConfData
);

/**
 * \brief  Creates the configuration table for the Marshaler Waypoint.
 *
 * \param portType Type of port, Subscription or Publication.
 * \param vertexData Pointer to data assocoated with the corresponding Waypoint.
 * \param edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
static xme_core_dataManager_dataPacketId_t*
addConfigDemarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeConfData
);

/**
 * \brief  Creates the configuration table for the UDP Send Waypoint.
 *
 * \param portType Type of port, Subscription or Publication.
 * \param vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
static xme_core_dataManager_dataPacketId_t*
addConfigUDPSend
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeConfData
);

/**
 * \brief  Creates the configuration table for the UDP Receieve Waypoint.
 *
 * \param portType Type of port, Subscription or Publication.
 * \param vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
static xme_core_dataManager_dataPacketId_t*
addConfigUDPRecv
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeConfData
);

/**
 * \brief  Creates the table for the unique portmappings which are to be registered with broker.
 *
 * \param srcPort The source port of data that is Publication port.
 * \param dstPort The destination port of data that is Subscription port.
 */
static void
createPortMappingArray
(
    xme_core_dataManager_dataPacketId_t* srcPort,
    xme_core_dataManager_dataPacketId_t* dstPort
);

/**
 * \brief  Iterates over the portmapping array and register the mappings with the broker.
 *
 * \retval XME_STATUS_SUCCESS if all mappings were successfully registered with broker.
 * \retval XME_STATUS_INTERNAL_ERROR if there was an error for any mapping.
 */
static xme_status_t
addPortMappingToBroker(void);

/**
 * \brief  This function initializes all the newly added components.
 *
 * \details It iterates over all the configuration tables, initializes the corresponding.
 *          parameters of the components and addes them to the scheduler to be ready
 *          for execution in the next cycle.
 *
 * \retval XME_STATUS_SUCCESS if the components were initialized successfully
 * \retval other status value returned by the failing component
 *
 */
static xme_status_t 
initComponents(void);

/**
 * \brief  Initializes a component descriptor. This is required for adding schedule to 
 *         Execution manager for the newly created component/waypoint.
 *
 * \param  fDesc The function descriptor which needs to be inserted in the schedule.
 *
 * \return Pointer to the newly created component descriptor.
 */
static xme_core_exec_componentDescriptor_t*
getComponentDescriptor
(
    xme_core_exec_functionDescriptor_t *fDesc
);

/**
 * \brief  Create instance of component 'sensorB'.
 *
 * \details Creates ports, allocates memory for and populates given function descriptor and
 *          registers component  functions at the broker.
 *          If any error occurred during initialization, the given descriptor is set to NULL.
 *
 * \param  readSensorValueDescriptor Descriptor for function 'readSensorValue'.
 * \param  componentId Id of this component instance.
 * \param  dataPacketIds Array of data packet ids that will be used for this components ports.
 *         There must be 1 elements in the array.
 *         FIXME: This parameter is only added as a temporary solution for the pnpClient and should be removed as soon as
 *                the pnpClient uses instanceManifests and does not call the main node file anymore.
 *
 * \retval XME_STATUS_SUCCESS When initialization succeded without errors.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 * \retval XME_STATUS_INTERNAL_ERROR When an error occured during initialization.
 */
xme_status_t
createSensorBInstance
(
    xme_core_component_t componentId,
    xme_core_exec_functionDescriptor_t** readSensorValueDescriptor,
    xme_core_dataManager_dataPacketId_t* dataPacketIds
);

/**
 * \brief  Create instance of component 'sensorKB'.
 *
 * \details Creates ports, allocates memory for and populates given function descriptor and
 *          registers component  functions at the broker.
 *          If any error occurred during initialization, the given descriptor is set to NULL.
 *
 * \param  readSensorValueDescriptor Descriptor for function 'readSensorValue'.
 * \param  componentId Id of this component instance.
 * \param  dataPacketIds Array of data packet ids that will be used for this components ports.
 *         There must be 1 elements in the array.
 *         FIXME: This parameter is only added as a temporary solution for the pnpClient and should be removed as soon as
 *                the pnpClient uses instanceManifests and does not call the main node file anymore.
 *
 * \retval XME_STATUS_SUCCESS When initialization succeded without errors.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 * \retval XME_STATUS_INTERNAL_ERROR When an error occured during initialization.
 */
xme_status_t
createSensorKBInstance
(
    xme_core_component_t componentId,
    xme_core_exec_functionDescriptor_t** readSensorValueDescriptor,
    xme_core_dataManager_dataPacketId_t* dataPacketIds
);

/**
 * \brief  Create instance of component 'sensorMB'.
 *
 * \details Creates ports, allocates memory for and populates given function descriptor and
 *          registers component  functions at the broker.
 *          If any error occurred during initialization, the given descriptor is set to NULL.
 *
 * \param  readSensorValueDescriptor Descriptor for function 'readSensorValue'.
 * \param  componentId Id of this component instance.
 * \param  dataPacketIds Array of data packet ids that will be used for this components ports.
 *         There must be 1 elements in the array.
 *         FIXME: This parameter is only added as a temporary solution for the pnpClient and should be removed as soon as
 *                the pnpClient uses instanceManifests and does not call the main node file anymore.
 *
 * \retval XME_STATUS_SUCCESS When initialization succeded without errors.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 * \retval XME_STATUS_INTERNAL_ERROR When an error occured during initialization.
 */
xme_status_t
createSensorMBInstance
(
    xme_core_component_t componentId,
    xme_core_exec_functionDescriptor_t** readSensorValueDescriptor,
    xme_core_dataManager_dataPacketId_t* dataPacketIds
);

/**
 * \brief  Create instance of component 'monitorB'.
 *
 * \details Creates ports, allocates memory for and populates given function descriptor and
 *          registers component  functions at the broker.
 *          If any error occurred during initialization, the given descriptor is set to NULL.
 *
 * \param  printSensorValueDescriptor Descriptor for function 'printSensorValue'.
 * \param  componentId Id of this component instance.
 * \param  dataPacketIds Array of data packet ids that will be used for this components ports.
 *         There must be 1 elements in the array.
 *         FIXME: This parameter is only added as a temporary solution for the pnpClient and should be removed as soon as
 *                the pnpClient uses instanceManifests and does not call the main node file anymore.
 *
 * \retval XME_STATUS_SUCCESS When initialization succeded without errors.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 * \retval XME_STATUS_INTERNAL_ERROR When an error occured during initialization.
 */
xme_status_t
createMonitorBInstance
(
    xme_core_component_t componentId,
    xme_core_exec_functionDescriptor_t** printSensorValueDescriptor,
    xme_core_dataManager_dataPacketId_t* dataPacketIds
);

/**
 * \brief  Create instance of component 'monitorKB'.
 *
 * \details Creates ports, allocates memory for and populates given function descriptor and
 *          registers component  functions at the broker.
 *          If any error occurred during initialization, the given descriptor is set to NULL.
 *
 * \param  printSensorValueDescriptor Descriptor for function 'printSensorValue'.
 * \param  componentId Id of this component instance.
 * \param  dataPacketIds Array of data packet ids that will be used for this components ports.
 *         There must be 1 elements in the array.
 *         FIXME: This parameter is only added as a temporary solution for the pnpClient and should be removed as soon as
 *                the pnpClient uses instanceManifests and does not call the main node file anymore.
 *
 * \retval XME_STATUS_SUCCESS When initialization succeded without errors.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 * \retval XME_STATUS_INTERNAL_ERROR When an error occured during initialization.
 */
xme_status_t
createMonitorKBInstance
(
    xme_core_component_t componentId,
    xme_core_exec_functionDescriptor_t** printSensorValueDescriptor,
    xme_core_dataManager_dataPacketId_t* dataPacketIds
);

/**
 * \brief  Create instance of component 'monitorMB'.
 *
 * \details Creates ports, allocates memory for and populates given function descriptor and
 *          registers component  functions at the broker.
 *          If any error occurred during initialization, the given descriptor is set to NULL.
 *
 * \param  printSensorValueDescriptor Descriptor for function 'printSensorValue'.
 * \param  componentId Id of this component instance.
 * \param  dataPacketIds Array of data packet ids that will be used for this components ports.
 *         There must be 1 elements in the array.
 *         FIXME: This parameter is only added as a temporary solution for the pnpClient and should be removed as soon as
 *                the pnpClient uses instanceManifests and does not call the main node file anymore.
 *
 * \retval XME_STATUS_SUCCESS When initialization succeded without errors.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 * \retval XME_STATUS_INTERNAL_ERROR When an error occured during initialization.
 */
xme_status_t
createMonitorMBInstance
(
    xme_core_component_t componentId,
    xme_core_exec_functionDescriptor_t** printSensorValueDescriptor,
    xme_core_dataManager_dataPacketId_t* dataPacketIds
);

/**
 * \brief  Create demarshaler waypoint instance.
 *         This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param  descriptor Will be set to a pointer to the allocation function descriptor.
 * \param  componentId Component id that will be used in the function descriptor.
 *
 * \retval XME_STATUS_SUCCESS when initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 */
extern xme_status_t
createDemarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

/**
 * \brief  Adds config entries for the demarshaler waypoint
 *         This function is present in main node file.
 *
 * \param descriptor The pointer to function descriptor obtained by corresponding Init function
 * \param inputDataPort The pointer to input data port registered by the component
 * \param outputDataPort The pointer to output data port registered by the component
 * \param instanceId The pointer to waypoint instance Id filled by the function.
 * \param topic topic for which this configuration is added for this waypoint.
 * \param topicSize size of the topic.
 *
 * \retval XME_STATUS_SUCCESS if configuration was successfuly added
 * \retval XME_STATUS_INTERNAL_ERROR if there was an error
 */
extern xme_status_t
demarshalerWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputDataPort,
    xme_core_dataManager_dataPacketId_t* outputDataPort,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_topic_t topic,
    uint16_t topicSize
);

/**
 * \brief  Create marshaler waypoint instance.
 *         This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param  descriptor Will be set to a pointer to the allocation function descriptor.
 * \param  componentId Component id that will be used in the function descriptor.
 *
 * \retval XME_STATUS_SUCCESS when initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 */
extern xme_status_t
createMarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

/**
 * \brief  Adds config entries for the marshaler waypoint
 *         This function is present in main node file.
 *
 * \param descriptor The pointer to function descriptor obtained by corresponding Init function
 * \param inputDataPort The pointer to input data port registered by the component
 * \param outputDataPort The pointer to output data port registered by the component
 * \param instanceId The pointer to waypoint instance Id filled by the function.
 * \param topic topic for which this configuration is added for this waypoint.
 * \param topicSize size of the topic.
 *
 * \retval XME_STATUS_SUCCESS if configuration was successfuly added
 * \retval XME_STATUS_INTERNAL_ERROR if there was an error
 */
extern xme_status_t
marshalerWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t *inputDataPort,
    xme_core_dataManager_dataPacketId_t *outputDataPort,
    xme_wp_waypoint_instanceId_t* instanceId, 
    xme_core_topic_t topic,
    uint16_t topicSize
);

/**
 * \brief  Create marshaler waypoint instance.
 *         This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param  descriptor Will be set to a pointer to the allocation function descriptor.
 * \param  componentId Component id that will be used in the function descriptor.
 *
 * \retval XME_STATUS_SUCCESS when initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 */
extern xme_status_t 
createUdpReceiveWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

/**
 * \brief  Adds config entries for the udpReceive waypoint
 *         This function is present in main node file.
 *
 * \param descriptor The pointer to function descriptor obtained by corresponding Init function
 * \param dataPort The pointer to data port registered by the component
 * \param key The key associated to the UDP.
 * \param ipPort The IP port used for configuring the UDP receive.
 * \param topic topic for which this configuration is added for this waypoint.
 * \param topicSize size of the topic.
 * \param instanceId The pointer to waypoint instance Id filled by the function.
 * \param buffer the buffer used in the configuration of UDP receive waypoint. 
 *
 * \retval XME_STATUS_SUCCESS if configuration was successfuly added
 * \retval XME_STATUS_INTERNAL_ERROR if there was an error
 */
extern xme_status_t 
udpReceiveWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t *dataPort,
    uint8_t *key,
    uint32_t ipPort, 
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t* instanceId,
    void *buffer
);

/**
 * \brief  Create udp send waypoint instance.
 *         This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param  descriptor Will be set to a pointer to the allocation function descriptor.
 * \param  componentId Component id that will be used in the function descriptor.
 *
 * \retval XME_STATUS_SUCCESS when initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES initialization failure due to insufficient resources.
 */
extern xme_status_t
createUdpSendWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

/**
 * \brief  Adds config entries for the udpSend waypoint
 *         This function is present in main node file.
 *
 * \param descriptor The pointer to function descriptor obtained by corresponding Init function
 * \param dataPort The pointer to data port registered by the component
 * \param key The key associated to the UDP.
 * \param destIP The target IP address used for configuring the UDP send.
 * \param ipPort The IP port used for configuring the UDP receive.
 * \param topic topic for which this configuration is added for this waypoint.
 * \param topicSize size of the topic.
 * \param instanceId The pointer to waypoint instance Id filled by the function.
 * \param buffer the buffer used in the configuration of UDP receive waypoint. 
 * \param isBroadcast if this configuration is to be used for broadcast.
 *
 * \retval XME_STATUS_SUCCESS if configuration was successfuly added
 * \retval XME_STATUS_INTERNAL_ERROR if there was an error
 */
extern xme_status_t 
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
    void *buffer,
    bool isBroadcast
);

/******************************************************************************/
/***   Type definitions and Variables                                       ***/
/******************************************************************************/

/**
 * \brief  Array of ports for various components.
 */
static xme_core_dataManager_dataPacketId_t ports[20];

/**
 * \brief  index into the array of ports
 */
static uint16_t gPortIndex = 0;

/**
 * \brief  Tracks the highest used componentID.
 */
static xme_core_component_t componentID = (xme_core_component_t) 16;

/**
 * \brief  Table to store configuration for components.
 */
static XME_HAL_TABLE(componentPortVertex_config_t, componentPortVertex_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for masrhaler waypoint.
 */
static XME_HAL_TABLE(marshaler_config_t, marshaler_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for demarshaler waypoint.
 */
static XME_HAL_TABLE(demarshaler_config_t, demarshaler_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for udpSend waypoint.
 */
static XME_HAL_TABLE(udpSend_config_t, udpSend_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for udpReceive waypoint.
 */
static XME_HAL_TABLE(udpRecv_config_t, udpRecv_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store port mappings.
 */
static XME_HAL_TABLE(portMapping_t, portMapping_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

static xme_core_dataManager_dataPacketId_t*
addConfigComponentPortVertex
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    xme_core_componentType_t componentType,
    xme_core_component_t componentId,
    bool early
)
{
    componentPortVertex_config_t* configItem;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize;

    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;

    //vertexdata has containerId and currently we don't need it.
    XME_UNUSED_PARAMETER(vertexData);
    //extract topic and size from edgeData
    sscanf(edgeData, "%" SCNu16 "%*c%" SCNu16, &topic, &topicSize);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT
    (
        componentPortVertex_config_table, 
        xme_hal_table_rowHandle_t,
        configItemHandle,
        componentPortVertex_config_t,
        configItem,
        (configItem->portType == portType && configItem->topic == topic && configItem->componentId == componentId)
    );
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(componentPortVertex_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(componentPortVertex_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->port = &ports[gPortIndex++];
            configItem->portType = portType;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->fDescriptor = NULL;
            configItem->componentType = componentType;
            configItem->componentId = componentId;
            configItem->toSchedule = true;
            configItem->totalToSchedule = 1;
            configItem->scheduleEarly = 0;
        }
    }
    else
    {
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION == portType)
        {
            configItem->toSchedule = true;
            configItem->totalToSchedule++;
        }
    }
    if (early)
    {
        configItem->scheduleEarly++;
    }
    return configItem->port;
}

static xme_core_dataManager_dataPacketId_t*
addConfigMarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    marshaler_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize;

    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;

    //vertexdata need not have anything here
    XME_UNUSED_PARAMETER(vertexData);
    //edgeData should have the topicid and size
    sscanf(edgeData, "%" SCNu16 "%*c%" SCNu16, &topic, &topicSize);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(marshaler_config_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    marshaler_config_t, configItem,
                    (configItem->topic==topic));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(marshaler_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(marshaler_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->outputPort = &ports[gPortIndex++];
            configItem->inputPort = &ports[gPortIndex++];
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->fDescriptor = NULL;
            configItem->toSchedule = true;
            configItem->totalToSchedule = 1;
        }
    }
    if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == portType)
    {
        return configItem->outputPort;
    }
    else
    {
        return configItem->inputPort;
    }
}

static xme_core_dataManager_dataPacketId_t*
addConfigDemarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    demarshaler_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle;
    xme_core_topic_t topic;
    uint16_t topicSize;

    configItem = NULL;
    configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    topic = XME_CORE_TOPIC_INVALID_TOPIC;
    topicSize = 0;

    //vertexdata need not have anything here
    XME_UNUSED_PARAMETER(vertexData);
    //edgeData should have the topicid and size
    sscanf(edgeData, "%" SCNu16 "%*c%" SCNu16, &topic, &topicSize);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(demarshaler_config_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    demarshaler_config_t, configItem,
                    (configItem->topic==topic));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(demarshaler_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(demarshaler_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->outputPort = &ports[gPortIndex++];
            configItem->inputPort = &ports[gPortIndex++];
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->fDescriptor = NULL;
            configItem->totalToSchedule = 0;
        }
    }
    configItem->toSchedule = true;
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

static xme_core_dataManager_dataPacketId_t*
addConfigUDPSend
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    udpSend_config_t *configItem;
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

    xme_hal_mem_set(&destIP,0x0,sizeof(destIP));
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
    xme_hal_safeString_snprintf(destIP,24,"%d.%d.%d.%d",intDestIP[0], intDestIP[1], intDestIP[2], intDestIP[3]);
    //sscanf((char *)&(vertexData[offset]), "%hu", &port);
    //port = 33221;
    //xme_hal_safeString_strncpy(destIP,"127.0.0.1", 24);
    //edgeData should have the topicid and size
    sscanf(edgeData, "%" SCNu16 "|%" SCNu16, &topic, &topicSize);

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(udpSend_config_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    udpSend_config_t, configItem,
                    ((xme_hal_mem_compare(configItem->destIP,destIP,strlen(destIP))==0) && configItem->port==port && xme_hal_mem_compare(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH)==0));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(udpSend_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(udpSend_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->inputPort = &ports[gPortIndex++];
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            xme_hal_mem_copy(configItem->destIP,destIP,strlen(destIP));
            xme_hal_mem_copy(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH);
            configItem->port = (uint16_t)port;
            configItem->fDescriptor = NULL;
            configItem->toSchedule = true;
            configItem->totalToSchedule = 1;
        }
    }
    return configItem->inputPort;
}    

static xme_core_dataManager_dataPacketId_t*
addConfigUDPRecv
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
)
{
    udpRecv_config_t *configItem;
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
    XME_HAL_TABLE_GET_NEXT(udpRecv_config_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    udpRecv_config_t, configItem,
                    (configItem->port==port && xme_hal_mem_compare(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH)==0));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(udpRecv_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(udpRecv_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->outputPort = &ports[gPortIndex++];
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            xme_hal_mem_copy(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH);
            configItem->port = (uint16_t) port;
            configItem->fDescriptor = NULL;
            configItem->totalToSchedule = 0;
        }
    }
    configItem->toSchedule = true;
    configItem->totalToSchedule++;
    return configItem->outputPort;
}

static void
createPortMappingArray
(
    xme_core_dataManager_dataPacketId_t *srcPort,
    xme_core_dataManager_dataPacketId_t *dstPort
)
{
    portMapping_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(portMapping_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    portMapping_t, configItem,
                    (configItem->srcPort == srcPort && configItem->dstPort==dstPort));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(portMapping_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(portMapping_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->srcPort = srcPort;
            configItem->dstPort = dstPort;
        }
    }
}

static xme_status_t
addPortMappingToBroker(void)
{
    XME_HAL_TABLE_ITERATE_BEGIN(portMapping_table, xme_hal_table_rowHandle_t, rh, portMapping_t, configItem);
    {
        xme_status_t status = xme_core_broker_addDataPacketTransferEntry(*(configItem->srcPort), *(configItem->dstPort));
        if (XME_STATUS_SUCCESS != status && XME_STATUS_ALREADY_EXIST !=status)
            return XME_STATUS_INTERNAL_ERROR;
    }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

static xme_status_t 
initComponents(void)
{
    xme_core_exec_transactionId_t trxnId = (xme_core_exec_transactionId_t)1;
    xme_status_t status;
    xme_core_exec_componentDescriptor_t* cDesc;

    XME_HAL_TABLE_ITERATE_BEGIN(componentPortVertex_config_table, xme_hal_table_rowHandle_t, rh, componentPortVertex_config_t, configItem);
    {
        if (true == configItem->toSchedule && XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == configItem->portType)
        {
            if (NULL == configItem->fDescriptor)
            {
                xme_core_dataManager_dataPacketId_t* temp = configItem->port;

                // FIXME: This is a temporary HACK to support plug and play for a selected set of components.
                //        This will be soon replaced by a manifest-based approach with symbol lookup.
                //        See Issue #3178.
                switch (configItem->componentType)
                {
                    /*case XME_CORE_COMPONENT_TYPE_SENSOR:
                        status = createSensorInstance(configItem->componentId, &(configItem->fDescriptor), temp);
                        break;*/

                    case XME_CORE_COMPONENT_TYPE_SENSOR_B:
                        status = createSensorBInstance(configItem->componentId, &(configItem->fDescriptor), temp);
                        break;

                    case XME_CORE_COMPONENT_TYPE_SENSOR_KB:
                        status = createSensorKBInstance(configItem->componentId, &(configItem->fDescriptor), temp);
                        break;

                    case XME_CORE_COMPONENT_TYPE_SENSOR_MB:
                        status = createSensorMBInstance(configItem->componentId, &(configItem->fDescriptor), temp);
                        break;

                    default:
                        status = XME_STATUS_INTERNAL_ERROR;
                        XME_LOG(XME_LOG_WARNING, "Attempt to instantiate unknown component type 0x%08X!", configItem->componentType);
                }

                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                cDesc = getComponentDescriptor(configItem->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                status = xme_core_exec_configurator_addComponentToSchedule
                (
                    trxnId, 
                    configItem->fDescriptor->componentId,
                    configItem->fDescriptor->functionId, 
                    configItem->fDescriptor->taskArgs,
                    0,
                    0,
                    0,
                    0,
                    NULL
                );
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    status,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );

                xme_core_exec_dispatcher_createFunctionExecutionUnit(configItem->fDescriptor, true);
                configItem->totalToSchedule--;
            }
            while (0 != configItem->totalToSchedule)
            {
                // we have to just schedule another instance of the same
                cDesc = getComponentDescriptor(configItem->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST, status);
                status = xme_core_exec_configurator_addComponentToSchedule
                (
                    trxnId, 
                    configItem->fDescriptor->componentId, configItem->fDescriptor->functionId, 
                    configItem->fDescriptor->taskArgs, 0, 0, 0, 0, NULL
                );
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    status,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );
                // We dont issue a createFunctionExecutionUnit
                configItem->totalToSchedule--;
            }
            configItem->toSchedule = false;
        }

    }
    XME_HAL_TABLE_ITERATE_END();
    
    XME_HAL_TABLE_ITERATE_BEGIN(marshaler_config_table, xme_hal_table_rowHandle_t, rh, marshaler_config_t, configItem);
    {
        if (true == configItem->toSchedule)
        {
            if (NULL == configItem->fDescriptor)
            {
                // We are here => there does not exists a waypoint instance for this topic
                // Hence create one and schedule it
                status = createMarshalerWaypointInstance(&(configItem->fDescriptor), (xme_core_component_t)componentID++);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                marshalerWaypointAddConfig
                (
                    (configItem->fDescriptor),
                    configItem->inputPort,
                    configItem->outputPort,
                    &configItem->instanceId,
                    configItem->topic,
                    configItem->topicSize
                );
                cDesc = getComponentDescriptor(configItem->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                        configItem->fDescriptor->componentId, configItem->fDescriptor->functionId, 
                                                                        (void *)(uintptr_t)(configItem->instanceId), 0, 0, 0, 0, NULL);
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    status,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );
                //We dont issue a createFunctionExecutionUnit for waypoints
                configItem->totalToSchedule--;
            }
            while (0!=configItem->totalToSchedule)
            {
                // we have to just schedule another instance of the same waypoint
                cDesc = getComponentDescriptor(configItem->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, status);
                status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                        configItem->fDescriptor->componentId, configItem->fDescriptor->functionId, 
                                                                        (void *)(uintptr_t)(configItem->instanceId), 0, 0, 0, 0, NULL);
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    status,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );
                //We dont issue a createFunctionExecutionUnit for waypoints
                configItem->totalToSchedule--;
            }
            configItem->toSchedule = false;
        }
    }
    XME_HAL_TABLE_ITERATE_END();
    
    XME_HAL_TABLE_ITERATE_BEGIN(udpSend_config_table, xme_hal_table_rowHandle_t, rh, udpSend_config_t, configItem);
    {
        if (true == configItem->toSchedule)
        {
            if (NULL == configItem->fDescriptor)
            {
                // We are here => there does not exists a waypoint instance for this topic
                // Hence create one and schedule it
                status = createUdpSendWaypointInstance(&(configItem->fDescriptor), componentID);
                componentID = (xme_core_component_t) (((xme_maxSystemValue_t)componentID) + 1);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                status = udpSendWaypointAddConfig((configItem->fDescriptor), configItem->inputPort, configItem->key, configItem->destIP, configItem->port, 
                                        configItem->topic, configItem->topicSize, &configItem->instanceId, configItem->buffer, false);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                cDesc = getComponentDescriptor(configItem->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, status);
                status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                        configItem->fDescriptor->componentId, configItem->fDescriptor->functionId, 
                                                                        (void *)(uintptr_t)(configItem->instanceId), 0, 0, 0, 0, NULL);
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    status,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );
                //We dont issue a createFunctionExecutionUnit for waypoints
                configItem->totalToSchedule--;
            }
            while (0 != configItem->totalToSchedule)
            {
                // we have to just schedule another instance of the same waypoint
                cDesc = getComponentDescriptor(configItem->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, status);
                status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                        configItem->fDescriptor->componentId, configItem->fDescriptor->functionId, 
                                                                        (void *)(uintptr_t)(configItem->instanceId), 0, 0, 0, 0, NULL);
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    status,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );
                // We dont issue a createFunctionExecutionUnit for waypoints
                configItem->totalToSchedule--;
            }
            configItem->toSchedule = false;
        }
    }
    XME_HAL_TABLE_ITERATE_END();
    // Now schedule the component port that is monitor but needs to be scheduled early
    // those component ports which are local and read from the publisher on the same node.
    XME_HAL_TABLE_ITERATE_BEGIN(componentPortVertex_config_table, xme_hal_table_rowHandle_t, rh, componentPortVertex_config_t, configItemComp);
    {
        while (true == configItemComp->toSchedule && XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION == configItemComp->portType && configItemComp->scheduleEarly != 0)
        {
            if (NULL == configItemComp->fDescriptor)
            {
                xme_core_dataManager_dataPacketId_t *temp = configItemComp->port;

                // FIXME: This is a temporary HACK to support plug and play for a selected set of components.
                //        This will be soon replaced by a manifest-based approach with symbol lookup.
                //        See Issue #3178.
                switch (configItemComp->componentType)
                {
                    /*case XME_CORE_COMPONENT_TYPE_MONITOR:
                        status = createMonitorInstance(configItemComp->componentId, &(configItemComp->fDescriptor), temp);
                        break;*/

                    case XME_CORE_COMPONENT_TYPE_SENSOR_B:
                        status = createMonitorBInstance(configItemComp->componentId, &(configItemComp->fDescriptor), temp);
                        break;

                    case XME_CORE_COMPONENT_TYPE_SENSOR_KB:
                        status = createMonitorKBInstance(configItemComp->componentId, &(configItemComp->fDescriptor), temp);
                        break;

                    case XME_CORE_COMPONENT_TYPE_SENSOR_MB:
                        status = createMonitorMBInstance(configItemComp->componentId, &(configItemComp->fDescriptor), temp);
                        break;

                    default:
                        status = XME_STATUS_INTERNAL_ERROR;
                        XME_LOG(XME_LOG_WARNING, "Attempt to instantiate unknown component type 0x%08X!", configItemComp->componentType);
                }

                XME_CHECK(XME_STATUS_SUCCESS == status, status);
           
                cDesc = getComponentDescriptor(configItemComp->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                      configItemComp->fDescriptor->componentId, configItemComp->fDescriptor->functionId, 
                                                                      configItemComp->fDescriptor->taskArgs, 0, 0, 0, 0, NULL);
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    status,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );
                xme_core_exec_dispatcher_createFunctionExecutionUnit(configItemComp->fDescriptor, true);
            }
            else
            {
                cDesc = getComponentDescriptor(configItemComp->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, status);
                status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                      configItemComp->fDescriptor->componentId, configItemComp->fDescriptor->functionId, 
                                                                      configItemComp->fDescriptor->taskArgs, 0, 0, 0, 0, NULL);
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    status,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );
            }
            configItemComp->totalToSchedule--;
            configItemComp->scheduleEarly--;
            if (0 == configItemComp->totalToSchedule)
            {
                configItemComp->toSchedule = false;
            }
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    // last we schedule those components which need the complete stack
    // here scheduling goes nested because we may need to schedule the complete stack muliple times
    XME_HAL_TABLE_ITERATE_BEGIN(udpRecv_config_table, xme_hal_table_rowHandle_t, rhUdpR, udpRecv_config_t, configItemUdpR);
    {
        uint8_t countDem;
        uint8_t countComp;
        while (true == configItemUdpR->toSchedule && 0!=configItemUdpR->totalToSchedule)
        {
            // components of same topic are to be scheduled together
            xme_core_topic_t topic= configItemUdpR->topic;
            if (NULL == configItemUdpR->fDescriptor)
            {
                // We are here => there does not exists a waypoint instance for this topic
                // Hence create one and schedule it
                status = createUdpReceiveWaypointInstance(&(configItemUdpR->fDescriptor), componentID);
                componentID = (xme_core_component_t) (((xme_maxSystemValue_t)componentID) + 1);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                status = udpReceiveWaypointAddConfig((configItemUdpR->fDescriptor), configItemUdpR->outputPort, configItemUdpR->key, configItemUdpR->port, 
                                        configItemUdpR->topic, configItemUdpR->topicSize, &configItemUdpR->instanceId, configItemUdpR->buffer);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                cDesc = getComponentDescriptor(configItemUdpR->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
                status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                        configItemUdpR->fDescriptor->componentId, configItemUdpR->fDescriptor->functionId, 
                                                                        (void *)(uintptr_t)(configItemUdpR->instanceId), 0, 0, 0, 0, NULL);
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    status,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );
                //We dont issue a createFunctionExecutionUnit for waypoints
            }
            else
            {
                // we have to just schedule another instance of the same waypoint
                cDesc = getComponentDescriptor(configItemUdpR->fDescriptor);
                status = xme_core_exec_componentRepository_registerComponent(cDesc);
                XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, status);
                status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                        configItemUdpR->fDescriptor->componentId, configItemUdpR->fDescriptor->functionId, 
                                                                        (void *)(uintptr_t)(configItemUdpR->instanceId), 0, 0, 0, 0, NULL);
                XME_CHECK_REC
                (
                    XME_STATUS_SUCCESS == status,
                    XME_STATUS_INTERNAL_ERROR,
                    {
                        xme_core_exec_configurator_rollback(trxnId);
                    }
                );
                //We dont issue a createFunctionExecutionUnit for waypoints
            }
            configItemUdpR->totalToSchedule--;
            //Now schedule the Demarshaler of the same topic
            XME_HAL_TABLE_ITERATE_BEGIN(demarshaler_config_table, xme_hal_table_rowHandle_t, rh, demarshaler_config_t, configItemDem);
            {
                if (true == configItemDem->toSchedule && configItemDem->topic == topic)
                {
                    if (NULL == configItemDem->fDescriptor)
                    {
                        // We are here => there does not exists a waypoint instance for this topic
                        // Hence create one and schedule it
                        status = createDemarshalerWaypointInstance(&(configItemDem->fDescriptor), componentID);
                        componentID = (xme_core_component_t) (((xme_maxSystemValue_t)componentID) + 1);
                        XME_CHECK(XME_STATUS_SUCCESS == status, status);
                        status = demarshalerWaypointAddConfig((configItemDem->fDescriptor), configItemDem->inputPort, configItemDem->outputPort, &configItemDem->instanceId, configItemDem->topic, configItemDem->topicSize);
                        XME_CHECK(XME_STATUS_SUCCESS == status, status);
                        cDesc = getComponentDescriptor(configItemDem->fDescriptor);
                        status = xme_core_exec_componentRepository_registerComponent(cDesc);
                        XME_CHECK(XME_STATUS_SUCCESS == status, status);
                        status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                        configItemDem->fDescriptor->componentId, configItemDem->fDescriptor->functionId, 
                                                                        (void *)(uintptr_t)(configItemDem->instanceId), 0, 0, 0, 0, NULL);
                        XME_CHECK_REC
                        (
                            XME_STATUS_SUCCESS == status,
                            status,
                            {
                                xme_core_exec_configurator_rollback(trxnId);
                            }
                        );
                        //We dont issue a createFunctionExecutionUnit for waypoints
                    }
                    else
                    {
                        // we have to just schedule another instance of the same waypoint
                        cDesc = getComponentDescriptor(configItemDem->fDescriptor);
                        status = xme_core_exec_componentRepository_registerComponent(cDesc);
                        XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, status);
                        status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                        configItemDem->fDescriptor->componentId, configItemDem->fDescriptor->functionId, 
                                                                        (void *)(uintptr_t)(configItemDem->instanceId), 0, 0, 0, 0, NULL);
                        XME_CHECK_REC
                        (
                            XME_STATUS_SUCCESS == status,
                            status,
                            {
                                xme_core_exec_configurator_rollback(trxnId);
                            }
                        );
                        //We dont issue a createFunctionExecutionUnit for waypoints
                    }
                    configItemDem->totalToSchedule--;
                    countDem = configItemDem->totalToSchedule;
                    if (0 == configItemDem->totalToSchedule)
                    {
                        configItemDem->toSchedule = false;
                    }
                    break;
                }
            }
            XME_HAL_TABLE_ITERATE_END();
            //Now schdule the component port that is monitor
            XME_HAL_TABLE_ITERATE_BEGIN(componentPortVertex_config_table, xme_hal_table_rowHandle_t, rh, componentPortVertex_config_t, configItemComp);
            {
                if (true == configItemComp->toSchedule && XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION == configItemComp->portType && configItemComp->topic == topic)
                {
                    if (NULL == configItemComp->fDescriptor)
                    {
                        xme_core_dataManager_dataPacketId_t *temp = configItemComp->port;

                        // FIXME: This is a temporary HACK to support plug and play for a selected set of components.
                        //        This will be soon replaced by a manifest-based approach with symbol lookup.
                        //        See Issue #3178.
                        switch (configItemComp->componentType)
                        {
                            /*case XME_CORE_COMPONENT_TYPE_MONITOR:
                                status = createMonitorInstance(configItemComp->componentId, &(configItemComp->fDescriptor), temp);
                                break;*/

                            case XME_CORE_COMPONENT_TYPE_MONITOR_B:
                                status = createMonitorBInstance(configItemComp->componentId, &(configItemComp->fDescriptor), temp);
                                break;

                            case XME_CORE_COMPONENT_TYPE_MONITOR_KB:
                                status = createMonitorKBInstance(configItemComp->componentId, &(configItemComp->fDescriptor), temp);
                                break;

                            case XME_CORE_COMPONENT_TYPE_MONITOR_MB:
                                status = createMonitorMBInstance(configItemComp->componentId, &(configItemComp->fDescriptor), temp);
                                break;

                            default:
                                status = XME_STATUS_INTERNAL_ERROR;
                                XME_LOG(XME_LOG_WARNING, "Attempt to instantiate unknown component type 0x%08X!", configItemComp->componentType);
                        }

                        XME_CHECK(XME_STATUS_SUCCESS == status, status);
           
                        cDesc = getComponentDescriptor(configItemComp->fDescriptor);
                        status = xme_core_exec_componentRepository_registerComponent(cDesc);
                        XME_CHECK(XME_STATUS_SUCCESS == status, status);
                        status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                      configItemComp->fDescriptor->componentId, configItemComp->fDescriptor->functionId, 
                                                                      configItemComp->fDescriptor->taskArgs, 0, 0, 0, 0, NULL);
                        XME_CHECK_REC
                        (
                            XME_STATUS_SUCCESS == status,
                            status,
                            {
                                xme_core_exec_configurator_rollback(trxnId);
                            }
                        );
                        xme_core_exec_dispatcher_createFunctionExecutionUnit(configItemComp->fDescriptor, true);
                    }
                    else
                    {
                        cDesc = getComponentDescriptor(configItemComp->fDescriptor);
                        status = xme_core_exec_componentRepository_registerComponent(cDesc);
                        XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST, status);
                        status = xme_core_exec_configurator_addComponentToSchedule(trxnId, 
                                                                      configItemComp->fDescriptor->componentId, configItemComp->fDescriptor->functionId, 
                                                                      configItemComp->fDescriptor->taskArgs, 0, 0, 0, 0, NULL);
                        XME_CHECK_REC
                        (
                            XME_STATUS_SUCCESS == status,
                            status,
                            {
                                xme_core_exec_configurator_rollback(trxnId);
                            }
                        );
                    }
                    configItemComp->totalToSchedule--;
                    countComp = configItemComp->totalToSchedule;
                    if (0 == configItemComp->totalToSchedule)
                    {
                        configItemComp->toSchedule = false;
                    }
                    break;
                }
            }
            XME_HAL_TABLE_ITERATE_END();
            //After scheduling each instance once, we must have similar remaining counts to be scheduled
            XME_ASSERT(configItemUdpR->totalToSchedule == countDem && countDem == countComp);
            if (0 == configItemUdpR->totalToSchedule)
            {
                configItemUdpR->toSchedule = false;
            }
        }
    }
    XME_HAL_TABLE_ITERATE_END();
    status = xme_core_exec_configurator_commit(trxnId);

    return status;
}

static xme_core_exec_componentDescriptor_t *
getComponentDescriptor(xme_core_exec_functionDescriptor_t *fDesc)
{
    xme_core_exec_componentDescriptor_t *temp = (xme_core_exec_componentDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_componentDescriptor_t));
    if (NULL == temp)
    {
        return NULL;
    }
    temp->componentId = fDesc->componentId;
    temp->init = (void *) 0;
    temp->fini = (void *) 0;
    temp->autoInit = false;
    temp->initParam = (void *) 0;
    temp->initWcet_ns = (xme_hal_time_timeInterval_t)0;
    XME_HAL_SINGLYLINKEDLIST_INIT(temp->functions);
    (void) xme_hal_singlyLinkedList_addItem( (void*) &(temp->functions), fDesc );
    return temp;
}

xme_status_t
xme_core_pnp_pnpClient_init(void)
{
    XME_HAL_TABLE_INIT(componentPortVertex_config_table);
    XME_HAL_TABLE_INIT(marshaler_config_table);
    XME_HAL_TABLE_INIT(demarshaler_config_table);
    XME_HAL_TABLE_INIT(udpSend_config_table);
    XME_HAL_TABLE_INIT(udpRecv_config_table);
    XME_HAL_TABLE_INIT(portMapping_table);
    
    return XME_STATUS_SUCCESS;
}

void
xme_core_pnp_pnpClient_fini(void)
{
    xme_core_exec_fini();
    
    XME_HAL_TABLE_ITERATE_BEGIN(componentPortVertex_config_table, xme_hal_table_rowHandle_t, rh, componentPortVertex_config_t, configItem);
    {
        xme_hal_mem_free(configItem->fDescriptor);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(componentPortVertex_config_table);
    
    XME_HAL_TABLE_ITERATE_BEGIN(marshaler_config_table, xme_hal_table_rowHandle_t, rh, marshaler_config_t, configItem);
    {
        xme_hal_mem_free(configItem->fDescriptor);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(marshaler_config_table);
    
    XME_HAL_TABLE_ITERATE_BEGIN(demarshaler_config_table, xme_hal_table_rowHandle_t, rh, demarshaler_config_t, configItem);
    {
        xme_hal_mem_free(configItem->fDescriptor);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(demarshaler_config_table);
    
    XME_HAL_TABLE_ITERATE_BEGIN(udpSend_config_table, xme_hal_table_rowHandle_t, rh, udpSend_config_t, configItem);
    {
        xme_hal_mem_free(configItem->fDescriptor);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(udpSend_config_table);
    
    XME_HAL_TABLE_ITERATE_BEGIN(udpRecv_config_table, xme_hal_table_rowHandle_t, rh, udpRecv_config_t, configItem);
    {
        xme_hal_mem_free(configItem->fDescriptor);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(udpRecv_config_table);
}

xme_status_t
xme_core_pnp_pnpClient_announceStaticComponentPort
(
    xme_core_component_portType_t portType,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_core_dataManager_dataPacketId_t *port,
    xme_core_componentType_t componentType,
    xme_core_component_t componentId,
    xme_core_exec_functionDescriptor_t *fDescriptor
)
{
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    componentPortVertex_config_t* configItem;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(componentPortVertex_config_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    componentPortVertex_config_t, configItem,
                    (configItem->componentId == componentId &&
                    *(configItem->port) == *port));
                    //configItem->portType == portType && configItem->topic == topic));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(componentPortVertex_config_table);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != configItemHandle, XME_STATUS_OUT_OF_RESOURCES);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(componentPortVertex_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->port = port;
            configItem->portType = portType;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->fDescriptor = fDescriptor;
            configItem->componentType = componentType;
            configItem->componentId = componentId;
            configItem->totalToSchedule = 0;
            configItem->toSchedule = false;
            configItem->scheduleEarly = 0;
        }
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpClient_announceStaticMarshaler
(
    xme_core_dataManager_dataPacketId_t *inputPort,
    xme_core_dataManager_dataPacketId_t *outputPort,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_exec_functionDescriptor_t *fDescriptor
)
{
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    marshaler_config_t* configItem;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(marshaler_config_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    marshaler_config_t, configItem,
                    (configItem->topic == topic));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(marshaler_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(marshaler_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->outputPort = outputPort;
            configItem->inputPort = inputPort;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->instanceId = instanceId;
            configItem->fDescriptor = fDescriptor;
            configItem->totalToSchedule = 0;
            configItem->toSchedule = false;
        }
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpClient_announceStaticDemarshaler
(
    xme_core_dataManager_dataPacketId_t *inputPort,
    xme_core_dataManager_dataPacketId_t *outputPort,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_exec_functionDescriptor_t *fDescriptor
)
{
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    demarshaler_config_t* configItem;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(demarshaler_config_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    demarshaler_config_t, configItem,
                    (configItem->topic == topic));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(demarshaler_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(demarshaler_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->outputPort = outputPort;
            configItem->inputPort = inputPort;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->instanceId = instanceId;
            configItem->fDescriptor = fDescriptor;
            configItem->totalToSchedule = 0;
            configItem->toSchedule = false;
        }
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpClient_announceStaticUDPSend
(
    xme_core_dataManager_dataPacketId_t *dataPort,
    const uint8_t *key,
    const char *destIP,
    uint16_t ipPort,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_exec_functionDescriptor_t *fDescriptor,
    void *buffer
)
{
    udpSend_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(udpSend_config_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    udpSend_config_t, configItem,
                    ((xme_hal_mem_compare(configItem->destIP,destIP,strlen(destIP)) == 0) && configItem->port == ipPort && xme_hal_mem_compare(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH) == 0));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(udpSend_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(udpSend_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->inputPort = dataPort;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            xme_hal_mem_copy(configItem->destIP,destIP,strlen(destIP));
            xme_hal_mem_copy(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH);
            configItem->port = ipPort;
            configItem->instanceId = instanceId;
            configItem->fDescriptor = fDescriptor;
            configItem->buffer = buffer;
            configItem->totalToSchedule = 0;
            configItem->toSchedule = false;
        }
    }
  
    return XME_STATUS_SUCCESS;
}    

xme_status_t
xme_core_pnp_pnpClient_announceStaticUDPReceive
(
    xme_core_dataManager_dataPacketId_t *dataPort,
    const uint8_t *key,
    uint16_t ipPort,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_exec_functionDescriptor_t *fDescriptor,
    void *buffer
)
{
    udpRecv_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(udpRecv_config_table, 
                    xme_hal_table_rowHandle_t, configItemHandle,
                    udpRecv_config_t, configItem,
                    ( configItem->port == ipPort && xme_hal_mem_compare(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH) == 0));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(udpRecv_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(udpRecv_config_table, configItemHandle);
        configItem->outputPort = dataPort;
        configItem->topic = topic;
        configItem->topicSize = topicSize;
        xme_hal_mem_copy(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH);
        configItem->port = ipPort;
        configItem->instanceId = instanceId;
        configItem->fDescriptor = fDescriptor;
        configItem->buffer = buffer;
        configItem->totalToSchedule = 0;
        configItem->toSchedule = false;
    }
  
    return XME_STATUS_SUCCESS;
}    

xme_status_t
xme_core_pnp_pnpClient_processGraph
(
    const xme_core_topic_pnpManager_runtime_graph_model_t* portPnpGraphInData
)
{
    int i;
    xme_status_t status;

    XME_CHECK(NULL != portPnpGraphInData, XME_STATUS_INVALID_PARAMETER);

    // TODO: Remove when Issue #2972 is fixed!
    XME_CHECK
    (
        XME_CORE_NODE_INVALID_NODE_ID != portPnpGraphInData->nodeId &&
        XME_CORE_NODE_INVALID_NODE_ID != xme_core_node_getCurrentNodeId(), 
        XME_STATUS_INVALID_CONFIGURATION
    );

    // Since we send same graph to all the nodes. The node receiving other nodes graph should silently drop it
    // This is not an error so we should return XME_STATUS_SUCCESS
    XME_CHECK_MSG
    (
        xme_core_node_getCurrentNodeId() == (xme_core_node_nodeId_t) portPnpGraphInData->nodeId,
        XME_STATUS_SUCCESS,
        XME_LOG_DEBUG,
        "Plug and Play Client on node %" PRIu16 " received plug and play graph for node %" PRIu16 "! It will be ignored.\n",
        xme_core_node_getCurrentNodeId(),
        portPnpGraphInData->nodeId
    );

    XME_LOG(XME_LOG_NOTE, "Received plug and play graph!\n");

    for (i=0; i<XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH; i++)
    { //for all the possible edges
        uint8_t srcI;
        uint8_t dstI;
        xme_core_pnp_dataLinkGraph_edgeTypes_t edgeType;

        srcI = portPnpGraphInData->edge[i].srcVertexIndex;
        dstI = portPnpGraphInData->edge[i].sinkVertexIndex;
        //Check for valid range which is 1..10
        srcI--;
        dstI--;
        edgeType = portPnpGraphInData->edge[i].edgeType;
        
        if (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_MEMCOPY == edgeType)
        {
            xme_core_pnp_dataLinkGraph_vertexTypes_t srcVType;
            xme_core_pnp_dataLinkGraph_vertexTypes_t dstVType;
            xme_core_dataManager_dataPacketId_t* srcPort;
            xme_core_dataManager_dataPacketId_t* dstPort;
            bool early = false;

            srcVType = portPnpGraphInData->vertex[srcI].vertexType;
            dstVType = portPnpGraphInData->vertex[dstI].vertexType;
            srcPort = NULL;
            dstPort = NULL;

            if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT)
            {
                srcPort = addConfigComponentPortVertex(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData,
                                                        portPnpGraphInData->vertex[srcI].componentType,
                                                        portPnpGraphInData->vertex[srcI].componentId,
                                                        false);
                early = true;
            }
            else if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER)
            {
                srcPort = addConfigMarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE)
            {
                srcPort = addConfigUDPRecv(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER)
            {
                srcPort = addConfigDemarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            
            if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER)
            {
                dstPort = addConfigMarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND)
            {
                dstPort = addConfigUDPSend(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER)
            {
                dstPort = addConfigDemarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT)
            {
                dstPort = addConfigComponentPortVertex(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData,
                                                        portPnpGraphInData->vertex[dstI].componentType,
                                                        portPnpGraphInData->vertex[dstI].componentId,
                                                        early);
            }
            createPortMappingArray(srcPort, dstPort);
        }
    }//for all the possible edges
    
    status = initComponents();
    XME_LOG(XME_LOG_DEBUG,"PnpClient: initComponents returned %d\n",status);
    status = addPortMappingToBroker();        
    XME_LOG(XME_LOG_DEBUG,"PnpClient: addPortMappingToBroker returned %d\n",status);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpClient_getManifest
(
    xme_core_topic_pnp_componentInstanceManifest_t* manifest
)
{
    xme_core_manifestRepository_iterator_t iter;
    uint16_t maxCount = sizeof(manifest->components) / sizeof(manifest->components[0]);
    uint16_t index = 0;

    manifest->nodeId = xme_core_node_getCurrentNodeId();
    XME_LOG(XME_LOG_NOTE, "[PlugAndPlayClient] Sending Component Manifest\n");

    iter = xme_core_manifestRepository_initIterator();
    while (xme_core_manifestRepository_hasNext(iter) && index < maxCount)
    {
        xme_core_componentManifest_t* current = xme_core_manifestRepository_next(&iter);
        XME_ASSERT(NULL != manifest);

        // Initially there is component ready so componentID is unknown
        manifest->components[index].componentId = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT;
        manifest->components[index].componentType = current->componentType;

        index++;
    }

    // Issue a warning if we overflow
    // TODO: Exchange XME_STATUS_OUT_OF_RESOURCES with new XME_STATUS_INCOMPLETE
    XME_CHECK_MSG(index < maxCount, XME_STATUS_OUT_OF_RESOURCES, XME_LOG_WARNING, "Too many manifests for collecting!\n");

    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
