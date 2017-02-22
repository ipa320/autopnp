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
 * $Id: plugAndPlayClientScheduling.c 7839 2014-03-14 12:56:40Z wiesmueller $
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
#include "xme/core/plugAndPlay/include/plugAndPlayClientScheduling.h"
#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"
#include "xme/core/plugAndPlay/include/plugAndPlayClientConfiguration.h"
#include "xme/core/plugAndPlay/include/plugAndPlayClientInternalTypes.h"

#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"

#include "xme/core/directory/include/topicRegistry.h"

#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/executionManager/include/internDescriptorTable.h"

#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpClientInterface.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \brief Used in createComponentInstance and associated functions to capture
 *        all information that is necessary for recover in case of an error.
 */
typedef struct
{
    bool isCompCpmRegistered; ///< Whether the component is already registered in the component port manager.
    bool isConfigAlloc; ///< Whether the component coniguration parameter has already been allocated.
    uint8_t funcDescAllocCount; ///< Number of function descriptors that have already been allocated.
    uint8_t funcBrokerRegCount; ///< Number of functions that have already been registered in the broker.
} recoverCreateComponentInstance_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \brief Table to store the local copy of schedule.
 */
static XME_HAL_TABLE(xme_core_exec_schedule_table_entry_t, localCopy, 10);

/**
 * \brief Variable to store the cycle length of the read schedule.
 */
static xme_hal_time_timeInterval_t majorCycleDuration_ns;

/**
 * \brief Variable to store the start of next cycle for the new entry to be
 *        added to the schedule.
 */
static xme_hal_time_timeInterval_t nextCycle_ns;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief This function helps in initializing the newly added component port vertices.
 *
 * \param configItem Pointer to the config Item entry which needs to be scheduled.
 *
 * \retval XME_STATUS_SUCCESS if the components were initialized successfully
 * \retval other status value returned by the failing component
 */
static xme_status_t
initComponentPortVertexHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_CPV_config_t *configItem
);

/**
 * \brief This function helps in initializing the newly added Channel Injector Waypoint
 *
 * \param configItem Pointer to the config Item entry which needs to be scheduled.
 *
 * \retval XME_STATUS_SUCCESS if the components were initialized successfully
 * \retval other status value returned by the failing component
 */
static xme_status_t
initChannelInjectorHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_channelInjector_config_t *configItem
);

/**
 * \brief This function helps in initializing the newly added Channel Selector Waypoint
 *
 * \param configItem Pointer to the config Item entry which needs to be scheduled.
 *
 * \retval XME_STATUS_SUCCESS if the components were initialized successfully
 * \retval other status value returned by the failing component
 */
static xme_status_t
initChannelSelectorHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_channelSelector_config_t *configItem
);

/**
 * \brief This function helps in initializing the newly added UDP Send Waypoint
 *
 * \param configItem Pointer to the config Item entry which needs to be scheduled.
 *
 * \retval XME_STATUS_SUCCESS if the components were initialized successfully
 * \retval other status value returned by the failing component
 */
static xme_status_t
initudpSendHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_udpSend_config_t *configItem
);

/**
 * \brief This function helps in initializing the newly added UDP Receive Waypoint
 *
 * \param configItem Pointer to the config Item entry which needs to be scheduled.
 *
 * \retval XME_STATUS_SUCCESS if the components were initialized successfully
 * \retval other status value returned by the failing component
 */
static xme_status_t
initudpRecvHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_udpRecv_config_t *configItem
);

/**
 * \brief This function helps in initializing the newly added marshaler Waypoint
 *
 * \param configItem Pointer to the config Item entry which needs to be scheduled.
 *
 * \retval XME_STATUS_SUCCESS if the components were initialized successfully
 * \retval other status value returned by the failing component
 */
static xme_status_t
initMarshalerHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_marshaler_config_t *configItem
);

/**
 * \brief This function helps in initializing the newly added demarshaler Waypoint
 *
 * \param configItem Pointer to the config Item entry which needs to be scheduled.
 *
 * \retval XME_STATUS_SUCCESS if the components were initialized successfully
 * \retval other status value returned by the failing component
 */
static xme_status_t
initDemarshalerHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_demarshaler_config_t *configItem
);

#if 0
/**
 * \brief Initializes a component descriptor. This is required for adding schedule to 
 *        Execution manager for the newly created component/waypoint.
 *
 * \param fDesc The function descriptor which needs to be inserted in the schedule.
 *
 * \return Pointer to the newly created component descriptor.
 */
static xme_core_exec_componentDescriptor_t*
getComponentDescriptor
(
    const xme_core_exec_functionDescriptor_t* const fDesc
);
#endif

/**
 * \brief Create demarshaler waypoint instance.
 *        This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param descriptor Will be set to a pointer to the allocation function descriptor.
 * \param componentId Component id that will be used in the function descriptor.
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
 * \brief Create demarshaler waypoint instance.
 * \details This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param[in] descriptor Will be set to a pointer to the allocation function descriptor.
 * \param[in] inputDataPort The input port data. 
 * \param[in] inputPortQueueSize The input port queue size. 
 * \param[in] outputDataPort The output port data. 
 * \param[in] instanceId The instance identifier.
 * \param[in] topic The topic identifier. 
 * \param[in] topicSize The topic size expressed in bytes. 
 * \param[in] channelID The channel identifier. 
 *
 * \retval XME_STATUS_SUCCESS When initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES Initialization failure due to insufficient resources.
 */
extern xme_status_t
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

/**
 * \brief Remove config entries for the demarshaler waypoint
 *        This function is present in main node file.
 *
 * \param[in] descriptor The pointer to waypoint descriptor to be removed.
 * \param[in] inputPort The pointer to input data port.
 * \param[in] outputPort The pointer to output data port.
 * \param[in] instanceId The instance identifier. 
 *
 * \retval XME_STATUS_SUCCESS If configuration was successfuly added
 * \retval XME_STATUS_INTERNAL_ERROR If there was an error
 */
extern xme_status_t
demarshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_wp_waypoint_instanceId_t instanceId
);

/**
 * \brief Create marshaler waypoint instance.
 *        This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param[in] descriptor Will be set to a pointer to the allocation function descriptor.
 * \param[in] componentId Component id that will be used in the function descriptor.
 *
 * \retval XME_STATUS_SUCCESS When initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES Initialization failure due to insufficient resources.
 */
extern xme_status_t
createMarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

/**
 * \brief Create marshaler waypoint instance.
 * \details This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param[in] descriptor Will be set to a pointer to the allocation function descriptor.
 * \param[in] inputDataPort The input port data. 
 * \param[in] inputPortQueueSize The input port queue size. 
 * \param[in] outputDataPort The output port data. 
 * \param[in] instanceId The instance identifier.
 * \param[in] topic The topic identifier. 
 * \param[in] topicSize The topic size expressed in bytes. 
 * \param[in] channelID The channel identifier. 
 *
 * \retval XME_STATUS_SUCCESS When initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES Initialization failure due to insufficient resources.
 */
extern xme_status_t
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

/**
 * \brief Remove config entries for the marshaler waypoint
 *        This function is present in main node file.
 *
 * \param[in] descriptor Will be set to a pointer to the allocation function descriptor.
 * \param[in] inputPort The pointer to input data port.
 * \param[in] outputPort The pointer to output data port.
 * \param[in] instanceId The pointer to waypoint instance Id to be removed.
 *
 * \retval XME_STATUS_SUCCESS If configuration was successfuly added
 * \retval XME_STATUS_INTERNAL_ERROR If there was an error
 */
extern xme_status_t
marshalerWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_wp_waypoint_instanceId_t instanceId
);

/**
 * \brief Create UDP Receieve waypoint instance.
 *        This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param descriptor Will be set to a pointer to the allocation function descriptor.
 * \param componentId Component id that will be used in the function descriptor.
 *
 * \retval XME_STATUS_SUCCESS When initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES Initialization failure due to insufficient resources.
 */
extern xme_status_t 
createUdpReceiveWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

/**
 * \brief Adds config entries for the udpReceive waypoint
 *        This function is present in main node file.
 *
 * \param descriptor The pointer to function descriptor obtained by corresponding Init function
 * \param dataPort The pointer to data port registered by the component
 * \param key The key associated to the UDP.
 * \param ipPort The IP port used for configuring the UDP receive.
 * \param topic Topic for which this configuration is added for this waypoint.
 * \param topicSize Size of the topic.
 * \param instanceId The pointer to waypoint instance Id filled by the function.
 * \param buffer The buffer used in the configuration of UDP receive waypoint. 
 *
 * \retval XME_STATUS_SUCCESS If configuration was successfuly added
 * \retval XME_STATUS_INTERNAL_ERROR If there was an error
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
    void** buffer
);

/**
 * \brief Removes config entries for the udpReceive waypoint
 *        This function is present in main node file.
 *
 * \param[in] descriptor The pointer to waypoint descriptor to be removed.
 * \param[in] outputPort The pointer to data port registered by the component.
 * \param[in] instanceId The instance identifier. 
 * \param[out] buffer The buffer used in the configuration of UDP receive waypoint. 
 *
 * \retval XME_STATUS_SUCCESS If configuration was successfuly removed.
 * \retval XME_STATUS_INTERNAL_ERROR If there was an error
 */
extern xme_status_t 
udpReceiveWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_wp_waypoint_instanceId_t instanceId,
    void* buffer
);

/**
 * \brief Create udp send waypoint instance.
 *        This function is present in main node file.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param descriptor Will be set to a pointer to the allocation function descriptor.
 * \param componentId Component id that will be used in the function descriptor.
 *
 * \retval XME_STATUS_SUCCESS When initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES Initialization failure due to insufficient resources.
 */
extern xme_status_t
createUdpSendWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

/**
 * \brief Adds config entries for the udpSend waypoint
 *        This function is present in main node file.
 *
 * \param descriptor The pointer to function descriptor obtained by corresponding Init function
 * \param dataPort The pointer to data port registered by the component
 * \param key The key associated to the UDP.
 * \param destIP The target IP address used for configuring the UDP send.
 * \param ipPort The IP port used for configuring the UDP receive.
 * \param topic Topic for which this configuration is added for this waypoint.
 * \param topicSize Size of the topic.
 * \param instanceId The pointer to waypoint instance Id filled by the function.
 * \param buffer The buffer used in the configuration of UDP receive waypoint. 
 * \param isBroadcast If this configuration is to be used for broadcast.
 *
 * \retval XME_STATUS_SUCCESS If configuration was successfuly added
 * \retval XME_STATUS_INTERNAL_ERROR If there was an error
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
    void** buffer,
    bool isBroadcast
);

/**
 * \brief Removes config entries for the udpSend waypoint
 *        This function is present in main node file.
 *
 * \param[in] descriptor The pointer to waypoint descriptor to be removed.
 * \param[in] inputPort The pointer to data port registered by the component.
 * \param[in] instanceId The instance identifier. 
 * \param buffer The buffer used in the configuration of UDP receive waypoint. 
 *
 * \retval XME_STATUS_SUCCESS If configuration was successfuly removed.
 * \retval XME_STATUS_INTERNAL_ERROR If there was an error.
 */
extern xme_status_t 
udpSendWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_wp_waypoint_instanceId_t instanceId,
    void* buffer
);

/**
 * \brief Create channel selector waypoint instance.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param descriptor Will be set to a pointer to the allocation function descriptor.
 * \param componentId Component id that will be used in the function descriptor.
 *
 * \retval XME_STATUS_SUCCESS When initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES Initialization failure due to insufficient resources.
 */
extern xme_status_t
createChannelSelectorWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

// Defined in generated node file.
extern xme_status_t
channelSelectorWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    uint8_t inputPortQueueSize,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_core_channelId_t sourceChannelID,
    xme_core_channelId_t destinationChannelID,
    void** recvBuffer
);

/**
 * \brief Removes a configuration entry of the channel selector waypoint.
 *        Implemented in the main node file.
 *
 * \param descriptor The function descriptor to be removed. 
 * \param instanceId Pointer to instanceId of the config entry to be removed.
 * \param inputPort Pointer to input data packet Id of that waypoint.
 * \param outputPort Pointer to output data packet Id of that waypoint.
 * \param topic Topic for this configuration.
 * \param srcChID The source channel identificator. 
 * \param dstChID The destination channel identificator. 
 * \param buffer Pointer the buffer used by waypoint configuration for data
 *
 * \retval XME_STATUS_SUCCESS When no errors occurred.
 * \retval XME_STATUS_INTERNAL_ERROR When an error occured.
 */
extern xme_status_t
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

/**
 * \brief Create channel injector waypoint instance.
 *
 * \details Allocates and populates given function descriptor and calls init function of waypoint.
 *          All calls to this function after the first one will do nothing except for setting
 *          the descriptor to the previously allocated one.
 *
 * \param descriptor Will be set to a pointer to the allocation function descriptor.
 * \param componentId Component id that will be used in the function descriptor.
 *
 * \retval XME_STATUS_SUCCESS When initialization was succesful.
 * \retval XME_STATUS_OUT_OF_RESOURCES Initialization failure due to insufficient resources.
 */
extern xme_status_t
createChannelInjectorWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

// Defined in generated node file.
extern xme_status_t
channelInjectorWaypointAddConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_core_dataManager_dataPacketId_t* inputPort,
    uint8_t inputPortQueueSize,
    xme_core_dataManager_dataPacketId_t* outputPort,
    xme_core_channelId_t injectedChannelID,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    xme_wp_waypoint_instanceId_t* instanceId,
    void** recvBuffer
);

/**
 * \brief Removes a configuration entry of the channel injector waypoint.
 *        Implemented in the main node file.
 *
 * \param descriptor Pointer to descriptor entry to be removed. 
 * \param instanceId Pointer to instanceId of the config entry to be removed.
 * \param inputPort Pointer to input data packet Id of that waypoint.
 * \param outputPort Pointer to output data packet Id of that waypoint.
 * \param buffer Pointer the buffer used by waypoint configuration for data
 *
 * \retval XME_STATUS_SUCCESS When no errors occurred.
 * \retval XME_STATUS_INTERNAL_ERROR When an error occured.
 */
extern xme_status_t
channelInjectorWaypointRemoveConfig
(
    xme_core_exec_functionDescriptor_t* descriptor,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_dataManager_dataPacketId_t* inputPort,
    xme_core_dataManager_dataPacketId_t* outputPort,
    void* buffer
);

/**
 * \brief Replaces the current schedule in the Execution Manager with the newly constructed schedule
 *
 * \retval XME_STATUS_SUCCESS When no errors occurred.
 * \retval XME_STATUS_INTERNAL_ERROR When an error occured.
 */
static xme_status_t
updateScheduleInTheExecutionManager(void);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

static xme_status_t
initComponentPortVertexHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_CPV_config_t *configItem
)
{
    xme_status_t status;
    uint16_t j = 0;
    xme_hal_table_rowHandle_t localCopyRH = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_exec_schedule_table_entry_t *tableEntry = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = configItem->componentHandle;

    //Check if it already exists in the existing schedule
    XME_HAL_TABLE_GET_NEXT(localCopy, 
        xme_hal_table_rowHandle_t, localCopyRH,
        xme_core_exec_schedule_table_entry_t, tableEntry,
        (tableEntry->componentId == configItem->componentId));
    
    if (NULL != tableEntry)
    {
        //It is already scheduled
        //Check for the periodDivider
        uint16_t i = 0;
        uint16_t counter = 0;
        for (i = 0; i < configItem->countfDescriptor; i++)
        {
            if (configItem->fDescriptor[i]->functionId == tableEntry->functionId)
            {
                counter++;
            }
        }
        if (counter == configItem->countfDescriptor)
        {
            //found the functionIDs in the current schedule entry
            //so no need to look further
            configItem->totalToSchedule--;
            return XME_STATUS_SUCCESS;
        }
        //other wise we need to see if there are some other entries in the schedule
        //which have the same componentID. 
        //The point here is each port can have dependent multiple functions 
        //and we manager our data structures port basis so functions listing 
        //can be split based on the port they are associated with
        XME_HAL_TABLE_GET_NEXT(localCopy, 
                               xme_hal_table_rowHandle_t, localCopyRH,
                               xme_core_exec_schedule_table_entry_t, tableEntry,
                               (tableEntry->componentId == configItem->componentId));
    }

    //We are here it means it does not exist in the existing schedule
    if (0 == configItem->countfDescriptor)
    {
        uint16_t i = 0u;
        uint16_t k = 0u;

        xme_core_componentManifest_t componentManifest;

        status = xme_core_manifestRepository_findComponentManifest(configItem->componentType, &componentManifest);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        {
            xme_core_nodeMgr_compRep_state_t state = xme_core_nodeMgr_compRep_getState(configItem->componentHandle);
            
            if (XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED == state)
            {
                // The node already knows this component, which means either:
                // this node sent an instance manifest for it,
                // or this component was plugged in on this node via xme_core_pnp_pnpManager_plugInNewComponent()
                if (XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT == xme_core_nodeMgr_compRep_getComponentID(componentHandle))
                {
                    uint16_t portDataI = 0u;

                    xme_core_nodeMgr_compRep_setComponentID(componentHandle, configItem->componentId);

                    // Manager might have adjusted the queue sizes
                    for (portDataI = 0u; portDataI < sizeof(configItem->portData) / sizeof(configItem->portData[0]); portDataI++)
                    {
                        xme_core_nodeMgr_compRep_portHandle_t portHandle;

                        if (0 == configItem->portData[portDataI].queueSize) { continue; }

                        portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, portDataI);

                        xme_core_nodeMgr_compRep_setQueueSize(portHandle, configItem->portData[portDataI].queueSize);
                    }
                }
                else
                {
                    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(configItem->componentId == xme_core_nodeMgr_compRep_getComponentID(componentHandle)));
                }
            }
            else if (XME_CORE_NODEMGR_COMPREP_STATE_INVALID == state)
            {
                // This node did not request the node and we must create a new component instance entry for it
                xme_core_nodeMgr_compRep_componentBuilder_t* builder = 
                xme_core_nodeMgr_compRep_createBuilder
                (
                    xme_core_node_getCurrentNodeId(),
                    configItem->componentType
                );
                XME_CHECK(NULL != builder, XME_STATUS_OUT_OF_RESOURCES);
                xme_core_nodeMgr_compRep_builderSetComponentID(builder, configItem->componentId);
                xme_core_nodeMgr_compRep_builderSetInitializationString(builder, configItem->initializationString);
            
                {
                    uint16_t portIndex = 0u;
                    uint16_t portCount = xme_core_manifestRepository_getPortCount(&componentManifest);

                    while (portIndex < portCount)
                    {
                        xme_core_nodeMgr_compRep_builderSetQueueSize
                        (
                            builder,
                            portIndex,
                            configItem->portData[portIndex].queueSize
                        );

                        portIndex++;
                    }
                }

                {
                    uint16_t functionIndex = 0u;
                    uint16_t functionCount = xme_core_manifestRepository_getFunctionCount(&componentManifest);

                    while (functionIndex < functionCount)
                    {
                        xme_core_nodeMgr_compRep_builderSetExecutionPeriod
                        (
                            builder,
                            functionIndex,
                            configItem->functionData[functionIndex].executionPeriod
                        );

                        functionIndex++;
                    }
                }
            
                status = xme_core_nodeMgr_compRep_build(builder, &componentHandle);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);

                status = xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle);
                XME_CHECK(XME_STATUS_SUCCESS == status, status);
            }
        }

        // When component is not created and registered, then do so
        if (XME_CORE_NODEMGR_COMPREP_STATE_CREATED > xme_core_nodeMgr_compRep_getState(componentHandle))
        {
            status = xme_core_nodeMgr_compRep_createAndRegisterComponent
            (
                componentHandle
            );
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
        }

        //After the call to the createAndRegisterComponent all the ports were initialized.
        //Here point is we keep track of only one port entry under one config entry
        //So we should assign all the other ports to the other config entries
        for (i = 0U; i < xme_core_manifestRepository_getPortCount(&componentManifest); i++)
        {   // for each port
            // Here i not only is an index into array but also is the same value
            // as the port array of the component so this means it is one to one mapping.
            xme_core_topic_t topic = componentManifest.portManifests[i].topic;
            xme_core_component_portType_t portType = componentManifest.portManifests[i].portType;
            xme_hal_table_rowHandle_t tempConfigItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            xme_core_pnp_pnpClientConfiguration_CPV_config_t* tempConfigItem = NULL;

            //Find the other entry with topic and portType and componentType
            XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_CPV_config_table, 
                xme_hal_table_rowHandle_t, tempConfigItemHandle,
                xme_core_pnp_pnpClientConfiguration_CPV_config_t, tempConfigItem,
                (
                    tempConfigItem->componentType == componentManifest.componentType &&
                    tempConfigItem->portType == portType &&
                    tempConfigItem->topic == topic &&
                    tempConfigItem->componentId == configItem->componentId
                    // TODO: What about components with two different ports with the same topic and type? We need to also store the port index in the config item.
                )
            );
            //it does not exist, this can happen when a port a component has a graph for one of the port
            //and others are not being used currently.
            //So we add the configEntry for we do not wish to loose the port and set the totalToSchedule to zero
            if (XME_HAL_TABLE_INVALID_ROW_HANDLE == tempConfigItemHandle)
            {
                tempConfigItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_CPV_config_table);
                tempConfigItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_CPV_config_table, tempConfigItemHandle);
                XME_CHECK(NULL != tempConfigItem, XME_STATUS_OUT_OF_RESOURCES);
                tempConfigItem->portType = portType;
                tempConfigItem->topic = topic;
                tempConfigItem->countfDescriptor = 0;
                tempConfigItem->componentType = componentManifest.componentType;
                tempConfigItem->initializationString = configItem->initializationString;
                tempConfigItem->componentId = configItem->componentId;
                tempConfigItem->totalToSchedule = 0;
                tempConfigItem->scheduleEarly = 0;
            }
            
            {
                xme_core_nodeMgr_compRep_portHandle_t portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, i);

                tempConfigItem->port = xme_core_nodeMgr_compRep_getDataPacketID(portHandle);
            }

            //Now each port can have multiple functions so we need to fill the function descriptor array
            for (j = 0U; j < xme_core_manifestRepository_getFunctionCount(&componentManifest); j++)
            {   //for each function
                // here again we assume the function list and the function descriptors are in the same order
                for (k = 0U; k < componentManifest.functionManifests[j].requiredPortIndicesLength; k++)
                { //For each of the required ComponentPort Vertices
                    if (componentManifest.functionManifests[j].requiredPortIndices[k] == i)
                    {
                        //So this is a function which needs the port
                        xme_core_nodeMgr_compRep_functionHandle_t functionHandle =
                            xme_core_nodeMgr_compRep_getFunction(componentHandle, j);
                        const xme_core_exec_functionDescriptor_t* funcDesc =
                            xme_core_nodeMgr_compRep_getFunctionDescriptor(functionHandle);

                        tempConfigItem->fDescriptor[tempConfigItem->countfDescriptor] = (xme_core_exec_functionDescriptor_t*)funcDesc; // TODO: Const specifier ignored (issue #3999)
                        tempConfigItem->countfDescriptor++;
                    }
                }
                for (k = 0; k < componentManifest.functionManifests[j].optionalPortIndicesLength; k++)
                { //Now iterarte over the Optional Port Vertices
                    if (componentManifest.functionManifests[j].optionalPortIndices[k] == i)
                    {
                        //So this is a function which needs the port
                        xme_core_nodeMgr_compRep_functionHandle_t functionHandle =
                            xme_core_nodeMgr_compRep_getFunction(componentHandle, j);
                        const xme_core_exec_functionDescriptor_t* funcDesc =
                            xme_core_nodeMgr_compRep_getFunctionDescriptor(functionHandle);

                        tempConfigItem->fDescriptor[tempConfigItem->countfDescriptor] = (xme_core_exec_functionDescriptor_t*)funcDesc; // TODO: Const specifier ignored (issue #3999)
                        tempConfigItem->countfDescriptor++;
                    }
                }
            }
        }

        //now which functions to add to schedule?
        //that is simple, all the functions which are under the current configItem
        //Note currently we add it at the end.
        for (j = 0; j < configItem->countfDescriptor; j++)
        {
            xme_hal_table_rowHandle_t localCopyRowHandle;
            xme_core_exec_schedule_table_entry_t *tableEntry;

            localCopyRowHandle = XME_HAL_TABLE_ADD_ITEM(localCopy);
            XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != localCopyRowHandle, XME_STATUS_OUT_OF_RESOURCES);
            tableEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(localCopy, localCopyRowHandle);
            tableEntry->componentId = configItem->fDescriptor[j]->componentId;
            tableEntry->functionId = configItem->fDescriptor[j]->functionId;
            tableEntry->functionArgs = configItem->fDescriptor[j]->taskArgs;
            tableEntry->slotStart_ns = nextCycle_ns;
    #ifdef XME_EXECUTION_MODEL_BEST_EFFORT // TODO: Temporary hack for supporting event-driven systems (issue #3946) until #4043 is implemented
            tableEntry->slotLength_ns = 1ull;
    #else
            tableEntry->slotLength_ns = configItem->fDescriptor[j]->wcet_ns;
    #endif
        
            // Compute period divider
            {
                xme_core_nodeMgr_compRep_functionHandle_t functionHandle =
                    xme_core_nodeMgr_compRep_getFunction(componentHandle, j);
                xme_hal_time_timeInterval_t executionPeriod =
                    xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle);

                if (0 == executionPeriod)
                {
                    tableEntry->periodDivider = 0;
                }
                else if (executionPeriod % majorCycleDuration_ns != 0)
                {
                    tableEntry->periodDivider = (uint32_t)(executionPeriod / majorCycleDuration_ns);
                    // We use the division result anywhy, because it will be more or less "near" the desired frequency.
                    // This means we always round down which means the function will be executed a bit more often as specified.
                    if (0 == tableEntry->periodDivider)
                    {
                        XME_LOG(XME_LOG_WARNING,
                            "[plugAndPlayClient] Requested execution period %" PRIu64 " for function (componentId = %" PRIu32 ", componentType = %d, functionTypeID = %" PRIu16 ") "
                            "cannot be divided evenly through the node's current schedule period %" PRIu64 ". Function will be executed every cycle.\n",
                            executionPeriod, configItem->componentId, configItem->componentType, j, majorCycleDuration_ns);
                    }
                    else
                    {
                        XME_LOG(XME_LOG_WARNING,
                            "[plugAndPlayClient] Requested execution period %" PRIu64 " for function (componentId = %" PRIu32 ", componentType = %d, functionTypeID = %" PRIu16 ") "
                            "cannot be divided evenly through the node's current schedule period %" PRIu64 ". Function will be executed each %" PRIu32 "-nth cycle.\n",
                            executionPeriod, configItem->componentId, configItem->componentType, j, majorCycleDuration_ns, tableEntry->periodDivider);
                    }
                }
                else
                {
                    tableEntry->periodDivider = (uint32_t)(executionPeriod / majorCycleDuration_ns);
                }
            }
        
            tableEntry->periodDividerOffset = configItem->periodDividerOffset[j];
            nextCycle_ns = tableEntry->slotStart_ns + tableEntry->slotLength_ns;
            XME_CHECK_MSG 
            (
                nextCycle_ns < majorCycleDuration_ns,
                XME_STATUS_INTERNAL_ERROR,
                XME_LOG_ERROR,
                "%s:%d New schedule is longer than the given cycle length for this node.",
                __FILE__,
                __LINE__
            );
            tableEntry->completion = true;
        
            //create the function execution unit.
            //TODO we need to create a data structure to keep the count if we have already created the
            //execution unit.
            status = xme_core_exec_dispatcher_createFunctionExecutionUnit(configItem->fDescriptor[j], true);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
        }
        configItem->totalToSchedule--;
        if (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == configItem->portType)
        {
            xme_hal_table_rowHandle_t tempConfigItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            xme_core_pnp_pnpClientConfiguration_CPV_config_t* tempConfigItem = NULL;

            //Check if there is another entry with same topic, componentID, componntType but portType=subscription
            //if this exists we need to decrement the scheduleEarly if it is > 0
            //This is needed because we were supposed to schedule the required subscription early but
            //this component has a publication port so will automatically get scheduled early.
            //So we can decrement the count
            while (true)
            {
                XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_CPV_config_table,
                    xme_hal_table_rowHandle_t, tempConfigItemHandle,
                    xme_core_pnp_pnpClientConfiguration_CPV_config_t, tempConfigItem,
                    (tempConfigItem->componentType == configItem->componentType &&
                     tempConfigItem->componentId == configItem->componentId &&
                     tempConfigItem->portType == XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION &&
                     tempConfigItem->topic == configItem->topic));
                
                if (NULL == tempConfigItem) { break; }

                if (0 < tempConfigItem->scheduleEarly)
                {
                    tempConfigItem->scheduleEarly--;
                }
            }

        }
    }
    
    return XME_STATUS_SUCCESS;
}

static xme_status_t
initChannelInjectorHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_channelInjector_config_t *configItem
)
{
    xme_status_t status;

    //We are here it means it does not exist in the existsing schedule
    if (NULL == configItem->fDescriptor)
    {
        // We are here => there does not exists a waypoint instance pointer for this in our database
        // create function will create a new instance or return the descriptor of already created one
        // Hence create one and then schedule it
        status = createChannelInjectorWaypointInstance(&(configItem->fDescriptor), (xme_core_component_t)XME_CORE_COMPONENT_ID_WAYPOINT_CHANNELINJECTOR);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        //Now add the corresponding config
        status = channelInjectorWaypointAddConfig
        (
            (configItem->fDescriptor),
            configItem->inputPort,
            configItem->inputPortQueueSize,
            configItem->outputPort,
            configItem->srcChID,
            configItem->topic,
            configItem->topicSize,
            &configItem->instanceId,
            &(configItem->buffer)
        );
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }
    {
        xme_hal_table_rowHandle_t localCopyRowHandle;
        xme_core_exec_schedule_table_entry_t *tableEntry;

        localCopyRowHandle = XME_HAL_TABLE_ADD_ITEM(localCopy);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != localCopyRowHandle, XME_STATUS_OUT_OF_RESOURCES);
        tableEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE( localCopy, localCopyRowHandle);
        tableEntry->componentId = configItem->fDescriptor->componentId;
        tableEntry->functionId = configItem->fDescriptor->functionId;
        tableEntry->functionArgs = (void *) (uintptr_t)configItem->instanceId;
        tableEntry->slotStart_ns = nextCycle_ns;
#ifdef XME_EXECUTION_MODEL_BEST_EFFORT // TODO: Temporary hack for supporting event-driven systems (issue #3946) until #4043 is implemented
        tableEntry->slotLength_ns = 1ull;
#else
        tableEntry->slotLength_ns = configItem->fDescriptor->wcet_ns;
#endif
        tableEntry->periodDivider = configItem->periodDivider;
        tableEntry->periodDividerOffset = configItem->periodDividerOffset;
        nextCycle_ns = tableEntry->slotStart_ns + tableEntry->slotLength_ns;
        XME_CHECK_MSG 
        (
            nextCycle_ns < majorCycleDuration_ns,
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            "%s:%d New schedule is longer than the given cycle length for this node.",
            __FILE__,
            __LINE__
        );
        tableEntry->completion = true;
        //We dont issue a createFunctionExecutionUnit for waypoints
    }
    configItem->totalToSchedule--;
    return XME_STATUS_SUCCESS;
}

static xme_status_t
initChannelSelectorHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_channelSelector_config_t *configItem
)
{
    xme_status_t status;
    if (NULL == configItem->fDescriptor)
    {
        // We are here => there does not exists a waypoint instance pointer for this in our database
        // create function will create a new instance or return the descriptor of already created one
        // Hence create one and then schedule it
        status = createChannelSelectorWaypointInstance(&(configItem->fDescriptor), (xme_core_component_t)XME_CORE_COMPONENT_ID_WAYPOINT_CHANNELSELECTOR);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        //Now add the corresponding config
        status = channelSelectorWaypointAddConfig
        (
            (configItem->fDescriptor),
            &configItem->instanceId,
            configItem->inputPort,
            configItem->inputPortQueueSize,
            configItem->outputPort,
            configItem->topic,
            configItem->topicSize,
            configItem->srcChID,
            configItem->dstChID,
            &(configItem->buffer)
        );
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }
    {
        xme_hal_table_rowHandle_t localCopyRowHandle;
        xme_core_exec_schedule_table_entry_t *tableEntry;

        localCopyRowHandle = XME_HAL_TABLE_ADD_ITEM(localCopy);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != localCopyRowHandle, XME_STATUS_OUT_OF_RESOURCES);
        tableEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(localCopy, localCopyRowHandle);
        tableEntry->componentId = configItem->fDescriptor->componentId;
        tableEntry->functionId = configItem->fDescriptor->functionId;
        tableEntry->functionArgs = (void *) (uintptr_t)configItem->instanceId;
        tableEntry->slotStart_ns = nextCycle_ns;
#ifdef XME_EXECUTION_MODEL_BEST_EFFORT // TODO: Temporary hack for supporting event-driven systems (issue #3946) until #4043 is implemented
        tableEntry->slotLength_ns = 1ull;
#else
        tableEntry->slotLength_ns = configItem->fDescriptor->wcet_ns;
#endif
        tableEntry->periodDivider = configItem->periodDivider;
        tableEntry->periodDividerOffset = configItem->periodDividerOffset;
        nextCycle_ns = tableEntry->slotStart_ns + tableEntry->slotLength_ns;
        XME_CHECK_MSG 
        (
            nextCycle_ns < majorCycleDuration_ns,
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            "%s:%d New schedule is longer than the given cycle length for this node.",
            __FILE__,
            __LINE__
        );
        tableEntry->completion = true;
    //We dont issue a createFunctionExecutionUnit for waypoints
    }
    configItem->totalToSchedule--;
    return XME_STATUS_SUCCESS;
}

static xme_status_t
initudpSendHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_udpSend_config_t *configItem
)
{
    xme_status_t status;
    if (NULL == configItem->fDescriptor)
    {
        // We are here => there does not exists a waypoint instance pointer for this in our database
        // create function will create a new instance or return the descriptor of already created one
        // Hence create one and schedule it
        status = createUdpSendWaypointInstance(&(configItem->fDescriptor), (xme_core_component_t)XME_CORE_COMPONENT_ID_WAYPOINT_UDPSEND);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        //Now add the corresponding config
        status = udpSendWaypointAddConfig((configItem->fDescriptor), configItem->inputPort, configItem->key, 
                                              configItem->destIP, configItem->port, configItem->topic, 
                                              configItem->topicSize, &configItem->instanceId, &(configItem->buffer), false);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }
    {
        xme_hal_table_rowHandle_t localCopyRowHandle;
        xme_core_exec_schedule_table_entry_t *tableEntry;

        localCopyRowHandle = XME_HAL_TABLE_ADD_ITEM(localCopy);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != localCopyRowHandle, XME_STATUS_OUT_OF_RESOURCES);
        tableEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(localCopy, localCopyRowHandle);
        tableEntry->componentId = configItem->fDescriptor->componentId;
        tableEntry->functionId = configItem->fDescriptor->functionId;
        tableEntry->functionArgs = (void *) (uintptr_t)configItem->instanceId;
        tableEntry->slotStart_ns = nextCycle_ns;
#ifdef XME_EXECUTION_MODEL_BEST_EFFORT // TODO: Temporary hack for supporting event-driven systems (issue #3946) until #4043 is implemented
        tableEntry->slotLength_ns = 1ull;
#else
        tableEntry->slotLength_ns = configItem->fDescriptor->wcet_ns;
#endif
        tableEntry->slotLength_ns = configItem->fDescriptor->wcet_ns;
        tableEntry->periodDivider = configItem->periodDivider;
        tableEntry->periodDividerOffset = configItem->periodDividerOffset;
        nextCycle_ns = tableEntry->slotStart_ns + tableEntry->slotLength_ns;
        XME_CHECK_MSG 
        (
            nextCycle_ns < majorCycleDuration_ns,
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            "%s:%d New schedule is longer than the given cycle length for this node.",
            __FILE__,
            __LINE__
        );
        tableEntry->completion = true;
        //We dont issue a createFunctionExecutionUnit for waypoints
    }
    configItem->totalToSchedule--;
    return XME_STATUS_SUCCESS;
}

static xme_status_t
initudpRecvHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_udpRecv_config_t *configItem
)
{
    xme_status_t status;
    if (NULL == configItem->fDescriptor)
    {
        // We are here => there does not exists a waypoint instance pointer for this in our database
        // create function will create a new instance or return the descriptor of already created one
        // Hence create one and then schedule it
        status = createUdpReceiveWaypointInstance(&(configItem->fDescriptor), (xme_core_component_t)XME_CORE_COMPONENT_ID_WAYPOINT_UDPRECEIVE);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        //Now add the corresponding config
        status = udpReceiveWaypointAddConfig((configItem->fDescriptor), configItem->outputPort, configItem->key, 
                                                 configItem->port, configItem->topic, configItem->topicSize, 
                                                 &configItem->instanceId, &(configItem->buffer));
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }
    {
        xme_hal_table_rowHandle_t localCopyRowHandle;
        xme_core_exec_schedule_table_entry_t *tableEntry;

        localCopyRowHandle = XME_HAL_TABLE_ADD_ITEM(localCopy);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != localCopyRowHandle, XME_STATUS_OUT_OF_RESOURCES);
        tableEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(localCopy, localCopyRowHandle);
        tableEntry->componentId = configItem->fDescriptor->componentId;
        tableEntry->functionId = configItem->fDescriptor->functionId;
        tableEntry->functionArgs = (void *) (uintptr_t)configItem->instanceId;
        tableEntry->slotStart_ns = nextCycle_ns;
#ifdef XME_EXECUTION_MODEL_BEST_EFFORT // TODO: Temporary hack for supporting event-driven systems (issue #3946) until #4043 is implemented
        tableEntry->slotLength_ns = 1ull;
#else
        tableEntry->slotLength_ns = configItem->fDescriptor->wcet_ns;
#endif
        tableEntry->periodDivider = configItem->periodDivider;
        tableEntry->periodDividerOffset = configItem->periodDividerOffset;
        nextCycle_ns = tableEntry->slotStart_ns + tableEntry->slotLength_ns;
        XME_CHECK_MSG 
        (
            nextCycle_ns < majorCycleDuration_ns,
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            "%s:%d New schedule is longer than the given cycle length for this node.",
            __FILE__,
            __LINE__
        );
        tableEntry->completion = true;
        //We dont issue a createFunctionExecutionUnit for waypoints
    }
    configItem->totalToSchedule--;
    return XME_STATUS_SUCCESS;
}

static xme_status_t
initMarshalerHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_marshaler_config_t *configItem
)
{
    xme_status_t status;
    if (NULL == configItem->fDescriptor)
    {
        // We are here => there does not exists a waypoint instance pointer for this in our database
        // create function will create a new instance or return the descriptor of already created one
        // Hence create one and then schedule it
        status = createMarshalerWaypointInstance(&(configItem->fDescriptor), (xme_core_component_t)XME_CORE_COMPONENT_ID_WAYPOINT_MARSHALER);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        //Now add the corresponding config
        status = marshalerWaypointAddConfig
        (
            (configItem->fDescriptor),
            configItem->inputPort,
            configItem->inputPortQueueSize,
            configItem->outputPort,
            &configItem->instanceId,
            configItem->topic,
            configItem->topicSize,
            configItem->channelID
        );
        XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, status);
    }
    {
        xme_hal_table_rowHandle_t localCopyRowHandle;
        xme_core_exec_schedule_table_entry_t *tableEntry;

        localCopyRowHandle = XME_HAL_TABLE_ADD_ITEM(localCopy);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != localCopyRowHandle, XME_STATUS_OUT_OF_RESOURCES);
        tableEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(localCopy, localCopyRowHandle);
        tableEntry->componentId = configItem->fDescriptor->componentId;
        tableEntry->functionId = configItem->fDescriptor->functionId;
        tableEntry->functionArgs = (void *) (uintptr_t)configItem->instanceId;
        tableEntry->slotStart_ns = nextCycle_ns;
#ifdef XME_EXECUTION_MODEL_BEST_EFFORT // TODO: Temporary hack for supporting event-driven systems (issue #3946) until #4043 is implemented
        tableEntry->slotLength_ns = 1ull;
#else
        tableEntry->slotLength_ns = configItem->fDescriptor->wcet_ns;
#endif
        tableEntry->periodDivider = configItem->periodDivider;
        tableEntry->periodDividerOffset = configItem->periodDividerOffset;
        nextCycle_ns = tableEntry->slotStart_ns + tableEntry->slotLength_ns;
        XME_CHECK_MSG 
        (
            nextCycle_ns < majorCycleDuration_ns,
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            "%s:%d New schedule is longer than the given cycle length for this node.",
            __FILE__,
            __LINE__
        );
        tableEntry->completion = true;
        //We dont issue a createFunctionExecutionUnit for waypoints
    }
    configItem->totalToSchedule--;
    return XME_STATUS_SUCCESS;
}

static xme_status_t
initDemarshalerHelperFunction
(
    xme_core_pnp_pnpClientConfiguration_demarshaler_config_t *configItem
)
{
    xme_status_t status;
    if (NULL == configItem->fDescriptor)
    {
        // We are here => there does not exists a waypoint instance pointer for this in our database
        // create function will create a new instance or return the descriptor of already created one
        // Hence create one and schedule it
        status = createDemarshalerWaypointInstance(&(configItem->fDescriptor), (xme_core_component_t)XME_CORE_COMPONENT_ID_WAYPOINT_DEMARSHALER);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        //Now add the corresponding config
        status = demarshalerWaypointAddConfig
        (
            (configItem->fDescriptor),
            configItem->inputPort,
            configItem->inputPortQueueSize,
            configItem->outputPort,
            &configItem->instanceId,
            configItem->topic,
            configItem->topicSize,
            configItem->channelID
        );
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }
    {
        xme_hal_table_rowHandle_t localCopyRowHandle;
        xme_core_exec_schedule_table_entry_t *tableEntry;

        localCopyRowHandle = XME_HAL_TABLE_ADD_ITEM(localCopy);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != localCopyRowHandle, XME_STATUS_OUT_OF_RESOURCES);
        tableEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(localCopy, localCopyRowHandle);
        tableEntry->componentId = configItem->fDescriptor->componentId;
        tableEntry->functionId = configItem->fDescriptor->functionId;
        tableEntry->functionArgs = (void *) (uintptr_t)configItem->instanceId;
        tableEntry->slotStart_ns = nextCycle_ns;
#ifdef XME_EXECUTION_MODEL_BEST_EFFORT // TODO: Temporary hack for supporting event-driven systems (issue #3946) until #4043 is implemented
        tableEntry->slotLength_ns = 1ull;
#else
        tableEntry->slotLength_ns = configItem->fDescriptor->wcet_ns;
#endif
        tableEntry->periodDivider = configItem->periodDivider;
        tableEntry->periodDividerOffset = configItem->periodDividerOffset;
        nextCycle_ns = tableEntry->slotStart_ns + tableEntry->slotLength_ns;
        XME_CHECK_MSG 
        (
            nextCycle_ns < majorCycleDuration_ns,
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_ERROR,
            "%s:%d New schedule is longer than the given cycle length for this node.\n",
            __FILE__,
            __LINE__
        );
        tableEntry->completion = true;
        //We dont issue a createFunctionExecutionUnit for waypoints
    }
    configItem->totalToSchedule--;

    return XME_STATUS_SUCCESS;
}

static xme_status_t
updateScheduleInTheExecutionManager(void)
{
    xme_hal_singlyLinkedList_t(1) tempList;
    xme_core_exec_schedule_table_t *nodeSchedules;
    xme_status_t status;
    XME_HAL_SINGLYLINKEDLIST_INIT(tempList);
    xme_core_exec_scheduler_createScheduleTable
    (
        &nodeSchedules,
        majorCycleDuration_ns
    );
    XME_LOG(XME_LOG_DEBUG,"################################################################\n");

    XME_HAL_TABLE_ITERATE_BEGIN(localCopy,
                                    xme_hal_table_rowHandle_t, rh,
                                    xme_core_exec_schedule_table_entry_t, schedule);
    {
        status = xme_core_exec_scheduler_addElementToScheduleTable
        (
            nodeSchedules,
            schedule->componentId,
            schedule->functionId,
            schedule->functionArgs,
            schedule->slotStart_ns,
            schedule->slotLength_ns,
            schedule->periodDivider,
            schedule->periodDividerOffset,
            schedule->completion
        );
        XME_ASSERT(XME_STATUS_SUCCESS == status);
        XME_LOG(XME_LOG_DEBUG,"*********************************************\n");
        XME_LOG(XME_LOG_DEBUG,"%3d    %3d    0x%x    %ld    %ld    %d    %d\n",schedule->componentId, schedule->functionId, 
                                    schedule->functionArgs, schedule->slotStart_ns, schedule->slotLength_ns, schedule->periodDivider, schedule->periodDividerOffset);
    }
    XME_HAL_TABLE_ITERATE_END();
    xme_hal_singlyLinkedList_addItem(&tempList, (void *)nodeSchedules);
    status = xme_core_exec_scheduler_overwriteFullScheduleSet((xme_hal_linkedList_descriptor_t *)&tempList);
    XME_CHECK_MSG 
    (
        XME_STATUS_SUCCESS == status,
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_ERROR,
        "Replacing the schedule failed.\n"
    );

    return XME_STATUS_SUCCESS;

}

void
xme_core_pnp_pnpClientScheduling_init(void)
{
    XME_HAL_TABLE_INIT(localCopy);
}

xme_status_t 
xme_core_pnp_pnpClientScheduling_initComponents(void)
{
    xme_hal_table_rowHandle_t localCopyRowHandle;
    xme_core_exec_schedule_table_entry_t *tableEntry;
    xme_status_t status;
    xme_core_exec_schedule_table_t *table = NULL;
    xme_core_exec_schedule_handle_t curH;
    XME_HAL_TABLE_CLEAR(localCopy);
    
  
    {
        //Extract the schedule information from the Execution Manager
        curH = xme_core_exec_scheduler_getCurrentScheduleHandle();
        xme_core_exec_scheduler_getSchedule(curH, &table);
        majorCycleDuration_ns = table->majorCycleDuration_ns;
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(((table)->entries), xme_core_exec_schedule_table_entry_t, srcSchedule);
        {
            XME_LOG(XME_LOG_DEBUG,"*********************************************\n");
            XME_LOG(XME_LOG_DEBUG,"%3d    %3d    0x%x    %ld    %ld    %d    %d\n",srcSchedule->componentId, srcSchedule->functionId, 
                                    srcSchedule->functionArgs, srcSchedule->slotStart_ns, srcSchedule->slotLength_ns, srcSchedule->periodDivider, srcSchedule->periodDividerOffset);
            localCopyRowHandle = XME_HAL_TABLE_ADD_ITEM(localCopy);
            XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != localCopyRowHandle, XME_STATUS_OUT_OF_RESOURCES);
            tableEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(localCopy, localCopyRowHandle);
            tableEntry->componentId = srcSchedule->componentId;
            tableEntry->functionId = srcSchedule->functionId;
            tableEntry->functionArgs = srcSchedule->functionArgs;
            tableEntry->slotStart_ns = srcSchedule->slotStart_ns;
            tableEntry->slotLength_ns = srcSchedule->slotLength_ns;
            tableEntry->periodDivider = srcSchedule->periodDivider;
            tableEntry->periodDividerOffset = srcSchedule->periodDividerOffset;
            nextCycle_ns = srcSchedule->slotStart_ns + srcSchedule->slotLength_ns;

            XME_LOG(XME_LOG_DEBUG,"*********************************************\n");
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    }

    //The current algorithm implemented follows the logic from all producers (PUBLISHERS) to consumers (SUBSCRIBERS) of data
    //A producer here is defined as a component that contains at least one output port, whereas consumers are all other components
    //In XMT the algorithm is a bit different, there the components which are producers are scheduled first
    //But then are those components scheduled which have both kinds of ports that is subscribers and publishers
    //And then come the components which have only the subscribers
    //This is directly not possible here because the graph is port based and not component based.
    //Although currently not done but this is possible in XME to do the same if we process all the nodes of the graph and club them
    //together based on their componentId.

    //First iterate over the Publishers and schedule them
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_CPV_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_CPV_config_t, configItem);
    {
        while (0 != configItem->totalToSchedule && 
                   (XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == configItem->portType ||
                    XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER == configItem->portType ||
                    XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER == configItem->portType) )
        {
            status = initComponentPortVertexHelperFunction(configItem);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
        }
    }
    XME_HAL_TABLE_ITERATE_END();
    
    //Then iterate over the Channel Injector way points and and schedule them
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_channelInjector_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_channelInjector_config_t, configItem);
    {
        while (0 != configItem->totalToSchedule)
        {
            status = initChannelInjectorHelperFunction(configItem);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    //Then iterate over the Channel Selector way points and and schedule them
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_channelSelector_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_channelSelector_config_t, configItem);
    {
        while (0 != configItem->totalToSchedule)
        {
            status = initChannelSelectorHelperFunction(configItem);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    //Now iterate over the Marshaler way points and and schedule them
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_marshaler_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_marshaler_config_t, configItem);
    {
        while (0 != configItem->totalToSchedule)
        {
            status = initMarshalerHelperFunction(configItem);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
        }
    }
    XME_HAL_TABLE_ITERATE_END();
    
    //iterate over the udp send way points and and schedule them
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_udpSend_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_udpSend_config_t, configItem);
    {
        while (0 != configItem->totalToSchedule)
        {
            status = initudpSendHelperFunction(configItem);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    // Now schedule all subscribers that need to be scheduled early, which are
    // those component ports which are local and read from the publisher on the same node.
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_CPV_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_CPV_config_t, configItem);
    {
        while (0 != configItem->scheduleEarly &&
                  (XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION == configItem->portType 
                       || XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == configItem->portType
                       || XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == configItem->portType) )
        {
            status = initComponentPortVertexHelperFunction(configItem);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
            configItem->scheduleEarly--;
        }
    }
    XME_HAL_TABLE_ITERATE_END();
    

    // last we schedule remaining components which need the complete data path, that is udp recv, demarshaler and subscribing component port
    // here scheduling goes nested because we may need to schedule the complete stack muliple times
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, xme_hal_table_rowHandle_t, rhUdpR, xme_core_pnp_pnpClientConfiguration_udpRecv_config_t, configItemUdpR);
    {
        while (0 != configItemUdpR->totalToSchedule)
        {
            // components  subscribed to same topic are to be scheduled together
            xme_core_topic_t topic = configItemUdpR->topic;
            status = initudpRecvHelperFunction(configItemUdpR);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
     
            //Now schedule the Demarshaler of the same topic
            XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_demarshaler_config_t, configItemDem);
            {
                if (configItemDem->topic == topic && 0 != configItemDem->totalToSchedule)
                {
                    status = initDemarshalerHelperFunction(configItemDem);
                    XME_CHECK(XME_STATUS_SUCCESS == status, status);
                    break;
                }
            }
            XME_HAL_TABLE_ITERATE_END();

            //Now schdule the component port that is a subscriber
            XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_CPV_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_CPV_config_t, configItemComp);
            {
                if (0 != configItemComp->totalToSchedule && configItemComp->topic == topic &&
                        (XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION == configItemComp->portType 
                             || XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == configItemComp->portType
                             || XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == configItemComp->portType) )
                {
                    status = initComponentPortVertexHelperFunction(configItemComp);
                    XME_CHECK(XME_STATUS_SUCCESS == status, status);
                    break;
                }
            }
            XME_HAL_TABLE_ITERATE_END();
        }
    }
    XME_HAL_TABLE_ITERATE_END();
    status = updateScheduleInTheExecutionManager();

    return status;
}

xme_status_t 
xme_core_pnp_pnpClientScheduling_finiComponents(void)
{
    xme_hal_table_rowHandle_t localCopyRH;
    xme_core_exec_schedule_table_entry_t *tableEntry;
    xme_status_t status;
    xme_core_exec_schedule_table_t *table = NULL;
    xme_core_exec_schedule_handle_t curH;
    XME_HAL_TABLE_CLEAR(localCopy);
    
  
    {
        //Extract the schedule information from the Execution Manager
        curH = xme_core_exec_scheduler_getCurrentScheduleHandle();
        xme_core_exec_scheduler_getSchedule(curH, &table);
        majorCycleDuration_ns = table->majorCycleDuration_ns;
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(((table)->entries), xme_core_exec_schedule_table_entry_t, srcSchedule);
        {
            XME_LOG(XME_LOG_DEBUG,"*********************************************\n");
            XME_LOG(XME_LOG_DEBUG,"%3d    %3d    0x%x    %ld    %ld    %d    %d\n",srcSchedule->componentId, srcSchedule->functionId, 
                                    srcSchedule->functionArgs, srcSchedule->slotStart_ns, srcSchedule->slotLength_ns, srcSchedule->periodDivider, srcSchedule->periodDividerOffset);
            localCopyRH = XME_HAL_TABLE_ADD_ITEM(localCopy);
            XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != localCopyRH, XME_STATUS_OUT_OF_RESOURCES);
            tableEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(localCopy, localCopyRH);
            tableEntry->componentId = srcSchedule->componentId;
            tableEntry->functionId = srcSchedule->functionId;
            tableEntry->functionArgs = srcSchedule->functionArgs;
            tableEntry->slotStart_ns = srcSchedule->slotStart_ns;
            tableEntry->slotLength_ns = srcSchedule->slotLength_ns;
            tableEntry->periodDivider = srcSchedule->periodDivider;
            tableEntry->periodDividerOffset = srcSchedule->periodDividerOffset;
            nextCycle_ns = srcSchedule->slotStart_ns + srcSchedule->slotLength_ns;

            XME_LOG(XME_LOG_DEBUG,"*********************************************\n");
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    }

    //The current algorithm is simple, remove everything marked deleted.

    //First iterate over the Components
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_CPV_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_CPV_config_t, configItem);
    {
        if (true == configItem->toDelete)
        {
            uint8_t i = 0;
            xme_hal_table_rowHandle_t rhTemp = rh;
            //xme_core_componentManifest_t componentManifest;
            xme_core_pnp_pnpClientConfiguration_CPV_config_t* configItemTemp = configItem;
            localCopyRH = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            tableEntry = NULL;

            //Call the exit thread for the component
            //but is there a way to do that??
            //The following is a hack which I figured out from the API of EM
            for (i = 0; i < configItem->countfDescriptor; i++)
               xme_core_exec_setTaskState(configItem->componentId, configItem->fDescriptor[i]->functionId, XME_CORE_EXEC_FUNCTION_STATE_TERMINATED);
#if 0
            //Disabling this piece of code because we can destroy the data structure of component which is still to be executed in the current cycle
            //This causes execution manager to panic and resulting in the shutdown of the node
            //The new schedule only gets implemented at the begining of the next cycle.
            status = xme_core_manifestRepository_findComponentManifest(configItem->componentType, &componentManifest);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
            XME_CHECK(NULL != componentManifest.destroyInstanceFunction, XME_STATUS_INTERNAL_ERROR);

            //Destroy the instance of the component using the pointer from the manifest
            //We don't need to pass any ports information to destroyConfig because it can be obtained from ComponentPortManager (xme_core_directory)
            status = componentManifest.destroyInstanceFunction(configItem->componentId, configItem->fDescriptor, NULL);
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
#endif
            //Now the Component has been destroyed so remove it from the schedule of EM
            //TODO This is wrong. Ideally we should just call the fini which should do the cleanup and then call the destroyInstanceFunction
            //TODO Here we would remove it from the schedule entry this will result in the fini (after setting the state to TERMINATED)
            //     not being called.
            //remove the entry from the localCopy
            XME_HAL_TABLE_GET_NEXT(localCopy, 
                               xme_hal_table_rowHandle_t, localCopyRH,
                               xme_core_exec_schedule_table_entry_t, tableEntry,
                               (tableEntry->componentId == configItem->componentId));
            //We iterate over and remove all the enteries of the said componentID
            while (NULL != tableEntry)
            {
                XME_HAL_TABLE_REMOVE_ITEM(localCopy, localCopyRH);
                tableEntry = NULL;
                XME_HAL_TABLE_GET_NEXT(localCopy, 
                                   xme_hal_table_rowHandle_t, localCopyRH,
                                   xme_core_exec_schedule_table_entry_t, tableEntry,
                                   (tableEntry->componentId == configItem->componentId));
            }

            //remove the entry from xme_core_pnp_pnpClientConfiguration_CPV_config_table
            //Iterate over all entries and remove them all. The destroyInstanceFunction will actually remove everything so
            //it should go from the config table too
            while (NULL != configItemTemp)
            {
                xme_hal_mem_free(configItemTemp->initializationString);
                XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpClientConfiguration_CPV_config_table, rh);
                configItemTemp = NULL;
                XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_CPV_config_table,
                                          xme_hal_table_rowHandle_t, rhTemp,
                                          xme_core_pnp_pnpClientConfiguration_CPV_config_t, configItemTemp,
                                          configItemTemp->componentId == configItem->componentId);
            }
        }
    }
    XME_HAL_TABLE_ITERATE_END();
    
    //Then iterate over the Channel Injector way points and remove
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_channelInjector_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_channelInjector_config_t, configItem);
    {
        if (true == configItem->toDelete)
        {
            localCopyRH = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            tableEntry = NULL;
            //We can't call exit component for a waypoint
            //Remove the entry from EM for this InstanceID;
            XME_HAL_TABLE_GET_NEXT(localCopy, 
                                       xme_hal_table_rowHandle_t, localCopyRH,
                                       xme_core_exec_schedule_table_entry_t, tableEntry,
                                       ((tableEntry->componentId == configItem->fDescriptor->componentId)
                                           && (tableEntry->functionArgs == (void *) (uintptr_t)configItem->instanceId)));
            XME_HAL_TABLE_REMOVE_ITEM(localCopy, localCopyRH);
            //remove the waypoint
            status = channelInjectorWaypointRemoveConfig
            (
                configItem->fDescriptor,
                configItem->instanceId,
                configItem->inputPort,
                configItem->outputPort,
                configItem->buffer
            );
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
            //remove the entry from table
            XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpClientConfiguration_channelInjector_config_table, rh);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    //Then iterate over the Channel Selector way points and remove
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_channelSelector_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_channelSelector_config_t, configItem);
    {
        if (true == configItem->toDelete)
        {
            localCopyRH = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            tableEntry = NULL;
            //We can't call exit component for a waypoint
            //Remove the entry from EM for this InstanceID;
            XME_HAL_TABLE_GET_NEXT(localCopy, 
                                       xme_hal_table_rowHandle_t, localCopyRH,
                                       xme_core_exec_schedule_table_entry_t, tableEntry,
                                       ((tableEntry->componentId == configItem->fDescriptor->componentId)
                                           && (tableEntry->functionArgs == (void *) (uintptr_t)configItem->instanceId)));
            XME_HAL_TABLE_REMOVE_ITEM(localCopy, localCopyRH);
            //remove the waypoint
            status = channelSelectorWaypointRemoveConfig
            (
                configItem->fDescriptor,
                configItem->instanceId,
                configItem->inputPort,
                configItem->outputPort,
                configItem->topic,
                configItem->srcChID,
                configItem->dstChID,
                configItem->buffer
            );
            XME_CHECK(XME_STATUS_SUCCESS == status, status);
            //remove the entry from table
            XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpClientConfiguration_channelSelector_config_table, rh);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    //Now iterate over the Marshaler way points and remove
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_marshaler_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_marshaler_config_t, configItem);
    {
        if (true == configItem->toDelete)
        {
            localCopyRH = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            tableEntry = NULL;
            //We can't call exit component for a waypoint
            //Remove the entry from EM for this InstanceID;
            XME_HAL_TABLE_GET_NEXT(localCopy, 
                                       xme_hal_table_rowHandle_t, localCopyRH,
                                       xme_core_exec_schedule_table_entry_t, tableEntry,
                                       ((tableEntry->componentId == configItem->fDescriptor->componentId)
                                           && (tableEntry->functionArgs == (void *) (uintptr_t)configItem->instanceId)));
            XME_HAL_TABLE_REMOVE_ITEM(localCopy, localCopyRH);
            //remove the waypoint
            status = marshalerWaypointRemoveConfig
            (
                configItem->fDescriptor,
                configItem->inputPort,
                configItem->outputPort,
                configItem->instanceId
            );
            //remove the entry from table
            XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpClientConfiguration_marshaler_config_table, rh);
        }
    }
    XME_HAL_TABLE_ITERATE_END();
    
    //iterate over the udp send way points and remove
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_udpSend_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_udpSend_config_t, configItem);
    {
        if (true == configItem->toDelete)
        {
            localCopyRH = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            tableEntry = NULL;
            //We can't call exit component for a waypoint
            //Remove the entry from EM for this InstanceID;
            XME_HAL_TABLE_GET_NEXT(localCopy, 
                                       xme_hal_table_rowHandle_t, localCopyRH,
                                       xme_core_exec_schedule_table_entry_t, tableEntry,
                                       ((tableEntry->componentId == configItem->fDescriptor->componentId)
                                           && (tableEntry->functionArgs == (void *) (uintptr_t)configItem->instanceId)));
            XME_HAL_TABLE_REMOVE_ITEM(localCopy, localCopyRH);
            //remove the waypoint
            status = udpSendWaypointRemoveConfig
            (
                configItem->fDescriptor,
                configItem->inputPort,
                configItem->instanceId,
                configItem->buffer
            );
            //remove the entry from xme_core_pnp_pnpClientConfiguration_udpSend_config_table
            XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpClientConfiguration_udpSend_config_table, rh);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    //iterate over the udp recv way points and remove
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_udpRecv_config_t, configItem);
    {
        if (true == configItem->toDelete)
        {
            localCopyRH = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            tableEntry = NULL;
            //We can't call exit component for a waypoint
            //Remove the entry from EM for this InstanceID;
            XME_HAL_TABLE_GET_NEXT(localCopy, 
                                       xme_hal_table_rowHandle_t, localCopyRH,
                                       xme_core_exec_schedule_table_entry_t, tableEntry,
                                       ((tableEntry->componentId == configItem->fDescriptor->componentId)
                                           && (tableEntry->functionArgs == (void *) (uintptr_t)configItem->instanceId)));
            XME_HAL_TABLE_REMOVE_ITEM(localCopy, localCopyRH);
            //remove the waypoint
            status = udpReceiveWaypointRemoveConfig
            (
                configItem->fDescriptor,
                configItem->outputPort,
                configItem->instanceId,
                configItem->buffer
            );
            //remove the entry from the table
            XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, rh);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    //iterate over the demarshaler way points and remove
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_demarshaler_config_t, configItem);
    {
        if (true == configItem->toDelete)
        {
            localCopyRH = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            tableEntry = NULL;
            //We can't call exit component for a waypoint
            //Remove the entry from EM for this InstanceID;
            XME_HAL_TABLE_GET_NEXT(localCopy, 
                                       xme_hal_table_rowHandle_t, localCopyRH,
                                       xme_core_exec_schedule_table_entry_t, tableEntry,
                                       ((tableEntry->componentId == configItem->fDescriptor->componentId)
                                           && (tableEntry->functionArgs == (void *) (uintptr_t)configItem->instanceId)));
            XME_HAL_TABLE_REMOVE_ITEM(localCopy, localCopyRH);
            //remove the waypoint
            status = demarshalerWaypointRemoveConfig
            (
                configItem->fDescriptor,
                configItem->inputPort,
                configItem->outputPort,
                configItem->instanceId
            );
            //remove the entry from table
            XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, rh);
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    status = updateScheduleInTheExecutionManager();

    return status;
}

/*static xme_core_exec_componentDescriptor_t *
getComponentDescriptor
(
    const xme_core_exec_functionDescriptor_t* const fDesc
)
{
    xme_core_exec_componentDescriptor_t *temp = (xme_core_exec_componentDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_componentDescriptor_t));
    if (NULL == temp)
    {
        return NULL;
    }
    temp->componentId = fDesc->componentId;
    temp->init = NULL;
    temp->fini = NULL;
    temp->autoInit = false;
    temp->initParam = NULL;
    temp->initWcet_ns = (xme_hal_time_timeInterval_t)0;
    XME_HAL_SINGLYLINKEDLIST_INIT(temp->functions);
    (void) xme_hal_singlyLinkedList_addItem( (void*) &(temp->functions), fDesc );
    return temp;
}*/

/**
 * @}
 */
