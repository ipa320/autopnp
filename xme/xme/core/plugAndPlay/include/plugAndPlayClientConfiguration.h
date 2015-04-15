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
 * $Id: plugAndPlayClientConfiguration.h 7828 2014-03-14 09:32:09Z ruiz $
 */

/**
 * \file
 *         Plug and Play Client.
 *
 */

#ifndef XME_CORE_PNP_PLUGANDPLAYCLIENTADDCONFIG_H
#define XME_CORE_PNP_PLUGANDPLAYCLIENTADDCONFIG_H
/**
 * \addtogroup core_pnp_pnpClient
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"

#include "xme/core/broker/include/broker.h"
#include "xme/core/manifestRepository/include/manifestRepository.h"

#include "xme/hal/include/mem.h"

#include <inttypes.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \struct xme_core_pnp_pnpClientConfiguration_CPV_config_t
 *
 * \brief  Configuration entry for Components Port.
 */
typedef struct
{
    xme_core_component_portType_t portType; ///< Type of port, Subscription or Publication.
    xme_core_topic_t topic; ///< Topic of the Component Port Vertex.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t port; ///< Data Packet ID of the Component Port Vertex.
    xme_core_exec_functionDescriptor_t* fDescriptor[XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT]; ///< Array of function descriptors of the scheduled component port.
    uint8_t countfDescriptor; ///< Count of function descriptors stored in the array. 0 means that the corresponding component is not yet created on this node.
    uint32_t periodDivider[XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT]; ///< Array of Number of major cycles to be set as a period for this slot for each function.
    uint32_t periodDividerOffset[XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT]; ///< Array of Offset to the value of cycle counter for each function.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    uint8_t scheduleEarly; ///< Number of instances of components to be scheduled early.
    xme_core_componentType_t componentType; ///< Component type.
    xme_core_topic_pnpManagerRuntimeGraphModelPortData_t portData[XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT]; ///< Port instance specific data.
    xme_core_topic_pnpManagerRuntimeGraphModelFunctionData_t functionData[XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT]; ///< Function instance specific data.
    char* initializationString; ///< Component type specific initialization string. FIXME: Move to component information structure!
    xme_core_component_t componentId; ///< Component ID.
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle; ///< Component Handle.
    uint8_t totalUsageCount; ///< Usage count of this port. We cannot delete it if it is greater than 0
    bool toDelete; ///< if this configuration has to be deleted from the table and removed from the Execution Manager
} xme_core_pnp_pnpClientConfiguration_CPV_config_t;

/**
 * \struct xme_core_pnp_pnpClientConfiguration_marshaler_config_t
 *
 * \brief Configuration entry for marshaler waypoint.
 */
typedef struct 
{
    xme_core_topic_t topic; ///< Topic of the marshaler waypoint.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t *inputPort; ///< Data Packet ID of the input port.
    uint8_t inputPortQueueSize; ///< Queue size of the input port.
    xme_core_dataManager_dataPacketId_t *outputPort; ///< Data Packet ID of the output port.
    xme_wp_waypoint_instanceId_t instanceId; ///< Waypoint instance ID.
    xme_core_channelId_t channelID; ///< ChannelID of the connection.
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component. NULL means that the waypoint is not yet inititalized on this node.
    uint32_t periodDivider; ///< Number of major cycles to be set as a period for this slot.
    uint32_t periodDividerOffset; ///< Offset to the value of cycle counter.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toDelete; ///< if this configuration has to be deleted from the table and removed from the Execution Manager
} xme_core_pnp_pnpClientConfiguration_marshaler_config_t;

/**
 * \struct xme_core_pnp_pnpClientConfiguration_demarshaler_config_t
 *
 * \brief Configuration entry for demarshaler waypoint.
 */
typedef struct 
{
    xme_core_topic_t topic; ///< Topic of the demarshaler waypoint.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t *inputPort; ///< Data Packet ID of the input port.
    uint8_t inputPortQueueSize; ///< Queue size of the input port.
    xme_core_dataManager_dataPacketId_t *outputPort; ///< Data Packet ID of the output port.
    xme_wp_waypoint_instanceId_t instanceId; ///< Waypoint instance ID.
    xme_core_channelId_t channelID; ///< ChannelID of the connection.
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component. NULL means that the waypoint is not yet inititalized on this node.
    uint32_t periodDivider; ///< Number of major cycles to be set as a period for this slot.
    uint32_t periodDividerOffset; ///< Offset to the value of cycle counter.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toDelete; ///< if this configuration has to be deleted from the table and removed from the Execution Manager
} xme_core_pnp_pnpClientConfiguration_demarshaler_config_t;

/**
 * \struct xme_core_pnp_pnpClientConfiguration_channelInjector_config_t
 *
 * \brief Configuration entry for channel injector waypoint.
 */
typedef struct 
{
    xme_core_topic_t topic; ///< Topic of the channel injector waypoint.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t *inputPort; ///< Data Packet ID of the input port.
    uint8_t inputPortQueueSize; ///< Queue size of the input port.
    xme_core_dataManager_dataPacketId_t *outputPort; ///< Data Packet ID of the output port.
    xme_wp_waypoint_instanceId_t instanceId; ///< Waypoint instance ID.
    xme_core_channelId_t srcChID; ///< Source channel ID.
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component. NULL means that the waypoint is not yet inititalized on this node.
    uint32_t periodDivider; ///< Number of major cycles to be set as a period for this slot.
    uint32_t periodDividerOffset; ///< Offset to the value of cycle counter.
    void *buffer; ///< Pointer to the buffer used in the copying the data in waypoint.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toDelete; ///< if this configuration has to be deleted from the table and removed from the Execution Manager
} xme_core_pnp_pnpClientConfiguration_channelInjector_config_t;

/**
 * \struct xme_core_pnp_pnpClientConfiguration_channelSelector_config_t
 *
 * \brief Configuration entry for channel selector waypoint.
 */
typedef struct 
{
    xme_core_topic_t topic; ///< Topic of the channel selector waypoint.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t *inputPort; ///< Data Packet ID of the input port.
    uint8_t inputPortQueueSize; ///< Queue size of the input port.
    xme_core_dataManager_dataPacketId_t *outputPort; ///< Data Packet ID of the output port.
    xme_wp_waypoint_instanceId_t instanceId; ///< Waypoint instance ID.
    xme_core_channelId_t srcChID; ///< Source channel ID
    xme_core_channelId_t dstChID; ///< Source channel ID
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component. NULL means that the waypoint is not yet inititalized on this node.
    uint32_t periodDivider; ///< Number of major cycles to be set as a period for this slot.
    uint32_t periodDividerOffset; ///< Offset to the value of cycle counter.
    void *buffer; ///< Pointer to the buffer used in the copying the data in waypoint.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toDelete; ///< if this configuration has to be deleted from the table and removed from the Execution Manager
} xme_core_pnp_pnpClientConfiguration_channelSelector_config_t;

/**
 * \struct xme_core_pnp_pnpClientConfiguration_udpSend_config_t
 *
 * \brief Configuration entry for UDP Send waypoint.
 */
typedef struct 
{
    xme_core_topic_t topic; ///< Topic of the UDP Send waypoint.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t *inputPort; ///< Data Packet ID of the input port.
    uint8_t key[XME_WP_UDP_HEADER_KEY_LENGTH]; ///< Key to the corresponding entry in UDP Receive.
    char destIP[24]; ///< IP of the destination node.
    uint16_t port; ///< Networking port of the destination.
    xme_wp_waypoint_instanceId_t instanceId; ///< Waypoint instance ID.
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component. NULL means that the waypoint is not yet inititalized on this node.
    uint32_t periodDivider; ///< Number of major cycles to be set as a period for this slot.
    uint32_t periodDividerOffset; ///< Offset to the value of cycle counter.
    void *buffer; ///< Pointer to the buffer used in the Send waypoint.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toDelete; ///< if this configuration has to be deleted from the table and removed from the Execution Manager
} xme_core_pnp_pnpClientConfiguration_udpSend_config_t;

/**
 * \struct xme_core_pnp_pnpClientConfiguration_udpRecv_config_t
 *
 * \brief Configuration entry for UDP Receive waypoint.
 */
typedef struct 
{
    xme_core_topic_t topic; ///< Topic of the UDP Receive waypoint.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t *outputPort; ///< Data Packet ID of the output port.
    uint8_t key[XME_WP_UDP_HEADER_KEY_LENGTH]; ///< Key to the corresponding entry in UDP Send.
    uint16_t port; ///< Networking port where the Receive waypoint listens for incoming connections.
    xme_wp_waypoint_instanceId_t instanceId; ///< Waypoint instance ID.
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component. NULL means that the waypoint is not yet inititalized on this node.
    uint32_t periodDivider; ///< Number of major cycles to be set as a period for this slot.
    uint32_t periodDividerOffset; ///< Offset to the value of cycle counter.
    void *buffer; ///< Pointer to the buffer used in the Receive waypoint.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toDelete; ///< if this configuration has to be deleted from the table and removed from the Execution Manager
} xme_core_pnp_pnpClientConfiguration_udpRecv_config_t;

/**
 * \struct xme_core_pnp_pnpClientConfiguration_portMapping_t
 *
 * \brief table for port mappings.
 */
typedef struct 
{
    xme_core_dataManager_dataPacketId_t *srcPort; ///< Source port. 
    xme_core_dataManager_dataPacketId_t *dstPort; ///< Destination port. 
} xme_core_pnp_pnpClientConfiguration_portMapping_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN
/**
 * \brief Creates the configuration table for the Component.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding Component.
 * \param[in] edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 * \param[in] componentID The component identifier. 
 * \param[in] componentType The component type. 
 * \param[in] componentHandle The component handle. 
 * \param[in] portData The configuration data associated to the port. 
 * \param[in] functionData The configuration data associated to the function. 
 * \param[in] early If this components needs to be scheduled early.
 *              This is true for memcopy operations.
 *
 * \return Pointer to the data packet created for that port.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigComponentPortVertex
( 
    xme_core_component_portType_t portType, 
    const char *vertexData, 
    const char *edgeConfData,
    xme_core_component_t componentID,
    xme_core_componentType_t componentType,
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    const xme_core_topic_pnpManagerRuntimeGraphModelPortData_t* const portData,
    const xme_core_topic_pnpManagerRuntimeGraphModelFunctionData_t* const functionData,
    bool early
);

/**
 * \brief Removes the configuration of the provided port vertex.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding Component.
 * \param[in] edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 * \param[in] componentType The component type. 
 * \param[in] componentId The component identifier. 
 *
 * \return The data packet identifier corresponding to the removed component port vertex.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigComponentPortVertex
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData,
    xme_core_componentType_t componentType,
    xme_core_component_t componentId
);

/**
 * \brief Creates the configuration table for the Marshaler Waypoint.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding Waypoint.
 * \param[in] edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigMarshaler
(
    xme_core_component_portType_t portType, 
    const char *vertexData, 
    const char *edgeConfData
);

/**
 * \brief Removes from configuration table a given marshaler Waypoint configuration.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeData Pointer to data associated with the corresponding data
 *                 link edge.
 *
 * \return Pointer to the data packet removed.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigMarshaler
(
    xme_core_component_portType_t portType, 
    const char *vertexData, 
    const char *edgeConfData
);

/**
 * \brief Creates the configuration table for the Marshaler Waypoint.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding Waypoint.
 * \param[in] edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigDemarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeConfData
);

/**
 * \brief Removes from configuration table a given demarshaler Waypoint configuration.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeData Pointer to data associated with the corresponding data
 *                 link edge.
 *
 * \return Pointer to the data packet removed.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigDemarshaler
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeConfData
);
/**
 * \brief Creates the configuration table for the UDP Send Waypoint.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigUDPSend
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeConfData
);

/**
 * \brief Removes from configuration table a given UDP Send Waypoint configuration.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeData Pointer to data associated with the corresponding data
 *                 link edge.
 *
 * \return Pointer to the data packet removed.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigUDPSend
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
);

/**
 * \brief Creates the configuration table for the UDP Receieve Waypoint.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeConfData Pointer to data associated with the corresponding data
 *                     link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigUDPRecv
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeConfData
);

/**
 * \brief Removes from configuration table a given UDP Receive Waypoint configuration.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeData Pointer to data associated with the corresponding data
 *                 link edge.
 *
 * \return Pointer to the data packet removed.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigUDPRecv
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
);

/**
 * \brief Creates the configuration table for the Channel Injector Waypoint.
 *
 * \param[in] portType   Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeData   Pointer to data associated with the corresponding data
 *                   link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigChannelInjector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
);

/**
 * \brief Removes from configuration table a given channel injector Waypoint configuration.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeData Pointer to data associated with the corresponding data
 *                 link edge.
 *
 * \return Pointer to the data packet removed.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigChannelInjector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
);

/**
 * \brief Creates the configuration table for the Channel Selector Waypoint.
 *
 * \param[in] portType   Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeData   Pointer to data associated with the corresponding data
 *                   link edge.
 *
 * \return Pointer to the data packet created for that port.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_addConfigChannelSelector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
);

/**
 * \brief Removes from configuration table a given channel selector Waypoint configuration.
 *
 * \param[in] portType Type of port, Subscription or Publication.
 * \param[in] vertexData Pointer to data assocoated with the corresponding waypoint.
 * \param[in] edgeData Pointer to data associated with the corresponding data
 *                 link edge.
 *
 * \return Pointer to the data packet removed.
 */
xme_core_dataManager_dataPacketId_t*
xme_core_pnp_pnpClientConfiguration_removeConfigChannelSelector
(
    xme_core_component_portType_t portType,
    const char *vertexData,
    const char *edgeData
);
/**
 * \brief Creates the table for the unique portmappings which are to be registered with broker.
 *
 * \param[in] srcPort The source port of data that is Publication port.
 * \param[in] dstPort The destination port of data that is Subscription port.
 */
void
xme_core_pnp_pnpClientConfiguration_createPortMappingArray
(
    xme_core_dataManager_dataPacketId_t* srcPort,
    xme_core_dataManager_dataPacketId_t* dstPort
);

/**
 * \brief Iterates over the portmapping array and register the mappings with the broker.
 *
 * \retval XME_STATUS_SUCCESS if all mappings were successfully registered with broker.
 * \retval XME_STATUS_INTERNAL_ERROR if there was an error for any mapping.
 */
xme_status_t
xme_core_pnp_pnpClientConfiguration_addPortMappingToBroker(void);

XME_EXTERN_C_END
/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_PLUGANDPLAYCLIENTADDCONFIG_H
