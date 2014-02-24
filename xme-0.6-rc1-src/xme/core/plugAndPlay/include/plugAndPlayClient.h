/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: plugAndPlayClient.h 5217 2013-09-27 16:26:43Z wiesmueller $
 */

/**
 * \file
 *         Plug and Play Client.
 */

#ifndef XME_CORE_DIRECTORY_PLUGANDPLAYCLIENT_H
#define XME_CORE_DIRECTORY_PLUGANDPLAYCLIENT_H

/**
 * \defgroup core_pnp_pnpClient Plug and Play Client group. 
 * @{
 *
 * \brief Plug and Play Client group is related with configuring and initiating
 *        each new component.
 *
 * \details The plug and play client gets new requests from plug and play manager
 *          to either deploy the pre-existing components ( or in future versions,
 *          deploy the sent component)
 */


/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/node.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/linkedList.h"
#include "xme/wp/waypoint.h"

#include "xme/core/topicData.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \struct componentPortVertex_config_t
 *
 * \brief  Configuration entry for Components.
 */
typedef struct
{
    xme_core_component_portType_t portType; ///< Type of port, Subscription or Publication.
    xme_core_topic_t topic; ///< Topic of the Component Port Vertex.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t *port; ///< Data Packet ID of the Component Port Vertex.
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    uint8_t scheduleEarly; ///< Number of instances of components to be scheduled early.
    xme_core_componentType_t componentType; ///< Component type.
    xme_core_component_t componentId; ///< Component ID.
    bool toSchedule; ///< To schedule a new instance of the component or not.
} componentPortVertex_config_t;

/**
 * \struct marshaler_config_t
 *
 * \brief Configuration entry for marshaler waypoint.
 */
typedef struct 
{
    xme_core_topic_t topic; ///< Topic of the marshaler waypoint.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t *inputPort; ///< Data Packet ID of the input port.
    xme_core_dataManager_dataPacketId_t *outputPort; ///< Data Packet ID of the output port.
    xme_wp_waypoint_instanceId_t instanceId; ///< Waypoint instance ID.
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toSchedule; ///< To schedule a new instance of the waypoint or not.
} marshaler_config_t;

/**
 * \struct demarshaler_config_t
 *
 * \brief Configuration entry for demarshaler waypoint.
 */
typedef struct 
{
    xme_core_topic_t topic; ///< Topic of the demarshaler waypoint.
    uint16_t topicSize; ///< Size of the topic.
    xme_core_dataManager_dataPacketId_t *inputPort; ///< Data Packet ID of the input port.
    xme_core_dataManager_dataPacketId_t *outputPort; ///< Data Packet ID of the output port.
    xme_wp_waypoint_instanceId_t instanceId; ///< Waypoint instance ID.
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toSchedule; ///< To schedule a new instance of the waypoint or not.
} demarshaler_config_t;

/**
 * \struct udpSend_config_t
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
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component.
    void *buffer; ///< Pointer to the buffer used in the Send waypoint.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toSchedule; ///< To schedule a new instance of the waypoint or not.
} udpSend_config_t;

/**
 * \struct udpRecv_config_t
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
    xme_core_exec_functionDescriptor_t *fDescriptor; ///< Function descriptor of the scheduled component.
    void *buffer; ///< Pointer to the buffer used in the Receive waypoint.
    uint8_t totalToSchedule; ///< Number of instances to be scheduled.
    bool toSchedule; ///< To schedule a new instance of the waypoint or not.
} udpRecv_config_t;

/**
 * \struct portMapping_t
 *
 * \brief table for port mappings.
 */
typedef struct 
{
    xme_core_dataManager_dataPacketId_t *srcPort; ///< Source port. 
    xme_core_dataManager_dataPacketId_t *dstPort; ///< Destination port. 
} portMapping_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the plug and play client component.
 *         Exactly one component of this type must be present on every node.
 * 
 * \retval XME_SUCCESS if the plug and play manager component has been properly initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if plug and play manager component initialization failed.
 */ 
xme_status_t
xme_core_pnp_pnpClient_init(void);

/**
 * \brief  Frees all resources occupied by the plug and play client component.
 */
void 
xme_core_pnp_pnpClient_fini(void);

/**
 * \brief  Tells pnpClient of the components which are statically initialized on the node.
 * 
 * \param  portType Type of port, publisher or subscriber.
 * \param  topic Topic of the port.
 * \param  topicSize Size of the topic.
 * \param  port Data Packet ID of the port.
 * \param  componentType Type of the component.
 * \param  componentId Unique identifier of the component.
 * \param  fDescriptor Function descriptor of the scheduled component.
 *
 * \retval XME_SUCCESS if the entry is successfully added.
 * \retval XME_STATUS_OUT_OF_RESOURCES if it fails to add.
 */ 
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
);

/**
 * \brief  Tells pnpClient of the marshaler waypoint which is statically initialized on the node.
 * 
 * \param  inputPort Data Packet ID of the input port.
 * \param  outputPort Data Packet ID of the output port.
 * \param  topic Topic of the port.
 * \param  topicSize Size of the topic.
 * \param  instanceId waypoint instance Id.
 * \param  fDescriptor Function descriptor of the scheduled component.
 *
 * \retval XME_SUCCESS if the entry is successfully added.
 * \retval XME_STATUS_OUT_OF_RESOURCES if it fails to add.
 */ 
xme_status_t
xme_core_pnp_pnpClient_announceStaticMarshaler
(
    xme_core_dataManager_dataPacketId_t *inputPort,
    xme_core_dataManager_dataPacketId_t *outputPort,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_exec_functionDescriptor_t *fDescriptor
);

/**
 * \brief  Tells pnpClient of the demarshaler waypoint which is statically initialized on the node.
 * 
 * \param  inputPort Data Packet ID of the input port.
 * \param  outputPort Data Packet ID of the output port.
 * \param  topic Topic of the port.
 * \param  topicSize Size of the topic.
 * \param  instanceId waypoint instance Id.
 * \param  fDescriptor Function descriptor of the scheduled component.
 *
 * \retval XME_SUCCESS if the entry is successfully added.
 * \retval XME_STATUS_OUT_OF_RESOURCES if it fails to add.
 */ 
xme_status_t
xme_core_pnp_pnpClient_announceStaticDemarshaler
(
    xme_core_dataManager_dataPacketId_t *inputPort,
    xme_core_dataManager_dataPacketId_t *outputPort,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_exec_functionDescriptor_t *fDescriptor
);

/**
 * \brief  Tells pnpClient of the udpSend waypoint which is statically initialized on the node.
 * 
 * \param  dataPort Data Packet ID of the input port.
 * \param  key the key passed in packet header for uinique connection identification.
 * \param  destIP the destination IP where the packet is to be sent.
 * \param  ipPort the port at the destination IP where the packet is to be sent.
 * \param  topic Topic of the port.
 * \param  topicSize Size of the topic.
 * \param  instanceId waypoint instance Id.
 * \param  fDescriptor Function descriptor of the scheduled component.
 * \param  buffer Pointer to receieve buffer to be used by that instance.
 *
 * \retval XME_SUCCESS if the entry is successfully added.
 * \retval XME_STATUS_OUT_OF_RESOURCES if it fails to add.
 */ 
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
);

/**
 * \brief  Tells pnpClient of the udpReceive waypoint which is statically initialized on the node.
 * 
 * \param  dataPort Data Packet ID of the output port.
 * \param  key the key passed in packet header for uinique connection identification.
 * \param  ipPort the port where the XME listens for incoming requests.
 * \param  topic Topic of the port.
 * \param  topicSize Size of the topic.
 * \param  instanceId waypoint instance Id.
 * \param  fDescriptor Function descriptor of the scheduled component.
 * \param  buffer Pointer to receieve buffer to be used by that instance.
 *
 * \retval XME_SUCCESS if the entry is successfully added.
 * \retval XME_STATUS_OUT_OF_RESOURCES if it fails to add.
 */ 
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
);

/**
 * \brief  It accepts the graph received by pnpClient component and generates the required
 *         configurations and initialized the corresponding functions.
 * 
 * \param  portPnpGraphInData the graph received by the client and sent by the pnp Manager.
 */ 
xme_status_t
xme_core_pnp_pnpClient_processGraph
(
    const xme_core_topic_pnpManager_runtime_graph_model_t* portPnpGraphInData
);

/**
 * \brief  It accepts the graph received by pnpClient component and generates the required
 *         configurations and initialized the corresponding functions.
 * \param  manifest the component instance manifest.
 * \retval XME_STATUS_SUCCESS if the manifest is correctly received. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot process the manifest. 
 */ 
xme_status_t
xme_core_pnp_pnpClient_getManifest
(
    xme_core_topic_pnp_componentInstanceManifest_t* manifest
);
XME_EXTERN_C_END
/**
 * @}
 */

#endif // #ifndef XME_CORE_DIRECTORY_PLUGANDPLAYCLIENT_H
