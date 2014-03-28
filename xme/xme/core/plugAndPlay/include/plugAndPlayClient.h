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
 * $Id: plugAndPlayClient.h 7802 2014-03-13 09:04:01Z geisinger $
 */

/**
 * \file
 *         Plug and Play Client Header.
 */

#ifndef XME_CORE_PNP_PLUGANDPLAYCLIENT_H
#define XME_CORE_PNP_PLUGANDPLAYCLIENT_H

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
#include "xme/core/nodeManager/include/componentRepository.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/linkedList.h"
#include "xme/wp/waypoint.h"

#include "xme/core/topicData.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the plug and play client component.
 *        Exactly one component of this type must be present on every node.
 * 
 * \retval XME_STATUS_SUCCESS if the plug and play manager component has been properly initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if plug and play manager component initialization failed.
 */ 
xme_status_t
xme_core_pnp_pnpClient_init(void);

/**
 * \brief Frees all resources occupied by the plug and play client component.
 */
void 
xme_core_pnp_pnpClient_fini(void);

/**
 * \brief Tells pnpClient of the components which are statically initialized on the node.
 * 
 * \param portType Type of port, publisher or subscriber.
 * \param topic Topic of the port.
 * \param topicSize Size of the topic.
 * \param port Data Packet ID of the port.
 * \param componentType Type of the component.
 * \param componentId Unique identifier of the component.
 * \param initializationString Component type specific initialization string.
 * \param fDescriptor Function descriptor of the scheduled component.
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
    xme_core_dataManager_dataPacketId_t port,
    xme_core_componentType_t componentType,
    const char* const initializationString,
    xme_core_component_t componentId,
    xme_core_exec_functionDescriptor_t *fDescriptor
);

/**
 * \brief Tells pnpClient of the marshaler waypoint which is statically initialized on the node.
 * 
 * \param[in] inputPort Data Packet ID of the input port.
 * \param[in] outputPort Data Packet ID of the output port.
 * \param[in] topic Topic of the port.
 * \param[in] topicSize Size of the topic.
 * \param[in] instanceId waypoint instance Id.
 * \param[in] fDescriptor Function descriptor of the scheduled component.
 * \param[in] channelId The channel identifier. 
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
    xme_core_exec_functionDescriptor_t *fDescriptor,
    xme_core_channelId_t channelId
);

/**
 * \brief Tells pnpClient of the demarshaler waypoint which is statically initialized on the node.
 * 
 * \param[in] inputPort Data Packet ID of the input port.
 * \param[in] outputPort Data Packet ID of the output port.
 * \param[in] topic Topic of the port.
 * \param[in] topicSize Size of the topic.
 * \param[in] instanceId waypoint instance Id.
 * \param[in] fDescriptor Function descriptor of the scheduled component.
 * \param[in] channelId The channel identifier. 
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
    xme_core_exec_functionDescriptor_t *fDescriptor,
    xme_core_channelId_t channelId
);

/**
 * \brief Tells pnpClient of the udpSend waypoint which is statically initialized on the node.
 * 
 * \param[in] dataPort Data Packet ID of the input port.
 * \param[in] key the key passed in packet header for uinique connection identification.
 * \param[in] destIP the destination IP where the packet is to be sent.
 * \param[in] ipPort the port at the destination IP where the packet is to be sent.
 * \param[in] topic Topic of the port.
 * \param[in] topicSize Size of the topic.
 * \param[in] instanceId waypoint instance Id.
 * \param[in] fDescriptor Function descriptor of the scheduled component.
 * \param[in] buffer Pointer to receieve buffer to be used by that instance.
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
 * \brief Tells pnpClient of the udpReceive waypoint which is statically initialized on the node.
 * 
 * \param[in] dataPort Data Packet ID of the output port.
 * \param[in] key the key passed in packet header for uinique connection identification.
 * \param[in] ipPort the port where the XME listens for incoming requests.
 * \param[in] topic Topic of the port.
 * \param[in] topicSize Size of the topic.
 * \param[in] instanceId waypoint instance Id.
 * \param[in] fDescriptor Function descriptor of the scheduled component.
 * \param[in] buffer Pointer to receieve buffer to be used by that instance.
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
 * \brief Tells pnpClient of the ChannelInjector waypoint which is statically initialized on the node.
 * 
 * \param[in] inputPort Data Packet ID of the input port.
 * \param[in] outputPort Data Packet ID of the output port.
 * \param[in] srcChID the source channel id of the request
 * \param[in] topic Topic of the port.
 * \param[in] topicSize Size of the topic.
 * \param[in] instanceId waypoint instance Id.
 * \param[in] fDescriptor Function descriptor of the scheduled component.
 * \param[in] buffer Pointer to receieve buffer to be used by that instance.
 *
 * \retval XME_SUCCESS if the entry is successfully added.
 * \retval XME_STATUS_OUT_OF_RESOURCES if it fails to add.
 */ 
xme_status_t
xme_core_pnp_pnpClient_announceStaticChannelInjector
(
    xme_core_dataManager_dataPacketId_t *inputPort,
    xme_core_dataManager_dataPacketId_t *outputPort,
    xme_core_channelId_t srcChID,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_exec_functionDescriptor_t *fDescriptor,
    void *buffer
);

/**
 * \brief Tells pnpClient of the ChannelInjector waypoint which is statically initialized on the node.
 * 
 * \param[in] inputPort Data Packet ID of the input port.
 * \param[in] outputPort Data Packet ID of the output port.
 * \param[in] srcChID the source channel id of the request
 * \param[in] dstChID the destination channel id of the request
 * \param[in] topic Topic of the port.
 * \param[in] topicSize Size of the topic.
 * \param[in] instanceId waypoint instance Id.
 * \param[in] fDescriptor Function descriptor of the scheduled component.
 * \param[in] buffer Pointer to receieve buffer to be used by that instance.
 *
 * \retval XME_SUCCESS if the entry is successfully added.
 * \retval XME_STATUS_OUT_OF_RESOURCES if it fails to add.
 */ 
xme_status_t
xme_core_pnp_pnpClient_announceStaticChannelSelector
(
    xme_core_dataManager_dataPacketId_t *inputPort,
    xme_core_dataManager_dataPacketId_t *outputPort,
    xme_core_channelId_t srcChID,
    xme_core_channelId_t dstChID,
    xme_core_topic_t topic,
    uint16_t topicSize,
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_exec_functionDescriptor_t *fDescriptor,
    void *buffer
);

/**
 * \brief It accepts the graph received by pnpClient component and generates the required
 *        configurations and initialized the corresponding functions.
 * 
 * \param[in] portPnpGraphInData the graph received by the client and sent by the pnp Manager.
 * \param[out] outIsShutdown Address of a variable that is initialized with a
 *             boolean value specifying whether a node shutdown should be
 *             invoked according to the information in the given PnP graph.
 *
 * \retval XME_STATUS_SUCESS If the graph processing was success. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the input runtime graph contains invalid
 *         configuration data. 
 */ 
xme_status_t
xme_core_pnp_pnpClient_processGraph
(
    const xme_core_topic_pnpManager_runtime_graph_model_t* portPnpGraphInData,
    char *outIsShutdown
);

/**
 * \brief Fills the outManifest structure with information about components
 *        to instantiate according to previous calls to
 *        xme_core_pnp_pnpClient_instantiateComponentOnThisNode().
 *
 * \param[out] outManifest Address of a variable where to store the
 *             component instance manifest. Must not be NULL.
 * \param[in] includeCreatedComponents Whether the resulting manifest should
 *            include components that are already created. Core components
 *            are always excluded.
 *
 * \retval XME_STATUS_SUCCESS if there was at least one component to
 *         instantiate and the respective manifest was successfully stored.
 * \retval XME_STATUS_NOT_FOUND if no component instantiation is currently
 *         pending.
 * \retval XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to complete the operation.
 */ 
xme_status_t
xme_core_pnp_pnpClient_getManifest
(
    xme_core_topic_pnp_componentInstanceManifest_t* const outManifest,
    char includeCreatedComponents
);

/**
 * \brief Checks the action queue to check if there is at least one component remove request. 
 * \details Fills the outRequest with a single component remove request, if there is already
 *          available some DESTROY action in the PnPClient. 
 *          This request will be used by the function ::plugAndPlayClientSendRemoveComponentRequest. 
 *
 * \param[out] outRequest Address of a variable where to store the
 *             outgoing request. Must not be NULL.
 *
 * \retval XME_STATUS_SUCCESS if there was at least one component requested to
 *         be destroy.
 * \retval XME_STATUS_NOT_FOUND if no component destroy request is currently
 *         pending.
 * \retval XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to complete the operation.
 */ 
xme_status_t
xme_core_pnp_pnpClient_getComponentToRemove
(
    xme_core_topic_pnp_removeComponentRequest_t* const outRequest
);

/**
 * \brief Arranges for a component of the given type to be instantiated on this node.
 *
 * \details This function returns immediately.
 *          Instantiation of components happens asynchronously.
 *
 * \deprecated Use xme_core_pnp_pnpClient_plugInNewComponent() instead.
 *
 * \param[in] componentType Type of component to instantiate.
 *            The special value  XME_CORE_COMPONENT_TYPE_INVALID can be used to
 *            arrange that all components known in the manifest repository are
 *            instantiated once. In this case, the given initializationString
 *            is passed to all instances.
 * \param[in] initializationString String value passed to the component
 *            initialization functions of the component(s) to be instantiated.
 *            This allows to pass configuration values in a component-specific
 *            format to the component instance(s).
 *
 * \retval XME_STATUS_SUCCESS instantiation if the request to instantiate a
 *         component of the given type has been registered.
 * \retval XME_STATUS_INVALID_PARAMETER if this node does not support the
 *         given component type.
 * \retval XME_STATUS_TEMPORARY_FAILURE if the request to instantiate the
 *         component could not be processed, because too many requests are
 *         currently pending. Try again later.
 */
xme_status_t
xme_core_pnp_pnpClient_instantiateComponentOnThisNode
(
    xme_core_componentType_t componentType,
    const char* initializationString
);

/**
 * \brief Plug in a new component.
 *
 * \details To create the component use a component builder, see
 *          xme_core_nodeMgr_compRep_createBuilder().
 *          This function will change the component state to
 *          XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED and announce it to
 *          the plug and play manager. The component will be created as soon as
 *          its connection constraints are met.
 *
 * \param[in] componentHandle Handle of a component in state
 *            XME_CORE_NODEMGR_COMPREP_STATE_PREPARED.
 *
 * \retval XME_STATUS_SUCCESS if operation was successful.
 * \retval XME_STATUS_INVALID_CONFIGURATION if given component is in an invalid
 *         (wrong sate, does not exist, its node ID is not the local one).
 * \retval XME_STATUS_OUT_OF_RESOURCES if the system has not enough resources
 *         to complete the opration.
 */
xme_status_t
xme_core_pnp_pnpClient_plugInNewComponent
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
);

/**
 * \brief Arranges for the component instance with the given identifier to be
 *        destroyed.
 *
 * \details This function returns immediately.
 *          Destruction of the component happens asynchronously.
 *          The run-time system ensures that the component is not executed
 *          when is is being destroyed.
 *
 * \param[in] componentHandle Handle of component to destroy.
 *
 * \retval XME_STATUS_SUCCESS instantiation if the request to destroy the
 *         given component instance has been registered.
 * \retval XME_STATUS_INVALID_HANDLE if the given component identifier
 *         is invalid.
 * \retval XME_STATUS_TEMPORARY_FAILURE if the request to instantiate the
 *         component could not be processed, because too many requests are
 *         currently pending. Try again later.
 */
xme_status_t
xme_core_pnp_pnpClient_destroyComponentOnThisNode
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
);

/**
 * \brief Adds a node to the list of nodes to be logged out.
 *
 * \param[in] nodeId Valid nodeId which needs to be logged out.
 *
 * \retval XME_STATUS_SUCCESS if the node is successfully successfully added.
 * \retval XME_STATUS_INVALID_PARAMETER if nodeId is invalid.
 */
xme_status_t
xme_core_pnp_pnpClient_logoutNode
(
    xme_core_node_nodeId_t nodeId
);

/**
 * \brief Adds this node to the list of nodes to be logged out.
 *
 * \retval XME_STATUS_SUCCESS if the node is successfully successfully added.
 */
xme_status_t
xme_core_pnp_pnpClient_logoutThisNode
(
    void
);

/**
 * \brief Retrieves the list of nodes which are to be logged out.
 *
 * \note Although it deals with link lists but there is a bug in the stack
 *       which is we need to complete write operation for every write.
 *       So effectively we cannot process the whole list in one go.
 *       So one solution is get one node at a time. This has a flaw that the
 *       if the node was asked to logout in the current cycle, the actual
 *       logout will happen in a later cycle.
 *
 * \param[in] nodeList List of node identifiers that need to be logged out. 
 *
 * \retval XME_STATUS_SUCCESS if the node is successfully successfully added.
 * \retval XME_STATUS_INVALID_PARAMETER if nodeId is invalid.
 */ 
xme_status_t
xme_core_pnp_pnpClient_getLoggedoutNodes
(
    void* nodeList
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_PLUGANDPLAYCLIENT_H
