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
 * $Id: waypointConfigInfrastructure.c 7702 2014-03-06 23:22:57Z ruiz $
 */

/**
 * \file
 *         Waypoint Support Infrastructure
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/wp/waypointConfigInfrastructure.h"

#include "xme/core/login/include-gen/loginManagerComponentWrapper.h"
#include "xme/core/plugAndPlay/include-gen/plugAndPlayManagerComponentWrapper.h"
#include "xme/core/plugAndPlay/include-gen/plugAndPlayClientComponentWrapper.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
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
    void** buffer,
    bool isBroadcast
);

/**
 * \brief Create marshaler waypoint instance.
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
createMarshalerWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

// Defined in generated node file.
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
 * \brief Create demarshaler waypoint instance.
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

// Defined in generated node file.
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
 * \brief Create udp receive waypoint instance.
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
createUdpReceiveWaypointInstance
(
    xme_core_exec_functionDescriptor_t** descriptor,
    xme_core_component_t componentId
);

/**
 * \brief Adds a configuration entry to the udp receive waypoint.
 *
 * \details Creates and adds configuration structure for udp receive waypoint.
 *          The required port and buffer are also created.
 *
 * \param descriptor Function descriptor of this waypoint.
 * \param dataPort Pointer to port id that will be used for the created port of the configuration.
 * \param key See key parameter of xme_wp_udp_udpReceive_addConfig.
 * \param ipPort See port parameter of xme_wp_udp_udpReceive_addConfig.
 * \param topic Topic for this configuration.
 * \param sizeOfTopic Size of the topic data structure.
 * \param instanceId See instanceId parameter of xme_wp_udp_udpReceive_addConfig.
 * \param recvBuffer See buffer parameter of xme_wp_udp_udpReceive_addConfig.
 *
 * \retval XME_STATUS_SUCCESS when no errors occurred.
 * \retval XME_STATUS_INTERNAL_ERROR when an error occured.
 */
extern xme_status_t
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

/**
 * \brief Creates a link key for UDP waypoints.
 *
 * \param[in] id the identifier to create the link key. 
 * \param[out] bArray the output array. 
 */
static void
createLinkKey
(
    uint32_t id,
    uint8_t* bArray
);

/**
 * \brief  Initializes a component descriptor. This is required for adding schedule to 
 *         Execution manager for the newly created component/waypoint.
 *
 * \param  fDesc The function descriptor which needs to be inserted in the schedule.
 *
 * \return Pointer to the newly created component descriptor.
 */
static xme_core_exec_componentDescriptor_t *
getComponentDescriptor
(
    const xme_core_exec_functionDescriptor_t *fDesc
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
static void
createLinkKey
(
    uint32_t id,
    uint8_t* bArray
)
{
    int i=0;

    bArray[i++] = (uint8_t)(id >> 24);
    bArray[i++] = (uint8_t)(id >> 16);
    bArray[i++] = (uint8_t)(id >> 8);
    bArray[i++] = (uint8_t)(id >> 0);
}

static xme_core_exec_componentDescriptor_t *
getComponentDescriptor
(
    const xme_core_exec_functionDescriptor_t *fDesc
)
{
    xme_core_exec_componentDescriptor_t *temp = (xme_core_exec_componentDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_componentDescriptor_t));
    if(NULL == temp || NULL == fDesc)
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
    xme_status_t status;
    xme_core_exec_transactionId_t trxnId = (xme_core_exec_transactionId_t) 11;
    xme_core_exec_componentDescriptor_t *cDesc = NULL;

    xme_core_dataManager_dataPacketId_t inputDataPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, outputDataPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    xme_core_dataManager_dataPacketId_t dataPortUdp = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    xme_core_dataManager_dataPacketId_t componentPort = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

    xme_wp_waypoint_instanceId_t marshalerId = XME_WP_WAYPOINT_INSTANCEID_INVALID, udpId = XME_WP_WAYPOINT_INSTANCEID_INVALID; // marshalerId is used for marshaler and demarshaler

    xme_core_exec_functionDescriptor_t *fDescriptorUdp = NULL;
    xme_core_exec_functionDescriptor_t *fDescriptor = NULL;

    uint8_t key[XME_WP_UDP_HEADER_KEY_LENGTH];
    void *buffer=NULL;
    const char* component = NULL; // Used for debug message only. Set to the name of the component(s) that is expected to have called xme_wp_wci_addConfig().
    const char* stringTopic = NULL;
    char hostname[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
    uint16_t port = xme_hal_net_ntohs(portAddress), sizeOfTopic = 0;

    XME_CHECK
    (
        0 < xme_hal_safeString_snprintf(hostname, XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE, "%u.%u.%u.%u",
                            (uint8_t)(0xFF & ipAddress),
                            (uint8_t)(0xFF & (ipAddress>>8) ),
                            (uint8_t)(0xFF & (ipAddress>>16) ),
                            (uint8_t)(0xFF & (ipAddress>>24) ) ),
        XME_STATUS_INTERNAL_ERROR
    );

    createLinkKey((uint32_t)channelID, key);
    //Ideally we should use xme_core_topicUtil_getTopicSize but that is coming from XMT generated
    //code this should change when we update auto generation of marshaler
    if (XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL == topic)
    {
        xme_core_nodeMgr_compRep_componentHandle_t pnpManagerHandle;
        xme_core_nodeMgr_compRep_portHandle_t portHandle;

        sizeOfTopic = sizeof(xme_core_topic_pnpManager_runtime_graph_model_t);
        component = "PnPManager";
        stringTopic = "XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL";
        
        status = xme_core_nodeMgr_compRep_getComponentInstance
        (
            XME_CORE_NODE_LOCAL_NODE_ID,
            XME_CORE_COMPONENT_ID_PNPMANAGER,
            &pnpManagerHandle
        );
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        portHandle = xme_core_nodeMgr_compRep_getPort(pnpManagerHandle, XME_CORE_PNP_PNPMANAGERCOMPONENTWRAPPER_PORT_OUTRUNTIMEGRAPH);
        componentPort = xme_core_nodeMgr_compRep_getDataPacketID(portHandle);
    }
    else if (XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST == topic)
    {
        sizeOfTopic = sizeof(xme_core_topic_pnp_componentInstanceManifest_t);
        stringTopic = "XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST";
        if (0 == ipAddress)
        {
            xme_core_nodeMgr_compRep_componentHandle_t pnpManagerHandle;
            xme_core_nodeMgr_compRep_portHandle_t portHandle;

            component = "PnpManager";
        
            status = xme_core_nodeMgr_compRep_getComponentInstance
            (
                XME_CORE_NODE_LOCAL_NODE_ID,
                XME_CORE_COMPONENT_ID_PNPMANAGER,
                &pnpManagerHandle
            );
            XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
            portHandle = xme_core_nodeMgr_compRep_getPort(pnpManagerHandle, XME_CORE_PNP_PNPMANAGERCOMPONENTWRAPPER_PORT_INCOMPONENTINSTANCEMANIFEST);
            componentPort = xme_core_nodeMgr_compRep_getDataPacketID(portHandle);
        }
        else
        {
            xme_core_nodeMgr_compRep_componentHandle_t pnpClientHandle;
            xme_core_nodeMgr_compRep_portHandle_t portHandle;

            component = "LoginClient/PnPClient";
            
            status = xme_core_nodeMgr_compRep_getComponentInstance
            (
                XME_CORE_NODE_LOCAL_NODE_ID,
                XME_CORE_COMPONENT_ID_PNPCLIENT,
                &pnpClientHandle
            );
            XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
            portHandle = xme_core_nodeMgr_compRep_getPort(pnpClientHandle, XME_CORE_PNP_PNPCLIENTCOMPONENTWRAPPER_PORT_OUTMANIFEST);
            componentPort = xme_core_nodeMgr_compRep_getDataPacketID(portHandle);
        }
        
    }
    else if (XME_CORE_TOPIC_LOGIN_LOGINRESPONSE == topic)
    {
        xme_core_nodeMgr_compRep_componentHandle_t loginManagerHandle;
        xme_core_nodeMgr_compRep_portHandle_t portHandle;

        sizeOfTopic = sizeof(xme_core_topic_login_loginResponse_t);
        component = "LoginManager";
        stringTopic = "XME_CORE_TOPIC_LOGIN_LOGINRESPONSE";
        
        status = xme_core_nodeMgr_compRep_getComponentInstance
        (
            XME_CORE_NODE_LOCAL_NODE_ID,
            XME_CORE_COMPONENT_ID_LOGINMANAGER,
            &loginManagerHandle
        );
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        portHandle = xme_core_nodeMgr_compRep_getPort(loginManagerHandle, XME_CORE_LOGIN_LOGINMANAGERCOMPONENTWRAPPER_PORT_OUTLOGINRESPONSE);
        componentPort = xme_core_nodeMgr_compRep_getDataPacketID(portHandle);
    }
    else if (XME_CORE_TOPIC_PNP_LOGOUTREQUEST == topic)
    {
        xme_core_nodeMgr_compRep_componentHandle_t pnpClientHandle;
        xme_core_nodeMgr_compRep_portHandle_t portHandle;

        sizeOfTopic = sizeof(xme_core_topic_pnp_logoutRequest_t);
        component = "LoginClient";
        stringTopic = "XME_CORE_TOPIC_PNP_LOGOUTREQUEST";

        status = xme_core_nodeMgr_compRep_getComponentInstance
        (
            XME_CORE_NODE_LOCAL_NODE_ID,
            XME_CORE_COMPONENT_ID_PNPCLIENT,
            &pnpClientHandle
        );
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        portHandle = xme_core_nodeMgr_compRep_getPort(pnpClientHandle, XME_CORE_PNP_PNPCLIENTCOMPONENTWRAPPER_PORT_OUTLOGOUTREQUEST);
        componentPort = xme_core_nodeMgr_compRep_getDataPacketID(portHandle);
    }
    else if (XME_CORE_TOPIC_PNP_LOGOUTACKNOWLEDGMENT == topic)
    {
        xme_core_nodeMgr_compRep_componentHandle_t pnpClientHandle;
        xme_core_nodeMgr_compRep_portHandle_t portHandle;

        sizeOfTopic = sizeof(xme_core_topic_pnp_logoutAcknowledgment_t);
        component = "LoginClient";
        stringTopic = "XME_CORE_TOPIC_PNP_LOGOUTACKNOWLEDGMENT";
        
        status = xme_core_nodeMgr_compRep_getComponentInstance
        (
            XME_CORE_NODE_LOCAL_NODE_ID,
            XME_CORE_COMPONENT_ID_PNPCLIENT,
            &pnpClientHandle
        );
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        portHandle = xme_core_nodeMgr_compRep_getPort(pnpClientHandle, XME_CORE_PNP_PNPCLIENTCOMPONENTWRAPPER_PORT_OUTLOGOUTACKNOWLEDGMENT);
        componentPort = xme_core_nodeMgr_compRep_getDataPacketID(portHandle);
    }
    else
    {
        return XME_STATUS_INVALID_CONFIGURATION;
    }
   

    if (0 == ipAddress)
    { //A case of UDP recv

        // Add a Demarshaler waypoint.
        // And get its function descriptor.
        status = createDemarshalerWaypointInstance(&fDescriptor, (xme_core_component_t) XME_CORE_COMPONENT_ID_WAYPOINT_DEMARSHALER);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        status = demarshalerWaypointAddConfig(fDescriptor, &inputDataPort, 1u, &outputDataPort, &marshalerId, topic, sizeOfTopic, channelID); 
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION);

        cDesc = getComponentDescriptor(fDescriptor);
        status = xme_core_exec_componentRepository_registerComponent(cDesc);
        XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, XME_STATUS_INTERNAL_ERROR);
        status = xme_core_exec_configurator_addComponentToSchedule(trxnId, fDescriptor->componentId, fDescriptor->functionId,
                                                                       (void *)(uintptr_t)(marshalerId), 0, 0, 0, 0, NULL);
        XME_CHECK_REC
        (
            XME_STATUS_SUCCESS == status, status,
            {
                XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_rollback(trxnId), XME_STATUS_INTERNAL_ERROR);
            }
        );

        // Add a udpRecv waypoint.
        // And get its function descriptor.
        status = createUdpReceiveWaypointInstance(&fDescriptorUdp, (xme_core_component_t) XME_CORE_COMPONENT_ID_WAYPOINT_UDPRECEIVE);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
        status = udpReceiveWaypointAddConfig(fDescriptorUdp, &dataPortUdp, key, port, topic,
                                        sizeOfTopic, &udpId, &buffer);
        XME_CHECK_REC
        (
            XME_STATUS_SUCCESS == status, status,
            {
                XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_rollback(trxnId), XME_STATUS_INTERNAL_ERROR);
            }
        );


        cDesc = getComponentDescriptor(fDescriptorUdp);
        status = xme_core_exec_componentRepository_registerComponent(cDesc);
        XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, XME_STATUS_INTERNAL_ERROR);
        status = xme_core_exec_configurator_addComponentToSchedule(trxnId, fDescriptorUdp->componentId, fDescriptorUdp->functionId,
                                                                       (void *)(uintptr_t)(udpId), 0, 0, 0, 0, NULL);

        //If we do not schedule the component successfully does it make sense to continue
        //for we will not have a udp config to send the RT Graph to the node.
        if (XME_STATUS_SUCCESS != status)
        {
            XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_rollback(trxnId), XME_STATUS_INTERNAL_ERROR);
            return XME_STATUS_INVALID_CONFIGURATION;
        }
        else 
        {
            XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_commit(trxnId), XME_STATUS_INTERNAL_ERROR);
            XME_CHECK(XME_STATUS_SUCCESS == xme_core_broker_addDataPacketTransferEntry(dataPortUdp, inputDataPort), XME_STATUS_INTERNAL_ERROR);
            XME_CHECK(XME_STATUS_SUCCESS == xme_core_broker_addDataPacketTransferEntry(outputDataPort, componentPort), XME_STATUS_INTERNAL_ERROR);
        }

        XME_LOG(XME_LOG_DEBUG, "[%s] Added %s:%d for %s [%d] with channel ID %d\n", component, hostname, port, stringTopic, topic, channelID);

        return XME_STATUS_SUCCESS;
    }
    // This is udpSend case
    // Add a marshaler waypoint.
    // And get its function descriptor.
    status = createMarshalerWaypointInstance(&fDescriptor, (xme_core_component_t) XME_CORE_COMPONENT_ID_WAYPOINT_MARSHALER);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    status = marshalerWaypointAddConfig(fDescriptor, &inputDataPort, 1u, &outputDataPort, &marshalerId, topic, sizeOfTopic, channelID); 
    XME_CHECK(XME_STATUS_SUCCESS==status, XME_STATUS_INVALID_CONFIGURATION);

    cDesc = getComponentDescriptor(fDescriptor);
    status = xme_core_exec_componentRepository_registerComponent(cDesc);
    XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, XME_STATUS_INTERNAL_ERROR);
    status = xme_core_exec_configurator_addComponentToSchedule(trxnId, fDescriptor->componentId, fDescriptor->functionId,
                                                                   (void *)(uintptr_t)(marshalerId), 0, 0, 0, 0, NULL);
    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS == status, status,
        {
            XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_rollback(trxnId), XME_STATUS_INTERNAL_ERROR);
        }
    );

    // Add a udpSend waypoint.
    // And get its function descriptor.
    status = createUdpSendWaypointInstance(&fDescriptorUdp, (xme_core_component_t) XME_CORE_COMPONENT_ID_WAYPOINT_UDPSEND);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    status = udpSendWaypointAddConfig(fDescriptorUdp, &dataPortUdp, key, hostname, port, topic,
                            sizeOfTopic, &udpId, &buffer, false);
    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS == status, status,
        {
            XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_rollback(trxnId), XME_STATUS_INTERNAL_ERROR);
        }
    );


    cDesc = getComponentDescriptor(fDescriptorUdp);
    status = xme_core_exec_componentRepository_registerComponent(cDesc);
    XME_CHECK(XME_STATUS_SUCCESS == status || XME_STATUS_ALREADY_EXIST == status, XME_STATUS_INTERNAL_ERROR);
    status = xme_core_exec_configurator_addComponentToSchedule(trxnId, fDescriptorUdp->componentId, fDescriptorUdp->functionId,
                                                                   (void *)(uintptr_t)(udpId), 0, 0, 0, 0, NULL);

    //If we do not schedule the component successfully does it make sense to continue
    //for we will not have a udp config to send the RT Graph to the node.
    if (XME_STATUS_SUCCESS != status)
    {
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_rollback(trxnId), XME_STATUS_INTERNAL_ERROR);
        return XME_STATUS_INVALID_CONFIGURATION;
    }
    else 
    {
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_configurator_commit(trxnId), XME_STATUS_INTERNAL_ERROR);
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_broker_addDataPacketTransferEntry(componentPort, inputDataPort), XME_STATUS_INTERNAL_ERROR);
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_broker_addDataPacketTransferEntry(outputDataPort, dataPortUdp), XME_STATUS_INTERNAL_ERROR);
    }

    XME_LOG(XME_LOG_DEBUG, "[%s] Added %s:%d for %s [%d] with channel ID %d\n", component, hostname, port, stringTopic, topic, channelID);

    return XME_STATUS_SUCCESS;
}
