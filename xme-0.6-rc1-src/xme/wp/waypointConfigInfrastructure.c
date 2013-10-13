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
 * $Id: waypointConfigInfrastructure.c 5364 2013-10-01 17:19:56Z wiesmueller $
 */

/**
 * \file
 *         Waypoint Support Infrastructure
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/wp/waypointConfigInfrastructure.h"

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
    void *buffer,
    bool isBroadcast
);

/**
 * \brief Creates a link key for UDP waypoints.
 *
 * \param[in] topicId the topic id. 
 * \param[out] bArray the output array. 
 */
static void
createLinkKey
(
    xme_core_topic_t topicId,
    uint8_t* bArray
)
{
    int i=0;

    bArray[i++] = 0;
    bArray[i++] = 0;
    bArray[i++] = (uint8_t)(topicId >> 8);
    bArray[i]   = (uint8_t) topicId;
}

/**
 * \brief  Initializes a component descriptor. This is required for adding schedule to 
 *         Execution manager for the newly created component/waypoint.
 *
 * \param  fDesc The function descriptor which needs to be inserted in the schedule.
 *
 * \return Pointer to the newly created component descriptor.
 */
static xme_core_exec_componentDescriptor_t *
getComponentDescriptor(xme_core_exec_functionDescriptor_t *fDesc)
{
    xme_core_exec_componentDescriptor_t *temp = (xme_core_exec_componentDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_exec_componentDescriptor_t));
    if(NULL == temp || NULL == fDesc)
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

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_wp_wci_addConfig
(
    uint32_t ipAddress,
    uint16_t portAddress,
    xme_core_topic_t topic
)
{
    xme_status_t status;
    xme_core_dataManager_dataPacketId_t inputDataPortMarshaler = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, outputDataPortMarshaler = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    xme_wp_waypoint_instanceId_t marshalerId = XME_WP_WAYPOINT_INSTANCEID_INVALID;
    char hostname[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
    uint16_t port = xme_hal_net_ntohs(portAddress);
    xme_hal_safeString_snprintf(hostname, XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE, "%u.%u.%u.%u",
                            (uint8_t)(0xFF & ipAddress),
                            (uint8_t)(0xFF & (ipAddress>>8) ),
                            (uint8_t)(0xFF & (ipAddress>>16) ),
                            (uint8_t)(0xFF & (ipAddress>>24) ) );

    status = xme_wp_marshal_marshaler_getConfig(&marshalerId, &topic, &inputDataPortMarshaler, &outputDataPortMarshaler);
    //we should have a marshaler present otherwise do not add udp config
    if (XME_STATUS_SUCCESS == status)
    {
        uint8_t key[XME_WP_UDP_HEADER_KEY_LENGTH];
        xme_core_exec_transactionId_t trxnId = (xme_core_exec_transactionId_t) 11;
        xme_core_exec_functionDescriptor_t *fDescriptor = NULL;
        xme_core_dataManager_dataPacketId_t dataPortUdpSend = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
        xme_wp_waypoint_instanceId_t udpSendId = XME_WP_WAYPOINT_INSTANCEID_INVALID;
        xme_core_exec_componentDescriptor_t *cDesc = NULL;

        createLinkKey((xme_core_topic_t)topic,key);
        if (XME_STATUS_NOT_FOUND == xme_wp_udp_udpSend_getConfig(&udpSendId, &dataPortUdpSend, key, hostname, port))
        {
            void *buffer=NULL;
            uint16_t sizeOfTopic = 0;
            char component[24];
            char stringTopic[50];

            //Ideally we should use xme_core_topicUtil_getTopicSize but that is coming from XMT generated
            //code this should change when we update auto generation of marshaler
            if (XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL == topic)
            {
                sizeOfTopic = sizeof(xme_core_topic_pnpManager_runtime_graph_model_t);
                xme_hal_safeString_strncpy(component, "PnPManager", sizeof(component));
                xme_hal_safeString_strncpy(stringTopic, "XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL", sizeof(stringTopic));
            }
            else if (XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST == topic)
            {
                sizeOfTopic = sizeof(xme_core_topic_pnp_componentInstanceManifest_t);
                xme_hal_safeString_strncpy(component, "LoginClient", sizeof(component));
                xme_hal_safeString_strncpy(stringTopic, "XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST", sizeof(stringTopic));
            }
            else if (XME_CORE_TOPIC_LOGIN_LOGINRESPONSE == topic)
            {
                sizeOfTopic = sizeof(xme_core_topic_login_loginResponse_t);
                xme_hal_safeString_strncpy(component, "LoginManager", sizeof(component));
                xme_hal_safeString_strncpy(stringTopic, "XME_CORE_TOPIC_LOGIN_LOGINRESPONSE", sizeof(stringTopic));
            }
          
            // 2 is just a number used. Ideally it should already be initialized
            // and this call should just return the function descriptor already initialized
            createUdpSendWaypointInstance(&fDescriptor, (xme_core_component_t) 2);
            status = udpSendWaypointAddConfig(fDescriptor, &dataPortUdpSend, key, hostname, port, topic,
                                    sizeOfTopic, &udpSendId, buffer, false);
            XME_CHECK(XME_STATUS_SUCCESS==status, XME_STATUS_INVALID_CONFIGURATION);
            XME_LOG
            (
                XME_LOG_NOTE, 
                "[%s] Added %s:%d for %s \n",
                component,
                hostname,
                port,
                stringTopic
            );
            cDesc = getComponentDescriptor(fDescriptor);
            xme_core_exec_componentRepository_registerComponent(cDesc);
            status = xme_core_exec_configurator_addComponentToSchedule(trxnId, fDescriptor->componentId, fDescriptor->functionId,
                                                                        (void *)(uintptr_t)(udpSendId), 0, 0, 0, 0, NULL);

            //If we do not schedule the component successfully does it make sense to continue
            //for we will not have a udp config to send the RT Graph to the node.
            if (XME_STATUS_SUCCESS != status)
            {
                xme_core_exec_configurator_rollback(trxnId);
                return XME_STATUS_INVALID_CONFIGURATION;
            }
            else 
            {
                xme_core_exec_configurator_commit(trxnId);
                xme_core_broker_addDataPacketTransferEntry(outputDataPortMarshaler, dataPortUdpSend);
            }

        }
        return XME_STATUS_SUCCESS;
    }
    return XME_STATUS_INVALID_CONFIGURATION;
}
