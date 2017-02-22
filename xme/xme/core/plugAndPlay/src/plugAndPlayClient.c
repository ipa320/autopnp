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
 * $Id: plugAndPlayClient.c 7838 2014-03-14 12:38:35Z geisinger $
 */

/**
 * \file
 *         Plug and Play Client Source File.
 *         This file covers the public interfaces of the PnPClient Component.
 *         Apart from the regular init and fini functions it has functions
 *         related to registeration of components run on the node from the
 *         node file. Also it has the main function of processing the
 *         incoming graph, constructing the component manifest and creating
 *         the list of components to be sent in the component manifest.
 */

/**
 * \addtogroup core_pnp_pnpClient
 * @{
 */
/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"
#include "xme/core/plugAndPlay/include/plugAndPlayClientConfiguration.h"
#include "xme/core/plugAndPlay/include/plugAndPlayClientScheduling.h"
#include "xme/core/plugAndPlay/include/plugAndPlayClientInternalTypes.h"

#include "xme/core/plugAndPlay/include-gen/pnpClientManifest.h"

#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpClientInterface.h"

#include "xme/core/logUtils.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \enum xme_core_pnp_pnpClient_componentActionType_e
 *
 * \brief Enumeration of component actions to perform.
 *
 * \see xme_core_pnp_pnpClient_componentActionType_t
 */
enum xme_core_pnp_pnpClient_componentActionType_e
{
    XME_CORE_PNP_PNPCLIENT_COMPONENT_ACTION_TYPE_INSTANTIATE,
    XME_CORE_PNP_PNPCLIENT_COMPONENT_ACTION_TYPE_DESTROY
};

/**
 * \typedef xme_core_pnp_pnpClient_componentActionType_t
 *
 * \brief Type for platform-independent representation of enumeration values
 *        declared in xme_core_pnp_pnpClient_componentActionType_e.
 *
 * \see xme_core_pnp_pnpClient_componentActionType_e
 */
typedef uint32_t xme_core_pnp_pnpClient_componentActionType_t;

/**
 * \struct xme_core_pnp_pnpClient_componentAction_t
 *
 * \brief Component action information.
 */
typedef struct
{
    xme_core_pnp_pnpClient_componentActionType_t actionType; ///< The action type.
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle; ///< The component handle.
} xme_core_pnp_pnpClient_componentAction_t;

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/

static xme_hal_singlyLinkedList_t(XME_CORE_PNP_PNPCLIENT_MAX_COMPONENT_ACTIONS) listOfComponents;
static xme_hal_singlyLinkedList_t(XME_CORE_PNP_PNPCLIENT_MAX_NODE_ACTIONS) listOfLoggedOutNodes;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Schedules a component for instantiation or destruction by appending
 *        it to the listOfComponents with the appropriate action type.
 *
 * \param[in] componentHandle Handle of the component to instantiate/destroy.
 * \param[in] actionType The action type.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_OUT_OF_RESOURCE if there is not enough memory to create
 *         a new entry in the list.
 * \retval XME_STATUS_TEMPORARY_FAILURE if not enough resources were available
 *         to complete the operation, that is, too many component action
 *         requests are currently pending.
 */
xme_status_t
scheduleComponentForInstantiationOrDestruction
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    xme_core_pnp_pnpClient_componentActionType_t actionType
);

/**
 * \brief Clears the list of components to instantiate or destroy
 *        (the listOfComponents list) and frees all allocated resources.
 */
static void
clearComponentActions(void);

/**
 * \brief Processes the Run Time Graph to generate the routes to be added to the
 *        node.
 * \param[in] portPnpGraphInData The runtime graph containing the routes to add. 
 *
 * \retval XME_STATUS_SUCCESS If the route removal was success. 
 */
static xme_status_t
addRoutes
(
    const xme_core_topic_pnpManager_runtime_graph_model_t* portPnpGraphInData
);

/**
 * \brief Processes the Run Time Graph to generate the routes to be removed from
 *        the node.
 *
 * \param[in] portPnpGraphInData The runtime graph containing the routes to remove. 
 *
 * \retval XME_STATUS_SUCCESS If the route removal was success. 
 */
static xme_status_t
removeRoutes
(
    const xme_core_topic_pnpManager_runtime_graph_model_t* portPnpGraphInData
);

/**
 * \brief Helper function for xme_core_pnp_pnpClient_getManifest() that
 *        adds a a new component entry to the instance manifest.
 *
 * \param[out] outManifest The manifest where we add the componetn.
 * \param[in] idx Index in components array of manifest where to add the component.
 * \param[in] componentHandle The handle of the component that is added to the manifest.
 */
static void
addToManifest
(
    xme_core_topic_pnp_componentInstanceManifest_t* const outManifest,
    xme_hal_linkedList_index_t idx,
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
);

/******************************************************************************/
/***   Helper Functions                                                     ***/
/******************************************************************************/

xme_status_t
scheduleComponentForInstantiationOrDestruction
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    xme_core_pnp_pnpClient_componentActionType_t actionType
)
{
    xme_status_t status;

    xme_core_pnp_pnpClient_componentAction_t* action =
        (xme_core_pnp_pnpClient_componentAction_t*) xme_hal_mem_alloc(sizeof(xme_core_pnp_pnpClient_componentAction_t));
    XME_CHECK(NULL != action, XME_STATUS_OUT_OF_RESOURCES);

    action->actionType = actionType;
    action->componentHandle = componentHandle;

    status = xme_hal_singlyLinkedList_addItem(&listOfComponents, (void*) action);
    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS == status,
        XME_STATUS_TEMPORARY_FAILURE,
        {
            xme_hal_mem_free(action);
        }
    );
    
    return XME_STATUS_SUCCESS;
}

static void
clearComponentActions(void)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(listOfComponents, xme_core_pnp_pnpClient_componentAction_t, action);
    {
        xme_hal_mem_free(action);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    
    xme_hal_singlyLinkedList_clear(&listOfComponents);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_pnp_pnpClient_init(void)
{
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_log_logUtils_init(), XME_STATUS_OUT_OF_RESOURCES);

    XME_HAL_TABLE_INIT(xme_core_pnp_pnpClientConfiguration_CPV_config_table);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpClientConfiguration_marshaler_config_table);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpClientConfiguration_udpSend_config_table);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table);
    XME_HAL_TABLE_INIT(xme_core_pnp_pnpClientConfiguration_portMapping_table);
    XME_HAL_SINGLYLINKEDLIST_INIT(listOfComponents);
    XME_HAL_SINGLYLINKEDLIST_INIT(listOfLoggedOutNodes);
    xme_core_pnp_pnpClientScheduling_init();

    return XME_STATUS_SUCCESS;
}

void
xme_core_pnp_pnpClient_fini(void)
{
    xme_status_t status = xme_core_exec_fini();
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
    
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpClientConfiguration_CPV_config_table);
    
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_marshaler_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_marshaler_config_t, configItem);
    {
        xme_hal_mem_free(configItem->fDescriptor);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpClientConfiguration_marshaler_config_table);
    
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_demarshaler_config_t, configItem);
    {
        xme_hal_mem_free(configItem->fDescriptor);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table);
    
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_udpSend_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_udpSend_config_t, configItem);
    {
        xme_hal_mem_free(configItem->fDescriptor);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpClientConfiguration_udpSend_config_table);
    
    XME_HAL_TABLE_ITERATE_BEGIN(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, xme_hal_table_rowHandle_t, rh, xme_core_pnp_pnpClientConfiguration_udpRecv_config_t, configItem);
    {
        xme_hal_mem_free(configItem->fDescriptor);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table);
    
    clearComponentActions();
    XME_HAL_SINGLYLINKEDLIST_FINI(listOfComponents);

    XME_HAL_SINGLYLINKEDLIST_FINI(listOfLoggedOutNodes);

    xme_core_log_logUtils_fini();
}

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
)
{
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_pnp_pnpClientConfiguration_CPV_config_t* configItem;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_CPV_config_table, 
                           xme_hal_table_rowHandle_t, configItemHandle,
                           xme_core_pnp_pnpClientConfiguration_CPV_config_t, configItem,
                           (configItem->portType == portType && configItem->topic == topic && configItem->componentId == componentId));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_CPV_config_table);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != configItemHandle, XME_STATUS_OUT_OF_RESOURCES);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_CPV_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->port = port;
            configItem->portType = portType;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->fDescriptor[0] = fDescriptor;
            configItem->countfDescriptor = (fDescriptor == NULL) ? 0U : 1U;
            configItem->componentType = componentType;
            configItem->componentId = componentId;
            configItem->totalToSchedule = 0U;
            configItem->scheduleEarly = 0U;
            configItem->toDelete = false;
            configItem->totalUsageCount = 1U;

            if (NULL != initializationString)
            {
                size_t length = strlen(initializationString) + 1U;
                configItem->initializationString = (char*) xme_hal_mem_alloc(length);
                XME_CHECK_REC
                (
                    NULL != configItem->initializationString,
                    XME_STATUS_OUT_OF_RESOURCES,
                    {
                        XME_HAL_TABLE_REMOVE_ITEM(xme_core_pnp_pnpClientConfiguration_CPV_config_table, configItemHandle);
                    }
                );
                
                xme_hal_safeString_strncpy(&configItem->initializationString[0], initializationString, length);
            }
        }
    }
    else
    {
        //else it may be a case of multiple functions getting mapped to same port
        //so check if you already have stored the function pointer and if not add it.
        uint8_t i;

        XME_ASSERT(NULL != configItem);

        if (configItem->fDescriptor[0] == NULL)
        {
            configItem->fDescriptor[0] = fDescriptor;
            configItem->countfDescriptor++;

            return XME_STATUS_SUCCESS;
        }

        for (i = 0U; i < configItem->countfDescriptor; i++)
        {
            if (configItem->fDescriptor[i] == fDescriptor)
            {
                return XME_STATUS_SUCCESS;
            }
        }
        XME_CHECK_MSG_C
        (
            (i < (uint8_t)XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT),
            XME_STATUS_INTERNAL_ERROR,
            XME_CORE_COMPONENT_TYPE_PNPCLIENT,
            XME_LOG_NOTE,
            "Function descriptor array overflow while registering static component ports!\n"
        );
        configItem->fDescriptor[i] = fDescriptor;
        configItem->countfDescriptor++;
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
    xme_core_exec_functionDescriptor_t *fDescriptor,
    xme_core_channelId_t channelId
)
{
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_pnp_pnpClientConfiguration_marshaler_config_t* configItem;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_marshaler_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_marshaler_config_t, configItem,
                               (configItem->topic == topic));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_marshaler_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_marshaler_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->outputPort = outputPort;
            configItem->inputPort = inputPort;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->instanceId = instanceId;
            configItem->fDescriptor = fDescriptor;
            configItem->totalToSchedule = 0U;
            configItem->toDelete = false;
            configItem->channelID = channelId;
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
    xme_core_exec_functionDescriptor_t *fDescriptor,
    xme_core_channelId_t channelId
)
{
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_pnp_pnpClientConfiguration_demarshaler_config_t* configItem;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_demarshaler_config_t, configItem,
                               (configItem->topic == topic));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->outputPort = outputPort;
            configItem->inputPort = inputPort;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->instanceId = instanceId;
            configItem->fDescriptor = fDescriptor;
            configItem->totalToSchedule = 0U;
            configItem->toDelete = false;
            configItem->channelID = channelId;
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
    xme_core_pnp_pnpClientConfiguration_udpSend_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_udpSend_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_udpSend_config_t, configItem,
                               ((xme_hal_mem_compare(configItem->destIP,destIP,strlen(destIP)) == 0) 
                                   && configItem->port == ipPort 
                                   && xme_hal_mem_compare(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH) == 0));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_udpSend_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_udpSend_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->inputPort = dataPort;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            (void) xme_hal_mem_copy(configItem->destIP, destIP, strlen(destIP));
            (void) xme_hal_mem_copy(configItem->key, key, XME_WP_UDP_HEADER_KEY_LENGTH);
            configItem->port = ipPort;
            configItem->instanceId = instanceId;
            configItem->fDescriptor = fDescriptor;
            configItem->buffer = buffer;
            configItem->totalToSchedule = 0U;
            configItem->toDelete = false;
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
    xme_core_pnp_pnpClientConfiguration_udpRecv_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_udpRecv_config_t, configItem,
                               (configItem->port == ipPort && 
                                   xme_hal_mem_compare(configItem->key, key, XME_WP_UDP_HEADER_KEY_LENGTH) == 0));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, configItemHandle);
        configItem->outputPort = dataPort;
        configItem->topic = topic;
        configItem->topicSize = topicSize;
        (void) xme_hal_mem_copy(configItem->key,key,XME_WP_UDP_HEADER_KEY_LENGTH);
        configItem->port = ipPort;
        configItem->instanceId = instanceId;
        configItem->fDescriptor = fDescriptor;
        configItem->buffer = buffer;
        configItem->totalToSchedule = 0U;
        configItem->toDelete = false;
    }
  
    return XME_STATUS_SUCCESS;
}    

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
)
{
    xme_core_pnp_pnpClientConfiguration_channelInjector_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_channelInjector_config_table, 
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_channelInjector_config_t, configItem,
                               (configItem->srcChID == srcChID && configItem->topic == topic));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_channelInjector_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_channelInjector_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->inputPort = inputPort;
            configItem->outputPort = outputPort;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->srcChID = srcChID;
            configItem->instanceId = instanceId;
            configItem->fDescriptor = fDescriptor;
            configItem->buffer = buffer;
            configItem->totalToSchedule = 0U;
            configItem->toDelete = false;
        }
    }
  
    return XME_STATUS_SUCCESS;
}

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
)
{
    xme_core_pnp_pnpClientConfiguration_channelSelector_config_t *configItem;
    xme_hal_table_rowHandle_t configItemHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    //check if it already exists
    XME_HAL_TABLE_GET_NEXT(xme_core_pnp_pnpClientConfiguration_channelSelector_config_table,
                               xme_hal_table_rowHandle_t, configItemHandle,
                               xme_core_pnp_pnpClientConfiguration_channelSelector_config_t, configItem,
                               (configItem->srcChID == srcChID 
                                   && configItem->topic == topic 
                                   && configItem->dstChID == dstChID));
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == configItemHandle)
    {
        //it does not exist
        configItemHandle = XME_HAL_TABLE_ADD_ITEM(xme_core_pnp_pnpClientConfiguration_channelSelector_config_table);
        configItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_pnp_pnpClientConfiguration_channelSelector_config_table, configItemHandle);
        if (configItem != NULL)
        {
            configItem->inputPort = inputPort;
            configItem->outputPort = outputPort;
            configItem->topic = topic;
            configItem->topicSize = topicSize;
            configItem->srcChID = srcChID;
            configItem->dstChID = dstChID;
            configItem->instanceId = instanceId;
            configItem->fDescriptor = fDescriptor;
            configItem->buffer = buffer;
            configItem->totalToSchedule = 0U;
            configItem->toDelete = false;
        }
    }
  
    return XME_STATUS_SUCCESS;
}    

static xme_status_t
removeRoutes
(
    const xme_core_topic_pnpManager_runtime_graph_model_t* portPnpGraphInData
)
{
    uint16_t i;
    xme_status_t status;

    for (i = 0U; i < XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH; i++)
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

            srcVType = portPnpGraphInData->vertex[srcI].vertexType;
            dstVType = portPnpGraphInData->vertex[dstI].vertexType;
            srcPort = NULL;
            dstPort = NULL;

            if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_removeConfigComponentPortVertex(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData,
                                                        portPnpGraphInData->vertex[srcI].componentType,
                                                        portPnpGraphInData->vertex[srcI].componentId);
            }
            else if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_removeConfigChannelInjector(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_removeConfigMarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_removeConfigUDPRecv(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_removeConfigDemarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (srcVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_removeConfigChannelSelector(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            /* Now the destination vertex */
            if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_removeConfigChannelInjector(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_removeConfigMarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_removeConfigUDPSend(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_removeConfigDemarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_removeConfigChannelSelector(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (dstVType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_removeConfigComponentPortVertex(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData,
                                                        portPnpGraphInData->vertex[dstI].componentType,
                                                        portPnpGraphInData->vertex[dstI].componentId);
            }
            if (srcPort != NULL && dstPort != NULL)
            {
                status = xme_core_broker_removeDataPacketTransferEntry(*srcPort, *dstPort);
                XME_LOG_C_IF
                (
                    XME_STATUS_SUCCESS != status,
                    XME_CORE_COMPONENT_TYPE_PNPCLIENT,
                    XME_LOG_ERROR,
                    "Removal of data packet ID mapping between %d and %d returned %d\n",
                    srcVType, dstPort, status
                );
            }
        }
    } // for all the edges

    status = xme_core_pnp_pnpClientScheduling_finiComponents();
    XME_CHECK_MSG_C
    (
        status == XME_STATUS_SUCCESS,
        status,
        XME_CORE_COMPONENT_TYPE_PNPCLIENT,
        XME_LOG_ERROR,
        "finiComponents returned %d\n",
        status
    );
    return status;
}

static xme_status_t
addRoutes
(
    const xme_core_topic_pnpManager_runtime_graph_model_t* portPnpGraphInData
)
{
    uint16_t i = 0;
    xme_status_t status;
    for (i = 0; i < XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH; i++)
    { //for all the possible edges
        uint8_t srcI;
        uint8_t dstI;
        xme_core_pnp_dataLinkGraph_edgeTypes_t edgeType;

        edgeType = portPnpGraphInData->edge[i].edgeType;

        if (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_INVALID_EDGETYPE == edgeType) { break; }

        srcI = portPnpGraphInData->edge[i].srcVertexIndex - 1; // Vertex index is one-based, whereas the array is zero-based
        dstI = portPnpGraphInData->edge[i].sinkVertexIndex - 1;
        XME_CHECK_MSG_C
        (
            srcI < XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH &&
            dstI < XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH,
            XME_STATUS_INTERNAL_ERROR,
            XME_CORE_COMPONENT_TYPE_PNPCLIENT,
            XME_LOG_ERROR,
            "Vertex index in received runtime graph model is out of range.\n"
        );

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

            if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == srcVType)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_addConfigComponentPortVertex(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData,
                                                        portPnpGraphInData->vertex[srcI].componentId,
                                                        portPnpGraphInData->vertex[srcI].componentType,
                                                        portPnpGraphInData->vertex[srcI].componentHandle,
                                                        portPnpGraphInData->vertex[srcI].portData,
                                                        portPnpGraphInData->vertex[srcI].functionData,
                                                        false);
                early = true;
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR == srcVType)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_addConfigChannelInjector(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER == srcVType)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_addConfigMarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPRECEIVE == srcVType)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_addConfigUDPRecv(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER == srcVType)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_addConfigDemarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, 
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR == srcVType)
            {
                srcPort = xme_core_pnp_pnpClientConfiguration_addConfigChannelSelector(XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                                        portPnpGraphInData->vertex[srcI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            /* Now the destination vertex */
            if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELINJECTOR == dstVType)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_addConfigChannelInjector(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_MARSHALER == dstVType)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_addConfigMarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_UDPSEND == dstVType)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_addConfigUDPSend(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_DEMARSHALER == dstVType)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_addConfigDemarshaler(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_WAYPOINT_CHANNELSELECTOR == dstVType)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_addConfigChannelSelector(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData);
            }
            else if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == dstVType)
            {
                dstPort = xme_core_pnp_pnpClientConfiguration_addConfigComponentPortVertex(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, 
                                                        portPnpGraphInData->vertex[dstI].vertexData,
                                                        portPnpGraphInData->edge[i].edgeData,
                                                        portPnpGraphInData->vertex[dstI].componentId,
                                                        portPnpGraphInData->vertex[dstI].componentType,
                                                        portPnpGraphInData->vertex[dstI].componentHandle,
                                                        portPnpGraphInData->vertex[dstI].portData,
                                                        portPnpGraphInData->vertex[dstI].functionData,
                                                        early);
            }

            xme_core_pnp_pnpClientConfiguration_createPortMappingArray(srcPort, dstPort);
        }
    } // for all the edges

    status = xme_core_pnp_pnpClientScheduling_initComponents();
    XME_CHECK_MSG_C
    (
        status == XME_STATUS_SUCCESS,
        status,
        XME_CORE_COMPONENT_TYPE_PNPCLIENT,
        XME_LOG_ERROR,
        "initComponents returned %d\n",
        status
    );

    status = xme_core_pnp_pnpClientConfiguration_addPortMappingToBroker();
    XME_CHECK_MSG_C
    (
        status == XME_STATUS_SUCCESS,
        status,
        XME_CORE_COMPONENT_TYPE_PNPCLIENT,
        XME_LOG_ERROR,
        "addPortMappingToBroker returned %d\n",
        status
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpClient_processGraph
(
    const xme_core_topic_pnpManager_runtime_graph_model_t* portPnpGraphInData,
    char *outIsShutdown
)
{
    //int i;
    xme_status_t status;

    XME_ASSERT(NULL != outIsShutdown);
    *outIsShutdown = false;

    XME_CHECK(NULL != portPnpGraphInData, XME_STATUS_INVALID_PARAMETER);

    // Check if we indeed have the correct graph
    // With channel ID in place maybe we don't need this check
    XME_CHECK
    (
        XME_CORE_NODE_INVALID_NODE_ID != portPnpGraphInData->nodeId &&
        XME_CORE_NODE_INVALID_NODE_ID != xme_core_node_getCurrentNodeId(), 
        XME_STATUS_INVALID_CONFIGURATION
    );

    // Since we send same graph to all the nodes. The node receiving other nodes graph should silently drop it
    // This is not an error so we should return XME_STATUS_SUCCESS
    XME_CHECK_MSG_C
    (
        xme_core_node_getCurrentNodeId() == (xme_core_node_nodeId_t) portPnpGraphInData->nodeId,
        XME_STATUS_SUCCESS,
        XME_CORE_COMPONENT_TYPE_PNPCLIENT,
        XME_LOG_DEBUG,
        "Plug and Play Client on node %" PRIu16 " received plug and play graph for node %" PRIu16 "! It will be ignored.\n",
        xme_core_node_getCurrentNodeId(),
        portPnpGraphInData->nodeId
    );

    XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPCLIENT, XME_LOG_NOTE, "Received plug and play graph!\n");

    switch (portPnpGraphInData->action)
    {
        case XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD:
            status = addRoutes(portPnpGraphInData);
            break;
        case XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_REMOVE:
            status = removeRoutes(portPnpGraphInData);
            break;
        case XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_LOGOUT:
            *outIsShutdown = true;
            status =  XME_STATUS_SUCCESS;
            break;
        default:
            XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPCLIENT, XME_LOG_ERROR, "Invalid action in the received RT Graph\n");
            status =  XME_STATUS_INVALID_CONFIGURATION;
            break;
    }

    return status;
}

static void
addToManifest
(
    xme_core_topic_pnp_componentInstanceManifest_t* const outManifest,
    xme_hal_linkedList_index_t idx,
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    xme_core_componentType_t componentType;
    xme_core_component_t component;
    const char* initializationString;

    componentType = xme_core_nodeMgr_compRep_getComponentType(componentHandle);
    component = xme_core_nodeMgr_compRep_getComponentID(componentHandle);
    initializationString = xme_core_nodeMgr_compRep_getInitializationString(componentHandle);

    outManifest->components[idx].componentId = component;
    outManifest->components[idx].componentType = componentType;
    outManifest->components[idx].componentHandle = componentHandle;
        
    {
        uint16_t portIndex = 0u;
        xme_core_nodeMgr_compRep_portHandle_t portHandle = 0u;

        while (true)
        {
            portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, portIndex);

            if (0u == portHandle) { break; }

            outManifest->components[idx].portData[portIndex].queueSize = (uint8_t) // TODO: 8 vs 16 bit
                xme_core_nodeMgr_compRep_getQueueSize(portHandle);

            portIndex++;
        }
    }

    {
        uint16_t functionIndex = 0U;
        xme_core_nodeMgr_compRep_functionHandle_t functionHandle = 0U;

        while (true)
        {
            functionHandle = xme_core_nodeMgr_compRep_getFunction(componentHandle, functionIndex);
                
            if (0u == functionHandle) { break; }
                
            outManifest->components[idx].functionData[functionIndex].executionPeriod =
                xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle);

            functionIndex++;
        }
    }

    if (NULL != initializationString)
    {
        xme_hal_safeString_strncpy(&outManifest->components[idx].initializationString[0], initializationString, sizeof(outManifest->components[idx].initializationString));
    }
}

xme_status_t
xme_core_pnp_pnpClient_getManifest
(
    xme_core_topic_pnp_componentInstanceManifest_t* const outManifest,
    char includeCreatedComponents
)
{
    xme_hal_linkedList_index_t componentCount = 0U;
    xme_hal_linkedList_index_t idx = 0U;

    XME_ASSERT(NULL != outManifest);

    outManifest->nodeId = xme_core_node_getCurrentNodeId();

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(listOfComponents, xme_core_pnp_pnpClient_componentAction_t, action);
    {
        // Only components for instantiation processed here
        if (XME_CORE_PNP_PNPCLIENT_COMPONENT_ACTION_TYPE_INSTANTIATE != action->actionType)
        {
            continue;
        }

        if (sizeof(outManifest->components) / sizeof(outManifest->components[0]) <= idx)
        {
            // Current instance manifest is "full", so we stop processing for now
            break;
        }

        componentCount++;
        addToManifest(outManifest, idx, action->componentHandle);
        idx++;

        xme_hal_singlyLinkedList_removeItem(&listOfComponents, action, (bool) true);
        xme_hal_mem_free(action);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    if (includeCreatedComponents)
    {
        xme_core_nodeMgr_compRep_componentIteratorInit();
        while (xme_core_nodeMgr_compRep_componentIteratorHasNext())
        {
            xme_core_nodeMgr_compRep_componentHandle_t componentHandle =
                xme_core_nodeMgr_compRep_componentIteratorNext();

            if (sizeof(outManifest->components) / sizeof(outManifest->components[0]) <= idx)
            {
                // Current instance manifest is "full", so we stop processing for now
                break;
            }

            if (XME_CORE_COMPONENT_TYPE_USER >= xme_core_nodeMgr_compRep_getComponentType(componentHandle))
            {
                continue; // Skip core components
            }

            if (XME_CORE_NODEMGR_COMPREP_STATE_CREATED ==
                    xme_core_nodeMgr_compRep_getState(componentHandle))
            {
                componentCount++;
                addToManifest(outManifest, idx, componentHandle);
                idx++;
            }
        }
        xme_core_nodeMgr_compRep_componentIteratorFini();
    }

    XME_CHECK(0u != componentCount, XME_STATUS_NOT_FOUND);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpClient_getComponentToRemove
(
    xme_core_topic_pnp_removeComponentRequest_t* const outRequest
)
{
    XME_ASSERT(NULL != outRequest);

    outRequest->nodeId = xme_core_node_getCurrentNodeId();
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != outRequest->nodeId, XME_STATUS_INVALID_CONFIGURATION);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(listOfComponents, xme_core_pnp_pnpClient_componentAction_t, action);
    {
        // Only components for instantiation processed here
        if (XME_CORE_PNP_PNPCLIENT_COMPONENT_ACTION_TYPE_DESTROY != action->actionType)
        {
            continue;
        }

        outRequest->componentId = xme_core_nodeMgr_compRep_getComponentID(action->componentHandle);
        XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != outRequest->componentId, XME_STATUS_INVALID_CONFIGURATION);

        xme_hal_singlyLinkedList_removeItem(&listOfComponents, action, (bool) true);
        xme_hal_mem_free(action);

        return XME_STATUS_SUCCESS;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_NOT_FOUND;
}

xme_status_t
xme_core_pnp_pnpClient_instantiateComponentOnThisNode
(
    xme_core_componentType_t componentType,
    const char* initializationString
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle;
    xme_core_node_nodeId_t localNodeID = xme_core_node_getCurrentNodeId();

    XME_CHECK_MSG_C
    (
        XME_CORE_NODE_INVALID_NODE_ID != localNodeID,
        XME_STATUS_INVALID_CONFIGURATION,
        XME_CORE_COMPONENT_TYPE_PNPCLIENT,
        XME_LOG_ERROR,
        "instantiateComponentOnThisNode(): Node is currently not logged in. Cannot instantiate new components.\n"
    );

    builder = xme_core_nodeMgr_compRep_createBuilder(localNodeID, componentType);
    XME_CHECK(NULL != builder, XME_STATUS_OUT_OF_RESOURCES);
    xme_core_nodeMgr_compRep_builderSetInitializationString(builder, initializationString);
    status = xme_core_nodeMgr_compRep_build(builder, &componentHandle);
    XME_ASSERT(XME_STATUS_SUCCESS == status || XME_STATUS_OUT_OF_RESOURCES == status);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_OUT_OF_RESOURCES);

    status = xme_core_pnp_pnpClient_plugInNewComponent(componentHandle);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpClient_plugInNewComponent
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_node_nodeId_t nodeID = xme_core_nodeMgr_compRep_getNodeID(componentHandle);
    xme_core_node_nodeId_t localNodeID = xme_core_node_getCurrentNodeId();

    XME_CHECK(XME_CORE_NODEMGR_COMPREP_STATE_PREPARED == xme_core_nodeMgr_compRep_getState(componentHandle), XME_STATUS_INVALID_CONFIGURATION); // This will also catch non-existing components
    XME_CHECK(localNodeID == XME_CORE_NODE_INVALID_NODE_ID || (localNodeID == nodeID || XME_CORE_NODE_LOCAL_NODE_ID == nodeID), XME_STATUS_INVALID_CONFIGURATION);

    status = xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION); // This will also catch non-existing components and wrong state

    status = scheduleComponentForInstantiationOrDestruction(componentHandle, XME_CORE_PNP_PNPCLIENT_COMPONENT_ACTION_TYPE_INSTANTIATE);
    XME_ASSERT(XME_STATUS_SUCCESS == status || XME_STATUS_OUT_OF_RESOURCES == status);

    return status;
}

xme_status_t
xme_core_pnp_pnpClient_destroyComponentOnThisNode
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    xme_core_component_t componentID;

    componentID = xme_core_nodeMgr_compRep_getComponentID(componentHandle);
    XME_CHECK_MSG_C
    (
        XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentID,
        XME_STATUS_INVALID_HANDLE,
        XME_CORE_COMPONENT_TYPE_PNPCLIENT,
        XME_LOG_WARNING,
        "Cannot destroy component on this node: component handle %d is not registered.\n",
        componentHandle
    );

    XME_CHECK(XME_CORE_NODEMGR_COMPREP_STATE_CREATED == xme_core_nodeMgr_compRep_getState(componentHandle), XME_STATUS_INVALID_CONFIGURATION); // This will also catch non-existing components

    return scheduleComponentForInstantiationOrDestruction(componentHandle, XME_CORE_PNP_PNPCLIENT_COMPONENT_ACTION_TYPE_DESTROY);
}

xme_status_t
xme_core_pnp_pnpClient_logoutNode
(
    xme_core_node_nodeId_t nodeId
)
{
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    xme_hal_singlyLinkedList_addItem(&listOfLoggedOutNodes, (void *)nodeId); //TODO ensure that node is also shut down when login manager is already shut down (Issue #4262)
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpClient_logoutThisNode
(
    void
)
{
    xme_core_node_nodeId_t nodeId = xme_core_node_getCurrentNodeId();

    if (XME_CORE_NODE_INVALID_NODE_ID != nodeId)
    {
        return xme_core_pnp_pnpClient_logoutNode(nodeId);
    }
    else
    {
        xme_core_exec_stop(true);

        return XME_STATUS_SUCCESS;
    }
}

xme_status_t
xme_core_pnp_pnpClient_getLoggedoutNodes
(
    void* nodeList
)
{   
    if(xme_hal_singlyLinkedList_getItemCount(&listOfLoggedOutNodes) == 0)
    {
        return XME_STATUS_NOT_FOUND;
    }
    else
    {
        //Iterarte over the list and add to the list of nodes
        
        XME_ASSERT(NULL != nodeList);
        
        XME_LOG_C(XME_CORE_COMPONENT_TYPE_PNPCLIENT, XME_LOG_NOTE, "Sending list of nodes to be logged out\n");
        
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(listOfLoggedOutNodes, void, tempNodeId);
        {
            xme_hal_singlyLinkedList_addItem(nodeList, tempNodeId);
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

        xme_hal_singlyLinkedList_clear(&listOfLoggedOutNodes);

        return XME_STATUS_SUCCESS;
    }
}

/**
 * @}
 */
