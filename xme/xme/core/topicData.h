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
 * $Id: topicData.h 7605 2014-02-26 11:36:55Z ruiz $
 */

/**
 * \file
 *         Preliminary topic registry.
 */

#ifndef XME_CORE_TOPICDATA_H
#define XME_CORE_TOPICDATA_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/topic.h"

#include "xme/com/interface.h"

#include "xme/core/component.h"
#include "xme/core/dataChannel.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/core/device.h"
#include "xme/core/log.h"
#include "xme/core/manifestTypes.h"
#include "xme/core/node.h"
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_core_topic_newNodeRequestData_t
 *
 * \brief  Data that are being transmitted along with a new node request.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t newNodeId; ///< Unique node identifier if assigned or XME_CORE_NODEMANAGER_INVALID_NODE_ID otherwise.
    xme_com_interface_interfaceId_t newNodeInterface; ///< Network interface of new node to reach edge node.
    xme_core_node_nodeId_t edgeNodeId; ///< Unique node identifier of the node that first forwarded the login request.
    xme_com_interface_interfaceId_t edgeNodeInterface; ///< Network interface of edge node to reach new node.

}
xme_core_topic_newNodeRequestData_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_newNodeResponseData_t
 *
 * \brief  Data that are being transmitted along with a new node response.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_dataChannel_t remoteAnnouncementDataChannel; ///< Data channel that should be used by the local directory of the new node to send announcements to its master directory.
    xme_core_dataChannel_t neighborhoodUpdateDataChannel; ///< Data channel that should be used to send neighborhood update to the master directory.
    xme_core_dataChannel_t remoteModifyRoutingTableDataChannel; ///< Data channel used by the master directory to send modify routing table commands to the new node.
}
xme_core_topic_newNodeResponseData_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_managementChannelsPacket_t
 *
 * \brief  Data that are being transmitted to construct the management channels during login.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_com_interface_interfaceId_t interfaceId; ///< Node unique identifier of the interface used for the management channels.
    xme_core_dataChannel_t remoteAnnouncementDataChannel; ///< Data channel that should be used by the local directory of the new node to send announcements to its master directory.
    xme_core_dataChannel_t remoteNeighborhoodUpdateDataChannel; ///< Data channel that should be used to send neighborhood update to the master directory.
    xme_core_dataChannel_t remoteModifyRoutingTableDataChannel; ///< Data channel used by the master directory to send modify routing table commands to the new node.
}
xme_core_topic_managementChannelsPacket_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_loginRequestData_t
 *
 * \brief  Data that are being transmitted along with a login request.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_device_type_t deviceType; ///< Device type.
    xme_core_device_guid_t deviceGuid; ///< Globally unique device identifier (e.g., serial number, MAC address).
    xme_core_node_nodeId_t edgeNodeId; ///< Unique node identifier of the node that first forwarded the login request.
    xme_com_interface_interfaceId_t newNodeInterfaceId; ///< Node unique identifier of the interface on the new node where the login request originated from.
    xme_com_interface_interfaceId_t edgeNodeInterfaceId; ///< Node unique identifier of the interface on the edge node that first received the login request.
}
xme_core_topic_loginRequestData_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_loginResponseData_t
 *
 * \brief  Data that are being transmitted along with a login response.
 */
#pragma pack(push, 1)
typedef struct
{
    // TODO (See ticket #766): The first two members are actually not necessary, because other RR mechanisms permit the association of the request to the response.
    //                         However, the respective members *are* necessary in the "last hop" to the node to be logged in, which typically does not use data centric communication.
    xme_core_device_type_t deviceType; ///< Device type (corresponds to the value of the respective request).
    xme_core_device_guid_t deviceGuid; ///< Globally unique device identifier (e.g., serial number, MAC address, corresponds to the value of the respective request).
    xme_core_node_nodeId_t newNodeId; ///< Assigned unique node identifier.
    xme_core_node_nodeId_t edgeNodeId; ///< Unique node identifier of the node that first forwarded the login request.
    xme_com_interface_interfaceId_t edgeNodeInterfaceId; ///< Node unique identifier of the interface on the edge node that first received the login request.
    xme_core_dataChannel_t remoteAnnouncementDataChannel; ///< Data channel that should be used by the local directory of the new node to send announcements to its master directory.
    xme_core_dataChannel_t remoteNeighborhoodUpdateDataChannel; ///< Data channel that should be used to send neighborhood update to the master directory.
    xme_core_dataChannel_t remoteModifyRoutingTableDataChannel; ///< Data channel used by the master directory to send modify routing table commands to the new node.
}
xme_core_topic_loginResponseData_t;
#pragma pack(pop)

/**
 * \enum xme_core_announcementType_t
 *
 * \brief  Announcement types.
 */
typedef enum
{
    XME_CORE_ANNOUNCEMENT_PUBLISH_TOPIC = 1, ///< A new topic has been published.
    XME_CORE_ANNOUNCEMENT_UNPUBLISH_TOPIC, ///< A topic has been unpublished.
    XME_CORE_ANNOUNCEMENT_SUBSCRIBE_TOPIC = 4, ///< A new topic subscription has been issued.
    XME_CORE_ANNOUNCEMENT_UNSUBSCRIBE_TOPIC, ///< A topic subscription has been discontinued.
    XME_CORE_ANNOUNCEMENT_PUBLISH_REQUEST = 8, ///< A new request (client) has been published.
    XME_CORE_ANNOUNCEMENT_UNPUBLISH_REQUEST, ///< A request (client) has been unpublished.
    XME_CORE_ANNOUNCEMENT_PUBLISH_REQUEST_HANDLER, ///< A new request handler (server) has been published.
    XME_CORE_ANNOUNCEMENT_UNPUBLISH_REQUEST_HANDLER ///< A request handler (server) has been unpublished.
}
xme_core_announcementType_t;

/**
 * \struct xme_core_topic_announcementPacket_t
 *
 * \brief  Data that are being transmitted along with a announcement.
 *
 *         An announcement is used to notify the global directory of changes
 *         in a component's configuration with respect to its ports.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_announcementType_t announcementType; ///< Announcement type. The union member is to be interpreted depending on the value of this field.
    xme_core_node_nodeId_t nodeId; ///< Node identifier of the node issuing this announcement.
    xme_core_component_t componentId; ///< Locally valid identifier of the component issuing this announcement.
    xme_core_dataManager_dataPacketId_t portId; ///< Locally valid identifier of the port issuing this announcement.
    union
    {
        // Publish topic
        struct
        {
            xme_core_topic_t publicationTopic; ///< Topic that has been published.
            // TODO: meta data. See ticket #646
            bool onlyLocal; ///< Defines if publication is limited to the local node or not.
        }
        publishTopic;

        // Unpublish topic
        // (no further parameters)

        // Subscribe topic
        struct
        {
            xme_core_topic_t subscriptionTopic; ///< Topic that has been subscribed to.
            // TODO: Filter meta data. See ticket #646
            bool onlyLocal; ///< Defines if subscription is limited to the local node or not.
        }
        subscribeTopic;

        // Unpublish subscription
        // (no further parameters)

        // Publish request (client)
        struct
        {
            xme_core_dataManager_dataPacketId_t responseHandlingPort; ///< Response handling port.
            xme_core_topic_t requestTopic; ///< Request topic that has been published.
            // TODO: meta data. See ticket #646
            xme_core_topic_t responseTopic; ///< Response topic that is expected by request publisher.
            // TODO: meta data. See ticket #646
            bool manyResponses; ///< Defines if request wants to get only one or as many as possilbe responses.
            bool onlyLocal; ///< Defines if request is limited to the local node or not.
        }
        publishRequest;

        // Unpublish request (client)
        // (no further parameters)

        // Publish request handler (server)
        struct
        {
            xme_core_dataManager_dataPacketId_t responseSendingPort; ///< Response sending port.
            xme_core_topic_t requestTopic; ///< Request topic that has been published.
            // TODO: meta data. See ticket #646
            xme_core_topic_t responseTopic; ///< Response topic that has been published.
            // TODO: meta data. See ticket #646
            bool onlyLocal; ///< Defines if request handler is limited to the local node or not.
        }
        publishRequestHandler;

        // Unpublish request handler (server)
        // (no further parameters)
    } announcementTypes; ///< the announcement types.
}
xme_core_topic_announcementPacket_t;
#pragma pack(pop)

/**
 * \enum xme_core_modifyRoutingTableType_t
 *
 * \brief  Routing table modification types.
 */
typedef enum
{
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_ADD_LOCAL_SOURCE_ROUTE = 0, ///< Add local source route.
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_REMOVE_LOCAL_SOURCE_ROUTE = 1, ///< Remove local source route.
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_ADD_LOCAL_DESTINATION_ROUTE = 2, ///< Add local destination route.
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_REMOVE_LOCAL_DESTINATION_ROUTE = 3, ///< Remove local destination route.
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_ADD_INBOUND_ROUTE = 4, ///< Add inbound route.
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_REMOVE_INBOUND_ROUTE = 5, ///< Remove inbound route.
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_ADD_OUTBOUND_ROUTE = 6, ///< Add outbound route.
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_REMOVE_OUTBOUND_ROUTE = 7, ///< Remove outbound route.
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_ADD_TRANSLATION_ROUTE = 8, ///< Add translation route.
    XME_CORE_MODIFY_ROUTING_TABLE_TYPE_REMOVE_TRANSLATION_ROUTE = 9 ///< Remove translation route.
}
xme_core_modifyRoutingTableType_t;

/**
 * \struct xme_core_topic_modifyRoutingTablePacket_t
 *
 * \brief  Data that are being transmitted along with a routing table modification request.
 *
 *         A routing table modification is used to request the routing table change.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_modifyRoutingTableType_t modifyRoutingTableType; ///< Routing table modification type. The union member is to be interpreted depending on the value of this field.
    xme_core_dataChannel_t dataChannel; ///< Data channel of route.
    union
    {
        // Add/remove local source route
        struct
        {
            xme_core_component_t component; ///< Locally valid identifier of component.
            xme_core_dataManager_dataPacketId_t port; ///< Locally valid identifier of port.
        }
        localSourceRoute; ///< the local source route. 

        // Add/remove local destination route
        struct
        {
            xme_core_component_t component; ///< Locally valid identifier of component.
            xme_core_dataManager_dataPacketId_t port; ///< Locally valid identifier of port.
        }
        localDestinationRoute; ///< the local destination route. 

        // Add/remove outbound route
        struct
        {
            xme_com_interface_interfaceId_t networkInterface; ///< Locally valid identifier of network interface.
        }
        outboundRoute; ///< the outbound route. 

        // Add/remove inbound route
        struct
        {
            xme_com_interface_interfaceId_t networkInterface; ///< Locally valid identifier of network interface.
        }
        inboundRoute; ///< the inbound route. 

        // Add/remove inbound route
        struct
        {
            xme_core_dataChannel_t newDataChannel; ///< New data channel of route.
        }
        translationRoute; ///< the translation route.
    } modifyRoutingTableTypes; ///< the struct for modification of routing tables. 
}
xme_core_topic_modifyRoutingTablePacket_t;
#pragma pack(pop)

/**
 * \typedef xme_core_topic_pnpManagerRuntimeGraphModelAction_t
 *
 * \brief Platform-independent type for representing values from enumeration
 *        xme_core_topic_pnpManagerRuntimeGraphModelAction_e.
 *
 * \see xme_core_topic_pnpManagerRuntimeGraphModelAction_e
 */
typedef uint32_t xme_core_topic_pnpManagerRuntimeGraphModelAction_t;

/**
 * \enum xme_core_topic_pnpManagerRuntimeGraphModelAction_e
 *
 * \brief Values for the action field of the
 *        xme_core_topic_pnpManager_runtime_graph_model_t structure.
 *
 * \see xme_core_topic_pnpManagerRuntimeGraphModelAction_t
 */
enum xme_core_topic_pnpManagerRuntimeGraphModelAction_e
{
    XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_INVALID = 0, ///< Invalid action.
    XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_ADD = 1, ///< Elements in runtime graph should be added.
    XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_REMOVE = 2, ///< Elements in runtime graph should be removed.
    XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL_ACTION_LOGOUT = 3 ///< Node with specified node ID should log out. All other fields in topic can be ignored.
};

#pragma pack(push, 1)
typedef struct
{
    xme_hal_time_timeInterval_t executionPeriod;
} xme_core_topic_pnpManagerRuntimeGraphModelFunctionData_t;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    uint8_t queueSize;
} xme_core_topic_pnpManagerRuntimeGraphModelPortData_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_pnpManager_runtime_graph_model_t
 *
 * \brief Data type for topic 'RuntimeGraphModel' (identifier: XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL).
 *
 * \details Graph datatype
 */
#pragma pack(push, 1)
// TODO: Rename to xme_core_topic_pnpManagerRuntimeGraphModel_t (Issue #1983)
typedef struct
{
    xme_core_topic_pnpManagerRuntimeGraphModelAction_t action; ///< Action to be performed by plug and play client.
    xme_core_node_nodeId_t nodeId; ///< node identifier: TODO: Temporary identifier for target node identifier
    struct
    {
        xme_core_pnp_dataLinkGraph_vertexTypes_t vertexType; ///< Vertex Type:
                                                             ///< The vertex type can be currently component or waypoint.
        char vertexData[256]; ///< Determines the vertex type-specific vertex information. See the implementation for details.
        xme_core_componentType_t componentType; ///< Component type.
        xme_core_component_t componentId; ///< Component identifier.
        xme_core_nodeMgr_compRep_componentHandle_t componentHandle; ///< This corresponds to the handle sent in the component instance manifest, if the component
                                                                    ///< was requested in that way (XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE otherwise).
        xme_core_topic_pnpManagerRuntimeGraphModelPortData_t portData[XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT];
        xme_core_topic_pnpManagerRuntimeGraphModelFunctionData_t functionData[XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT];
    }
    vertex[XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH]; ///< The definition of vertex in the graph. 
    struct
    {
        uint8_t srcVertexIndex; ///< source vertex index
        uint8_t sinkVertexIndex; ///< destination vertex index
        xme_core_pnp_dataLinkGraph_edgeTypes_t edgeType; ///< edge type:
                                                         ///< determines what kind of operation should be made between the two related vertices.
                                                         ///< This include, memcpy, etc.
        char edgeData[32]; ///< edge configuration data:
                           ///< - user topic size. 
    }
    edge[XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH]; ///< The definition of edge in the graph.
}
xme_core_topic_pnpManager_runtime_graph_model_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_pnp_logoutAcknowledgment_t
 *
 * \brief Data type for topic 'logoutAcknowledgment' (identifier: XME_CORE_TOPIC_PNP_LOGOUTACKNOWLEDGMENT).
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< Node ID of the node that the acknowledgment belongs to.
}
xme_core_topic_pnp_logoutAcknowledgment_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_pnp_logoutRequest_t
 *
 * \brief Data type for topic 'logoutRequest' (identifier: XME_CORE_TOPIC_PNP_LOGOUTREQUEST).
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< Node ID of the node that should be logged out.
}
xme_core_topic_pnp_logoutRequest_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_pnp_logoutNotification_t
 *
 * \brief Data type for topic 'logoutNotification' (identifier: XME_CORE_TOPIC_PNP_LOGOUTNOTIFICATION).
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< Node ID of the node that has been logged out.
}
xme_core_topic_pnp_logoutNotification_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_pnp_removeComponentRequest_t
 *
 * \brief Data type for topic 'removeComponentRequest' (identifier: XME_CORE_TOPIC_PNP_REMOVECOMPONENTREQUEST).
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< Node ID of the component to remove.
    xme_core_component_t componentId; ///< Component ID of the component to remove.
}
xme_core_topic_pnp_removeComponentRequest_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_flagData_t
 *
 * \brief  Data that are being transmitted along with a flag topic.
 */
#pragma pack(push, 1)
typedef struct
{
    bool flag; ///< Boolean flag.
}
xme_core_topic_flagData_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_integerData_t
 *
 * \brief  Data that are being transmitted along with an integer topic.
 */
#pragma pack(push, 1)
typedef struct
{
    int number; ///< Number.
}
xme_core_topic_integerData_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_decimalData_t
 *
 * \brief  Data that are being transmitted along with a decimal topic.
 */
#pragma pack(push, 1)
typedef struct
{
    float decimal; ///< Decimal number.
}
xme_core_topic_decimalData_t;

/**
 * \struct xme_core_topic_arrayData_t
 *
 * \brief  Data that are being transmitted along with a array topic.
 */
#pragma pack(pop)
typedef struct
{
    char str[10]; ///< the string.
    uint16_t number; ///< decimal number. 
}
xme_core_topic_arrayData_t;

/**
 * \struct xme_core_topic_stringData_t
 *
 * \brief  Data that are being transmitted along with a string topic.
 */
typedef struct
{
    char str[10]; ///< string.
}
xme_core_topic_stringData_t;

/**
 * \struct xme_core_topic_logMsgData_t
 *
 * \brief  Data that are being transmitted along with a log message topic (XME_CORE_TOPIC_LOG_MESSAGE).
 */
// TODO: Change log message data definition when metadata filtering is available (see ticket #665).
//       The struct itself will only contain the message size, all other information will be delivered
//       via metadata.
//       Candidates for metadata: severity, componentId/name, deviceGuid, timestamp
#pragma pack(push, 1)
typedef struct
{
    xme_core_device_guid_t deviceGuid; ///< Device guid of node that sent this message.
    xme_core_device_type_t deviceType; ///< Device type of node that sent this message.
    xme_core_component_t componentId;  ///< Local id of sending component.
    xme_log_severity_t severity;       ///< Severity.
    uint16_t msgSize;      ///< Size of message string in bytes.
    uint16_t compNameSize; ///< Size of component name string in bytes.
    
    // The following data must be appended in the given order after the struct when sending log message data:
    //
    // char[msgSize] msg           ///< Log message.
    // char[compNameSize] compName ///< Name of component which sent the message.
}
xme_core_topic_logMsgData_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_exec_cycleCounter_t
 *
 * \brief  Cycle counter published by the execution manager.
 */
#pragma pack(push, 1)
typedef struct
{
    uint32_t cycleCounter; ///< Cycle counter itself
}
xme_core_topic_exec_cycleCounter_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_neighborhoodUpdate_messageHeader_t
 *
 * \brief  Packet header for neighborhood update messages to the directory.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< Node ID sending the neighborhood update message.
    bool overflow; ///< Overflow indicator. Is true if problems occured during the storage of neighborhood information.
    uint16_t neighborsCount; ///< The amount of neighbors contained in neighborhood update message.
}
xme_core_topic_neighborhoodUpdate_messageHeader_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_neighborhoodUpdate_neighborItem_t
 *
 * \brief  One neighborhood entry for the neighborhood update message to the directory.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_com_interface_interfaceId_t sendInterfaceId; ///< Interface ID to reach neighbor.
    xme_core_node_nodeId_t nodeId; ///< Node ID of neighbor node detected by this node.
    xme_com_interface_interfaceId_t receiveInterfaceId; ///< Interface ID to receive messages form neighbor.
}
xme_core_topic_neighborhoodUpdate_neighborItem_t;
#pragma pack(pop)

/**
 * \enum xme_core_topic_login_loginResponse_loginStatus_t
 *
 * \brief This enumeration enumerates the possible response status after a login request. 
 */
#pragma pack(push, 1)
typedef enum
{
    XME_CORE_TOPIC_LOGIN_LOGINRESPONSE_LOGINSTATUS_LOGIN_SUCCESS = 0, ///< The login request was approved by the login manager. 
    XME_CORE_TOPIC_LOGIN_LOGINRESPONSE_LOGINSTATUS_LOGIN_ALREADY_LOGGED_IN = 1, ///< The node is already logged in the login manager. 
    XME_CORE_TOPIC_LOGIN_LOGINRESPONSE_LOGINSTATUS_LOGIN_ERROR = 2 ///< The login request was rejected by the login manager. 
}
xme_core_topic_login_loginResponse_loginStatus_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_login_loginRequest_t
 *
 * \brief  Data type for topic 'loginRequest' (identifier: XME_CORE_TOPIC_LOGIN_LOGINREQUEST).
 *
 * \details The login request is composed of node identification parameters and interface address for communicating back to the requesting node from the Login Manager. 
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_guid_t guid; ///< This value corresponds to the node generated unique id.
    xme_core_node_nodeId_t nodeId; ///< The nodeId. The value of this variable is always 0 (XME_CORE_NODE_INVALID_NODE_ID).
                                   ///< This is due to control that the requester node does not receive the same request from itself.
    char nodeName[256]; ///< Name of the node from which the login request is sent.
    uint32_t ipAddress; ///< This field specifies the IP address of the Login Client associated to the calling interface address. This IP address will be used as the response receive address. The data format of this field is endianness (network byte order). 
    uint16_t portAddress; ///< This field specifies the port address of the Login Client associated to the calling interface address. This port will be used as the response receive port. The data format of this field is endianness (network byte order). 
}
xme_core_topic_login_loginRequest_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_login_loginResponse_t
 *
 * \brief  Data type for topic 'loginResponse' (identifier: XME_CORE_TOPIC_LOGIN_LOGINRESPONSE).
 *
 * \details The login response is composed of request status, node identification parameters and plug and play manager address. 
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_topic_login_loginResponse_loginStatus_t loginStatus; ///< This enumerated value represents the possible status for a given request.
    xme_core_node_guid_t guid; ///< This value corresponds to the node generated unique id.
    xme_core_node_nodeId_t nodeId; ///< The nodeId. The value of this variable is always 0 (XME_CORE_NODE_INVALID_NODE_ID).
                                   ///< This is due to control that the requester node does not receive the same request from itself.
    uint32_t ipAddress; ///< This field specifies the IP address of the PnPManager. This IP address will be used by the PnPClient of the requesting node to publish its component instance manifest. The data format of this field is endianness (network byte order). 
    uint16_t portAddress; ///< This field specifies the IP address of the PnPManager. This IP address will be used by the PnPClient of the requesting node to publish its component instance manifest. The data format of this field is endianness (network byte order). 
    xme_core_channelId_t channelID; ///< This field specifies the channel ID needed for sending the Component Instance Manifest from PnPClient to PnPManager. 
}
xme_core_topic_login_loginResponse_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_pnp_componentInstanceManifest_t
 *
 * \brief Data type for topic 'componentInstanceManifest' (identifier: XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST).
 *
 * \details This manifest is related with all user component instances declared and instantiated by the sending node. 
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< The nodeId. The value of this variable is always 0 (XME_CORE_NODE_INVALID_NODE_ID).
                                   ///< This is due to control that the requester node does not receive the same request from itself.
    struct
    {
        xme_core_component_t componentId; ///< The internal component id.
        xme_core_componentType_t componentType; ///< The component type of the component.
        xme_core_nodeMgr_compRep_componentHandle_t componentHandle; ///< Component handle from sending node (only valid there).
        char initializationString[256]; ///< Component-specific initialization value.
        struct
        {
            uint8_t queueSize;
        } portData[XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT];
        struct
        {
            xme_hal_time_timeInterval_t executionPeriod;
        } functionData[XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT];
    }
    components[XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST_LENGTH]; ///< The list of components that are already running in the requestor node.
}
xme_core_topic_pnp_componentInstanceManifest_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_login_pnpLoginRequest_t
 *
 * \brief  Data type for topic 'pnpLoginRequest' (identifier: XME_CORE_TOPIC_LOGIN_PNPLOGINREQUEST).
 *
 * \details This is the request to communicate the PnPManager the new login attempts from other nodes.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< The generated nodeId. The value of this variable is assigned at Login Manager level. 
    uint32_t ipAddress; ///< This field specifies the IP address of the requesting node. This IP address will be used for registering the interface in the Plug and Play Manager. The data format of this field is endianness (network byte order). 
    uint16_t portAddress; ///< This field specifies the port address of the requesting node. This port address will be used for registering the interface in the Plug and Play Manager. The data format of this field is endianness (network byte order). 
}
xme_core_topic_login_pnpLoginRequest_t;
#pragma pack(pop)

/**
 * \struct xme_core_topic_login_pnpLoginResponse_t
 *
 * \brief  Data type for topic 'pnpLoginResponse' (identifier: XME_CORE_TOPIC_LOGIN_PNPLOGINRESPONSE).
 *
 * \details This topic provides the response to the PnPLoginRequest topic. The information exchanged corresponds to the identification of the new PnPManager interface address in that network. 
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< The nodeId. The value of this variable is always 0 (XME_CORE_NODE_INVALID_NODE_ID).
                                   ///< This is due to control that the requester node does not receive the same request from itself.
    uint32_t ipAddress; ///< This field specifies the IP address of the listening port of PnPManager. In this address the PnPManager will receive calls from PnPClients. The data format of this field is endianness (network byte order). 
    uint16_t portAddress; ///< This field specifies the port of the listening address of PnPManager. In this address the PnPManager will receive calls from PnPClients. The data format of this field is endianness (network byte order). 
    xme_core_channelId_t channelID; ///< This field specifies the channel ID needed for sending the Component Instance Manifest from PnPClient to PnPManager. 
}
xme_core_topic_login_pnpLoginResponse_t;
#pragma pack(pop)


/**
 * \struct xme_core_topic_login_loginAcknowledgment_t
 *
 * \brief  Data type for topic 'loginAcknowledgment' (identifier: XME_CORE_TOPIC_LOGIN_LOGINACKNOWLEDGMENT).
 *
 * \details This topic is sent from login client to pnp client to signal that the login process succeeded.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< Id of the node for which this message is meant.
}
xme_core_topic_login_loginAcknowledgment_t;
#pragma pack(pop)

#endif // #ifndef XME_CORE_TOPICDATA_H
