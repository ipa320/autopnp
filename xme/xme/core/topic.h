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
 * $Id: topic.h 7607 2014-02-26 12:06:00Z wiesmueller $
 */

/**
 * \file
 *         Core topics.
 */

#ifndef XME_CORE_TOPIC_H
#define XME_CORE_TOPIC_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def XME_CORE_TOPIC
 *
 * \brief Converts the given topic into an XME core topic.
 *
 * \deprecated Converting topic ids is obsolete since version 0.6 of XME,
 *             as all topic ids are now of type xme_core_topic_t.
 *             This macro will be removed in future versions.
 */
#define XME_CORE_TOPIC(topic) ((xme_core_topic_t)(topic))

#define XME_CORE_TOPIC_INVALID_TOPIC 0x00000000 ///< Invalid topic identifier.

#define XME_CORE_TOPIC_ROUTES_LOCAL_ANNOUNCEMENT 0x00000001 ///< XME local announcement topic.
#define XME_CORE_TOPIC_ROUTES_REMOTE_ANNOUNCEMENT 0x00000002 ///< XME remote announcement topic.
#define XME_CORE_TOPIC_ROUTES_REMOTE_MODIFY_ROUTING_TABLE 0x00000003 ///< XME remote modify routing table topic.
#define XME_CORE_TOPIC_ROUTES_LOCAL_NEIGHBORHOOD_UPDATE 0x00000004 ///< Topic containing information about neighborhood of local node.
#define XME_CORE_TOPIC_ROUTES_REMOTE_NEIGHBORHOOD_UPDATE 0x00000005 ///< Topic containing information about neighborhood of a remote node.
    
#define XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL 0x0000000A ///< Topic for exchange of data between Plug and Play Manager and Plug and Play Client.

#define XME_CORE_TOPIC_LOGIN_REQUEST 0x00000010 ///< Login request topic.
#define XME_CORE_TOPIC_LOGIN_RESPONSE 0x00000011 ///< Login response topic.
#define XME_CORE_TOPIC_LOGIN_NEW_NODE_REQUEST 0x00000012 ///< New node request topic.
#define XME_CORE_TOPIC_LOGIN_NEW_NODE_RESPONSE 0x00000013 ///< New node response topic.
#define XME_CORE_TOPIC_LOGIN_MANAGEMENT_CHANNELS_TO_EDGE_NODE 0x00000014 ///< Topic containing management channels to edge node.
#define XME_CORE_TOPIC_LOGIN_MANAGEMENT_CHANNELS_TO_NEW_NODE 0x00000015 ///< Topic containing management channels to edge node.
    
#define XME_CORE_TOPIC_LOGIN_LOGINREQUEST 22 ///< loginRequest topic (data type: xme_core_topic_login_loginRequest_t).
                                             ///< The Login Request is the information that is sent from the Login Client. 
                                             ///< This information should include all configuration parameters needed to login in the core node.
#define XME_CORE_TOPIC_LOGIN_LOGINRESPONSE 23 ///< loginResponse topic (data type: xme_core_topic_login_loginResponse_t).
                                              ///< The Login Response is the information that is sent from the Login Manager to the Login Client after the Login Client requested a login attemp. 
                                              ///< This information should include all configuration parameters needed to configure the login requesting node to start with calling and receiving calls.
#define XME_CORE_TOPIC_LOGIN_PNPLOGINREQUEST 24 ///< pnpLoginRequest topic (data type: xme_core_topic_login_pnpLoginRequest_t).
                                                ///< This is the request to communicate the PnPManager the new login attempts from other nodes.
#define XME_CORE_TOPIC_LOGIN_PNPLOGINRESPONSE 25 ///< pnpLoginResponse topic (data type: xme_core_topic_login_pnpLoginResponse_t).
                                                 ///< This topic provides the response to the PnPLoginRequest topic. The information exchanged corresponds to the identification of the new PnPManager interface address in that network.
#define XME_CORE_TOPIC_LOGIN_LOGINACKNOWLEDGMENT 26 ///< Used by loginClient to signal to the pnpClient that login was successful (data type: xme_core_topic_login_loginAcknowledgment_t).
    
#define XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST 30 ///< componentInstanceManifest topic (data type: xme_core_topic_login_componentInstanceManifest_t).
                                                        ///< This manifest is related with all user component instances declared and instantiated by the sending node. 
#define XME_CORE_TOPIC_PNP_LOGOUTACKNOWLEDGMENT 31 ///< Acknowledgment for logout command from plug and play manager (sent via a runtime graph message with action LOGOUT).
#define XME_CORE_TOPIC_PNP_LOGOUTREQUEST 32 ///< Triggers a logout of a node.
#define XME_CORE_TOPIC_PNP_LOGOUTNOTIFICATION 33 ///< Used to notify the loginManager about a successful logout.

#define XME_CORE_TOPIC_PNP_REMOVECOMPONENTREQUEST 34 ///< Request the removal of a component (data type: xme_core_topic_pnp_removeComponentRequest_t).

#define XME_CORE_TOPIC_LOG_MESSAGE 0x00000050 ///< Log message topic.

#define XME_CORE_TOPIC_EXEC_CYCLE_COUNTER 0x00000060 ///< Cycle counter published by the TT execution manager.

#define XME_CORE_TOPIC_LIFESIGN_HEARTBEAT 0x00000061 ///< Hearbeat signal.
#define XME_CORE_TOPIC_LIFESIGN_ERROR_MESSAGE 0x00000062 ///< Error message topic.

#define XME_CORE_TOPIC_GENERAL_COMMON 0x00000100 ///< Minimum topic identifier for common (not reserved for XME itself) topics.
#define XME_CORE_TOPIC_GENERAL_EVENT 0x00000101 ///< Notification event (with no actual data).
#define XME_CORE_TOPIC_GENERAL_FLAG 0x00000102 ///< Boolean flag.
#define XME_CORE_TOPIC_GENERAL_INTEGER 0x00000103 ///< Integer number.
#define XME_CORE_TOPIC_GENERAL_UNSIGNED_INTEGER 0x00000104 ///< Unsigned integer number.
#define XME_CORE_TOPIC_GENERAL_DECIMAL 0x00000105 ///< Decimal or integer number.
#define XME_CORE_TOPIC_GENERAL_STRING 0x00000106 ///< String.

#define XME_CORE_TOPIC_USER 0x00001000 ///< Minimum topic identifier for user-defined (not reserved for XME itself) topics.

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_core_topic_t
 *
 * \brief Unique numeric identifier for a topic.
 *        Every topic name maps to a topic identifier.
 */
typedef uint16_t xme_core_topic_t;

#endif // #ifndef XME_CORE_TOPIC_H
