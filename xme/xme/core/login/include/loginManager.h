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
 * $Id: loginManager.h 7806 2014-03-13 10:17:51Z geisinger $
 */

/**
 * \file
 *         Login Manager.
 */

#ifndef XME_CORE_LOGIN_LOGINMANAGER_H
#define XME_CORE_LOGIN_LOGINMANAGER_H

/**
 * \defgroup core_login_manager Login Manager group
 * @{
 *
 * \brief The objective of the login manager is to receive requests from
 *        login client components and process the request by registering
 *        the node in the node registry controller. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/topicData.h"

#include "xme/core/node.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the login manager component.
 *         Exactly one component of this type must be present in the network.
 * 
 * \param params Initialization parameters for login manager component (TBD)
 * 
 * \retval XME_SUCCESS if the login manager component has been properly initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if the login manager component initialization failed.
 */ 
xme_status_t
xme_core_login_loginManager_init
(
    void *params
);

/**
 * \brief  Frees all resources occupied by the login manager component.
 */
void
xme_core_login_loginManager_fini (void);

/**
 * \brief Requests a connection from a remote or local node. 
 *
 * \details This function call will initialize all the configuration associated
 *          to the new node, including logical routes and node registry. 
 *
 * \param[in] loginRequest the topic associated to the login request. 
 * \param[out] pnpLoginRequest the output request to pnp manager.
 *             If return status is XME_STATUS_ALREADY_EXIST then pnpLoginRequest->nodeId
 *             will be set to the previously assigned nodeID.
 *
 * \retval XME_STATUS_SUCCESS if the login request is success. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot process the incoming request. 
 * \retval XME_STATUS_ALREADY_EXIST if the node already exists in the registered
 *         node list. 
 */
xme_status_t
xme_core_login_loginManager_loginRequest
(
    const xme_core_topic_login_loginRequest_t* const loginRequest,
    xme_core_topic_login_pnpLoginRequest_t* pnpLoginRequest
); 

/**
 * \brief Process a login response. 
 *
 * \details This function will process the response obtained from PnPManager
 *          and includes the remaining tasks associated targeted to deliver
 *          a response to the requesting node. 
 *
 * \param[in] pnpLoginResponse the response obtained from PnPManager. 
 * \param[out] loginResponse the output login response. 
 *
 * \retval XME_STATUS_SUCCESS if the login response is success. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot process the login response. 
 */ 
xme_status_t
xme_core_login_loginManager_loginResponse
(
    const xme_core_topic_login_pnpLoginResponse_t* const pnpLoginResponse,
    xme_core_topic_login_loginResponse_t* loginResponse
); 

#if 0
/**
 * \brief Processes a logoff received from an already logged in node. 
 *
 * \details Checks if a given node is already logged in.
 * 
 * \param[in] nodeID The node identifier to log-off (or logout). 
 * 
 * \retval XME_STATUS_SUCCESS if the node is already logged-in.
 * \retval XME_STATUS_NOT_FOUND if the node is not logged in. 
 * \retval XME_STATUS_INVALID_PARAMETER if the provided node identifier
 *         is an invalid node identifier. 
 */
xme_status_t
xme_core_login_loginManager_logoffRequest
(
    xme_core_node_nodeId_t nodeID
);
#endif

/**
 * \brief Gets the node id associated to a given guid. 
 * \details The guid is already logged in into the login manager.
 *          This function get the reference of assignment to 
 *          the requesting node. 
 * \param[in] guid the generated unique id from the requesting node.
 * \param[out] nodeId the node id associated to that guid. 
 *
 * \retval XME_STATUS_SUCCESS if the node id is successfully obtained from
 *         the login manager. 
 * \retval XME_STATUS_NOT_FOUND if the guid is not registered on the node id. 
 * \retval XME_STATUS_INTERNAL_ERROR if there is some error that avoids to get the 
 *         corresponding nodeId.
 */
xme_status_t
xme_core_login_loginManager_getNodeId
(
    xme_core_node_guid_t guid,
    xme_core_node_nodeId_t* nodeId
);

/**
 * \brief Process the received logout acknowledgment for the node with the
 *        given unique identifier.
 *
 * \details Removes the node from the Node Registry Controller.
 *
 * \param[in] nodeID Unique node identifier received in the logout
 *            acknowledgment. 
 *
 * \retval XME_STATUS_SUCCESS if the node has been successfully removed.
 * \retval XME_STATUS_INVALID_PARAMETER if the given node identifier was
 *         invalid.
 * \retval XME_STATUS_NOT_FOUND if the node is not registered.
 */
xme_status_t
xme_core_login_loginManager_logoutAcknowledgement
(
    xme_core_node_nodeId_t nodeID
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_LOGIN_LOGINMANAGER_H
