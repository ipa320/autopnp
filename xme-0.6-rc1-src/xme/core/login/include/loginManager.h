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
 * $Id: loginManager.h 4863 2013-08-30 07:39:30Z ruiz $
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
/***   Type definitions                                                     ***/
/******************************************************************************/

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
 *
 * \retval XME_STATUS_SUCCESS if the login request is success. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot process the incoming request. 
 * \retval XME_STATUS_ALREADY_EXIST if the node already exists in the registered
 *         node list. 
 */
xme_status_t
xme_core_login_loginManager_loginRequest
(
    xme_core_topic_login_loginRequest_t* loginRequest,
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
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse,
    xme_core_topic_login_loginResponse_t* loginResponse
); 

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

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_LOGIN_LOGINMANAGER_H
