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
 * $Id: loginClient.h 5613 2013-10-25 09:04:14Z ruiz $
 */

/**
 * \file
 *         Login Client.
 */

#ifndef XME_CORE_LOGIN_LOGINCLIENT_H
#define XME_CORE_LOGIN_LOGINCLIENT_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/node.h"
#include "xme/core/topicData.h"
#include "xme/core/topic.h"
#include "xme/defines.h"
/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the Login Client component.
 *         Exactly one component of this type must be present on every node.
 *
 * \note   The node component should already be initialized before the 
 *         loginClient_init is invoked.
 *
 * \retval XME_STATUS_SUCCESS if the component has been successfully initialized.
 * \retval XME_STATUS_INVALID_CONFIGURATION if a component of this type has already
 *         been initialized and has a valid nodeID. Exactly one component of this
 *         type must be present on every node.
 * \retval XME_STATUS_INTERNAL_ERROR if the init fails because of invalid interface
 *         of the node.
 */
xme_status_t
xme_core_login_loginClient_init(void);

/**
 * \brief  Finalized the Login Client component.
 *         Exactly one component of this type must be present on every node.
 */
void
xme_core_login_loginClient_fini(void);

/**
 * \brief Fills the login request
 * \param[in,out] config Pointer to the data stucture which needs to be filled
 * \retval XME_STATUS_SUCCESS if the component has successfully copied the 
 *         login Information
 * \retval XME_STATUS_INVALID_PARAMETER if config is NULL
 */
xme_status_t
xme_core_login_loginClient_fillLoginRequest
(
    xme_core_topic_login_loginRequest_t* config
);

/**
 * \brief  Updates the local node structure with the allocated node id
 * \param config Pointer the login response received by login client
 * \retval XME_STATUS_SUCCESS if the nodeId has been updated successfully
 * \retval XME_STATUS_INVALID_PARAMETER if config is NULL
 * \retval XME_STATUS_INVALID_CONFIGURATION if data pointed to by config is not
 *         for this node
 */
xme_status_t
xme_core_login_loginClient_processLoginResponse
(
    const xme_core_topic_login_loginResponse_t* const config
);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_LOGIN_LOGINCLIENT_H
