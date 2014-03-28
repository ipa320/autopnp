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
 * $Id: loginManagerInternalMethods.h 6623 2014-02-05 10:43:23Z wiesmueller $
 */

/**
 * \file
 *         Login Manager internal methods.
 */

#ifndef XME_CORE_LOGIN_LOGINMANAGERINTERNALMETHODS_H
#define XME_CORE_LOGIN_LOGINMANAGERINTERNALMETHODS_H

/**
 * \addtogroup core_login_manager
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/login/include/loginManagerInternalTypes.h"

#include "xme/core/node.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Calculates a new node id.
 * \details This new node id is unique in the network. This node id will be
 *          sent to the target node in the form of response to the send 
 *          request. 
 * \param[out] nodeId the newly generated node id. If it is not possible to generate
 *             the node id, XME_CORE_NODE_INVALID_NODE_ID is returned insted. 
 * \retval XME_STATUS_SUCCESS if the new node identifier has been calculated
 *         correctly.
 * \retval XME_STATUS_INTERNAL_ERROR if the node identifier cannot be generated.
 */
xme_status_t
xme_core_login_loginManager_calculateNewNodeId
(
    xme_core_node_nodeId_t* nodeId
);

/**
 * \brief Checks if the requesting node is already registered. 
 * \details to be defined. The node registering should be based on request details.
 *
 * \param[in] guid the generated unique id for requestor node.
 * \param[out] outNodeID When non-null and the return value is XME_STATUS_ALREADY_EXISTS
 *             the previously assigned nodeID will be written here.
 *
 * \retval XME_STATUS_SUCCESS if the node is not logged in in the network. 
 * \retval XME_STATUS_ALREADY_EXISTS if the node is already logged in.
 */
xme_status_t
xme_core_login_loginManager_checkAlreadyLoggedIn
(
    xme_core_node_guid_t guid,
    xme_core_node_nodeId_t* const outNodeID
);


XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_LOGIN_LOGINMANAGERINTERNALMETHODS_H
