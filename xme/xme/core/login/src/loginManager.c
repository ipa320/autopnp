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
 * $Id: loginManager.c 7806 2014-03-13 10:17:51Z geisinger $
 */

/**
 * \file
 *         Login Manager.
 */

/**
 * \addtogroup core_login_manager
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/login/include/loginManager.h"
#include "xme/core/login/include/loginManagerInternalMethods.h"

#include "xme/core/logUtils.h"

#include "xme/wp/waypointConfigInfrastructure.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \var xme_core_login_loginManager_lastUnassignedNodeId
 *
 * \brief The last unassigned node id. For each request from other nodes,
 *        this number will be increased sequentally. 
 *
 * \note The value 0 (aka XME_CORE_NODE_INVALID_NODE_ID) is reserved to specify
 *       an invalid node identifier.
 * \note The value 1 (aka XME_CORE_NODE_LOCAL_NODE_ID) is reserved for the
 *       "local" node identifier.
 *
 * \see  xme_core_node_nodeId_t
 */
static xme_core_node_nodeId_t xme_core_login_loginManager_lastUnassignedNodeId = (xme_core_node_nodeId_t) 10u;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_login_loginManager_init
(
    void *params
)
{
    XME_UNUSED_PARAMETER(params);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_log_logUtils_init(), XME_STATUS_OUT_OF_RESOURCES);

    return XME_STATUS_SUCCESS;
}

void
xme_core_login_loginManager_fini (void)
{
    xme_core_log_logUtils_fini();
}

xme_status_t
xme_core_login_loginManager_loginRequest
(
    const xme_core_topic_login_loginRequest_t* const loginRequest,
    xme_core_topic_login_pnpLoginRequest_t* pnpLoginRequest
)
{
    xme_core_node_guid_t guid;
    xme_core_node_nodeId_t* nodeId;
    char nodeName[256];

    XME_CHECK(NULL != loginRequest, XME_STATUS_INVALID_PARAMETER);

    nodeId = (xme_core_node_nodeId_t*) xme_hal_mem_alloc(sizeof(xme_core_node_nodeId_t));

    // Extract the data from login request.
    guid = loginRequest->guid;

    XME_CHECK(0 != guid, XME_STATUS_INVALID_PARAMETER);

    // Before generating the node id, we should first check if the guid is 
    // already in the node registry. In positive case, an XME_STATUS_ALREADY_EXIST 
    // status will be returned. 
    XME_CHECK(
        XME_STATUS_SUCCESS == 
        xme_core_login_loginManager_checkAlreadyLoggedIn(guid, &(pnpLoginRequest->nodeId)),
        XME_STATUS_ALREADY_EXIST
    ); 

    // Generate a new node id for target node. 
    XME_CHECK
    (
        XME_STATUS_SUCCESS ==
        xme_core_login_loginManager_calculateNewNodeId(nodeId),
        XME_STATUS_INTERNAL_ERROR
    );

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != *nodeId, XME_STATUS_INTERNAL_ERROR);

    // Set a name for the node: nodeIdX (nodeId-parameter dependent)
    XME_CHECK(0 < xme_hal_safeString_snprintf(nodeName, sizeof(nodeName), "%s", loginRequest->nodeName), XME_STATUS_INTERNAL_ERROR);
    
    // Register the node in the node registry controller. 
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_core_directory_nodeRegistryController_registerNode(*nodeId, nodeName, guid),
        XME_STATUS_INTERNAL_ERROR
    );

    // Setup pnpLoginRequest
    pnpLoginRequest->nodeId = *nodeId;
    pnpLoginRequest->ipAddress = loginRequest->ipAddress;
    pnpLoginRequest->portAddress = loginRequest->portAddress;

    // Return the value to the request node. 
    return XME_STATUS_SUCCESS;
}


xme_status_t
xme_core_login_loginManager_loginResponse
(
    const xme_core_topic_login_pnpLoginResponse_t* const pnpLoginResponse,
    xme_core_topic_login_loginResponse_t* loginResponse
)
{
    xme_core_node_guid_t guid;
    xme_core_node_nodeId_t nodeId;

    XME_CHECK(NULL != pnpLoginResponse, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != pnpLoginResponse->nodeId, XME_STATUS_INVALID_PARAMETER);

    nodeId = pnpLoginResponse->nodeId;

    XME_CHECK(
        XME_STATUS_SUCCESS == xme_core_directory_nodeRegistryController_getGUIDFromNodeId(nodeId, &guid),
        XME_STATUS_INTERNAL_ERROR
    );
    loginResponse->guid = guid;
    loginResponse->nodeId = pnpLoginResponse->nodeId;
    loginResponse->ipAddress = pnpLoginResponse->ipAddress;
    loginResponse->portAddress = pnpLoginResponse->portAddress;
    loginResponse->channelID = pnpLoginResponse->channelID;
    loginResponse->loginStatus = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE_LOGINSTATUS_LOGIN_SUCCESS;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_login_loginManager_getNodeId
(
    xme_core_node_guid_t guid,
    xme_core_node_nodeId_t* nodeId
)
{
    XME_CHECK(0 != guid, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_directory_nodeRegistryController_getNodeIdFromGUID(guid, nodeId), XME_STATUS_NOT_FOUND);

    return XME_STATUS_SUCCESS;
}

#if 0
xme_status_t
xme_core_login_loginManager_logoffRequest
(
    xme_core_node_nodeId_t nodeID
)
{
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);

    if (!xme_core_directory_nodeRegistryController_isNodeRegistered(nodeID))
    {
        return XME_STATUS_NOT_FOUND;
    }

    return XME_STATUS_SUCCESS;
}
#endif

xme_status_t
xme_core_login_loginManager_logoutAcknowledgement
(
    xme_core_node_nodeId_t nodeID
)
{
    return xme_core_directory_nodeRegistryController_deregisterNode(nodeID);
}

//////////////////////////////////////////////////////////////////////////
xme_status_t
xme_core_login_loginManager_checkAlreadyLoggedIn
(
    xme_core_node_guid_t guid,
    xme_core_node_nodeId_t* const outNodeID
)
{
    if (xme_core_directory_nodeRegistryController_isNodeGuidRegistered(guid, outNodeID))
    {
        return XME_STATUS_ALREADY_EXIST;
    }
    else
    {
        return XME_STATUS_SUCCESS;
    }
}

xme_status_t
xme_core_login_loginManager_calculateNewNodeId
(
    xme_core_node_nodeId_t* nodeId
)
{
    *nodeId = xme_core_login_loginManager_lastUnassignedNodeId;
        
    XME_ASSERT(XME_CORE_NODE_INVALID_NODE_ID != *nodeId);
    XME_ASSERT(XME_CORE_NODE_LOCAL_NODE_ID != *nodeId);

    xme_core_login_loginManager_lastUnassignedNodeId = (xme_core_node_nodeId_t) (((xme_maxSystemValue_t)xme_core_login_loginManager_lastUnassignedNodeId) + 1);

    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
