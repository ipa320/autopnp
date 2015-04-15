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
 * $Id: loginClient.c 7806 2014-03-13 10:17:51Z geisinger $
 */

/**
 * \file
 *         Login Client.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/login/include/loginClient.h"
#include "xme/core/login/include-gen/loginClientManifest.h"

#include "xme/core/logUtils.h"

#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"

#include "xme/hal/include/random.h"

#include "xme/wp/waypointConfigInfrastructure.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \brief variable stores the required login Information for this node
 */
static xme_core_topic_login_loginRequest_t loginInformation;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_login_loginClient_init(void)
{
    xme_com_interface_address_t intf;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID == loginInformation.nodeId, XME_STATUS_INVALID_CONFIGURATION);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_log_logUtils_init(), XME_STATUS_OUT_OF_RESOURCES);

    // Generate random GUID
    loginInformation.guid =
        ((uint64_t)xme_hal_random_randRange(0x0000, 0xFFFF)) << 48 |
        ((uint64_t)xme_hal_random_randRange(0x0000, 0xFFFF)) << 32 |
        ((uint64_t)xme_hal_random_randRange(0x0000, 0xFFFF)) << 16 |
        ((uint64_t)xme_hal_random_randRange(0x0000, 0xFFFF));

    loginInformation.nodeId = xme_core_node_getCurrentNodeId();
    xme_core_node_getNodeName(loginInformation.nodeName, sizeof(loginInformation.nodeName));
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_node_getInterface(&intf), XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4(&intf, &loginInformation.ipAddress, &loginInformation.portAddress), XME_STATUS_INTERNAL_ERROR);
    
    return XME_STATUS_SUCCESS;
}

void
xme_core_login_loginClient_fini(void)
{
    (void) xme_hal_mem_set(&loginInformation, 0x0,  sizeof(xme_core_topic_login_loginRequest_t));

    xme_core_log_logUtils_fini();
}

xme_status_t
xme_core_login_loginClient_fillLoginRequest
(
    xme_core_topic_login_loginRequest_t* config
)
{
    XME_CHECK(NULL!=config, XME_STATUS_INVALID_PARAMETER);
    (void) xme_hal_mem_copy(config, &loginInformation, sizeof(xme_core_topic_login_loginRequest_t));
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_login_loginClient_processLoginResponse
(
    const xme_core_topic_login_loginResponse_t* const config
)
{
    XME_CHECK(NULL!=config, XME_STATUS_INVALID_PARAMETER);
    if(config->guid == loginInformation.guid)
    {
        xme_status_t status;
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t)config->nodeId), XME_STATUS_INTERNAL_ERROR);
        loginInformation.nodeId = (xme_core_node_nodeId_t)config->nodeId;
        //Add path for Component Instance Manifest and use the channel ID supplied in the LoginResponse
        status = xme_wp_wci_addConfig(config->ipAddress, config->portAddress, config->channelID, XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST);
        XME_CHECK_MSG_C(XME_STATUS_SUCCESS == status, status, XME_CORE_COMPONENT_TYPE_LOGINCLIENT, XME_LOG_ERROR, "[LoginClient] xme_wp_wci_addConfig for Componnt Instance Manifest returned error.\n");
        //Add the logout path for this node
        status = xme_wp_wci_addConfig(config->ipAddress, config->portAddress, (xme_core_channelId_t)XME_CORE_TOPIC_PNP_LOGOUTREQUEST, XME_CORE_TOPIC_PNP_LOGOUTREQUEST);
        XME_CHECK_MSG_C(XME_STATUS_SUCCESS == status, status, XME_CORE_COMPONENT_TYPE_LOGINCLIENT, XME_LOG_ERROR, "[LoginClient] xme_wp_wci_addConfig for logout path returned error.\n");
        //Add the logout acknowledgement path for this node
        status = xme_wp_wci_addConfig(config->ipAddress, config->portAddress, (xme_core_channelId_t)XME_CORE_TOPIC_PNP_LOGOUTACKNOWLEDGMENT, XME_CORE_TOPIC_PNP_LOGOUTACKNOWLEDGMENT);
        XME_CHECK_MSG_C(XME_STATUS_SUCCESS == status, status, XME_CORE_COMPONENT_TYPE_LOGINCLIENT, XME_LOG_ERROR, "[LoginClient] xme_wp_wci_addConfig for logout acknowledgement returned error.\n");
        return XME_STATUS_SUCCESS;
    }
    return XME_STATUS_INVALID_CONFIGURATION;
}
