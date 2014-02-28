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
 * $Id: loginClient.c 5266 2013-10-01 12:55:29Z gulati $
 */

/**
 * \file
 *         Login Client.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/login/include/loginClient.h"

#include "xme/wp/waypointConfigInfrastructure.h"

#include "xme/hal/include/random.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/******************************************************************************/
/***   Static variables                                                     ***/
/******************************************************************************/

/**
 * \brief variable stores the required login Information for this node
 */
static xme_core_topic_login_loginRequest_t loginInformation;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_login_loginClient_init(void)
{
    xme_com_interface_address_t intf;

    XME_CHECK( XME_CORE_NODE_INVALID_NODE_ID == loginInformation.nodeId, XME_STATUS_INVALID_CONFIGURATION);

    // Generate random GUID
    loginInformation.guid =
        ((uint64_t)xme_hal_random_randRange(0x0000, 0xFFFF)) << 48 |
        ((uint64_t)xme_hal_random_randRange(0x0000, 0xFFFF)) << 32 |
        ((uint64_t)xme_hal_random_randRange(0x0000, 0xFFFF)) << 16 |
        ((uint64_t)xme_hal_random_randRange(0x0000, 0xFFFF));

    loginInformation.nodeId = xme_core_node_getCurrentNodeId();

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_node_getInterface(&intf), XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4(&intf, &loginInformation.ipAddress, &loginInformation.portAddress), XME_STATUS_INTERNAL_ERROR);
    
    return XME_STATUS_SUCCESS;
}

void
xme_core_login_loginClient_fini(void)
{
    (void) xme_hal_mem_set(&loginInformation, 0x0,  sizeof(xme_core_topic_login_loginRequest_t));
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
    xme_core_topic_login_loginResponse_t *config
)
{
    XME_CHECK(NULL!=config, XME_STATUS_INVALID_PARAMETER);
    if(config->guid == loginInformation.guid)
    {
        xme_status_t status;
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_node_setCurrentNodeId((xme_core_node_nodeId_t)config->nodeId), XME_STATUS_INTERNAL_ERROR);
        loginInformation.nodeId = (xme_core_node_nodeId_t)config->nodeId;
        status = xme_wp_wci_addConfig(config->ipAddress, config->portAddress, XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST);
        return status;
    }
    return XME_STATUS_INVALID_CONFIGURATION;
}
