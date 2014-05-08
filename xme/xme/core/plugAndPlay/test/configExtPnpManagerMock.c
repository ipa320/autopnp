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
 * $Id: configExtPnpManagerMock.c 5838 2013-11-18 16:48:51Z wiesmueller $
 */

/**
 * \file
 *         Plug and Play Manager mock for configurator extension tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/plugAndPlayManager.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_pnp_pnpManager_init
(
    void *params
)
{
    return XME_STATUS_SUCCESS;
}

void
xme_core_pnp_pnpManager_fini(void);

xme_status_t
xme_core_pnp_pnpManager_updateConfiguration(void)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_instantiateComponentOnNode
(
    xme_core_componentType_t componentType,
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t existingComponentId
)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_announceNewComponentOnNode
(
    xme_core_componentType_t componentType,
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t* componentId
)
{
    return XME_STATUS_SUCCESS;
}

bool
xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode
(
    xme_core_node_nodeId_t nodeId
)
{
    return false;
}

xme_status_t
xme_core_pnp_pnpManager_getNextRuntimeGraphForNode
(
    xme_core_node_nodeId_t nodeId,
    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph
)
{
    return XME_STATUS_SUCCESS;
}

bool
xme_core_pnp_pnpManager_hasNewRuntimeGraphs(void)
{
    return false;
}

xme_status_t
xme_core_pnp_pnpManager_getNextRuntimeGraph
(
    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph
)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_status_t status
)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_registerNode
(
    xme_core_node_nodeId_t nodeId
)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_deregisterNode
(
    xme_core_node_nodeId_t nodeId
)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_registerInterfaceAddress
(
    xme_core_node_nodeId_t nodeId,
    xme_com_interface_address_t interfaceAddress
)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_announceComponentInstanceManifest
(
    xme_core_topic_pnp_componentInstanceManifest_t* manifest
)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished(void)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_pnpManager_generateChannelIDForManifestReception
(
    xme_core_node_nodeId_t nodeID,
    xme_core_channelId_t* channelID
)
{
    return XME_STATUS_SUCCESS;
}
