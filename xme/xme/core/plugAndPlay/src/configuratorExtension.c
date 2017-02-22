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
 * $Id: configuratorExtension.c 7838 2014-03-14 12:38:35Z geisinger $
 */

/**
 * \file
 *         Configurator Extension.
 */

/**
 * \addtogroup core_pnp_configExt
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/configuratorExtension.h"

#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/plugAndPlay/include/plugAndPlayManager.h"

#include "xme/hal/include/mem.h"

#include <inttypes.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \brief Registration entry for a configurator in the configuratorRegistry table.
 */
typedef struct
{
    xme_core_pnp_configExt_configuratorType_t type; ///< Type of the configurator.
    xme_core_pnp_configExt_configuratorCallback_t callback; ///< Callback method of configurator.
} configuratorEntry_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief Table for storing all configurator registrations.
 */
static XME_HAL_TABLE
(
    configuratorEntry_t,
    configuratorRegistry,
    XME_CORE_PNP_CONFIGEXT_CONFIGURATORS_MAX
);

/**
 * \brief Table for storing all edges that are marked for removal by the currently
 *        executed configurator.
 *        This will be cleared before each configurator is executed.
 */
static XME_HAL_TABLE
(
    xme_hal_graph_edgeId_t,
    edgesToRemove,
    XME_CORE_PNP_CONFIGEXT_EDGES_TO_REMOVE_MAX
);

/**
 * \brief Indicates whether we need to restart the configuration process after
 *        all configurators have been executed.
 */
// TODO: triggerConfigRerun and edgesToRemove should ideally not be global data structures
static bool triggerConfigRerun;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_pnp_configExt_init(void)
{
    XME_HAL_TABLE_INIT(configuratorRegistry);
    XME_HAL_TABLE_INIT(edgesToRemove);

    return XME_STATUS_SUCCESS;
}

void
xme_core_pnp_configExt_fini(void)
{
    XME_HAL_TABLE_FINI(edgesToRemove);
    XME_HAL_TABLE_FINI(configuratorRegistry);
}

xme_status_t
xme_core_pnp_configExt_addConfigurator
(
    xme_core_pnp_configExt_configuratorType_t type,
    xme_core_pnp_configExt_configuratorCallback_t callback,
    xme_core_pnp_configExt_configuratorHandle_t* handle
)
{
    xme_hal_table_rowHandle_t rowHandle;
    configuratorEntry_t* item;

    rowHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    item = NULL;

    XME_CHECK(NULL != callback, XME_STATUS_INVALID_PARAMETER);

    rowHandle = XME_HAL_TABLE_ADD_ITEM(configuratorRegistry);
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != rowHandle, XME_STATUS_OUT_OF_RESOURCES);
    XME_CHECK(UCHAR_MAX >= rowHandle, XME_STATUS_OUT_OF_RESOURCES);

    item = XME_HAL_TABLE_ITEM_FROM_HANDLE(configuratorRegistry, rowHandle);
    XME_ASSERT(NULL != item);

    item->type = type;
    item->callback = callback;

    if (NULL != handle)
    {
        *handle = (xme_core_pnp_configExt_configuratorHandle_t)rowHandle;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_configExt_removeConfigurator
(
    xme_core_pnp_configExt_configuratorHandle_t handle
)
{
    return XME_HAL_TABLE_REMOVE_ITEM(configuratorRegistry, (xme_hal_table_rowHandle_t) handle);
}

bool
xme_core_pnp_configExt_executeConfigurators
(
    xme_core_pnp_configExt_configuratorType_t type,
    xme_core_pnp_lrm_logicalRoutes_t* logicalRouteGraph
)
{
    triggerConfigRerun = false;

    XME_ASSERT(NULL != logicalRouteGraph);

    XME_HAL_TABLE_ITERATE_BEGIN
    ( 
        configuratorRegistry,
        xme_hal_table_rowHandle_t, 
        handle, 
        configuratorEntry_t, 
        configuratorEntry
    );
    {
        if (configuratorEntry->type == type)
        {
            // We create a copy of the original graph that we pass to the configurator, so that any modifications
            // will not affect the original
            xme_core_pnp_lrm_logicalRoutes_t logicalRouteGraphCopy;
            xme_hal_graph_init(&logicalRouteGraphCopy);

            xme_hal_graph_clone(logicalRouteGraph, &logicalRouteGraphCopy);

            // Clear list of edges to remove for use by next configurator
            xme_hal_table_clear(&edgesToRemove);
            
            // Execute configurator
            configuratorEntry->callback(&logicalRouteGraphCopy);

            xme_hal_graph_fini(&logicalRouteGraphCopy);

            // Remove all edges that have been scheduled for removal by the configurator
            XME_HAL_TABLE_ITERATE_BEGIN
            ( 
                edgesToRemove,
                xme_hal_table_rowHandle_t, 
                handle, 
                xme_hal_graph_edgeId_t, 
                edgeToRemove
            );
            {
                xme_core_pnp_dataLinkGraph_edgeData_t edgeData;
                xme_core_pnp_dataLinkGraph_edgeData_t* edgeDataPtr = &edgeData;

                xme_hal_graph_getEdgeData(logicalRouteGraph, *edgeToRemove, (void**)&edgeDataPtr);
                if (edgeDataPtr->edgeData.logicalRouteEdge.established)
                {
                    XME_LOG(XME_LOG_WARNING, "A configurator extension tries to remove an established logical route. Removal of established routes is not supported yet.\n");
                    continue;
                }
                
                xme_hal_graph_removeEdge(logicalRouteGraph, *edgeToRemove);
            }
            XME_HAL_TABLE_ITERATE_END();
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return triggerConfigRerun;
}

// TODO (Issue #4310): Pass only component handle here! or deprecate and add new function (same name as in pnpManager).
//                     Error return value!?
void
xme_core_pnp_configExt_addComponent
(
    xme_core_componentType_t componentType,
    const char* const initializationString,
    xme_core_node_nodeId_t nodeId
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle;

    if (XME_CORE_COMPONENT_TYPE_INVALID == componentType) { return; }
    if (XME_CORE_NODE_INVALID_NODE_ID == nodeId) { return; }

    builder = xme_core_nodeMgr_compRep_createBuilder(nodeId, componentType);
    XME_ASSERT_NORVAL(NULL != builder);
    xme_core_nodeMgr_compRep_builderSetInitializationString(builder, initializationString);
    status = xme_core_nodeMgr_compRep_build(builder, &componentHandle);
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

    status = xme_core_pnp_pnpManager_announceNewComponentOnNode(componentHandle);
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

    triggerConfigRerun = true;
}

void
xme_core_pnp_configExt_removeComponent
(
    xme_core_component_t componentID,
    xme_core_node_nodeId_t nodeID
)
{
    xme_status_t status;

    status = xme_core_pnp_pnpManager_unannounceComponentOnNode(nodeID, componentID);
    XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_CHECK_RVAL_VOID,
        XME_LOG_ERROR, "[configuratorExtension] Error removing component (nodeID = %d, componentID = %d). Cannot remove component from Logical Route Manager.\n",
        nodeID, componentID);

    triggerConfigRerun = true;
}

xme_status_t
xme_core_pnp_configExt_removeLink
(
    xme_hal_graph_edgeId_t edgeId
)
{
    xme_hal_table_rowHandle_t handle;
    xme_hal_graph_edgeId_t* item;

    handle = XME_HAL_TABLE_ADD_ITEM(edgesToRemove);
    XME_CHECK(handle != XME_HAL_TABLE_INVALID_ROW_HANDLE, XME_STATUS_OUT_OF_RESOURCES);
 
    item = XME_HAL_TABLE_ITEM_FROM_HANDLE(edgesToRemove, handle);
    XME_ASSERT(NULL != item);

    *item = edgeId;

    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
