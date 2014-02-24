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
 * $Id: nodeRegistryController.c 5096 2013-09-17 16:22:43Z ruiz $
 */

/**
 * \file
 *         Node instance list.
 */

/**
 * \addtogroup core_directory_nodeRegistryController
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/directory/include/nodeRegistryController.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/******************************************************************************/
/***   Static variables                                                     ***/
/******************************************************************************/

static XME_HAL_TABLE(xme_core_node_nodeData_t, xme_core_directory_nodeRegistryController_nodeTable, XME_CORE_DIRECTORY_NODEREGISTRYCONTROLLER_NODE_TABLE_SIZE); ///< Table to store all the nodes. 

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_directory_nodeRegistryController_init(void)
{
    XME_HAL_TABLE_INIT(xme_core_directory_nodeRegistryController_nodeTable);

    return XME_STATUS_SUCCESS;
}

void
xme_core_directory_nodeRegistryController_fini(void)
{
    XME_HAL_TABLE_FINI(xme_core_directory_nodeRegistryController_nodeTable);
}

xme_status_t
xme_core_directory_nodeRegistryController_registerNode
(
    xme_core_node_nodeId_t nodeId,
    const char* nodeName,
    xme_core_node_guid_t guid
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != nodeName, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(0 != guid, XME_STATUS_INVALID_PARAMETER);

    // Search for an existing item for this node
    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_nodeRegistryController_nodeTable, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_core_node_nodeData_t, 
        nodeItem, 
        nodeItem->nodeId == nodeId
    );
    {
        // The node should not yet be registered
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE == handle, XME_STATUS_ALREADY_EXIST);

        // Insert the node
        handle = XME_HAL_TABLE_ADD_ITEM(xme_core_directory_nodeRegistryController_nodeTable);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_OUT_OF_RESOURCES);

        nodeItem = (xme_core_node_nodeData_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_nodeRegistryController_nodeTable, handle);
        XME_CHECK(NULL != nodeItem, XME_STATUS_OUT_OF_RESOURCES);

        nodeItem->nodeId = nodeId;
        nodeItem->nodeName = nodeName;
        nodeItem->guid = guid;
        XME_HAL_TABLE_INIT(nodeItem->interfaces);
    }

    return XME_STATUS_SUCCESS;
}

bool
xme_core_directory_nodeRegistryController_isNodeGuidRegistered
(
    xme_core_node_guid_t guid
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;

    XME_CHECK(0 != guid, false);

    // Search for an existing item for this node
    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_nodeRegistryController_nodeTable, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_core_node_nodeData_t, 
        nodeItem, 
        nodeItem->guid == guid
    );
    {
        // The node is not yet registered
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE == handle, true);
    }

    return false;
}

xme_status_t
xme_core_directory_nodeRegistryController_getNodeIdFromGUID
(
    xme_core_node_guid_t guid,
    xme_core_node_nodeId_t* nodeId
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;

    XME_CHECK(0 != guid, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != nodeId, XME_STATUS_INVALID_PARAMETER);

    // Search for an existing item for this node
    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_nodeRegistryController_nodeTable, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_core_node_nodeData_t, 
        nodeItem, 
        nodeItem->guid == guid
    );

    // The node is not yet registered
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == handle || 
        NULL == nodeItem)
    {
        *nodeId = XME_CORE_NODE_INVALID_NODE_ID;
        return XME_STATUS_NOT_FOUND;
    }

    *nodeId = nodeItem->nodeId;
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_nodeRegistryController_getGUIDFromNodeId
(
    xme_core_node_nodeId_t nodeId,
    xme_core_node_guid_t* guid
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != guid, XME_STATUS_INVALID_PARAMETER);

    // Search for an existing item for this node
    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_nodeRegistryController_nodeTable, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_core_node_nodeData_t, 
        nodeItem, 
        nodeItem->nodeId == nodeId
    );

    // The node is not yet registered
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == handle || 
        NULL == nodeItem)
    {
        *guid = 0;
        return XME_STATUS_NOT_FOUND;
    }

    // The guid is not yet registered
    *guid = nodeItem->guid;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_nodeRegistryController_addInterface
(
    xme_core_node_nodeId_t nodeId,
    xme_com_interface_address_t interfaceAddress
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;
    xme_com_interface_address_t* newInterfaceAddress;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);

    // Search for an existing item for this node
    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;
    XME_HAL_TABLE_GET_NEXT(xme_core_directory_nodeRegistryController_nodeTable, xme_hal_table_rowHandle_t, handle, xme_core_node_nodeData_t, nodeItem, nodeItem->nodeId == nodeId);

    // The node should already be registered
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NOT_FOUND);
    
    if (nodeItem != NULL)
    {
        handle = XME_HAL_TABLE_ADD_ITEM(nodeItem->interfaces);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_OUT_OF_RESOURCES);
        newInterfaceAddress = (xme_com_interface_address_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(nodeItem->interfaces, handle);
        XME_ASSERT(NULL != newInterfaceAddress);

        (void) xme_hal_mem_copy(newInterfaceAddress, &interfaceAddress, sizeof(xme_com_interface_address_t));
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_nodeRegistryController_getInterface
(
    xme_core_node_nodeId_t nodeId,
    xme_com_interface_addressType_t addressType,
    xme_com_interface_address_t** outInterface
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;
    xme_com_interface_address_t* interfaceItem;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_COM_INTERFACE_ADDRESS_TYPE_INVALID != addressType, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != outInterface, XME_STATUS_INVALID_PARAMETER);

    // Search for an existing item for this node
    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;
    XME_HAL_TABLE_GET_NEXT(xme_core_directory_nodeRegistryController_nodeTable, xme_hal_table_rowHandle_t, handle, xme_core_node_nodeData_t, nodeItem, nodeItem->nodeId == nodeId);

    // Return if the node is not yet registered
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NOT_FOUND);
    
    if (nodeItem != NULL)
    {
        // Within the node item, look for the appropriate interface
        handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
        interfaceItem = NULL;
        XME_HAL_TABLE_GET_NEXT(nodeItem->interfaces, xme_hal_table_rowHandle_t, handle, xme_com_interface_address_t, interfaceItem, interfaceItem->addressType == addressType);

        // Return if no matching interface was found
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NOT_FOUND);
        XME_ASSERT(NULL != interfaceItem);

        *outInterface = interfaceItem;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_nodeRegistryController_initNodeInterfaceIterator
(
    xme_core_node_nodeId_t nodeId, 
    xme_com_interface_addressType_t addressType,
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t **iterator
)
{
    xme_hal_table_rowHandle_t handle=XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_node_nodeData_t* nodeItem = NULL;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_NOT_FOUND);
    
    XME_HAL_TABLE_GET_NEXT(xme_core_directory_nodeRegistryController_nodeTable, xme_hal_table_rowHandle_t, handle, xme_core_node_nodeData_t, nodeItem, nodeItem->nodeId == nodeId);
    // The node should already be registered
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NOT_FOUND);
    XME_ASSERT(NULL != nodeItem);
    
    *iterator = (xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t));
    XME_CHECK( NULL != iterator, XME_STATUS_OUT_OF_RESOURCES);
        
    (*iterator)->nodeId = nodeId;
    (*iterator)->nodeItem = nodeItem;
    (*iterator)->addressType = addressType;
    (*iterator)->currentHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    (*iterator)->interfaceItem = NULL;
    
    return XME_STATUS_SUCCESS;
}

bool
xme_core_directory_nodeRegistryController_hasNextInterface
(
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator
)
{
    XME_CHECK(NULL != iterator, false);

    iterator->interfaceItem=NULL;

    if(XME_COM_INTERFACE_ADDRESS_TYPE_INVALID == iterator->addressType)
        XME_HAL_TABLE_GET_NEXT(iterator->nodeItem->interfaces, xme_hal_table_rowHandle_t, iterator->currentHandle, xme_core_directory_node_nodeInterface_t, iterator->interfaceItem, (1));
    else 
        XME_HAL_TABLE_GET_NEXT(iterator->nodeItem->interfaces, xme_hal_table_rowHandle_t, iterator->currentHandle, xme_core_directory_node_nodeInterface_t, iterator->interfaceItem, iterator->interfaceItem->addressType == iterator->addressType);

    XME_CHECK((XME_HAL_TABLE_INVALID_ROW_HANDLE != iterator->currentHandle && NULL != iterator->interfaceItem), false);
    
    return true;
}

xme_com_interface_address_t*
xme_core_directory_nodeRegistryController_nextInterface
(
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator
)
{
    XME_CHECK(NULL != iterator, NULL);

    return iterator->interfaceItem;
}

xme_status_t
xme_core_directory_nodeRegistryController_finiNodeInterfaceIterator
(
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator
)
{
    XME_CHECK(NULL != iterator, XME_STATUS_INVALID_PARAMETER);
        
    xme_hal_mem_free(iterator);
    
    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */

