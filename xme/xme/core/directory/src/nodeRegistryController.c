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
 * $Id: nodeRegistryController.c 7702 2014-03-06 23:22:57Z ruiz $
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

#include "xme/core/log.h"

/******************************************************************************/
/***   Variables                                                            ***/
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
    // Free all memory
    XME_HAL_TABLE_ITERATE_BEGIN
    (
        xme_core_directory_nodeRegistryController_nodeTable,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_node_nodeData_t,
        nodeItem
    );
    {
        XME_HAL_TABLE_FINI(nodeItem->interfaces);
    }
    XME_HAL_TABLE_ITERATE_END();

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
        XME_ASSERT(NULL != nodeItem);

        nodeItem->nodeId = nodeId;
        xme_hal_safeString_strncpy(nodeItem->nodeName, nodeName, sizeof(nodeItem->nodeName));
        nodeItem->guid = guid;
        XME_HAL_TABLE_INIT(nodeItem->interfaces);
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_nodeRegistryController_deregisterNode
(
    xme_core_node_nodeId_t nodeID
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;
    xme_status_t status;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);

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
        nodeItem->nodeId == nodeID
    );

    // The node id is not registered
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NOT_FOUND);

    status = XME_HAL_TABLE_REMOVE_ITEM(xme_core_directory_nodeRegistryController_nodeTable, handle);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    return XME_STATUS_SUCCESS;
}

bool
xme_core_directory_nodeRegistryController_isNodeGuidRegistered
(
    xme_core_node_guid_t guid,
    xme_core_node_nodeId_t* const outNodeID
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

    if (XME_HAL_TABLE_INVALID_ROW_HANDLE != handle)
    {
        if (NULL != outNodeID)
        {
            *outNodeID = nodeItem->nodeId;
        }

        return true;
    }
    
    return false;
}

xme_status_t
xme_core_directory_nodeRegistryController_getNodeIdFromGUID
(
    xme_core_node_guid_t guid,
    xme_core_node_nodeId_t* outNodeId
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;

    XME_CHECK(0 != guid, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != outNodeId, XME_STATUS_INVALID_PARAMETER);

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
        *outNodeId = XME_CORE_NODE_INVALID_NODE_ID;
        return XME_STATUS_NOT_FOUND;
    }

    *outNodeId = nodeItem->nodeId;
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_nodeRegistryController_getGUIDFromNodeId
(
    xme_core_node_nodeId_t nodeId,
    xme_core_node_guid_t* outGuid
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != outGuid, XME_STATUS_INVALID_PARAMETER);

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
        *outGuid = 0;
        return XME_STATUS_NOT_FOUND;
    }

    // The guid is registered
    *outGuid = nodeItem->guid;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_nodeRegistryController_getNodeName
(
    xme_core_node_nodeId_t nodeId,
    char* const nodeName,
    uint16_t size
)
{
    xme_hal_table_rowHandle_t handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_node_nodeData_t* nodeData = NULL;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != nodeName, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_nodeRegistryController_nodeTable, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_core_node_nodeData_t, 
        nodeData, 
        (nodeId == nodeData->nodeId)
    );

    // The node is not yet registered
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == handle || 
        NULL == nodeData)
    {
        return XME_STATUS_NOT_FOUND;
    }

    (void)xme_hal_safeString_strncpy(nodeName, nodeData->nodeName, size);
    
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
        if (0 != XME_HAL_TABLE_ITEM_COUNT(nodeItem->interfaces))
        {
            xme_hal_table_rowHandle_t rh = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            xme_com_interface_address_t *interfaceItem = NULL;
            XME_HAL_TABLE_GET_NEXT
            (
                nodeItem->interfaces, 
                xme_hal_table_rowHandle_t, rh, 
                xme_com_interface_address_t, interfaceItem, 
                interfaceItem->addressType == interfaceAddress.addressType
            );
            while (NULL != interfaceItem)
            {
                if (0 == xme_hal_mem_compare(interfaceItem, &interfaceAddress, sizeof(xme_com_interface_address_t))) 
                {
                    return XME_STATUS_ALREADY_EXIST;
                }
                interfaceItem = NULL;
                XME_HAL_TABLE_GET_NEXT
                (
                    nodeItem->interfaces, 
                    xme_hal_table_rowHandle_t, rh, 
                    xme_com_interface_address_t, interfaceItem, 
                    interfaceItem->addressType == interfaceAddress.addressType
                );
            }
        }
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
    xme_com_interface_address_t** outInterfaceAddress
)
{
    xme_hal_table_rowHandle_t handle;
    xme_core_node_nodeData_t* nodeItem;
    xme_com_interface_address_t* interfaceItem;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_COM_INTERFACE_ADDRESS_TYPE_INVALID != addressType, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != outInterfaceAddress, XME_STATUS_INVALID_PARAMETER);

    // Search for an existing item for this node
    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    nodeItem = NULL;
    XME_HAL_TABLE_GET_NEXT(xme_core_directory_nodeRegistryController_nodeTable, xme_hal_table_rowHandle_t, handle, xme_core_node_nodeData_t, nodeItem, nodeItem->nodeId == nodeId);

    // Return if the node is not yet registered
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NOT_FOUND);
    
    // FIXME: If nodeItem is NULL, we still return XME_STATUS_SUCCESS -- is this intended?
    if (nodeItem != NULL)
    {
        // Within the node item, look for the appropriate interface
        handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
        interfaceItem = NULL;
        XME_HAL_TABLE_GET_NEXT(nodeItem->interfaces, xme_hal_table_rowHandle_t, handle, xme_com_interface_address_t, interfaceItem, interfaceItem->addressType == addressType);

        // Return if no matching interface was found
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NOT_FOUND);
        XME_ASSERT(NULL != interfaceItem);

        *outInterfaceAddress = interfaceItem;
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
    xme_hal_table_rowHandle_t handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_node_nodeData_t* nodeItem = NULL;

    XME_CHECK(NULL != iterator, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_nodeRegistryController_nodeTable,
        xme_hal_table_rowHandle_t, handle,
        xme_core_node_nodeData_t, nodeItem,
        nodeItem->nodeId == nodeId
    );
    // The node should already be registered
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_NOT_FOUND);
    XME_ASSERT(NULL != nodeItem);
    
    *iterator = (xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t*) xme_hal_mem_alloc(sizeof(xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t));
    XME_CHECK(NULL != iterator, XME_STATUS_OUT_OF_RESOURCES);
        
    (*iterator)->nodeId = nodeId;
    (*iterator)->nodeItem = nodeItem;
    (*iterator)->addressType = addressType;
    (*iterator)->currentHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    (*iterator)->interfaceItem = NULL;

    XME_HAL_TABLE_GET_NEXT
    (
        (*iterator)->nodeItem->interfaces,
        xme_hal_table_rowHandle_t, (*iterator)->currentHandle,
        xme_com_interface_address_t, (*iterator)->interfaceItem,
        (*iterator)->interfaceItem->addressType == (*iterator)->addressType || XME_COM_INTERFACE_ADDRESS_TYPE_INVALID == (*iterator)->addressType
    );

    return XME_STATUS_SUCCESS;
}

bool
xme_core_directory_nodeRegistryController_hasNextInterface
(
    const xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t* const iterator
)
{
    XME_CHECK(NULL != iterator, false);

    XME_CHECK((XME_HAL_TABLE_INVALID_ROW_HANDLE != iterator->currentHandle && NULL != iterator->interfaceItem), false);
    
    return true;
}

xme_com_interface_address_t*
xme_core_directory_nodeRegistryController_nextInterface
(
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t *iterator
)
{
    xme_com_interface_address_t* interfaceItem = NULL;

    XME_CHECK(NULL != iterator, NULL);

    // Store the pointer to current interfaceItem in the iterator
    // This will be returned in this call
    interfaceItem = iterator->interfaceItem;

    // This check is added for wrap around of the iterator.
    // After the last item in the list the XME_HAL_TABLE_GET_NEXT sets the
    // row handle as INVALID_ROW_HANDLE and the pointer as NULL.
    // Then accordingly hasNextInterface returns false.
    // But if this function is called even when hasNextInterface has returned false
    // the iterator begins from the start and wraps around.
    // This happens because the XME_HAL_TABLE_GET_NEXT assumes INVALID_ROW_HANDLE as a parameter
    // for the table to start the iteration.
    // This is a safe check because the corresponding init function initializes the pointer to first value if it exists
    // Then this check will pass and return the correct value for nextInterface after the init.
    // And if it does not exist then this will fail and return NULL from here.
    XME_CHECK((XME_HAL_TABLE_INVALID_ROW_HANDLE != iterator->currentHandle && NULL != iterator->interfaceItem), NULL);

    iterator->interfaceItem = NULL;

    // Move the iterator to next item
    XME_HAL_TABLE_GET_NEXT
    (
        iterator->nodeItem->interfaces,
        xme_hal_table_rowHandle_t, iterator->currentHandle,
        xme_com_interface_address_t, iterator->interfaceItem,
        iterator->interfaceItem->addressType == iterator->addressType || XME_COM_INTERFACE_ADDRESS_TYPE_INVALID == iterator->addressType
    );

    return interfaceItem;
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

xme_status_t
xme_core_directory_nodeRegistryController_initNodeIDIterator
(
    xme_core_directory_nodeRegistryController_nodeIDIterator_t* iterator
)
{
    xme_hal_table_rowHandle_t handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_core_node_nodeData_t* nodeData = NULL;

    XME_CHECK(NULL != iterator, XME_STATUS_INVALID_PARAMETER);
    
    iterator->currentHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_nodeRegistryController_nodeTable,
        xme_hal_table_rowHandle_t, handle,
        xme_core_node_nodeData_t, nodeData,
        true
    );
    
    iterator->currentHandle = handle;
    
    return XME_STATUS_SUCCESS;
}

bool
xme_core_directory_nodeRegistryController_hasNextNodeID
(
    const xme_core_directory_nodeRegistryController_nodeIDIterator_t* const iterator
)
{
    XME_CHECK(NULL != iterator, false);

    return iterator->currentHandle != XME_HAL_TABLE_INVALID_ROW_HANDLE;
}

xme_core_node_nodeId_t
xme_core_directory_nodeRegistryController_nextNodeID
(
    xme_core_directory_nodeRegistryController_nodeIDIterator_t* const iterator
)
{
    xme_core_node_nodeData_t* nodeData;
    xme_core_node_nodeId_t nextNodeID = XME_CORE_NODE_INVALID_NODE_ID;

    XME_CHECK(NULL != iterator, XME_CORE_NODE_INVALID_NODE_ID);
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != iterator->currentHandle, XME_CORE_NODE_INVALID_NODE_ID);

    nodeData = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_nodeRegistryController_nodeTable, iterator->currentHandle);
    XME_CHECK(NULL != nodeData, XME_CORE_NODE_INVALID_NODE_ID);

    nextNodeID = nodeData->nodeId;

    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_nodeRegistryController_nodeTable,
        xme_hal_table_rowHandle_t,
        iterator->currentHandle,
        xme_core_node_nodeData_t,
        nodeData,
        true
    );

    return nextNodeID;
}

xme_status_t
xme_core_directory_nodeRegistryController_finiNodeIDIterator
(
    xme_core_directory_nodeRegistryController_nodeIDIterator_t* iterator
)
{
    XME_CHECK(NULL != iterator, XME_STATUS_INVALID_PARAMETER);

    // Nothing to do

    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
