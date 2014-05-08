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
 * $Id: node.c 7702 2014-03-06 23:22:57Z ruiz $
 */

/**
 * \file
 *         Current local node abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/node.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/** 
 * \var xme_core_node_nodeData
 * \brief Variable to store the current node data.
 */
static xme_core_node_nodeData_t xme_core_node_nodeData;


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

/**
 * \brief initializes the self node data. 
 * \retval XME_STATUS_SUCCESS if the node data structures are correctly initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the node data structures cannot be correctly initialized. 
 */
xme_status_t
xme_core_node_init(void)
{
    xme_core_node_nodeData.nodeId = XME_CORE_NODE_INVALID_NODE_ID;
	xme_hal_safeString_strncpy(xme_core_node_nodeData.nodeName, "", sizeof(xme_core_node_nodeData.nodeName));
    XME_HAL_TABLE_INIT(xme_core_node_nodeData.interfaces);

    return XME_STATUS_SUCCESS;
}

/**
 * \brief frees the internal node data structures.
 */
void
xme_core_node_fini(void)
{
    XME_HAL_TABLE_FINI(xme_core_node_nodeData.interfaces);
}

xme_status_t
xme_core_node_setCurrentNodeId
(
    xme_core_node_nodeId_t nodeId
)
{
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);

    xme_core_node_nodeData.nodeId = nodeId;
    
    return XME_STATUS_SUCCESS;
}

xme_core_node_nodeId_t
xme_core_node_getCurrentNodeId(void)
{
    return xme_core_node_nodeData.nodeId;
}

xme_status_t
xme_core_node_setNodeName(const char* const nodeName)
{
    XME_CHECK(NULL != nodeName, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(sizeof(xme_core_node_nodeData.nodeName) > xme_hal_safeString_strnlen(nodeName, sizeof(xme_core_node_nodeData.nodeName)), XME_STATUS_INVALID_PARAMETER);

    xme_hal_safeString_strncpy(xme_core_node_nodeData.nodeName, nodeName, sizeof(xme_core_node_nodeData.nodeName));

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_node_getNodeName(char* const nodeName, uint16_t size)
{
    XME_CHECK(NULL != nodeName, XME_STATUS_INVALID_PARAMETER);	
    XME_CHECK(size > xme_hal_safeString_strnlen(xme_core_node_nodeData.nodeName, sizeof(xme_core_node_nodeData.nodeName)), XME_STATUS_INVALID_PARAMETER);

    xme_hal_safeString_strncpy(nodeName, xme_core_node_nodeData.nodeName, size);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_node_addInterface
(
    xme_com_interface_address_t interfaceAddress
)
{
    XME_HAL_TABLE_ITERATE_BEGIN
    (
        xme_core_node_nodeData.interfaces, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_com_interface_address_t, 
        addressItem
    );
    {
        if (addressItem->addressType == interfaceAddress.addressType)
        {
            if (addressItem->addressType == XME_COM_INTERFACE_ADDRESS_TYPE_IPV4)
            {
                char addressItemStr[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE+2];
                char interfaceAddressStr[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE+2];
                XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4String(addressItem, addressItemStr, sizeof(addressItemStr)), XME_STATUS_INTERNAL_ERROR);
                XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4String(&interfaceAddress, interfaceAddressStr, sizeof(interfaceAddressStr)), XME_STATUS_INTERNAL_ERROR);

                XME_CHECK(0 == strcmp(addressItemStr, interfaceAddressStr), XME_STATUS_ALREADY_EXIST);
            }
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    {
        xme_hal_table_rowHandle_t handle;
        xme_com_interface_address_t* addressItem;

        handle = XME_HAL_TABLE_ADD_ITEM(xme_core_node_nodeData.interfaces);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_OUT_OF_RESOURCES);
        addressItem = (xme_com_interface_address_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_node_nodeData.interfaces, handle);
        XME_ASSERT(NULL != addressItem);

        (void) xme_hal_mem_copy(addressItem, &interfaceAddress, sizeof(xme_com_interface_address_t));
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_node_getInterface
(
    xme_com_interface_address_t* const interfaceAddress
)
{
    XME_CHECK(NULL != interfaceAddress, XME_STATUS_INVALID_PARAMETER);

    // Search for an existing item for this node
    XME_HAL_TABLE_ITERATE_BEGIN
    (
        xme_core_node_nodeData.interfaces, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_com_interface_address_t, 
        addressItem
    );
    {
        if (addressItem->addressType == XME_COM_INTERFACE_ADDRESS_TYPE_IPV4)
        {
            (void) xme_hal_mem_copy(interfaceAddress, addressItem, sizeof(xme_com_interface_address_t));

            return XME_STATUS_SUCCESS;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_NOT_FOUND;

}

