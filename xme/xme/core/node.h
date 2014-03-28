/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: node.h 6311 2014-01-14 14:01:15Z geisinger $
 */

/**
 * \file
 *         Node related defines.
 */

#ifndef XME_CORE_NODE_H
#define XME_CORE_NODE_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include "xme/com/interface.h"
#include "xme/hal/include/table.h"

#include <stdint.h>
/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_core_node_nodeId_t
 *
 * \brief  Unique node identifier.
 */
typedef enum
{
    XME_CORE_NODE_INVALID_NODE_ID = 0, ///< Invalid node identifier.
    XME_CORE_NODE_LOCAL_NODE_ID = 1, ///< Node identifier for local-only communication.
    XME_CORE_NODE_MAX_NODE_ID = XME_MAX_SYSTEM_VALUE ///< Largest possible node identifier.
}
xme_core_node_nodeId_t;

/**
 * \typedef xme_core_node_guid_t
 * \brief This datatype represents the randomly generated id by the login requester node. 
 * \details This generated id identifies unambiguously the node. 
 */
typedef uint64_t xme_core_node_guid_t;


/**
 * \struct xme_core_node_nodeData_t
 * \brief The structure to store the node and corresponding table of interface addresses.
 */
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< Node identifier of the node.
    char nodeName[256];  ///< Human readable node name.
    xme_core_node_guid_t guid; ///< Globally unique identifier of the node.
    XME_HAL_TABLE(xme_com_interface_address_t, interfaces, XME_CORE_NODE_INTERFACE_TABLE_SIZE); ///< Table of network interfaces of the node.
}
xme_core_node_nodeData_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief initializes the self node data.
 *
 * \retval XME_STATUS_SUCCESS if the node data structures are correctly initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the node data structures cannot be correctly initialized. 
 */
xme_status_t
xme_core_node_init(void);

/**
 * \brief frees the internal data structures.
 */
void
xme_core_node_fini(void);

/**
 * \brief sets the current Node Id on which the instance is running.
 *
 * \param nodeId Id of the node.
 *
 * \retval XME_STATUS_SUCCESS if the current local node id has been successfully established. 
 * \retval XME_STATUS_INVALID_PARAMETER if the provided input parameter is not a valid 
 *         node id. 
 */
xme_status_t
xme_core_node_setCurrentNodeId
(
    xme_core_node_nodeId_t nodeId
);

/**
 * \brief gets the current Node Id on which the instance is running.
 *
 * \return xme_core_node_nodeId_t the current node Id
 */
xme_core_node_nodeId_t
xme_core_node_getCurrentNodeId(void);

/**
 * \brief Sets the name of the node on which the instance is running.
 *
 * \param nodeName Name of the node.
 *
 * \retval XME_STATUS_SUCCESS if the node name has been successfully set. 
 * \retval XME_STATUS_INVALID_PARAMETER if the node name is to long.
 */
xme_status_t
xme_core_node_setNodeName(const char* const nodeName);

/**
 * \brief Gets the name of the node on which the instance is running.
 *
 * \param nodeName Buffer for the name of the node.
 * \param size Size of the node name buffer.
 *
 * \retval XME_STATUS_SUCCESS if the node name has been successfully set. 
 * \retval XME_STATUS_INVALID_PARAMETER if the buffer is to short for the name.
 */
xme_status_t
xme_core_node_getNodeName(char* const nodeName, uint16_t size);

/**
 * \brief adds the interface address to the current node.
 *
 * \param interfaceAddress Address of the interface to be added.
 *
 * \return xme_status_t XME_STATUS_SUCCESS if the interface gets successfully added to the give nodeId.
 * \return xme_status_t XME_STATUS_INVALID_PARAMETER if the interface address is invalid.
 * \return xme_status_t XME_STATUS_OUT_OF_RESOURCES if the addition fails.
 */
// TODO: Pass interface address by reference instead of by value?
// FIXME: Shouldn't this API be deprecated after the introduction of the Node Registry Controller?
xme_status_t
xme_core_node_addInterface
(
    xme_com_interface_address_t interfaceAddress
);

/**
 * \brief This function returns the interface address for the given node. 
 * \details if there are more than one interfaces  the interface address to the current node.
 *
 * \param interfaceAddress Address of a variable to fill with the interface address.
 *
 * \return xme_status_t XME_STATUS_SUCCESS if the interface gets successfully added to the give nodeId.
 * \return xme_status_t XME_STATUS_INVALID_PARAMETER if interfaceAddress was NULL.
 * \return xme_status_t XME_STATUS_OUT_OF_RESOURCES if the addition fails.
 */
// FIXME: Shouldn't this API be deprecated after the introduction of the Node Registry Controller?
xme_status_t
xme_core_node_getInterface
(
    xme_com_interface_address_t* const interfaceAddress
);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_NODE_H
