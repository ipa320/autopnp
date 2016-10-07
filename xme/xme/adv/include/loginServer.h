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
 * $Id: loginServer.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 * \brief Login server component
 */

#ifndef XME_ADV_LOGINSERVER_H
#define XME_ADV_LOGINSERVER_H

/**
 * \defgroup adv_loginServer Login Server Component
 * @{
 *
 * \brief This component provides the operations for creating and running a
 *        login server. All nodes wanting to join to the infrastructure, should
 *        request a login request to this component.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/com/interface.h"

#include "xme/core/device.h"
#include "xme/core/node.h"

#include "xme/hal/include/table.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_adv_loginServer_nodeIdAssignment_t
 * \brief The node assignment structure to store all information about the joining node.
 */
typedef struct
{
	xme_core_device_type_t deviceType; ///< Device type (corresponds to the value of the respective request).
	xme_core_device_guid_t deviceGuid; ///< Globally unique device identifier (e.g., serial number, MAC address, corresponds to the value of the respective request).
	xme_core_node_nodeId_t nodeId; ///< Assigned unique node identifier.
	xme_core_node_nodeId_t edgeNodeId; ///< Unique node identifier of the node that first forwarded the login request.
	xme_com_interface_interfaceId_t edgeNodeInterfaceId; ///< Node unique identifier of the interface on the edge node that first received the login request.
	xme_core_dataChannel_t remoteAnnouncementDataChannel; ///< Data channel that should be used by the local directory of the new node to send announcements to its master directory.
	xme_core_dataChannel_t remoteNeighborhoodUpdateDataChannel; ///< Data channel that should be used to send neighborhood update to the master directory.
	xme_core_dataChannel_t remoteModifyRoutingTableDataChannel; ///< Data channel used by the master directory to send modify routing table commands to the new node.
	xme_core_rr_responseInstanceHandle_t responseInstanceHandle; ///< Response instance handler for sending confirmations.
}
xme_adv_loginServer_nodeIdAssignment_t;

/**
 * \struct xme_adv_loginServer_configStruct_t
 *
 * \brief  Login server configuration structure.
 */
typedef struct
{
	// public
	xme_core_node_nodeId_t nextNodeId; ///< next node Id.
	// private
	XME_HAL_TABLE(xme_adv_loginServer_nodeIdAssignment_t, nodeIdAssignments, XME_HAL_DEFINES_MAX_LOGIN_SERVER_NODE_ID_ASSIGNMENT_ITEMS); ///< assignment table.
	xme_core_rr_requestHandlerHandle_t loginRequestHandlerHandle; ///< login request handler handle.
	xme_core_rr_requestHandle_t newNodeRequestHandle; ///< new node request handle.
}
xme_adv_loginServer_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates a login server component.
 *
 * \param config the login server configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully initialized.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_adv_loginServer_create
(
	xme_adv_loginServer_configStruct_t* config
);

/**
 * \brief  Activates a login server component.
 *
 * \param config the login server configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the login server component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the login server component cannot be activated due to an error.
 */
xme_status_t
xme_adv_loginServer_activate
(
	xme_adv_loginServer_configStruct_t* config
);

/**
 * \brief  Deactivates a login server component.
 *
 * \param config the login server configuration structure.
 */
void
xme_adv_loginServer_deactivate
(
	xme_adv_loginServer_configStruct_t* config
);

/**
 * \brief  Destroys a login server component.
 *
 * \param config the login server configuration structure.
 */
void
xme_adv_loginServer_destroy
(
	xme_adv_loginServer_configStruct_t* config
);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_ADV_LOGINSERVER_H
