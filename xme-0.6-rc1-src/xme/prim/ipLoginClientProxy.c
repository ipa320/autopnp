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
 * $Id: ipLoginClientProxy.c 3170 2013-05-06 12:02:44Z ruiz $
 */

/**
 * \file
 *         IP login client proxy.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/prim/ipLoginClientProxy.h"

#include "xme/defines.h"
#include "xme/com/packet.h"
#include "xme/core/topic.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Callback function for checking for login (address assignment)
 *         requests sent over UDP on the associated interface.
 * \param userData user data. 
 * \see    xme_hal_sched_taskCallback_t
 */
void
xme_prim_ipLoginClientProxy_taskCheckLoginRequests(void* userData);

/**
 * \brief  Function that gets called when a login reponse has been received.
 *
 * \param status status. 
 * \param request request. 
 * \param requestInstance request instance. 
 * \param responseTopic response topic. 
 * \param responseData response data. 
 * \param responseMetaData response metadata. 
 * \param userData user data. 
 * \param instanceUserData instance user data. 
 * \see    xme_core_rr_receiveResponseCallback_t
 */
void
xme_prim_ipLoginClientProxy_receiveLoginResponse
(
	xme_core_rr_responseStatus_t status,
	xme_core_rr_requestHandle_t request,
	xme_core_rr_requestInstanceHandle_t requestInstance,
	xme_core_topic_t responseTopic,
	void* responseData,
	xme_core_md_topicMetaDataHandle_t responseMetaData,
	void* userData,
	void* instanceUserData
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_prim_ipLoginClientProxy_create(xme_prim_ipLoginClientProxy_configStruct_t* config)
{
	// Initialize configuration structure
	config->loginRequestHandle = XME_CORE_RR_INVALID_REQUEST_HANDLE;
	config->managementChannelsToNewNode = XME_CORE_DCC_INVALID_PUBLICATION_HANDLE;
	config->loginTaskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;
	config->loginServerSocket = XME_HAL_NET_INVALID_SOCKET_HANDLE;

	// Announce that this component is going to send login requests
	// when it receives a broadcast IP frame on the login UDP port
	XME_CHECK
	(
		XME_CORE_RR_INVALID_REQUEST_HANDLE !=
		(
			config->loginRequestHandle =
				xme_core_rr_publishRequest
				(
					XME_CORE_TOPIC_LOGIN_REQUEST,
					XME_CORE_MD_EMPTY_META_DATA,
					XME_CORE_TOPIC_LOGIN_RESPONSE,
					XME_CORE_MD_EMPTY_META_DATA,
					false,
					false,
					&xme_prim_ipLoginClientProxy_receiveLoginResponse,
					config
				)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	// Publish managment channels to new node topic
	XME_CHECK
	(
		XME_CORE_DCC_INVALID_PUBLICATION_HANDLE !=
		(
			config->managementChannelsToNewNode =
				xme_core_dcc_publishTopic
				(
					XME_CORE_TOPIC_LOGIN_MANAGEMENT_CHANNELS_TO_NEW_NODE,
					XME_CORE_MD_EMPTY_META_DATA,
					true,
					NULL
				)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	// Create a UDP socket that listens on the port where login requests are sent to
	XME_CHECK_REC
	(
		XME_HAL_NET_INVALID_SOCKET_HANDLE !=
		(
			config->loginServerSocket =
				xme_hal_net_createSocket
				(
					xme_com_interface_getInterface(config->interfaceId),
					XME_HAL_NET_SOCKET_UDP | XME_HAL_NET_SOCKET_BROADCAST | XME_HAL_NET_SOCKET_NONBLOCKING,
					0,
					XME_HAL_NET_IP_PORT_NUMBER_LOGIN
				)
		),
		XME_STATUS_OUT_OF_RESOURCES,
		{
			xme_core_rr_unpublishRequest(config->loginRequestHandle);
			config->loginRequestHandle = XME_CORE_RR_INVALID_REQUEST_HANDLE;
		}
	);

	// Open the UDP socket. This will bind the port to the local address.
	XME_CHECK_REC
	(
		XME_STATUS_SUCCESS == xme_hal_net_openSocket(config->loginServerSocket),
		XME_STATUS_INVALID_CONFIGURATION,
		{
			xme_hal_net_destroySocket(config->loginServerSocket);
			config->loginServerSocket = XME_HAL_NET_INVALID_SOCKET_HANDLE;
			xme_core_rr_unpublishRequest(config->loginRequestHandle);
			config->loginRequestHandle = XME_CORE_RR_INVALID_REQUEST_HANDLE;
		}
	);

	// Create a task that will be called every few seconds and check for incoming login requests
	XME_CHECK_REC
	(
		XME_HAL_SCHED_INVALID_TASK_HANDLE !=
		(
			config->loginTaskHandle =
				xme_hal_sched_addTask
				(
					XME_HAL_SCHED_TASK_INITIALLY_SUSPENDED,
					1000,
					XME_HAL_SCHED_PRIORITY_NORMAL,
					&xme_prim_ipLoginClientProxy_taskCheckLoginRequests,
					config
				)
		),
		XME_STATUS_OUT_OF_RESOURCES,
		{
			xme_hal_net_destroySocket(config->loginServerSocket);
			config->loginServerSocket = XME_HAL_NET_INVALID_SOCKET_HANDLE;
			xme_core_rr_unpublishRequest(config->loginRequestHandle);
			config->loginRequestHandle = XME_CORE_RR_INVALID_REQUEST_HANDLE;
		}
	);

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_prim_ipLoginClientProxy_activate(xme_prim_ipLoginClientProxy_configStruct_t* config)
{
	// Run the "checkLoginRequests" task
	XME_CHECK
	(
		XME_STATUS_SUCCESS == xme_hal_sched_setTaskExecutionState(config->loginTaskHandle, true),
		XME_STATUS_INTERNAL_ERROR
	);

	return XME_STATUS_SUCCESS;
}

void
xme_prim_ipLoginClientProxy_deactivate(xme_prim_ipLoginClientProxy_configStruct_t* config)
{
	// Stop the "checkLoginRequests" task
	XME_CHECK
	(
		XME_STATUS_SUCCESS == xme_hal_sched_setTaskExecutionState(config->loginTaskHandle, false),
	);
}

void
xme_prim_ipLoginClientProxy_destroy(xme_prim_ipLoginClientProxy_configStruct_t* config)
{
	xme_hal_net_destroySocket(config->loginServerSocket);
	config->loginServerSocket = XME_HAL_NET_INVALID_SOCKET_HANDLE;

	{
		xme_status_t rval = xme_core_dcc_unpublishTopic(config->managementChannelsToNewNode);
		XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == rval);
	}

	xme_core_rr_unpublishRequest(config->loginRequestHandle);
	config->loginRequestHandle = XME_CORE_RR_INVALID_REQUEST_HANDLE;
}

void
xme_prim_ipLoginClientProxy_taskCheckLoginRequests(void* userData)
{
	bool requestServed = false;

	xme_prim_ipLoginClientProxy_configStruct_t* config = (xme_prim_ipLoginClientProxy_configStruct_t*)userData;
	XME_ASSERT_NORVAL(NULL != config);
	XME_ASSERT_NORVAL(XME_HAL_SCHED_INVALID_TASK_HANDLE != config->loginTaskHandle);

	// Exit if node is not already logged in
	if (XME_CORE_NODE_LOCAL_NODE_ID == xme_core_nodeManager_getNodeId())
	{
		return;
	}

	// Check for login requests
	while (XME_STATUS_SUCCESS == xme_hal_net_selectSocket(config->loginServerSocket, true, false, 1))
	{
		// A request has been received
		xme_com_packet_loginRequest_t loginRequest;
		requestServed = true;

		// TODO: Handle partial reception! See ticket #756
		if (sizeof(loginRequest) == xme_hal_net_readSocket(config->loginServerSocket, &loginRequest, sizeof(loginRequest)))
		{
			// Ensure that the packet is valid
			if (!XME_COM_PACKET_VALID(loginRequest))
			{
				continue;
			}

			// Ensure that the packet contains a login request
			if (XME_COM_PACKET_HEADER_TYPE_LOGINREQUEST != XME_COM_PACKET_TYPE(loginRequest))
			{
				continue;
			}

			XME_LOG
			(
				XME_LOG_VERBOSE,
				"Received login request (DT=0x%08X%08X, GUID=0x%08X%08X, N=0:%d, E=%d:%d)\n",
				(uint32_t)(loginRequest.data.deviceType >> 32), (uint32_t)(loginRequest.data.deviceType & 0xFFFFFFFF),
				(uint32_t)(loginRequest.data.deviceGuid >> 32), (uint32_t)(loginRequest.data.deviceGuid & 0xFFFFFFFF),
				loginRequest.data.newNodeInterfaceId,
				loginRequest.data.edgeNodeId, loginRequest.data.edgeNodeInterfaceId
			);

			// Overwrite edge node ID with own node ID
			loginRequest.data.edgeNodeId = xme_core_nodeManager_getNodeId();
			XME_ASSERT_NORVAL(XME_CORE_NODE_INVALID_NODE_ID != loginRequest.data.edgeNodeId);
			XME_ASSERT_NORVAL(XME_CORE_NODE_LOCAL_NODE_ID != loginRequest.data.edgeNodeId);

			// Overwrite edge node interface ID with own interface ID
			loginRequest.data.edgeNodeInterfaceId = 1; // TODO: replace with real interface ID! (cmp. ticket #536) See #547
			XME_ASSERT_NORVAL(XME_COM_INTERFACEMANAGER_INVALID_INTERFACE_ID != loginRequest.data.edgeNodeInterfaceId);

			XME_LOG
			(
				XME_LOG_VERBOSE,
				"Forwarding login request (DT=0x%08X%08X, GUID=0x%08X%08X, N=0:%d, E=%d:%d)\n",
				(uint32_t)(loginRequest.data.deviceType >> 32), (uint32_t)(loginRequest.data.deviceType & 0xFFFFFFFF),
				(uint32_t)(loginRequest.data.deviceGuid >> 32), (uint32_t)(loginRequest.data.deviceGuid & 0xFFFFFFFF),
				loginRequest.data.newNodeInterfaceId,
				loginRequest.data.edgeNodeId, loginRequest.data.edgeNodeInterfaceId
			);

			// Send a login request matching the received data
			// TODO: Maybe use the timeout value of the original request! See ticket #797
			XME_LOG_IF
			(
				XME_CORE_RR_INVALID_REQUEST_INSTANCE_HANDLE ==
					xme_core_rr_sendRequest
					(
						config->loginRequestHandle,
						&loginRequest.data,
						sizeof(loginRequest.data),
						0, // TODO: encode request issuer IP address. See ticket #834
						1000
					),
				XME_LOG_ERROR,
				"Unable to forward login request!\n"
			);
		}
	}

	if (!requestServed)
	{
		XME_LOG(XME_LOG_VERBOSE, "No pending login requests\n");
	}
}

void
xme_prim_ipLoginClientProxy_receiveLoginResponse
(
	xme_core_rr_responseStatus_t status,
	xme_core_rr_requestHandle_t request,
	xme_core_rr_requestInstanceHandle_t requestInstance,
	xme_core_topic_t responseTopic,
	void* responseData,
	xme_core_md_topicMetaDataHandle_t responseMetaData,
	void* userData,
	void* instanceUserData
)
{
	// TODO: Login response data sent from login server to login client proxy does not need to be completely forwarded to login server proxy! (ticket #567)
	xme_prim_ipLoginClientProxy_configStruct_t* config = (xme_prim_ipLoginClientProxy_configStruct_t*)userData;
	xme_core_topic_loginResponseData_t* loginResponse = (xme_core_topic_loginResponseData_t*)responseData;
	XME_ASSERT_NORVAL(NULL != config);
	XME_ASSERT_NORVAL(NULL != loginResponse);

	XME_UNUSED_PARAMETER(request);
	XME_UNUSED_PARAMETER(requestInstance);
	XME_UNUSED_PARAMETER(responseTopic);
	XME_UNUSED_PARAMETER(responseMetaData);
	XME_UNUSED_PARAMETER(instanceUserData);

	// TODO: In case the request timed out, we might have to clean up our peer table! See ticket #833

	if (XME_CORE_RR_STATUS_SUCCESS != status)
	{
		// Ignore errors and timeouts - those have to be detected at the
		// receiving side anyway
		return;
	}

	XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(loginResponse->edgeNodeId == xme_core_nodeManager_getNodeId()));

	// Forward the response to the request issuer
	// TODO: Extract the correct peer address from instanceUserData. See ticket #834
	XME_LOG
	(
		XME_LOG_VERBOSE,
		"Forwarding login response to peer (DT=0x%08X%08X, GUID=0x%08X%08X, N=%d, E=%d:%d, AC=%d, NC=%d, MC=%d)\n",
		(uint32_t)(loginResponse->deviceType >> 32), (uint32_t)(loginResponse->deviceType & 0xFFFFFFFF),
		(uint32_t)(loginResponse->deviceGuid >> 32), (uint32_t)(loginResponse->deviceGuid & 0xFFFFFFFF),
		loginResponse->newNodeId,
		loginResponse->edgeNodeId, loginResponse->edgeNodeInterfaceId,
		loginResponse->remoteAnnouncementDataChannel,
		loginResponse->remoteNeighborhoodUpdateDataChannel,
		loginResponse->remoteModifyRoutingTableDataChannel
	);

	// Forward the response to new node
	{
		xme_com_packet_loginResponse_t loginResponsePacket;
		XME_COM_PACKET_INIT(loginResponsePacket, XME_COM_PACKET_HEADER_TYPE_LOGINRESPONSE);

		loginResponsePacket.data.deviceType = loginResponse->deviceType;
		loginResponsePacket.data.deviceGuid = loginResponse->deviceGuid;
		loginResponsePacket.data.newNodeId = loginResponse->newNodeId;
		loginResponsePacket.data.edgeNodeId = loginResponse->edgeNodeId;
		loginResponsePacket.data.edgeNodeInterfaceId = loginResponse->edgeNodeInterfaceId;
		loginResponsePacket.data.remoteAnnouncementDataChannel = loginResponse->remoteAnnouncementDataChannel;
		loginResponsePacket.data.remoteNeighborhoodUpdateDataChannel = loginResponse->remoteNeighborhoodUpdateDataChannel;
		loginResponsePacket.data.remoteModifyRoutingTableDataChannel = loginResponse->remoteModifyRoutingTableDataChannel;

		// Send reply
		// TODO: Send it out on interface loginResponse->edgeNodeInterfaceId (cmp. ticket #536)
		XME_LOG_IF
		(
			sizeof(loginResponsePacket) != xme_hal_net_writeSocket(config->loginServerSocket, &loginResponsePacket, sizeof(loginResponsePacket)),
			XME_LOG_WARNING,
			"Unable to forward login response!\n"
		);
	}

	// Notify directory to set up management channels for new node
	{
		xme_core_topic_managementChannelsPacket_t managementChannels;

		managementChannels.interfaceId = loginResponse->edgeNodeInterfaceId;
		managementChannels.remoteAnnouncementDataChannel = loginResponse->remoteAnnouncementDataChannel;
		managementChannels.remoteNeighborhoodUpdateDataChannel = loginResponse->remoteNeighborhoodUpdateDataChannel;
		managementChannels.remoteModifyRoutingTableDataChannel = loginResponse->remoteModifyRoutingTableDataChannel;

		// Publish publication
		{
			xme_status_t rval = xme_core_dcc_sendTopicData(config->managementChannelsToNewNode, &managementChannels, sizeof(managementChannels));
			XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == rval);
		}
	}
}
