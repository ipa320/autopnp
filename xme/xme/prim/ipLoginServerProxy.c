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
 * $Id: ipLoginServerProxy.c 3458 2013-05-23 09:37:04Z ruiz $
 */

/**
 * \file
 *         IP login server proxy.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include "xme/prim/ipLoginServerProxy.h"

#include "xme/com/packet.h"

#include "xme/core/resourceManager.h"
#include "xme/core/topic.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Function that gets called when a login request has been received.
 *
 * \param requestTopic request topic. 
 * \param requestData request data. 
 * \param responseInstanceHandle response instance handle. 
 * \param responseData response data. 
 * \param responseSize response size. 
 * \param responseTimeoutMs response timeout in miliseconds. 
 * \param userData user data. 
 * \return the response status. 
 * \see    xme_core_rr_receiveRequestCallback_t
 */
xme_core_rr_responseStatus_t
xme_prim_ipLoginServerProxy_receiveLoginRequest
(
	xme_core_topic_t requestTopic,
	void* requestData,
	xme_core_rr_responseInstanceHandle_t responseInstanceHandle,
	void* responseData,
	uint16_t* responseSize,
	xme_hal_time_timeInterval_t responseTimeoutMs,
	void* userData
);

/**
 * \brief  Callback function for checking for login responses.
 *
 * \param userData user data. 
 * \see    xme_hal_sched_taskCallback_t
 */
void
xme_prim_ipLoginServerProxy_taskCheckLoginResponses(void* userData);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_prim_ipLoginServerProxy_create(xme_prim_ipLoginServerProxy_configStruct_t* config)
{
	// TODO: How to determine on which interface this login client proxy instance listens on? See ticket #746
	static xme_com_interfaceDescr_t intf;

	// Initialize configuration structure
	config->loginRequestHandlerHandle = XME_CORE_RR_INVALID_REQUEST_HANDLER_HANDLE;
	config->loginClientSocket = XME_HAL_NET_INVALID_SOCKET_HANDLE;
	config->loginResponseTaskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;

	XME_HAL_TABLE_INIT(config->pendingResponses);

	// Announce that this component can handle login requests
	XME_CHECK
	(
		XME_CORE_RR_INVALID_REQUEST_HANDLER_HANDLE !=
		(
			config->loginRequestHandlerHandle =
				xme_core_rr_publishRequestHandler
				(
					XME_CORE_TOPIC_LOGIN_REQUEST,
					XME_CORE_MD_EMPTY_META_DATA,
					XME_CORE_TOPIC_LOGIN_RESPONSE,
					XME_CORE_MD_EMPTY_META_DATA,
					true,
					&xme_prim_ipLoginServerProxy_receiveLoginRequest,
					config
				)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	// Create a UDP socket to the port where login requests are sent to
	XME_CHECK_REC
	(
		XME_HAL_NET_INVALID_SOCKET_HANDLE !=
		(
			config->loginClientSocket =
				xme_hal_net_createSocket
				(
					&intf,
					XME_HAL_NET_SOCKET_UDP | XME_HAL_NET_SOCKET_NONBLOCKING | XME_HAL_NET_SOCKET_BROADCAST,
					0,
					XME_HAL_NET_IP_PORT_NUMBER_LOGIN
				)
		),
		XME_STATUS_OUT_OF_RESOURCES,
		{
			xme_core_rr_unpublishRequestHandler(config->loginRequestHandlerHandle);
			config->loginRequestHandlerHandle = XME_CORE_RR_INVALID_REQUEST_HANDLER_HANDLE;
		}
	);

	// Create a task that will check for incoming login responses
	XME_CHECK_REC
	(
		XME_HAL_SCHED_INVALID_TASK_HANDLE !=
		(
			config->loginResponseTaskHandle =
				xme_core_resourceManager_scheduleTask
				(
					XME_HAL_SCHED_TASK_INITIALLY_SUSPENDED,
					1000,
					XME_HAL_SCHED_PRIORITY_NORMAL,
					&xme_prim_ipLoginServerProxy_taskCheckLoginResponses,
					config
				)
		),
		XME_STATUS_OUT_OF_RESOURCES,
		{
			xme_hal_net_destroySocket(config->loginClientSocket);
			config->loginClientSocket = XME_HAL_NET_INVALID_SOCKET_HANDLE;
			xme_core_rr_unpublishRequestHandler(config->loginRequestHandlerHandle);
			config->loginRequestHandlerHandle = XME_CORE_RR_INVALID_REQUEST_HANDLER_HANDLE;
		}
	);

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_prim_ipLoginServerProxy_activate(xme_prim_ipLoginServerProxy_configStruct_t* config)
{
	// Open the UDP socket
	XME_CHECK
	(
		XME_STATUS_SUCCESS == xme_hal_net_openSocket(config->loginClientSocket),
		XME_STATUS_OUT_OF_RESOURCES
	);

	return XME_STATUS_SUCCESS;
}

void
xme_prim_ipLoginServerProxy_deactivate(xme_prim_ipLoginServerProxy_configStruct_t* config)
{
	// Close the UDP socket
	xme_hal_net_closeSocket(config->loginClientSocket);
}

void
xme_prim_ipLoginServerProxy_destroy(xme_prim_ipLoginServerProxy_configStruct_t* config)
{
	XME_HAL_TABLE_FINI(config->pendingResponses);

	xme_hal_sched_removeTask(config->loginResponseTaskHandle);
	config->loginResponseTaskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;

	xme_hal_net_destroySocket(config->loginClientSocket);
	config->loginClientSocket = XME_HAL_NET_INVALID_SOCKET_HANDLE;

	xme_core_rr_unpublishRequestHandler(config->loginRequestHandlerHandle);
	config->loginRequestHandlerHandle = XME_CORE_RR_INVALID_REQUEST_HANDLER_HANDLE;
}

xme_core_rr_responseStatus_t
xme_prim_ipLoginServerProxy_receiveLoginRequest
(
	xme_core_topic_t requestTopic,
	void* requestData,
	xme_core_rr_responseInstanceHandle_t responseInstanceHandle,
	void* responseData,
	uint16_t* responseSize,
	xme_hal_time_timeInterval_t responseTimeoutMs,
	void* userData
)
{
	xme_prim_ipLoginServerProxy_configStruct_t* config;
	xme_core_topic_loginRequestData_t* loginRequest;

	XME_UNUSED_PARAMETER(responseData);

	XME_ASSERT_RVAL(XME_CORE_TOPIC_LOGIN_REQUEST == requestTopic, XME_CORE_RR_STATUS_SERVER_ERROR);
	XME_ASSERT_RVAL(responseSize != NULL, XME_CORE_RR_STATUS_SERVER_ERROR);

	{
		uint16_t oldSize = *responseSize;

		// Set response size
		*responseSize = sizeof(xme_core_topic_loginResponseData_t);

		XME_CHECK(oldSize >= sizeof(xme_core_topic_loginResponseData_t), XME_CORE_RR_STATUS_BUFFER_TOO_SMALL);
	}

	loginRequest = (xme_core_topic_loginRequestData_t*)requestData;
	config = (xme_prim_ipLoginServerProxy_configStruct_t*)userData;

	XME_LOG
	(
		XME_LOG_VERBOSE,
		"Forwarding login request (DT=0x%08X%08X, GUID=0x%08X%08X, N=0:%d, E=%d:%d)\n",
		(uint32_t)(loginRequest->deviceType >> 32), (uint32_t)(loginRequest->deviceType & 0xFFFFFFFF),
		(uint32_t)(loginRequest->deviceGuid >> 32), (uint32_t)(loginRequest->deviceGuid & 0xFFFFFFFF),
		loginRequest->newNodeInterfaceId,
		loginRequest->edgeNodeId, loginRequest->edgeNodeInterfaceId
	);

	{
		// Send a login request via UDP broadcast
		xme_com_packet_loginRequest_t loginRequestPacket;
		XME_COM_PACKET_INIT(loginRequestPacket, XME_COM_PACKET_HEADER_TYPE_LOGINREQUEST);

		xme_hal_mem_copy(&XME_COM_PACKET_PAYLOAD(loginRequestPacket), loginRequest, sizeof(xme_core_topic_loginRequestData_t));

		XME_LOG
		(
			XME_LOG_VERBOSE,
			"Broadcasting login request (DT=0x%08X%08X, GUID=0x%08X%08X, N=0:%d, E=%d:%d)\n",
			(uint32_t)(XME_COM_PACKET_PAYLOAD(loginRequestPacket).deviceType >> 32), (uint32_t)(XME_COM_PACKET_PAYLOAD(loginRequestPacket).deviceType & 0xFFFFFFFF),
			(uint32_t)(XME_COM_PACKET_PAYLOAD(loginRequestPacket).deviceGuid >> 32), (uint32_t)(XME_COM_PACKET_PAYLOAD(loginRequestPacket).deviceGuid & 0xFFFFFFFF),
			XME_COM_PACKET_PAYLOAD(loginRequestPacket).newNodeInterfaceId,
			XME_COM_PACKET_PAYLOAD(loginRequestPacket).edgeNodeId, XME_COM_PACKET_PAYLOAD(loginRequestPacket).edgeNodeInterfaceId
		);

		if (sizeof(loginRequestPacket) != xme_hal_net_writeSocket(config->loginClientSocket, &loginRequestPacket, sizeof(loginRequestPacket)))
		{
			XME_LOG(XME_LOG_WARNING, "Unable to broadcast login request!\n");
		}
		else
		{
			xme_prim_ipLoginServerProxy_pendingResponseItem_t* responseItem = NULL;

			// Clean up the list of response handles where the corresponding
			// responses have already timed out and also determine whether we
			// have received a login request from the same device before
			XME_HAL_TABLE_ITERATE_BEGIN
			(
				config->pendingResponses,
				xme_hal_table_rowHandle_t, handle,
				xme_prim_ipLoginServerProxy_pendingResponseItem_t, item
			);
			{
				if (item->deviceType == loginRequest->deviceType && item->deviceGuid == loginRequest->deviceGuid)
				{
					responseItem = item;
				}
				else
				{
					if (xme_hal_time_getTimeInterval(&item->lastUpdate, false) > item->responseTimeoutMs)
					{
						XME_HAL_TABLE_REMOVE_ITEM(config->pendingResponses, handle);
					}
				}
			}
			XME_HAL_TABLE_ITERATE_END();

			// Remember the response handle
			if (NULL == responseItem)
			{
				xme_hal_table_rowHandle_t responseHandle;

				XME_CHECK
				(
					XME_HAL_TABLE_INVALID_ROW_HANDLE != (responseHandle = XME_HAL_TABLE_ADD_ITEM(config->pendingResponses)),
					XME_CORE_RR_STATUS_SERVER_ERROR
				);

				responseItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(config->pendingResponses, responseHandle);
				XME_ASSERT_RVAL(NULL != responseItem, XME_CORE_RR_STATUS_SERVER_ERROR);
			}

			responseItem->responseInstanceHandle = responseInstanceHandle;
			responseItem->deviceType = loginRequest->deviceType;
			responseItem->deviceGuid = loginRequest->deviceGuid;
			responseItem->responseTimeoutMs = responseTimeoutMs;
			responseItem->lastUpdate = xme_hal_time_getCurrentTime();

			// Activate the response handler task
			xme_hal_sched_setTaskExecutionState(config->loginResponseTaskHandle, true);
		}
	}

	// Response will follow as soon as available
	return XME_CORE_RR_STATUS_RESPONSE_DELAYED;
}

void
xme_prim_ipLoginServerProxy_taskCheckLoginResponses(void* userData)
{
	// TODO (See ticket #762): When can the task be suspended again? --> Only when it is asserted
	//                         that no pending login requests are still present, i.e., it is
	//                         guaranteed that all previous login requests have timed out!

	// TODO (See ticket #815): Is it possible that multiple login requests for different devices
	//                         are served over a single login server proxy? If not, we can assume
	//                         that we will only serve responses for the device type and device
	//                         GUID of the first request arriving at this component, which
	//                         simplifies things a lot.

	xme_prim_ipLoginServerProxy_configStruct_t* config = (xme_prim_ipLoginServerProxy_configStruct_t*)userData;

	// Check for login responses
	if (XME_STATUS_SUCCESS == xme_hal_net_selectSocket(config->loginClientSocket, true, false, 100))
	{
		// A login response has been received
		xme_com_packet_loginResponse_t loginResponsePacket;

		// TODO: Handle partial reception! See ticket #756
		xme_hal_net_readSocket(config->loginClientSocket, &loginResponsePacket, sizeof(loginResponsePacket));

		if (
			XME_COM_PACKET_VALID(loginResponsePacket) &&
			XME_COM_PACKET_HEADER_TYPE_LOGINRESPONSE == XME_COM_PACKET_TYPE(loginResponsePacket)
		)
		{
			xme_core_rr_responseInstanceHandle_t responseInstanceHandle = XME_CORE_RR_INVALID_RESPONSE_INSTANCE_HANDLE;
			xme_core_topic_loginResponseData_t* loginResponse;

			// Check whether we are still capable of forwarding the response to the sender
			XME_HAL_TABLE_ITERATE_BEGIN
			(
				config->pendingResponses,
				xme_hal_table_rowHandle_t, handle,
				xme_prim_ipLoginServerProxy_pendingResponseItem_t, item
			);
			{
				if (item->deviceType == XME_COM_PACKET_PAYLOAD(loginResponsePacket).deviceType && item->deviceGuid == loginResponsePacket.data.deviceGuid)
				{
					// Found a matching item
					responseInstanceHandle = item->responseInstanceHandle;
					XME_HAL_TABLE_REMOVE_ITEM(config->pendingResponses, handle);
					break;
				}
			}
			XME_HAL_TABLE_ITERATE_END();

			XME_CHECK
			(
				XME_CORE_RR_INVALID_RESPONSE_INSTANCE_HANDLE != responseInstanceHandle,
			);

			loginResponse = (xme_core_topic_loginResponseData_t*)&XME_COM_PACKET_PAYLOAD(loginResponsePacket);

			XME_LOG
			(
				XME_LOG_VERBOSE,
				"Received login response (DT=0x%08X%08X, GUID=0x%08X%08X, N=%d, E=%d:%d, AC=%d, MC=%d)\n",
				(uint32_t)(loginResponse->deviceType >> 32), (uint32_t)(loginResponse->deviceType & 0xFFFFFFFF),
				(uint32_t)(loginResponse->deviceGuid >> 32), (uint32_t)(loginResponse->deviceGuid & 0xFFFFFFFF),
				loginResponse->newNodeId,
				loginResponse->edgeNodeId, loginResponse->edgeNodeInterfaceId,
				loginResponse->remoteAnnouncementDataChannel,
				loginResponse->remoteModifyRoutingTableDataChannel
			);

			// Overwrite edge node ID with invalid node ID
			XME_ASSERT_NORVAL(XME_CORE_NODE_INVALID_NODE_ID != loginResponse->edgeNodeId);
			XME_ASSERT_NORVAL(XME_CORE_NODE_LOCAL_NODE_ID != loginResponse->edgeNodeId);
			loginResponse->edgeNodeId = XME_CORE_NODE_INVALID_NODE_ID;

			XME_LOG
			(
				XME_LOG_VERBOSE,
				"Forwarding login response (DT=0x%08X%08X, GUID=0x%08X%08X, N=%d, E=%d:%d, AC=%d, MC=%d)\n",
				(uint32_t)(loginResponse->deviceType >> 32), (uint32_t)(loginResponse->deviceType & 0xFFFFFFFF),
				(uint32_t)(loginResponse->deviceGuid >> 32), (uint32_t)(loginResponse->deviceGuid & 0xFFFFFFFF),
				loginResponse->newNodeId,
				loginResponse->edgeNodeId, loginResponse->edgeNodeInterfaceId,
				loginResponse->remoteAnnouncementDataChannel,
				loginResponse->remoteModifyRoutingTableDataChannel
			);

			XME_LOG_IF
			(
				XME_STATUS_SUCCESS !=
					xme_core_rr_sendResponse
					(
						XME_CORE_RR_STATUS_SUCCESS,
						config->loginRequestHandlerHandle,
						responseInstanceHandle,
						loginResponse,
						sizeof(*loginResponse)
					),
				XME_LOG_WARNING,
				"Unable to send login response!\n"
			);
		}
	}
}
