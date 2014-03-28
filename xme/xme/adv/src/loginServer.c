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
 * $Id: loginServer.c 2693 2013-03-17 01:51:56Z camek $
 */

/**
 * \file
 * \brief Login server component implementation.
 */

/**
 * \addtogroup adv_loginServer
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/loginServer.h"

#include "xme/defines.h"
#include "xme/core/topicData.h"

#ifndef XME_CORE_DIRECTORY_TYPE_MASTER
	#error Login Server can only be used on a node that has XME_CORE_DIRECTORY_TYPE_MASTER defined!
#endif // #ifndef XME_CORE_DIRECTORY_TYPE_MASTER

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Callback function for incoming login requests.
 *
 * \param  requestTopic Topic for which data have been received.
 * \param  requestData The request data that have been received. The size,
 *         format and semantics depend on the type of request topic.
 * \param  responseInstanceHandle Identifier of the current response issue.
 *         When the function returns XME_CORE_RR_RESPONSE_DELAYED, the
 *         value of this parameter has to be passed on to the respective
 *         call of the xme_core_rr_sendResponse() function.
 * \param  responseData Response data to send to the client.
 * \param  responseSize Initially, the number of bytes available in
 *         responseData. The callback function should modify this value to
 *         represent the number of bytes actually used in responseData.
 *         If the initial number of bytes is not enough to represent the
 *         response, the callback function should set responseSize to the
 *         required size and return XME_CORE_RR_STATUS_BUFFER_TOO_SMALL.
 * \param  responseTimeout Timeout of the request in nanoseconds.
 *         Note that this parameter is just a rule of thumb when
 *         it might be too late to deliver a response, since the
 *         transmission of the request to the request handler and vice
 *         versa takes additional time.
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_rr_publishRequestHandler() where this callback function
 *         has been registered.
 *
 * \return xme_core_rr_receiveRequestCallback_t
 * \see    xme_core_rr_receiveRequestCallback_t
 */
static
xme_core_rr_responseStatus_t
xme_adv_loginServer_receiveLoginRequest
(
	xme_core_topic_t requestTopic,
	void* requestData,
	xme_core_rr_responseInstanceHandle_t responseInstanceHandle,
	void* responseData,
	uint16_t* responseSize,
	xme_hal_time_timeInterval_t responseTimeout,
	void* userData
);

/**
 * \brief  Callback function for incoming new node responses.
 *
 * \param  status One of the following response status code or a server
 *         defined value larger than or equal to XME_CORE_RR_STATUS_USER
 *         indicating whether the request completed successfully:
 *          - XME_CORE_RR_STATUS_SUCCESS if the request was served
 *            successfully.
 *          - XME_CORE_RR_STATUS_TIMEOUT if the timout interval specified in
 *            the call to xme_core_rr_sendRequest() has elapsed. If no server
 *            could be found for the given request, the callback function will
 *            be called only once with the status parameter set to this value.
 *            Otherwise, multiple invocations with success status may occur
 *            before this status is finally passed to the callback function.
 *          - XME_CORE_RR_STATUS_SERVER_ERROR if the server encountered an
 *            error while processing the given request.
 * \param  request Request handle that denotes which request type the response
 *         belongs to. This is the same value returned by the respective call
 *         to the xme_core_rr_publishRequest() function.
 * \param  requestInstanceHandle Request instance handle that denotes which
 *         request the response belongs to. This is the same value returned
 *         by the respective call to the xme_core_rr_sendRequest() function.
 * \param  responseTopic Topic for which data have been received.
 * \param  responseData The response data that have been received. The size,
 *         format and semantics depend on the type of request topic.
 * \param  responseMetaData Meta data associated with the response.
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_rr_publishRequest() where this callback function has been
 *         registered.
 * \param  instanceUserData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_rr_sendRequest().
 *
 * \see    xme_core_rr_receiveResponseCallback_t
 */
static
void
xme_adv_loginServer_receiveNewNodeResponse
(
	xme_core_rr_responseStatus_t status,
	xme_core_rr_requestHandle_t request,
	xme_core_rr_requestInstanceHandle_t requestInstanceHandle,
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
xme_adv_loginServer_create
(
	xme_adv_loginServer_configStruct_t* config
)
{
	// Initialize configuration structure
	XME_ASSERT(config->nextNodeId > XME_CORE_NODE_LOCAL_NODE_ID); // First node ID to be assigned must be at least XME_CORE_NODE_LOCAL_NODE_ID + 1
	XME_HAL_TABLE_INIT(config->nodeIdAssignments);
	config->loginRequestHandlerHandle = XME_CORE_RR_INVALID_REQUEST_HANDLER_HANDLE;
	config->newNodeRequestHandle = XME_CORE_RR_INVALID_REQUEST_HANDLE;

	XME_CHECK
	(
		XME_CORE_RR_INVALID_REQUEST_HANDLER_HANDLE !=
		(
			config->loginRequestHandlerHandle = xme_core_rr_publishRequestHandler
			(
				XME_CORE_TOPIC_LOGIN_REQUEST,
				XME_CORE_MD_EMPTY_META_DATA,
				XME_CORE_TOPIC_LOGIN_RESPONSE,
				XME_CORE_MD_EMPTY_META_DATA,
				false,
				&xme_adv_loginServer_receiveLoginRequest,
				config
			)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	XME_CHECK
	(
		XME_CORE_RR_INVALID_REQUEST_HANDLE !=
		(
			config->newNodeRequestHandle = xme_core_rr_publishRequest
			(
				XME_CORE_TOPIC_LOGIN_NEW_NODE_REQUEST,
				XME_CORE_MD_EMPTY_META_DATA,
				XME_CORE_TOPIC_LOGIN_NEW_NODE_RESPONSE,
				XME_CORE_MD_EMPTY_META_DATA,
				false,
				true,
				xme_adv_loginServer_receiveNewNodeResponse,
				config
			)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_adv_loginServer_activate
(
	xme_adv_loginServer_configStruct_t* config
)
{
	XME_UNUSED_PARAMETER(config);

	return XME_STATUS_SUCCESS;
}

void
xme_adv_loginServer_deactivate
(
	xme_adv_loginServer_configStruct_t* config
)
{
	XME_UNUSED_PARAMETER(config);
}

void
xme_adv_loginServer_destroy
(
	xme_adv_loginServer_configStruct_t* config
)
{
	xme_core_rr_unpublishRequest(config->newNodeRequestHandle);
	config->newNodeRequestHandle = XME_CORE_RR_INVALID_REQUEST_HANDLE;

	xme_core_rr_unpublishRequestHandler(config->loginRequestHandlerHandle);
	config->loginRequestHandlerHandle = XME_CORE_RR_INVALID_REQUEST_HANDLER_HANDLE;

	XME_HAL_TABLE_FINI(config->nodeIdAssignments);
}

static
xme_core_rr_responseStatus_t
xme_adv_loginServer_receiveLoginRequest
(
	xme_core_topic_t requestTopic,
	void* requestData,
	xme_core_rr_responseInstanceHandle_t responseInstanceHandle,
	void* responseData,
	uint16_t* responseSize,
	xme_hal_time_timeInterval_t responseTimeout,
	void* userData
)
{
	xme_adv_loginServer_configStruct_t* config;
	xme_core_topic_loginRequestData_t* loginRequest;
	xme_hal_table_rowHandle_t nodeIdAssignmentHandle;
	xme_adv_loginServer_nodeIdAssignment_t* assignment;

	XME_ASSERT_RVAL(XME_CORE_TOPIC_LOGIN_REQUEST == requestTopic, XME_CORE_RR_STATUS_SERVER_ERROR);
	XME_ASSERT_RVAL(responseSize != NULL, XME_CORE_RR_STATUS_SERVER_ERROR);

	XME_UNUSED_PARAMETER(responseTimeout);

	{
		uint16_t oldSize = *responseSize;

		// Set response size
		*responseSize = sizeof(xme_core_topic_loginResponseData_t);

		XME_CHECK(oldSize >= sizeof(xme_core_topic_loginResponseData_t), XME_CORE_RR_STATUS_BUFFER_TOO_SMALL);
	}

	// Set response size
	*responseSize = sizeof(xme_core_topic_loginResponseData_t);

	loginRequest = (xme_core_topic_loginRequestData_t*)requestData;
	config = (xme_adv_loginServer_configStruct_t*)userData;

	XME_LOG
	(
		XME_LOG_VERBOSE,
		"Received login request (DT=0x%08X%08X, GUID=0x%08X%08X, N=0:%d, E=%d:%d)\n",
		(uint32_t)(loginRequest->deviceType >> 32), (uint32_t)(loginRequest->deviceType & 0xFFFFFFFF),
		(uint32_t)(loginRequest->deviceGuid >> 32), (uint32_t)(loginRequest->deviceGuid & 0xFFFFFFFF),
		loginRequest->newNodeInterfaceId,
		loginRequest->edgeNodeId, loginRequest->edgeNodeInterfaceId
	);

	// Check whether we have served this request before
	nodeIdAssignmentHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
	assignment = NULL;
	XME_HAL_TABLE_GET_NEXT
	(
		config->nodeIdAssignments, xme_hal_table_rowHandle_t, nodeIdAssignmentHandle, xme_adv_loginServer_nodeIdAssignment_t, assignment,
		assignment->deviceType == loginRequest->deviceType && assignment->deviceGuid == loginRequest->deviceGuid
	);

	// TODO: Node ID assignments probably need to be persisted. See ticket #832

	if (XME_HAL_TABLE_INVALID_ROW_HANDLE != nodeIdAssignmentHandle)
	{
		XME_ASSERT_RVAL(NULL != assignment, XME_CORE_RR_STATUS_SERVER_ERROR);

		// TODO: See ticket #535!
		if (assignment->edgeNodeId != loginRequest->edgeNodeId || assignment->edgeNodeInterfaceId != loginRequest->edgeNodeInterfaceId)
		{
			// Abort
			XME_LOG
			(
				XME_LOG_VERBOSE, "Abandoning login request with different edge node or edge node interface (DT=0x%08X%08X, GUID=0x%08X%08X, N=%d, E=%d:%d, prevE=%d:%d, AC=%d, NC=%d, MC=%d)\n",
				(uint32_t)(loginRequest->deviceType >> 32), (uint32_t)(loginRequest->deviceType & 0xFFFFFFFF),
				(uint32_t)(loginRequest->deviceGuid >> 32), (uint32_t)(loginRequest->deviceGuid & 0xFFFFFFFF),
				assignment->nodeId,
				loginRequest->edgeNodeId, loginRequest->edgeNodeInterfaceId,
				assignment->edgeNodeId, assignment->edgeNodeInterfaceId,
				assignment->remoteAnnouncementDataChannel,
				assignment->remoteNeighborhoodUpdateDataChannel,
				assignment->remoteModifyRoutingTableDataChannel
			);

			return XME_CORE_RR_STATUS_SERVER_ERROR;
		}

		// XME_CORE_RR_INVALID_RESPONSE_INSTANCE_HANDLE for responseInstanceHandle means that
		// there is no pending request for login of this node at the directory.
		if (XME_CORE_RR_INVALID_RESPONSE_INSTANCE_HANDLE == assignment->responseInstanceHandle)
		{
			xme_core_topic_loginResponseData_t* loginResponse = (xme_core_topic_loginResponseData_t*)responseData;

			// We already sent a node identifier assignment to that node and the
			// original directory request w.r.t. data management channels was successful.
			// Immediately send a subsequent node identifier assignment message.
			XME_LOG
			(
				XME_LOG_VERBOSE, "Sending subsequent node identifier assignment (DT=0x%08X%08X, GUID=0x%08X%08X, N=%d, E=%d:%d, AC=%d, NC=%d, MC=%d)\n",
				(uint32_t)(loginRequest->deviceType >> 32), (uint32_t)(loginRequest->deviceType & 0xFFFFFFFF),
				(uint32_t)(loginRequest->deviceGuid >> 32), (uint32_t)(loginRequest->deviceGuid & 0xFFFFFFFF),
				assignment->nodeId,
				assignment->edgeNodeId, assignment->edgeNodeInterfaceId,
				assignment->remoteAnnouncementDataChannel,
				assignment->remoteNeighborhoodUpdateDataChannel,
				assignment->remoteModifyRoutingTableDataChannel
			);

			// Prepare the reply
			loginResponse->deviceType = loginRequest->deviceType;
			loginResponse->deviceGuid = loginRequest->deviceGuid;
			loginResponse->newNodeId = assignment->nodeId;
			loginResponse->edgeNodeId = loginRequest->edgeNodeId;
			loginResponse->edgeNodeInterfaceId = loginRequest->edgeNodeInterfaceId;
			loginResponse->remoteAnnouncementDataChannel = assignment->remoteAnnouncementDataChannel;
			loginResponse->remoteNeighborhoodUpdateDataChannel = assignment->remoteNeighborhoodUpdateDataChannel;
			loginResponse->remoteModifyRoutingTableDataChannel = assignment->remoteModifyRoutingTableDataChannel;

			return XME_CORE_RR_STATUS_SUCCESS;
		}
	}
	else
	{
		xme_core_node_nodeId_t newNodeId;

		XME_ASSERT_RVAL(NULL == assignment, XME_CORE_RR_STATUS_SERVER_ERROR);

		nodeIdAssignmentHandle = XME_HAL_TABLE_ADD_ITEM(config->nodeIdAssignments);
		assignment = (xme_adv_loginServer_nodeIdAssignment_t*)XME_HAL_TABLE_ITEM_FROM_HANDLE(config->nodeIdAssignments, nodeIdAssignmentHandle);

		// Allocate a new node identifier
		newNodeId = config->nextNodeId++;

		assignment->deviceType = loginRequest->deviceType;
		assignment->deviceGuid = loginRequest->deviceGuid;
		assignment->nodeId = newNodeId;
		assignment->edgeNodeId = loginRequest->edgeNodeId;
		assignment->edgeNodeInterfaceId = loginRequest->edgeNodeInterfaceId;
		assignment->remoteAnnouncementDataChannel = XME_CORE_DATACHANNEL_INVALID_DATACHANNEL;
		assignment->remoteModifyRoutingTableDataChannel = XME_CORE_DATACHANNEL_INVALID_DATACHANNEL;
		assignment->remoteNeighborhoodUpdateDataChannel = XME_CORE_DATACHANNEL_INVALID_DATACHANNEL;
		assignment->responseInstanceHandle = XME_CORE_RR_INVALID_RESPONSE_INSTANCE_HANDLE;
	}

	// Actually, we would have to manage a list here that contains
	// all relevant response instance handles that correspond to
	// the given node (to guarantee that all requests are actually
	// replied to in case multiple requests for the same node appear
	// in fast succession). However, we simplify it here by only
	// remembering the most recent response instance handle.
	if (XME_CORE_RR_INVALID_RESPONSE_INSTANCE_HANDLE != assignment->responseInstanceHandle)
	{
		xme_core_rr_sendResponse(XME_CORE_RR_STATUS_SERVER_ERROR, config->loginRequestHandlerHandle, assignment->responseInstanceHandle, NULL, 0);
	}
	assignment->responseInstanceHandle = responseInstanceHandle;

	// Ask the directory for management channel numbers for the new node.
	// Since the response will be delivered asynchronously, we return from
	// this function and send our response at a later point in time.
	{
		xme_core_topic_newNodeRequestData_t newNodeRequest;
		newNodeRequest.newNodeId = assignment->nodeId;
		newNodeRequest.newNodeInterface = loginRequest->newNodeInterfaceId;
		newNodeRequest.edgeNodeId = loginRequest->edgeNodeId;
		newNodeRequest.edgeNodeInterface = loginRequest->edgeNodeInterfaceId;

		XME_CHECK
		(
			XME_CORE_RR_INVALID_REQUEST_INSTANCE_HANDLE !=
			(
				xme_core_rr_sendRequest(config->newNodeRequestHandle, &newNodeRequest, sizeof(newNodeRequest), assignment, 10000)
			),
			XME_CORE_RR_STATUS_SERVER_ERROR
		);
	}

	return XME_CORE_RR_STATUS_RESPONSE_DELAYED;
}

static
void
xme_adv_loginServer_receiveNewNodeResponse
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
	// Prepare node identifier assignment
	xme_core_topic_loginResponseData_t loginResponse;
	xme_adv_loginServer_configStruct_t* config = (xme_adv_loginServer_configStruct_t*)userData;
	xme_adv_loginServer_nodeIdAssignment_t* assignment = (xme_adv_loginServer_nodeIdAssignment_t*)instanceUserData;
	xme_core_topic_newNodeResponseData_t* newNodeResponse = (xme_core_topic_newNodeResponseData_t*)responseData;

	XME_ASSERT_NORVAL(XME_CORE_RR_STATUS_SUCCESS == status);
	XME_CHECK(XME_CORE_RR_INVALID_RESPONSE_INSTANCE_HANDLE != assignment->responseInstanceHandle, );

	XME_UNUSED_PARAMETER(request);
	XME_UNUSED_PARAMETER(requestInstance);
	XME_UNUSED_PARAMETER(responseTopic);
	XME_UNUSED_PARAMETER(responseMetaData);

	assignment->remoteAnnouncementDataChannel = newNodeResponse->remoteAnnouncementDataChannel;
	assignment->remoteNeighborhoodUpdateDataChannel = newNodeResponse->neighborhoodUpdateDataChannel;
	assignment->remoteModifyRoutingTableDataChannel = newNodeResponse->remoteModifyRoutingTableDataChannel;

	XME_LOG
	(
		XME_LOG_VERBOSE, "Sending initial node identifier assignment (DT=0x%08X%08X, GUID=0x%08X%08X, N*=%d, E=%d:%d, AC=%d, NC=%d, MC=%d)\n",
		(uint32_t)(assignment->deviceType >> 32), (uint32_t)(assignment->deviceType & 0xFFFFFFFF),
		(uint32_t)(assignment->deviceGuid >> 32), (uint32_t)(assignment->deviceGuid & 0xFFFFFFFF),
		assignment->nodeId,
		assignment->edgeNodeId, assignment->edgeNodeInterfaceId,
		assignment->remoteAnnouncementDataChannel,
		assignment->remoteNeighborhoodUpdateDataChannel,
		assignment->remoteModifyRoutingTableDataChannel
	);

	// Prepare the reply
	loginResponse.deviceType = assignment->deviceType;
	loginResponse.deviceGuid = assignment->deviceGuid;
	loginResponse.newNodeId = assignment->nodeId;
	loginResponse.edgeNodeId = assignment->edgeNodeId;
	loginResponse.edgeNodeInterfaceId = assignment->edgeNodeInterfaceId;
	loginResponse.remoteAnnouncementDataChannel = assignment->remoteAnnouncementDataChannel;
	loginResponse.remoteNeighborhoodUpdateDataChannel = assignment->remoteNeighborhoodUpdateDataChannel;
	loginResponse.remoteModifyRoutingTableDataChannel = assignment->remoteModifyRoutingTableDataChannel;

	xme_core_rr_sendResponse(XME_CORE_RR_STATUS_SUCCESS, config->loginRequestHandlerHandle, assignment->responseInstanceHandle, &loginResponse, sizeof(loginResponse));
	assignment->responseInstanceHandle = XME_CORE_RR_INVALID_RESPONSE_INSTANCE_HANDLE;
}

/**
 * @}
 */
