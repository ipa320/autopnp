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
 * $Id: canTransceiver.c 3170 2013-05-06 12:02:44Z ruiz $
 */

/**
 * \file
 *         CAN transceiver.
 *
 *         This component is the wrapper between the HAL and the DCC for CAN.
 *
  */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/prim/canTransceiver.h"

/******************************************************************************/
/***   Forward declarations                                                 ***/
/******************************************************************************/
/**
 * \brief  Sends a CAN request.
 *
 * \param  dataHandle the data handler.
 * \param  param parameters.
 */
void 
xme_prim_canTransceiver_sendRequest
(
	xme_hal_sharedPtr_t dataHandle, 
	void* param
);

/**
 * \brief  Sets the working task for the CAN.
 *
 * \param  param parameters containing the working task.
 */
void 
xme_prim_canTransceiver_workinkgTask
(
	void* param
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_prim_canTransceiver_create(xme_prim_canTransceiver_configStruct_t* config)
{
	config->isActivated = false;
	xme_hal_can_init(config->port,config->speed,NULL);

	config->handleSubCANSendRaw = xme_core_dcc_subscribeTopic(
		config->topicCANSendRaw,
		XME_CORE_MD_EMPTY_META_DATA,
		true,
		xme_prim_canTransceiver_sendRequest,
		config
	);

	XME_CHECK_MSG
	(
		config->handleSubCANSendRaw != XME_CORE_DCC_INVALID_SUBSCRIPTION_HANDLE,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_FATAL,
		"xme_prim_canTransceiver_create failed\n"
	);

	config->handlePubCANReceiveRaw = xme_core_dcc_publishTopic(
		config->topicCANReceiveRaw,
		XME_CORE_MD_EMPTY_META_DATA,
		true,
		NULL
	);

	XME_CHECK_MSG
	(
		config->handlePubCANReceiveRaw != XME_CORE_DCC_INVALID_PUBLICATION_HANDLE,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_FATAL,
		"xme_prim_canTransceiver_create failed\n"
	);

	config->handlePubCANError = xme_core_dcc_publishTopic(
		config->topicCANError,
		XME_CORE_MD_EMPTY_META_DATA,
		true,
		NULL
	);

	XME_CHECK_MSG
	(
		config->handlePubCANError != XME_CORE_DCC_INVALID_PUBLICATION_HANDLE,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_FATAL,
		"xme_prim_canTransceiver_create failed\n"
	);
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_prim_canTransceiver_activate(xme_prim_canTransceiver_configStruct_t* config)
{
    xme_status_t rval = XME_STATUS_INTERNAL_ERROR;
	if ((config->handlePubCANError != XME_CORE_DCC_INVALID_PUBLICATION_HANDLE ) &&
	    (config->handleSubCANSendRaw != XME_CORE_DCC_INVALID_SUBSCRIPTION_HANDLE ) &&
	    (config->handlePubCANError != XME_CORE_DCC_INVALID_PUBLICATION_HANDLE ))
	{
	    config->handleWorkingTask = xme_hal_sched_addTask(
	            0,
	            config->periodOfWorkingTask,
	            XME_HAL_SCHED_PRIORITY_NORMAL,
	            xme_prim_canTransceiver_workinkgTask,
	            config);

	    XME_CHECK_MSG
	    (
	        config->handleWorkingTask != XME_HAL_SCHED_INVALID_TASK_HANDLE,
	        XME_STATUS_INTERNAL_ERROR,
	        XME_LOG_FATAL,
	        "xme_hal_sched_addTask(xme_prim_canTransceiver_workinkgTask) failed\n"
	    );

	    config->isActivated = true;

	    rval = XME_STATUS_SUCCESS;
	}
	return rval;
}

xme_status_t
xme_prim_canTransceiver_deactivate(xme_prim_canTransceiver_configStruct_t* config)
{
	if ( xme_hal_sched_removeTask( config->handleWorkingTask ) != XME_STATUS_SUCCESS )
	{
		XME_LOG_CONSOLE(XME_LOG_WARNING, "Could not remove task in xme_prim_canTransceiver_deactivate\n");
	}
	config->isActivated = false;
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_prim_canTransceiver_destroy(xme_prim_canTransceiver_configStruct_t* config)
{
	xme_status_t rval;

	rval = xme_core_dcc_unsubscribeTopic(config->handleSubCANSendRaw);

	XME_CHECK_MSG
	(
		rval == XME_STATUS_SUCCESS,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_FATAL,
		"xme_prim_canTransceiver_destroy failed\n"
	);

	rval = xme_core_dcc_unpublishTopic(config->handlePubCANReceiveRaw);

	XME_CHECK_MSG
	(
		rval == XME_STATUS_SUCCESS,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_FATAL,
		"xme_prim_canTransceiver_destroy failed\n"
	);

	rval = xme_core_dcc_unpublishTopic(config->topicCANError);

	XME_CHECK_MSG
	(
		rval == XME_STATUS_SUCCESS,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_FATAL,
		"xme_prim_canTransceiver_destroy failed\n"
	);

	config->isActivated = false;
	xme_hal_can_fini(config->port);
	return rval;
}

void
xme_prim_canTransceiver_sendRequest(xme_hal_sharedPtr_t dataHandle, void* param)
{
	xme_hal_can_message_t msg;
	xme_prim_canTransceiver_configStruct_t* config = (xme_prim_canTransceiver_configStruct_t*)param;
	xme_prim_canTransceiver_can_message_t *tmsg;
	register uint8_t x;

	if ( config->isActivated == false )
	{
		XME_LOG(XME_LOG_DEBUG, "CAN message not sent because of failed initialization.\n");
		return;
	}

	if ( xme_hal_can_getStateTX(config->port) == XME_HAL_CAN_STATE_TX_BUSY )
	{
		XME_LOG(XME_LOG_DEBUG, "CAN message not sent because of buffer overrun.\n");
		return;
	}

	tmsg = (xme_prim_canTransceiver_can_message_t *)xme_hal_sharedPtr_getPointer(dataHandle);
	msg.msgID = tmsg->msgID;
	msg.msgLen = tmsg->msgLen;
	for ( x = 0; x < tmsg->msgLen; ++x)
	{
		msg.data[x] = tmsg->data[x];
	}

	xme_hal_can_sendMessage(config->port,msg);
}

void
xme_prim_canTransceiver_workinkgTask(void* param)
{
	register uint8_t x;
	xme_prim_canTransceiver_configStruct_t* config = (xme_prim_canTransceiver_configStruct_t*)param;
	xme_prim_canTransceiver_can_message_t tmsg;
	xme_prim_canTransceiver_can_error_t emsg;
	xme_hal_can_message_t msg;

	if ( xme_hal_can_getLastError(config->port) != XME_HAL_CAN_ERROR_NONE )
	{
		xme_hal_can_clearLastError(config->port);
		if ( config->handlePubCANError != XME_CORE_DCC_INVALID_PUBLICATION_HANDLE )
		{
			xme_core_dcc_sendTopicData(
					config->handlePubCANError,
					(void *)&emsg,
					sizeof(emsg)
			);
		}
	}

	while ( xme_hal_can_getStateRX(config->port) == XME_HAL_CAN_STATE_RX_MESSAGE_AVAILABLE )
	{
		xme_hal_can_getMessage(config->port, &msg);

		tmsg.msgID = msg.msgID;
		tmsg.msgLen = msg.msgLen;
		for ( x = 0; x<msg.msgLen; ++x)
		{
			tmsg.data[x] = msg.data[x];
		}

		if ( config->handlePubCANReceiveRaw != XME_CORE_DCC_INVALID_PUBLICATION_HANDLE )
		{
			xme_core_dcc_sendTopicData(
					config->handlePubCANReceiveRaw,
					(void *)&tmsg,
					sizeof(tmsg)
			);
		}
	}
}
