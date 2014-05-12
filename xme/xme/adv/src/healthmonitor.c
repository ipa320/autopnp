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
 * $Id: healthmonitor.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/** 
 * \file
 * \brief Health monitor component implementation.
 */

/** \addtogroup adv_healthmonitor
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/adv/include/healthmonitor.h"
#include "xme/adv/include/heartbeat.h"
#include "xme/core/nodeManager.h"
#include "xme/hal/include/time.h"
#include "xme/hal/include/linkedList.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
XME_HAL_TABLE
(
	xme_adv_hmon_errorMessage_t,
	xme_adv_hmon_errorLog,
	XME_ADV_HMON_ERRORS_MAX
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_adv_hmon_create
(
	xme_adv_hmon_configStruct_t* config
)
{
	// if componentId == -1 then HM is monitoring a complete node
	if ((config->componentId == -1) &&
	    (config->monitorComponent == 1))
	{
		XME_LOG(XME_LOG_ERROR, "health monitor should monitor a component, but no ID was given.\n");
		return XME_STATUS_INTERNAL_ERROR;
	}

	XME_HAL_TABLE_INIT( xme_adv_hmon_errorLog );

	// create error message publication, subscription and routes
	if (XME_STATUS_SUCCESS != xme_adv_hmon_createHealthmonitorErrorMessages(config))
	{
		return XME_STATUS_INTERNAL_ERROR;
	}
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_adv_hmon_activate
(
	xme_adv_hmon_configStruct_t* config
)
{
	return XME_STATUS_SUCCESS;
}

void
xme_adv_hmon_deactivate
(
	xme_adv_hmon_configStruct_t* config
)
{
}

void
xme_adv_hmon_destroy
(
	xme_adv_hmon_configStruct_t* config
)
{
	if ( xme_core_dcc_unsubscribeTopic(config->subHandleErrors) != XME_STATUS_SUCCESS )
	{
		XME_LOG(XME_LOG_WARNING, "Failure xme_core_dcc_unsubscribeTopic\n");
	}
	if ( xme_core_dcc_unpublishTopic(config->pubHandleErrors) != XME_STATUS_SUCCESS )
	{
		XME_LOG(XME_LOG_WARNING, "Failure xme_core_dcc_unsubscribeTopic\n");
	}

	XME_HAL_TABLE_FINI(xme_adv_hmon_errorLog);
}

void
xme_adv_hmon_receiveErrorMessage
(
	xme_hal_sharedPtr_t dataHandle,
	void* userData
)
{
	xme_adv_hmon_errorMessage_t* error;
	xme_adv_hmon_configStruct_t* config;
	xme_adv_hmon_errorMessage_t* inRow;
	xme_adv_hmon_errorMessage_t escalateError;
	xme_hal_table_rowHandle_t handle;

	error = (xme_adv_hmon_errorMessage_t*) xme_hal_sharedPtr_getPointer(dataHandle);
	config = (xme_adv_hmon_configStruct_t*)userData;

	// handle error message from monitored node
	if (error->nodeId == config->nodeId)
	{
		XME_LOG(XME_LOG_DEBUG, "Health Monitor: Received error message (nodeId %d) '%s'", error->nodeId, printHealthmonitorErrNo(error->status));

		// insert error message in error log
		if (XME_ADV_HEALTHMONITOR_TEST_OK != error->status &&
			((0 == config->monitorComponent) || (1 == config->monitorComponent && error->componentId == config->componentId)))
		{
			handle = XME_HAL_TABLE_ADD_ITEM(xme_adv_hmon_errorLog);
			inRow = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_adv_hmon_errorLog, handle);
			inRow = error;
			XME_LOG(XME_LOG_DEBUG, " and stored it in error log. \n");
		} else {
			// check for possible test reset
#if 0 //FIXME: check if this is the correct new usage of XME_HAL_TABLE_ITERATE_BEGIN
		    XME_HAL_TABLE_ITERATE_BEGIN(xme_adv_hmon_errorLog, xme_hal_table_rowHandle_t, errorMsgHandle, xme_adv_hmon_errorMessage_t, tableItem);
		    if ((error->nodeId == tableItem->nodeId) &&
                (error->identifier == tableItem->identifier) &&
                (error->status == XME_ADV_HEALTHMONITOR_TEST_OK) &&
                ((0 == config->monitorComponent) ||
                 ((1 == config->monitorComponent) &&
                  (error->componentId == tableItem->componentId))))
            {
                // if the same test reported an error earlier but is now ok, then remove it
                XME_HAL_TABLE_REMOVE_ITEM(xme_adv_healthmonitor_errorLog, errorMsgHandle);
                XME_LOG(XME_LOG_DEBUG, " and reset it in error log.\n");
            }
		    XME_HAL_TABLE_ITERATE_END();
#else
			XME_HAL_TABLE_ITERATE
			(
				xme_adv_hmon_errorLog,
				xme_hal_table_rowHandle_t, errorMsgHandle,
				xme_adv_hmon_errorMessage_t, tableItem,
				{
					if (error->nodeId == tableItem->nodeId &&
						error->identifier == tableItem->identifier &&
						error->status == XME_ADV_HEALTHMONITOR_TEST_OK &&
						((0 == config->monitorComponent) || (1 == config->monitorComponent && error->componentId == tableItem->componentId)))
					{
						// if the same test reported an error earlier but is now ok, then remove it
						XME_HAL_TABLE_REMOVE_ITEM(xme_adv_healthmonitor_errorLog, errorMsgHandle);
						XME_LOG(XME_LOG_DEBUG, " and reset it in error log.");
					}
				}
			);
#endif
			XME_LOG(XME_LOG_DEBUG, "\n");
		}

		// check health
		if (XME_STATUS_SUCCESS != checkHealth())
		{
			// init error reaction
			escalateError.nodeId = error->nodeId;

			// distinguish between node and component monitoring
			if (0 == config->monitorComponent)
			{
				escalateError.status = XME_ADV_HMON_STATUS_NODE_EXCEPTION;
			} else {
				escalateError.status = XME_ADV_HMON_STATUS_COMPONENT_EXCEPTION;
				escalateError.componentId = error->componentId;
			}
			escalateError.identifier = XME_ADV_HMON_TYPE_NOTEST;
			xme_core_dcc_sendTopicData(config->pubHandleErrors, &escalateError, sizeof(escalateError));
			XME_LOG(XME_LOG_DEBUG, "Health Monitor: Sent error message (nodeId %d): %s \n", escalateError.nodeId, printHealthmonitorErrNo(escalateError.status));

		}
	}
}

char*
xme_adv_hmon_printHealthmonitorErrNo
(
	xme_adv_hmon_status_t error
)
{
	switch(error)
	{
	case 0:
		return "XME_ADV_HMON_STATUS_COMPONENT_OK";
	case 1:
		return "XME_ADV_HMON_STATUS_COMPONENT_EXCEPTION";
	case 2:
		return "XME_ADV_HMON_STATUS_COMPONENT_UNKNOWN";
	case 3:
		return "XME_ADV_HMON_STATUS_TEST_OK";
	case 4:
		return "XME_ADV_HMON_STATUS_TEST_UNKNOWN";
	case 5:
		return "XME_ADV_HMON_STATUS_TEST_FAILED";
	case 6:
		return "XME_ADV_HMON_STATUS_NODE_OK";
	case 7:
		return "XME_ADV_HMON_STATUS_NODE_UNKNOWN";
	case 8:
		return "XME_ADV_HMON_STATUS_NODE_EXCEPTION";
	default:
	    return "XME_ADV_HMON_STATUS_UNKNOWN";
	}
}

xme_status_t
xme_adv_hmon_createHealthMonitorErrorMessages(
	xme_adv_hmon_configStruct_t* config
)
{
	// Create publisher for error messages
	config->pubHandleErrors = xme_core_dcc_publishTopic(
		XME_ADV_HEALTHMONITOR_TOPIC_ERROR_MESSAGE,
		XME_CORE_MD_EMPTY_META_DATA,
		false,
		NULL
	);

	if ( config->pubHandleErrors == XME_CORE_DCC_INVALID_PUBLICATION_HANDLE )
	{
		return XME_STATUS_INTERNAL_ERROR;
	}

	// Create subscription to error messages
	config->subHandleErrors = xme_core_dcc_subscribeTopic(
		XME_ADV_HEALTHMONITOR_TOPIC_ERROR_MESSAGE,
		XME_CORE_MD_EMPTY_META_DATA,
		false,
		receive_errorMessage,
		config
	);

	if ( config->subHandleErrors == XME_CORE_DCC_INVALID_SUBSCRIPTION_HANDLE )
	{
		return XME_STATUS_INTERNAL_ERROR;
	}

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_adv_hmon_checkHealth(void)
{
	/* TODO: implement rule check. See ticket #1387 */
	return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
