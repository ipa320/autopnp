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
 * $Id: heartbeat.c 2693 2013-03-17 01:51:56Z camek $
 */

/**
 * \file
 * \brief Heartbeat sender component implementation.
 */

/**
 * \addtogroup adv_heartbeat
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/heartbeat.h"
#include "xme/defines.h"
#include "xme/core/core.h"
#include "xme/core/broker/include/broker.h"


/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/******************************************************************************/
/***   Component configurations                                             ***/
/******************************************************************************/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_adv_hbeat_create
(
	xme_adv_hbeat_configStruct_t* config
)
{
	config->taskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;

	// Create publisher for heartbeat topic
	config->pubHandle = xme_core_dcc_publishTopic(
		XME_CORE_TOPIC_LIFESIGN_HEARTBEAT,
		XME_CORE_MD_EMPTY_META_DATA,
		false,
		NULL
	);

	if ( config->pubHandle == XME_CORE_DCC_INVALID_PUBLICATION_HANDLE )
	{
		return XME_STATUS_INTERNAL_ERROR;
	}

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_adv_hbeat_activate
(
	xme_adv_hbeat_configStruct_t* config
)
{
	config->taskHandle = xme_hal_sched_addTask(config->interval, config->interval, 0, xme_adv_hbeat_ComponentCallback, config);
	return XME_STATUS_SUCCESS;
}

void
xme_adv_hbeat_deactivate
(
	xme_adv_hbeat_configStruct_t* config
)
{
	xme_hal_sched_removeTask(config->taskHandle);
	config->taskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;
}

void
xme_adv_hbeat_destroy
(
	xme_adv_hbeat_configStruct_t* config
)
{
	// remove publication
	if ( xme_core_dcc_unpublishTopic(config->pubHandle) != XME_STATUS_SUCCESS )
	{
		XME_LOG(XME_LOG_WARNING, "Failure xme_core_dcc_unsubscribeTopic\n");
	}
	config->pubHandle = XME_CORE_DCC_INVALID_PUBLICATION_HANDLE;
}

void
xme_adv_hbeat_componentCallback
(
	void* userData
)
{
	xme_adv_hbeat_configStruct_t* config = (xme_adv_hbeat_configStruct_t*)userData;
	xme_adv_hmon_heartbeat_t hb;

	hb.nodeId = xme_core_nodeManager_getDeviceGuid();
	XME_LOG(XME_LOG_DEBUG, "sending heartbeat (id %d)\n", hb.nodeId);
	xme_core_dcc_sendTopicData(config->pubHandle, &hb, sizeof(hb));
}

/** @} */
