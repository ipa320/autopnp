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
 * $Id: testMemory.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/** 
 * \file
 * \brief Memory test component implementation.
 */

/**
 * \addtogroup adv_testMemory
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/adv/include/testMemory.h"

#include "xme/core/interfaceManager.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t 
xme_adv_testMemory_create
(
	xme_adv_testMemory_configStruct_t* config
)
{
	// Create publisher for error message topic
	config->pubHandle = xme_core_dcc_publishTopic( 
		ERROR_MESSAGE_TOPIC_ID, 
		XME_CORE_MD_EMPTY_META_DATA, 
		false,
		NULL
	);

	if ( config->pubHandle == XME_CORE_DCC_INVALID_PUBLICATION_HANDLE )
	{
		return XME_STATUS_INTERNAL_ERROR;
	} 

	// Local channel from memory test component
	if ( xme_core_routingTable_addLocalSourceRoute( 
		ERROR_MESSAGE_CHANNEL, 
		xme_core_dcc_getComponentFromPublicationHandle(config->pubHandle),
		xme_core_dcc_getPortFromPublicationHandle(config->pubHandle) 
	) != XME_STATUS_SUCCESS ) 	
	{
		return XME_STATUS_INTERNAL_ERROR;
	}

	// Remote channel from memory test component
	if ( xme_core_routingTable_addOutboundRoute(
		ERROR_MESSAGE_CHANNEL,
		(xme_core_interface_interfaceId_t) 1
	) != XME_STATUS_SUCCESS ) 	
	{
		return XME_STATUS_INTERNAL_ERROR;
	};
	
	return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_adv_testMemory_activate
(
	xme_adv_testMemory_configStruct_t* config
)
{
	 xme_adv_hmon_errorMessage_t error;

	 xme_hal_sched_addTask(config->interval, config->interval, 0, xme_adv_testMemory_callback, config);

	// notification that test is running
	error.componentId = xme_core_resourceManager_getCurrentComponentId();
	error.nodeId = xme_core_nodeManager_getNodeId();
	error.status = XME_ADV_HMON_STATUS_TEST_OK;
	error.identifier = XME_ADV_HMON_TYPE_MEMORY;
	xme_core_dcc_sendTopicData(config->pubHandle, &error, sizeof(error));

	return XME_STATUS_SUCCESS;
}

void 
xme_adv_testMemory_deactivate
(
	xme_adv_testMemory_configStruct_t* config
)
{
}

void 
xme_adv_testMemory_destroy
(
	xme_adv_testMemory_configStruct_t* config
) 
{
	if ( xme_core_dcc_unpublishTopic(config->pubHandle) != XME_STATUS_SUCCESS )
	{
		XME_LOG(XME_LOG_WARNING, "Failure xme_core_dcc_unsubscribeTopic\n");
	}
}

void
xme_adv_testMemory_callback
(
	void* userData
) 
{
	xme_adv_hmon_errorMessage_t error;
	xme_status_t testResult;
	xme_adv_testMemory_configStruct_t* config = (xme_adv_testMemory_configStruct_t*)userData;

	switch(config->algorithm) 
	{
		case XME_ADV_TEST_MEMORY_ALGORITHM_WALKPATH:
			testResult = xme_adv_testMemory_walkpath(config->startAddress, config->stopAddress);
			break;

		case XME_ADV_TEST_MEMORY_ALGORITHM_INVERT:
			testResult = xme_adv_testMemory_invert(config->startAddress, config->stopAddress);
			break;

		default:
			XME_LOG(XME_LOG_WARNING, "Selected memory test algoritm not implemented\n");
			break;
	}

	if (testResult != XME_STATUS_SUCCESS)
	{
		error.componentId = xme_core_resourceManager_getCurrentComponentId();
		error.nodeId = xme_core_nodeManager_getNodeId();
		error.status = XME_ADV_HMON_STATUS_TEST_FAILED;
		error.identifier = XME_ADV_HMON_TYPE_MEMORY;
		xme_core_dcc_sendTopicData(config->pubHandle, &error, sizeof(error));

		// instant error reaction
		if (config->callback != NULL) 
		{
			config->callback();
		} else {
			printf("error function not defined\n");
		}
	}
}

xme_status_t
xme_adv_testMemory_walkpath
(
	int start, 
	int stop
) 
{
	return XME_STATUS_INTERNAL_ERROR;
}

xme_status_t
xme_adv_testMemory_invert
(
	int start, 
	int stop
) 
{
	int i = start;
	char pointer; // pointer to the current memory cell
	char newValue;

	while (i < stop) 
	{
		pointer = *((char *) (i));
		if ('a' == *((char *) (i))) 
		{
			newValue = 'b';
		} else {
			newValue = 'a';
		}
		
		*((char *) (i)) = newValue; // overwrite memory cell
		/* check, if the overwriting was successful */
		if (newValue != *((char *) (i)))
		{
			// overwriting was not successful; return an error
			return XME_STATUS_INTERNAL_ERROR;
		} else {
			// restore memory cell
			*((char *) (i)) = pointer; 
		}
		i++;
	}

	return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
