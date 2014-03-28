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
 * $Id: testStub.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/** 
 * \file
 * \brief  This is the stub of a test component
 *
 * \detail This stub is based on the SAFE design. It
 *         can be used to create new test components very easily by making a
 *         copy of it and changing the sections, marked with 'TODO'. In addition,
 *         it might be necessary to extend xme_adv_testStub_configStruct_t, if
 *         additional information is required to execute the test. Please have
 *         a look at the heartbeatComponent and the heartbeatTestComponent for
 *         an example how tests with external evidence generators can be imple-
 *         mented.
 */

/**
 * \addtogroup adv_testStub
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/testStub.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t 
xme_adv_testStub_create
(
	xme_adv_testStub_configStruct_t* config
)
{
	// init instant error reaction
	errorReaction = NULL;
	errorReaction = config->callback;

	// Create publisher for error message topic
	config->pubHandle = xme_core_dcc_publishTopic( 
		ERROR_MESSAGE_TOPIC_ID, 
		XME_CORE_MD_EMPTY_META_DATA, 
		false,
		NULL
	);

	if ( pubHandle == XME_CORE_DCC_INVALID_PUBLICATION_HANDLE )
	{
		return XME_STATUS_INTERNAL_ERROR;
	} 

	// Local channel from testStub component
	if ( xme_core_routingTable_addLocalSourceRoute( 
		ERROR_MESSAGE_CHANNEL, 
		xme_core_dcc_getComponentFromPublicationHandle(pubHandle),
		xme_core_dcc_getPortFromPublicationHandle(pubHandle) 
	) != XME_STATUS_SUCCESS ) 	
	{
		return XME_STATUS_INTERNAL_ERROR;
	}
	
	return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_adv_testStub_activate
(
	xme_adv_testStub_configStruct_t* config
)
{
	xme_adv_hmon_errorMessage_t error;

	xme_hal_sched_addTask(config->interval, config->interval, 0, test_callback, config);

	// notification that test is running
	error.componentId = xme_core_resourceManager_getCurrentComponentId();
	error.nodeId = xme_core_nodeManager_getNodeId();
	error.status = XME_ADV_HMON_STATUS_TEST_OK;
	error.identifier = XME_ADV_HMON_TYPE_CPU; // TODO: change identifier for specific test
	xme_core_dcc_sendTopicData(pubHandle, &error, sizeof(error));

	return XME_STATUS_SUCCESS;
}

void 
xme_adv_testStub_deactivate
(
	xme_adv_testStub_configStruct_t* config
)
{
}

void 
xme_adv_testStub_destroy
(
	xme_adv_testStub_configStruct_t* config
)
{
	if ( xme_core_dcc_unpublishTopic(pubHandle) != XME_STATUS_SUCCESS )
	{
		XME_LOG(XME_LOG_WARNING, "Failure xme_core_dcc_unsubscribeTopic\n");
	}
}

void 
xme_adv_testStub_callback 
(
	void* userData
)
{
	xme_adv_testStub_configStruct_t* config = (xme_adv_testStub_configStruct_t*)userData;
	xme_adv_healthmonitor_errorMessage_t error;
	xme_status_t testResult;

	/* TODO: implement specific test function. Set 'testResult' to
	XME_STATUS_SUCCESS for successful tests and set it to 
	XME_STATUS_INTERNAL_ERROR if an error as been detected. */

	if (testResult != XME_STATUS_SUCCESS)
	{
		error.componentId = xme_core_resourceManager_getCurrentComponentId();
		error.nodeId = xme_core_nodeManager_getNodeId();
		error.status = XME_ADV_HMON_STATUS_TEST_FAILED;
		error.identifier = XME_ADV_HMON_TYPE_MEMORY; // TODO: change identifier for specific test
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
