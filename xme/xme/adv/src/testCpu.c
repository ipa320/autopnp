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
 * $Id: testCpu.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/** 
 * \file
 * \brief CPU test component implementation. 
 */

/**
 * \addtogroup adv_testCPU
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/testCpu.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t 
xme_adv_testCpu_create
(
	xme_adv_testCpu_configStruct_t* config
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

	// Local channel from testCpu component
	if ( xme_core_routingTable_addLocalSourceRoute( 
		ERROR_MESSAGE_CHANNEL, 
		xme_core_dcc_getComponentFromPublicationHandle(config->pubHandle),
		xme_core_dcc_getPortFromPublicationHandle(config->pubHandle) 
	) != XME_STATUS_SUCCESS ) 	
	{
		return XME_STATUS_INTERNAL_ERROR;
	}
	
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_adv_testCpu_activate
(
	xme_adv_testCpu_configStruct_t* config
)
{
	xme_adv_hmon_errorMessage_t error;

	xme_hal_sched_addTask(config->interval, config->interval, 0, testCpu_callback, config);

	// notification that test is running
	error.componentId = xme_core_resourceManager_getCurrentComponentId();
	error.nodeId = xme_core_nodeManager_getNodeId();
	error.status = XME_ADV_HMON_STATUS_TEST_OK;
	error.identifier = XME_ADV_HMON_TYPE_CPU;
	xme_core_dcc_sendTopicData(config->pubHandle, &error, sizeof(error));

	return XME_STATUS_SUCCESS;
}

void 
xme_adv_testCpu_deactivate
(
	xme_adv_testCpu_configStruct_t* config
)
{
}

void 
xme_adv_testCpu_destroy
(
	xme_adv_testCpu_configStruct_t* config
) {
	if ( xme_core_dcc_unpublishTopic(config->pubHandle) != XME_STATUS_SUCCESS )
	{
		XME_LOG(XME_LOG_WARNING, "Failure xme_core_dcc_unsubscribeTopic\n");
	}
}

void 
xme_adv_testCpu_callback 
(
	void* userData
)
{
	xme_adv_hmon_errorMessage_t error;
	xme_status_t testResult;
	xme_adv_testCpu_configStruct_t* config = (xme_adv_testCpu_configStruct_t*)userData;

	switch(config->algorithm) 
	{
		case XME_ADV_TEST_MEMORY_ALGORITHM_LIMITED_NR_PATTERNS:
			testResult = xme_adv_testCpu_limitedNrPatterns();
			break;

		case XME_ADV_TEST_CPU_ALGORITHM_WALKING_BIT:
			testResult = xme_adv_testCpu_walkingBit(config->runlength);
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
		error.identifier = XME_ADV_HMON_TYPE_CPU;
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
xme_adv_testCpu_limitedNrPatterns(void)
{
	int calc1op1 = 85; // binary 01010101
	int calc1op2 = 170; // binary 10101010
	int calc1result= 255; // binary 11111111

	// perform test calculation
	if (calc1op1 + calc1op2 != calc1result) {
		return XME_STATUS_INTERNAL_ERROR;
	} 

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_adv_testCpu_walkingBit
(
	unsigned int runlength
)
{
	unsigned int i; /* iterator variable */
	unsigned int bitOp1, bitOp2; /* variables for bit tests */
	unsigned int intOp1, intOp2; /* variables for integer tests */

	unsigned int maxInt = 0xFFFFFFFF; /* assuming 32bit int */
	unsigned int minInt = 0x00000000;

	/* capping runlength to max value */
	if (runlength > maxInt || runlength == 0) {
		runlength = maxInt;
	}
	/* bit tests */
	bitOp1 = minInt;
	bitOp2 = maxInt;
	for (i = minInt; i <= runlength; i++) {
		if ((bitOp1 & bitOp2) != 0x00000000) {
			return XME_STATUS_INTERNAL_ERROR;
		}
		/* incrementing operands */
		bitOp1++;
		bitOp2--;
	}
	/* int tests */
	intOp1 = minInt;
	intOp2 = maxInt;
	for (i = minInt; i <= runlength; i++) {
		if (intOp1 + intOp2 != -1) {
			return XME_STATUS_INTERNAL_ERROR;
		}
		/* incrementing operands */
		intOp1++;
		intOp2--;
	}

	return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
