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
 * $Id: randomNumberGenerator.c 3172 2013-05-06 12:43:28Z ruiz $
 */

/**
 * \file
 *         Random number generator.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/prim/randomNumberGenerator.h"

#include "xme/defines.h"
#include "xme/core/dcc.h"

#include "xme/hal/include/random.h"

/******************************************************************************/
/***   Forward declarations                                                 ***/
/******************************************************************************/
/**
 * \brief creates a task callback with user data.
 * \param userData user data. 
 */
void
xme_prim_randomNumberGenerator_taskCallback(void* userData);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_prim_randomNumberGenerator_create(xme_prim_randomNumberGenerator_configStruct_t* config)
{
	// Initialize configuration structure
	config->publicationHandle = XME_CORE_DCC_INVALID_PUBLICATION_HANDLE;
	config->taskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;

	XME_CHECK
	(
		XME_CORE_DCC_INVALID_PUBLICATION_HANDLE !=
		(
			config->publicationHandle = xme_core_dcc_publishTopic
			(
				XME_CORE_TOPIC_UNSIGNED_INTEGER,
				XME_CORE_MD_EMPTY_META_DATA,
				false,
				NULL
			)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	XME_CHECK
	(
		XME_HAL_SCHED_INVALID_TASK_HANDLE !=
		(
			config->taskHandle = xme_core_resourceManager_scheduleTask
			(
				XME_HAL_SCHED_TASK_INITIALLY_SUSPENDED,
				config->interval,
				XME_HAL_SCHED_PRIORITY_NORMAL,
				&xme_prim_randomNumberGenerator_taskCallback,
				config
			)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_prim_randomNumberGenerator_activate(xme_prim_randomNumberGenerator_configStruct_t* config)
{
	XME_CHECK
	(
		XME_STATUS_SUCCESS == xme_hal_sched_setTaskExecutionState(config->taskHandle, true),
		XME_STATUS_INTERNAL_ERROR
	);

	return XME_STATUS_SUCCESS;
}

void
xme_prim_randomNumberGenerator_deactivate(xme_prim_randomNumberGenerator_configStruct_t* config)
{
	XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == xme_hal_sched_setTaskExecutionState(config->taskHandle, false));
}

void
xme_prim_randomNumberGenerator_destroy(xme_prim_randomNumberGenerator_configStruct_t* config)
{
	XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == xme_hal_sched_removeTask(config->taskHandle));
	config->taskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;

	XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == xme_core_dcc_unpublishTopic(config->publicationHandle));
	config->publicationHandle = XME_CORE_DCC_INVALID_PUBLICATION_HANDLE;
}

void
xme_prim_randomNumberGenerator_taskCallback(void* userData)
{
	xme_prim_randomNumberGenerator_configStruct_t* config = (xme_prim_randomNumberGenerator_configStruct_t*)userData;

	xme_core_topic_integerData_t data;
	data.number = xme_hal_random_randRange(config->minValue, config->maxValue);

	printf("Generated random number: %d <= %d <= %d\n", config->minValue, data.number, config->maxValue);

	{
		xme_status_t rval = xme_core_dcc_sendTopicData(config->publicationHandle, &data, sizeof(data));
		XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == rval);
	}
}
