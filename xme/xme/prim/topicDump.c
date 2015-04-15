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
 * $Id: topicDump.c 3170 2013-05-06 12:02:44Z ruiz $
 */

/**
 * \file
 *         Topic dump component.
 *         Subscribes to a (configurable) topic, creates a dump of the received
 *         topic data and creates a log for the dump.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/prim/topicDump.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Receives a data callback for a topic dump. 
 * \param dataHandle the data handle. 
 * \param userData the user data.
 */
void
xme_prim_topicDump_receiveDataCallback
(
	xme_hal_sharedPtr_t dataHandle,
	void* userData
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_prim_topicDump_create
(
	xme_prim_topicDump_configStruct_t* config
)
{
	// Intiailize configuration structure
	config->subscriptionHandle = XME_CORE_DCC_INVALID_SUBSCRIPTION_HANDLE;

	// Subscribe to topic
	XME_CHECK
	(
		XME_CORE_DCC_INVALID_SUBSCRIPTION_HANDLE !=
		(
			config->subscriptionHandle = xme_core_dcc_subscribeTopic(
				config->subscribedTopic,
				XME_CORE_MD_EMPTY_META_DATA,
				false,
				&xme_prim_topicDump_receiveDataCallback,
				config
			)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_prim_topicDump_activate
(
	xme_prim_topicDump_configStruct_t* config
)
{
	// Nothing to do
	return XME_STATUS_SUCCESS;
}

void
xme_prim_topicDump_deactivate
(
	xme_prim_topicDump_configStruct_t* config
)
{
	// Nothing to do
}

void
xme_prim_topicDump_destroy
(
	xme_prim_topicDump_configStruct_t* config
)
{
	// Unsubscribe from topic
	xme_core_dcc_unsubscribeTopic(config->subscriptionHandle);
	config->subscriptionHandle = XME_CORE_DCC_INVALID_SUBSCRIPTION_HANDLE;
}

void
xme_prim_topicDump_receiveDataCallback
(
	xme_hal_sharedPtr_t dataHandle,
	void* userData
)
{
	uint16_t size;
	void* data;

	xme_prim_topicDump_configStruct_t* config = (xme_prim_topicDump_configStruct_t*)userData;
	XME_ASSERT_NORVAL(NULL != config);

	size = xme_hal_sharedPtr_getSize(dataHandle);
	if (0 == size)
	{
		printf("Received empty data block\n");
		return;
	}

	data = xme_hal_sharedPtr_getPointer(dataHandle);
	XME_ASSERT_NORVAL(NULL != data);

	{
		// Print hex dump of received data block

		uint16_t numLines = ((size > 0 ? size-1 : size) >> 4) + 1;
		uint16_t line, col;

		printf("Received data block:\n");
		for (line=0; line<numLines; line++)
		{
			printf("| %04X | ", line << 4);

			for (col=0; col<16; col++)
			{
				uint16_t pos = (line << 4) + col;

				if (8 == col)
				{
					printf(" ");
				}

				if (pos < size)
				{
					printf("%02X ", ((uint8_t*)data)[pos]);
				}
				else
				{
					printf("   ");
				}
			}

			printf("| ", line);

			for (col=0; col<16; col++)
			{
				uint16_t pos = (line << 4) + col;

				if (8 == col)
				{
					printf(" ");
				}

				if (pos < size)
				{
					uint8_t c = ((uint8_t*)data)[pos];
					if (c < 32) c = '.';

					printf("%c", c);
				}
				else
				{
					printf(" ");
				}
			}

			printf("|\n");
		}
	}
}
