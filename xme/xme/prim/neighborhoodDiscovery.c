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
 * $Id: neighborhoodDiscovery.c 3286 2013-05-14 15:38:20Z geisinger $
 */

/**
 * \file
 *         Component for automatic neigborhood detection.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/prim/neighborhoodDiscovery.h"

#include "xme/core/nodeManager/loginClient.h"

/******************************************************************************/
/***   Forward declarations                                                 ***/
/******************************************************************************/
/**
 * \brief callbacks the working task with associated data. 
 * \param userData user data. 
 */
void
xme_prim_neighborhoodDiscovery_workingTask_callback
(
	void* userData
);

/**
 * \brief Sends an announcement. 
 * \param config the configuration information. 
 */
void
xme_prim_neighborhoodDiscovery_sendAnnouncement(
	xme_prim_neighborhoodDiscovery_configStruct_t* config
);

/**
 * \brief Process a received announcement. 
 * \param config the configuration data. 
 * \param data the received data. 
 * \param receivedInterfaceID received interface id. 
 */
void
xme_prim_neighborhoodDiscovery_processReceivedAnnouncement(
	xme_prim_neighborhoodDiscovery_configStruct_t* config,
	xme_prim_neighborhoodDiscovery_packetFormatNodeAnnouncements_t* data,
	xme_com_interface_interfaceId_t receivedInterfaceID
);

/**
 * \brief Sends the information to the directory. 
 * \param config the configuration data. 
 */
void
xme_prim_neighborhoodDiscovery_sendInformationToDirectory(
	xme_prim_neighborhoodDiscovery_configStruct_t* config
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_prim_neighborhoodDiscovery_create(xme_prim_neighborhoodDiscovery_configStruct_t* config)
{
	xme_com_interface_interfaceId_t interfaceIterator;

	config->neighborhoodUpdateHandle = XME_CORE_DCC_INVALID_PUBLICATION_HANDLE;

	XME_CHECK
	(
		XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != (config->criticalSectionHandle = xme_hal_sync_createCriticalSection()),
		XME_STATUS_OUT_OF_RESOURCES
	);

	XME_CHECK
	(
	 	XME_HAL_SCHED_INVALID_TASK_HANDLE !=
		(
			config->taskHandle = xme_core_resourceManager_scheduleTask
			(
				XME_HAL_SCHED_TASK_STATE_SUSPENDED,
				config->announcementInterval,
				XME_HAL_SCHED_PRIORITY_NORMAL,
				xme_prim_neighborhoodDiscovery_workingTask_callback,
				config
			)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	XME_HAL_TABLE_INIT( config->neighborTable );

	interfaceIterator = xme_com_interfaceManager_getFirstInterface();
	while ( interfaceIterator != XME_COM_INTERFACEMANAGER_INVALID_INTERFACE_ID )
	{
		// Incoming traffic
		xme_com_interfaceManager_join_channel
		(
			interfaceIterator,
			config->dataChannelForAnnouncements
		);

		// Outgoing traffic
		xme_com_interfaceManager_provide_channel(
			interfaceIterator,
			config->dataChannelForAnnouncements
		);

		interfaceIterator = xme_com_interfaceManager_getNextInterface( interfaceIterator );
	}

	// Publish neighborhood update
	XME_CHECK
	(
		XME_CORE_DCC_INVALID_PUBLICATION_HANDLE !=
		(
			config->neighborhoodUpdateHandle =
				xme_core_dcc_publishTopic
				(
					XME_CORE_TOPIC_ROUTES_LOCAL_NEIGHBORHOOD_UPDATE,
					XME_CORE_MD_EMPTY_META_DATA,
					true,
					NULL
				)
		),
		XME_STATUS_OUT_OF_RESOURCES
	);

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_prim_neighborhoodDiscovery_activate(xme_prim_neighborhoodDiscovery_configStruct_t* config)
{
	XME_CHECK
	(
		XME_STATUS_SUCCESS == xme_hal_sched_setTaskExecutionState(config->taskHandle, true),
		XME_STATUS_INTERNAL_ERROR
	);

	xme_com_interfaceManager_registerCallback( xme_prim_neighborhoodDiscovery_interfaceManager_callback, config );

	return XME_STATUS_SUCCESS;
}

void
xme_prim_neighborhoodDiscovery_deactivate(xme_prim_neighborhoodDiscovery_configStruct_t* config)
{
	xme_status_t rval;

#if 0
	// TODO: When the interface manager is implemented as a real registry, this has to vanish (Issue #1961)!
	xme_com_interfaceManager_registerCallback( xme_core_broker_interfaceManager_callback, NULL );
#else // #if 0
	XME_LOG(XME_LOG_WARNING, "Calling of Interface Manager not implemented!\n");
#endif // #if 0

	rval = xme_hal_sched_setTaskExecutionState(config->taskHandle, false);
	XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == rval);
}

void
xme_prim_neighborhoodDiscovery_destroy(xme_prim_neighborhoodDiscovery_configStruct_t* config)
{
	xme_status_t rval;
	xme_com_interface_interfaceId_t interfaceIterator;

	rval = xme_core_dcc_unpublishTopic(config->neighborhoodUpdateHandle);
	XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == rval);

	rval = xme_hal_sched_removeTask(config->taskHandle);
	XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == rval);

	config->taskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;

	XME_HAL_TABLE_FINI( config->neighborTable );

	interfaceIterator = xme_com_interfaceManager_getFirstInterface();
	while ( interfaceIterator != XME_COM_INTERFACEMANAGER_INVALID_INTERFACE_ID )
	{
		// Incoming traffic
		xme_com_interfaceManager_leave_channel
		(
			interfaceIterator,
			config->dataChannelForAnnouncements
		);

		// Outgoing traffic
		xme_com_interfaceManager_unprovide_channel(
			interfaceIterator,
			config->dataChannelForAnnouncements
		);

		interfaceIterator = xme_com_interfaceManager_getNextInterface( interfaceIterator );
	}

	xme_hal_sync_destroyCriticalSection(config->criticalSectionHandle);
	config->criticalSectionHandle = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;
}

void
xme_prim_neighborhoodDiscovery_interfaceManager_callback
(
	xme_com_interface_state_t status,
	xme_com_interface_interfaceId_t interfaceHandle,
	xme_core_dataChannel_t channel,
	xme_hal_sharedPtr_t dataHandle,
	void* userData
)
{
	xme_prim_neighborhoodDiscovery_packetFormatNodeAnnouncements_t* data;
	xme_prim_neighborhoodDiscovery_configStruct_t* config = (xme_prim_neighborhoodDiscovery_configStruct_t*)userData;

	// Check if announcement packet received
	if ( status == XME_COM_INTERFACE_STATE_DATA_AVAILABLE )
	{
		if ( channel == config->dataChannelForAnnouncements )
		{
			data = (xme_prim_neighborhoodDiscovery_packetFormatNodeAnnouncements_t *)xme_hal_sharedPtr_getPointer( dataHandle );

			XME_LOG(
				XME_LOG_VERBOSE,
				"neighborhoodDiscovery: Received neighborhood discovery announcement: Sending Node %d, sending Interface %d\n",
				data->sendingNodeID,
				data->sendingInterfaceID
			);

			xme_prim_neighborhoodDiscovery_processReceivedAnnouncement(
				config,
				data,
				interfaceHandle
			);
			return;
		}
	}

#if 0
	// TODO: When the interface manager is implemented as a real registry, this has to vanish (Issue #1961)!
	xme_core_broker_interfaceManager_callback(
		status,
		interfaceHandle,
		channel,
		dataHandle,
		NULL
	);
#else // #if 0
	XME_LOG(XME_LOG_WARNING, "Calling of Interface Manager not implemented!\n");
#endif // #if 0
}

void
xme_prim_neighborhoodDiscovery_workingTask_callback
(
	void* userData
)
{
	xme_prim_neighborhoodDiscovery_configStruct_t* config = (xme_prim_neighborhoodDiscovery_configStruct_t *)userData;
	xme_core_node_nodeId_t myNodeID;

	myNodeID = xme_core_nodeManager_loginClient_getNodeId();

	if (( myNodeID != XME_CORE_NODE_INVALID_NODE_ID ) &&
		( myNodeID != XME_CORE_NODE_LOCAL_NODE_ID ) )
	{
		xme_prim_neighborhoodDiscovery_sendAnnouncement( config );
		xme_prim_neighborhoodDiscovery_sendInformationToDirectory( config );
	}

	XME_LOG(XME_LOG_DEBUG, "neighborhoodDiscovery: Executing working task\n");

	xme_hal_sync_enterCriticalSection(config->criticalSectionHandle);
	{
		XME_HAL_TABLE_ITERATE_BEGIN
		(
			config->neighborTable,
			xme_hal_table_rowHandle_t, tableHandle,
			xme_prim_neighborhoodDiscovery_neighborInformation_t, item
		);
		{
			XME_LOG(
				XME_LOG_DEBUG,
				"neighborhoodDiscovery: Table entry> Sender Node %d / Sender Interface %d / Receiving Interface %d / Ticks until removal %d\n",
				item->senderNodeID,
				item->senderInterfaceID,
				item->receivingInterfaceID,
				item->lastSeenBeforeTicks
			);

			if ( --(item->lastSeenBeforeTicks) <= 0 )
			{
				XME_LOG(
					XME_LOG_VERBOSE,
					"neighborhoodDiscovery: Removing entry from sender node %d sender interface %d receiving interface %d due to timeout\n",
					item->senderNodeID,
					item->senderInterfaceID,
					item->receivingInterfaceID
				);

				XME_HAL_TABLE_REMOVE_ITEM
				(
					config->neighborTable,
					tableHandle
				);
				config->overflow = false;
				break;
			}
		}
		XME_HAL_TABLE_ITERATE_END();
	}
	xme_hal_sync_leaveCriticalSection(config->criticalSectionHandle);

}

void
xme_prim_neighborhoodDiscovery_sendAnnouncement(xme_prim_neighborhoodDiscovery_configStruct_t* config)
{
	xme_prim_neighborhoodDiscovery_packetFormatNodeAnnouncements_t *data;
	xme_hal_sharedPtr_t packetToSend;
	xme_com_interface_interfaceId_t interfaceIterator;

	packetToSend = xme_hal_sharedPtr_create( sizeof(xme_prim_neighborhoodDiscovery_packetFormatNodeAnnouncements_t) );
	data = (xme_prim_neighborhoodDiscovery_packetFormatNodeAnnouncements_t *)xme_hal_sharedPtr_getPointer( packetToSend );
	data->sendingNodeID = xme_core_nodeManager_loginClient_getNodeId();

	interfaceIterator = xme_com_interfaceManager_getFirstInterface();

	XME_LOG(
		XME_LOG_VERBOSE,
		"neighborhoodDiscovery: Sending node announcement via channel %d\n",
		config->dataChannelForAnnouncements
	);

	while ( interfaceIterator != XME_COM_INTERFACEMANAGER_INVALID_INTERFACE_ID )
	{
		data->sendingInterfaceID = interfaceIterator;

		xme_com_interfaceManager_write_blocking(
			(xme_com_interface_interfaceId_t)data->sendingInterfaceID,
			config->dataChannelForAnnouncements,
			packetToSend
		);

		interfaceIterator = xme_com_interfaceManager_getNextInterface( interfaceIterator );
	}

	xme_hal_sharedPtr_destroy( packetToSend );
}

void
xme_prim_neighborhoodDiscovery_processReceivedAnnouncement(
	xme_prim_neighborhoodDiscovery_configStruct_t* config,
	xme_prim_neighborhoodDiscovery_packetFormatNodeAnnouncements_t* data,
	xme_com_interface_interfaceId_t receivedInterfaceID
)
{
	xme_prim_neighborhoodDiscovery_neighborInformation_t* item;
	xme_hal_table_rowHandle_t handle;

	xme_hal_sync_enterCriticalSection(config->criticalSectionHandle);
	{
		XME_HAL_TABLE_ITERATE_BEGIN
		(
			config->neighborTable,
			xme_hal_table_rowHandle_t, tableHandle,
			xme_prim_neighborhoodDiscovery_neighborInformation_t, item
		);
		{
			if ( item->senderInterfaceID == data->sendingInterfaceID &&
				item->senderNodeID == data->sendingNodeID &&
				item->receivingInterfaceID == receivedInterfaceID)
			{
				item->lastSeenBeforeTicks = config->ticksBeforeRemovingReceivedAnnouncements;
				xme_hal_sync_leaveCriticalSection(config->criticalSectionHandle);
				return;
			}
		}
		XME_HAL_TABLE_ITERATE_END();

		handle = XME_HAL_TABLE_ADD_ITEM ( config->neighborTable );
		if ( handle == XME_HAL_TABLE_INVALID_ROW_HANDLE )
		{
			XME_LOG(
				XME_LOG_WARNING,
				"neighborhoodDiscovery: Overflow. Not able to keep complete list of neighborhood.\n"
			);
			config->overflow = true;
			xme_hal_sync_leaveCriticalSection(config->criticalSectionHandle);
			return;
		}
		item = XME_HAL_TABLE_ITEM_FROM_HANDLE( config->neighborTable, handle);

		item->senderInterfaceID = (xme_com_interface_interfaceId_t) data->sendingInterfaceID;
		item->senderNodeID = (xme_core_node_nodeId_t)data->sendingNodeID;
		item->receivingInterfaceID = receivedInterfaceID;
		item->lastSeenBeforeTicks = config->ticksBeforeRemovingReceivedAnnouncements;
	}
	xme_hal_sync_leaveCriticalSection(config->criticalSectionHandle);
}

void
xme_prim_neighborhoodDiscovery_sendInformationToDirectory(
	xme_prim_neighborhoodDiscovery_configStruct_t* config
)
{

	xme_hal_sharedPtr_t mem;

	xme_hal_sync_enterCriticalSection(config->criticalSectionHandle);
	{
		xme_core_topic_neighborhoodUpdate_messageHeader_t* msgHeader;
		xme_core_topic_neighborhoodUpdate_neighborItem_t* msgItem;
		uint16_t sizeOfMem;

		sizeOfMem = sizeof(xme_core_topic_neighborhoodUpdate_messageHeader_t) +
			( sizeof(xme_core_topic_neighborhoodUpdate_neighborItem_t) *
			XME_HAL_TABLE_ITEM_COUNT( config->neighborTable ) );

		mem = xme_hal_sharedPtr_create( sizeOfMem );
		if ( mem == XME_HAL_SHAREDPTR_INVALID_POINTER )
		{
			XME_LOG(
				XME_LOG_WARNING,
				"neighborhoodDiscovery: Out of ressources. Not able to send information to directory.\n"
			);
			xme_hal_sync_leaveCriticalSection(config->criticalSectionHandle);
		}

		msgHeader = (xme_core_topic_neighborhoodUpdate_messageHeader_t *)xme_hal_sharedPtr_getPointer( mem );
		msgHeader->nodeId =  xme_core_nodeManager_loginClient_getNodeId();
		msgHeader->overflow = config->overflow;
		msgHeader->neighborsCount = XME_HAL_TABLE_ITEM_COUNT( config->neighborTable );

		msgItem = (xme_core_topic_neighborhoodUpdate_neighborItem_t *)(msgHeader+1);

		XME_HAL_TABLE_ITERATE_BEGIN
		(
			config->neighborTable,
			xme_hal_table_rowHandle_t, tableHandle,
			xme_prim_neighborhoodDiscovery_neighborInformation_t, item
		);
		{
			msgItem->sendInterfaceId = item->senderInterfaceID;
			msgItem->nodeId = item->senderNodeID;
			msgItem->receiveInterfaceId = item->receivingInterfaceID;

			msgItem++;
		}
		XME_HAL_TABLE_ITERATE_END();
	}
	xme_hal_sync_leaveCriticalSection(config->criticalSectionHandle);

	// Send message to local directory
	xme_core_dcc_sendTopicData(
		config->neighborhoodUpdateHandle,
		xme_hal_sharedPtr_getPointer( mem ),
		xme_hal_sharedPtr_getSize( mem )
	);

	xme_hal_sharedPtr_destroy(mem);
}
