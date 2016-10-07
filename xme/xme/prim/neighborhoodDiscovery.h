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
 * $Id: neighborhoodDiscovery.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         Component for automatic neigborhood detection.
 *
 */

#ifndef XME_PRIM_NEIGBORHOODDISCOVERY_H
#define XME_PRIM_NEIGBORHOODDISCOVERY_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/broker/include/broker.h"
#include "xme/core/dataChannel.h"
#include "xme/core/topic.h"

#include "xme/com/interfaceManager.h"
#include "xme/com/packet.h"

#include "xme/hal/include/sync.h"

/******************************************************************************/
/***   Model constraints                                                    ***/
/******************************************************************************/

// TODO: Broker has to be started before neighborhood discovery!

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
#define XME_PRIM_NEIGHBORHOODDISCOVERY_STANDARD_VALUE_ANNOUNCEMENT_INTERVAL ((xme_hal_time_timeInterval_t)1000000000ULL) ///< announcement interval. 
#define XME_PRIM_NEIGHBORHOODDISCOVERY_STANDARD_VALUE_TICKS_BEFORE_REMOVAL (5) ///< ticks before removal. 

/**
 * \struct xme_prim_neighborhoodDiscovery_neighborInformation_t
 *
 * \brief  Structure for storing information about one node of the neighborhood.
 */
typedef struct
{
	xme_com_interface_interfaceId_t senderInterfaceID; ///< Interface of sending node
	xme_core_node_nodeId_t senderNodeID; ///< ID of sending node
	xme_com_interface_interfaceId_t receivingInterfaceID; ///< ID of interface that received the announcement
	int8_t lastSeenBeforeTicks; ///< Entry will be removed if lastSeenBeforeTicks becomes zero.
}
xme_prim_neighborhoodDiscovery_neighborInformation_t;

/**
 * \struct xme_prim_neighborhoodDiscovery_configStruct_t
 *
 * \brief  Neigborhood detection configuration structure.
 */
XME_COMPONENT_CONFIG_STRUCT
(
	xme_prim_neighborhoodDiscovery,

	// public
	xme_hal_time_timeInterval_t announcementInterval; ///< Interval in nanoseconds annoucements will be sent by this node.
	xme_core_dataChannel_t dataChannelForAnnouncements; ///< The data channel for the announcements.
	uint8_t ticksBeforeRemovingReceivedAnnouncements; ///< Number of ticks before the information of a detected neighbor times out.

	// private
	xme_core_resourceManager_taskHandle_t taskHandle; ///< Handle to the working task.
	XME_HAL_TABLE(xme_prim_neighborhoodDiscovery_neighborInformation_t, neighborTable, XME_PRIM_NEIGHBORHOODISCOVERY_MAX_NUMBER_OF_TRACKED_NEIGHBORS); ///< Table with the information about the neighbors.
	bool overflow; ///< Overflow is true if not all neighborhood information could be stored.
	xme_hal_sync_criticalSectionHandle_t criticalSectionHandle; ///< Critical section handle for protecting critical regions.
	xme_core_dcc_publicationHandle_t neighborhoodUpdateHandle; ///< Publication handle for neighborhood update messages.
);

/*	Example configuration

	XME_COMPONENT_CONFIG_INSTANCE(xme_prim_neighborhoodDiscovery) =
	{
		// public
		XME_PRIM_NEIGHBORHOODDISCOVERY_STANDARD_VALUE_ANNOUNCEMENT_INTERVAL, // Interval in ms annoucements will be sent by this node.
		XME_CORE_DATACHANNEL_NEIGHBORHOOD_ANNOUNCEMENTS, // The data channel for the announcements.
		XME_PRIM_NEIGHBORHOODDISCOVERY_STANDARD_VALUE_TICKS_BEFORE_REMOVAL, // Number of ticks before the information of a detected neighbor times out.
	};

*/

/**
 * \struct xme_prim_neighborhoodDiscovery_packetFormatNodeAnnouncements_t
 *
 * \brief  Packet format for node announcements.
 */
#pragma pack(push, 1)
typedef struct
{
	uint16_t sendingInterfaceID; ///< Interface ID of sending node.
	uint16_t sendingNodeID; ///< Node ID of sending node.
}
xme_prim_neighborhoodDiscovery_packetFormatNodeAnnouncements_t;
#pragma pack(pop)

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Create the neighborhodd discovery component.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be activated.
 */
xme_status_t
xme_prim_neighborhoodDiscovery_create(xme_prim_neighborhoodDiscovery_configStruct_t* config);

/**
 * \brief  Activate the neighborhodd discovery component.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be activated.
 */
xme_status_t
xme_prim_neighborhoodDiscovery_activate(xme_prim_neighborhoodDiscovery_configStruct_t* config);

/**
 * \brief  Deactivate the neighborhodd discovery component.
 *
 * \param  config the configuration parameters.
 */
void
xme_prim_neighborhoodDiscovery_deactivate(xme_prim_neighborhoodDiscovery_configStruct_t* config);

/**
 * \brief  Destroys the neighborhodd discovery component.
 *
 * \param  config the configuration parameters.
 */
void
xme_prim_neighborhoodDiscovery_destroy(xme_prim_neighborhoodDiscovery_configStruct_t* config);

/**
 * \brief  Establish the interface manager callback for the neighborhood discovery component.
 *
 * \param  status the status of the interface.
 * \param  interfaceHandle the interface handle.
 * \param  channel the channel.
 * \param  dataHandle the data handle.
 * \param  userData the user data.
 */
void
xme_prim_neighborhoodDiscovery_interfaceManager_callback
(
	xme_com_interface_state_t status,
	xme_com_interface_interfaceId_t interfaceHandle,
	xme_core_dataChannel_t channel,
	xme_hal_sharedPtr_t dataHandle,
	void* userData
);

XME_EXTERN_C_END

#endif // #ifndef XME_PRIM_NEIGBORHOODDISCOVERY_H
