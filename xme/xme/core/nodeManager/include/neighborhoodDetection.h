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
 * $Id: neighborhoodDetection.h 5626 2013-10-25 12:37:07Z ruiz $
 */

/**
 * \file
 *         Neighborhood Detection.
 */

#ifndef XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_H
#define XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include "xme/core/directory/include/networkTopologyCalculator.h"
#include "xme/core/directory/include/nodeRegistryController.h"
#include "xme/hal/include/linkedList.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \brief The interval in which the announcements of a node are sent.
 */
#define XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_ANNOUNCEMENT_INTERVAL ((xme_hal_time_timeInterval_t)1000000000ULL)
/**
 * \brief Number for the countdown before an announcement is removed from the list.
 */
#define XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL 6

/**
 * \struct xme_core_nodeManager_neighborhoodDetection_neighborInformation_t
 *
 * \brief  Structure for storing information about one node of the neighborhood.
 */
typedef struct
{
    xme_com_interface_address_t senderInterfaceID; ///< Interface of sending node
    xme_core_node_nodeId_t senderNodeID; ///< ID of sending node
    xme_com_interface_address_t receivingInterfaceID; ///< ID of interface that received the announcement
    int8_t lastSeenBeforeTicks; ///< Entry will be removed if lastSeenBeforeTicks becomes zero.
}
xme_core_nodeManager_neighborhoodDetection_neighborInformation_t;

/**
 * \struct xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t
 *
 * \brief  Packet format for node announcements.
 */
#pragma pack(push, 1)
typedef struct
{
    xme_com_interface_address_t sendingInterfaceID; ///< Interface ID of sending node.
    xme_core_node_nodeId_t sendingNodeID; ///< Node ID of sending node.
}
xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t;
#pragma pack(pop)

/**
 * \typedef xme_core_nodeManager_neighborhoodDetection_neighborsList_t
 *
 * \brief Neighbors list.
 */
typedef xme_hal_singlyLinkedList_t(10) xme_core_nodeManager_neighborhoodDetection_neighborsList_t;

/**
 * \typedef xme_core_nodeManager_neighborhoodDetection_socketList_t
 *
 * \brief Socket list.
 */
typedef xme_hal_singlyLinkedList_t(10) xme_core_nodeManager_neighborhoodDetection_socketList_t;

/**
 * \brief Number of possible neighbors.
 */
#define NEIGHBOR_COUNT 20

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initialize the neighborhood detection.
 *
 * \param[in] initStruct Initialization data. In this case the node ID.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if initialization was successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR if it is not possible to perform
 *            full initialization.
 */
xme_status_t
xme_core_nodeManager_neighborhoodDetection_init
(
    void* initStruct
);

/**
 * \brief  Stop the neighborhood detection.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if all operations were successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR if it is not possible to perform
 *            full finalization.
 */
void
xme_core_nodeManager_neighborhoodDetection_fini(void);

/**
 * \brief  Send announcement.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if all operations were successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR if it is not possible to perform
 *            full finalization.
 */
xme_status_t
xme_core_nodeManager_neighborhoodDetection_sendAnnouncement(void);

/**
 * \brief  Evaluate received announcements.
 *
 * \param[in] receivedPacketAnnouncement Address of the received announcement packet.
 * \param[in] receivedInterfaceId Address of a variable containing the address of the
 *            interface where the packet was received.
 *
 * \retval XME_CORE_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the parameters was NULL
 *         or contains invalid or unsupported values.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements
(
    const xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t* receivedPacketAnnouncement,
    const xme_com_interface_address_t* receivedInterfaceId
);

/**
 * \brief  Send information to the network topology calculator.
 *
 * \param[out] neighborhoodInfo Address of a variable in which the gathered data
 *             is to be stored.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if all operations were successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR if it is not possible to perform
 *            full finalization.
 */
xme_status_t
xme_core_nodeManager_neighborhoodDetection_forwardNeighborhoodInformation(xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t* neighborhoodInfo);

/**
 * \brief  Update the neighborhood information.
 *
 */
void
xme_core_nodeManager_neighborhoodDetection_updateNeighborhoodInformation(void);

/**
 * \brief  Get the neighborhood list.
 *
 * \return Returns the current neighborhood list.
 */
xme_core_nodeManager_neighborhoodDetection_neighborsList_t
xme_core_nodeManager_neighborhoodDetection_getNeighborList(void);

/*void
receiveFromNetwork(void);*/

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_H
