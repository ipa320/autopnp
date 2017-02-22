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
 * $Id: brokerInternalTypes.h 7668 2014-03-04 10:48:37Z gulati $
 */

/**
 * \file
 *         Broker internal types.
 */

/**
 * \addtogroup core_broker
 * @{
 */

#ifndef XME_CORE_BROKER_BROKERINTERNALTYPES_H
#define XME_CORE_BROKER_BROKERINTERNALTYPES_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/broker/include/broker.h"

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

#include "xme/hal/include/linkedList.h"
#include "xme/hal/include/table.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \struct xme_core_broker_transferDataPacketItem_t
 * \brief the structure for relating output data packets and input data packets.
 */
typedef struct
{
    xme_core_dataManager_dataPacketId_t srcDataPacketId; ///< source data packet.
    xme_core_dataManager_dataPacketId_t destDataPacketId; ///< destination data packet.
    uint16_t referenceCount; ///< total reference count for this source and destination DataPackeIds
    bool srcLocked; ///< the source data packet is locked during a complete write operation. 
} 
xme_core_broker_transferDataPacketItem_t;

/**
 * \enum xme_core_broker_dataPacketReadiness_t
 * \brief enumeration type definition for data packet readiness. 
 */
typedef enum
{
    XME_CORE_BROKER_DATAPACKET_READINESS_READY = 0, ///< the data packet is ready for consumption.
    XME_CORE_BROKER_DATAPACKET_READINESS_UNAVAILABLE ///< the data packet is not available for consumption.
}
xme_core_broker_dataPacketReadiness_t;

/**
 * \struct xme_core_broker_dataSubscriberItem_t
 *
 * \brief the entry relating a data packets to associated functions.
 */
typedef struct
{
    xme_core_dataManager_dataPacketId_t dataPacketId; ///< the data packet identifier.
    xme_core_broker_dataPacketReadiness_t readiness; ///< the readiness of the data packet.
    xme_hal_singlyLinkedList_t (XME_CORE_BROKER_MAX_SUBSCRIBER_FUNCTIONS) subscribers; ///< the list of subscriber functions.
}
xme_core_broker_dataSubscriberItem_t;

/**
 * \enum xme_core_broker_functionDataPacketUsage_t
 * \brief Enumeration type definition for the determining data packet usage by the function.
 */
typedef enum
{
    XME_CORE_BROKER_DATAPACKET_USAGE_REQUIRED = 0, ///< the data packet usage in function is mandatory.
    XME_CORE_BROKER_DATAPACKET_USAGE_OPTIONAL ///< the data packet usage in function is optional.
}
xme_core_broker_functionDataPacketUsage_t;

/**
 * \struct xme_core_broker_functionDataPacket_t
 *
 * \brief information about a parameter (data packet) associated to a function.
 */
typedef struct
{
    xme_core_dataManager_dataPacketId_t dataPacketId; ///< the data packet identifier
    xme_core_broker_functionDataPacketUsage_t usage; ///< the data packet usage in the function
}
xme_core_broker_functionDataPacket_t;

/**
 * \struct xme_core_broker_functionDescriptionItem_t
 *
 * \brief an item for relating a function to the parameters.
 */
typedef struct
{
    xme_core_component_t componentId; ///< the componenet identifier.
    xme_core_component_functionId_t functionId; ///< the function identifier
    xme_core_component_functionVariantId_t functionVariantId; ///< the function variant identifier
    xme_hal_singlyLinkedList_t (XME_CORE_BROKER_MAX_FUNCTION_PARAMETERS) parameters; ///< parameters associated to the function.
}
xme_core_broker_functionDescriptionItem_t;

/**
 * \brief xme_core_broker_functionDescriptionTable_t
 *
 * \details Table type associating functions and parameters.
 */
typedef XME_HAL_TABLE
(
    xme_core_broker_functionDescriptionItem_t,
    xme_core_broker_functionDescriptionTable_t,
    XME_CORE_BROKER_MAX_FUNCTION_DATA_ITEMS
);

/**
 * \brief xme_core_broker_dataSubscriberTable_t
 *
 * \details Table type associating data packets and associated subscriber
 *        functions.
 */
typedef XME_HAL_TABLE
(
    xme_core_broker_dataSubscriberItem_t,
    xme_core_broker_dataSubscriberTable_t,
    XME_CORE_BROKER_MAX_DATAPACKET_ITEMS
);

/**
 * \brief transfer relationships linked list. 
 */
typedef xme_hal_singlyLinkedList_t(XME_CORE_BROKER_TRANSFERTABLE_SIZE) transferDataPacketTable_t;

#endif // #ifndef XME_CORE_BROKER_BROKERINTERNALTYPES_H

/**
 * @}
 */

