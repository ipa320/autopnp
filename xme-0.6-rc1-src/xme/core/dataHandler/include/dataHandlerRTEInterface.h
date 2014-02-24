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
 * $Id: dataHandlerRTEInterface.h 3925 2013-06-28 16:40:36Z camek $
 */

/**
 * \file
 *         Data Handler.
 */

#ifndef XME_CORE_DATAHANDLER_RTE_INTERFACE_H
#define XME_CORE_DATAHANDLER_RTE_INTERFACE_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataManagerTypes.h"

#include <stdbool.h>

//******************************************************************************//
//***   Static variables                                                     ***//
//******************************************************************************//

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

// FIXME: How shall we add infos about accessing RTE components to this specific topic.
/**
 * \brief  Creates a port for Runtime-Environment internal components.
 *         Exactly one port of this type will be present.
 *         With this command memory is allocated in the database or for queues.
 *         The memory is accessible only by the component and port to which it is associated.
 *         The memory contains all given informations, which allows the database to organize this.
 *         Beside, not only the topic, but also the corresponding attributes are stored.
 *
 * \param topic is the global unique identifier of the data element matching a topic.
 * \param bufferSize specifies how big the allocated memory should be.
 * \param metadata contains a list of all topic related attributes.
 * \param queueSize gives the dataHandler the information how many items shall be stored, when
 *                  queues are used. This parameter is unused when topics are stored in a database.
 * \param overwrite indicates if port data should be overwritten when new values arrive.
 * \param persistent indicates if data of a port is stored persistent. This means that the data will
 *                   not change during runtime.
 * \param historyDepth specifies how many data items are stored in the port. This allows to store old
 *                     data item whether from former time slots if a time triggered system will be used or
 *                     from previous occurred interrupts if an event triggered system will be used.
 * \param dataPacketId is the identifier returned by the database or queue. This allows a component to
 *                   read or write data from the allocated memory by using the dataHandler API.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the port has been successfully
 *            initialized.
 *          - XME_CORE_STATUS_INVALID_CONFIGURATION if a component of this
 *            type has already been initialized. Exactly one component of this
 *            type must be present on every node.
 *          - XME_CORE_STATUS_INVALID_PARAMETER if a parameter has invalid values
 */
extern xme_status_t
xme_core_dataHandler_createRTEData
(
 xme_core_topic_t topic,
 uint32_t bufferSize,
 xme_core_attribute_descriptor_list_t metadata,
 uint32_t queueSize,
 bool overwrite,
 bool persistent,
 uint32_t historyDepth,
 xme_core_dataManager_dataPacketId_t * const dataPacketId
);

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_RTE_INTERFACE_H */
