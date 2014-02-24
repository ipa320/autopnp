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
 * $Id: dataHandler.h 5121 2013-09-19 13:51:44Z camek $
 */

/**
 * \file
 *         Data Handler.
 */

#ifndef XME_CORE_DATAHANDLER_DATAHANDLER_H
#define XME_CORE_DATAHANDLER_DATAHANDLER_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

#include <stdbool.h>

//******************************************************************************//
//***   Static variables                                                     ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief Defines a void attribute descriptor list.
 */
extern xme_core_attribute_descriptor_list_t XME_CORE_NO_ATTRIBUTE;

XME_EXTERN_C_END

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

// TODO: CHECK DOCU IF PROBLEM WITH MULTIPLE AND DUPLICATED GENERATED PORTS ARE EXPLAINED
/**
 * \brief  Creates a port for a component. Exactly one port of this type will be present.
 *         With this command memory is allocated in the database or for queues.
 *         The memory is accessible only by the component and port to which it is associated.
 *         The memory contains all given informations, which allows the database to organize this.
 *         Beside, not only the topic, but also the corresponding attributes are stored.
 *
 * \param componentID contains the global unique identifier of a component, which tries to allocate
 *                    memory in the database or queues.
 * \param type contains the communication type of the port, which can be as follows:
 *             - XME_CORE_COMPONENT_PORTTYPE_INVALID
 *             - XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION
 *             - XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION
 *             - XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER
 *             - XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER
 *             - XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER
 *             - XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER
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
xme_core_dataHandler_createPort
(
     xme_core_component_t componentID,
     xme_core_component_portType_t type,
     xme_core_topic_t topic,
     uint32_t bufferSize,
     xme_core_attribute_descriptor_list_t metadata,
     uint32_t queueSize,
     bool overwrite,
     bool persistent,
     uint32_t historyDepth,
     xme_core_dataManager_dataPacketId_t * const dataPacketId
);

// only used by the broker

/**
 * \brief  Transfers one topic from one port to another port.
 *         The source port must be a publisher and the sink port will then be a subscriber.
 * \param portSource provides the handle to identify the source memory allocated in the database.
 * \param portSink specifies the handle of the destination memory in the database, where the data
 *                 given by portSource will be copied to.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the transfer was done correct.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if one of the specified ports has a wrong type
 *          - XME_CORE_STATUS_INVALID_CONFIGURATION if a component of this
 *            type has already been initialized. Exactly one component of this
 *            type must be present on every node.
 */
extern xme_status_t
xme_core_dataHandler_transferData
(
    xme_core_dataManager_dataPacketId_t portSource,
    xme_core_dataManager_dataPacketId_t portSink
);

/**
 * \brief  Initializes the database.
 * \param amountOfMemory specifies the basis value to calculate the needed amount of memory, which
 *        will be managed by the datahandler. The memory is calculated with 2^param.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the database has been successfully
 *            initialized.
 *          - XME_CORE_STATUS_INTERNAL_ERROR if the database got an error during initialization.
 */
extern xme_status_t 
xme_core_dataHandler_init
(
    size_t amountOfMemory
);

/**
 * \brief  Shuts down the database.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the shutdown  has been successfully done.
 *          - XME_CORE_STATUS_INTERNAL_ERROR if an error occurred.
 */
extern xme_status_t xme_core_dataHandler_fini(void);

/**
 * \brief  Reads data from a port into the given buffer.
 * \param port specifies the handle for an allocated memory in the database or queue.
 *             The memory contains the data which shall be read.
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param bytesRead indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *                  then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *                  is smaller then at least the given bufferSize will be returned.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if a correct read was done.
 *          - XME_CORE_STATUS_INVALID_PARAMETER if a parameter value is incorrect
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Publication-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t port,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead
);

/**
 * \brief  Reads an attribute given by the attribute key to the given buffer.
 * \param port is the identifier returned by the database or queue.
 * \param attributeKey identifies which attribute of a given topic shall be read.
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param bytesRead indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *                  then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *                  is smaller then at least the given bufferSize will be returned.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if a correct read was done.
 *          - XME_CORE_STATUS_INVALID_PARAMETER if a parameter is invalid, e.g., the key
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Publication-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandler_readAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead
);

/**
 * \brief  Writes data to a port. Data is provided by a user's buffer.
 * \param port is the identifier returned by the database or queue.
 * \param buffer is a user provided storage, from which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It indicates how much data shall be copied to the database or queue.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if a correct write was done.
 *          - XME_CORE_STATUS_INVALID_PARAMETER if a parameter is invalid
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Subscription-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    uint32_t bufferSize
);

/**
 * \brief  Writes an attribute given by the attribute key to the port. Data is provided by a user's buffer.
 * \param port is the identifier returned by the database or queue.
 * \param attributeKey identifies to which attribute of a given topic data should be written.
 * \param buffer is a user provided storage, from which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It indicates how much data shall be copied to the database or queue.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if a correct write was done.
 *          - XME_CORE_STATUS_INVALID_PARAMETER if a parameter is invalid, e.g., attribute key
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Subscription-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandler_writeAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void const * const buffer,
    uint32_t bufferSize
);

/**
 *\brief This function returns the current cycle counter that is stored in the data handler by the execution manager.
 */
extern uint32_t xme_core_dataHandler_readCycle(void);

/**
 *\brief This function sets the current cycle counter that is stored in the data handler by the execution manager.
 */
extern void xme_core_dataHandler_writeCycle(uint32_t counter);


/**
 * \brief  Prepares the input port.
 * \param port is the identifier returned by the database or queue.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if a correct read was done.
 *          - XME_CORE_STATUS_INTERNAL_ERROR
 *          - XME_CORE_STATUS_INVALID_PARAMETER
 */
extern xme_status_t xme_core_dataHandler_completeReadOperation(xme_core_dataManager_dataPacketId_t port);

/**
 * \brief This function signals, that all write operation to this port have been finished.
 *        Then the written data elements will be locked to fix a steady state, which will be used for
 *        future work by depending components.
 * \param port is the identifier returned by the database or queue.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if a correct read was done.
 *          - XME_CORE_STATUS_INTERNAL_ERROR
 *          - XME_CORE_STATUS_INVALID_PARAMETER
 */
extern xme_status_t xme_core_dataHandler_completeWriteOperation(xme_core_dataManager_dataPacketId_t port);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DATAHANDLER_DATAHANDLER_H
