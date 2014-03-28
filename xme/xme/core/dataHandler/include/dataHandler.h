/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: dataHandler.h 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Data Handler.
 */

#ifndef XME_CORE_DATAHANDLER_DATAHANDLER_H
#define XME_CORE_DATAHANDLER_DATAHANDLER_H

/**
 * \defgroup core_dataHandler Data Handler group
 * @{
 *
 * \brief The objective of the data handler is to allow read and write operations
 *        in conjunction with the Broker.
 *
 * \details The Data Handler itargeted to perform read and write operations over data
 *          and attributes. The initial configuration of the data handler is performed
 *          using a configuration, that incrementally builds the database structure
 *          for enabling data centric communication. This involves not only the read and
 *          write operations, but also the needed transfers in the set of packages 
 *          defined in the Broker.
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

#include <stdbool.h>

//******************************************************************************//
//***   Variables                                                            ***//
//******************************************************************************//

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief  Writes data to the port data of provided data packet. 
 * \details Stores in the port data the data provided in the buffer.
 *          The buffer will contain the data to be stores. 
 * \note The bufferSize is not allowed to be bigger than the port data size
 *       specified during port data creation or either during data packet 
 *       creation. 
 * \note If the queue size specified for data packet is more than one, the
 *       data packet will be written in the next position of the queue. 
 * \note In case there were previous data in that data packet, the data is erased
 *       before writing to that port. 
 *
 * \param[in] dataPacketID The identifier of the data packet.
 * \param[in] buffer User provided data buffer, from which the data is copied.
 * \param[in] bufferSizeInBytes Specifies the size of the provided storage.
 *                   It indicates how much data shall be copied to the database or queue. 
 *
 * \retval XME_STATUS_SUCCESS if the buffer is successfully copied into the port data. 
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are null-valued. 
 * \retval XME_STATUS_PERMISSION_DENIED If the user has not write permission to that data port. 
 */
xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    const void* const buffer,
    size_t bufferSizeInBytes
    // uint32_t* const writtenBytes // Check if this should be included for symmetry. 
);

/**
 * \brief  Writes an attribute value to the attribute data of provided data packet. 
 * \details Stores in the attribute data the data provided in the buffer with
 *          the corresponding key.
 * \note The bufferSize is not allowed to be bigger than the port data size
 *       specified during attribute creation. 
 * \note In case there were previous attribute for the same key, the previous attribute
 *       is emptied before writting the new attribute. 
 *
 * \param[in] dataPacketID The identifier of the data packet.
 * \param[in] key The key of the attribute to write. 
 * \param[in] buffer User provided data buffer, from which the data is copied.
 * \param[in] bufferSizeInBytes Specifies the size of the provided storage.
 *                   It indicates how much data shall be copied to the database or queue.
 *
 * \retval XME_STATUS_SUCCESS if the buffer is successfully copied into the port data. 
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are null-valued. 
 * \retval XME_STATUS_PERMISSION_DENIED If the user has not write permission to that data port. 
 */
xme_status_t
xme_core_dataHandler_writeAttribute
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t key,
    const void* const buffer,
    size_t bufferSizeInBytes
    //uint32_t* const writtenBytes // Check if this should be included for symmetry.
);

/**
 * \brief Reads data from the port data inside a data packet into the given buffer.
 * \details Gets the port data from the data packet and copies it in the provided user
 *          buffer. 
 *
 * \param[in] dataPacketID The source data packet from which the data is read.
 * \param[in] buffer User provided buffer.
 * \param[in] bufferSizeInBytes specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param[out] readBytes indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *                  then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *                  is smaller then at least the given bufferSize will be returned.
 *
 * \retval XME_CORE_STATUS_SUCCESS if a correct read was done.
 * \retval XME_STATUS_NO_SUCH_VALUE when the port currently does not hold new data.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if a parameter value is incorrect.
 * \retval XME_CORE_STATUS_PERMISSION_DENIED if given port is a Publication-Port.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR in case of any other error.
 */
xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t * const readBytes
);

/**
 * \brief Reads data from the port data inside a data packet into the given buffer.
 *
 * \param dataPacketID The source data packet from which the data is read.
 * \param[in] key The key of the attribute to write. 
 * \param buffer User provided buffer.
 * \param bufferSizeInBytes specifies the size of the provided storage.
 *        It can be bigger, equal, or smaller than the data element stored in the
 *        database or queue.
 * \param bytesRead indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *        then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *        is smaller then at least the given bufferSize will be returned.
 *
 * \retval XME_CORE_STATUS_SUCCESS if a correct read was done.
 * \retval XME_STATUS_NO_SUCH_VALUE when the port currently does not hold new data.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if a parameter value is incorrect.
 * \retval XME_CORE_STATUS_PERMISSION_DENIED if given port is a Publication-Port.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR in case of any other error.
 */
xme_status_t
xme_core_dataHandler_readAttribute
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t key,
    void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t* const bytesRead
);

/**
 * \brief Initiates a write operation and locks a data packet to ensure atomicity of write operation. 
 * \details Locks the data packet for read and prepares the data packet to write data. 
 * 
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the data packet was successfully locked for write operation. 
 * \retval XME_STATUS_INVALID_PARAMETER If provided data packet is an invalid data packet. 
 */
xme_status_t 
xme_core_dataHandler_startWriteOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Completes the write operation and unlocks a data packet to be read concurrently. 
 * \details Sets up the data packet to allow further operations (read or write). 
 * 
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the data packet was successfully unlocked for allowing 
 *         further operations (read/write). 
 * \retval XME_STATUS_INVALID_PARAMETER If provided data packet is an invalid data packet. 
 */
xme_status_t 
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Initiates a read operation and locks a data packet to ensure atomicity of read operation. 
 * \details Locks the data packet for write and prepares the data packet to read data. 
 * 
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the data packet was successfully locked for read operation. 
 * \retval XME_STATUS_INVALID_PARAMETER If provided data packet is an invalid data packet. 
 */
xme_status_t 
xme_core_dataHandler_startReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Completes the read operation and prepares the data packet for further operations. 
 *
 * \param[in] dataPacketID The data packet identifier.
 *
 * \retval XME_STATUS_SUCCESS If the complete read operation was success. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If cannot perform the complete write operation. 
 */
xme_status_t 
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Transfers data (topic + attributes) from source data packet to sink data packet.
 *
 * \param[in] sourceDataPacketID The source data packet identifier to copy from.
 * \param[in] destDataPacketID The destinkation data packet identifier to copy to.
 *
 * \retval XME_STATUS_SUCCESS If the transfer was success. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If cannot complete the transfer. 
 */
xme_status_t 
xme_core_dataHandler_transferData
(
    xme_core_dataManager_dataPacketId_t sourceDataPacketID,
    xme_core_dataManager_dataPacketId_t destDataPacketID
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_DATAHANDLER_DATAHANDLER_H
