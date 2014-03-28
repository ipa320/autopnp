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
 * $Id: dataHandlerTestProbe.h 7773 2014-03-11 17:22:36Z ruiz $
 */

/**
 * \file
 *         Data Handler Test Probe.
 */

#ifndef XME_CORE_DATAHANDLER_DATAHANDLERTESTPROBE_H
#define XME_CORE_DATAHANDLER_DATAHANDLERTESTPROBE_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataHandler/include/dataHandler.h"

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

#include <stdbool.h>

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief  Writes data to the port data of provided data packet. 
 * \details Stores in the port data the data provided in the buffer.
 *          The buffer will contain the data to be stores. 
 * \note If the queue size specified for data packet is more than one, the
 *       data packet will be written in the next position of the queue. 
 * \note In case there were previous data in that data packet, the data is erased
 *       before writing to that port. 
 * \note In case the data packet size is smaller than the buffer size, the number of
 *       written bytes will be the data packet size. 
 *
 * \param[in] databaseIndex The index of the database to use for the write operation. 
 *            For easiness purposes:
 *            - 0: default database. It will use the active database. 
 *            - 1: master database.
 *            - 2: shadow database. 
 * \param[in] dataPacketID The identifier of the data packet.
 * \param[in] offsetInBytes The relative offset inside the data packet in which to
 *            write. 
 * \param[in] buffer User provided data buffer, from which the data is copied.
 * \param[in] bufferSizeInBytes Specifies the size in bytes of the provided buffer.
 *                   It indicates how much data shall be copied to the database or queue.
 * \param[out] writtenBytes The number of written bytes in the data packet. 
 *
 * \retval XME_STATUS_SUCCESS if the buffer is successfully copied into the port data. 
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are null-valued. 
 * \retval XME_STATUS_PERMISSION_DENIED If the user has not write permission to that data port. 
 */
xme_status_t
xme_core_dataHandlerTestProbe_writeDataInDatabase
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    const void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t* const writtenBytes
);

/**
 * \brief  Writes an attribute value to the attribute data of provided data packet
 *         in the given database. 
 * \details Stores in the attribute data the data provided in the buffer with
 *          the corresponding key.
 * \note In case there were previous attribute for the same key, the previous attribute
 *       is emptied before writting the new attribute. 
 *
 * \param[in] databaseIndex The index of the database to use for the write operation. 
 *            For easiness purposes:
 *            - 0: default database. It will use the active database. 
 *            - 1: master database.
 *            - 2: shadow database.
 * \param[in] dataPacketID The identifier of the data packet.
 * \param[in] key The key of the attribute to write. 
 * \param[in] offsetInBytes The relative offset inside the data packet in which to
 *            write. 
 * \param[in] buffer User provided data buffer, from which the data is copied.
 * \param[in] bufferSize Specifies the size of the provided storage.
 *                   It indicates how much data shall be copied to the database or queue.
 * \param[out] writtenBytes The number of written bytes in the data packet. 
 *
 * \retval XME_STATUS_SUCCESS if the buffer is successfully copied into the port data. 
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are null-valued. 
 * \retval XME_STATUS_PERMISSION_DENIED If the user has not write permission to that data port. 
 */
xme_status_t
xme_core_dataHandlerTestProbe_writeAttributeInDatabase
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t key,
    uint32_t offsetInBytes,
    const void* const buffer,
    size_t bufferSize,
    uint32_t* const writtenBytes
);

/**
 * \brief Reads data from the data inside a data packet into the given buffer
 *        from the given database.
 * \details Gets the port data from the data packet and copies it in the provided user
 *          buffer. 
 * \note If the queue size specified for data packet is more than one, the read operation
 *       will read the last unread value. 
 * \note In case the data packet size is smaller than the buffer size, the number of
 *       read bytes will be the data packet size. 
 *
 * \param[in] databaseIndex The index of the database to use for the read operation. 
 *            For easiness purposes:
 *            - 0: default database. It will use the active database. 
 *            - 1: master database.
 *            - 2: shadow database. 
 * \param[in] dataPacketID The source data packet from which the data is read.
 * \param[in] offsetInBytes The relative offset inside the data packet in which to
 *            read. 
 * \param[in] buffer User provided buffer.
 * \param[in] bufferSizeInBytes Specifies the size of the provided storage.
 *            It can be bigger, equal, or smaller than the data element stored in the
 *            database or queue.
 * \param[out] readBytes The number of read bytes from the data packet. 
 *
 * \retval XME_CORE_STATUS_SUCCESS if a correct read was done.
 * \retval XME_STATUS_NO_SUCH_VALUE when the port currently does not hold new data.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if a parameter value is incorrect.
 * \retval XME_CORE_STATUS_PERMISSION_DENIED if the data packet is already locked
           (e.g. the data is not completely written).
 * \retval XME_CORE_STATUS_INTERNAL_ERROR in case of any other error.
 */
xme_status_t
xme_core_dataHandlerTestProbe_readDataInDatabase
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    void* const buffer,
    size_t bufferSizeInBytes, 
    uint32_t* const readBytes
);

/**
 * \brief Reads data from the attribute data inside a data packet into the given buffer.
 *
 * \param[in] databaseIndex The index of the database to use for the write operation. 
 *            For easiness purposes:
 *            - 0: default database. It will use the active database. 
 *            - 1: master database.
 *            - 2: shadow database.  
 * \param[in] dataPacketID The source data packet from which the data is read.
 * \param[in] key The key of the attribute to write. 
 * \param[in] offsetInBytes The relative offset inside the data packet in which to
 *            read. 
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
xme_core_dataHandlerTestProbe_readAttributeInDatabase
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t key,
    uint32_t offsetInBytes,
    void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t* const readBytes
);

/**
 * \brief Establishes the active database for the given operations. 
 * \details Sets the active database index for operations with the provided data packet. 
 *          The database index indicates when using the (1) master database or either
 *          (2) shadow database. This will work only for read/write calls that do not
 *          provide a database index as a parameters. 
 * \note The databases indices should be configured properly beforehand. In case that we
 *       we are providing an invalid index, an error message will be displayed and 
 *       an error status will be returned. 
 *
 * \param[in] databaseIndex The database index referring to the database to activate. 
 * \param[in] dataPacketID The data packet identifier to which apply the policy. 
 * 
 * \retval XME_STATUS_SUCCESS If the corresponding database was successfully activated. 
 * \retval XME_STATUS_INVALID_PARAMETER If the provided input parameters are either invalid
 *         or out of range. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If provided database index does not corresponds
 *         to an existing database. 
 */
xme_status_t
xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Starts a manipulation. 
 *
 * \param[in] dataPacketID The data packet identifier to which apply the policy. 
 * \param[in] offsetInBytes The relative offset inside the data packet in which to
 *            read. 
 * 
 * \retval XME_STATUS_SUCCESS If start of manipulation is success. 
 */
xme_status_t
xme_core_dataHandlerTestProbe_startManipulation
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes
);

/**
 * \brief Ends a manipulation. 
 *
 * \param[in] dataPacketID The data packet identifier to which apply the policy. 
 * \param[in] offsetInBytes The relative offset inside the data packet in which to
 *            read. 
 * 
 * \retval XME_STATUS_SUCCESS If start of manipulation is success. 
 */
xme_status_t
xme_core_dataHandlerTestProbe_endManipulation
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes
);

/**
 * \brief Starts a manipulation. 
 *
 * \param[in] dataPacketID The data packet identifier to which apply the policy. 
 * \param[in] offsetInBytes The relative offset inside the data packet in which to
 *            read. 
 * \param[in] attributeKey The attribute key to which apply the manipulation. 
 *            If 0-valued, the manipulation applies only to topic data. 
 * 
 * \retval XME_STATUS_SUCCESS If start of manipulation is success. 
 */
xme_status_t
xme_core_dataHandlerTestProbe_startAttributeManipulation
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    uint32_t attributeKey
);

/**
 * \brief Ends a manipulation. 
 *
 * \param[in] dataPacketID The data packet identifier to which apply the policy. 
 * \param[in] offsetInBytes The relative offset inside the data packet in which to
 *            read. 
 * \param[in] attributeKey The attribute key to which apply the manipulation. 
 *            If 0-valued, the manipulation applies only to topic data. 
 * 
 * \retval XME_STATUS_SUCCESS If start of manipulation is success. 
 */
xme_status_t
xme_core_dataHandlerTestProbe_endAttributeManipulation
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    uint32_t attributeKey
);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DATAHANDLER_DATAHANDLERTESTPROBE_H
