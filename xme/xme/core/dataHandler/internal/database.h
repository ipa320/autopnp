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
 * $Id: database.h 7757 2014-03-11 10:08:27Z ruiz $
 */

/**
 * \file
 *         Database Abstraction.
 *
 * \brief The database contains all the needed information to interact with HAL
 *        components to get/set information in the database.
 */

#ifndef XME_CORE_DATAHANDLER_DATABASE_H
#define XME_CORE_DATAHANDLER_DATABASE_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataManagerTypes.h"
#include "xme/defines.h"

#include <stdint.h>
#include <stdbool.h>

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes a database. 
 * \details Initializes a database and establishes the default values to
 *          use. The initialization should include database allocation. 
 *          The provided database object is used to complete this configuration.
 *
 * \note The database should be a valid database struct.
 * \note The allocation is guided by an internal XME algorithm. 
 *
 * \retval XME_STATUS_SUCCESS if the database is successfully initialized. 
 * \retval XME_STATUS_INVALID_PARAMETER If the provided reference is not
 *         a valid database structure. 
 */
xme_status_t
xme_core_dataHandler_database_init(void);

/**
 * \brief Frees all resources occupied by the current database structure. 
 */
void
xme_core_dataHandler_database_fini(void);

/**
 * \brief Writes the data associated to a data packet from database. 
 * \details The data to be written is placed in the input buffer. 
 *
 * \note If the size of the buffer is smaller than the size of the data packet,
 *       the written data will correspond to the bytes specified by the bufferSize. 
 * \note If the size of the buffer is larger than the size of the data packet, 
 *       the written data will correspond to the bytes specified by the data packet. 
 * \note When the database index is set to zero, the associated database to use will
 *       be the active one. 
 * \note The offset inside the data packet will be used to calculate where to write 
 *       the data. This offset is taken into account for the calculation of data size
 *       to write. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 * \param[in] offsetInBytes The offset to use inside the data in data packet. 
 * \param[in] sourceBuffer The input buffer from which the data is copied. 
 * \param[in] sourceBufferSizeInBytes The size of the buffer expressed in bytes. 
 * \param[out] writtenBytes Specifies the effective written bytes in the database. 
 *
 * \retval XME_STATUS_SUCCESS If the write operation is completed after writting
 *         in the database. 
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are 
 *         invalid. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the current database configuration
 *         does not allow to complete the write operation.
 */
xme_status_t
xme_core_dataHandler_database_writeData
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    const void* const sourceBuffer,
    size_t sourceBufferSizeInBytes,
    uint32_t* const writtenBytes
);

/**
 * \brief Writes the attribute associated to a data packet from database. 
 * \details The attribute data to be written is placed in the input buffer. 
 *
 * \note If the size of the buffer is smaller than the size of the data packet,
 *       the written attribute data will correspond to the bytes specified by the bufferSize. 
 * \note If the size of the buffer is larger than the size of the data packet, 
 *       the written attribute data will correspond to the bytes specified by the data packet. 
 * \note When the database index is set to zero, the associated database to use will
 *       be the active one. 
 * \note The offset inside the data packet will be used to calculate where to write 
 *       the attribute data. This offset is taken into account for the calculation of data size
 *       to write. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 * \param[in] attributeKey The attribute key. 
 * \param[in] offsetInBytes The offset to use inside the data in data packet. 
 * \param[in] sourceBuffer The input buffer from which the data is copied. 
 * \param[in] sourceBufferSizeInBytes The size of the buffer expressed in bytes. 
 * \param[out] writtenBytes Specifies the effective written bytes in the database. 
 *
 * \retval XME_STATUS_SUCCESS If the write operation is complete after reading
 *         from database. 
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are 
 *         invalid. 
 * \retval XME_STATUS_NOT_FOUND If there is no matching data packet associated
 *         to the provided Data Packet identifier or the attribute key. 
 */
xme_status_t
xme_core_dataHandler_database_writeAttribute
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t attributeKey,
    uint32_t offsetInBytes,
    const void* const sourceBuffer,
    size_t sourceBufferSizeInBytes,
    uint32_t* const writtenBytes
);

/**
 * \brief Reads the data associated to a data packet from the database. 
 * \details The data obtained from the database is copied to an output buffer. 
 *
 * \note If the size of the buffer is smaller than the size of the data packet,
 *       the read data will correspond to the bytes specified by the bufferSize. 
 *       Additionally, the bytesRead variable will contain the effective read bytes
 *       (the size specified by the buffer size). 
 * \note If the size of the buffer is larger than the size of the data packet, 
 *       the read data will correspond to the bytes specified by the data packet. 
 *       Additionally, the bytesRead variable will contain the effective read bytes
 *       (the size specified by the buffer size). 
 * \note When the database index is set to zero, the associated database to use will
 *       be the active one. 
 * \note The offset inside the data packet will be used to calculate where to read 
 *       the data. This offset is taken into account for the calculation of data size
 *       to read. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 * \param[in] offsetInBytes The offset to use inside the data in data packet. 
 * \param[in,out] targetBuffer The output buffer in which to place the data. 
 * \param[in] targetBufferSizeInBytes The size of the buffer expressed in bytes. 
 * \param[out] readBytes Specifies the effective read bytes in the database. 
 *
 * \retval XME_STATUS_SUCCESS If the read operation is completed successfully. 
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are 
 *         invalid. 
 * \retval XME_STATUS_NOT_FOUND If there is no matching data packet associated
 *         to the provided Data Packet identifier. 
 */
xme_status_t
xme_core_dataHandler_database_readData
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    void* const targetBuffer,
    size_t targetBufferSizeInBytes,
    uint32_t* const readBytes
);

/**
 * \brief Reads the attribute associated to a data packet from database. 
 * \details The attribute data obtained from the database is copied to an output buffer. 
 *
 * \note If the size of the buffer is smaller than the size of the attribute defined
 *       in the data packet, the read attribute data will correspond to the bytes 
 *       specified by the bufferSize. 
 *       Additionally, the bytesRead variable will contain the effective read bytes
 *       (the size specified by the buffer size). 
 * \note If the size of the buffer is larger than the size of the attribute defined
 *       in the data packet, the read attribute data will correspond to the bytes 
 *       specified by the data packet. 
 *       Additionally, the bytesRead variable will contain the effective read bytes
 *       (the size specified by the buffer size). 
 * \note When the database index is set to zero, the associated database to use will
 *       be the active one. 
 * \note The offset inside the data packet will be used to calculate from where to read 
 *       the data. This offset is taken into account for the calculation of data size
 *       to read. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 * \param[in] attributeKey The attribute key. 
 * \param[in] offsetInBytes The offset to use inside the data in data packet. 
 * \param[in,out] targetBuffer The output buffer in which to place the data. 
 * \param[in] targetBufferSizeInBytes The size of the buffer expressed in bytes. 
 * \param[out] readBytes Specifies the effective read bytes in the database. 
 *
 * \retval XME_STATUS_SUCCESS If the read operation is completed successfully. 
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are 
 *         invalid. 
 * \retval XME_STATUS_NOT_FOUND If there is no matching data packet associated
 *         to the provided Data Packet identifier or the attribute key. 
 */
xme_status_t
xme_core_dataHandler_database_readAttribute
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t attributeKey,
    uint32_t offsetInBytes,
    void* const targetBuffer,
    size_t targetBufferSizeInBytes,
    uint32_t* const readBytes
);

/**
 * \brief Transfers the data from the source data packet to the destination data packet. 
 * \details Makes a data copy between source data packet and the destination data packet. 
 *          This copies all the data in the data packet, including attributes. 
 *
 * \note In the case the target attributes keys do not match, it is only copies the 
 *       set of attributes in destination data packet which match with the keys in
 *       the source data packet. 
 * \note For the transfer operation the database index taken into account is the 
 *       default one associated to each data packet. 
 * 
 * \param[in] sourceDataPacketID The source data packet identifier. 
 * \param[in] sinkDataPacketID The sink data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the transfer operation was success. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the database configuration or
 *         the data packets do not match in size (mainly data associated to 
 *         data packets). 
 */
xme_status_t
xme_core_dataHandler_database_transfer
(
    xme_core_dataManager_dataPacketId_t sourceDataPacketID,
    xme_core_dataManager_dataPacketId_t sinkDataPacketID
);

/**
 * \brief Resets the data in the data packet. 
 * \details Sets to zero the data in the data packet and reduces the data availability
 *          in the data packet. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the data reset was success. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the database configuration 
 *         is not a valid configuration. 
 */
xme_status_t
xme_core_dataHandler_database_resetData
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Checks if a given data packet is already defined inside the current database. 
 *
 * \param[in] dataPacketID The data packet identifier to find.
 *
 * \retval true if the data packet is already present in this database. 
 * \retval false if the data packet is not present in current database. 
 */
bool
xme_core_dataHandler_database_isDataStoreDefined
(
    xme_core_dataManager_dataPacketId_t const dataPacketID
);

/**
 * \brief Checks if a given data packet is locked for write in the current database. 
 *
 * \param[in] dataPacketID The data packet identifier to find.
 *
 * \retval true if the data packet is locked for write in this database. 
 * \retval false if the data packet is not locked for write in current database. 
 */
bool
xme_core_dataHandler_database_isWriteLocked
(
    xme_core_dataManager_dataPacketId_t const dataPacketID
);

/**
 * \brief Checks if a given data packet is locked for read in the current database. 
 *
 * \param[in] dataPacketID The data packet identifier to find.
 *
 * \retval true if the data packet is locked for read in this database. 
 * \retval false if the data packet is not locked for read in current database. 
 */
bool
xme_core_dataHandler_database_isReadLocked
(
    xme_core_dataManager_dataPacketId_t const dataPacketID
);

/**
 * \brief Locks the write operation in a given data packet. 
 * \details For a given data packet locks that data packet for write operation. 
 * 
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the lock operation was success.
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are 
 *         invalid. 
 * \retval XME_STATUS_NOT_FOUND If there is no matching data packet associated
 *         to the provided Data Packet identifier. 
 */
xme_status_t
xme_core_dataHandler_database_lockWrite
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Unlocks the write operation in a given data packet. 
 * \details For a given data packet, if that data packet is locked,
 *          unlocks that data packet. 
 * 
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the unlock operation was success.
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are 
 *         invalid. 
 * \retval XME_STATUS_NOT_FOUND If there is no matching data packet associated
 *         to the provided Data Packet identifier. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the data packet is not locked. 
 */
xme_status_t
xme_core_dataHandler_database_unlockWrite
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Locks the read operation in a given data packet. 
 * \details For a given data packet locks that data packet for read operation. 
 * 
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the lock operation was success.
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are 
 *         invalid. 
 * \retval XME_STATUS_NOT_FOUND If there is no matching data packet associated
 *         to the provided Data Packet identifier. 
 */
xme_status_t
xme_core_dataHandler_database_lockRead
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Unlocks the read operation in a given data packet. 
 * \details For a given data packet, if that data packet is locked for read operation,
 *          unlocks that data packet. 
 * 
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the unlock operation was success.
 * \retval XME_STATUS_INVALID_PARAMETER If some of the provided parameters are 
 *         invalid. 
 * \retval XME_STATUS_NOT_FOUND If there is no matching data packet associated
 *         to the provided Data Packet identifier. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the data packet is not locked. 
 */
xme_status_t
xme_core_dataHandler_database_unlockRead
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Returns whether the data packet is persistent. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval true If the data packet is a persistent data packet. 
 * \retval false If the data packet is NOT a persistent data packet. 
 */
bool 
xme_core_dataHandler_database_isPersistent
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Returns whether the data packet a queue. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval true If the data packet is a queued data packet. 
 * \retval false If the data packet is NOT a queued data packet. 
 */
bool 
xme_core_dataHandler_database_isQueue
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Advances one position on the queue for write operation. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the write position advance in the queue was success. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the database configuration 
 *         is not a valid configuration. 
 */
xme_status_t
xme_core_dataHandler_database_advanceWriteQueue
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Advances one position on the queue for read operation. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 *
 * \retval XME_STATUS_SUCCESS If the read position advance in the queue was success. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the database configuration 
 *         is not a valid configuration. 
 */
xme_status_t
xme_core_dataHandler_database_advanceReadQueue
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Gets the current availability of data for the data packet. 
 *
 * \param[in] dataPacketID The data packet identifier. 
 * \param[in] dataAvailability The output variable storing the data availability. 
 *
 * \retval XME_STATUS_SUCCESS If the read position advance in the queue was success. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the database configuration 
 *         is not a valid configuration. 
 */
xme_status_t
xme_core_dataHandler_database_getDataAvailability
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint8_t* const dataAvailability
);

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_DATABASE_H */

