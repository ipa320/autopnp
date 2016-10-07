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
 * $Id: databaseTestProbe.h 7773 2014-03-11 17:22:36Z ruiz $
 */

/**
 * \file
 *         Database for Test Probe Abstraction.
 *
 * \brief This component is focused only on operation performed by the Test Probe.
 */

#ifndef XME_CORE_DATAHANDLER_DATABASETESTPROBE_H
#define XME_CORE_DATAHANDLER_DATABASETESTPROBE_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataManagerTypes.h"

#include "xme/defines.h"

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/****************************************************************************/
/***   Write and read operations                                          ***/
/****************************************************************************/

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
 * \param[in] databaseIndex The index of the associated database.
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
xme_core_dataHandler_databaseTestProbe_writeData
(
    uint16_t databaseIndex,
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
 * \param[in] databaseIndex The index of the associated database.
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
xme_core_dataHandler_databaseTestProbe_writeAttribute
(
    uint16_t databaseIndex,
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
 * \param[in] databaseIndex The index of the associated database.
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
xme_core_dataHandler_databaseTestProbe_readData
(
    uint16_t databaseIndex,
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
 * \param[in] databaseIndex The index of the associated database.
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
xme_core_dataHandler_databaseTestProbe_readAttribute
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t attributeKey,
    uint32_t offsetInBytes,
    void* const targetBuffer,
    size_t targetBufferSizeInBytes,
    uint32_t* const readBytes
);

/**
 * \brief Starts a manipulation. 
 * \details Copies from the master database to the shadow database the data packet
 *          and applies the existing manipulations. Additionally the data packet is
 *          labeled as manipulated. 
 *
 * \param[in] dataStoreID The data store identifier. 
 * \param[in] offset The offset to which apply the manipulation. 
 * 
 * \retval XME_STATUS_SUCCESS If the manipulation can start. 
 * \retval TBC
 */
xme_status_t
xme_core_dataHandler_databaseTestProbe_startTopicManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t offset
);

/**
 * \brief Ends a manipulation. 
 * \details Restore the status when not applied this manipulation. 
 *          This means that if there are still pending manipulation on the
 *          current data store, the manipulations still apply. If there
 *          are no more manipulations left, the data packet is switched to 
 *          master database. 
 *
 * \param[in] dataStoreID The data store identifier. 
 * \param[in] offset The offset to which apply the manipulation. 
 * 
 * \retval XME_STATUS_SUCCESS If the manipulation can start. 
 * \retval TBC
 */
xme_status_t
xme_core_dataHandler_databaseTestProbe_endTopicManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t offset
);

/**
 * \brief Starts a manipulation on the attribute. 
 * \details Copies from the master database to the shadow database the data packet
 *          and applies the existing manipulations. Additionally the data packet is
 *          labeled as manipulated. 
 *
 * \param[in] dataStoreID The data store identifier. 
 * \param[in] offset The offset to which apply the manipulation. 
 * \param[in] attributeKey The attribute key to which apply the manipulation. 
 * 
 * \retval XME_STATUS_SUCCESS If the manipulation can start. 
 * \retval TBC
 */
xme_status_t
xme_core_dataHandler_databaseTestProbe_startAttributeManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t offset,
    uint32_t attributeKey
);

/**
 * \brief Ends a manipulation on an attribute. 
 * \details Restore the status when not applied this manipulation. 
 *          This means that if there are still pending manipulation on the
 *          current data store, the manipulations still apply. If there
 *          are no more manipulations left, the data packet is switched to 
 *          master database. 
 *
 * \param[in] dataStoreID The data store identifier. 
 * \param[in] offset The offset to which apply the manipulation. 
 * \param[in] attributeKey The attribute key to which apply the manipulation. 
 * 
 * \retval XME_STATUS_SUCCESS If the manipulation can start. 
 * \retval TBC
 */
xme_status_t
xme_core_dataHandler_databaseTestProbe_endAttributeManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t offset,
    uint32_t attributeKey
);

/**
 * \brief Activates a database for a given data packet. 
 * \details The activation is associated to the data packet, not to the whole 
 *          database. 
 *
 * \param[in] databaseIndex The index of the associated database.
 * \param[in] dataPacketID The data packet identifier. 
 * 
 * \retval XME_STATUS_SUCCESS If the activation takes place correctly. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the database configuration does not
 *         support such activation. 
 * \deprecated This function is overriden by startManipualtion and endManipulation. 
 *
 */
xme_status_t
xme_core_dataHandler_databaseTestProbe_setActiveDatabase
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint16_t databaseIndex
);

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_DATABASETESTPROBE_H */

