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
 * $Id: dataHandlerTestProbe.c 7773 2014-03-11 17:22:36Z ruiz $
 */

/**
 * \file
 *         Data Handler.
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataHandler/include/dataHandlerTestProbe.h"
#include "xme/core/dataHandler/internal/databaseTestProbe.h"

#include "xme/core/log.h"

#include <inttypes.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_dataHandlerTestProbe_writeDataInDatabase
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    const void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t* const writtenBytes
)
{
    xme_status_t status;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0U != bufferSizeInBytes, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != buffer, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != writtenBytes, XME_STATUS_INVALID_PARAMETER);

    // Perform the write operation. 
    status = xme_core_dataHandler_databaseTestProbe_writeData
        (
            databaseIndex, // The database index to use in write operation. 
            dataPacketID, // The data packet identifier. 
            offsetInBytes, // The target offset inside the data in which the write operation should take place. 
            buffer, // The buffer for the data. 
            bufferSizeInBytes, // The buffer size in bytes.
            writtenBytes // The written bytes. 
        );

    // Check if the write operation was success.
    XME_CHECK_MSG
    (
        XME_STATUS_SUCCESS == status,
        status,
        XME_LOG_WARNING,
        "[DataHandlerTestProbe] Writing data to data packet failed (dataPacketID=%d).\n",
        dataPacketID
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandlerTestProbe_writeAttributeInDatabase
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t attributeKey,
    uint32_t offsetInBytes,
    const void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t* const writtenBytes
)
{
    xme_status_t status;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0U != bufferSizeInBytes, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != buffer, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != writtenBytes, XME_STATUS_INVALID_PARAMETER);

    // Perform the write operation. 
    status = xme_core_dataHandler_databaseTestProbe_writeAttribute
        (
            databaseIndex, // The database index to use in write operation. 
            dataPacketID, // The data packet identifier. 
            attributeKey, // The attribute key. 
            offsetInBytes, // The target offset inside the data in which the write operation should take place. 
            buffer, // The buffer for the data. 
            bufferSizeInBytes, // The buffer size in bytes. 
            writtenBytes // The written bytes. 
        );

    // Check if the write operation was success. 
    XME_CHECK_MSG
    (
        XME_STATUS_SUCCESS == status,
        status,
        XME_LOG_WARNING,
        "[DataHandlerTestProbe] Writing attribute value to data packet failed (dataPacketID=%d, attributeKey=%d).\n",
        dataPacketID,
        attributeKey
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandlerTestProbe_readDataInDatabase
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t* const readBytes
)
{
    xme_status_t status;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0U != bufferSizeInBytes, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != buffer, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != readBytes, XME_STATUS_INVALID_PARAMETER);

    // Perform the read operation. 
    status = xme_core_dataHandler_databaseTestProbe_readData
        (
            databaseIndex, // The database index to use in write operation. 
            dataPacketID, // The data packet identifier. 
            offsetInBytes, // The target offset inside the data in which the write operation should take place. 
            buffer, // The buffer for the data. 
            bufferSizeInBytes, // The buffer size in bytes. 
            readBytes // The output read bytes.
        );

    if (XME_STATUS_SUCCESS != status)
    {
        XME_LOG
        (
            XME_LOG_DEBUG,
            "[DataHandlerTestProbe] Read operation from data packet failed (dataPacketID=%d). status=%d\n",
            dataPacketID,
            status
        );
    }

    return status;
}

xme_status_t
xme_core_dataHandlerTestProbe_readAttributeInDatabase
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t attributeKey,
    uint32_t offsetInBytes,
    void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t* const readBytes
)
{
    xme_status_t status;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0U != bufferSizeInBytes, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != buffer, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != readBytes, XME_STATUS_INVALID_PARAMETER);

    // Perform the read operation. 
    status = xme_core_dataHandler_databaseTestProbe_readAttribute
        (
            databaseIndex, // The database index to use in write operation. 
            dataPacketID, // The data packet identifier. 
            attributeKey, // The attribute key. 
            offsetInBytes, // The target offset inside the data in which the write operation should take place. 
            buffer, // The buffer for the data. 
            bufferSizeInBytes, // The buffer size in bytes. 
            readBytes
        );

    if (XME_STATUS_SUCCESS != status)
    {
        XME_LOG
        (
            XME_LOG_DEBUG,
            "[DataHandlerTestProbe] Read operation from data packet failed (dataPacketID=%d, attributeKey=%d). status=%d\n",
            dataPacketID,
            attributeKey,
            status
        );
    }

    return status;
}

xme_status_t
xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    xme_status_t status;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);

    // Direct call to the database function activating the corresponding index.
    status = xme_core_dataHandler_databaseTestProbe_setActiveDatabase
        (
            dataPacketID, // The data packet identifier. 
            databaseIndex // The database index to use in write operation. 
        );

    // 2. Check the result of the activation. 
    XME_CHECK_MSG
    (
        XME_STATUS_SUCCESS == status,
        status,
        XME_LOG_WARNING,
        "[DataHandlerTestProbe] Cannot activate the %d database.\n",
        databaseIndex
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandlerTestProbe_startManipulation
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes
)
{
    return xme_core_dataHandlerTestProbe_startAttributeManipulation(dataPacketID, offsetInBytes, 0U);
}

xme_status_t
xme_core_dataHandlerTestProbe_startAttributeManipulation
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    uint32_t attributeKey
)
{
    xme_status_t status;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);

    if (0U == attributeKey)
    {
        status = xme_core_dataHandler_databaseTestProbe_startTopicManipulation(dataPacketID, offsetInBytes);

        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status,
            status,
            XME_LOG_WARNING,
            "[DataHandlerTestProbe] Cannot start manipulation for dataPacketID=%d and offsetInBytes=%d (status=%d).\n",
            dataPacketID,
            offsetInBytes,
            status
        );
    }
    else
    {
        status = xme_core_dataHandler_databaseTestProbe_startAttributeManipulation(dataPacketID, offsetInBytes, attributeKey);

        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status,
            status,
            XME_LOG_WARNING,
            "[DataHandlerTestProbe] Cannot start manipulation for dataPacketID=%d, attributeKey=%d and offsetInBytes=%d (status=%d).\n",
            dataPacketID,
            attributeKey,
            offsetInBytes,
            status
        );
    }

    return status;
}

xme_status_t
xme_core_dataHandlerTestProbe_endManipulation
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes
)
{
    return xme_core_dataHandlerTestProbe_endAttributeManipulation(dataPacketID, offsetInBytes, 0U);
}

xme_status_t
xme_core_dataHandlerTestProbe_endAttributeManipulation
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t offsetInBytes,
    uint32_t attributeKey
)
{
    xme_status_t status;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);

    if (0U == attributeKey)
    {
        status = xme_core_dataHandler_databaseTestProbe_endTopicManipulation(dataPacketID, offsetInBytes);

        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status,
            status,
            XME_LOG_WARNING,
            "[DataHandlerTestProbe] Cannot finish manipulation for dataPacketID=%d and offsetInBytes=%d (status=%d).\n",
            dataPacketID,
            offsetInBytes,
            status
        );
    }
    else
    {
        status = xme_core_dataHandler_databaseTestProbe_endAttributeManipulation(dataPacketID, offsetInBytes, attributeKey);

        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status,
            status,
            XME_LOG_WARNING,
            "[DataHandlerTestProbe] Cannot start manipulation for dataPacketID=%d, attributeKey=%d and offsetInBytes=%d (status=%d).\n",
            dataPacketID,
            attributeKey,
            offsetInBytes,
            status
        );
    }

    return status;
}
