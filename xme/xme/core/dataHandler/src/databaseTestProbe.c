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
 * $Id: databaseTestProbe.c 7844 2014-03-14 14:11:49Z ruiz $
 */

/**
 * \file
 *         Database Test Probe Abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/dataHandler/internal/databaseTestProbe.h"
#include "xme/core/dataHandler/internal/databaseTestProbeManipulations.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Adds a manipulation to the table which stores the manipulations. 
 *
 * \note If the manipulation is already stored, the target value is modified with 
 *       the provided parameter. 
 * 
 * \param[in] dataStoreID The data store identifier. 
 * \param[in] address The address modified by the manipulation. 
 * \param[in] value The value to manipulate. 
 *
 * \retval XME_STATUS_SUCCESS If the manipulation was successfully added to the manipulations table.
 * \retval TBD
 */
xme_status_t
addManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uintptr_t address,
    uint32_t value
);

/**
 * \brief Removes a manipulation from the table which stores the manipulations. 
 *
 * \param[in] dataStoreID The data store identifier. 
 * \param[in] address The address modified by the manipulation. 
 *
 * \retval XME_STATUS_SUCCESS If the manipulation was successfully added to the manipulations table.
 * \retval XME_STATUS_NOT_FOUND If there are no manipulations corresponding to the provided data store and address. 
 * \retval TBD
 */
xme_status_t
removeManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uintptr_t address
);

/**
 * \brief Gets the number of remaining manipulations for a given data store. 
 *
 * \param[in] dataStoreID The data store identifier. 
 *
 * \return The number of remaining manipulations for the data store.
 */
uint32_t
getNumberOfManipulationsForDataStore
(
    xme_core_dataManager_dataStoreID_t dataStoreID
);

/**
 * \brief Makes a copy between two different data store copies.
 *
 * \param[in] dataStoreID The data packet identifier.
 * \param[in] sourceDatabaseIndex The index of the source database index.
 * \param[in] targetDatabaseIndex The index of the target database index.
 *
 * \retval XME_STATUS_SUCCESS If the copy from source to target database in data store was success.
 */
xme_status_t
copyBetweenMemoryRegionCopies
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint16_t sourceDatabaseIndex,
    uint16_t targetDatabaseIndex
);

/**
 * \brief Reapplies the manipulations to the shadow database.
 *
 * \param[in] dataStoreID The data store identifier.
 *
 * \retval XME_STATUS_SUCCESS If the copy from source to target database in data store was success.
 */
xme_status_t
reapplyManipulations
(
    xme_core_dataManager_dataStoreID_t dataStoreID
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_dataHandler_databaseTestProbe_writeData
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataStoreID,
    uint32_t offsetInBytes,
    const void* const sourceBuffer,
    size_t sourceBufferSizeInBytes,
    uint32_t* const writtenBytes
)
{
    uintptr_t dataAddress;
    size_t dataSizeInBytes;
    xme_core_dataHandler_database_dataStore_t* dataStore;
    uint32_t totalOffset;
    void* returnAddress;
    void* databaseAddress;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);

    XME_CHECK(databaseIndex <= dataStore->memoryRegion->numOfCopies, XME_STATUS_INVALID_CONFIGURATION);

    if (offsetInBytes >= dataStore->topicSize)
    {
        // We can not write outside the limit of the data packet. An error is returned. 
        *writtenBytes = 0U;
        return XME_STATUS_INVALID_PARAMETER;
    }

    // Calculate the total offset to add.
    totalOffset = dataStore->topicOffset + offsetInBytes;

    // Add queue write position. 
    if (dataStore->queueSize > 1)
    {
        totalOffset += (dataStore->dataStoreSize * dataStore->currentCBWritePosition);
    }

    // Get the number of bytes to use in the write operation.
    dataSizeInBytes = getBytesToCopy(dataStore->topicSize, sourceBufferSizeInBytes, offsetInBytes);

    XME_CHECK(XME_CORE_DATAHANDLER_DATABASE_INDEX_DEFAULT != databaseIndex, XME_STATUS_INVALID_PARAMETER);

    // Get database address. 
    databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, databaseIndex - 1U);

    // Check that the data address and size contains correct values. 
    XME_CHECK(0U != databaseAddress, XME_STATUS_NOT_FOUND);

    // Add the offset.
    dataAddress = ((uintptr_t) databaseAddress + totalOffset);

    // Do the copy operation
    returnAddress = xme_hal_mem_copy((void*) dataAddress, sourceBuffer, dataSizeInBytes);
    XME_CHECK(returnAddress == (void*) dataAddress, XME_STATUS_INTERNAL_ERROR);

    // Set the effectively written bytes. 
    *writtenBytes = dataSizeInBytes;

    // Check that the target buffer is not null-valued. 
    XME_CHECK(NULL != sourceBuffer, XME_STATUS_INVALID_CONFIGURATION);

    // Set the data availability to true. 
    XME_CHECK(XME_STATUS_SUCCESS == setDataAvailability(dataStore, true), XME_STATUS_INVALID_CONFIGURATION);

    if (XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW == databaseIndex)
    {
        // Add the manipulation. 
        uint32_t* value = (uint32_t*) sourceBuffer;
        xme_status_t status = addManipulation(dataStoreID, dataAddress, *value);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION);

        // Activate the shadow database. 
        dataStore->activeDatabaseIndex = XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_databaseTestProbe_writeAttribute
(
    uint16_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataStoreID,
    uint32_t attributeKey,
    uint32_t offsetInBytes,
    const void* const sourceBuffer,
    size_t sourceBufferSizeInBytes,
    uint32_t* const writtenBytes
)
{
    uintptr_t attributeAddress;
    size_t attributeSizeInBytes;
    xme_core_dataHandler_database_dataStore_t* dataStore;
    xme_core_dataHandler_database_attribute_t* attribute;
    uint32_t totalOffset;
    void* returnAddress;
    void* databaseAddress;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Obtain first the data packet and then the attribute.  
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);

    XME_CHECK(XME_STATUS_SUCCESS == getAttribute(dataStore, attributeKey, &attribute), XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(databaseIndex <= dataStore->memoryRegion->numOfCopies, XME_STATUS_INVALID_CONFIGURATION);

    if (offsetInBytes >= attribute->attributeValueSize)
    {
        // We can not write outside the limit of the data packet. An error is returned. 
        *writtenBytes = 0U;
        return XME_STATUS_INVALID_PARAMETER;
    }

    // Calculate the total offset to add.
    totalOffset = attribute->attributeOffset + offsetInBytes;

    // Add queue write position. 
    if (dataStore->queueSize > 1)
    {
        totalOffset += (dataStore->dataStoreSize * dataStore->currentCBWritePosition);
    }

    // Get the number of bytes to use in the write operation.
    attributeSizeInBytes = getBytesToCopy(attribute->attributeValueSize, sourceBufferSizeInBytes, offsetInBytes);

    XME_CHECK(XME_CORE_DATAHANDLER_DATABASE_INDEX_DEFAULT != databaseIndex, XME_STATUS_INVALID_PARAMETER);

    // Get database address. 
    databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, databaseIndex - 1U);

    // Check that the data address and size contains correct values. 
    XME_CHECK(0U != databaseAddress, XME_STATUS_NOT_FOUND);

    // Add the offset.
    attributeAddress = ((uintptr_t) databaseAddress + totalOffset);

    // Do the copy operation
    returnAddress = xme_hal_mem_copy((void*) attributeAddress, sourceBuffer, attributeSizeInBytes);
    XME_CHECK(returnAddress == (void*) attributeAddress, XME_STATUS_INTERNAL_ERROR);

    // Set the effectively written bytes. 
    *writtenBytes = attributeSizeInBytes;

    // Check that the target buffer is not null-valued. 
    XME_CHECK(NULL != sourceBuffer, XME_STATUS_INVALID_CONFIGURATION);

    // Set attribute availability to true. 
    XME_CHECK(XME_STATUS_SUCCESS == setAttributeAvailability(attribute, true), XME_STATUS_INVALID_CONFIGURATION);

    if (XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW == databaseIndex)
    {
        // Add the manipulation. 
        uint32_t* value = (uint32_t*) sourceBuffer;
        xme_status_t status = addManipulation(dataStoreID, attributeAddress, *value);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION);

        // Activate the shadow database. 
        dataStore->activeDatabaseIndex = XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_databaseTestProbe_readData
(
    xme_core_dataHandler_database_index_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataStoreID,
    uint32_t offsetInBytes,
    void* const targetBuffer,
    size_t targetBufferSizeInBytes,
    uint32_t* const readBytes
)
{
    uintptr_t dataAddress;
    size_t dataSizeInBytes;
    xme_core_dataHandler_database_dataStore_t* dataStore;
    uint32_t totalOffset;
    void* returnAddress;
    void* databaseAddress;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);

    XME_CHECK(databaseIndex <= dataStore->memoryRegion->numOfCopies, XME_STATUS_INVALID_CONFIGURATION);

    if (!dataStore->dataAvailable)
    {
        *readBytes = 0U;
        return XME_STATUS_NO_SUCH_VALUE;
    }

    if (offsetInBytes >= dataStore->topicSize)
    {
        // We can not read outside the limit of the data packet. An error is returned. 
        *readBytes = 0U;
        return XME_STATUS_INVALID_PARAMETER;
    }

    // Calculate the total offset to add.
    totalOffset = dataStore->topicOffset + offsetInBytes;

    // Add queue write position. 
    if (dataStore->queueSize > 1)
    {
        totalOffset += (dataStore->dataStoreSize * dataStore->currentCBReadPosition);
    }

    // Get the number of bytes to use in the read operation.
    dataSizeInBytes = getBytesToCopy(dataStore->topicSize, targetBufferSizeInBytes, offsetInBytes);

    if (((xme_core_dataHandler_database_index_t)XME_CORE_DATAHANDLER_DATABASE_INDEX_DEFAULT) == databaseIndex)
    {
        // Get database address from the active database in the data packet. 
        databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, dataStore->activeDatabaseIndex - 1U);
    }
    else
    {
        // Get database address from the provided database index. 
        databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, databaseIndex - 1U);
    }

    // Check that the database address and size contains correct values. 
    XME_CHECK(0U != databaseAddress, XME_STATUS_NOT_FOUND);

    // Add the offset.
    dataAddress = ((uintptr_t) databaseAddress + totalOffset);

    if (targetBufferSizeInBytes > dataSizeInBytes)
    {
        uintptr_t bufferToCopy;

        bufferToCopy = ((uintptr_t) targetBuffer + (targetBufferSizeInBytes - dataSizeInBytes));
        returnAddress = xme_hal_mem_copy((void*) bufferToCopy, (void*) dataAddress, dataSizeInBytes);
        XME_CHECK(returnAddress == (void*) bufferToCopy, XME_STATUS_INTERNAL_ERROR);
    }
    else
    {
        // Do the read operation
        returnAddress = xme_hal_mem_copy(targetBuffer, (void*) dataAddress, dataSizeInBytes);
        XME_CHECK(returnAddress == (void*) targetBuffer, XME_STATUS_INTERNAL_ERROR);
    }

    // Set the effectively read bytes. 
    *readBytes = dataSizeInBytes;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_databaseTestProbe_readAttribute
(
    xme_core_dataHandler_database_index_t databaseIndex,
    xme_core_dataManager_dataPacketId_t dataStoreID,
    uint32_t attributeKey,
    uint32_t offsetInBytes,
    void* const targetBuffer,
    size_t targetBufferSizeInBytes,
    uint32_t* const readBytes
)
{
    uintptr_t attributeAddress;
    size_t attributeSizeInBytes;
    xme_core_dataHandler_database_dataStore_t* dataStore;
    xme_core_dataHandler_database_attribute_t* attribute;
    uint32_t totalOffset;
    void* returnAddress;
    void* databaseAddress;

    XME_ASSERT(NULL != xme_core_dataHandler_database);
    XME_ASSERT(NULL != readBytes);

    *readBytes = 0U;

    // Obtain first the data packet and then the attribute.  
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);
    XME_CHECK(XME_STATUS_SUCCESS == getAttribute(dataStore, attributeKey, &attribute), XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(databaseIndex <= dataStore->memoryRegion->numOfCopies, XME_STATUS_INVALID_CONFIGURATION);

    XME_CHECK(attribute->dataAvailable, XME_STATUS_NO_SUCH_VALUE);

    // Return an error if an attempt is made to read outside the bounds of the data packet
    XME_CHECK(offsetInBytes < attribute->attributeValueSize, XME_STATUS_INVALID_PARAMETER);

    // Calculate the total offset to add.
    totalOffset = attribute->attributeOffset + offsetInBytes;

    // Add queue write position. 
    if (dataStore->queueSize > 1)
    {
        totalOffset += (dataStore->dataStoreSize * dataStore->currentCBReadPosition);
    }

    // Get the number of bytes to use in the read operation.
    attributeSizeInBytes = getBytesToCopy(attribute->attributeValueSize, targetBufferSizeInBytes, offsetInBytes);

    if (((xme_core_dataHandler_database_index_t)XME_CORE_DATAHANDLER_DATABASE_INDEX_DEFAULT) == databaseIndex)
    {
        // Get database address from the active database in the data packet. 
        databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, dataStore->activeDatabaseIndex - 1U);
    }
    else
    {
        // Get database address from the provided database index. 
        databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, databaseIndex - 1U);
    }

    // Check that the database address and size contains correct values. 
    XME_CHECK(0U != databaseAddress, XME_STATUS_NOT_FOUND);

    // Add the offset.
    attributeAddress = ((uintptr_t) databaseAddress + totalOffset);

    if (targetBufferSizeInBytes > attributeSizeInBytes)
    {
        uintptr_t bufferToCopy;

        bufferToCopy = ((uintptr_t) targetBuffer + (targetBufferSizeInBytes - attributeSizeInBytes));
        returnAddress = xme_hal_mem_copy(targetBuffer, (void*) bufferToCopy, attributeSizeInBytes);
        XME_CHECK(returnAddress == (void*) targetBuffer, XME_STATUS_INTERNAL_ERROR);
    }
    else
    {
        // Do the read operation
        returnAddress = xme_hal_mem_copy(targetBuffer, (void*) attributeAddress, attributeSizeInBytes);
        XME_CHECK(returnAddress == (void*) targetBuffer, XME_STATUS_INTERNAL_ERROR);
    }

    // Set the effectively read bytes. 
    *readBytes = attributeSizeInBytes;

    // Check that the target buffer is not null-valued. 
    XME_CHECK(NULL != targetBuffer, XME_STATUS_INVALID_CONFIGURATION);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_databaseTestProbe_startTopicManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t offset
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;
    xme_status_t status;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information. 
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);

    if (offset > dataStore->dataStoreSize)
    {
        return XME_STATUS_INVALID_CONFIGURATION;
    }

    if (!dataStore->manipulated)
    {
        status = copyBetweenMemoryRegionCopies
            (
                dataStoreID, 
                XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER, 
                XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW
            );
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        status = reapplyManipulations(dataStore->dataStoreID);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        dataStore->manipulated = true;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_databaseTestProbe_endTopicManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t offset
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;
    void* databaseAddress;
    uintptr_t dataAddress;
    xme_status_t status;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information. 
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);

    if (offset > dataStore->dataStoreSize)
    {
        return XME_STATUS_INVALID_CONFIGURATION;
    }

    databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW - 1U);
    XME_CHECK(0U != databaseAddress, XME_STATUS_NOT_FOUND);
    dataAddress = (((uintptr_t) databaseAddress) + dataStore->topicOffset + offset);

    status = removeManipulation(dataStoreID, dataAddress);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    if (0U == getNumberOfManipulationsForDataStore(dataStoreID))
    {
        dataStore->manipulated = false;

        // Activate master database.
        dataStore->activeDatabaseIndex = XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER;
    }
    else
    {
        status = copyBetweenMemoryRegionCopies
            (
                dataStoreID, 
                XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER, 
                XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW
            );
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        status = reapplyManipulations(dataStore->dataStoreID);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_databaseTestProbe_startAttributeManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t offset,
    uint32_t attributeKey
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;
    xme_core_dataHandler_database_attribute_t* attribute;
    xme_status_t status;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information. 
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);
    XME_CHECK(XME_STATUS_SUCCESS == getAttribute(dataStore, attributeKey, &attribute), XME_STATUS_INVALID_PARAMETER);

    if (offset > attribute->attributeValueSize)
    {
        return XME_STATUS_INVALID_CONFIGURATION;
    }

    if (!dataStore->manipulated)
    {
        status = copyBetweenMemoryRegionCopies
            (
                dataStoreID, 
                XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER, 
                XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW
            );
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        status = reapplyManipulations(dataStore->dataStoreID);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        dataStore->manipulated = true;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_databaseTestProbe_endAttributeManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t offset,
    uint32_t attributeKey
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;
    xme_core_dataHandler_database_attribute_t* attribute;
    void* databaseAddress;
    uintptr_t dataAddress;
    xme_status_t status;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information. 
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);
    XME_CHECK(XME_STATUS_SUCCESS == getAttribute(dataStore, attributeKey, &attribute), XME_STATUS_INVALID_PARAMETER);

    if (offset > attribute->attributeValueSize)
    {
        return XME_STATUS_INVALID_CONFIGURATION;
    }

    databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW - 1U);
    XME_CHECK(0U != databaseAddress, XME_STATUS_NOT_FOUND);
    dataAddress = (((uintptr_t) databaseAddress) + attribute->attributeOffset + offset);

    status = removeManipulation(dataStoreID, dataAddress);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    if (0U == getNumberOfManipulationsForDataStore(dataStoreID))
    {
        dataStore->manipulated = false;

        // Activate master database.
        dataStore->activeDatabaseIndex = XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER;
    }
    else
    {
        status = copyBetweenMemoryRegionCopies
            (
                dataStoreID, 
                XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER, 
                XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW
            );
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        status = reapplyManipulations(dataStore->dataStoreID);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }

    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
// FIXME: Remove.
xme_status_t
xme_core_dataHandler_databaseTestProbe_setActiveDatabase
(
    xme_core_dataManager_dataPacketId_t dataStoreID,
    uint16_t databaseIndex
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;

    XME_ASSERT(NULL != xme_core_dataHandler_database);
    XME_CHECK(XME_CORE_DATAHANDLER_DATABASE_INDEX_DEFAULT != databaseIndex, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);

    XME_CHECK(databaseIndex <= dataStore->memoryRegion->numOfCopies, XME_STATUS_INVALID_CONFIGURATION);

    dataStore->activeDatabaseIndex = databaseIndex;

    return XME_STATUS_SUCCESS;
}


//******************************************************************************//

uint32_t
getNumberOfManipulationsForDataStore
(
    xme_core_dataManager_dataStoreID_t dataStoreID
)
{
    xme_core_dataHandler_database_manipulationItem_t* manipulationItem;
    xme_hal_table_rowHandle_t internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    uint32_t numberOfRemainingManipulations = 0U;

    do
    {
        // Obtain the next element. 
        XME_HAL_TABLE_GET_NEXT
        (
            manipulationsTable,
            xme_hal_table_rowHandle_t, internalHandle, 
            xme_core_dataHandler_database_manipulationItem_t, manipulationItem, 
            (manipulationItem->dataStoreID == dataStoreID)
        );

        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == internalHandle) continue;

        numberOfRemainingManipulations++;
    } 
    while(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle);

    return numberOfRemainingManipulations;
}

xme_status_t
addManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uintptr_t address,
    uint32_t value
)
{
    xme_core_dataHandler_database_manipulationItem_t* manipulationItem = NULL;
    xme_hal_table_rowHandle_t internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    // Obtain the next element. 
    XME_HAL_TABLE_GET_NEXT
    (
        manipulationsTable,
        xme_hal_table_rowHandle_t, internalHandle, 
        xme_core_dataHandler_database_manipulationItem_t, manipulationItem, 
        (manipulationItem->dataStoreID == dataStoreID && manipulationItem->address == (uintptr_t) address)
    );

    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == internalHandle)
    {
        // Create a new element. 
        internalHandle = XME_HAL_TABLE_ADD_ITEM(manipulationsTable);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, XME_STATUS_OUT_OF_RESOURCES);
        manipulationItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(manipulationsTable, internalHandle);
        XME_ASSERT(NULL != manipulationItem);
        manipulationItem->dataStoreID = dataStoreID;
        manipulationItem->address = address;
        manipulationItem->value = value;
    }
    else
    {
        // Use existing element. 
        manipulationItem->value = value;
    }
    return XME_STATUS_SUCCESS;
}

xme_status_t
removeManipulation
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uintptr_t address
)
{
    xme_core_dataHandler_database_manipulationItem_t* manipulationItem;
    xme_hal_table_rowHandle_t internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    xme_status_t status;

    // Obtain the next element. 
    XME_HAL_TABLE_GET_NEXT
    (
        manipulationsTable,
        xme_hal_table_rowHandle_t, internalHandle, 
        xme_core_dataHandler_database_manipulationItem_t, manipulationItem, 
        (manipulationItem->dataStoreID == dataStoreID && manipulationItem->address == address)
    );

    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == internalHandle)
    {
        return XME_STATUS_NOT_FOUND;
    }

    // Remove the element. 
    status = XME_HAL_TABLE_REMOVE_ITEM(manipulationsTable, internalHandle);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_databaseTestProbe_applyManipulations
(
    xme_core_dataHandler_database_dataStore_t* const dataStore
)
{
    xme_status_t status;
        
    // Copy again the data packet. 
    status = copyBetweenMemoryRegionCopies
        (
            dataStore->dataStoreID, 
            XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER, 
            XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW
        );
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    status = reapplyManipulations(dataStore->dataStoreID);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    return XME_STATUS_SUCCESS;
}

xme_status_t
copyBetweenMemoryRegionCopies
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint16_t sourceDatabaseIndex,
    uint16_t targetDatabaseIndex
)
{
    uintptr_t sourceDataAddress;
    uintptr_t sinkDataAddress;
    xme_core_dataHandler_database_dataStore_t* dataStore;
    uint32_t transferSizeInBytes;
    uint32_t totalOffset;
    void* returnAddress;
    void* sourceDatabaseAddress;
    void* sinkDatabaseAddress;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information.
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);

    // Calculate the total offsets to add.
    totalOffset = dataStore->topicOffset;

    // Calculate the position in the queue for the source and sink address.
    if (dataStore->queueSize > 1)
    {
        totalOffset += dataStore->dataStoreSize * dataStore->currentCBWritePosition;
    }

    // Get the number of bytes to use in the transfer operation.
    transferSizeInBytes = getBytesToCopy(dataStore->topicSize, dataStore->topicSize, 0U);

    // Get source database address from the source database copy.
    sourceDatabaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, sourceDatabaseIndex - 1U);

    // Get source database address from the source database copy.
    sinkDatabaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, targetDatabaseIndex - 1U); 

    // Check that the database addresses contain correct values.
    XME_CHECK(0U != sourceDatabaseAddress, XME_STATUS_NOT_FOUND);
    XME_CHECK(0U != sinkDatabaseAddress, XME_STATUS_NOT_FOUND);

    if (dataStore->dataAvailable)
    {
        // Add the offset corresponding to the source and sink addresses.
        sourceDataAddress = ((uintptr_t) sourceDatabaseAddress + totalOffset);
        sinkDataAddress = ((uintptr_t) sinkDatabaseAddress + totalOffset);

        // Do the transfer for the current copy of the database->
        returnAddress = xme_hal_mem_copy((void*) sinkDataAddress, (void*) sourceDataAddress, transferSizeInBytes);
        XME_CHECK(returnAddress == (void*) sinkDataAddress, XME_STATUS_INTERNAL_ERROR);
    }

    // Try the transfer for all attributes as well.
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(dataStore->attributes, xme_core_dataHandler_database_attribute_t, attribute);
    {
        if (attribute->dataAvailable == true)
        {
            // If the attribute exists in the target data packet.
            uint32_t totalAttributeOffset;
            uintptr_t sourceAttributeAddress;
            uintptr_t sinkAttributeAddress;
            uint32_t totalAttributeSizeInBytes;

            // Calculate the total offsets to add.
            totalAttributeOffset = attribute->attributeOffset;

            // Calculate the position in the queue for the source and sink address.
            if (dataStore->queueSize > 1)
            {
                totalAttributeOffset += dataStore->dataStoreSize * dataStore->currentCBWritePosition;
            }

            // Get trasnfer bytes to copy.
            totalAttributeSizeInBytes = getBytesToCopy(attribute->attributeValueSize, attribute->attributeValueSize, 0U);

            // Add the offset corresponding to the source and sink addresses.
            sourceAttributeAddress = ((uintptr_t) sourceDatabaseAddress + totalAttributeOffset);
            sinkAttributeAddress = ((uintptr_t) sinkDatabaseAddress + totalAttributeOffset);

            // Do the transfer for the current copy of the database->
            returnAddress = xme_hal_mem_copy((void*) sinkAttributeAddress, (void*) sourceAttributeAddress, totalAttributeSizeInBytes);
            XME_CHECK(returnAddress == (void*) sinkAttributeAddress, XME_STATUS_INTERNAL_ERROR);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

xme_status_t
reapplyManipulations
(
    xme_core_dataManager_dataStoreID_t dataStoreID
)
{
    xme_core_dataHandler_database_manipulationItem_t* manipulationItem = NULL;
    xme_hal_table_rowHandle_t internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    do
    {
        void* returnAddress;

        // Obtain the next element. 
        XME_HAL_TABLE_GET_NEXT
        (
            manipulationsTable,
            xme_hal_table_rowHandle_t, internalHandle, 
            xme_core_dataHandler_database_manipulationItem_t, manipulationItem, 
            (manipulationItem->dataStoreID == dataStoreID)
        );

        if (XME_HAL_TABLE_INVALID_ROW_HANDLE == internalHandle) continue;

        XME_ASSERT(NULL != manipulationItem);

        returnAddress = xme_hal_mem_copy((void*) manipulationItem->address, &(manipulationItem->value), sizeof(manipulationItem->value));
        XME_CHECK(returnAddress == (void*) manipulationItem->address, XME_STATUS_INTERNAL_ERROR);
    } 
    while(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle);

    return XME_STATUS_SUCCESS;
}

