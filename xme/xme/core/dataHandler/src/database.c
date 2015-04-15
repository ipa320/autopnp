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
 * $Id: database.c 7844 2014-03-14 14:11:49Z ruiz $
 */

/**
 * \file
 *         Database Abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/dataHandler/internal/database.h"

#include "xme/core/dataHandler/internal/databaseInternalMethods.h"
#include "xme/core/dataHandler/internal/databaseTestProbeManipulations.h"

#include "xme/core/component.h"
#include "xme/core/log.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/mmap.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_dataHandler_database_init(void)
{
    return xme_core_dataHandler_databaseInternal_init();
}

void
xme_core_dataHandler_database_fini(void)
{
    xme_core_dataHandler_databaseInternal_fini();
}

xme_status_t
xme_core_dataHandler_database_writeData
(
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
    size_t totalOffset;
    void* returnAddress;
    void* databaseAddress;
    bool enableWrite;
    xme_status_t status;
    bool isOverwrite;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);

    status = checkOverwriteBeforeWrite(dataStore, dataStore->dataAvailable, &enableWrite, &isOverwrite);
    XME_CHECK_REC(
        XME_STATUS_SUCCESS == status && enableWrite == true,
        status,
        {
            *writtenBytes = 0U;
        }
    );

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

    // The write should always be performed in master database. 
    // Get database address. 
    databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER - 1U);

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

    if (dataStore->manipulated == true)
    {
        // Normal write. 
        status = xme_core_dataHandler_databaseTestProbe_applyManipulations(dataStore);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION);
    }

    if (isOverwrite == true)
    {
        status = checkOverwriteAfterWrite(dataStore);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_database_writeAttribute
(
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
    size_t totalOffset;
    void* returnAddress;
    void* databaseAddress;
    bool enableWrite;
    bool isOverwrite;
    xme_status_t status;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Obtain first the data packet and then the attribute.  
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);
    XME_CHECK(XME_STATUS_SUCCESS == getAttribute(dataStore, attributeKey, &attribute), XME_STATUS_INVALID_PARAMETER);

    status = checkOverwriteBeforeWrite(dataStore, attribute->dataAvailable, &enableWrite, &isOverwrite);
    XME_CHECK_REC(
        XME_STATUS_SUCCESS == status && enableWrite == true,
        status,
        {
            *writtenBytes = 0U;
        }
    );

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

    // The write should always be performed in master database. 
    // Get database address. 
    databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER - 1U);
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
    XME_CHECK(XME_STATUS_SUCCESS == setAttributeAvailability(attribute, (bool) true), XME_STATUS_INVALID_CONFIGURATION);

    if (dataStore->manipulated)
    {
        // Normal write. 
        status = xme_core_dataHandler_databaseTestProbe_applyManipulations(dataStore);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION);
    }

    // Check overwrite. 
    if (isOverwrite == (bool) true)
    {
        status = checkOverwriteAfterWrite(dataStore);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_database_readData
(
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
    size_t totalOffset;
    void* returnAddress;
    void* databaseAddress;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);

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

    // Get database address from the active database in the data packet. 
    databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, dataStore->activeDatabaseIndex - 1U);

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
xme_core_dataHandler_database_readAttribute
(
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
    size_t totalOffset;
    void* returnAddress;
    void* databaseAddress;

    XME_ASSERT(NULL != xme_core_dataHandler_database);
    XME_ASSERT(NULL != readBytes);

    *readBytes = 0U;

    // Obtain first the data packet and then the attribute.  
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);
    XME_CHECK(XME_STATUS_SUCCESS == getAttribute(dataStore, attributeKey, &attribute), XME_STATUS_INVALID_PARAMETER);

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

    // Get database address from the active database in the data packet. 
    databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, dataStore->activeDatabaseIndex - 1U);

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
xme_core_dataHandler_database_transfer
(
    xme_core_dataManager_dataPacketId_t sourceDataStoreID,
    xme_core_dataManager_dataPacketId_t sinkDataStoreID
)
{
    uintptr_t sourceDataAddress;
    uintptr_t sinkDataAddress;
    xme_status_t status;
    xme_core_dataHandler_database_dataStore_t* sourceDataStore;
    xme_core_dataHandler_database_dataStore_t* sinkDataStore;
    uint32_t transferSizeInBytes;
    size_t totalSourceOffset;
    size_t totalSinkOffset;
    void* returnAddress;
    void* sourceDatabaseAddress;
    void* sinkDatabaseAddress;
    uint8_t databaseCopyIndex;
    bool enableWrite;
    bool isOverwrite;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information. 
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(sourceDataStoreID, &sourceDataStore), XME_STATUS_INVALID_HANDLE);
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(sinkDataStoreID, &sinkDataStore), XME_STATUS_INVALID_HANDLE);

    status = checkOverwriteBeforeWrite(sinkDataStore, sinkDataStore->dataAvailable, &enableWrite, &isOverwrite);
    XME_CHECK(XME_STATUS_SUCCESS == status && enableWrite == true, status);

    // Make the comparations between source and destination size. 
    XME_CHECK(sourceDataStore->topicSize == sinkDataStore->topicSize, XME_STATUS_INVALID_CONFIGURATION);

    // Calculate the total offsets to add.
    totalSourceOffset = sourceDataStore->topicOffset;
    totalSinkOffset = sinkDataStore->topicOffset;

    // Calculate the position in the queue for the source and sink address.
    if (sinkDataStore->queueSize > 1)
    {
        totalSinkOffset += sinkDataStore->dataStoreSize * sinkDataStore->currentCBWritePosition;
    }

    if (sourceDataStore->queueSize > 1)
    {
        totalSourceOffset += sourceDataStore->dataStoreSize * sourceDataStore->currentCBReadPosition;
    }

    // Get the number of bytes to use in the transfer operation.
    transferSizeInBytes = getBytesToCopy(sourceDataStore->topicSize, sinkDataStore->topicSize, 0U);

    for (databaseCopyIndex = 0; databaseCopyIndex < sourceDataStore->memoryRegion->numOfCopies; databaseCopyIndex++)
    {
        // Get source database address from the source database copy. 
        sourceDatabaseAddress = getAddressFromMemoryCopy(sourceDataStore->memoryRegion, databaseCopyIndex);

        // Get source database address from the source database copy. 
        sinkDatabaseAddress = getAddressFromMemoryCopy(sinkDataStore->memoryRegion, databaseCopyIndex);

        // Check that the database addresses contain correct values. 
        XME_CHECK(0U != sourceDatabaseAddress, XME_STATUS_NOT_FOUND);
        XME_CHECK(0U != sinkDatabaseAddress, XME_STATUS_NOT_FOUND);

        if (sourceDataStore->dataAvailable)
        {
            // Add the offset corresponding to the source and sink addresses.
            sourceDataAddress = ((uintptr_t) sourceDatabaseAddress + totalSourceOffset);
            sinkDataAddress = ((uintptr_t) sinkDatabaseAddress + totalSinkOffset);

            // Do the transfer for the current copy of the database-> 
            returnAddress = xme_hal_mem_copy((void*) sinkDataAddress, (void*) sourceDataAddress, transferSizeInBytes);
            XME_CHECK(returnAddress == (void*) sinkDataAddress, XME_STATUS_INTERNAL_ERROR);
        }

        // Try the transfer for all attributes as well. 
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(sinkDataStore->attributes, xme_core_dataHandler_database_attribute_t, sinkAttribute);
        {
            xme_core_dataHandler_database_attribute_t* sourceAttribute;

            status = getAttribute(sourceDataStore, sinkAttribute->key, &sourceAttribute);
            if ((XME_STATUS_SUCCESS == status) && sourceAttribute->dataAvailable)
            {
                // If the attribute exists in the target data packet.
                size_t totalSourceAttributeOffset;
                size_t totalSinkAttributeOffset;
                uintptr_t sourceAttributeAddress;
                uintptr_t sinkAttributeAddress;
                uint32_t totalAttributeSizeInBytes;

                // Calculate the total offsets to add.
                totalSourceAttributeOffset = sourceAttribute->attributeOffset;
                totalSinkAttributeOffset = sinkAttribute->attributeOffset;

                // Calculate the position in the queue for the source and sink address.
                if (sinkDataStore->queueSize > 1)
                {
                    totalSinkAttributeOffset += sinkDataStore->dataStoreSize * sinkDataStore->currentCBWritePosition;
                }

                if (sourceDataStore->queueSize > 1)
                {
                    totalSourceAttributeOffset += sourceDataStore->dataStoreSize * sourceDataStore->currentCBReadPosition;
                }

                // Get trasnfer bytes to copy. 
                totalAttributeSizeInBytes = getBytesToCopy(sourceAttribute->attributeValueSize, sinkAttribute->attributeValueSize, 0U);

                // Add the offset corresponding to the source and sink addresses.
                sourceAttributeAddress = ((uintptr_t) sourceDatabaseAddress + totalSourceAttributeOffset);
                sinkAttributeAddress = ((uintptr_t) sinkDatabaseAddress + totalSinkAttributeOffset);

                // Do the transfer for the current copy of the database-> 
                returnAddress = xme_hal_mem_copy((void*) sinkAttributeAddress, (void*) sourceAttributeAddress, totalAttributeSizeInBytes);
                XME_CHECK(returnAddress == (void*) sinkAttributeAddress, XME_STATUS_INTERNAL_ERROR);

                XME_CHECK(XME_STATUS_SUCCESS == setAttributeAvailability(sinkAttribute, (bool) true), XME_STATUS_INTERNAL_ERROR);
            }
            else
            {
                XME_CHECK(XME_STATUS_SUCCESS == setAttributeAvailability(sinkAttribute, (bool) false), XME_STATUS_INTERNAL_ERROR);
            }
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    }
    // Update data availability in sink data packet. 
    XME_CHECK(XME_STATUS_SUCCESS == setDataAvailability(sinkDataStore, sourceDataStore->dataAvailable), XME_STATUS_INVALID_CONFIGURATION);

    if (isOverwrite == true)
    {
        status = checkOverwriteAfterWrite(sinkDataStore);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_database_resetData
(
    xme_core_dataManager_dataPacketId_t dataStoreID
)
{
    uintptr_t dataAddress;
    xme_core_dataHandler_database_dataStore_t* dataStore;
    void* returnAddress;
    void* databaseAddress;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information. 
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_INVALID_HANDLE);

    // Get database address. 
    databaseAddress = getAddressFromMemoryCopy(dataStore->memoryRegion, dataStore->activeDatabaseIndex - 1U);

    // Check that the data address and size contains correct values. 
    XME_CHECK(0U != databaseAddress, XME_STATUS_NOT_FOUND);

    // Add the offset.
    dataAddress = ((uintptr_t) databaseAddress + dataStore->topicOffset);

    // Do the copy operation
    returnAddress = xme_hal_mem_set((void*) dataAddress, 0U, dataStore->topicSize);
    XME_CHECK(returnAddress == (void*) dataAddress, XME_STATUS_INTERNAL_ERROR);

    // Do the same for each attribute. 
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(dataStore->attributes, xme_core_dataHandler_database_attribute_t, attributeItem);
    {
        uintptr_t attributeAddress;
        size_t attributeSizeInBytes;
        xme_core_dataHandler_database_attribute_t* attribute;

        // Get the attribute information. 
        XME_CHECK(XME_STATUS_SUCCESS == getAttribute(dataStore, attributeItem->key, &attribute), XME_STATUS_NOT_FOUND);

        // Get attribute address. 
        attributeAddress = ((uintptr_t) databaseAddress + attribute->attributeOffset);

        // Set the attribute value size. 
        attributeSizeInBytes = attribute->attributeValueSize;

        // Reset the attribute data.
        returnAddress = xme_hal_mem_set((void*) attributeAddress, 0U, attributeSizeInBytes);
        XME_CHECK(returnAddress == (void*) attributeAddress, XME_STATUS_INTERNAL_ERROR);

        attribute->dataAvailable = (bool) false;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    if (!dataStore->persistency)
    {
        dataStore->dataAvailable = (bool) false;
    }

    return XME_STATUS_SUCCESS;
}

bool
xme_core_dataHandler_database_isDataStoreDefined
(
    xme_core_dataManager_dataPacketId_t const dataStoreID
)
{
    uint16_t numberOfDefinedDataStores;
    xme_core_dataHandler_database_dataStore_t* dataStore;
    xme_status_t status;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // The database is created using table index as data packet idenfier. 
    numberOfDefinedDataStores = XME_HAL_TABLE_ITEM_COUNT(xme_core_dataHandler_database->dataStores);

    // If the data packet identifier is greated than the number of elements in the table,
    // we ensure that the data packet is not defined. 
    // Assumption: we never remove from the table any element. So we can safely access to the index directly. 
    XME_CHECK((uint32_t) dataStoreID <= numberOfDefinedDataStores, (bool) false);
    
    status = getDataStore(dataStoreID, &dataStore);
    XME_CHECK(XME_STATUS_SUCCESS == status, (bool) false);

    return dataStore->configured;
}

bool
xme_core_dataHandler_database_isReadLocked
(
    xme_core_dataManager_dataPacketId_t const dataStoreID
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;
    xme_status_t status;
    
    XME_ASSERT(NULL != xme_core_dataHandler_database);

    status = getDataStore(dataStoreID, &dataStore);
    XME_CHECK(XME_STATUS_SUCCESS == status, (bool) false);

    return dataStore->readLock;
}

bool
xme_core_dataHandler_database_isWriteLocked
(
    xme_core_dataManager_dataPacketId_t const dataStoreID
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;
    xme_status_t status;
    
    XME_ASSERT(NULL != xme_core_dataHandler_database);

    status = getDataStore(dataStoreID, &dataStore);
    XME_CHECK(XME_STATUS_SUCCESS == status, (bool) false);

    return dataStore->writeLock;
}

xme_status_t
xme_core_dataHandler_database_lockWrite
(
    xme_core_dataManager_dataPacketId_t dataStoreID
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);

    return setLockWrite(dataStore, (bool) true);
}

xme_status_t
xme_core_dataHandler_database_unlockWrite
(
    xme_core_dataManager_dataPacketId_t dataStoreID
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);
    
    return setLockWrite(dataStore, (bool) false);
}

xme_status_t
xme_core_dataHandler_database_lockRead
(
    xme_core_dataManager_dataPacketId_t dataStoreID
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);

    return setLockRead(dataStore, (bool) true);
}

xme_status_t
xme_core_dataHandler_database_unlockRead
(
    xme_core_dataManager_dataPacketId_t dataStoreID
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);
    
    return setLockRead(dataStore, (bool) false);
}

bool 
xme_core_dataHandler_database_isPersistent
(
    xme_core_dataManager_dataPacketId_t dataStoreID
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), (bool) false);

    return dataStore->persistency;
}

bool 
xme_core_dataHandler_database_isQueue
(
    xme_core_dataManager_dataPacketId_t dataStoreID
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), (bool) false);

    return dataStore->queueSize > 1;
}

xme_status_t
xme_core_dataHandler_database_advanceWriteQueue
(
    xme_core_dataManager_dataPacketId_t dataStoreID
)
{
    uint32_t currentCBWritePosition;
    xme_status_t status;
    bool valueForCB = (bool) false;
    xme_core_dataHandler_database_dataStore_t* dataStore;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information. 
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);

    // Perform a push operation with the value set to false.
    status = xme_hal_circularBuffer_pushBack(dataStore->circularBuffer, &valueForCB);
    XME_CHECK(XME_STATUS_SUCCESS == status, dataStore->statusOnOverwrite);

    status = xme_hal_circularBuffer_getHeadPosition(dataStore->circularBuffer, &currentCBWritePosition);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    // Establish this write position for write operations. 
    dataStore->currentCBWritePosition = currentCBWritePosition;

    dataStore->dataAvailable = (dataStore->circularBuffer->count > 0);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_database_advanceReadQueue
(
    xme_core_dataManager_dataPacketId_t dataStoreID
)
{
    uint32_t currentCBReadPosition;
    xme_status_t status;
    bool valueFromCB;
    xme_core_dataHandler_database_dataStore_t* dataStore;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information. 
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);

    // Perform a pop operation and check that the value is set to false.
    status = xme_hal_circularBuffer_popFront(dataStore->circularBuffer, &valueFromCB);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    XME_ASSERT((bool) false == valueFromCB);

    status = xme_hal_circularBuffer_getTailPosition(dataStore->circularBuffer, &currentCBReadPosition);
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);

    // Establish this read position for read operations. 
    dataStore->currentCBReadPosition = currentCBReadPosition;

    dataStore->dataAvailable = (dataStore->circularBuffer->count > 0);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_database_getDataAvailability
(
    xme_core_dataManager_dataPacketId_t dataStoreID,
    uint8_t* const dataAvailability
)
{
    xme_core_dataHandler_database_dataStore_t* dataStore;
    bool attributeAvailability;
    xme_status_t status;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Get the data packet information. 
    XME_CHECK(XME_STATUS_SUCCESS == getDataStore(dataStoreID, &dataStore), XME_STATUS_NOT_FOUND);
    status = getAttributeAvailability(dataStore, &attributeAvailability);
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    if (!dataStore->dataAvailable && !attributeAvailability)
    {
        *dataAvailability = 0U;
        return XME_STATUS_SUCCESS;
    }

    if (dataStore->queueSize > 1)
    {
        *dataAvailability = dataStore->circularBuffer->count;
    }
    else
    {
        *dataAvailability = 1U;
    }

    return XME_STATUS_SUCCESS;
}

