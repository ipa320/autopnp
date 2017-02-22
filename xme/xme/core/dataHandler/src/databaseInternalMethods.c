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
 * $Id: databaseInternalMethods.c 7835 2014-03-14 11:51:05Z ruiz $
 */

/**
 * \file
 *         Database Internal Methods Implementation.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/dataHandler/internal/databaseInternalMethods.h"
#include "xme/core/dataHandler/internal/databaseConfigurator.h"

#include "xme/core/log.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/mmap.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \brief The database containing all information about data packets and memory regions. 
 */
xme_core_dataHandler_database_t* xme_core_dataHandler_database =  NULL;

/**
 * \var manipulationsTable
 * \brief The variable storing the manipulations. 
 */
xme_core_dataHandler_database_manipulationTable_t manipulationsTable;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_dataHandler_databaseInternal_init(void)
{
    // This check is needed, because any database is created by the builder. 
    xme_core_dataHandler_database =  (xme_core_dataHandler_database_t*) xme_hal_mem_alloc(sizeof(xme_core_dataHandler_database_t));

    // Initialize needed internal structures. 
    XME_HAL_TABLE_INIT(xme_core_dataHandler_database->dataStores);
    XME_HAL_TABLE_INIT(xme_core_dataHandler_database->memoryRegions);

    // Establish the database as initialized but non-configured. 
    xme_core_dataHandler_database->configured = false;

    // RACE Specific.
    XME_HAL_TABLE_INIT(manipulationsTable);

    return XME_STATUS_SUCCESS;
}

void
xme_core_dataHandler_databaseInternal_fini(void)
{
    lastAssignedDataPacketID = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
    lastAssignedMemoryRegionID = XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID;

    if (NULL != xme_core_dataHandler_database)
    {
        XME_HAL_TABLE_FINI(xme_core_dataHandler_database->dataStores);
        XME_HAL_TABLE_FINI(xme_core_dataHandler_database->memoryRegions);
        xme_core_dataHandler_database->configured = false;

        xme_hal_mem_free(xme_core_dataHandler_database);
        xme_core_dataHandler_database =  NULL;
    }

    // RACE Specific.
    (void) xme_hal_table_clear(&manipulationsTable);
}


xme_status_t
createPhysicalMemoryRegion
(
    xme_core_dataHandler_database_memoryRegion_t* const memoryRegion
)
{
    xme_core_dataHandler_database_memoryRegionCopy_t* newMemoryRegionAddress;
    uint16_t memoryRegionCopiesCount;
    uint8_t i;
    void* outAddress;
    size_t memoryRegionSizeInBytes;
    xme_status_t status;
    
    XME_CHECK(NULL != memoryRegion, XME_STATUS_INVALID_PARAMETER);

    // Obtain how many memory region copies should be available. 
    memoryRegionCopiesCount = memoryRegion->numOfCopies;

    // Obtain the size of each memory region copy. 
    memoryRegionSizeInBytes = memoryRegion->sizeInBytes;

    // Create the memory region for each memory copy. 
    for (i = 0; i < memoryRegionCopiesCount; i++)
    {
        newMemoryRegionAddress = (xme_core_dataHandler_database_memoryRegionCopy_t*) xme_hal_mem_alloc(sizeof(xme_core_dataHandler_database_memoryRegionCopy_t));
        XME_CHECK(NULL != newMemoryRegionAddress, XME_STATUS_OUT_OF_RESOURCES);

        if (memoryRegion->isShared)
        {
            uint8_t protection;

            protection = XME_HAL_MMAP_PROTECTION_READ | XME_HAL_MMAP_PROTECTION_WRITE;

            status = xme_hal_mmap_mapSharedMemory(NULL, memoryRegionSizeInBytes, protection, 0U, 0U, XME_HAL_MMAP_ANONYMOUS, &outAddress);
            XME_ASSERT(XME_STATUS_SUCCESS == status);
            
        }
        else
        {
            // Just reserve dinamically the memory region. 
            outAddress = xme_hal_mem_alloc(memoryRegionSizeInBytes);
            XME_ASSERT(NULL != outAddress);
        }

        // Copy the address to the structure created on top. 
        newMemoryRegionAddress->memoryRegionCopyAddress = outAddress;

        // Add to the memory regions copies in the memory region. 
        status = xme_hal_singlyLinkedList_addItem(&memoryRegion->memoryRegionCopies, newMemoryRegionAddress);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_OUT_OF_RESOURCES);
    }

    return XME_STATUS_SUCCESS;
}

size_t
calculateMemoryRegionSize
(
    const xme_core_dataHandler_database_memoryRegion_t* const memoryRegion
)
{
    size_t memoryRegionSize = 0U;

    XME_ASSERT(NULL != memoryRegion);

    // Iterate over all datapackets declared for this xme_core_dataHandler_database-> 
    XME_HAL_TABLE_ITERATE_BEGIN
    (
        xme_core_dataHandler_database->dataStores, 
        xme_hal_table_rowHandle_t, internalHandle, 
        xme_core_dataHandler_database_dataStore_t, dataStore
    );
    {
        xme_status_t status;

        status = setDataStoreSize(dataStore);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION);

        // Add to the memory region size the size of the data store multiplied by the queue size. 
        memoryRegionSize += (dataStore->dataStoreSize * dataStore->queueSize);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return memoryRegionSize;
}

xme_status_t
setDataStoreSize
(
    xme_core_dataHandler_database_dataStore_t* const dataStore
)
{
    size_t dataStoreSize = 0U;

    XME_ASSERT(NULL != dataStore);

    // If the data store is already configured, just return success.
    if (dataStore->configured || dataStore->dataStoreSize != 0U)
    {
        return XME_STATUS_SUCCESS;
    }

    // Add to the data store size the size occupied by the topic. 
    dataStoreSize += dataStore->topicSize;

    // Iterate over the attributes. 
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(dataStore->attributes, xme_core_dataHandler_database_attribute_t, attribute);
    {
        // For each defined attribute, we store only the value size.
        dataStoreSize += attribute->attributeValueSize;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    dataStore->dataStoreSize = dataStoreSize;

    return XME_STATUS_SUCCESS;
}


size_t
calculateOptimalDatabaseSize
(
    size_t currentSize
)
{
    // FIXME: Here we can include the algorithm for calculating
    //       the optimal database size. 
    //       For simplicity reasons, we will reserve the double size
    //       of current size for plug and play components. 
    return currentSize * 1U;
}

xme_status_t
configureDataStores(void)
{
    xme_status_t status;
    size_t currentOffset;
    size_t maxMemoryRegionSize;

    // Iterate over all datapackets declared for this xme_core_dataHandler_database-> 
    XME_HAL_TABLE_ITERATE_BEGIN
    (
        xme_core_dataHandler_database->dataStores, 
        xme_hal_table_rowHandle_t, internalHandle, 
        xme_core_dataHandler_database_dataStore_t, dataStore
    );
    {
        if (dataStore->configured)
        {
            // If the data store is already initialized, just step to the next element. 
            continue;
        }

        // Calculate and establish data store size based on topic and attributes size. 
        status = setDataStoreSize(dataStore);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION);

        // Obtain the memory region associated to the current data store. 
        currentOffset = dataStore->memoryRegion->firstAvailableOffset;

        // Get the maximum database size. 
        if (dataStore->memoryRegion->sizeInBytes > 0)
        {
            maxMemoryRegionSize = dataStore->memoryRegion->sizeInBytes;
        }
        else
        {
            maxMemoryRegionSize = 0U;
        }

        // Check that the database size is at least greater than zero. 
        XME_CHECK(maxMemoryRegionSize > 0, XME_STATUS_INVALID_CONFIGURATION);

        // Check if there is sufficient free space for configuring the current data packet. 
        {
            size_t dataStoreCompleteSize = (dataStore->dataStoreSize * dataStore->queueSize);

            if (maxMemoryRegionSize < (currentOffset + dataStoreCompleteSize))
            {
                status = memoryRegionReallocate(dataStore->memoryRegion, currentOffset + dataStoreCompleteSize);
                XME_CHECK_MSG
                (
                    XME_STATUS_SUCCESS == status, 
                    XME_STATUS_OUT_OF_RESOURCES,
                    XME_LOG_WARNING, 
                    "[Database] Cannot reserve more memory for the current data handler. .\n."
                );
                //XME_CHECK_MSG(maxMemoryRegionSize > (currentOffset + dataStoreCompleteSize), XME_STATUS_OUT_OF_RESOURCES, XME_LOG_WARNING, "[database] Memory reserved by data handler not sufficient to create further ports. Consider increasing reserved memory.\n.");
            }
        }

        // Calculate the offset for the topic inside in the data store.
        // Note that for queues, the data is stored in a different way. 
        dataStore->topicOffset = currentOffset;

        // Add to the current offset the topic size. This means to advance the offset to the last 
        // position of the topic inside the data store. 
        currentOffset += dataStore->topicSize;

        // Calculate the offset associated to the attributes. 
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
        (
            dataStore->attributes, 
            xme_core_dataHandler_database_attribute_t, 
            attribute
        );
        {
            // Assign the offset to the attribute. 
            attribute->attributeOffset = currentOffset;

            // Note that we do not need to add the key, because it 
            // is not stored in the xme_core_dataHandler_database-> 
            currentOffset += attribute->attributeValueSize;
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

        // The offset was configured for the first data store. 
        // In case we have queues defined, configure the circular buffer. 
        if (dataStore->queueSize > 1U)
        {
            // Add to the current offset the remainins elements in the queue. 
            currentOffset += (dataStore->dataStoreSize * (dataStore->queueSize - 1));

            // FIXME: This is a temporary solution. The circular buffer is not directly pointing to the data.
            dataStore->circularBuffer = (xme_hal_circularBuffer_t*) xme_hal_mem_alloc(sizeof(xme_hal_circularBuffer_t)); 
            XME_CHECK(NULL != dataStore->circularBuffer, XME_STATUS_OUT_OF_RESOURCES);

            XME_CHECK
            (
                XME_STATUS_SUCCESS ==
                xme_hal_circularBuffer_init(dataStore->circularBuffer, dataStore->queueSize, sizeof(bool), dataStore->overwrite),
                XME_STATUS_INTERNAL_ERROR
            );
            dataStore->currentCBWritePosition = 0U;
            dataStore->currentCBReadPosition = 0U;
        }

        dataStore->activeDatabaseIndex = XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER;

        // Establish the new offset in the memory region. 
        dataStore->memoryRegion->firstAvailableOffset = currentOffset;

        // RACE Specific. 
        dataStore->manipulated = false;

        // Establish as configured. 
        dataStore->configured = true;
    }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

xme_status_t
configureMemoryRegions(void)
{
    xme_status_t status;

    XME_ASSERT(NULL != xme_core_dataHandler_database);

    XME_HAL_TABLE_ITERATE_BEGIN
    (
        xme_core_dataHandler_database->memoryRegions,
        xme_hal_table_rowHandle_t, internalHandle,
        xme_core_dataHandler_database_memoryRegion_t, memoryRegion
    );
    {
        if (!(memoryRegion->configured))
        {
            // Get database size. 
            if (!(memoryRegion->sizeInBytes > 0U))
            {
                uint32_t minimumDatabaseSize;

                minimumDatabaseSize = calculateMemoryRegionSize(memoryRegion);

                // Reserve memory for plug and play data packets.
                memoryRegion->sizeInBytes = calculateOptimalDatabaseSize(minimumDatabaseSize);
            }

            // Physically create memory regions. 
            status = createPhysicalMemoryRegion(memoryRegion);
            XME_CHECK_MSG
            (
                XME_STATUS_SUCCESS == status, 
                XME_STATUS_INVALID_CONFIGURATION,
                XME_LOG_WARNING, 
                "[Database] Invalid configuration of the memory regions."
            );

            // Establish the memory region as configured. 
            memoryRegion->configured = true;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

xme_status_t
getDataStore
(
    xme_core_dataManager_dataPacketId_t dataStoreID,
    xme_core_dataHandler_database_dataStore_t** const dataStore
)
{
    xme_core_dataHandler_database_dataStore_t* temporaryDataStore;
    xme_hal_table_rowHandle_t tableHandle;
    uint16_t definedDataStores;
    
    XME_ASSERT(NULL != xme_core_dataHandler_database);

    definedDataStores = XME_HAL_TABLE_ITEM_COUNT(xme_core_dataHandler_database->dataStores);

    XME_CHECK(dataStoreID <= (uint32_t) definedDataStores, XME_STATUS_INVALID_HANDLE);

    tableHandle = (xme_hal_table_rowHandle_t) dataStoreID;
    temporaryDataStore = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_dataHandler_database->dataStores, tableHandle);
    XME_CHECK(NULL != temporaryDataStore, XME_STATUS_NOT_FOUND);
    XME_CHECK(temporaryDataStore->configured, XME_STATUS_NOT_FOUND);

    *dataStore = temporaryDataStore;
    return XME_STATUS_SUCCESS;
}

xme_status_t
setDataAvailability
(
    xme_core_dataHandler_database_dataStore_t* const dataStore,
    bool dataAvailable
)
{
    dataStore->dataAvailable = dataAvailable;
    return XME_STATUS_SUCCESS;
}

xme_status_t
getAttribute
(
    xme_core_dataHandler_database_dataStore_t* dataStore,
    uint32_t attributeKey,
    xme_core_dataHandler_database_attribute_t** const attribute
)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(dataStore->attributes, xme_core_dataHandler_database_attribute_t, attributeItem);
    {
        if (attributeItem->key == attributeKey)
        {
            *attribute = attributeItem;
            return XME_STATUS_SUCCESS;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_NOT_FOUND;
}

xme_status_t
getAttributeAvailability
(
    xme_core_dataHandler_database_dataStore_t* const dataStore,
    bool* const attributeAvailable
)
{
    // Try the transfer for all attributes as well. 
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(dataStore->attributes, xme_core_dataHandler_database_attribute_t, attribute);
    {
        if (attribute->dataAvailable)
        {
            // If we find just one with data available, we return true. 
            *attributeAvailable = true;
            return XME_STATUS_SUCCESS;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    *attributeAvailable = false;
    return XME_STATUS_SUCCESS;
}

xme_status_t
setAttributeAvailability
(
    xme_core_dataHandler_database_attribute_t* const attribute,
    bool dataAvailable
)
{
    attribute->dataAvailable = dataAvailable;
    return XME_STATUS_SUCCESS;
}

xme_status_t
setLockWrite
(
    xme_core_dataHandler_database_dataStore_t* const dataStore,
    bool writeLock
)
{
    dataStore->writeLock = writeLock;
    return XME_STATUS_SUCCESS;
}

xme_status_t
setLockRead
(
    xme_core_dataHandler_database_dataStore_t* const dataStore,
    bool readLock
)
{
    dataStore->readLock = readLock;
    return XME_STATUS_SUCCESS;
}

uint32_t
getBytesToCopy
(
    size_t dataSizeInBytes,
    size_t bufferSizeInBytes,
    uint32_t offsetInBytes
)
{
    uint32_t bytesToCopy;

    if (bufferSizeInBytes > dataSizeInBytes)
    {
        // If the target buffer is larger than the data size. 
        bytesToCopy = (uint32_t) (dataSizeInBytes - offsetInBytes);
    }
    else if (dataSizeInBytes < (bufferSizeInBytes + offsetInBytes))
    {
        // If the buffer size is smaller than the data size, or
        // if the remaining buffer size (taking into account the offset to shift)
        //    is smaller than the attribute size. 
        bytesToCopy = (uint32_t) (bufferSizeInBytes - (dataSizeInBytes - offsetInBytes));
    }
    else
    {
        // Establish the buffer size as the data to read.
        bytesToCopy = (uint32_t) bufferSizeInBytes;
    }

    return bytesToCopy;
}

xme_status_t
memoryRegionReallocate
(
    xme_core_dataHandler_database_memoryRegion_t* const memoryRegion, 
    size_t newSize
)
{
    XME_ASSERT(memoryRegion->sizeInBytes < newSize);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        memoryRegion->memoryRegionCopies, 
        xme_core_dataHandler_database_memoryRegionCopy_t, 
        memoryRegionCopy
    );
    {
        void* newAddress;
        newAddress = xme_hal_mem_realloc(memoryRegionCopy->memoryRegionCopyAddress, newSize);
        XME_CHECK(NULL != newAddress, XME_STATUS_OUT_OF_RESOURCES);

        memoryRegionCopy->memoryRegionCopyAddress = newAddress;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    memoryRegion->sizeInBytes = newSize;

    return XME_STATUS_SUCCESS;
}

void*
getAddressFromMemoryCopy
(
    xme_core_dataHandler_database_memoryRegion_t* const memoryRegion,
    xme_core_dataHandler_database_index_t index
)
{
    xme_core_dataHandler_database_memoryRegionCopy_t* memoryRegionCopy;

    XME_CHECK(NULL != memoryRegion, NULL);

    memoryRegionCopy = (xme_core_dataHandler_database_memoryRegionCopy_t*) 
        xme_hal_singlyLinkedList_itemFromIndex(&memoryRegion->memoryRegionCopies, index);
    XME_CHECK(NULL != memoryRegionCopy, NULL);

    return memoryRegionCopy->memoryRegionCopyAddress;
}

xme_status_t
checkOverwriteBeforeWrite
(
    xme_core_dataHandler_database_dataStore_t* const dataStore,
    bool dataAvailable,
    bool* const enableWriteOperation,
    bool* const isOverwrite
)
{
    // First check the queue. 
    if (dataStore->queueSize > 1)
    {
        if (dataStore->currentCBReadPosition != dataStore->currentCBWritePosition)
        {
            // Enable write operation when there is no overwrite
            *enableWriteOperation = true;
            *isOverwrite = false;
            return XME_STATUS_SUCCESS;
        }
    }

    // Check the overwrite. 
    if (dataStore->overwrite == false && 
        dataAvailable == true)
    {
        // Return at this point. 
        if (dataStore->displayWarningOnOverwrite == true)
        {
            XME_LOG
            (
                XME_LOG_WARNING, 
                "[Database] Overwrite is not allowed for data store %d. Please, check your configuration for this port.\n", 
                dataStore->dataStoreID
            );
        }

        *enableWriteOperation = false;
        return dataStore->statusOnOverwrite;
    }
    else if (dataStore->overwrite == true && 
        dataAvailable == true)
    {
        *enableWriteOperation = true;
        *isOverwrite = true;
        return XME_STATUS_SUCCESS;
    }

    *enableWriteOperation = true;
    *isOverwrite = false;
    return XME_STATUS_SUCCESS;
}

xme_status_t
checkOverwriteAfterWrite
(
    xme_core_dataHandler_database_dataStore_t* const dataStore
)
{

    // An overwrite took place (allowed). 
    // New value was stored in the database. 
    // Return at this point. 
    if (dataStore->displayWarningOnOverwrite == true)
    {
        XME_LOG
        (
            XME_LOG_WARNING, 
            "[Database] Overwrite took place for data store %d.\n", 
            dataStore->dataStoreID
        );
    }
    return dataStore->statusOnOverwrite;
}

