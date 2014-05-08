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
 * $Id: dataHandler.c 7844 2014-03-14 14:11:49Z ruiz $
 */

/**
 * \file
 *         Data Handler.
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"
#include "xme/core/dataHandler/include/dataHandlerTestProbe.h"

#include "xme/core/dataHandler/internal/databaseBuilder.h"
#include "xme/core/dataHandler/internal/database.h"

#include "xme/core/broker/include/brokerDataManagerInterface.h"

#include "xme/core/log.h"

#include "xme/hal/include/mem.h"

#ifdef XME_MULTITHREAD
#include "xme/hal/include/sync.h"
#include "xme/hal/include/tls.h"
#endif // #ifdef XME_MULTITHREAD

#include <inttypes.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
#ifdef XME_MULTITHREAD
/**
 * \brief Data Handler thread self-locking counter.
 */
typedef uint8_t xme_core_dataHandler_threadLockCount_t;
#endif // #ifdef XME_MULTITHREAD

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \brief Determines if the data handler is already initialized.
 */
static bool initialized = false;

/**
 * \brief This is the active database builder.
 * \details This database builder will be active during the whole lifecycle of 
 *          the data handler.
 */
static xme_core_dataHandler_databaseBuilder_t* builder = NULL;

#ifdef XME_MULTITHREAD
/**
 * \brief Thread-local storage handle for locking of Data Handler.
 */
static xme_hal_tls_handle_t xme_core_dataHandler_tlsHandle = XME_HAL_TLS_INVALID_TLS_HANDLE;

/**
 * \brief Critical section handle for locking of Data Handler.
 */
static xme_hal_sync_criticalSectionHandle_t xme_core_dataHandler_criticalSectionHandle = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;
#endif // #ifdef XME_MULTITHREAD

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

#ifdef XME_MULTITHREAD // #ifdef XME_MULTITHREAD

/**
 * \brief Returns a pointer to the thread-specific lock counter.
 *
 * \return Pointer to the thread-specific lock counter for the calling thread.
 */
static xme_core_dataHandler_threadLockCount_t*
xme_core_dataHandler_getThreadLock(void);

/**
 * \brief Blocks until the calling thread is granted access to the Data Handler.
 *
 * \details The same thread may call this function multiple times, in which
 *          case the seubsequent requests are immediately granted. However, a
 *          matching number of calls to xme_core_dataHandler_leaveCriticalSection()
 *          need to follow in this case.
 *
 * \see xme_core_dataHandler_leaveCriticalSection()
 */
void
xme_core_dataHandler_enterCriticalSection(void);

/**
 * \brief Frees the Data handler lock granted to the calling thread by a
 *        previous call to xme_core_dataHandler_enterCriticalSection().
 *
 * \details If xme_core_dataHandler_enterCriticalSection() has been called
 *          multiple times, a matching number of calls to this function
 *          needs to be made to release the calling thread's lock.
 *
 * \see xme_core_dataHandler_enterCriticalSection()
 */
void
xme_core_dataHandler_leaveCriticalSection(void);

#endif // #ifdef XME_MULTITHREAD

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t 
xme_core_dataHandler_init(void)
{
    xme_core_dataManager_memoryRegionID_t defaultMemoryRegionID;
    xme_status_t status;

    // If the data handler is already initialized, just return success.
    XME_CHECK(!initialized, XME_STATUS_SUCCESS);

    // Create a builder to define new data packets. 
    // The build process will be automatically configured as soon as the first read/write takes places. 
    builder = xme_core_dataHandler_databaseBuilder_create();
    XME_ASSERT(NULL != builder);

    // Temporary solution for creating at least the default memory region. 
    status = xme_core_dataHandler_createMemoryRegion(&defaultMemoryRegionID);
    XME_ASSERT(XME_STATUS_SUCCESS == status);
    XME_ASSERT(((xme_core_dataManager_memoryRegionID_t)XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT) == defaultMemoryRegionID);

    initialized = true;

#ifdef XME_MULTITHREAD
    {
        xme_status_t status;

        status = xme_hal_sync_init();
        XME_ASSERT(XME_STATUS_SUCCESS == status);
        status = xme_hal_tls_init();
        XME_ASSERT(XME_STATUS_SUCCESS == status);

        xme_core_dataHandler_tlsHandle = xme_hal_tls_alloc(sizeof(xme_core_dataHandler_threadLockCount_t));
        XME_ASSERT(XME_HAL_TLS_INVALID_TLS_HANDLE != xme_core_dataHandler_tlsHandle);

        xme_core_dataHandler_criticalSectionHandle = xme_hal_sync_createCriticalSection();
        XME_ASSERT(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != xme_core_dataHandler_criticalSectionHandle);
    }
#endif // #ifdef XME_MULTITHREAD

    return XME_STATUS_SUCCESS;
}

void
xme_core_dataHandler_fini(void)
{
    xme_status_t status;

    if (!initialized) return;

    // Release the builder.
    status = xme_core_dataHandler_databaseBuilder_destroy(builder);
    if (XME_STATUS_SUCCESS != status)
    {
        XME_LOG(XME_LOG_WARNING, "[DataHandler] Cannot destroy the database builder (status=%d)\n", status);
    }
    builder = NULL;

    // Destroy database. 
    xme_core_dataHandler_database_fini();

    initialized = false;

#ifdef XME_MULTITHREAD
    xme_hal_sync_destroyCriticalSection(xme_core_dataHandler_criticalSectionHandle);
    xme_core_dataHandler_criticalSectionHandle = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;

    xme_hal_tls_free(xme_core_dataHandler_tlsHandle);
    xme_core_dataHandler_tlsHandle = XME_HAL_TLS_INVALID_TLS_HANDLE;

    xme_hal_tls_fini();
    xme_hal_sync_fini();
#endif // #ifdef XME_MULTITHREAD
}

xme_status_t
xme_core_dataHandler_createMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
)
{
    xme_core_dataHandler_databaseBuilder_t* activeBuilder;
    XME_CHECK(NULL != memoryRegionID, XME_STATUS_INVALID_PARAMETER);

    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != builder));
    activeBuilder = builder->createGenericMemoryRegion(memoryRegionID);
    XME_CHECK(NULL != activeBuilder, XME_STATUS_INTERNAL_ERROR);

    XME_CHECK(((uint32_t)XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID) != *memoryRegionID, XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setNumberOfDatabases
(
    uint16_t numberOfCopies
)
{
    return xme_core_dataHandler_setNumberOfCopiesInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT, numberOfCopies);
}

xme_status_t
xme_core_dataHandler_setNumberOfCopiesInMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    uint16_t numberOfCopies
)
{
    xme_core_dataHandler_databaseBuilder_t* activeBuilder;

    XME_CHECK(((xme_core_dataManager_memoryRegionID_t)XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID) != memoryRegionID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0 != numberOfCopies, XME_STATUS_INVALID_PARAMETER);

    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != builder));

    activeBuilder = builder->setNumberOfMemoryRegionCopies(memoryRegionID, numberOfCopies);
    XME_CHECK(NULL != activeBuilder, XME_STATUS_INVALID_HANDLE);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setDatabaseSize
(
    size_t sizeInBytes
)
{
    return xme_core_dataHandler_setDatabaseSizeInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT, sizeInBytes);
}

xme_status_t
xme_core_dataHandler_setDatabaseSizeInMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    size_t sizeInBytes
)
{
    xme_core_dataHandler_databaseBuilder_t* activeBuilder;

    XME_CHECK(((xme_core_dataManager_memoryRegionID_t)XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID) != memoryRegionID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0 != sizeInBytes, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(NULL != builder, XME_STATUS_INVALID_CONFIGURATION);

    activeBuilder = builder->setMemoryRegionSize(memoryRegionID, sizeInBytes);
    XME_CHECK(NULL != activeBuilder, XME_STATUS_INVALID_HANDLE);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_createDataPacket
(
    size_t dataPacketSize,
    xme_core_dataManager_dataPacketId_t* dataPacketID
)
{
    xme_core_dataHandler_databaseBuilder_t* activeBuilder;

    XME_CHECK(NULL != dataPacketID, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(0U != dataPacketSize, XME_STATUS_INVALID_PARAMETER);

    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(initialized));
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != builder));

    activeBuilder = builder->createDataStore(dataPacketSize, dataPacketID);
    XME_CHECK(NULL != activeBuilder, XME_STATUS_INTERNAL_ERROR);

    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != *dataPacketID, XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setDataPacketPersistency
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    bool persistency
)
{
    xme_core_dataHandler_databaseBuilder_t* activeBuilder;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);

    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(initialized));
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != builder));
    activeBuilder = builder->setPersistency(dataPacketID, persistency);

    XME_CHECK(NULL != activeBuilder, XME_STATUS_INVALID_HANDLE);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setDataPacketPersistent
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    return xme_core_dataHandler_setDataPacketPersistency(dataPacketID, (bool) true);
}

xme_status_t
xme_core_dataHandler_setDataPacketOverwrite
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    bool overwrite,
    xme_status_t statusOnOverwrite,
    bool displayWarning
)
{
    xme_core_dataHandler_databaseBuilder_t* activeBuilder;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);

    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(initialized));
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != builder));
    activeBuilder = builder->setOverwrite(dataPacketID, overwrite, statusOnOverwrite, displayWarning);

    XME_CHECK(NULL != activeBuilder, XME_STATUS_INVALID_HANDLE);

    return XME_STATUS_SUCCESS;

}

xme_status_t
xme_core_dataHandler_setDataPacketQueueSize
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t queueSize
)
{
    xme_core_dataHandler_databaseBuilder_t* activeBuilder;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0U != queueSize, XME_STATUS_INVALID_PARAMETER);

    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(initialized));
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != builder));

    activeBuilder = builder->setQueueSize(dataPacketID, queueSize);
    XME_CHECK(NULL != activeBuilder, XME_STATUS_INVALID_HANDLE);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_createAttribute
(
    size_t attributeSize,
    uint32_t attributeKey, // or either xme_core_attribute_key_t key,
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    xme_core_dataHandler_databaseBuilder_t* activeBuilder;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != attributeKey, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(0U != attributeSize, XME_STATUS_INVALID_PARAMETER);

    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(initialized));
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != builder));

    activeBuilder = builder->createAttribute(dataPacketID, attributeKey, attributeSize);
    XME_CHECK(NULL != activeBuilder, XME_STATUS_INVALID_HANDLE);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_configure(void)
{
    xme_core_dataHandler_database_t* database;

    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(initialized));
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(NULL != builder));

    database = builder->build();

    XME_CHECK(NULL != database, XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    const void* const buffer,
    size_t bufferSizeInBytes
)
{
    xme_status_t status;
    uint32_t writtenBytes;
    
    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0U != bufferSizeInBytes, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != buffer, XME_STATUS_INVALID_PARAMETER);

    // Call the generic function for getting writting the data. 
    status = xme_core_dataHandler_database_writeData(
        dataPacketID,
        0U, // offset in bytes. 
        buffer,
        bufferSizeInBytes,
        &writtenBytes);

    XME_CHECK(XME_STATUS_SUCCESS == status, status);

#if 0
    XME_CHECK_MSG
    (
        writtenBytes == bufferSizeInBytes,
        XME_STATUS_SUCCESS,
        XME_LOG_WARNING,
        "[DataHandler] Written bytes do not match with provided buffer for data packet %d (bufferSize=%d - writtenBytes=%d).\n",
        dataPacketID,
        bufferSizeInBytes,
        writtenBytes
    );
#endif

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_writeAttribute
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t attributeKey,
    const void* const buffer,
    size_t bufferSizeInBytes
)
{
    xme_status_t status;
    uint32_t writtenBytes;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0U != bufferSizeInBytes, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != buffer, XME_STATUS_INVALID_PARAMETER);

    // Call the generic function for getting writting the data. 
    status = xme_core_dataHandler_database_writeAttribute(
        dataPacketID,
        attributeKey,
        0U, // offset in bytes. 
        buffer,
        bufferSizeInBytes,
        &writtenBytes);

    XME_CHECK(XME_STATUS_SUCCESS == status, status);

#if 0
    XME_CHECK_MSG
    (
        writtenBytes == bufferSizeInBytes,
        XME_STATUS_SUCCESS,
        XME_LOG_WARNING,
        "[DataHandler] Written bytes do not match with provided buffer for data packet %d and attribute key %d (bufferSize=%d - writtenBytes=%d).\n",
        dataPacketID,
        attributeKey,
        bufferSizeInBytes,
        writtenBytes
    );
#endif

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    void * const buffer,
    size_t bufferSizeInBytes,
    uint32_t * const readBytes
)
{
    xme_status_t status;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(0U != bufferSizeInBytes, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != buffer, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != readBytes, XME_STATUS_INVALID_PARAMETER);

    // Call the generic function for getting writting the data. 
    status = xme_core_dataHandler_database_readData(
        dataPacketID,
        0U, // offset in bytes. 
        buffer,
        bufferSizeInBytes,
        readBytes);

    if (XME_STATUS_SUCCESS != status)
    {
        XME_LOG
        (
            XME_LOG_DEBUG,
            "[DataHandler] Read operation from data packet failed (dataPacketID=%d). status=%d\n",
            dataPacketID,
            status
        );
    }

    return status;
}

xme_status_t
xme_core_dataHandler_readAttribute
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t attributeKey,
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

    // Call the generic function for getting writting the data. 
    status = xme_core_dataHandler_database_readAttribute(
        dataPacketID,
        attributeKey,
        0U, // offset in bytes. 
        buffer,
        bufferSizeInBytes,
        readBytes);

    if (XME_STATUS_SUCCESS != status)
    {
        XME_LOG
        (
            XME_LOG_DEBUG,
            "[DataHandler] Read operation from data packet failed (dataPacketID=%d, attributeKey=%d). status=%d\n",
            dataPacketID,
            attributeKey,
            status
        );
    }

    return status;
}

xme_status_t 
xme_core_dataHandler_startWriteOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    xme_status_t status;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(true == xme_core_dataHandler_database_isDataStoreDefined(dataPacketID), XME_STATUS_INVALID_HANDLE);

    XME_LOG(XME_LOG_DEBUG, "[DataHandler] StartWriteOperation for data packet %d\n", dataPacketID);

#ifdef XME_MULTITHREAD
    XME_LOG(XME_LOG_VERBOSE, "[DataHandler] Entering critical section for data packet %d in startWriteOperation ... ", dataPacketID);
    xme_core_dataHandler_enterCriticalSection();
    XME_LOG(XME_LOG_VERBOSE, "done\n");
#endif // #ifdef XME_MULTITHREAD

    if (!xme_core_dataHandler_database_isWriteLocked(dataPacketID))
    {
        status = xme_core_dataHandler_database_lockRead(dataPacketID);
        XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_WARNING, "[DataHandler] Cannot lock read operation for CompleteWriteOperation in data packet %d.\n", dataPacketID);
        return XME_STATUS_SUCCESS;
    }
    else
    {
        XME_LOG(XME_LOG_WARNING, "[DataHandler] Cannot start a write operation on data packet %d while still write locked.\n", dataPacketID);

        return XME_STATUS_PERMISSION_DENIED;
    }
}

xme_status_t 
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    xme_status_t status;
    uint8_t dataAvailability;

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(true == xme_core_dataHandler_database_isDataStoreDefined(dataPacketID), XME_STATUS_INVALID_HANDLE);

    XME_LOG(XME_LOG_DEBUG, "[DataHandler] CompleteWriteOperation for data packet %d\n", dataPacketID);

    // If the data packet is locked, this means a startReadOperation and the consumption of the data. 
    if (xme_core_dataHandler_database_isReadLocked(dataPacketID))
    {
        
        status = xme_core_dataHandler_database_unlockRead(dataPacketID);
#ifdef XME_MULTITHREAD
        XME_CHECK_MSG_REC
        (
            XME_STATUS_SUCCESS == status, 
            XME_STATUS_INVALID_CONFIGURATION,
            {
                xme_core_dataHandler_leaveCriticalSection();
            },
            XME_LOG_WARNING, 
            "[DataHandler] Cannot unlock read operation for data packet %d.\n", dataPacketID
        );
#else // #ifdef XME_MULTITHREAD
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status, 
            XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_WARNING, 
            "[DataHandler] Cannot unlock read operation for data packet %d.\n", dataPacketID
        );
#endif // #ifdef XME_MULTITHREAD

        // Call data availability change. 
        if (xme_core_dataHandler_database_isQueue(dataPacketID))
        {
            status = xme_core_dataHandler_database_advanceWriteQueue(dataPacketID);
#ifdef XME_MULTITHREAD
            XME_CHECK_MSG_REC
            (
                XME_STATUS_SUCCESS == status,
                XME_STATUS_INVALID_CONFIGURATION,
                {
                    xme_core_dataHandler_leaveCriticalSection();
                },
                XME_LOG_DEBUG,
                "Cannot advance a queue position for data packet %d\n", dataPacketID
            );
#else // #ifdef XME_MULTITHREAD
            XME_CHECK_MSG
            (
                XME_STATUS_SUCCESS == status,
                status,
                XME_LOG_DEBUG,
                "Cannot advance a queue position for data packet %d\n", dataPacketID
            );
#endif // #ifdef XME_MULTITHREAD
        }

        // Get current data availability. 
        status = xme_core_dataHandler_database_getDataAvailability(dataPacketID, &dataAvailability);
#ifdef XME_MULTITHREAD
        XME_CHECK_MSG_REC
        (
            XME_STATUS_SUCCESS == status,
            XME_STATUS_INVALID_CONFIGURATION,
            {
                xme_core_dataHandler_leaveCriticalSection();
            },
            XME_LOG_WARNING,
            "[DataHandler] Cannot obtain data availability for completing write operation for data packet %d.\n", dataPacketID
        );
#else // #ifdef XME_MULTITHREAD
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status,
            XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_WARNING,
            "[DataHandler] Cannot obtain data availability for completing write operation for data packet %d.\n", dataPacketID
        );
#endif // #ifdef XME_MULTITHREAD

        status = xme_core_broker_dataAvailabilityChange(dataPacketID, dataAvailability);
#ifdef XME_MULTITHREAD
        XME_CHECK_MSG_REC
        (
            XME_STATUS_SUCCESS == status || XME_STATUS_NOT_FOUND == status,
            status,
            {
                xme_core_dataHandler_leaveCriticalSection();
            },
            XME_LOG_DEBUG,
            "[DataHandler] Cannot change data availability for completing write operation of data packet %d to %d\n", dataPacketID, dataAvailability
        );
#else // #ifdef XME_MULTITHREAD
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status || XME_STATUS_NOT_FOUND == status,
            status,
            XME_LOG_DEBUG,
            "[DataHandler] Cannot change data availability for completing write operation of data packet %d to %d\n", dataPacketID, dataAvailability
        );
#endif // #ifdef XME_MULTITHREAD

#ifdef XME_MULTITHREAD
        // Leave critical section
        XME_LOG(XME_LOG_VERBOSE, "[DataHandler] Leaving critical section for data packet %d in completeWriteOperation ... ", dataPacketID);
        xme_core_dataHandler_leaveCriticalSection();
        XME_LOG(XME_LOG_VERBOSE, "done\n");
#endif // #ifdef XME_MULTITHREAD

        if (XME_STATUS_NOT_FOUND == status && !xme_core_dataHandler_database_isQueue(dataPacketID))
        {
            // No subscriber for the current port. 
            status = xme_core_dataHandler_startReadOperation(dataPacketID);
            XME_CHECK_MSG
            (
                XME_STATUS_SUCCESS == status, 
                XME_STATUS_INTERNAL_ERROR, 
                XME_LOG_DEBUG, 
                "[DataHandler] Cannot Start Read Operation for the output data packet %d without subscribers.\n",
                dataPacketID
            );
            status = xme_core_dataHandler_completeReadOperation(dataPacketID);
            XME_CHECK_MSG
            (
                XME_STATUS_SUCCESS == status, 
                XME_STATUS_INTERNAL_ERROR, 
                XME_LOG_DEBUG, 
                "[DataHandler] Cannot Complete Read Operation for the output data packet %d without subscribers (effective data consumption).\n",
                dataPacketID
            );
        }
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_startReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(true == xme_core_dataHandler_database_isDataStoreDefined(dataPacketID), XME_STATUS_INVALID_HANDLE);

    XME_LOG(XME_LOG_DEBUG, "[DataHandler] StartReadOperation for data packet %d\n", dataPacketID);

#ifdef XME_MULTITHREAD
    XME_LOG(XME_LOG_VERBOSE, "[DataHandler] Entering critical section for data packet %d in startReadOperation ...", dataPacketID);
    xme_core_dataHandler_enterCriticalSection();
    XME_LOG(XME_LOG_VERBOSE, "done\n");
#endif // #ifdef XME_MULTITHREAD

    if (!xme_core_dataHandler_database_isReadLocked(dataPacketID))
    {
        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_dataHandler_database_lockWrite(dataPacketID), XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_WARNING, "[DataHandler] Cannot lock write operation for CompleteReadOperation for data packet %d.\n", dataPacketID);
        return XME_STATUS_SUCCESS;
    }
    else
    {
        XME_LOG(XME_LOG_WARNING, "[DataHandler] Cannot start a read operation on data packet %d while still read locked.\n", dataPacketID);

        return XME_STATUS_PERMISSION_DENIED;
    }

}

xme_status_t 
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    xme_status_t status;
    uint8_t dataAvailability; 

    XME_CHECK(((xme_core_dataManager_dataPacketId_t)XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) != dataPacketID, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(true == xme_core_dataHandler_database_isDataStoreDefined(dataPacketID), XME_STATUS_INVALID_HANDLE);

    XME_LOG(XME_LOG_DEBUG, "[DataHandler] CompleteReadOperation for data packet %d\n", dataPacketID);

    // If the data packet is locked, this means a startReadOperation and the consumption of the data. 
    if (xme_core_dataHandler_database_isWriteLocked(dataPacketID))
    {
        // Get current data availability. 
        status = xme_core_dataHandler_database_getDataAvailability(dataPacketID, &dataAvailability);
#ifdef XME_MULTITHREAD
        XME_CHECK_MSG_REC
        (
            XME_STATUS_SUCCESS == status, 
            XME_STATUS_INVALID_CONFIGURATION,
            {
                xme_core_dataHandler_leaveCriticalSection();
            },
            XME_LOG_WARNING, "[DataHandler] Cannot obtain data availability for complete read operation on data packet %d.\n", dataPacketID
        );
#else // #ifdef XME_MULTITHREAD
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status, 
            XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_WARNING, "[DataHandler] Cannot obtain data availability for complete write operation on data packet %d.\n", dataPacketID
        );
#endif // #ifdef XME_MULTITHREAD

        if (dataAvailability > 0)
        {
            // if it was data to read
            if (xme_core_dataHandler_database_isQueue(dataPacketID))
            {
                status = xme_core_dataHandler_database_advanceReadQueue(dataPacketID);
                XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION,
                    XME_LOG_WARNING, "[DataHandler] Cannot advance read position in the queue for data packet %d.\n", dataPacketID);
            }
            else if (!xme_core_dataHandler_database_isPersistent(dataPacketID))
            {
                // Set all values in data to zero.
                status = xme_core_dataHandler_database_resetData(dataPacketID);
                XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION, 
                    XME_LOG_WARNING, "[DataHandler] Cannot reset data on data packet %d.\n", dataPacketID);
            }

            // Get current data availability after consumption. 
            status = xme_core_dataHandler_database_getDataAvailability(dataPacketID, &dataAvailability);
#ifdef XME_MULTITHREAD
            XME_CHECK_MSG_REC
            (
                XME_STATUS_SUCCESS == status, 
                XME_STATUS_INVALID_CONFIGURATION,
                {
                    xme_core_dataHandler_leaveCriticalSection();
                },
                XME_LOG_WARNING, "[DataHandler] Cannot obtain data availability for completing read operation on data packet %d.\n", dataPacketID
            );
#else // #ifdef XME_MULTITHREAD
            XME_CHECK_MSG
            (   
                XME_STATUS_SUCCESS == status, 
                XME_STATUS_INVALID_CONFIGURATION,
                XME_LOG_WARNING, "[DataHandler] Cannot obtain data availability for completing read operation on data packet %d.\n", dataPacketID
            );
#endif // #ifdef XME_MULTITHREAD
        }

        // Change data availability.
        status = xme_core_broker_dataAvailabilityChange(dataPacketID, dataAvailability);
#ifdef XME_MULTITHREAD
        XME_CHECK_MSG_REC
        (
            XME_STATUS_SUCCESS == status || XME_STATUS_NOT_FOUND == status, 
            status,
            {
                xme_core_dataHandler_leaveCriticalSection();
            },
            XME_LOG_DEBUG, 
            "[DataHandler] Cannot update data availability for completing read operation on data packet %d and new value to %d.\n", dataPacketID, dataAvailability
        );
#else // #ifdef XME_MULTITHREAD
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status || XME_STATUS_NOT_FOUND == status, 
            status,
            XME_LOG_DEBUG, 
            "[DataHandler] Cannot update data availability for completing read operation on data packet %d and new value to %d.\n", dataPacketID, dataAvailability
        );
#endif // #ifdef XME_MULTITHREAD

        status = xme_core_dataHandler_database_unlockWrite(dataPacketID);
        XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_WARNING, "[DataHandler] Cannot unlock the write operation on complete write operation for data packet %d.\n", dataPacketID);

#ifdef XME_MULTITHREAD
        XME_LOG(XME_LOG_VERBOSE, "[DataHandler] Leaving critical section for data packet %d in completeReadOperation ... ", dataPacketID);
        xme_core_dataHandler_leaveCriticalSection();
        XME_LOG(XME_LOG_VERBOSE, "done\n");
#endif // #ifdef XME_MULTITHREAD

        return XME_STATUS_SUCCESS;
    }
    else
    {
        XME_LOG(XME_LOG_WARNING, "[DataHandler] Cannot complete a read operation on data packet %d because the data packet is not write locked.\n", dataPacketID);
        return XME_STATUS_PERMISSION_DENIED;
    }
}

xme_status_t 
xme_core_dataHandler_transferData
(
    xme_core_dataManager_dataPacketId_t sourceDataPacketID,
    xme_core_dataManager_dataPacketId_t destDataPacketID
)
{
    // Do the transfer. 
    xme_status_t status;
    xme_status_t returnStatus;

    status = xme_core_dataHandler_startWriteOperation(destDataPacketID);
    XME_CHECK_MSG
    (
        XME_STATUS_SUCCESS == status, 
        XME_STATUS_INVALID_CONFIGURATION,
        XME_LOG_WARNING, 
        "[DataHandler] Cannot start write operation during the transfer for data packets %d -> %d.\n", sourceDataPacketID, destDataPacketID
    );

    // 2. Perform the transfer operation (copy of all data and attributes).
    returnStatus = xme_core_dataHandler_database_transfer
            (
                sourceDataPacketID,
                destDataPacketID
            );

    status = xme_core_dataHandler_completeWriteOperation(destDataPacketID);
    XME_CHECK_MSG
    (
        XME_STATUS_SUCCESS == status, 
        XME_STATUS_INVALID_CONFIGURATION,
        XME_LOG_WARNING, 
        "[DataHandler] Cannot complete write operation during the transfer for data packets %d -> %d.\n", sourceDataPacketID, destDataPacketID
    );

    // 3. Check the return value. 
    XME_CHECK_MSG
    (
        XME_STATUS_SUCCESS == returnStatus,
        returnStatus,
        XME_LOG_DEBUG,
        "[DataHandler] Cannot perform the transfer between %d and %d.\n",
        sourceDataPacketID,
        destDataPacketID
    );

    return XME_STATUS_SUCCESS;
}

/******************************************************************************/

#ifdef XME_MULTITHREAD

static xme_core_dataHandler_threadLockCount_t*
xme_core_dataHandler_getThreadLock(void)
{
    xme_core_dataHandler_threadLockCount_t* tlc;

    XME_ASSERT_RVAL(XME_HAL_TLS_INVALID_TLS_HANDLE != xme_core_dataHandler_tlsHandle, NULL);

    tlc = (xme_core_dataHandler_threadLockCount_t*) xme_hal_tls_get(xme_core_dataHandler_tlsHandle);
    XME_ASSERT_RVAL(NULL != tlc, NULL);

    return tlc;
}

void
xme_core_dataHandler_enterCriticalSection(void)
{
    xme_core_dataHandler_threadLockCount_t* tlc = xme_core_dataHandler_getThreadLock();
    XME_ASSERT_NORVAL(NULL != tlc);

    if (0U == (*tlc))
    {
        // This thread has not yet locked the critical section
        XME_LOG(XME_LOG_DEBUG, "[DataHandler] Entering the critical section ... ");
        xme_hal_sync_enterCriticalSection(xme_core_dataHandler_criticalSectionHandle);
        XME_LOG(XME_LOG_DEBUG, "OK\n");
    }
    else
    {
        XME_ASSERT_NORVAL((*tlc) < (xme_core_dataHandler_threadLockCount_t) -1);
    }
    (*tlc)++;
}

void
xme_core_dataHandler_leaveCriticalSection(void)
{
    xme_core_dataHandler_threadLockCount_t* tlc = xme_core_dataHandler_getThreadLock();
    XME_ASSERT_NORVAL(NULL != tlc);

    XME_ASSERT_NORVAL((*tlc) > 0U);

    (*tlc)--;
    if (0U == (*tlc))
    {
        // This thread is about to unlock the critical section
        xme_hal_sync_leaveCriticalSection(xme_core_dataHandler_criticalSectionHandle);
        XME_LOG(XME_LOG_DEBUG, "[DataHandler] Left the critical section\n");
    }
}

#endif // #ifdef XME_MULTITHREAD
