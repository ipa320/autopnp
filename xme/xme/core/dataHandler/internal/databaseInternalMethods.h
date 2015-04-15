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
 * $Id: databaseInternalMethods.h 7835 2014-03-14 11:51:05Z ruiz $
 */

/**
 * \file
 *         Database Internal Methods Abstraction.
 *
 * \brief The internal methods and structures that are needed to work with the database.
 */

#ifndef XME_CORE_DATAHANDLER_DATABASEINTERNALMETHODS_H
#define XME_CORE_DATAHANDLER_DATABASEINTERNALMETHODS_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataManagerTypes.h"
#include "xme/defines.h"
#include "xme/hal/include/circularBuffer.h"
#include "xme/hal/include/table.h"
#include "xme/hal/include/linkedList.h"

#include <stdint.h>

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

/**
 * \struct xme_core_dataHandler_database_attribute_t
 * \brief Defines the structure for an attribute inside a data packet. 
 * \details An attribute is composed of the key plus attributes. 
 * \note The database is attribute agnostic. It only reserves space for the data, without
 *       evaluate what kind of data will be stored there. 
 * \note The database is agnostic about attribute arrays. This means that is the user the one who
 *       should reserve as much memory size as needed for the whole attribute value. 
 */
typedef struct
{
    size_t attributeOffset; ///< The offset inside the database in which is located the attribute region. 
    uint32_t key; ///< The attribute key. 
    size_t attributeValueSize; ///< The size of the attribute value expressed in bytes. 
    bool dataAvailable; ///< Indicates that there is data available for this data packet.
} xme_core_dataHandler_database_attribute_t;

/**
 * \enum xme_core_dataHandler_database_index_e
 * \brief Defines the database index enumerated values. 
 */
enum xme_core_dataHandler_database_index_e
{
    XME_CORE_DATAHANDLER_DATABASE_INDEX_DEFAULT = 0U, ///< The default database (active). 
    XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER = 1U, ///< This is pointing to the master database. 
    XME_CORE_DATAHANDLER_DATABASE_INDEX_SHADOW = 2U
};

/**
 * \typedef xme_core_dataHandler_database_index_t
 * \brief Defines the database index. 
 */
typedef uint16_t xme_core_dataHandler_database_index_t;

/**
 * \struct xme_core_dataHandler_database_memoryRegionCopy_t
 * \brief The struct storing the address of the memory copy.
 */
typedef struct
{
    void* memoryRegionCopyAddress; ///< The address in which is located the memory copy. 
} xme_core_dataHandler_database_memoryRegionCopy_t;

/**
 * \struct xme_core_dataHandler_database_memoryRegion_t
 * \brief The struct storing all the information associated to the database.
 */
typedef struct
{
    xme_core_dataManager_memoryRegionID_t memoryRegionID; ///< The identifier of the memory region. 
    bool configured; ///< Determines if the database is fully configured. 
    xme_hal_singlyLinkedList_t(XME_CORE_DATAHANDLER_MAXMEMORYREGIONCOPIES_COUNT) memoryRegionCopies; ///< The pointer to memory region addresses. Note: The index 0 corresponds to master memory region. 
    bool isShared; ///< Flag indicating that this memory region should be marked as shared memory. 
    bool isPrivate; ///< Flag indicating that this memory region should be marked as shared memory. 
    size_t sizeInBytes; ///< The size of the memory region expressed in bytes. If not provided, it will be calculated in relation with the currently defined data stores. 
    uint16_t numOfCopies; ///< The number of copies of the current memory region. 
    uint32_t firstAvailableOffset; ///< The first available offset for the current memory region. 
} xme_core_dataHandler_database_memoryRegion_t;

/**
 * \struct xme_core_dataHandler_database_dataStore_t
 * \brief Defines the structure of a data packet.
 * \details A data packet is composed of data plus attributes. 
 *          Additionally, we can have more than one queues. 
 */
typedef struct
{
    xme_core_dataManager_dataStoreID_t dataStoreID; ///< The identifier for this data store. 
    size_t topicSize; ///< The size of the topic (without taking into account the queues) content expressed in bytes. 
    size_t dataStoreSize; ///< The size of a single data packet item (topic + attributes), without taking into account queues. 
    uint32_t topicOffset; ///< The offset w.r.t. memory region start address in which is placed the topic.
    xme_hal_singlyLinkedList_t(XME_CORE_DATAHANDLER_MAXDATASTORE_ATTRIBUTES) attributes; ///< The set of attributes associated to the current data packet. 
    uint32_t queueSize; ///< The size of the queue associated to the current data packet. 
    bool writeLock; ///< This flag indicates if the data packet is locked for write operations. 
    bool readLock; ///< This flag indicates if the data packet is locked for read operations. 
    bool configured; ///< Determines if the data packet is configured. 
    bool dataAvailable; ///< Indicates that there is data available for this data packet.
    bool persistency; ///< Indicates whether the data packet is persistent (the read operation does NOT consume the data) or non-persistent (read operation consumes data). 
    bool overwrite; ///< Indicates whether the data packet can be overwritten. 
    xme_status_t statusOnOverwrite; ///< Stores the return value on overwrite. 
    bool displayWarningOnOverwrite; ///< Indicates if a warning message should be displayed on overwrite. 
    uint16_t activeDatabaseIndex; ///< Indicates the active database index for the current package. 
    xme_hal_circularBuffer_t* circularBuffer; ///< The circular buffer.
    uint32_t currentCBWritePosition; ///< The reference to circular buffer write index to enable writeAttribute operation for queues. 
    uint32_t currentCBReadPosition; ///< The reference to circular buffer read index to enable readAttribute operation for queues. 
    xme_core_dataHandler_database_memoryRegion_t* memoryRegion; ///< The memory region associated to the current data store. 
    bool manipulated; ///< Determines if the data packet is manipulated. 
} xme_core_dataHandler_database_dataStore_t;

/**
 * \brief xme_core_dataHandler_dataStoreTable_t
 *
 * \details Table type definition in which will be stored all the data stores.
 */
typedef XME_HAL_TABLE
(
    xme_core_dataHandler_database_dataStore_t,
    xme_core_dataHandler_dataStoreTable_t,
    XME_CORE_DATAHANDLER_MAXDATASTORE_COUNT
);

/**
 * \brief xme_core_dataHandler_memoryRegionTable_t
 *
 * \details Table type definition in which will be stored all the defined memory regions.
 *          Note that a memory region can refer both to a master memory region, a memory
 *          region copy or even to other special memory regions (e.g. shared, private). 
 */
typedef XME_HAL_TABLE
(
    xme_core_dataHandler_database_memoryRegion_t,
    xme_core_dataHandler_memoryRegionTable_t,
    XME_CORE_DATAHANDLER_MAXMEMORYREGION_COUNT
);

/**
 * \struct xme_core_dataHandler_database_t
 *
 * \brief Struct storing all the information of the database.
 * \details This structure store all the information needed to deal with CHROMOSOME database. 
 *          This includes data stores, memory regions and memory regions copies. 
 */
typedef struct
{
    xme_core_dataHandler_dataStoreTable_t dataStores; ///< The set of data stores stored on the database. 
    xme_core_dataHandler_memoryRegionTable_t memoryRegions; ///< The set of memory regions on the database. 
    bool configured; ///< Determines if the database is configured. 
} xme_core_dataHandler_database_t;

/**
 * \brief The database global variable.
 */
extern xme_core_dataHandler_database_t* xme_core_dataHandler_database;

/**
 * \brief Individual manipulation item. 
 */
typedef struct
{
    xme_core_dataManager_dataStoreID_t dataStoreID; ///< The data store identifier. 
    uintptr_t address; ///< The address in which the manipulation took place. 
    uint32_t value; ///< The value of manipulation. 
} xme_core_dataHandler_database_manipulationItem_t;

/**
 * \brief The type definition for the table storing all the manipulations. 
 */
typedef XME_HAL_TABLE
(
    xme_core_dataHandler_database_manipulationItem_t,
    xme_core_dataHandler_database_manipulationTable_t,
    XME_CORE_DATAHANDLER_DATABASETESTPROBE_MAXMANIPULATIONS
);

/**
 * \var manipulationsTable
 * \brief The variable storing the manipulations. 
 */
extern xme_core_dataHandler_database_manipulationTable_t manipulationsTable;

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
xme_core_dataHandler_databaseInternal_init(void);

/**
 * \brief Frees all resources occupied by the current database structure. 
 */
void
xme_core_dataHandler_databaseInternal_fini(void);

/**
 * \brief Creates in memory a new physical memory region.
 * \details Creates a new memory region with the provided incoming configuration
 *          of the memory region. 
 *
 * \param[in,out] memoryRegion The memory region to configure. 
 *
 * \retval XME_STATUS_SUCCESS If the memory region was successfully created and
 *         the corresponding memory region address was stored in the memory
 *         region.
 * \retval XME_STATUS_INVALID_PARAMETER If the provided memory region is NULL-valued. 
 * \retval TODO: Add more return codes. 
 */
xme_status_t
createPhysicalMemoryRegion
(
    xme_core_dataHandler_database_memoryRegion_t* const memoryRegion
);

/**
 * \brief Calculates the size for a given memory region.
 * \details Explores all the data packages in the database, and in those which
 *          are associated to the provided memory region, are obtained the 
 *          data store size. 
 *
 * \param[in] memoryRegion The memory region from which should be obtained 
 *            the size. 
 *
 * \return The current reserved space in the database expressed in bytes in which
 *         data packets are associated to the provided memory region. 
 */
size_t
calculateMemoryRegionSize
(
    const xme_core_dataHandler_database_memoryRegion_t* const memoryRegion
);

/**
 * \brief Calculates and stores the size of the data store.
 * \details Read the size associated to the topic and attributes, and
 *          multiplies this value by the queue size. In this way, it is
 *          obtained the data size, which will be stored at data packet
 *          level. 
 *
 * \param[in] dataStore The data store containing a non-configured data store size. 
 *
 * \retval XME_STATUS_SUCCESS If the data store size was successfully calculated and stored 
 *         in the data store struct.
 */
xme_status_t
setDataStoreSize
(
    xme_core_dataHandler_database_dataStore_t* const dataStore
);

/**
 * \brief Calculates the optimal database size for a given size. 
 * \details This method will calculate the optimal minimum size of the
 *          memory to be reserved for the database-> 
 * \note The exact size of the database should be calculated at HAL level
 *       to ensure that the database matches with page size.
 *
 * \param[in] currentSize The current database size in bytes. 
 *
 * \return The optimal size after the application of the internal algorithm 
 *         for calculating the optimal memory size.
 */
size_t
calculateOptimalDatabaseSize
(
    size_t currentSize
);

/**
 * \brief Configures the data stores currently stored in the database-> 
 * \details Checks all the data stores in the database and establish missing
 *          parameters. 
 *
 * \retval XME_STATUS_SUCCESS If data stores has been properly configured. 
 */
xme_status_t
configureDataStores(void);

/**
 * \brief Configures the memory regions currently stored in the database-> 
 * \details For each memory region defined in the database, if this memory
 *          region is not configured, it is physically created in physical
 *          memory.
 * \note The configuration of memory regions should be prepared from 
 *       the builder. 
 *
 * \retval XME_STATUS_SUCCESS If memory regions have been properly configured. 
 */
xme_status_t
configureMemoryRegions(void);

/**
 * \brief Gets the struct for a data store. 
 *
 * \param[in] dataStoreID The data store identifier. 
 * \param[out] dataStore The output struct with the data store struct. 
 *
 * \retval XME_STATUS_SUCCESS If the data store was successfully obtained. 
 * \retval XME_STATUS_NOT_FOUND If cannot locate the specified data store. 
 */
xme_status_t
getDataStore
(
    xme_core_dataManager_dataPacketId_t dataStoreID,
    xme_core_dataHandler_database_dataStore_t** const dataStore
);

/**
 * \brief Gets the data struct for an attribute inside a data packet. 
 *
 * \param[in] dataStore The struct storing data store information. 
 * \param[in] attributeKey The key of the attribute. 
 * \param[out] attribute The output struct with the attribute. 
 *
 * \retval XME_STATUS_SUCCESS If the attribute was successfully obtained. 
 * \retval XME_STATUS_NOT_FOUND If cannot locate the specified attribute. 
 */
xme_status_t
getAttribute
(
    xme_core_dataHandler_database_dataStore_t* dataStore,
    uint32_t attributeKey,
    xme_core_dataHandler_database_attribute_t** const attribute
);

/**
 * \brief Establishes the boolean value for the lockWrite variable in a given data packet. 
 *
 * \note The lock is established at data packet level. So, this will work for all associated
 *       databases. 
 *
 * \param[in] dataPacket The struct storing the data packet information. 
 * \param[in] writeLock The boolean value to establish in the writeLock variable. 
 * 
 * \retval XME_STATUS_SUCCESS If the lock write was successfully established. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the current database configuration
 *         does not allow to establish lock write value. 
 * \retval XME_STATUS_NOT_FOUND If there is no matching data packet associated
 *         to the provided Data Packet identifier. 
 */
xme_status_t
setLockWrite
(
    xme_core_dataHandler_database_dataStore_t* const dataPacket,
    bool writeLock
);

/**
 * \brief Establishes the boolean value for the lockRead variable in a given data packet. 
 *
 * \note The lock is established at data packet level. So, this will work for all associated
 *       databases. 
 *
 * \param[in] dataPacket The struct storing the data packet information. 
 * \param[in] readLock The boolean value to establish in the readLock variable. 
 * 
 * \retval XME_STATUS_SUCCESS If the lock read was successfully established. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the current database configuration
 *         does not allow to establish lock read value. 
 * \retval XME_STATUS_NOT_FOUND If there is no matching data packet associated
 *         to the provided Data Packet identifier. 
 */
xme_status_t
setLockRead
(
    xme_core_dataHandler_database_dataStore_t* const dataPacket,
    bool readLock
);

/**
 * \brief Establishes the boolean value for the data availability in a given data packet. 
 *
 * \param[in] dataStore The struct storing the data store information. 
 * \param[in] dataAvailable The boolean value to establish in the data availability variable. 
 * 
 * \retval XME_STATUS_SUCCESS If the data availability was successfully established. 
 */
xme_status_t
setDataAvailability
(
    xme_core_dataHandler_database_dataStore_t* const dataStore,
    bool dataAvailable
);

/**
 * \brief Obtains if there is at least one attribute available for the given data store. 
 *
 * \param[in] dataStore The struct storing the data store information. 
 * \param[out] attributeAvailable The boolean value that determines if any attribute has data available. 
 * 
 * \retval XME_STATUS_SUCCESS If the data availability was successfully obtained. 
 */
xme_status_t
getAttributeAvailability
(
    xme_core_dataHandler_database_dataStore_t* const dataStore,
    bool* const attributeAvailable
);

/**
 * \brief Establishes the boolean value for the attribute availability in a given data packet. 
 *
 * \param[in] attribute The struct storing the attribute information. 
 * \param[in] dataAvailable The boolean value to establish in the attribute availability variable. 
 * 
 * \retval XME_STATUS_SUCCESS If the data availability was successfully established. 
 */
xme_status_t
setAttributeAvailability
(
    xme_core_dataHandler_database_attribute_t* const attribute,
    bool dataAvailable
);

/**
 * \brief Get the number of bytes to copy in a memory operation. 
 * \details Given the data size, the buffer size and the offset with respect
 *          to initial data position, calculates the number of bytes to 
 *          copy. 
 *
 * \param[in] dataSizeInBytes The data size expressed in bytes. 
 * \param[in] bufferSizeInBytes The available buffer size from which the data is copied. 
 * \param[in] offsetInBytes The position w.r.t. data initial position from which the
 *            memory operation should start (aka, offset). 
 *
 * \return The total size of allowed bytes to use in the copy operation.
 */
uint32_t
getBytesToCopy
(
    size_t dataSizeInBytes,
    size_t bufferSizeInBytes,
    uint32_t offsetInBytes
);

/**
 * \brief Given a memory region, reallocates this memory region to a new size. 
 * \details The reallocation is targeted to ensure that it is not obtained the
 *          XME_STATUS_OUT_OF_RESOURCES error during configuration time. 
 *
 * \param[in,out] memoryRegion The memory region to reallocate. 
 * \param[in] newSize The new size of the memory region. 
 *
 * \retval XME_STATUS_SUCCESS If the reallocation was success.
 * \retval XME_STATUS_OUT_OF_RESOURCES If reallocation was not success. 
 */
xme_status_t
memoryRegionReallocate
(
    xme_core_dataHandler_database_memoryRegion_t* const memoryRegion, 
    size_t newSize
);

/**
 * \brief Gets the memory address in which is located the memory region copy. 
 *
 * \note The memory region index is zero based. 
 *
 * \param[in] memoryRegion The memory region to reallocate. 
 * \param[in] index The index of the memory region copy to obtain. 
 *
 * \return The address of the database. 
 */
void*
getAddressFromMemoryCopy
(
    xme_core_dataHandler_database_memoryRegion_t* const memoryRegion,
    xme_core_dataHandler_database_index_t index
);

/**
 * \brief Check the overwrite policy for the current data store before write operation. 
 * \note A warning message is displayed if the overwrite policy indicates so and the overwrite will take place.
 *
 * \param[in] dataStore The data store to check. 
 * \param[in] dataAvailable The flag determining if data is already available in the data store. 
 * \param[out] enableWriteOperation If the overwrite checking do not apply, this variable is set to true. 
 * \param[out] isOverwrite If the operation is an overwrite, this flag is set to true. 
 *
 * \retval XME_STATUS_SUCCESS If the overwrite has no effect before the write operation. 
 * \return The error code associated to the overwrite policy, if the overwrite is established to return an error before writing. 
 */
xme_status_t
checkOverwriteBeforeWrite
(
    xme_core_dataHandler_database_dataStore_t* const dataStore,
    bool dataAvailable,
    bool* const enableWriteOperation,
    bool* const isOverwrite
);

/**
 * \brief Check the overwrite policy for the current data store after write operation. 
 * \note A warning message is displayed if the overwrite policy indicates so and the overwrite took place.
 *
 * \param[in] dataStore The data store to check. 
 *
 * \retval XME_STATUS_SUCCESS If the overwrite has no effect after the write operation. 
 * \return The error code associated to the overwrite policy, if the overwrite is established to return an error after writing. 
 */
xme_status_t
checkOverwriteAfterWrite
(
    xme_core_dataHandler_database_dataStore_t* const dataStore
);

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_DATABASEINTERNALMETHODS_H */

