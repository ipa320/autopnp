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
 * $Id: databaseBuilder.c 7833 2014-03-14 10:52:23Z ruiz $
 */

/**
 * \file
 *         Database Builder.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/dataHandler/internal/databaseBuilder.h"
#include "xme/core/dataHandler/internal/databaseConfigurator.h"

#include "xme/core/component.h"
#include "xme/core/log.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \struct xme_core_dataHandler_databaseBuilder_dataStore_t
 * \brief Defines the structure of a data store.
 * \details A data store is composed of topic plus attributes. 
 *          Additionally, we can have more than one queues. 
 */
typedef struct
{
    xme_core_dataManager_dataStoreID_t dataStoreID; ///< The identifier for this data packet. 
    size_t topicSize; ///< The size of the topic. 
    uint32_t queueSize; ///< The size of the queue associated to the current data packet. 
    bool persistency; ///< The persistency property associated to the data packet. 
    bool overwrite; ///< The overwrite flag associated to the data packet. 
    xme_status_t statusOnOverwrite; ///< The return status if an overwrite is tried. 
    bool displayWarningOnOverwrite; ///< The flag associated to display a warning when an overwrite is tried. 
    xme_core_dataManager_memoryRegionID_t memoryRegionID; ///< The memory region associated to the data packet. 
} xme_core_dataHandler_databaseBuilder_dataStore_t;

/**
 * \struct xme_core_dataHandler_databaseBuilder_attribute_t
 * \brief Defines the structure for an attribute inside a data packet. 
 * \details An attribute is composed of the key plus attributes. 
 * \note The database is attribute agnostic. It only reserves space for the data, without
 *       evaluate what kind of data will be stored there. 
 * \note The database is agnostic about attribute arrays. This means that is the user the one who
 *       should reserve as much memory size as needed for the whole attribute value. 
 */
typedef struct
{
    xme_core_dataManager_dataStoreID_t dataStoreID; ///< The identifier for this data packet. 
    uint32_t key; ///< The attribute key. 
    size_t attributeValueSize; ///< The size of the attribute value. 
} xme_core_dataHandler_databaseBuilder_attribute_t;

/**
 * \struct xme_core_dataHandler_databaseBuilder_memoryRegion_t
 * \brief Defines the structure of a memory region. 
 */
typedef struct
{
    xme_core_dataManager_memoryRegionID_t memoryRegionID; ///< The identifier that identifies the memory region. This identifier is used to create data packets specifying the memory region. 
    bool isShared; ///< Determines if the memory region is a shared memory region. 
    bool isPrivate; ///< Determines if the memory region is a private memory region. 
    //uint32_t permissions; ///< The permissions associated to the memory region. Possible values are none, read, write, and execute. 
    size_t memoryRegionSize; ///< The size expressed in bytes of the memory region. If not specified, it is calculated dynamically. 
    uint16_t numberOfCopies; ///< Number of copies associated to the current database. 
} xme_core_dataHandler_databaseBuilder_memoryRegion_t;


/**
 * \enum xme_core_dataHandler_memoryRegionType_e
 * \brief Defines the memory region type to create at database level. 
 */
enum xme_core_dataHandler_memoryRegionType_e
{
    XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_INVALID = 0U, // The memory region type is invalid. 
    XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_DEFAULT, // The memory region type is the default (heap allocated). 
    XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_SHARED, // The memory region type is shared between different threads. 
    XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_PRIVATE // The memory region type is private. 
};

/**
 * \typedef xme_core_dataHandler_memoryRegionType_t
 * \brief Defines the datatype for defining memory region type. 
 */
typedef uint16_t xme_core_dataHandler_memoryRegionType_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \brief Defines the current database builder.
 */
static xme_core_dataHandler_databaseBuilder_t* currentBuilder = NULL; //< "this" pointer for the current builder.

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Defines a generic memory region. 
 * \details Creates a generic memory region in the current database. 
 *          This memory region dynamically allocated by the operating system. 
 * 
 * \param[out] memoryRegionID The identifier associated to the memory region. 
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
createGenericMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
);

/**
 * \brief Defines a shared memory region. 
 * \details Creates a memory region in the current database. 
 * 
 * \param[out] memoryRegionID The identifier associated to the memory region. 
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
createSharedMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
);

/**
 * \brief Defines a private memory region. 
 * \details Creates a memory region in the current database. 
 * 
 * \param[out] memoryRegionID The identifier associated to the memory region. 
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
createPrivateMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
);

/**
 * \brief Establishes the static configuration of the memory region size. 
 *
 * \param[in] memoryRegionID The identifier associated to the memory region. 
 * \param[in] sizeInBytes The size of the memory region expressed in bytes. 
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
setMemoryRegionSize
(
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    size_t sizeInBytes
);

/**
 * \brief Defines the number of memory region copies to be used in the database. 
 *
 * \note The default memory region copies is established to one. 
 *
 * \param[in] memoryRegionID The identifier associated to the memory region. 
 * \param[in] numberOfMemoryCopies The number of memory copies to establish. 
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
setNumberOfMemoryRegionCopies
(
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    uint16_t numberOfMemoryCopies
);

/**
 * \brief Defines a data store. 
 * \details Creates a data store in the database. 
 * \note This data packet is specifically created in the default memory region. 
 * 
 * \param[in] topicSize The topic size for current datapacket. 
 * \param[out] dataStoreID The identifier associated to the data store. 
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
createDataStore
(
    size_t topicSize,
    xme_core_dataManager_dataStoreID_t* dataStoreID
);

/**
 * \brief Defines a data store. 
 * \details Creates a data store in the specified memory region in the database. 
 * 
 * \param[in] dataSize The data size for current datapacket. 
 * \param[in] memoryRegionID The target memory region in which to place the data store. 
 * \param[out] dataStoreID The identifier associated to the data store. 
 *
 * \return The database builder object reference. NULL when not enough memory to create
 *         the builder.
 */
static xme_core_dataHandler_databaseBuilder_t*
createDataStoreInMemoryRegion
(
    size_t dataSize,
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    xme_core_dataManager_dataStoreID_t* dataStoreID
);

/**
 * \brief Establishes the queue size for the data store. 
 *
 * \note The default queue size is established to 1. If we do not
 *       call this method, the queue will remain to one. 
 * 
 * \param[in] dataStoreID The data store identifier. 
 * \param[in] queueSize The queue size for the data sotre.  
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
setQueueSize
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t queueSize
);

/**
 * \brief Establishes the data store persistency flag. 
 * \details A non-persistent data store consumes the data after the
 *          read operation takes place. A persistent data store
 *          does not consume the data. 
 * \note The default data store status is persistent set to true. 
 * 
 * \param[in] dataStoreID The data packet identifier. 
 * \param[in] persistency Boolean value to establish.  
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
setPersistency
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    bool persistency
);

/**
 * \brief Establish the data packet overwrite flag. 
 * \details If the overwrite flag is true, the overwrite takes places, and replaces the old value in the database. 
 *          If the overwrite flag is false, the overwrite does not take places, and the old value remains in the database. 
 *
 * \param[in] dataStoreID The data store identifier.
 * \param[in] overwrite The boolean overwrite flag. 
 * \param[in] statusOnOverwrite The value to return if an overwrite takes place. 
 * \param[in] displayWarning A boolean value which decides if a warning message should be displayed when trying to overwrite. 
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
setOverwrite
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    bool overwrite,
    xme_status_t statusOnOverwrite,
    bool displayWarning
);

/**
 * \brief Defines an attribute associated to a data store. 
 * \details Creates a data store in the current database. 
 * 
 * \param[in] dataStoreID The data store to add the attribute. 
 * \param[in] key The attribute key. 
 * \param[in] attributeSize The total size of the attribute value. 
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
createAttribute
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t key, 
    size_t attributeSize
);


/**
 * \brief Builds and stores the structure for a database. 
 * \details Allocates and stores the needed information for the current database. 
 *
 * \return The database object reference. 
 */
static xme_core_dataHandler_database_t* 
build(void);

/******************************************************************************/
/***   Helper Methods                                                       ***/
/******************************************************************************/

/**
 * \brief Checks if the data stpre is already defined in the current builder. 
 *
 * \param[in] dataStoreID The data store identifier to find in the current builder. 
 * 
 * \retval true if the data store is already defined. 
 * \retval false if the data store is still not defined. 
 */
bool
isDataStoreDefined
(
    xme_core_dataManager_dataStoreID_t dataStoreID
);

/**
 * \brief Creates in the database the element corresponding to the memory region. 
 * 
 * \details Provided the builder element for memory regions, it populates in the database
 *          the memory regions. 
 *
 * \param[in] builderMemoryRegion The struct with the information collected from the builder. 
 * \param[in,out] targetDatabase The database struct containing the current data handler configuration. 
 *
 * \retval XME_STATUS_SUCCESS If the memory region was successfully created in the database. 
 */
xme_status_t
createMemoryRegionInDatabase
(
    const xme_core_dataHandler_databaseBuilder_memoryRegion_t* const builderMemoryRegion, 
    const xme_core_dataHandler_database_t* const targetDatabase
);

/**
 * \brief Creates in the database the element corresponding to the data stores. 
 * 
 * \details Provided the builder element for memory regions, it populates in the database
 *          with corresponding data stores. 
 *
 * \param[in] builderDataStore The struct with the information collected from the builder. 
 * \param[in,out] targetDatabase The database struct containing the current data handler configuration. 
 *
 * \retval XME_STATUS_SUCCESS If the data store was successfully created in the database. 
 */
xme_status_t
createDataStoreInDatabase
(
    xme_core_dataHandler_databaseBuilder_dataStore_t* const builderDataStore, 
    xme_core_dataHandler_database_t* const targetDatabase
);

/**
 * \brief Creates in the database the element corresponding to the attributes. 
 * 
 * \details Provided the builder element for attributes, it populates in the database
 *          the data packets with the included attributes. 
 *
 * \param[in] builderAttributes The struct with the information collected from the builder. 
 * \param[in,out] targetDatabase The database struct containing the current data handler configuration. 
 *
 * \retval XME_STATUS_SUCCESS If attributes were successfully created in the database. 
 */
xme_status_t
addAttributesToDataStoreInDatabase
(
    const xme_core_dataHandler_databaseBuilder_attribute_t* const builderAttributes, 
    const xme_core_dataHandler_database_t* const targetDatabase
);

/**
 * \brief Creates a new logical memory region to be created in the database. 
 *
 * \note This method should not be called directly. We should use specialized signatures
 *       for creating generic, shared or private memory regions. 
 *
 * \param[in] memoryRegionType The memory region type to create. 
 * \param[out] memoryRegionID The identifier of that memory region. 
 *
 * \return The database builder object reference. 
 */
static xme_core_dataHandler_databaseBuilder_t*
createMemoryRegion
(
    xme_core_dataHandler_memoryRegionType_t memoryRegionType,
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_core_dataHandler_databaseBuilder_t*
xme_core_dataHandler_databaseBuilder_create(void)
{
    xme_core_dataHandler_databaseBuilder_t* databaseBuilder;

    databaseBuilder = (xme_core_dataHandler_databaseBuilder_t*) xme_hal_mem_alloc(sizeof(xme_core_dataHandler_databaseBuilder_t));

    // Initialize linked lists associated to data packets and attributes. 
    XME_HAL_SINGLYLINKEDLIST_INIT(databaseBuilder->dataStores);
    XME_HAL_SINGLYLINKEDLIST_INIT(databaseBuilder->attributes);
    XME_HAL_SINGLYLINKEDLIST_INIT(databaseBuilder->memoryRegions);

    // Assign the callbacks to the corresponding methods.
    databaseBuilder->createGenericMemoryRegion = createGenericMemoryRegion;
    databaseBuilder->createPrivateMemoryRegion = createPrivateMemoryRegion;
    databaseBuilder->createSharedMemoryRegion = createSharedMemoryRegion;
    databaseBuilder->setMemoryRegionSize = setMemoryRegionSize;
    databaseBuilder->setNumberOfMemoryRegionCopies = setNumberOfMemoryRegionCopies;
    databaseBuilder->createDataStore = createDataStore;
    databaseBuilder->createDataStoreInMemoryRegion = createDataStoreInMemoryRegion;
    databaseBuilder->setQueueSize = setQueueSize;
    databaseBuilder->setPersistency = setPersistency;
    databaseBuilder->setOverwrite = setOverwrite;
    databaseBuilder->createAttribute = createAttribute;
    databaseBuilder->build = build;

    currentBuilder = databaseBuilder;

    return databaseBuilder;
}

xme_status_t
xme_core_dataHandler_databaseBuilder_destroy
(
    xme_core_dataHandler_databaseBuilder_t* databaseBuilder
)
{
    XME_HAL_SINGLYLINKEDLIST_FINI(databaseBuilder->dataStores);
    XME_HAL_SINGLYLINKEDLIST_FINI(databaseBuilder->attributes);
    XME_HAL_SINGLYLINKEDLIST_FINI(databaseBuilder->memoryRegions);

    xme_hal_mem_free(databaseBuilder);

    currentBuilder = NULL;

    xme_core_dataHandler_databaseInternal_fini();

    return XME_STATUS_SUCCESS;
}

/******************************************************************************/


static xme_core_dataHandler_databaseBuilder_t*
createGenericMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
)
{
    return createMemoryRegion(XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_DEFAULT, memoryRegionID);
}

static xme_core_dataHandler_databaseBuilder_t*
createSharedMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
)
{
    return createMemoryRegion(XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_SHARED, memoryRegionID);
}

static xme_core_dataHandler_databaseBuilder_t*
createPrivateMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
)
{
    return createMemoryRegion(XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_PRIVATE, memoryRegionID);
}

static xme_core_dataHandler_databaseBuilder_t*
createMemoryRegion
(
    xme_core_dataHandler_memoryRegionType_t memoryRegionType,
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;
    xme_core_dataHandler_databaseBuilder_memoryRegion_t* newMemoryRegion;
    
    XME_CHECK((xme_core_dataHandler_memoryRegionType_t)XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_INVALID != memoryRegionType, NULL);

    newMemoryRegion = (xme_core_dataHandler_databaseBuilder_memoryRegion_t*) xme_hal_mem_alloc(sizeof(xme_core_dataHandler_databaseBuilder_memoryRegion_t));

    // Initialize the memory region.
    newMemoryRegion->memoryRegionID = xme_core_dataHandler_databaseConfigurator_createMemoryRegionID();

    newMemoryRegion->memoryRegionSize = 0U;
    newMemoryRegion->numberOfCopies = 1U;
    //newMemoryRegion->permissions = XME_HAL_MMAP_PROTECTION_READ | XME_HAL_MMAP_PROTECTION_WRITE;

    switch(memoryRegionType)
    {
        case XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_DEFAULT:
            newMemoryRegion->isPrivate = false;
            newMemoryRegion->isShared = false;
            break;
        case XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_PRIVATE:
            newMemoryRegion->isPrivate = true;
            newMemoryRegion->isShared = false;
            break;
        case XME_CORE_DATAHANDLER_MEMORYREGIONTYPE_SHARED:
            newMemoryRegion->isPrivate = false;
            newMemoryRegion->isShared = true;
            break;
        default:
            XME_LOG(XME_LOG_DEBUG, "[DatabaseBuilder] Unsupported memory region type: %d", memoryRegionType);
            return NULL;
    }

    // Add this memory region to the builder. 
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(&builder->memoryRegions, newMemoryRegion), NULL);

    // Establish in the output variable the memory region identifier. 
    *memoryRegionID = newMemoryRegion->memoryRegionID;

    return builder;
}

static xme_core_dataHandler_databaseBuilder_t*
setMemoryRegionSize
(
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    size_t sizeInBytes
)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;

    XME_CHECK((xme_core_dataHandler_memoryRegionType_t)XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID != memoryRegionID, NULL);
    XME_CHECK(0 != sizeInBytes, NULL);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        builder->memoryRegions, 
        xme_core_dataHandler_databaseBuilder_memoryRegion_t, 
        builderMemoryRegion
    );
    {
        if (builderMemoryRegion->memoryRegionID == memoryRegionID)
        {
            builderMemoryRegion->memoryRegionSize = sizeInBytes;
            return builder;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return NULL;
}

static xme_core_dataHandler_databaseBuilder_t*
setNumberOfMemoryRegionCopies
(
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    uint16_t numberOfMemoryCopies
)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;

    XME_CHECK(XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID != memoryRegionID, NULL);
    XME_CHECK(0U != numberOfMemoryCopies, NULL);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        builder->memoryRegions, 
        xme_core_dataHandler_databaseBuilder_memoryRegion_t, 
        builderMemoryRegion
    );
    {
        if (builderMemoryRegion->memoryRegionID == memoryRegionID)
        {
            builderMemoryRegion->numberOfCopies = numberOfMemoryCopies;
            return builder;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return NULL;
}

/******************************************************************************/

static xme_core_dataHandler_databaseBuilder_t*
createDataStore
(
    size_t topicSize,
    xme_core_dataManager_dataStoreID_t* dataStoreID
)
{
    return createDataStoreInMemoryRegion(topicSize, XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT, dataStoreID);
}

static xme_core_dataHandler_databaseBuilder_t*
createDataStoreInMemoryRegion
(
    size_t topicSize,
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    xme_core_dataManager_dataStoreID_t* dataStoreID
)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;
    xme_core_dataHandler_databaseBuilder_dataStore_t* newDataStore;
    
    XME_CHECK(XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID != memoryRegionID, NULL);
    XME_CHECK(topicSize != 0U, NULL);

    newDataStore = (xme_core_dataHandler_databaseBuilder_dataStore_t*) xme_hal_mem_alloc(sizeof(xme_core_dataHandler_databaseBuilder_dataStore_t));
    XME_CHECK(NULL != newDataStore, NULL);

    // Initialize data packet associated fields
    newDataStore->topicSize = topicSize;
    newDataStore->queueSize = 1U;

    // Establish by default NON-PERSISTENT data packet. 
    newDataStore->persistency = (bool) false;

    // Establish the default overwrite policy. 
    newDataStore->overwrite = (bool) true;
    newDataStore->statusOnOverwrite = XME_STATUS_SUCCESS;
    newDataStore->displayWarningOnOverwrite = (bool) true;

    // Establish the memory region.
    newDataStore->memoryRegionID = memoryRegionID;

    // Assign a new data packet identifier.
    newDataStore->dataStoreID = xme_core_dataHandler_databaseConfigurator_createDataPacketID();

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(&builder->dataStores, newDataStore), NULL);

    *dataStoreID = newDataStore->dataStoreID;

    return builder;
}

static xme_core_dataHandler_databaseBuilder_t*
setQueueSize
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t queueSize
)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;

    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataStoreID, NULL);
    XME_CHECK(queueSize != 0, NULL);

    // Lookup for the corresponding data packet. 
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        builder->dataStores, 
        xme_core_dataHandler_databaseBuilder_dataStore_t, 
        dataStore
    );
    {
        if (dataStore->dataStoreID == dataStoreID)
        {
            dataStore->queueSize = queueSize;

            // Additional condition to not show additional messages to overwrite queues. 
            if (queueSize > 1U)
            {
                dataStore->displayWarningOnOverwrite = (bool) false;
            }

            return builder;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // If after the lookup process, no data packets were found, just return NULL. 
    return NULL;

}

static xme_core_dataHandler_databaseBuilder_t*
setPersistency
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    bool persistency
)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;

    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataStoreID, NULL);

    // Lookup for the corresponding data packet. 
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        builder->dataStores, 
        xme_core_dataHandler_databaseBuilder_dataStore_t, 
        dataStore
    );
    {
        if (dataStore->dataStoreID == dataStoreID)
        {
            dataStore->persistency = persistency;
            dataStore->displayWarningOnOverwrite = (bool) false;

            return builder;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // If after the lookup process, no data packets were found, just return NULL. 
    return NULL;
}

static xme_core_dataHandler_databaseBuilder_t*
setOverwrite
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    bool overwrite,
    xme_status_t statusOnOverwrite,
    bool displayWarning
)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;

    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataStoreID, NULL);

    // Lookup for the corresponding data packet. 
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        builder->dataStores, 
        xme_core_dataHandler_databaseBuilder_dataStore_t, 
        dataStore
    );
    {
        if (dataStore->dataStoreID == dataStoreID)
        {
            dataStore->overwrite = overwrite;
            dataStore->statusOnOverwrite = statusOnOverwrite;
            dataStore->displayWarningOnOverwrite = displayWarning;

            return builder;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // If after the lookup process, no data packets were found, just return NULL. 
    return NULL;
}

static xme_core_dataHandler_databaseBuilder_t*
createAttribute
(
    xme_core_dataManager_dataStoreID_t dataStoreID,
    uint32_t key,
    size_t attributeSize
)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;

    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataStoreID, NULL);
    XME_CHECK(attributeSize != 0, NULL);

    if (isDataStoreDefined(dataStoreID))
    {
        // Checks if the data packet is already created. 
        xme_core_dataHandler_databaseBuilder_attribute_t* newAttribute;

        newAttribute = (xme_core_dataHandler_databaseBuilder_attribute_t*) xme_hal_mem_alloc(sizeof(xme_core_dataHandler_databaseBuilder_attribute_t));

        newAttribute->key = key;
        newAttribute->attributeValueSize = attributeSize;
        newAttribute->dataStoreID = dataStoreID;
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(&builder->attributes, newAttribute), NULL);

        return builder;
    }
    else
    {
        return NULL;
    }
}

/******************************************************************************/

static xme_core_dataHandler_database_t*
build(void)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;
    xme_core_dataHandler_database_t* newDatabase;
    xme_status_t status;

    // Creates and initializes the database object
    newDatabase = xme_core_dataHandler_databaseConfigurator_create();

    // Create the memory regions. 
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        builder->memoryRegions, 
        xme_core_dataHandler_databaseBuilder_memoryRegion_t, 
        builderMemoryRegion
    );
    {
        status = createMemoryRegionInDatabase(builderMemoryRegion, newDatabase);
        XME_CHECK(XME_STATUS_SUCCESS == status, NULL);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // Populate the data stores with collected information in builder. 
    // Data Packets
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        builder->dataStores, 
        xme_core_dataHandler_databaseBuilder_dataStore_t, 
        builderDataStore
    );
    {
        status = createDataStoreInDatabase(builderDataStore, newDatabase);
        XME_CHECK(XME_STATUS_SUCCESS == status, NULL);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    
    // Attributes
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        builder->attributes, 
        xme_core_dataHandler_databaseBuilder_attribute_t, 
        builderAttribute
    );
    {
        status = addAttributesToDataStoreInDatabase(builderAttribute, newDatabase);
        XME_CHECK(XME_STATUS_SUCCESS == status, NULL);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    status = xme_core_dataHandler_databaseConfigurator_configure();
    XME_CHECK_MSG
    (
        XME_STATUS_SUCCESS == status,
        NULL,
        XME_LOG_WARNING, 
        "[DatabaseBuilder] Cannot configure the database.\n"
    );

    // Clean-up the current builder structures. 
    status = xme_hal_singlyLinkedList_clear(&builder->memoryRegions);
    XME_CHECK_MSG(XME_STATUS_SUCCESS == status, NULL, XME_LOG_WARNING, "[DatabaseBuilder] Cannot clear memory regions from builder.\n");
    status = xme_hal_singlyLinkedList_clear(&builder->dataStores);
    XME_CHECK_MSG(XME_STATUS_SUCCESS == status, NULL, XME_LOG_WARNING, "[DatabaseBuilder] Cannot clear data stores from builder.\n");
    status = xme_hal_singlyLinkedList_clear(&builder->attributes);
    XME_CHECK_MSG(XME_STATUS_SUCCESS == status, NULL, XME_LOG_WARNING, "[DatabaseBuilder] Cannot clear attributes from builder.\n");

    // Return the database object. 
    return newDatabase;
}

/******************************************************************************/

bool
isDataStoreDefined
(
    xme_core_dataManager_dataStoreID_t dataStoreID
)
{
    xme_core_dataHandler_databaseBuilder_t* builder = currentBuilder;

    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataStoreID, (bool) false);

    // Lookup for the corresponding data packet. 
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
    (
        builder->dataStores, 
        xme_core_dataHandler_databaseBuilder_dataStore_t, 
        dataStore
    );
    {
        if (dataStore->dataStoreID == dataStoreID)
        {
           return true;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // If after the lookup process, no data packets were found, just return false.
    return (bool) false;
}

xme_status_t
createMemoryRegionInDatabase
(
    const xme_core_dataHandler_databaseBuilder_memoryRegion_t* const builderMemoryRegion, 
    const xme_core_dataHandler_database_t* const targetDatabase
)
{
    // Create in the database a memory region with the provided information by the builder.
    xme_core_dataHandler_database_memoryRegion_t* databaseMemoryRegion;
    xme_hal_table_rowHandle_t handle;

    handle = XME_HAL_TABLE_ADD_ITEM(targetDatabase->memoryRegions);
    XME_ASSERT_RVAL(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, XME_STATUS_OUT_OF_RESOURCES);

    databaseMemoryRegion = XME_HAL_TABLE_ITEM_FROM_HANDLE(targetDatabase->memoryRegions, handle);
    XME_ASSERT(NULL != databaseMemoryRegion);

    databaseMemoryRegion->memoryRegionID = builderMemoryRegion->memoryRegionID;
    //databaseMemoryRegion->databaseFlags = 0U; // TODO: We assume that memory regions will be READ/WRITE. Collect this information from a dedicated interface. 
    // Initialize the memory regions copy. 
    XME_HAL_SINGLYLINKEDLIST_INIT(databaseMemoryRegion->memoryRegionCopies);
    databaseMemoryRegion->numOfCopies = builderMemoryRegion->numberOfCopies;

    // Update the database type. 
    databaseMemoryRegion->isShared = builderMemoryRegion->isShared;
    databaseMemoryRegion->isPrivate = builderMemoryRegion->isPrivate;

    // Update the database size. 
    databaseMemoryRegion->sizeInBytes = builderMemoryRegion->memoryRegionSize;

    // The memory region remains not configured until the physical memory is not specifically defined.
    databaseMemoryRegion->configured = (bool) false;

    return XME_STATUS_SUCCESS;
}

xme_status_t
createDataStoreInDatabase
(
    xme_core_dataHandler_databaseBuilder_dataStore_t* const builderDataStore, 
    xme_core_dataHandler_database_t* const targetDatabase
)
{
    // Create in the data store with the provided information by the builder.
    xme_core_dataHandler_database_dataStore_t* databaseDataStore;
    xme_core_dataHandler_database_memoryRegion_t* memoryRegion = NULL;
    xme_hal_table_rowHandle_t tableHandle;

    tableHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    // Lookup if there is an existing memory region with the corresponding memory region ID. 
    XME_HAL_TABLE_GET_NEXT
    (
        targetDatabase->memoryRegions, 
        xme_hal_table_rowHandle_t, tableHandle, 
        xme_core_dataHandler_database_memoryRegion_t, memoryRegion,
        memoryRegion->memoryRegionID == builderDataStore->memoryRegionID
    );
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == tableHandle)
    {
        return XME_STATUS_INVALID_CONFIGURATION;
    }

    XME_ASSERT(NULL != memoryRegion);

    tableHandle = XME_HAL_TABLE_ADD_ITEM(targetDatabase->dataStores);
    XME_ASSERT_RVAL(XME_HAL_TABLE_INVALID_ROW_HANDLE != tableHandle, XME_STATUS_OUT_OF_RESOURCES);

    databaseDataStore = XME_HAL_TABLE_ITEM_FROM_HANDLE(targetDatabase->dataStores, tableHandle);
    XME_ASSERT(NULL != databaseDataStore);

    databaseDataStore->dataStoreID = builderDataStore->dataStoreID;

    // Establish the memory region.
    databaseDataStore->memoryRegion = memoryRegion;

    databaseDataStore->topicSize = builderDataStore->topicSize;
    databaseDataStore->topicOffset = 0U;
    databaseDataStore->queueSize = builderDataStore->queueSize;
    databaseDataStore->dataAvailable = (bool) false;
    databaseDataStore->writeLock = (bool) false;
    databaseDataStore->readLock = (bool) false;
    databaseDataStore->activeDatabaseIndex = XME_CORE_DATAHANDLER_DATABASE_INDEX_MASTER;
    databaseDataStore->persistency = builderDataStore->persistency;
    databaseDataStore->overwrite =  builderDataStore->overwrite;
    databaseDataStore->statusOnOverwrite = builderDataStore->statusOnOverwrite;
    databaseDataStore->displayWarningOnOverwrite =  builderDataStore->displayWarningOnOverwrite;

    // Initialize attributes. 
    XME_HAL_SINGLYLINKEDLIST_INIT(databaseDataStore->attributes);

    databaseDataStore->configured = (bool) false;

    return XME_STATUS_SUCCESS;
}

xme_status_t
addAttributesToDataStoreInDatabase
(
    const xme_core_dataHandler_databaseBuilder_attribute_t* const builderAttribute, 
    const xme_core_dataHandler_database_t* const targetDatabase
)
{
    // Create in the data store with the provided information by the builder.
    xme_core_dataHandler_database_dataStore_t* dataStore = NULL;
    xme_core_dataHandler_database_attribute_t* newAttribute;
    xme_hal_table_rowHandle_t tableHandle;

    tableHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    // Lookup if there is an existing memory region with the corresponding memory region ID. 
    XME_HAL_TABLE_GET_NEXT
    (
        targetDatabase->dataStores, 
        xme_hal_table_rowHandle_t, tableHandle, 
        xme_core_dataHandler_database_dataStore_t, dataStore,
        dataStore->dataStoreID == builderAttribute->dataStoreID
    );
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == tableHandle)
    {
        return XME_STATUS_INVALID_CONFIGURATION;
    }

    XME_ASSERT(NULL != dataStore);

    newAttribute = (xme_core_dataHandler_database_attribute_t*) xme_hal_mem_alloc(sizeof(xme_core_dataHandler_database_attribute_t));

    newAttribute->key = builderAttribute->key;
    newAttribute->attributeValueSize = builderAttribute->attributeValueSize;
    newAttribute->attributeOffset = 0U; 
    newAttribute->dataAvailable = (bool) false;

    // This is the right place to include the attribute. 
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(&dataStore->attributes, newAttribute), XME_STATUS_INVALID_CONFIGURATION);

    return XME_STATUS_SUCCESS;
}

