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
 * $Id: databaseBuilder.h 7833 2014-03-14 10:52:23Z ruiz $
 */

/**
 * \file
 *         Database builder.
 *
 * \brief The database builder acts as a creator of databases for the data handler.
 *        We should establish all the fields that will be stored in the database
 *        before creating the builder. 
 *        This builder is based on builder design pattern.
 */

#ifndef XME_CORE_DATAHANDLER_DATABASEBUILDER_H
#define XME_CORE_DATAHANDLER_DATABASEBUILDER_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataHandler/internal/databaseInternalMethods.h"

#include "xme/core/dataManagerTypes.h"
#include "xme/defines.h"

#include <stdint.h>

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

/**
 * \struct xme_core_dataHandler_databaseBuilder
 * \brief The struct storing all the information to be stored in the database. 
 */
struct xme_core_dataHandler_databaseBuilder
{
    /* Database fields */
    xme_hal_singlyLinkedList_t(XME_CORE_DATAHANDLER_MAXDATASTORE_COUNT) dataStores; ///< Data stores to be stored in the database. 
    xme_hal_singlyLinkedList_t(XME_CORE_DATAHANDLER_MAXATTRIBUTE_COUNT) attributes; ///< Attributes associated to the data packets to be stored in the database. 
    xme_hal_singlyLinkedList_t(XME_CORE_DATAHANDLER_MAXMEMORYREGION_COUNT) memoryRegions; ///< Memory regions to create in the database. 

    /* Database builder configurator */
    /* - Memory regions */
    struct xme_core_dataHandler_databaseBuilder* (*createGenericMemoryRegion)(xme_core_dataManager_memoryRegionID_t* /*memoryRegionID*/); ///< Declares a generic memory region. 
    struct xme_core_dataHandler_databaseBuilder* (*createPrivateMemoryRegion)(xme_core_dataManager_memoryRegionID_t* /*memoryRegionID*/); ///< Declares a private memory region. 
    struct xme_core_dataHandler_databaseBuilder* (*createSharedMemoryRegion)(xme_core_dataManager_memoryRegionID_t* /*memoryRegionID*/); ///< Declares a shared memory region. 
    struct xme_core_dataHandler_databaseBuilder* (*setMemoryRegionSize)(xme_core_dataManager_memoryRegionID_t /*memoryRegionID*/, size_t /*sizeInBytes*/); ///< Establishes the memory region size. 
    struct xme_core_dataHandler_databaseBuilder* (*setNumberOfMemoryRegionCopies)(xme_core_dataManager_memoryRegionID_t /*memoryRegionID*/, uint16_t /*numberOfMemoryCopies*/); ///< Establishes the number of copies for the memory region. 

    /* - Data Stores */
    struct xme_core_dataHandler_databaseBuilder* (*createDataStore)(size_t /*topicSize*/, xme_core_dataManager_dataStoreID_t* /*dataStoreID*/); ///< Declares a data store.
    struct xme_core_dataHandler_databaseBuilder* (*createDataStoreInMemoryRegion)(size_t /*topicSize*/, xme_core_dataManager_memoryRegionID_t /*memoryRegion*/, xme_core_dataManager_dataStoreID_t* /*dataStoreID*/); ///< Declares a data store.
    struct xme_core_dataHandler_databaseBuilder* (*setQueueSize)(xme_core_dataManager_dataStoreID_t /*dataStoreID*/, uint32_t /*queueSize*/); ///< Associates a queue size to a data store. 
    struct xme_core_dataHandler_databaseBuilder* (*setPersistency)(xme_core_dataManager_dataStoreID_t /*dataStoreID*/, bool /*persistent*/); ///< Determines if the data store is persistent or not. 
    struct xme_core_dataHandler_databaseBuilder* (*setOverwrite)(xme_core_dataManager_dataStoreID_t /*dataStoreID*/, bool /*overwrite*/, xme_status_t /*statusOnOverwrite*/, bool /*displayWarning*/); ///< Determines if the data store is persistent or not. 
    struct xme_core_dataHandler_databaseBuilder* (*createAttribute)(xme_core_dataManager_dataStoreID_t /*dataStoreID*/, uint32_t /*key*/, size_t /*attributeSize*/); ///< Adds an attribute to a given data store. 

    /* - Build() */
    xme_core_dataHandler_database_t* (*build)(void); ///< Builds the database with provided parameters. 
};

/**
 * \typedef xme_core_dataHandler_databaseBuilder_t
 * \brief Defines the database builder object. 
 */
typedef struct xme_core_dataHandler_databaseBuilder xme_core_dataHandler_databaseBuilder_t;

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief Creates a new database builder object for the data handler. 
 * \details Generates a new struct that will be used to build a new database. 
 *          The management of the database builder is responsibility of 
 *          the data handler. 
 * \note If the obtained struct is NULL, the database builder cannot be created. 
 *
 * \return The reference to the database builder object. This object contains necessary
 *         callbacks to add data packets, attributes and configurations of the database.
 */
xme_core_dataHandler_databaseBuilder_t*
xme_core_dataHandler_databaseBuilder_create(void);

/**
 * \brief Destroys a database and frees resources associated to this builder. 
 * \details Given the reference of a database builder, destroys the reference
 *          and frees all resources associated to the database. 
 *
 * \param[in] databaseBuilder The database builder pointer. 
 *
 * \retval XME_STATUS_SUCCESS if the database builder has removed all resources
 *         associated to that database builder.
 * \retval XME_STATUS_INVALID_PARAMETER if the database builder is not a valid
 *         database builder. 
 */
xme_status_t
xme_core_dataHandler_databaseBuilder_destroy
(
    xme_core_dataHandler_databaseBuilder_t* databaseBuilder
);

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_DATABASEBUILDER_H */

