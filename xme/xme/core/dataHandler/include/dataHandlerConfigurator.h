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
 * $Id: dataHandlerConfigurator.h 7844 2014-03-14 14:11:49Z ruiz $
 */

/**
 * \file
 *         Data Handler Configurator.
 */

#ifndef XME_CORE_DATAHANDLER_DATAHANDLERCONFIGURATOR_H
#define XME_CORE_DATAHANDLER_DATAHANDLERCONFIGURATOR_H

/**
 * \addtogroup core_dataHandler
 * @{
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

#include <stdbool.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN
/**
 * \brief Initializes the data handler. 
 * \details Initializes the initial structure of data handler.
 * \note Data Handler can be initialized as many times as needed. The initialization
 *       will maintain the data handler stateful status, as far as the ::xme_core_dataHanlder_fini
 *       method is not called. 
 *
 * \retval XME_STATUS_SUCCESS if the initialization of data handler was success. 
 * \retval XME_STATUS_INTERNAL_ERROR if the data handler cannot be initialized due to internal
 *         problems in XME (see log for further details). 
 */
xme_status_t 
xme_core_dataHandler_init(void);

/**
 * \brief Finalizes the information associated to the Data Handler.
 * \details Frees all resources associated to the data handler, including the elimination
 *          of occupied resources by the data handler and its associated databases. 
 */
void
xme_core_dataHandler_fini(void);

/**
 * \brief Creates a memory region in which will be placed the data. 
 * \details Specifies a new memory region.
 * \note The generic case is to have only one default memory region. This method
 *       is mandatory to be called at least once.
 *
 * \param[out] memoryRegionID the output parameter indicating the memory region identifier.
 *
 * \retval XME_STATUS_SUCCESS if the memory region was successfully reserved. 
 * \retval XME_STATUS_INVALID_PARAMETER when provided parameters are invalid. 
 * \retval XME_STATUS_INTERNAL_ERROR if some error arised during the configuration. 
 */
xme_status_t
xme_core_dataHandler_createMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t* memoryRegionID
);

/**
 * \brief Establishes the number of databases.
 * \details Defines the number of databases associated to the data. 
 *          These databases will be later used to switch between them
 *          for testing purposes. 
 * \note By default, we assume that one database is always created.
 *       This default database is always active. 
 *
 * \param[in] numberOfCopies The number of databases to define.
 *
 * \retval XME_STATUS_SUCCESS If the number of databases associated to
 *         the current data is successfully established.
 * \retval XME_STATUS_INVALID_PARAMETER If the provided number of databases
 *         is zero-valued. 
 */
xme_status_t
xme_core_dataHandler_setNumberOfDatabases
(
    uint16_t numberOfCopies
);

/**
 * \brief Establishes the number of databases for a determined memory region.
 * \details Defines the number of memory region copies associated to a memory region. 
 *          These copies will be later used to switch between them
 *          for testing purposes. 
 *
 * \param[in] memoryRegionID The memory region identifier.
 * \param[in] numberOfCopies The number of databases to define.
 *
 * \retval XME_STATUS_SUCCESS If the number of databases associated to
 *         the current data is successfully established.
 * \retval XME_STATUS_INVALID_PARAMETER If the provided number of databases
 *         is zero-valued. 
 */
xme_status_t
xme_core_dataHandler_setNumberOfCopiesInMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    uint16_t numberOfCopies
);

/**
 * \brief Establishes the current database size. 
 * \details For embedded systems, the database size should be established
 *          beforehand. This value should be used during the database memory
 *          allocation process.
 * \note The database size should include the size to be reserved for
 *       databases associated to the current database. (find right name: maybe mem region and database). 
 * \note The database size can only be established during build time. 
 * \note The database size is assigned for a single database. For each associate database,
 *       it will be used the quantity of memory resulting from multiplying the database size
 *       provided in this function by the number of databases.
 *
 * \param[in] sizeInBytes The size in bytes of the database. 
 *
 * \retval XME_STATUS_SUCCESS If the database size was successfully established. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If there is not an active builder for the database. 
 */
xme_status_t
xme_core_dataHandler_setDatabaseSize
(
    size_t sizeInBytes
);

/**
 * \brief Establishes the size for the given memory region. 
 * \details For embedded systems, the database size should be established
 *          beforehand. This value should be used during the database memory
 *          allocation process.
 * \note The database size should include the size to be reserved for
 *       databases associated to the current database. (find right name: maybe mem region and database). 
 * \note The database size can only be established during build time. 
 * \note The database size is assigned for a single database. For each associate database,
 *       it will be used the quantity of memory resulting from multiplying the database size
 *       provided in this function by the number of databases.
 *
 * \param[in] memoryRegionID The memory region identifier.
 * \param[in] sizeInBytes The size in bytes of the database. 
 *
 * \retval XME_STATUS_SUCCESS If the database size was successfully established. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If there is not an active builder for the database. 
 */
xme_status_t
xme_core_dataHandler_setDatabaseSizeInMemoryRegion
(
    xme_core_dataManager_memoryRegionID_t memoryRegionID,
    size_t sizeInBytes
);

/**
 * \brief Defines a data packet for the current builder or the active database. 
 * \details Specifies a new data packet associated to the current builder 
 *          (if the builder has not still been configured) or either to the
 *          active database (if the builder is already configured and it is needed
 *          to define dynamic data packets as in Plug and play). 
 * \note For this method we should provide just the data packet size. 
 *
 * \param[in] dataPacketSizeInBytes The size expressed in bytes for the data to be
 *            stored in the data packet.
 * \param[out] dataPacketID the output parameter indicating the data packet identifier.
 *
 * \retval XME_STATUS_SUCCESS if the data packet was successfully reserved. 
 * \retval XME_STATUS_INVALID_PARAMETER when provided parameters are invalid. 
 * \retval XME_STATUS_INTERNAL_ERROR if some error arised during the configuration. 
 */
xme_status_t
xme_core_dataHandler_createDataPacket
(
    size_t dataPacketSizeInBytes,
    xme_core_dataManager_dataPacketId_t* dataPacketID
);

/**
 * \brief Establish the data packet queue size. 
 *
 * \param[in] dataPacketID the data packet identifier.
 * \param[in] queueSize The number of elements to store in the queue. 
 *
 * \retval XME_STATUS_SUCCESS If the data packet is associated to
 *         a queue. 
 * \retval XME_STATUS_INVALID_PARAMETER when the queue size is zero.
 * \retval XME_STATUS_INVALID_HANDLE If the provided data packet ID is not a valid
 *         identifier. 
 * \retval XME_STATUS_INTERNAL_ERROR if some error arised during the configuration. 
 */
xme_status_t
xme_core_dataHandler_setDataPacketQueueSize
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t queueSize
);

/**
 * \brief Establish the data packet persistency flag. 
 *
 * \param[in] dataPacketID the data packet identifier.
 * \param[in] persistency A boolean variable defining the persistence.
 *
 * \retval XME_STATUS_SUCCESS If the data packet persistency flag was established.
 * \retval XME_STATUS_INVALID_HANDLE If the provided data packet ID is not a valid
 *         identifier. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the data handler configuration has not been properly established.
 */
xme_status_t
xme_core_dataHandler_setDataPacketPersistency
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    bool persistency
);

/**
 * \brief Old method for establishing persistency.
 * \param[in] dataPacketID the data packet identifier.
 * \deprecated Use ::xme_core_dataHandler_setDataPacketPersistency instead.
 */
xme_status_t
xme_core_dataHandler_setDataPacketPersistent
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Establish the data packet overwrite flag.
 *
 * \details If the overwrite flag is true, the overwrite takes place, and replaces the old value in the database.
 *          If the overwrite flag is false, the overwrite does not take place, and the old value remains in the database.
 *
 * \param[in] dataPacketID The data packet identifier.
 * \param[in] overwrite The boolean overwrite flag.
 * \param[in] statusOnOverwrite The value to return if an overwrite takes place.
 * \param[in] displayWarning A boolean value which decides if a warning message should be displayed when trying to overwrite.
 *
 * \retval XME_STATUS_SUCCESS If the data packet overwrite flag is established.
 * \retval XME_STATUS_INVALID_HANDLE If the provided data packet ID is not a valid
 *         identifier. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the data handler configuration has not been properly established.
 */
xme_status_t
xme_core_dataHandler_setDataPacketOverwrite
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    bool overwrite,
    xme_status_t statusOnOverwrite,
    bool displayWarning
);

/**
 * \brief Creates an attribute associated to a data packet.
 * \details An attribute is the set of associated elements to a given data port. 
 *          This associates an attribute to the data  is the data that should be made available for reading/writing
 *          operations. The data port creation assignes a position in the database
 *          to store information there. 
 * \note This function can be only called when the function ::xme_core_dataHandler_createPortData
 *       is called before. If not called, this function will return XME_STATUS_OUT_OF_RESOURCES
 *       because the data packet is coincident with port data. 
 * \note The available amount of free space for attribute definition
 *       relies on the available data packet size. The calculation is made as
 *       follows: dataPacketSize - portDataSize - previouslyDefinedAttributesSize. 
 *
 * \param[in] attributeSizeInBytes The size in bytes associated to the attribute value. 
 * \param[in] attributeKey The attribute key. 
 * \param[in] dataPacketID The target data packet in the database. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute association to the data packet is success.
 * \retval XME_STATUS_INVALID_PARAMETER if some of the input parameters are not valid
 *         parameters. 
 * \retval XME_STATUS_INVALID_HANDLE If the provided data packet ID is not a valid
 *         identifier. 
 * \retval XME_STATUS_INVALID_CONFIGURATION If the data handler configuration has not been properly established.
 */
// FIXME: The other configurator methods have the dataPacketID as first parameter; it should be made consistent here as well!
xme_status_t
xme_core_dataHandler_createAttribute
(
    size_t attributeSizeInBytes,
    uint32_t attributeKey, // or either xme_core_attribute_key_t key,
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/**
 * \brief Configures the data handler to start read/write operation.
 * \details Configures dynamically the databases to allow working with them. 
 *          This means to allow read/write operations on the data handler. 
 * \note TODO: Add missing information needed for XME users. 
 *
 * \retval XME_STATUS_SUCCESS if the configuration of databases is success. 
 */
xme_status_t
xme_core_dataHandler_configure(void);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_DATAHANDLER_DATAHANDLERCONFIGURATOR_H
