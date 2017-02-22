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
 * $Id: databaseConfigurator.h 7702 2014-03-06 23:22:57Z ruiz $
 */

/**
 * \file
 *         Database Abstraction.
 *
 * \brief The database configurator.
 */

#ifndef XME_CORE_DATAHANDLER_DATABASECONFIGURATOR_H
#define XME_CORE_DATAHANDLER_DATABASECONFIGURATOR_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataHandler/internal/databaseInternalMethods.h"

#include "xme/core/dataManagerTypes.h"

#include <stdint.h>

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \brief The last assigned data packet identifier. 
 */
extern xme_core_dataManager_dataPacketId_t lastAssignedDataPacketID;

/**
 * \brief The last assigned memory region identifier. 
 */
extern xme_core_dataManager_memoryRegionID_t lastAssignedMemoryRegionID;

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief Creates a new database object. 
 * \details Creates a new database object to store all information that is relevant
 *          for creating the database. 
 * \note This method should never be used outside the context of the Data Handler. 
 *       Exposing this header to other components outside the Data Handler could
 *       cause high risks for memory management and data management. 
 *
 * \return The data structure in which the database is created. 
 */
xme_core_dataHandler_database_t*
xme_core_dataHandler_databaseConfigurator_create(void);

/**
 * \brief Destroys the database content and its associated data. 
 * \details Frees all resources associated to the current database, including the storage
 *          structures and the data contained inside the database. Once a database
 *          is destroyed, the data inside the database can not be longer accessed by
 *          the data handler.
 * \note This method should never be used outside the context of the Data Handler. 
 *       Exposing this header to other components outside the Data Handler could
 *       cause high risks for memory management and data management. 
 *
 * \retval XME_STATUS_SUCCESS If the database is successfully destroyed. 
 * \retval XME_STATUS_INVALID_PARAMETER If the provided database is not a valid reference
 *         to a database.
 * \retval XME_STATUS_INVALID_CONFIGURATION If the provided database data does not correspond
 *         to a valid configuration. 
 */
xme_status_t
xme_core_dataHandler_databaseConfigurator_destroy(void);

/**
 * \brief Configures the input database. 
 * \details Creates a new data packet identifier and increments the internal
 *          counter of this identifier. 
 * \note If the database is already initialized, this function will reserve
 *       the corresponding space for new added data packets. 
 * \note If the database is still not initialized, this component will reserve
 *       dynamic memory. 
 *
 * \retval XME_STATUS_SUCCESS If the database has been properly configured. 
 * \retval XME_STATUS_INVALID_PARAMETER If the provided database is an invalid reference.
 * \retval XME_STATUS_OUT_OF_RESOURCES If cannot configure the database due to a lack
 *         of resources of HAL components. 
 */
xme_status_t
xme_core_dataHandler_databaseConfigurator_configure(void);

/**
 * \brief Creates a new data packet identifier.
 * \details Creates a new data packet identifier and increments the internal
 *          counter of this identifier. 
 *
 * \return A new data packet identifier not previously assigned. 
 */
xme_core_dataManager_dataPacketId_t
xme_core_dataHandler_databaseConfigurator_createDataPacketID(void);

/**
 * \brief Creates a new memory region identifier.
 * \details Creates a new memory region identifier and increments the internal
 *          counter of this identifier. 
 *
 * \return A new memory region identifier not previously assigned. 
 */
xme_core_dataManager_memoryRegionID_t
xme_core_dataHandler_databaseConfigurator_createMemoryRegionID(void);

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_DATABASECONFIGURATOR_H */

