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
 * $Id: databaseConfigurator.c 7702 2014-03-06 23:22:57Z ruiz $
 */

/**
 * \file
 *         Database Abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/dataHandler/internal/databaseConfigurator.h"

#include "xme/core/log.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/mmap.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \brief The last assigned data packet identifier. 
 */
xme_core_dataManager_dataPacketId_t lastAssignedDataPacketID = (xme_core_dataManager_dataPacketId_t) XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

/**
 * \brief The last assigned memory region identifier. 
 */
xme_core_dataManager_memoryRegionID_t lastAssignedMemoryRegionID = (xme_core_dataManager_memoryRegionID_t) XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_core_dataHandler_database_t*
xme_core_dataHandler_databaseConfigurator_create(void)
{
    if (NULL == xme_core_dataHandler_database)
    {
        xme_status_t status;

        status = xme_core_dataHandler_databaseInternal_init();
        XME_CHECK(XME_STATUS_SUCCESS == status, NULL);
    }

    return xme_core_dataHandler_database;
}

xme_status_t
xme_core_dataHandler_databaseConfigurator_destroy(void)
{
    xme_core_dataHandler_databaseInternal_fini();

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_databaseConfigurator_configure(void)
{
    XME_ASSERT(NULL != xme_core_dataHandler_database);

    // Check if the database is configured.
    // - The database is not configured if we call for the very first time to configure() from the builder. 
    // - The database is not configured if we have added/removed/updated data stores or memory regions. 
    if (!xme_core_dataHandler_database->configured)
    {
        xme_status_t status;

        // The builder should provide at least one memory region to configure. 
        XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(0U < XME_HAL_TABLE_ITEM_COUNT(xme_core_dataHandler_database->memoryRegions)));

        status = configureMemoryRegions();
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status, 
            XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_WARNING, 
            "[Database] Cannot configure memory regions.\n"
        );

        // Setup and configure data stores. 
        status = configureDataStores();
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status, 
            XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_WARNING, 
            "[Database] Cannot configure data stores.\n"
        );

        xme_core_dataHandler_database->configured = true;
    }

    return XME_STATUS_SUCCESS;
}

xme_core_dataManager_dataPacketId_t
xme_core_dataHandler_databaseConfigurator_createDataPacketID(void)
{
    lastAssignedDataPacketID = (xme_core_dataManager_dataPacketId_t) (((xme_maxSystemValue_t)lastAssignedDataPacketID) + 1);

    if (NULL != xme_core_dataHandler_database)
    {
        xme_core_dataHandler_database->configured = false;
    }

    return lastAssignedDataPacketID;
}

xme_core_dataManager_memoryRegionID_t
xme_core_dataHandler_databaseConfigurator_createMemoryRegionID(void)
{
    lastAssignedMemoryRegionID = (xme_core_dataManager_memoryRegionID_t) (((xme_maxSystemValue_t)lastAssignedMemoryRegionID) + 1);

    if (NULL != xme_core_dataHandler_database)
    {
        xme_core_dataHandler_database->configured = false;
    }

    return lastAssignedMemoryRegionID;
}
