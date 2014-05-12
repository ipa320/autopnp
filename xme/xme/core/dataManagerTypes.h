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
 * $Id: dataManagerTypes.h 7828 2014-03-14 09:32:09Z ruiz $
 */

#ifndef XME_CORE_DATAMANAGERTYPES_H
#define XME_CORE_DATAMANAGERTYPES_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include <stdint.h>
#include "xme/defines.h"

//******************************************************************************//
//***   Defines                                                              ***//
//******************************************************************************//

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//
/**
 * \typedef xme_core_dataManager_dataPacketId_t
 *
 * \brief  Locally valid identifier of a data packet.
 */
/*
typedef enum
{
    XME_CORE_DATAMANAGER_DATAPACKETID_INVALID = 0, ///< Invalid data packet id.
    XME_CORE_DATAMANAGER_DATAPACKETID_MAX = XME_MAX_SYSTEM_VALUE ///< Largest possible data packet id.
}
xme_core_dataManager_dataPacketId_t;
*/

#if 0
typedef uint32_t xme_core_dataManager_dataPacketId_t;

#if 0
const xme_core_dataManager_dataPacketId_t XME_CORE_DATAMANAGER_DATAPACKETID_INVALID = 0U; ///< Invalid data packet id.
const xme_core_dataManager_dataPacketId_t XME_CORE_DATAMANAGER_DATAPACKETID_MAX = XME_MAX_SYSTEM_VALUE; ///< Largest possible data packet id.
#else
#define XME_CORE_DATAMANAGER_DATAPACKETID_INVALID 0U ///< Invalid data packet id.
#define XME_CORE_DATAMANAGER_DATAPACKETID_MAX XME_MAX_SYSTEM_VALUE ///< Largest possible data packet id.
#endif
#endif

/**
 * \enum xme_core_dataManager_dataPacketId_t
 * \typedef Defines the possible values of memory regions identifiers. 
 */
enum xme_core_dataManager_dataPacketID_e
{
    XME_CORE_DATAMANAGER_DATAPACKETID_INVALID = 0U, ///< The invalid data packet identifier. 
    XME_CORE_DATAMANAGER_DATAPACKETID_MAX = XME_MAX_SYSTEM_VALUE, ///< Largest possible data packet identifier. 
};

/**
 * \typedef xme_core_dataManager_dataStoreID_t
 * \brief Defines the data store identifier for interactions with the Data Handler. 
 */
typedef uint32_t xme_core_dataManager_dataStoreID_t;

/**
 * \typedef xme_core_dataManager_dataPacketId_t
 * \brief Defines the data packet identifier for interactions with Data Handler. 
 * \deprecated This type is depprecated, use the equivalent type xme_core_dataManager_dataPacketID_t instead!
 */
typedef xme_core_dataManager_dataStoreID_t xme_core_dataManager_dataPacketId_t;

/**
 * \enum xme_core_dataManager_memoryRegionID_e
 * \typedef Defines the possible values of memory regions identifiers. 
 */
enum xme_core_dataManager_memoryRegionID_e
{
    XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID = 0U, ///< The invalid memory region. 
    XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT = 1U, ///< The default memory region identifier, when nothing was specified at data handler level. 
    XME_CORE_DATAMANAGER_MEMORYREGIONID_MAX = XME_MAX_SYSTEM_VALUE, ///< Largest possible memory region identifier.  
};

/**
 * \typedef xme_core_dataManager_memoryRegionID_t
 * \brief Defines the memory region indentifiers inside the data handler. 
 */
typedef uint32_t xme_core_dataManager_memoryRegionID_t;

#endif /* XME_CORE_DATAMANAGERTYPES_H */
