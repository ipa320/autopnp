/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: dataManagerTypes.h 5048 2013-09-12 08:07:59Z camek $
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

typedef uint32_t xme_core_dataManager_dataPacketId_t;

#if 0
const xme_core_dataManager_dataPacketId_t XME_CORE_DATAMANAGER_DATAPACKETID_INVALID = 0U; ///< Invalid data packet id.
const xme_core_dataManager_dataPacketId_t XME_CORE_DATAMANAGER_DATAPACKETID_MAX = XME_MAX_SYSTEM_VALUE; ///< Largest possible data packet id.
#else
#define XME_CORE_DATAMANAGER_DATAPACKETID_INVALID 0U ///< Invalid data packet id.
#define XME_CORE_DATAMANAGER_DATAPACKETID_MAX XME_MAX_SYSTEM_VALUE ///< Largest possible data packet id.
#endif

#endif /* XME_CORE_DATAMANAGERTYPES_H */
