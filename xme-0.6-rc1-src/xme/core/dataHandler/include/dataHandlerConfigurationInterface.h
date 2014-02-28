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
 * $Id: dataHandlerConfigurationInterface.h 3754 2013-06-18 13:21:18Z camek $
 */

/**
 * \file
 *         Data Handler.
 */

#ifndef XME_CORE_DATAHANDLER_DATAHANDLER_CONFIGURATION_INTERFACE_H
#define XME_CORE_DATAHANDLER_DATAHANDLER_CONFIGURATION_INTERFACE_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/component.h"
#include "xme/core/topic.h"

#include <stdbool.h>

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN
/**
*/
extern xme_status_t xme_core_dataHandler_dataPacketFeasible(xme_core_topic_t topic,
											xme_core_dataHandler_memoryLocation_t location,
											uint32_t queueSize,
											xme_core_attr_listHandle_t metaData,
											bool overwrite,
											xme_core_dataManager_dataPacketId packet,
											bool persistent,
											uint32_t historyDepth,
											xme_core_directory_transactionId_t transactionId);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DATAHANDLER_DATAHANDLER_CONFIGURATION_INTERFACE_H
