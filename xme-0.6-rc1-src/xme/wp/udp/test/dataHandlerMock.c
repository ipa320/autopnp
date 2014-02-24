/*
 * Copyright (c) 2011-2013, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: dataHandlerMock.c 4663 2013-08-13 08:50:59Z geisinger $
 */

/**
 * \file
 *         UDP Waypoint test Data Handler mock.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

#include "xme/hal/include/mem.h"

#include <stdbool.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
xme_status_t
xme_core_dataHandler_createPort
(
    xme_core_component_t componentID,
    xme_core_component_portType_t type,
    xme_core_topic_t topic,
    int bufferSize,
    xme_core_attribute_descriptor_list_t metadata,
    int queueSize,
    bool overwrite,
    bool persistent,
    int historyDepth,
    xme_core_dataManager_dataPacketId_t * const dataPacketId
);

xme_status_t
xme_core_dataHandler_writeData
(
        xme_core_dataManager_dataPacketId_t port,
        void const * const buffer,
        unsigned int bufferSize
);

xme_status_t
xme_core_dataHandler_completeWriteOperation
(
        xme_core_dataManager_dataPacketId_t port
);

xme_status_t
xme_core_dataHandler_readData
(
        xme_core_dataManager_dataPacketId_t port,
        void * const buffer,
        unsigned int bufferSize,
        unsigned int * const bytesRead
);

xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t port
);

/******************************************************************************/
/***   Static variables                                                     ***/
/******************************************************************************/
static char portArray[5][100] = { {'\0'}, {'\0'}, {'\0'}, {'\0'}, {'\0'} };

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_dataHandler_createPort
(
    xme_core_component_t componentID,
    xme_core_component_portType_t type,
    xme_core_topic_t topic,
    int bufferSize,
    xme_core_attribute_descriptor_list_t metadata,
    int queueSize,
    bool overwrite,
    bool persistent,
    int historyDepth,
    xme_core_dataManager_dataPacketId_t * const dataPacketId
)
{
    XME_UNUSED_PARAMETER(componentID);
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(bufferSize);
    XME_UNUSED_PARAMETER(metadata);
    XME_UNUSED_PARAMETER(queueSize);
    XME_UNUSED_PARAMETER(overwrite);
    XME_UNUSED_PARAMETER(persistent);
    XME_UNUSED_PARAMETER(historyDepth);

    if (type == XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION)
    {
        *dataPacketId = (xme_core_dataManager_dataPacketId_t) 1;
    }
    else
    {
        *dataPacketId = (xme_core_dataManager_dataPacketId_t) 2;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    unsigned int bufferSize
)
{
    xme_hal_mem_copy(portArray[port], buffer, bufferSize);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t port
)
{
    XME_UNUSED_PARAMETER(port);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t port,
    void * const buffer,
    unsigned int bufferSize,
    unsigned int * const bytesRead
)
{
    xme_hal_mem_copy(buffer, portArray[port], bufferSize);
    *bytesRead = bufferSize;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t port
)
{
    XME_UNUSED_PARAMETER(port);

    return XME_STATUS_SUCCESS;
}
