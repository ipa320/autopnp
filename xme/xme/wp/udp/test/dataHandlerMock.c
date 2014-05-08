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
 * $Id: dataHandlerMock.c 7664 2014-03-04 08:47:41Z geisinger $
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

#include "xme/core/dataHandler/include/dataHandler.h"

#include <stdbool.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/


/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief Storage for port data (only "topic" data, no attributes) of this mock.
 *        First index corresponds to dataPacketId.
 */
char portArray[5][100] = { {'\0'}, {'\0'}, {'\0'}, {'\0'}, {'\0'} };

/**
 * \brief Storage for attribute data of this mock.
 *        First index corresponds to dataPacketId.
 *
 * \details When there are multiple attributes for a single port, the
 *          attribute data will be appended. The size and position for each entry
 *          are determined from attributeSizes.
 *          
 */
uint8_t attributeStorage[10][50];

/**
 * \brief Expected attribute sizes.
 *        Index corresponds to attributeKey.
 */
uint32_t attributeSizes[10] = { 0, 4, 8, 0, 0, 0, 0, 0, 0, 0 };

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    const void* const buffer,
    size_t bufferSizeInBytes
)
{
    xme_hal_mem_copy(portArray[dataPacketID], buffer, bufferSizeInBytes);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_startWriteOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    XME_UNUSED_PARAMETER(dataPacketID);

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
    xme_core_dataManager_dataPacketId_t dataPacketID,
    void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t * const readBytes
)
{
    xme_hal_mem_copy(buffer, portArray[dataPacketID], bufferSizeInBytes);
    *readBytes = bufferSizeInBytes;

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_startReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    XME_UNUSED_PARAMETER(dataPacketID);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    XME_UNUSED_PARAMETER(dataPacketID);

    return XME_STATUS_SUCCESS;
}

/**
 * \brief Writes attribute into attributeStorage.
 */
xme_status_t
xme_core_dataHandler_writeAttribute
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t key,
    const void* const buffer,
    size_t bufferSizeInBytes
)
{
    uint8_t* ptr;

    XME_ASSERT(dataPacketID <= (sizeof(portArray) / sizeof(portArray[0])));
    XME_ASSERT(key <= (sizeof(attributeStorage) / sizeof(attributeStorage[0])));

    ptr = (uint8_t*)&attributeStorage[dataPacketID] + attributeSizes[key];
    xme_hal_mem_copy(ptr, buffer, bufferSizeInBytes);

    return XME_STATUS_SUCCESS;
}

/**
 * \brief Reads attribute from attributeStorage.
 */
xme_status_t
xme_core_dataHandler_readAttribute
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t key,
    void* const buffer,
    size_t bufferSizeInBytes,
    uint32_t* const bytesRead
)
{
    uint8_t* ptr;

    XME_ASSERT(dataPacketID <= (sizeof(portArray) / sizeof(portArray[0])));
    XME_ASSERT(key <= (sizeof(attributeStorage) / sizeof(attributeStorage[0])));

    ptr = (uint8_t*)&attributeStorage[dataPacketID] + attributeSizes[key];
    xme_hal_mem_copy(buffer, ptr, bufferSizeInBytes);
    *bytesRead = bufferSizeInBytes;

    return XME_STATUS_SUCCESS;
}
