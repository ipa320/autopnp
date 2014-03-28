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
 *         Channel waypoint test Data Handler mock.
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
xme_core_dataHandler_createDataPacket
(
    int bufferSize,
    xme_core_dataManager_dataPacketId_t * const dataPacketId
);

xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t port
);

xme_status_t
xme_core_dataHandler_writeAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void const * const buffer,
    uint32_t bufferSize
);

xme_status_t
xme_core_dataHandler_readAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead
);
xme_status_t
xme_core_directory_attribute_getAttributeDescriptorList(xme_core_topic_t topic, xme_core_attribute_descriptor_list_t *attributeDescriptorList);

xme_status_t
xme_core_dataHandler_configure(void);

xme_status_t
xme_core_dataHandler_startWriteOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

xme_status_t
xme_core_dataHandler_startReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
);

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static char portArray[5][100] = { {'\0'}, {'\0'}, {'\0'}, {'\0'}, {'\0'} };

static int portArrayIndex = 1;

#define DATASIZE 10

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_dataHandler_createDataPacket
(
    int bufferSize,
    xme_core_dataManager_dataPacketId_t * const dataPacketId
)
{
    XME_UNUSED_PARAMETER(bufferSize);

    *dataPacketId = (xme_core_dataManager_dataPacketId_t) portArrayIndex++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_configure(void)
{
    return XME_STATUS_SUCCESS;
}

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

xme_status_t
xme_core_dataHandler_writeAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void const * const buffer,
    uint32_t bufferSize
)
{
    if ( (xme_core_attribute_key_t) XME_CORE_ATTRIBUTE_KEY_CHANNELID == attributeKey )
    {
        xme_hal_mem_copy(portArray[port]+DATASIZE, (void *)&attributeKey, sizeof(xme_core_attribute_key_t));
        xme_hal_mem_copy(portArray[port]+DATASIZE+sizeof(xme_core_attribute_key_t), buffer, bufferSize);
    }
    else
    {
        xme_hal_mem_copy(portArray[port]+DATASIZE+sizeof(xme_core_attribute_key_t)+sizeof(uint32_t), (void *)&attributeKey, sizeof(xme_core_attribute_key_t));
        xme_hal_mem_copy(portArray[port]+DATASIZE+sizeof(xme_core_attribute_key_t)+sizeof(uint32_t)+sizeof(xme_core_attribute_key_t), buffer, bufferSize);
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_readAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead
)
{
    if ( (xme_core_attribute_key_t) XME_CORE_ATTRIBUTE_KEY_CHANNELID == attributeKey )
    {
        //xme_hal_mem_copy(portArray[port]+DATASIZE, (void *)&attributeKey, sizeof(xme_core_attribute_key_t));
        xme_hal_mem_copy(buffer, portArray[port]+DATASIZE+sizeof(xme_core_attribute_key_t), bufferSize);
    }
    else
    {
        //xme_hal_mem_copy(portArray[port]+DATASIZE+sizeof(xme_core_attribute_key_t)+sizeof(uint32_t), (void *)&attributeKey, sizeof(xme_core_attribute_key_t));
        xme_hal_mem_copy(buffer, portArray[port]+DATASIZE+sizeof(xme_core_attribute_key_t)+sizeof(uint32_t)+sizeof(xme_core_attribute_key_t), bufferSize);
    }
    *bytesRead = bufferSize;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_attribute_getAttributeDescriptorList(xme_core_topic_t topic, xme_core_attribute_descriptor_list_t *attributeDescriptorList)
{
    XME_UNUSED_PARAMETER(topic);
    attributeDescriptorList->length = 2;
    attributeDescriptorList->element = (xme_core_attribute_descriptor_t *) xme_hal_mem_alloc(sizeof(xme_core_attribute_descriptor_t) * 2);
    attributeDescriptorList->element[0].key = (xme_core_attribute_key_t) XME_CORE_ATTRIBUTE_KEY_CHANNELID;
    attributeDescriptorList->element[0].size = sizeof(uint32_t);
    attributeDescriptorList->element[1].key = (xme_core_attribute_key_t) (XME_CORE_ATTRIBUTE_KEY_CHANNELID+1);
    attributeDescriptorList->element[1].size = sizeof(uint64_t);

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
xme_core_dataHandler_startReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    XME_UNUSED_PARAMETER(dataPacketID);
    return XME_STATUS_SUCCESS;
}

