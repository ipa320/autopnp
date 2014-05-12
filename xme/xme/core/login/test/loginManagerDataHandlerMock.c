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
 * $Id: loginManagerDataHandlerMock.c 4431 2013-07-31 08:24:55Z ruiz $
 */

/**
 * \file
 *
 * \brief  A mock of xme_core_dataHandler, to be used with the broker
 *         for testing xme_core_broker_notifyOutputDataPacketAvailability.
 *
 * \note This mock will use the following functions:
 *       * DataHandler's completeWriteOperation(). This function will call broker's 
 *         notifyOutputDataPacketAvailability.
 *       * DataHandler's transferData(). This is called directly from the broker
 *         notifyOutputDataPacketAvailability (currently static call). Several
 *         responses to implement. 
 *       * DataHandler completeReadOperation(). This function will "unblock" the port. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/log.h"

#include "xme/core/topicData.h"

#include "xme/hal/include/mem.h"

#include <stdbool.h>

XME_EXTERN_C_BEGIN

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
xme_core_topic_login_loginRequest_t* internalLoginRequest;
xme_core_topic_login_pnpLoginRequest_t* internalPnPLoginRequest;

bool sentOnce = false;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
xme_status_t
xme_core_dataHandlerMock_setDataToRead(void);

xme_status_t
xme_core_dataHandlerMock_getWrittenData
(
    xme_core_topic_login_pnpLoginRequest_t* pnpLoginRequest
);


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t port,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead
)
{
    XME_UNUSED_PARAMETER(port);
    XME_UNUSED_PARAMETER(bufferSize);

    xme_core_dataHandlerMock_setDataToRead();

    if (!sentOnce) 
    {
        xme_hal_mem_copy(buffer, (void*) internalLoginRequest, sizeof(xme_core_topic_login_loginRequest_t));
        *bytesRead = sizeof(xme_core_topic_login_loginRequest_t);
        sentOnce = true;
    }
    else
    {
        return XME_STATUS_INTERNAL_ERROR;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    uint32_t bufferSize
)
{
    XME_UNUSED_PARAMETER(port);
    XME_UNUSED_PARAMETER(buffer);
    XME_UNUSED_PARAMETER(bufferSize);

    xme_core_dataHandlerMock_getWrittenData((xme_core_topic_login_pnpLoginRequest_t*) buffer);

    return XME_STATUS_SUCCESS;

}


xme_status_t 
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_LOG(XME_LOG_DEBUG, "DataHandler: xme_core_dataHandler_completeWriteOperation(%i)\n", dataPacketId);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{

    XME_LOG(XME_LOG_DEBUG, "DataHandler: xme_core_dataHandler_completeReadOperation(%i)\n", dataPacketId);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_init(size_t amountOfMemory)
{
    XME_UNUSED_PARAMETER(amountOfMemory);

    internalLoginRequest = (xme_core_topic_login_loginRequest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_loginRequest_t));

    return XME_STATUS_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////
xme_status_t
xme_core_dataHandlerMock_setDataToRead(void)
{
    char* ip;
    uint16_t port;
    uint32_t ipNetworkByteOrder;
    uint16_t portNetworkByteOrder;
    xme_com_interface_address_t interfaceAddress;

    internalLoginRequest = (xme_core_topic_login_loginRequest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_loginRequest_t));

    internalLoginRequest->guid = 256;
    internalLoginRequest->nodeId = XME_CORE_NODE_INVALID_NODE_ID;
    ip = "192.168.0.1";
    port = 33331u;
    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_ipv4ToGenericAddress(ip, port, &interfaceAddress), XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder), XME_STATUS_INTERNAL_ERROR);
    xme_hal_mem_copy(&internalLoginRequest->ipAddress, &ipNetworkByteOrder, 4);
    xme_hal_mem_copy(&internalLoginRequest->portAddress, &portNetworkByteOrder, 2);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandlerMock_getWrittenData
(
    xme_core_topic_login_pnpLoginRequest_t* pnpLoginRequest
)
{
    internalPnPLoginRequest = (xme_core_topic_login_pnpLoginRequest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginRequest_t));

    xme_hal_mem_copy(internalPnPLoginRequest, pnpLoginRequest, sizeof(xme_core_topic_login_pnpLoginRequest_t));
    return XME_STATUS_SUCCESS;
}

XME_EXTERN_C_END
