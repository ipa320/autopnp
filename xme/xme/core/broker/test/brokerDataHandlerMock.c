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
 * $Id: brokerDataHandlerMock.c 6514 2014-01-27 23:16:51Z ruiz $
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
#include "xme/core/broker/include/brokerDataManagerInterface.h"
#include "xme/core/log.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"
#include "xme/core/dataHandler/include/dataHandler.h"

#include <stdbool.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define TRANSFERCALLBACK1 1U
#define TRANSFERCALLBACK2 2U
#define TRANSFERCALLBACK3 3U

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

xme_core_transferDataCallback_t
getTransferCallback
(
    uint8_t callbackNumber
);

xme_status_t
xme_core_dataHandler_transferData2
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
);

xme_status_t
xme_core_dataHandler_transferData3
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t 
xme_core_dataHandler_startWriteOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_LOG(XME_LOG_DEBUG, "DataHandler: xme_core_dataHandler_completeWriteOperation(%i)\n", dataPacketId);

    // XME_CORE_STATUS_SUCCESS if a correct read was done.
    // XME_CORE_STATUS_INTERNAL_ERROR
    // XME_CORE_STATUS_INVALID_PARAMETER
    return xme_core_broker_dataAvailabilityChange(dataPacketId, 1);
}

xme_status_t 
xme_core_dataHandler_startReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{

    xme_status_t status = XME_STATUS_SUCCESS;

    XME_LOG(XME_LOG_DEBUG, "DataHandler: xme_core_dataHandler_completeReadOperation(%i)\n", dataPacketId);

    // XME_CORE_STATUS_SUCCESS if a correct read was done.
    // XME_CORE_STATUS_INTERNAL_ERROR
    // XME_CORE_STATUS_INVALID_PARAMETER
    return status;
}

xme_status_t
xme_core_dataHandler_transferData
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{

    //xme_status_t status = XME_STATUS_INTERNAL_ERROR;

    XME_LOG(XME_LOG_DEBUG, "DataHandler: xme_core_dataHandler_transferData(%i, %i)\n", srcDataPacketId, dstDataPacketId);

    return xme_core_broker_dataAvailabilityChange(dstDataPacketId, 1);
    //XME_CORE_STATUS_SUCCESS if the transfer was done correct.
    //XME_CORE_STATUS_PERMISSION_DENIED if one of the specified ports has a wrong type
    //XME_CORE_STATUS_INVALID_CONFIGURATION if a component of this
    //type has already been initialized. Exactly one component of this
    //type must be present on every node.
}

xme_status_t
xme_core_dataHandler_transferData2
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{

    //xme_status_t status = XME_STATUS_INTERNAL_ERROR;

    XME_LOG(XME_LOG_DEBUG, "DataHandler: xme_core_dataHandler_transferData(%i, %i)\n", srcDataPacketId, dstDataPacketId);

    return XME_STATUS_OUT_OF_RESOURCES;
    //XME_CORE_STATUS_SUCCESS if the transfer was done correct.
    //XME_CORE_STATUS_PERMISSION_DENIED if one of the specified ports has a wrong type
    //XME_CORE_STATUS_INVALID_CONFIGURATION if a component of this
    //type has already been initialized. Exactly one component of this
    //type must be present on every node.
}

xme_status_t
xme_core_dataHandler_transferData3
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{

    //xme_status_t status = XME_STATUS_INTERNAL_ERROR;

    XME_LOG(XME_LOG_DEBUG, "DataHandler: xme_core_dataHandler_transferData(%i, %i)\n", srcDataPacketId, dstDataPacketId);

    return XME_STATUS_INVALID_CONFIGURATION;
    //XME_CORE_STATUS_SUCCESS if the transfer was done correct.
    //XME_CORE_STATUS_PERMISSION_DENIED if one of the specified ports has a wrong type
    //XME_CORE_STATUS_INVALID_CONFIGURATION if a component of this
    //type has already been initialized. Exactly one component of this
    //type must be present on every node.
}

xme_status_t 
xme_core_dataHandler_init(void)
{
    // note: this data handler mock uses this init function to test
    //       different implementations of transferData function. 
    //       Additional mechanisms should be provided to configure 
    //       transfer callback in the broker (to be defined). 

    //if (amountOfMemory == (size_t) 1)
    //{
    //    xme_core_broker_setTransferCallback(&xme_core_dataHandler_transferData);
    //}
    //else if (amountOfMemory == (size_t) 2)
    //{
    //    xme_core_broker_setTransferCallback(&xme_core_dataHandler_transferData2);
    //}
    //else if (amountOfMemory == (size_t) 3)
    //{
    //    xme_core_broker_setTransferCallback(&xme_core_dataHandler_transferData3);
    //}

    return XME_STATUS_SUCCESS;
}

xme_core_transferDataCallback_t
getTransferCallback
(
    uint8_t callbackNumber
)
{
    switch (callbackNumber)
    {
        case TRANSFERCALLBACK1:
            return &xme_core_dataHandler_transferData;
        case TRANSFERCALLBACK2:
            return &xme_core_dataHandler_transferData2;
        case TRANSFERCALLBACK3:
            return &xme_core_dataHandler_transferData3;
        default:
            return NULL;
    }
}

