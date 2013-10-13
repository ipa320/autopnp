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
 * $Id: dataHandlerMock.c 4597 2013-08-07 14:18:28Z ruiz $
 */

/**
 * \file
 *         A mock of xme_core_dataHandler.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/mem.h"

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/core/log.h"

#include "xme/defines.h"

#include <stdbool.h>

XME_EXTERN_C_BEGIN

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \brief  Length of the arrays used to store the call count per port.
 */
#define XME_ADV_WRAPPERTEST_DATAHANDLERMOCK_ARRAY_LENGTH 10

/******************************************************************************/
/***   Global Variables                                                     ***/
/******************************************************************************/
/**
 * \brief  This arrays stores how often the xme_core_dataHandler_writeData function
 *         has been called. The index corresponds to the packetId of a port, the
 *         value to call count for that port.
 */
uint8_t xme_adv_wrapperTest_dataHandlerMock_writeDataCountArray[XME_ADV_WRAPPERTEST_DATAHANDLERMOCK_ARRAY_LENGTH] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/**
 * \brief  This arrays stores how often the xme_core_dataHandler_readData function
 *         has been called. The index corresponds to the packetId of a port, the
 *         value to call count for that port.
 */
uint8_t xme_adv_wrapperTest_dataHandlerMock_readDataCountArray[XME_ADV_WRAPPERTEST_DATAHANDLERMOCK_ARRAY_LENGTH] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/**
 * \brief  This arrays stores how often the xme_core_dataHandler_completeWriteOperation function
 *         has been called. The index corresponds to the packetId of a port, the
 *         value to call count for that port.
 */
uint8_t xme_adv_wrapperTest_dataHandlerMock_completeWriteOperationCountArray[XME_ADV_WRAPPERTEST_DATAHANDLERMOCK_ARRAY_LENGTH] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/**
 * \brief  This arrays stores how often the xme_core_dataHandler_completeReadOperation function
 *         has been called. The index corresponds to the packetId of a port, the
 *         value to call count for that port.
 */
uint8_t xme_adv_wrapperTest_dataHandlerMock_completeReadOperationCountArray[XME_ADV_WRAPPERTEST_DATAHANDLERMOCK_ARRAY_LENGTH] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Mocked function. Does nothing except to increase call count in
 *         xme_adv_wrapperTest_dataHandlerMock_writeDataCountArray for given
 *         port.
 */
xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    unsigned int bufferSize
);

/**
 * \brief  Mocked function. Does nothing except to increase call count in
 *         xme_adv_wrapperTest_dataHandlerMock_readDataCountArray for given
 *         port.
 */
xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t port,
    void * const buffer,
    unsigned int bufferSize,
    unsigned int * const bytesRead
);

/**
 * \brief  Mocked function. Does nothing except to increase call count in
 *         xme_adv_wrapperTest_dataHandlerMock_completeWriteOperationCountArray
 *         for given port.
 */
xme_status_t 
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t port
);

/**
 * \brief  Mocked function. Does nothing except to increase call count in
 *         xme_adv_wrapperTest_dataHandlerMock_completeReadOperationCountArray
 *         for givenport.
 */
xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t port
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    unsigned int bufferSize
)
{
    XME_UNUSED_PARAMETER(buffer);
    XME_UNUSED_PARAMETER(bufferSize);

    XME_CHECK_MSG
    (
        XME_ADV_WRAPPERTEST_DATAHANDLERMOCK_ARRAY_LENGTH > port,
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_ERROR,
        "%s:%d: Given port %d is greater than array length for port count.",
        __FILE__,
        __LINE__,
        port
    );

    xme_adv_wrapperTest_dataHandlerMock_writeDataCountArray[port]++;

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
    XME_UNUSED_PARAMETER(buffer);
    XME_UNUSED_PARAMETER(bufferSize);
    XME_UNUSED_PARAMETER(bytesRead);

    XME_CHECK_MSG
    (
        XME_ADV_WRAPPERTEST_DATAHANDLERMOCK_ARRAY_LENGTH > port,
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_ERROR,
        "%s:%d: Given port %d is greater than array length for port count.",
        __FILE__,
        __LINE__,
        port
    );

    xme_adv_wrapperTest_dataHandlerMock_readDataCountArray[port]++;

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_completeWriteOperation
(
    xme_core_dataManager_dataPacketId_t port
)
{
    XME_CHECK_MSG
    (
        XME_ADV_WRAPPERTEST_DATAHANDLERMOCK_ARRAY_LENGTH > port,
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_ERROR,
        "%s:%d: Given port %d is greater than array length for port count.",
        __FILE__,
        __LINE__,
        port
    );

    xme_adv_wrapperTest_dataHandlerMock_completeWriteOperationCountArray[port]++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_completeReadOperation
(
    xme_core_dataManager_dataPacketId_t port
)
{
    XME_CHECK_MSG
    (
        XME_ADV_WRAPPERTEST_DATAHANDLERMOCK_ARRAY_LENGTH > port,
        XME_STATUS_INTERNAL_ERROR,
        XME_LOG_ERROR,
        "%s:%d: Given port %d is greater than array length for port count.",
        __FILE__,
        __LINE__,
        port
    );

    xme_adv_wrapperTest_dataHandlerMock_completeReadOperationCountArray[port]++;

    return XME_STATUS_SUCCESS;
}

XME_EXTERN_C_END
