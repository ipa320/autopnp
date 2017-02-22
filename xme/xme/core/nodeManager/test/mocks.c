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
 * $Id: mocks.c 7844 2014-03-14 14:11:49Z ruiz $
 */

/**
 * \file Mocks for the component repository tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/broker/include/broker.h"
#include "xme/core/broker/include/brokerDataManagerInterface.h"
#include "xme/core/broker/include/brokerPnpManagerInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
extern uint32_t xme_core_nodeMgr_compRep_test_brokerRegisterFunctionCallCount;

extern uint32_t xme_core_nodeMgr_compRep_test_brokerRemoveFunctionCallCount;

extern uint32_t xme_core_nodeMgr_compRep_test_dataHandlerCreatePortCallCount;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     Broker Mock                                                            //
//----------------------------------------------------------------------------//
xme_status_t
xme_core_broker_registerFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);

    xme_core_nodeMgr_compRep_test_brokerRegisterFunctionCallCount++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_addDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{
    XME_UNUSED_PARAMETER(srcDataPacketId);
    XME_UNUSED_PARAMETER(dstDataPacketId);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{
    XME_UNUSED_PARAMETER(srcDataPacketId);
    XME_UNUSED_PARAMETER(dstDataPacketId);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);

    xme_core_nodeMgr_compRep_test_brokerRemoveFunctionCallCount++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_dataAvailabilityChange
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint8_t size
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(size);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_addDataPacketToFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    bool mandatory
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);
    XME_UNUSED_PARAMETER(mandatory);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeDataPacketFromFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);

    return XME_STATUS_SUCCESS;
}

bool
xme_core_broker_isFunctionRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);

    return true;
}

bool
xme_core_broker_isFunctionVariantRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);

    return true;
}

xme_status_t
xme_core_broker_getFunctionDataPackets
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    xme_hal_linkedList_descriptor_t* dataPacketsList
)
{
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
    XME_UNUSED_PARAMETER(functionVariantId);
    XME_UNUSED_PARAMETER(dataPacketsList);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_getDataPacketFunctions
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_hal_linkedList_descriptor_t* functionsList
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(functionsList);

    return XME_STATUS_SUCCESS;
}

//----------------------------------------------------------------------------//
//     Data Handler Mock                                                      //
//----------------------------------------------------------------------------//
xme_status_t
xme_core_dataHandler_createDataPacket
(
    size_t dataPacketSizeInBytes,
    xme_core_dataManager_dataPacketId_t* dataPacketID
)
{
    XME_UNUSED_PARAMETER(dataPacketSizeInBytes);

    xme_core_nodeMgr_compRep_test_dataHandlerCreatePortCallCount++;

    *dataPacketID = (xme_core_dataManager_dataPacketId_t)1;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_createAttribute
(
    size_t attributeSizeInBytes,
    uint32_t attributeKey,
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    XME_UNUSED_PARAMETER(attributeSizeInBytes);
    XME_UNUSED_PARAMETER(attributeKey);
    XME_UNUSED_PARAMETER(dataPacketID);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_configure(void)
{
    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_dataHandler_init(void)
{
    return XME_STATUS_SUCCESS;
}

void
xme_core_dataHandler_fini(void)
{
    
}

xme_status_t
xme_core_dataHandler_setDataPacketOverwrite
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    bool overwrite,
    xme_status_t statusOnOverwrite,
    bool displayWarning
)
{
    XME_UNUSED_PARAMETER(dataPacketID);
    XME_UNUSED_PARAMETER(overwrite);
    XME_UNUSED_PARAMETER(statusOnOverwrite);
    XME_UNUSED_PARAMETER(displayWarning);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setDataPacketQueueSize
(
    xme_core_dataManager_dataPacketId_t dataPacketID,
    uint32_t queueSize
)
{
    XME_UNUSED_PARAMETER(dataPacketID);
    XME_UNUSED_PARAMETER(queueSize);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setDataPacketPersistent
(
    xme_core_dataManager_dataPacketId_t dataPacketID
)
{
    XME_UNUSED_PARAMETER(dataPacketID);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setNumberOfDatabases
(
    uint16_t numberOfDatabases
)
{
    XME_UNUSED_PARAMETER(numberOfDatabases);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandler_setDatabaseSize
(
    size_t sizeInBytes
)
{
    XME_UNUSED_PARAMETER(sizeInBytes);

    return XME_STATUS_SUCCESS;
}

