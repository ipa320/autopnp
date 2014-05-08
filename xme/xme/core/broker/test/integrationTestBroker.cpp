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
 * $Id: integrationTestBroker.cpp 7741 2014-03-10 14:01:38Z ruiz $
 */

/**
 * \file
 *         Broker test.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/broker/include/broker.h"
#include "xme/core/broker/include/brokerDataManagerInterface.h"
#include "xme/core/broker/include/brokerPnpManagerInterface.h"

#include "xme/core/component.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"

#include "xme/hal/include/linkedList.h"


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN
extern xme_core_transferDataCallback_t
getTransferCallback
(
    uint8_t callbackNumber
);
XME_EXTERN_C_END

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

class BrokerIntegrationTest: public ::testing::Test
{
protected:
    BrokerIntegrationTest()
    : functDesc (NULL)
    , dataPacketDesc (NULL)
    {
    }

    virtual ~BrokerIntegrationTest()
    {
    }

    virtual void SetUp()
    {
        xme_core_broker_init(NULL);

        componentId = (xme_core_component_t) 10u;
        functionId1 = (xme_core_component_functionId_t) 21u;
        functionId2 = (xme_core_component_functionId_t) 22u;
        functionVariantId1 = (xme_core_component_functionVariantId_t) 31u;
        functionVariantId2 = (xme_core_component_functionVariantId_t) 32u;

        outDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 41u;
        outDataPacketId2 = (xme_core_dataManager_dataPacketId_t) 42u;
        outDataPacketId3 = (xme_core_dataManager_dataPacketId_t) 43u;
        outDataPacketId4 = (xme_core_dataManager_dataPacketId_t) 44u;
        outDataPacketId5 = (xme_core_dataManager_dataPacketId_t) 45u;

        inDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 51u;
        inDataPacketId2 = (xme_core_dataManager_dataPacketId_t) 52u;
        inDataPacketId3 = (xme_core_dataManager_dataPacketId_t) 53u;
        inDataPacketId4 = (xme_core_dataManager_dataPacketId_t) 54u;
        inDataPacketId5 = (xme_core_dataManager_dataPacketId_t) 55u;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId2, componentId, functionId1, functionVariantId1, true));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId2, functionVariantId1));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId3, componentId, functionId2, functionVariantId1, true));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId4, componentId, functionId2, functionVariantId1, false));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId3, componentId, functionId2, functionVariantId2, true));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId5, componentId, functionId2, functionVariantId2, true));

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(outDataPacketId1, inDataPacketId2));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(outDataPacketId1, inDataPacketId3));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(outDataPacketId2, inDataPacketId5));
    }

    virtual void TearDown()
    {
        xme_core_broker_fini();
    }

    xme_core_component_t componentId;
    xme_core_component_functionId_t functionId1;
    xme_core_component_functionId_t functionId2;
    xme_core_component_functionVariantId_t functionVariantId1;
    xme_core_component_functionVariantId_t functionVariantId2;

    xme_core_dataManager_dataPacketId_t outDataPacketId1;
    xme_core_dataManager_dataPacketId_t outDataPacketId2;
    xme_core_dataManager_dataPacketId_t outDataPacketId3;
    xme_core_dataManager_dataPacketId_t outDataPacketId4;
    xme_core_dataManager_dataPacketId_t outDataPacketId5;

    xme_core_dataManager_dataPacketId_t inDataPacketId1;
    xme_core_dataManager_dataPacketId_t inDataPacketId2;
    xme_core_dataManager_dataPacketId_t inDataPacketId3;
    xme_core_dataManager_dataPacketId_t inDataPacketId4;
    xme_core_dataManager_dataPacketId_t inDataPacketId5;

    xme_hal_singlyLinkedList_t(10) functionList;
    xme_hal_linkedList_descriptor_t* functDesc;

    xme_hal_singlyLinkedList_t(10) dataPacketList;
    xme_hal_linkedList_descriptor_t* dataPacketDesc;
};


/*****************************************************************************/
/***   DataHandler-related functions                                       ***/
/*****************************************************************************/
TEST_F(BrokerIntegrationTest, CallDataHandlerFunctionsWithInvalidDataPacketId)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_completeWriteOperation(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(BrokerIntegrationTest, CallDataHandlerFunctionsWithUnregisteredDataPacketId)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_completeWriteOperation(outDataPacketId5));
}

TEST_F(BrokerIntegrationTest, CallDataHandlerFunctionsWithSourceDataPacketId)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(outDataPacketId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(outDataPacketId2));
}

/*****************************************************************************/
/***   Function availability-related functions                             ***/
/*****************************************************************************/
TEST_F(BrokerIntegrationTest, CallExecutionManagerFunctionsWithSourceDataPacketId)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(outDataPacketId1));
    // dataAvailability(outDP1)
    // \_ transferData(outDP1, inDP2)
    //    \_ dataAvailability(inDP2, 1)
    // \_ transferData(outDP1, inDP3)
    //    \_ dataAvailability(inDP3, 1)
    // completeReadOperation(outDP1)
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(outDataPacketId2));
    // notifyOutputDataPacketAvailability(outDP2)
    // \_ transferData(outDP2, inDP5)
    //    \_ dataAvailability(inDP5, 1)
    // completeReadOperation(outDP2)
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
}

TEST_F(BrokerIntegrationTest, CallDataHandlerFunctionsWithUnregisteredCallbackFunction)
{
    // when not initialized, default datahandler's datatransfer
    xme_core_dataHandler_init();
    xme_core_broker_setTransferCallback(NULL);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(outDataPacketId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(outDataPacketId2));
}

TEST_F(BrokerIntegrationTest, CallDataHandlerFunctionsWithRegisteredCallbackFunction)
{
    xme_core_dataHandler_init();
    xme_core_broker_setTransferCallback(getTransferCallback(1U));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(outDataPacketId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(outDataPacketId2));
}

TEST_F(BrokerIntegrationTest, CallDataHandlerFunctionsWithRegisteredCallbackFunctionReturningOutOfResourcesError)
{
    xme_core_dataHandler_init();
    xme_core_broker_setTransferCallback(getTransferCallback(2U));
    ASSERT_EQ(XME_STATUS_OUT_OF_RESOURCES, xme_core_dataHandler_completeWriteOperation(outDataPacketId1));
}

TEST_F(BrokerIntegrationTest, CallDataHandlerFunctionsWithRegisteredCallbackFunctionReturningInvalidConfError)
{
    xme_core_dataHandler_init();
    xme_core_broker_setTransferCallback(getTransferCallback(3U));
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_dataHandler_completeWriteOperation(outDataPacketId1));
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
