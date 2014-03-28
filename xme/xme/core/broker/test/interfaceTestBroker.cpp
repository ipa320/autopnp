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
 * $Id: interfaceTestBroker.cpp 7813 2014-03-13 13:50:33Z gulati $
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

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/core/broker/include/broker.h"
#include "xme/core/broker/include/brokerDataManagerInterface.h"
#include "xme/core/broker/include/brokerPnpManagerInterface.h"

#include "xme/hal/include/linkedList.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

class BrokerInterfaceTest: public ::testing::Test
{
protected:
    BrokerInterfaceTest()
    : functDesc (NULL)
    , dataPacketDesc (NULL)
    {
    }

    virtual ~BrokerInterfaceTest()
    {
    }

    virtual void SetUp()
    {
        xme_core_broker_init(NULL);

        componentId = (xme_core_component_t) 10u;
        functionId1 = (xme_core_component_functionId_t) 21u;
        functionId2 = (xme_core_component_functionId_t) 22u;
        functionId3 = (xme_core_component_functionId_t) 23u;

        functionVariantId1 = (xme_core_component_functionVariantId_t) 31u;
        functionVariantId2 = (xme_core_component_functionVariantId_t) 32u;
        functionVariantId3 = (xme_core_component_functionVariantId_t) 33u;
        functionVariantId4 = (xme_core_component_functionVariantId_t) 34u;

        inDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 41u;
        inDataPacketId2 = (xme_core_dataManager_dataPacketId_t) 42u;
        inDataPacketId3 = (xme_core_dataManager_dataPacketId_t) 43u;
        inDataPacketId4 = (xme_core_dataManager_dataPacketId_t) 44u;
        inDataPacketId5 = (xme_core_dataManager_dataPacketId_t) 45u;

        XME_HAL_SINGLYLINKEDLIST_INIT(functionList);
        functDesc = (xme_hal_linkedList_descriptor_t*) &functionList;

        XME_HAL_SINGLYLINKEDLIST_INIT(dataPacketList);
        dataPacketDesc = (xme_hal_linkedList_descriptor_t*) &dataPacketList;
    }

    virtual void TearDown()
    {
        xme_core_broker_fini();
    }

    xme_core_component_t componentId;
    xme_core_component_functionId_t functionId1;
    xme_core_component_functionId_t functionId2;
    xme_core_component_functionId_t functionId3;

    xme_core_component_functionVariantId_t functionVariantId1;
    xme_core_component_functionVariantId_t functionVariantId2;
    xme_core_component_functionVariantId_t functionVariantId3;
    xme_core_component_functionVariantId_t functionVariantId4;

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

xme_status_t
xme_test_transferData
(
    xme_core_dataManager_dataPacketId_t src,
    xme_core_dataManager_dataPacketId_t dst
)
{
    //doSomething();
    XME_UNUSED_PARAMETER(src);
    XME_UNUSED_PARAMETER(dst);
    return XME_STATUS_SUCCESS;
}



/*****************************************************************************/
/***   brokerPnPManagerInterface                                           ***/
/*****************************************************************************/

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterFunction) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId2, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterFunctionWithDoubleRegistration) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterFunctionWithRACETests) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId2, functionVariantId1));
        //register functions
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_registerFunction((xme_core_component_t) 46346, (xme_core_component_functionId_t)0, NULL));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction((xme_core_component_t) 345345, (xme_core_component_functionId_t)546546, NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_registerFunction((xme_core_component_t) 0, (xme_core_component_functionId_t)564, NULL));
    //register subscriber-ports
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_addDataPacketToFunction((xme_core_dataManager_dataPacketId_t) 1, (xme_core_component_t) 0,  (xme_core_component_functionId_t)4363, NULL, true));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_addDataPacketToFunction((xme_core_dataManager_dataPacketId_t) 2, (xme_core_component_t) 0, (xme_core_component_functionId_t)4363, NULL, true));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_addDataPacketToFunction((xme_core_dataManager_dataPacketId_t) 3,  (xme_core_component_t) 46346, (xme_core_component_functionId_t)0, NULL, true));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_addDataPacketToFunction((xme_core_dataManager_dataPacketId_t) 4,  (xme_core_component_t) 0, (xme_core_component_functionId_t)4363, NULL, true));
    //set broker table transfers
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry((xme_core_dataManager_dataPacketId_t) 1, (xme_core_dataManager_dataPacketId_t) 1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry((xme_core_dataManager_dataPacketId_t) 1, (xme_core_dataManager_dataPacketId_t) 2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry((xme_core_dataManager_dataPacketId_t) 1, (xme_core_dataManager_dataPacketId_t) 3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry((xme_core_dataManager_dataPacketId_t) 1, (xme_core_dataManager_dataPacketId_t) 4));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterFunctionWithInvalidComponentID) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_registerFunction(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_registerFunction(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId1, functionVariantId2));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_registerFunction(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId2, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterFunctionWithInvalidFunctionId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_registerFunction(componentId, XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterFunctionWithMaximumComponentID) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId1, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId2, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterFunctionWithMaximumFunctionId) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, XME_CORE_COMPONENT_MAX_FUNCTION_CONTEXT, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterMandatoryParameter) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterWithParameterWithDifferentOptionsForAddDataPacketToFunction) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId2, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId2, functionVariantId1, true));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId2, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_broker_registerFunction(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId2, functionVariantId1)); // registered in the previous add datapacket to function

    ASSERT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_broker_addDataPacketToFunction(inDataPacketId1, XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId2, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId1, functionVariantId1, true));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterOptionalParameter) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, false));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRegisterParameterTwoTimes) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzAddDataPacketFunctionWithInvalidDataPacketId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_addDataPacketToFunction(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, componentId, functionId1, functionVariantId1, true));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionRegistered) {
    ASSERT_FALSE(xme_core_broker_isFunctionRegistered(componentId, functionId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_TRUE(xme_core_broker_isFunctionRegistered(componentId, functionId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionVariantRegistered) {
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_TRUE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsDataPacketRegistered) {
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId2, componentId, functionId1, functionVariantId1, false));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId2));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveDataPacketFromFunction) {
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true)); // mandatory
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId2, componentId, functionId1, functionVariantId1, false)); // optional
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketFromFunction(inDataPacketId1, componentId, functionId1, functionVariantId1)); // mandatory
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketFromFunction(inDataPacketId2, componentId, functionId1, functionVariantId1)); // optional
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId2));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveDataPacketFromFunctionWithInvalidDataPacketId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_removeDataPacketFromFunction(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveDataPacketFromFunctionWithInvalidComponentId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_removeDataPacketFromFunction(inDataPacketId1, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveDataPacketFromFunctionWithInvalidFunctionId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_removeDataPacketFromFunction(inDataPacketId1, componentId, XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveDataPacketFromFunctionWithNoFunctionsRegistered) {
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeDataPacketFromFunction(inDataPacketId1, componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveDataPacketFromFunctionWithFunctionRegisteredButNotAssociatedDataPackets) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeDataPacketFromFunction(inDataPacketId1, componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeDataPacketFromFunction(inDataPacketId1, componentId, functionId2, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveDataPacketFromFunctionWithFunctionRegistered) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true)); // mandatory
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeDataPacketFromFunction(inDataPacketId1, componentId, functionId2, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketFromFunction(inDataPacketId1, componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveDataPacketWithInvalidDataPacketId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_removeDataPacket(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveDataPacketWithNonRegisteredDataPacketId) {
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeDataPacket(inDataPacketId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionVariantWithInvalidComponentId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_removeFunctionVariant(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId1, NULL));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionVariantWithInvalidFunctionId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_removeFunctionVariant(componentId, XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, NULL));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionVariantWithNonRegisteredComponentId) {
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunctionVariant(componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionVariantWithRegisteredFunction) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeFunctionVariant(componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionVariantWithOtherRegisteredFunction) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunctionVariant(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunctionVariant(componentId, functionId2, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunctionVariant(componentId, functionId2, functionVariantId2));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionVariantWithOtherRegisteredFunctionAndAssociatedParameters) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunctionVariant(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunctionVariant(componentId, functionId2, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunctionVariant(componentId, functionId2, functionVariantId2));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionWithInvalidComponentId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_removeFunction(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionWithInvalidFunctionId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_removeFunction(componentId, XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionWithNonRegisteredFunction) {
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunction(componentId, functionId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionWithRegisteredFunction) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeFunction(componentId, functionId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionWithOtherRegisteredFunction) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunction(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunction(componentId, functionId2));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzRemoveFunctionWithRegisteredFunctionAndAssociatedParameters) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunction(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeFunction(componentId, functionId2));
}


TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionRegisteredWithInvalidComponentId) {
    ASSERT_FALSE(xme_core_broker_isFunctionRegistered(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionRegisteredWithInvalidFunctionId) {
    ASSERT_FALSE(xme_core_broker_isFunctionRegistered(componentId, XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionRegisteredWithNonRegisteredFunction) {
    ASSERT_FALSE(xme_core_broker_isFunctionRegistered(componentId, functionId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionRegisteredWithRegisteredFunction) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_TRUE(xme_core_broker_isFunctionRegistered(componentId, functionId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionVariantRegisteredWithInvalidComponentId) {
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionVariantRegisteredWithInvalidFunctionId) {
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(componentId, XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionVariantRegisteredWithNonRegisteredFunction) {
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsFunctionVariantRegisteredWithRegisteredFunction) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_TRUE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsDataPacketRegisteredWithInvalidDataPacketId) {
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsDataPacketRegisteredWithMaximumDataPacketId) {
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(XME_CORE_DATAMANAGER_DATAPACKETID_MAX));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsDataPacketRegisteredWithNonRegisteredDataPacket) {
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzIsDataPacketRegisteredWithRegisteredDataPacket) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzGetFunctionDataPacketsWithInvalidComponentId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_getFunctionDataPackets(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId1, functionVariantId1, NULL));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzGetFunctionDataPacketsWithInvalidFunctionId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_getFunctionDataPackets(componentId, XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, functionVariantId1, NULL));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzGetFunctionDataPacketsWithNullValuedDestination) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_getFunctionDataPackets(componentId, functionId1, functionVariantId1, NULL));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzGetFunctionDataPacketsWithOnlyFunctionRegistered) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_getFunctionDataPackets(componentId, functionId1, functionVariantId1, functDesc));
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzGetFunctionDataPacketsWithFunctionAndDataPacketRegistered) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getFunctionDataPackets(componentId, functionId1, functionVariantId1, functDesc));
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzGetDataPacketFunctionsWithInvalidDataPacketId) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_getDataPacketFunctions(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzGetDataPacketFunctionsWithNullValuedReturnVariable) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_getDataPacketFunctions(inDataPacketId1, NULL));
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzGetDataPacketFunctionsWithOnlyFunctionRegistered) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_getDataPacketFunctions(inDataPacketId1, functDesc));
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
}

TEST_F(BrokerInterfaceTest, PnPManagerIfzGetDataPacketFunctionsWithFunctionAndDataPacketRegistered) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getDataPacketFunctions(inDataPacketId1, functDesc));
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
}

// new broker features
TEST_F(BrokerInterfaceTest, BrokerPnPManagerInterfaceComplete)
{
    ASSERT_FALSE(xme_core_broker_isFunctionRegistered(componentId, functionId1));
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId1, functionVariantId1));
    ASSERT_FALSE(xme_core_broker_isFunctionRegistered(componentId, functionId2));
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId2, functionVariantId2));
    ASSERT_FALSE(xme_core_broker_isFunctionRegistered(componentId, functionId3));
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId3, functionVariantId3));
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId3, functionVariantId4));
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId2));
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId3));
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId4));
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId5));
    
    // 1. register just one function
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_TRUE(xme_core_broker_isFunctionRegistered(componentId, functionId1));
    ASSERT_TRUE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_getFunctionDataPackets(componentId, functionId1, functionVariantId1, functDesc));

    // 2. add a datapacket to the function
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getDataPacketFunctions(inDataPacketId1, dataPacketDesc));
    ASSERT_EQ(1, xme_hal_singlyLinkedList_getItemCount(dataPacketDesc));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getFunctionDataPackets(componentId, functionId1, functionVariantId1, functDesc));
    ASSERT_EQ(1, xme_hal_singlyLinkedList_getItemCount(functDesc));
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);

    // 3. add a second datapacket to the function
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId2, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getDataPacketFunctions(inDataPacketId2, dataPacketDesc));
    ASSERT_EQ(1, xme_hal_singlyLinkedList_getItemCount(dataPacketDesc));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getFunctionDataPackets(componentId, functionId1, functionVariantId1, functDesc));
    ASSERT_EQ(2, xme_hal_singlyLinkedList_getItemCount(functDesc));
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);

    // 4. register two more functions, and associate different dataPackets
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId2, functionVariantId2, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId3, componentId, functionId2, functionVariantId2, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId4, componentId, functionId2, functionVariantId2, false));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId3, functionVariantId3, false));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId2, componentId, functionId3, functionVariantId3, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId4, componentId, functionId3, functionVariantId3, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId5, componentId, functionId3, functionVariantId3, false));

    ASSERT_TRUE(xme_core_broker_isFunctionRegistered(componentId, functionId3));
    ASSERT_TRUE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId3, functionVariantId3));
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId3, functionVariantId4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId3, functionVariantId4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId4, componentId, functionId3, functionVariantId4, false));

    // 5. testing of isFunctionRegistered and isDataPacketRegistered
    ASSERT_TRUE(xme_core_broker_isFunctionRegistered(componentId, functionId1));
    ASSERT_TRUE(xme_core_broker_isFunctionRegistered(componentId, functionId2));
    ASSERT_TRUE(xme_core_broker_isFunctionRegistered(componentId, functionId3));
    ASSERT_TRUE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId1, functionVariantId1));
    ASSERT_TRUE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId2, functionVariantId2));
    ASSERT_TRUE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId3, functionVariantId3));
    ASSERT_TRUE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId3, functionVariantId4));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId1));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId2));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId3));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId4));
    ASSERT_TRUE(xme_core_broker_isDataPacketRegistered(inDataPacketId5));

    // 6. testing of getFunctionDataPackets
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getFunctionDataPackets(componentId, functionId1, functionVariantId1, functDesc));
    ASSERT_EQ(2, xme_hal_singlyLinkedList_getItemCount(functDesc)); // two associated datapackets
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getFunctionDataPackets(componentId, functionId2, functionVariantId2, functDesc));
    ASSERT_EQ(3, xme_hal_singlyLinkedList_getItemCount(functDesc)); // three associated datapackets
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getFunctionDataPackets(componentId, functionId3, functionVariantId3, functDesc));
    ASSERT_EQ(4, xme_hal_singlyLinkedList_getItemCount(functDesc)); // four associated datapackets
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getFunctionDataPackets(componentId, functionId3, functionVariantId4, functDesc));
    ASSERT_EQ(1, xme_hal_singlyLinkedList_getItemCount(functDesc)); // four associated datapackets
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);

    // 7. testing of getDataPacketFunctions
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getDataPacketFunctions(inDataPacketId1, dataPacketDesc));
    ASSERT_EQ(3, xme_hal_singlyLinkedList_getItemCount(dataPacketDesc)); // three associated functions
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getDataPacketFunctions(inDataPacketId2, dataPacketDesc));
    ASSERT_EQ(2, xme_hal_singlyLinkedList_getItemCount(dataPacketDesc)); // two associated functions
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getDataPacketFunctions(inDataPacketId3, dataPacketDesc));
    ASSERT_EQ(1, xme_hal_singlyLinkedList_getItemCount(dataPacketDesc)); // one associated function
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getDataPacketFunctions(inDataPacketId4, dataPacketDesc));
    ASSERT_EQ(3, xme_hal_singlyLinkedList_getItemCount(dataPacketDesc)); // two associated functions
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getDataPacketFunctions(inDataPacketId5, dataPacketDesc));
    ASSERT_EQ(1, xme_hal_singlyLinkedList_getItemCount(dataPacketDesc)); // one associated function
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);

    // 8. testing of remove data packet from function
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketFromFunction(inDataPacketId1, componentId, functionId3, functionVariantId3)); // f1 -> -dp1
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getFunctionDataPackets(componentId, functionId3, functionVariantId3, functDesc));
    ASSERT_EQ(3, xme_hal_singlyLinkedList_getItemCount(functDesc)); // only three associated datapackets left
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getDataPacketFunctions(inDataPacketId1, dataPacketDesc));
    ASSERT_EQ(2, xme_hal_singlyLinkedList_getItemCount(dataPacketDesc)); // only two associated functions left
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketFromFunction(inDataPacketId2, componentId, functionId3, functionVariantId3)); // f1 -> -dp2
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketFromFunction(inDataPacketId4, componentId, functionId3, functionVariantId3)); // f1 -> -dp4
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketFromFunction(inDataPacketId5, componentId, functionId3, functionVariantId3)); // f1 -> -dp5
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_getFunctionDataPackets(componentId, functionId3, functionVariantId3, functDesc));
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_getFunctionDataPackets(componentId, functionId3, functionVariantId4, functDesc));
    ASSERT_EQ(1, xme_hal_singlyLinkedList_getItemCount(functDesc)); // only three associated datapackets left
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);

    // 9. testing of removeFunction (for all data packages)
    ASSERT_TRUE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeFunctionVariant(componentId, functionId2, functionVariantId2));
    ASSERT_FALSE(xme_core_broker_isFunctionVariantRegistered(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_getFunctionDataPackets(componentId, functionId2, functionVariantId2, functDesc));
    XME_HAL_SINGLYLINKEDLIST_INIT(*functDesc);

    // 10. testing of removeDataPacket (for all functions)
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacket(inDataPacketId2));
    ASSERT_FALSE(xme_core_broker_isDataPacketRegistered(inDataPacketId2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_getDataPacketFunctions(inDataPacketId2, dataPacketDesc));
    XME_HAL_SINGLYLINKEDLIST_INIT(*dataPacketDesc);
}



/*****************************************************************************/
/***   brokerDataManagerInterface (I): transfer                            ***/
/*****************************************************************************/

TEST_F(BrokerInterfaceTest, AddDataPacketTransferEntryWithTwoInvalidHandles)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_broker_addDataPacketTransferEntry(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                   XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(BrokerInterfaceTest,  AddDataPacketTransferEntryWithFirstInvalidAndSecondMaxiumHandle)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_broker_addDataPacketTransferEntry(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                XME_CORE_DATAMANAGER_DATAPACKETID_MAX));
}

TEST_F(BrokerInterfaceTest, AddDataPacketTransferEntryWithFirstMaximumAndSecondInvalidHandle)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_broker_addDataPacketTransferEntry(XME_CORE_DATAMANAGER_DATAPACKETID_MAX,
                                                XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(BrokerInterfaceTest, AddDataPacketTransferEntryWithTwoMaximumHandle)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_broker_addDataPacketTransferEntry(XME_CORE_DATAMANAGER_DATAPACKETID_MAX,
                                                        XME_CORE_DATAMANAGER_DATAPACKETID_MAX));
}

TEST_F(BrokerInterfaceTest, AddDataPacketTransferEntryWithSourceAndSinkPortAreTheSame)
{
    xme_core_dataManager_dataPacketId_t src = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t dst = (xme_core_dataManager_dataPacketId_t) 1u;
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(src, dst));
}

TEST_F(BrokerInterfaceTest, AddDataPacketTransferEntryNormalTest)
{
    xme_core_dataManager_dataPacketId_t src = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t dst = (xme_core_dataManager_dataPacketId_t) 2u;
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(src, dst));
}

TEST_F(BrokerInterfaceTest, AddDataPacketTransferEntryTwoTimes)
{
    xme_core_dataManager_dataPacketId_t src = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t dst = (xme_core_dataManager_dataPacketId_t) 2u;
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(src, dst));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(src, dst));
}

TEST_F(BrokerInterfaceTest, RemoveDataPacketTransferEntryWithTwoInvalidHandles)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_broker_removeDataPacketTransferEntry(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                   XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(BrokerInterfaceTest,  RemoveDataPacketTransferEntryWithFirstInvalidAndSecondMaxiumHandle)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_broker_removeDataPacketTransferEntry(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                XME_CORE_DATAMANAGER_DATAPACKETID_MAX));
}

TEST_F(BrokerInterfaceTest, RemoveDataPacketTransferEntryWithFirstMaximumAndSecondInvalidHandle)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_broker_removeDataPacketTransferEntry(XME_CORE_DATAMANAGER_DATAPACKETID_MAX,
                                                XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(BrokerInterfaceTest, RemoveDataPacketTransferEntryWithTwoMaximumHandle)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
              xme_core_broker_removeDataPacketTransferEntry(XME_CORE_DATAMANAGER_DATAPACKETID_MAX,
                                                XME_CORE_DATAMANAGER_DATAPACKETID_MAX));
}

TEST_F(BrokerInterfaceTest, RemoveDataPacketTransferEntryWithSourceAndSinkPortAreTheSame)
{
    xme_core_dataManager_dataPacketId_t src = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t dst = (xme_core_dataManager_dataPacketId_t) 1u;
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
              xme_core_broker_removeDataPacketTransferEntry(src, dst));
}

TEST_F(BrokerInterfaceTest, RemoveDataPacketTransferEntryUsingAddFunction)
{
    xme_core_dataManager_dataPacketId_t src = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t dst = (xme_core_dataManager_dataPacketId_t) 2u;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(src, dst));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketTransferEntry(src, dst));
}

TEST_F(BrokerInterfaceTest, RemoveDataPacketTransferEntryWithOtherDataPackets)
{
    xme_core_dataManager_dataPacketId_t src = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t src2 = (xme_core_dataManager_dataPacketId_t) 2u;
    xme_core_dataManager_dataPacketId_t dst = (xme_core_dataManager_dataPacketId_t) 3u;
    xme_core_dataManager_dataPacketId_t dst2 = (xme_core_dataManager_dataPacketId_t) 4u;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(src, dst));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeDataPacketTransferEntry(src, dst2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeDataPacketTransferEntry(src2, dst));
}

TEST_F(BrokerInterfaceTest, RemoveDataPacketTransferEntryUsingAddFunctionAndDoubleRemoval)
{
    xme_core_dataManager_dataPacketId_t src = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t dst = (xme_core_dataManager_dataPacketId_t) 2u;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(src, dst));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketTransferEntry(src, dst));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_removeDataPacketTransferEntry(src, dst));
}

TEST_F(BrokerInterfaceTest, DataAvailabilityChangeWithInvalidDataPacket)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_dataAvailabilityChange(
        XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 0));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_dataAvailabilityChange(
        XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1));
}

TEST_F(BrokerInterfaceTest, DataAvailabilityChangeWithMaximumDataPacket)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_dataAvailabilityChange(
        XME_CORE_DATAMANAGER_DATAPACKETID_MAX, 0));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_dataAvailabilityChange(
        XME_CORE_DATAMANAGER_DATAPACKETID_MAX, 1));
}

TEST_F(BrokerInterfaceTest, DataAvailabilityChangeWitNoRegisteredDataPacket)
{
    xme_core_dataManager_dataPacketId_t dataPacketId = (xme_core_dataManager_dataPacketId_t) 1u;
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_dataAvailabilityChange(
        dataPacketId, 1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_dataAvailabilityChange(
        dataPacketId, 0));
}

TEST_F(BrokerInterfaceTest, DataAvailabilityChangeWithRegisteredDataPacketCheckSrc)
{
    xme_core_dataManager_dataPacketId_t srcDataPacketId = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId = (xme_core_dataManager_dataPacketId_t) 2u;
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(srcDataPacketId, dstDataPacketId));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(
        srcDataPacketId, 1)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(
        srcDataPacketId, 0)); 
}

TEST_F(BrokerInterfaceTest, DataAvailabilityChangeWithRegisteredDataPacketCheckDst)
{
    xme_core_dataManager_dataPacketId_t srcDataPacketId = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId = (xme_core_dataManager_dataPacketId_t) 2u;
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(srcDataPacketId, dstDataPacketId));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(
        dstDataPacketId, 1)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(
        dstDataPacketId, 0)); 
}

TEST_F(BrokerInterfaceTest, DataAvailabilityChangeToZeroWithTransferRegisteredDataPacket)
{
    xme_core_dataManager_dataPacketId_t srcDataPacketId = (xme_core_dataManager_dataPacketId_t) 1u;
    xme_core_dataManager_dataPacketId_t srcDataPacketId2 = (xme_core_dataManager_dataPacketId_t) 2u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId = (xme_core_dataManager_dataPacketId_t) 3u;
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(srcDataPacketId, dstDataPacketId));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(dstDataPacketId, 0)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(srcDataPacketId, 0)); 
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_dataAvailabilityChange(srcDataPacketId2, 0)); 
}

/*****************************************************************************/
/***   brokerDataManagerInterface (II): function availability              ***/
/*****************************************************************************/
TEST_F(BrokerInterfaceTest, BrokerDataManagerDataAvailabilityChange) {
    // setup
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId2, componentId, functionId1, functionVariantId1, false));

    // test
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId1, 1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId1, 0));
}

TEST_F(BrokerInterfaceTest, BrokerDataManagerDataAvailabilityChangeWithoutSetup) {
    // test
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_dataAvailabilityChange(inDataPacketId1, 1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_dataAvailabilityChange(inDataPacketId1, 0));
}

TEST_F(BrokerInterfaceTest, BrokerDataManagerDataAvailabilityChangeWithInvalidDataPacketID) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_dataAvailabilityChange(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1));
}

TEST_F(BrokerInterfaceTest, BrokerDataManagerIsFunctionReady) {
    // setup
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId2, componentId, functionId1, functionVariantId1, false));

    // test
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId1, 1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId1, 0));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId2, 1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId2, 0));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, BrokerDataManagerIsFunctionReadyWithInvalidComponentID) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_isFunctionReady(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, BrokerDataManagerIsFunctionReadyWithInvalidFunctionID) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_isFunctionReady(componentId, XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, BrokerDataManagerIsFunctionReadyWithMaximumComponentID) {
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId1, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, BrokerDataManagerIsFunctionReadyWithMaximumFunctionID) {
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, XME_CORE_COMPONENT_MAX_FUNCTION_CONTEXT, functionVariantId1));
}

TEST_F(BrokerInterfaceTest, BrokerDataManagerIsFunctionReadyWithOtherFunctionReady) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId1, 1));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId2));
}

TEST_F(BrokerInterfaceTest, BrokerDataManagerInterfaceComplete)
{
    // 1. register functions and data packets
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId1, functionVariantId1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId2, componentId, functionId1, functionVariantId1, true));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId2, functionVariantId2, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId3, componentId, functionId2, functionVariantId2, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId4, componentId, functionId2, functionVariantId2, false));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId1, componentId, functionId3, functionVariantId3, false));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId2, componentId, functionId3, functionVariantId3, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId4, componentId, functionId3, functionVariantId3, true));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId5, componentId, functionId3, functionVariantId3, false));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_registerFunction(componentId, functionId3, functionVariantId4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketToFunction(inDataPacketId4, componentId, functionId3, functionVariantId4, true));

    // 2. send availability signals
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId1, 1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId5, 1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId4, 1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId2, 1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId3, 1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    // 3. send consumption signals. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId5, 0)); // optional dataPacket for f3
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId4, 0)); // optional dataPacket for f2 and mandatory for f3
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId1, 0)); // optional dataPacket for f3 and mandatory for f1
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId4, 1)); // optional dataPacket for f2 and mandatory for f3
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId3, 0)); // mandatory for f2
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId2, 0)); // mandatory for f1 and f3
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_dataAvailabilityChange(inDataPacketId4, 0)); // mandatory for f1 and f3
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId1, functionVariantId1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId2, functionVariantId2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId3));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_broker_isFunctionReady(componentId, functionId3, functionVariantId4));
}

TEST_F(BrokerInterfaceTest, BrokerCallbackRegisterWithoutInitialization) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_broker_setTransferCallback(NULL));
}

TEST_F(BrokerInterfaceTest, BrokerCallbackRegisterNormal) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_setTransferCallback(&xme_test_transferData));
}

TEST_F(BrokerInterfaceTest, TestNonRegisteredTransferEntryCount_Issue3797)
{
    uint16_t returnValue;

    xme_core_dataManager_dataPacketId_t srcDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 11u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 21u;

    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(0U, returnValue);
}

TEST_F(BrokerInterfaceTest, TestSingleTransferEntryCount_Issue3797)
{
    uint16_t returnValue;

    xme_core_dataManager_dataPacketId_t srcDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 11u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 21u;

    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
}

TEST_F(BrokerInterfaceTest, TestTwoTargetTransferEntryCount_Issue3797)
{
    uint16_t returnValue;

    xme_core_dataManager_dataPacketId_t srcDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 11u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 21u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId2 = (xme_core_dataManager_dataPacketId_t) 22u;

    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId2));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId2);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(1U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId2);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(1U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId2));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId2);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(0U, returnValue);
}

TEST_F(BrokerInterfaceTest, TestTwoSourceTransferEntryCount_Issue3797)
{
    uint16_t returnValue;

    xme_core_dataManager_dataPacketId_t srcDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 11u;
    xme_core_dataManager_dataPacketId_t srcDataPacketId2 = (xme_core_dataManager_dataPacketId_t) 12u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 21u;

    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId2, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(2U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketTransferEntry(srcDataPacketId2, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
}

TEST_F(BrokerInterfaceTest, TestTwoSourceAndTwoDestinationTransferEntryCount_Issue3797)
{
    uint16_t returnValue;

    xme_core_dataManager_dataPacketId_t srcDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 11u;
    xme_core_dataManager_dataPacketId_t srcDataPacketId2 = (xme_core_dataManager_dataPacketId_t) 12u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 21u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId2 = (xme_core_dataManager_dataPacketId_t) 22u;

    // src1 -> dst1
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(0U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(0U, returnValue);

    // src2 -> dst2
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId2, dstDataPacketId2));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, dstDataPacketId2);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(1U, returnValue);

    // src1 -> dst2
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId2));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId2);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(2U, returnValue);

    // src2 -> dst1
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId2, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(2U, returnValue);

    // src1 -> dst1 Adding it second time
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(2U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(3U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(3U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(2U, returnValue);

    // src1 -> dst1 removing it
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(2U, returnValue);

    // src1 -> dst1 removing it once more
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_removeDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId2, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);
    EXPECT_EQ(2U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    returnValue = xme_core_broker_dataPacketTransferEntryCount(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, dstDataPacketId2);
    EXPECT_EQ(2U, returnValue);
}

TEST_F(BrokerInterfaceTest, TestTwiceSingleTransferEntryCount_Issue3797)
{
    uint16_t returnValue;

    xme_core_dataManager_dataPacketId_t srcDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 11u;
    xme_core_dataManager_dataPacketId_t dstDataPacketId1 = (xme_core_dataManager_dataPacketId_t) 21u;

    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(0U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(1U, returnValue);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(srcDataPacketId1, dstDataPacketId1));
    returnValue = xme_core_broker_dataPacketTransferEntryCount(srcDataPacketId1, dstDataPacketId1);
    EXPECT_EQ(2U, returnValue);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
