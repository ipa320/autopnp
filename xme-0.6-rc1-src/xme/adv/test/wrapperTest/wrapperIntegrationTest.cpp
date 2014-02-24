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
 * $Id: wrapperIntegrationTest.cpp 4948 2013-09-04 08:25:51Z ruiz $
 */

/**
 * \file
 *         Marshaler waypoint test.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include <cstring>
#include <vector>

#include "include/wrapperTestComponentWrapper.h"
#include "include/funcFunction.h"
#include "include/funcFunctionWrapper.h"

/******************************************************************************/
/***   Global Variables                                                     ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

extern bool xme_adv_wrapperTest_dispatcherMock_waitForStartRunOnce;

extern uint8_t xme_adv_wrapperTest_dataHandlerMock_writeDataCountArray[];

extern uint8_t xme_adv_wrapperTest_dataHandlerMock_readDataCountArray[];

extern uint8_t xme_adv_wrapperTest_dataHandlerMock_completeWriteOperationCountArray[];

extern uint8_t xme_adv_wrapperTest_dataHandlerMock_completeReadOperationCountArray[];

XME_EXTERN_C_END

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class WrapperTestIntegrationTest : public testing::Test
{
protected:
    /**
     * \brief  Function descriptor of test function.
     */
    xme_core_exec_functionDescriptor_t descriptor;

    /**
     * \brief  Data packet id used for required port portOut0.
     */
    static const xme_core_dataManager_dataPacketId_t portOut0 = (xme_core_dataManager_dataPacketId_t)1;

    /**
     * \brief  Data packet id used for optional port portOut1.
     */
    static const xme_core_dataManager_dataPacketId_t portOut1 = (xme_core_dataManager_dataPacketId_t)2;

    /**
     * \brief  Data packet id used for required port portIn0.
     */
    static const xme_core_dataManager_dataPacketId_t portIn0 = (xme_core_dataManager_dataPacketId_t)3;

    /**
     * \brief  Data packet id used for optional port portIn1.
     */
    static const xme_core_dataManager_dataPacketId_t portIn1 = (xme_core_dataManager_dataPacketId_t)4;

    WrapperTestIntegrationTest()
    {
        xme_adv_wrapperTest_dispatcherMock_waitForStartRunOnce = false;

        descriptor.componentId = (xme_core_component_t)1;
        descriptor.init = &xme_adv_wrapperTest_funcFunction_init;
        descriptor.fini = &xme_adv_wrapperTest_funcFunction_fini;
        descriptor.functionId = (xme_core_component_functionId_t)1;
        descriptor.initParam = NULL;
        descriptor.wcet_ns = 100;

        xme_adv_wrapperTest_wrapperTestComponentWrapper_receivePort
        (
            portOut0,
            XME_ADV_WRAPPERTEST_WRAPPERTESTCOMPONENTWRAPPER_PORT_OUT0
        );
        xme_adv_wrapperTest_wrapperTestComponentWrapper_receivePort
        (
            portOut1,
            XME_ADV_WRAPPERTEST_WRAPPERTESTCOMPONENTWRAPPER_PORT_OUT1
        );
        xme_adv_wrapperTest_wrapperTestComponentWrapper_receivePort
        (
            portIn0,
            XME_ADV_WRAPPERTEST_WRAPPERTESTCOMPONENTWRAPPER_PORT_IN0
        );
        xme_adv_wrapperTest_wrapperTestComponentWrapper_receivePort
        (
            portIn1,
            XME_ADV_WRAPPERTEST_WRAPPERTESTCOMPONENTWRAPPER_PORT_IN1
        );
    }

    ~WrapperTestIntegrationTest()
    {
        // Do nothing
    }
};

/**
 * \brief  Write NULL to optional output port should not lead to any write calls
 *         in the data handler.
 */
TEST_F(WrapperTestIntegrationTest, writeNullToOptionalOutputPort)
{
    xme_adv_wrapperTest_mode = 0;
    uint8_t oldWriteDataCountOut0 = 
        xme_adv_wrapperTest_dataHandlerMock_writeDataCountArray[portOut0];
    uint8_t oldCompleteWriteOperationCountOut0 = 
        xme_adv_wrapperTest_dataHandlerMock_completeWriteOperationCountArray[portOut0];
    uint8_t oldWriteDataCountOut1 = 
        xme_adv_wrapperTest_dataHandlerMock_writeDataCountArray[portOut1];
    uint8_t oldCompleteWriteOperationCountOut1 = 
        xme_adv_wrapperTest_dataHandlerMock_completeWriteOperationCountArray[portOut1];
    
    xme_adv_wrapperTest_funcFunctionWrapper_execute(&descriptor);

    // No write operations should occur on optional port
    ASSERT_EQ
    (
        oldWriteDataCountOut1,
        xme_adv_wrapperTest_dataHandlerMock_writeDataCountArray[portOut1]
    );
    ASSERT_EQ
    (
        oldCompleteWriteOperationCountOut1,
        xme_adv_wrapperTest_dataHandlerMock_completeWriteOperationCountArray[portOut1]
    );

    // Required port should be written to
    ASSERT_EQ
    (
        oldWriteDataCountOut0 + 1,
        xme_adv_wrapperTest_dataHandlerMock_writeDataCountArray[portOut0]
    );
    ASSERT_EQ
    (
        oldCompleteWriteOperationCountOut0 + 1,
        xme_adv_wrapperTest_dataHandlerMock_completeWriteOperationCountArray[portOut0]
    );
}

/**
 * \brief  TODO, see issue #3184
 */
TEST_F(WrapperTestIntegrationTest, writeNullToRequiredOutputPort)
{
    // Not writing to a required port should probably not call writeData
    // but call completeWriteOperation, so that the last value is used
    // Please verify this first before implementation
}

int main(int argc, char **argv)
{
    int retval;

    ::testing::InitGoogleTest(&argc, argv);
    retval = RUN_ALL_TESTS();
    
    return retval;
}
