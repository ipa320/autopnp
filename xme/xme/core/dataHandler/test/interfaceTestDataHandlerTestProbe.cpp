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
 * $Id: interfaceTestDataHandlerTestProbe.cpp 7844 2014-03-14 14:11:49Z ruiz $
 */

/**
 * \file
 *         Data Handler Smoke Unit Tests.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include <cstring>

#include "xme/core/component.h"
#include "xme/core/dataHandler/include/dataHandlerTestProbe.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"
#include "xme/core/broker/include/broker.h"
#include "xme/core/testUtils.h"
#include "xme/hal/include/mem.h"

#define DB_MASTER 1
#define DB_SHADOW 2

#define OFFSET_ZERO 0U
#define OFFSET_ONE 4U
#define OFFSET_TWO 8U
#define OFFSET_THREE 12U
#define OFFSET_FOUR 16U
#define OFFSET_FIVE 20U
#define OFFSET_SIX 24U

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

typedef struct
{
    uint32_t field0;
    uint32_t field1;
    uint32_t field2;
    uint32_t field3;
    uint32_t field4;
    uint32_t field5;
} xme_core_dataHandler_topic_test_t;


/******************************************************************************/
/***   Class Definition                                                     ***/
/******************************************************************************/

class DataHandlerTestProbeInterfaceTest : public ::testing::Test {

    protected:
        DataHandlerTestProbeInterfaceTest()
        {
            // Initialize the database.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            // Create data packet 1. 
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(xme_core_dataHandler_topic_test_t), &dataPacketID1));
            EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataPacketID1);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketPersistency(dataPacketID1, true));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), 1U, dataPacketID1));

            // This is for the creation of shadow database. 
            // Note: Each database will get a sequential number starting in 1.
            // For RACE purposes, 1 will be the master database, while 2 will be the shadow database. 
            // The new data handler will not contain the semantics for shadowing or not. This should be implemented
            // by the user. For that purpose, we do provide two different interfaces:
            // - dataHandler.h, for simple read/write operations, that will use always the active(default) database.
            // - dataHandlerAdvanced.h, for read/write operations containing more the following parameters: databaseIndex and offset. 
            // FIXME: Assertion inside the function. 
            xme_core_dataHandler_setNumberOfDatabases(2U);

            // Configuration of the database. 
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

            // Write the initial values. 
            testTopic.field0 = 10U;
            testTopic.field1 = 11U;
            testTopic.field2 = 12U;
            testTopic.field3 = 13U;
            testTopic.field4 = 14U;
            testTopic.field5 = 15U;

            EXPECT_EQ(XME_STATUS_SUCCESS, 
                xme_core_dataHandler_writeData(dataPacketID1, &testTopic, sizeof(xme_core_dataHandler_topic_test_t)));
            {
                uint32_t attributeValue = 10U;
                EXPECT_EQ(XME_STATUS_SUCCESS, 
                    xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &attributeValue, sizeof(attributeValue)));
            }
        }

        virtual
        ~DataHandlerTestProbeInterfaceTest() {
            // Finalize the database. 
            xme_core_dataHandler_fini();
        }

    xme_core_dataManager_dataPacketId_t dataPacketID1;
    xme_core_dataHandler_topic_test_t testTopic;
};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

TEST_F(DataHandlerTestProbeInterfaceTest, NormalDataManipulation)
{
    uint32_t manipulatedValue = 24U;
    uint32_t bytesRead;
    uint32_t bytesWritten;


    // Purspose of the test:
    // Activate a manipulation, manipulate from TstPrb, read the values, and deactivate the manipulation. 

    // Step 1: Activate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_startManipulation(dataPacketID1, OFFSET_FOUR));
    }

    // Step 2: Manipulate data.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &manipulatedValue, sizeof(uint32_t), &bytesWritten)); 
        EXPECT_EQ(sizeof(uint32_t), bytesWritten);
    }

    // Step 3: Read data (from TstSrv) (six read operations) 
    {
        uint32_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 10U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ONE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 11U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_TWO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 12U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_THREE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 13U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FIVE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 15U);
    }

    // Step 3: Read data (from Application) (one read operations) 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 10U);
        EXPECT_EQ(buffer.field1, 11U);
        EXPECT_EQ(buffer.field2, 12U);
        EXPECT_EQ(buffer.field3, 13U);
        EXPECT_EQ(buffer.field4, manipulatedValue);
        EXPECT_EQ(buffer.field5, 15U);
    }

    // Step 4: Deactivate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_endManipulation(dataPacketID1, OFFSET_FOUR));
    }

    // Step 5: Read data again (from Application). 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 10U);
        EXPECT_EQ(buffer.field1, 11U);
        EXPECT_EQ(buffer.field2, 12U);
        EXPECT_EQ(buffer.field3, 13U);
        EXPECT_EQ(buffer.field4, 14U);
        EXPECT_EQ(buffer.field5, 15U);
    }
}

TEST_F(DataHandlerTestProbeInterfaceTest, NormalAttributeManipulation)
{
    uint32_t manipulatedValue = 20U;
    uint32_t bytesRead;
    uint32_t bytesWritten;


    // Purspose of the test:
    // Activate a manipulation, manipulate from TstPrb, read the values, and deactivate the manipulation. 

    // Step 1: Activate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_startAttributeManipulation(dataPacketID1, OFFSET_ZERO, 1U));
    }

    // Step 2: Manipulate attribute.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(DB_SHADOW, dataPacketID1, 1U, OFFSET_ZERO, &manipulatedValue, sizeof(uint32_t), &bytesWritten)); 
        EXPECT_EQ(sizeof(uint32_t), bytesWritten);
    }

    // Step 3: Read attribute (from TstSrv)
    {
        uint32_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readAttributeInDatabase(DB_SHADOW, dataPacketID1, 1U, OFFSET_ZERO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue);
    }

    // Step 3: Read attribute (from Application) (one read operations) 
    {
        uint32_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &buffer, sizeof(buffer), &bytesRead)); 
        EXPECT_EQ(sizeof(buffer), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue);
    }

    // Step 4: Deactivate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_endAttributeManipulation(dataPacketID1, OFFSET_ZERO, 1U));
    }

    // Step 5: Read data again (from Application). 
    {
        uint32_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 10U);
    }
}


TEST_F(DataHandlerTestProbeInterfaceTest, ManipulationWithWrite)
{
    uint32_t manipulatedValue = 24U;
    uint32_t bytesRead;
    uint32_t bytesWritten;


    // Purspose of the test:
    // Activate a manipulation, manipulate from TstPrb, write from Application, read the values, and deactivate the manipulation. 

    // Step 1: Activate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_startManipulation(dataPacketID1, OFFSET_FOUR));
    }

    // Step 2: Manipulate data.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &manipulatedValue, sizeof(uint32_t), &bytesWritten)); 
        EXPECT_EQ(sizeof(uint32_t), bytesWritten);
    }

    // Step 3: Read data (from TstSrv) (six read operations) 
    {
        uint32_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 10U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ONE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 11U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_TWO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 12U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_THREE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 13U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FIVE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 15U);
    }

    // Step 3: Read data (from Application) (one read operations) 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 10U);
        EXPECT_EQ(buffer.field1, 11U);
        EXPECT_EQ(buffer.field2, 12U);
        EXPECT_EQ(buffer.field3, 13U);
        EXPECT_EQ(buffer.field4, manipulatedValue);
        EXPECT_EQ(buffer.field5, 15U);
    }

    // Step 4: Write new values. 
    {
        xme_core_dataHandler_topic_test_t newTestTopic;

        newTestTopic.field0 = 30U;
        newTestTopic.field1 = 31U;
        newTestTopic.field2 = 32U;
        newTestTopic.field3 = 33U;
        newTestTopic.field4 = 34U;
        newTestTopic.field5 = 35U;

        EXPECT_EQ(XME_STATUS_SUCCESS, 
            xme_core_dataHandler_writeData(dataPacketID1, &newTestTopic, sizeof(xme_core_dataHandler_topic_test_t)));
    }

    // Step 5: Read data again (from TstSrv) (six read operations) 
    {
        uint32_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 30U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ONE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 31U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_TWO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 32U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_THREE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 33U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FIVE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 35U);
    }

    // Step 6: Read data (from Application) (one read operations) 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 30U);
        EXPECT_EQ(buffer.field1, 31U);
        EXPECT_EQ(buffer.field2, 32U);
        EXPECT_EQ(buffer.field3, 33U);
        EXPECT_EQ(buffer.field4, manipulatedValue);
        EXPECT_EQ(buffer.field5, 35U);
    }

    // Step 7: Deactivate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_endManipulation(dataPacketID1, OFFSET_FOUR));
    }

    // Step 8: Read data again (from Application). 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 30U);
        EXPECT_EQ(buffer.field1, 31U);
        EXPECT_EQ(buffer.field2, 32U);
        EXPECT_EQ(buffer.field3, 33U);
        EXPECT_EQ(buffer.field4, 34U);
        EXPECT_EQ(buffer.field5, 35U);
    }
}

TEST_F(DataHandlerTestProbeInterfaceTest, TwoManipulationsWithWrite)
{
    uint32_t manipulatedValue1 = 24U;
    uint32_t manipulatedValue2 = 22U;
    uint32_t bytesRead;
    uint32_t bytesWritten;


    // Purspose of the test:
    // Activate a manipulation, manipulate from TstPrb, write from Application, read the values, and deactivate the manipulation. 

    // Step 1a: Activate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_startManipulation(dataPacketID1, OFFSET_FOUR));
    }

    // Step 2a: Manipulate data.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &manipulatedValue1, sizeof(uint32_t), &bytesWritten)); 
        EXPECT_EQ(sizeof(uint32_t), bytesWritten);
    }

    // Step 1b: Activate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_startManipulation(dataPacketID1, OFFSET_TWO));
    }

    // Step 2b: Manipulate data.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_TWO, &manipulatedValue2, sizeof(uint32_t), &bytesWritten)); 
        EXPECT_EQ(sizeof(uint32_t), bytesWritten);
    }

    // Step 3: Read data (from TstSrv) (six read operations) 
    {
        uint32_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 10U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ONE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 11U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_TWO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue2);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_THREE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 13U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue1);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FIVE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 15U);
    }

    // Step 3: Read data (from Application) (one read operations) 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 10U);
        EXPECT_EQ(buffer.field1, 11U);
        EXPECT_EQ(buffer.field2, manipulatedValue2);
        EXPECT_EQ(buffer.field3, 13U);
        EXPECT_EQ(buffer.field4, manipulatedValue1);
        EXPECT_EQ(buffer.field5, 15U);
    }

    // Step 4: Write new values. 
    {
        xme_core_dataHandler_topic_test_t newTestTopic;

        newTestTopic.field0 = 30U;
        newTestTopic.field1 = 31U;
        newTestTopic.field2 = 32U;
        newTestTopic.field3 = 33U;
        newTestTopic.field4 = 34U;
        newTestTopic.field5 = 35U;

        EXPECT_EQ(XME_STATUS_SUCCESS, 
            xme_core_dataHandler_writeData(dataPacketID1, &newTestTopic, sizeof(xme_core_dataHandler_topic_test_t)));
    }

    // Step 5: Read data again (from TstSrv) (six read operations) 
    {
        uint32_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 30U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ONE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 31U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_TWO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue2);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_THREE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 33U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue1);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FIVE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 35U);
    }

    // Step 6: Read data (from Application) (one read operations) 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 30U);
        EXPECT_EQ(buffer.field1, 31U);
        EXPECT_EQ(buffer.field2, manipulatedValue2);
        EXPECT_EQ(buffer.field3, 33U);
        EXPECT_EQ(buffer.field4, manipulatedValue1);
        EXPECT_EQ(buffer.field5, 35U);
    }

    // Step 7: Deactivate the manipulations.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_endManipulation(dataPacketID1, OFFSET_FOUR));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_endManipulation(dataPacketID1, OFFSET_TWO));
    }

    // Step 8: Read data again (from Application). 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 30U);
        EXPECT_EQ(buffer.field1, 31U);
        EXPECT_EQ(buffer.field2, 32U);
        EXPECT_EQ(buffer.field3, 33U);
        EXPECT_EQ(buffer.field4, 34U);
        EXPECT_EQ(buffer.field5, 35U);
    }
}

TEST_F(DataHandlerTestProbeInterfaceTest, TwoManipulationsEndingInDifferentTime)
{
    uint32_t manipulatedValue1 = 24U;
    uint32_t manipulatedValue2 = 22U;
    uint32_t bytesRead;
    uint32_t bytesWritten;


    // Purspose of the test:
    // Activate a manipulation, manipulate from TstPrb, write from Application, read the values, and deactivate the manipulation. 

    // Step 1a: Activate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_startManipulation(dataPacketID1, OFFSET_FOUR));
    }

    // Step 2a: Manipulate data.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &manipulatedValue1, sizeof(uint32_t), &bytesWritten)); 
        EXPECT_EQ(sizeof(uint32_t), bytesWritten);
    }

    // Step 1b: Activate the manipulation.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_startManipulation(dataPacketID1, OFFSET_TWO));
    }

    // Step 2b: Manipulate data.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_TWO, &manipulatedValue2, sizeof(uint32_t), &bytesWritten)); 
        EXPECT_EQ(sizeof(uint32_t), bytesWritten);
    }

    // Step 3: Read data (from TstSrv) (six read operations) 
    {
        uint32_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 10U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ONE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 11U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_TWO, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue2);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_THREE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 13U);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FOUR, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, manipulatedValue1);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_FIVE, &buffer, sizeof(uint32_t), &bytesRead)); 
        EXPECT_EQ(sizeof(uint32_t), bytesRead);
        EXPECT_EQ(buffer, 15U);
    }

    // Step 3: Read data (from Application) (one read operations) 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 10U);
        EXPECT_EQ(buffer.field1, 11U);
        EXPECT_EQ(buffer.field2, manipulatedValue2);
        EXPECT_EQ(buffer.field3, 13U);
        EXPECT_EQ(buffer.field4, manipulatedValue1);
        EXPECT_EQ(buffer.field5, 15U);
    }

    // Step 7: Deactivate the manipulations.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_endManipulation(dataPacketID1, OFFSET_FOUR));
    }

    // Step 8: Read data again (from Application). 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 10U);
        EXPECT_EQ(buffer.field1, 11U);
        EXPECT_EQ(buffer.field2, manipulatedValue2);
        EXPECT_EQ(buffer.field3, 13U);
        EXPECT_EQ(buffer.field4, 14U);
        EXPECT_EQ(buffer.field5, 15U);
    }

    // Step 7: Deactivate the manipulations.
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_endManipulation(dataPacketID1, OFFSET_TWO));
    }

    // Step 8: Read data again (from Application). 
    {
        xme_core_dataHandler_topic_test_t buffer;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(xme_core_dataHandler_topic_test_t), &bytesRead)); 
        EXPECT_EQ(sizeof(xme_core_dataHandler_topic_test_t), bytesRead);
        EXPECT_EQ(buffer.field0, 10U);
        EXPECT_EQ(buffer.field1, 11U);
        EXPECT_EQ(buffer.field2, 12U);
        EXPECT_EQ(buffer.field3, 13U);
        EXPECT_EQ(buffer.field4, 14U);
        EXPECT_EQ(buffer.field5, 15U);
    }
}

TEST_F(DataHandlerTestProbeInterfaceTest, WriteDataInDatabaseWithDifferentDatabaseIndices)
{
    uint32_t offset = 0U;
    uint32_t buffer = 24U;
    uint32_t bytesWritten;

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeDataInDatabase(0U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(0U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(2U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_dataHandlerTestProbe_writeDataInDatabase(3U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(3U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
}

TEST_F(DataHandlerTestProbeInterfaceTest, WriteDataInDatabaseWithDifferentDataStores)
{
    uint32_t offset = 0U;
    uint32_t buffer = 24U;
    uint32_t bytesWritten;
    
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, 42U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, 42U, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
}

TEST_F(DataHandlerTestProbeInterfaceTest, WriteDataInDatabaseWithDifferentOffsets)
{
    uint32_t buffer = 24U;
    uint32_t bytesWritten;
    uint8_t offset;
    
    for (offset = 0; offset < 24U; offset = offset + 4U)
    {
        ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, sizeof(buffer), &bytesWritten));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
        ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, 42U, offset, &buffer, sizeof(buffer), &bytesWritten));
    }

    offset = offset + 4U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, 42U, offset, &buffer, sizeof(buffer), &bytesWritten));

    offset = 0U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, 42U, offset, 1U, &buffer, sizeof(buffer), &bytesWritten));

    offset = 4U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, 42U, offset, 1U, &buffer, sizeof(buffer), &bytesWritten));
}

TEST_F(DataHandlerTestProbeInterfaceTest, WriteDataInDatabaseWithDifferentBufferSize)
{
    uint32_t offset = 0U;
    uint32_t buffer = 24U;
    uint32_t bytesWritten;
    uint32_t bufferSize;
    
    // Buffersize = 0
    bufferSize = 0U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, 42U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, 42U, 1U, offset, &buffer, bufferSize, &bytesWritten));

    // Buffersize = sizeof(buffer) - 1U
    bufferSize = sizeof(buffer) - 1U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, 42U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, 42U, 1U, offset, &buffer, bufferSize, &bytesWritten));

    // Buffersize = sizeof(buffer)
    bufferSize = sizeof(buffer);
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, 42U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, 42U, 1U, offset, &buffer, bufferSize, &bytesWritten));

    // Buffersize = sizeof(buffer) + 1U
    bufferSize = sizeof(buffer) + 1U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, 42U, offset, &buffer, bufferSize, &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, 42U, 1U, offset, &buffer, bufferSize, &bytesWritten));
}

TEST_F(DataHandlerTestProbeInterfaceTest, WriteDataInDatabaseWithNullBuffer)
{
    uint32_t offset = 0U;
    uint32_t bytesWritten;
    
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, NULL, sizeof(uint32_t), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, NULL, sizeof(uint32_t), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, NULL, sizeof(uint32_t), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, NULL, sizeof(uint32_t), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, 42U, offset, NULL, sizeof(uint32_t), &bytesWritten));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, 42U, 1U, offset, NULL, sizeof(uint32_t), &bytesWritten));
}

TEST_F(DataHandlerTestProbeInterfaceTest, WriteDataInDatabaseWithDifferentNullWrittenBytesOutputVariable)
{
    uint32_t offset = 0U;
    uint32_t buffer = 24U;
    
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, 42U, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, 42U, 1U, offset, &buffer, sizeof(buffer), NULL));
}

TEST_F(DataHandlerTestProbeInterfaceTest, ReadDataInDatabaseWithDifferentDatabaseIndices)
{
    uint32_t offset = 0U;
    uint32_t buffer = 24U;
    uint32_t bytesRead;
    uint32_t bytesWritten;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(2U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(2U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(0U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readAttributeInDatabase(0U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(2U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_dataHandlerTestProbe_readDataInDatabase(3U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_dataHandlerTestProbe_readAttributeInDatabase(3U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
}

TEST_F(DataHandlerTestProbeInterfaceTest, ReadDataInDatabaseWithDifferentDataStores)
{
    uint32_t offset = 0U;
    uint32_t buffer = 24U;
    uint32_t bytesRead;
    uint32_t bytesWritten;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, 42U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, 42U, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
}

TEST_F(DataHandlerTestProbeInterfaceTest, ReadDataInDatabaseWithDifferentOffsets)
{
    uint32_t buffer = 24U;
    uint32_t bytesRead;
    uint8_t offset;
    uint32_t bytesWritten;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, 0U, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, 0U, &buffer, sizeof(buffer), &bytesWritten));
    
    for (offset = 0; offset < 24U; offset = offset + 4U)
    {
        ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, sizeof(buffer), &bytesRead));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesRead));
        ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, 42U, offset, &buffer, sizeof(buffer), &bytesRead));
    }

    offset = offset + 4U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, 42U, offset, &buffer, sizeof(buffer), &bytesRead));

    offset = 0U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, 42U, offset, 1U, &buffer, sizeof(buffer), &bytesRead));

    offset = 4U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, 42U, offset, 1U, &buffer, sizeof(buffer), &bytesRead));
}

TEST_F(DataHandlerTestProbeInterfaceTest, ReadDataInDatabaseWithDifferentBufferSize)
{
    uint32_t offset = 0U;
    uint32_t writeBuffer = 24U;
    char buffer[5];
    uint32_t bytesRead;
    uint32_t bufferSize;
    uint32_t bytesWritten;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &writeBuffer, sizeof(writeBuffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &writeBuffer, sizeof(writeBuffer), &bytesWritten));
    
    // Buffersize = 0
    bufferSize = 0U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, dataPacketID1, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, 42U, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, 42U, 1U, offset, &buffer, bufferSize, &bytesRead));

    // Buffersize = sizeof(buffer) - 1U
    bufferSize = sizeof(buffer) - 1U;
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, dataPacketID1, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, 42U, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, 42U, 1U, offset, &buffer, bufferSize, &bytesRead));

    // Buffersize = sizeof(buffer)
    bufferSize = sizeof(buffer);
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, dataPacketID1, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, 42U, offset, &buffer, bufferSize, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, 42U, 1U, offset, &buffer, bufferSize, &bytesRead));
}

TEST_F(DataHandlerTestProbeInterfaceTest, ReadDataInDatabaseWithNullBuffer)
{
    uint32_t offset = 0U;
    uint32_t buffer = 24U;
    uint32_t bytesRead;
    uint32_t bytesWritten;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, NULL, sizeof(uint32_t), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, NULL, sizeof(uint32_t), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, dataPacketID1, offset, NULL, sizeof(uint32_t), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, NULL, sizeof(uint32_t), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, 42U, offset, NULL, sizeof(uint32_t), &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, 42U, 1U, offset, NULL, sizeof(uint32_t), &bytesRead));
}

TEST_F(DataHandlerTestProbeInterfaceTest, ReadDataInDatabaseWithDifferentNullReadBytesOutputVariable)
{
    uint32_t offset = 0U;
    uint32_t buffer = 24U;
    uint32_t bytesWritten;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), &bytesWritten));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), &bytesWritten));
    
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, dataPacketID1, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, dataPacketID1, 1U, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readDataInDatabase(1U, 42U, offset, &buffer, sizeof(buffer), NULL));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_readAttributeInDatabase(1U, 42U, 1U, offset, &buffer, sizeof(buffer), NULL));
}

TEST_F(DataHandlerTestProbeInterfaceTest, ActivateDatabaseWithDifferentValues)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(0U, dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(1U, dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(2U, dataPacketID1));
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(3U, dataPacketID1));
}

TEST_F(DataHandlerTestProbeInterfaceTest, ActivateDatabaseWithInvalidDataPacket)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(0U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(2U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(3U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DataHandlerTestProbeInterfaceTest, StartAndEndManipulationWithInvalidDataPacketID)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_startManipulation(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_endManipulation(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_startAttributeManipulation(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 0U, 1U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandlerTestProbe_endAttributeManipulation(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 0U, 1U));
}


int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
