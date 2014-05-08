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
 * $Id: smokeTestDataHandler.cpp 7757 2014-03-11 10:08:27Z ruiz $
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
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"
#include "xme/core/testUtils.h"
#include "xme/hal/include/mem.h"

#include "xme/core/broker/include/broker.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define BUFFER ((int) 16)
#define DATAHANDLER_TEST_MEMORYVALUE 10

/******************************************************************************/
/***   Class Definition                                                     ***/
/******************************************************************************/

// NOTE: These are old tests associated to the DataHandler. 
class DataHandlerSmokeTest : public ::testing::Test {
    protected:
        DataHandlerSmokeTest() :
                        component(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT), type(
                                        XME_CORE_COMPONENT_PORTTYPE_INVALID), topic(
                                        XME_CORE_TOPIC_INVALID_TOPIC), inport(
                                        XME_CORE_DATAMANAGER_DATAPACKETID_INVALID), outport(
                                        XME_CORE_DATAMANAGER_DATAPACKETID_INVALID), key(
                                        XME_CORE_ATTRIBUTE_KEY_UNDEFINED), got(0) {

        }

        virtual
        ~DataHandlerSmokeTest() {
        }

        xme_core_component_t component;
        xme_core_component_portType_t type;
        xme_core_topic_t topic;
        xme_core_dataManager_dataPacketId_t inport;
        xme_core_dataManager_dataPacketId_t outport;
        xme_core_attribute_key_t key;
        unsigned int got;
        char buffer[BUFFER ];
};

typedef DataHandlerSmokeTest DataHandlerSmokeTestDeathTest;

/******************************************************************************/

// NOTE: These are new tests associated to the new interface database.
// As soon as we reproduce old tests, old tests should be removed. 

class DataHandlerNewSmokeTest : public ::testing::Test {
    protected:
        DataHandlerNewSmokeTest()
        {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
        }

        virtual
        ~DataHandlerNewSmokeTest() {
            xme_core_broker_fini();
        }

    // This is only for storing the output data packets. 
    xme_core_dataManager_dataPacketId_t dataPacketID1;
    xme_core_dataManager_dataPacketId_t dataPacketID2;
    xme_core_dataManager_dataPacketId_t dataPacketID3;

    xme_core_dataManager_memoryRegionID_t defaultMemoryRegion;
};

class DataHandlerBasicSmokeTest : public ::testing::Test {
    protected:
        DataHandlerBasicSmokeTest()
        {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());
        }

        virtual
        ~DataHandlerBasicSmokeTest() {
            xme_core_dataHandler_fini();
            xme_core_broker_fini();
        }

    // This is only for storing the output data packets. 
    xme_core_dataManager_dataPacketId_t dataPacketID1;
    xme_core_dataManager_dataPacketId_t dataPacketID2;
    xme_core_dataManager_dataPacketId_t dataPacketID3;

    xme_core_dataManager_memoryRegionID_t defaultMemoryRegion;
};

typedef DataHandlerSmokeTest DataHandlerSmokeTestDeathTest;

class DataHandleNewQueueSmokeTest : public ::testing::Test {
    protected:
        DataHandleNewQueueSmokeTest()
        {

    // <-- Data Packet 1 -->
    // ---------------------
    // |     4    |  4 |  4 | (size in bytes)
    // ---------------------
    //              A1   A2  Attributes

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
            // Initialize the database.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            // Create Data Packet 1
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataPacketID1));
            EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataPacketID1);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataPacketID1)); // key 1
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 2U, dataPacketID1)); // key 2

            // Set queue size to 5.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataPacketID1, 5U));

            // Configuration of the database. 
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
        }

        virtual
        ~DataHandleNewQueueSmokeTest() {
            xme_core_broker_fini();
            xme_core_dataHandler_fini();
        }

    // This is only for storing the output data packets. 
    xme_core_dataManager_dataPacketId_t dataPacketID1;
};

class DataHandleNewQueueWitTransferSmokeTest : public ::testing::Test {
    protected:
        DataHandleNewQueueWitTransferSmokeTest()
        {

    // <-- Data Packet 1 --> (publication)
    // ---------------------
    // |     4    |  4 |  4 | (size in bytes)
    // ---------------------
    //              A1   A2  Attributes

    // <-- Data Packet 2 --> (subscription)
    // ---------------------
    // |     4    |  4 |  4 | (size in bytes)
    // ---------------------
    //              A1   A2  Attributes

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
            // Initialize the database.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            // Create Data Packet 1
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataPacketID1));
            EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataPacketID1);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataPacketID1)); // key 1
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 2U, dataPacketID1)); // key 2

            // Set queue size to 5.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataPacketID1, 5U));

            // Create Data Packet 2
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataPacketID2));
            EXPECT_EQ((xme_core_dataManager_dataPacketId_t)2, dataPacketID2);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataPacketID2)); // key 1
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 2U, dataPacketID2)); // key 2

            // Set queue size to 5.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataPacketID2, 5U));

            // Configuration of the database. 
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(dataPacketID1, dataPacketID2));
        }

        virtual
        ~DataHandleNewQueueWitTransferSmokeTest() {
            xme_core_broker_fini();
            xme_core_dataHandler_fini();
        }

    // This is only for storing the output data packets. 
    xme_core_dataManager_dataPacketId_t dataPacketID1;
    xme_core_dataManager_dataPacketId_t dataPacketID2;
};

class DataHandleNewQueueWitTransferSmokeTest_Issue4015 : public ::testing::Test {
    protected:
        DataHandleNewQueueWitTransferSmokeTest_Issue4015()
        {

    // <-- Data Packet 1 --> (publication)
    // ---------------------
    // |     4    |  4 |  4 | (size in bytes)
    // ---------------------
    //              A1   A2  Attributes

    // <-- Data Packet 2 --> (subscription)
    // ---------------------
    // |     4    |  4 |  4 | (size in bytes)
    // ---------------------
    //              A1   A2  Attributes

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
            // Initialize the database.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            // Create Data Packet 1
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataPacketID1));
            EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataPacketID1);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataPacketID1)); // key 1
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 2U, dataPacketID1)); // key 2

            // Create Data Packet 2
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataPacketID2));
            EXPECT_EQ((xme_core_dataManager_dataPacketId_t)2, dataPacketID2);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataPacketID2)); // key 1
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 2U, dataPacketID2)); // key 2

            // Set queue size to 2.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataPacketID2, 2U));

            // Configuration of the database. 
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(dataPacketID1, dataPacketID2));
        }

        virtual
        ~DataHandleNewQueueWitTransferSmokeTest_Issue4015() {
            xme_core_broker_fini();
            xme_core_dataHandler_fini();
        }

    // This is only for storing the output data packets. 
    xme_core_dataManager_dataPacketId_t dataPacketID1;
    xme_core_dataManager_dataPacketId_t dataPacketID2;
};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

TEST_F(DataHandlerNewSmokeTest, InitialTest)
{
    uint32_t bytesRead;

    // <--- Data Packet 1 ---><----- Data Packet 2 -----><--- DP3 --->
    // --------------------------------------------------------------
    // |     24    |  4 | 16 ||    128    | 10 |  8 | 16 ||    128    | (size in bytes)
    // --------------------------------------------------------------
    //               A1   A2                A1   A2   A3                   Attributes

    // Write buffers
    char writeBufferDP1[24] = "asdf";
    char writeBufferDP3[128] = "qwertz";
    char writeBufferDP1A1[4] = "123"; // + \0
    
    // Read buffers. 
    char readBufferDP1[24];
    char readBufferDP1A1[4];
    char readBufferDP1A2[16];
    char readBufferDP2[128];
    char readBufferDP2A1[10];
    char readBufferDP3[128];

    // Initialize the database.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

    // Create Data Packet 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(24U, &dataPacketID1));
    ASSERT_EQ((xme_core_dataManager_dataPacketId_t)1, dataPacketID1);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataPacketID1)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(16U, 2U, dataPacketID1)); // key 2

    // xme_core_dataHandler_setQueueSize(dataPacketID1, 4);

    // Create data packet 2. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(128U, &dataPacketID2));
    ASSERT_EQ((xme_core_dataManager_dataPacketId_t)2, dataPacketID2);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(10U, 1U, dataPacketID2)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(8U, 2U, dataPacketID2)); // key 2
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(16U, 3U, dataPacketID2)); // key 3

    // Create data packet 3. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(128U, &dataPacketID3));
    ASSERT_EQ((xme_core_dataManager_dataPacketId_t)3, dataPacketID3);

    // Configuration of the database. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Write operations. 

    // Write DP1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, writeBufferDP1, sizeof(writeBufferDP1))); 
    // Write DP1-Attribute Key 1. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, writeBufferDP1A1, sizeof(writeBufferDP1A1))); 

    // Write DP3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID3, writeBufferDP3, sizeof(writeBufferDP3))); 

    // Read operations. 
    // Read DP1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, readBufferDP1, sizeof(readBufferDP1), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP1), bytesRead);
    EXPECT_EQ(0, strcmp(writeBufferDP1, readBufferDP1));

    // Read DP1-Attribute Key 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, readBufferDP1A1, sizeof(readBufferDP1A1), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP1A1), bytesRead);
    EXPECT_EQ(0, strcmp(writeBufferDP1A1, readBufferDP1A1));

    // Read DP1-Attribute-Key 2. Reading something that has not been initialized (no previous write). 
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, readBufferDP1A2, sizeof(readBufferDP1A2), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);

    // Read DP2. Error expected (no previous write). 
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readData(dataPacketID2, readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);

    // Read DP2-Attribute-Key 1. Error expected (no previous write). 
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, readBufferDP2A1, sizeof(readBufferDP2A1), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);

    // Read DP3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID3, readBufferDP3, sizeof(readBufferDP3), &bytesRead)); 
    //EXPECT_EQ(128U, bytesRead);
    EXPECT_EQ(0, strcmp(writeBufferDP3, readBufferDP3));

    // Finalize the database. 
    xme_core_dataHandler_fini();
}

TEST_F(DataHandleNewQueueSmokeTest, InitialTest)
{

    // Write buffers
    uint32_t writeBufferDP1Queue1 = 1U;
    uint32_t writeBufferDP1Queue2 = 2U;
    uint32_t writeBufferDP1Queue3 = 3U;
    uint32_t writeBufferDP1Queue4 = 4U;
    uint32_t writeBufferDP1Queue5 = 5U;

    uint32_t writeBufferDP1A1Queue1 = 11U;
    uint32_t writeBufferDP1A2Queue1 = 12U;
    uint32_t writeBufferDP1A1Queue2 = 21U;
    uint32_t writeBufferDP1A2Queue2 = 22U;
    uint32_t writeBufferDP1A1Queue3 = 31U;
    uint32_t writeBufferDP1A2Queue3 = 32U;
    uint32_t writeBufferDP1A1Queue4 = 41U;
    uint32_t writeBufferDP1A2Queue4 = 42U;
    uint32_t writeBufferDP1A1Queue5 = 51U;
    uint32_t writeBufferDP1A2Queue5 = 52U;

    // Read buffers
    uint32_t bytesRead;
    uint32_t readBuffer;


    // Write data in the queue. 
    // Write Position 1. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue1, sizeof(writeBufferDP1Queue1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue1, sizeof(writeBufferDP1A1Queue1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue1, sizeof(writeBufferDP1A2Queue1))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 2. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue2, sizeof(writeBufferDP1Queue2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue2, sizeof(writeBufferDP1A1Queue2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue2, sizeof(writeBufferDP1A2Queue2))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 3. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue3, sizeof(writeBufferDP1Queue3))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue3, sizeof(writeBufferDP1A1Queue3))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue3, sizeof(writeBufferDP1A2Queue3))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 4. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue4, sizeof(writeBufferDP1Queue4))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue4, sizeof(writeBufferDP1A1Queue4))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue4, sizeof(writeBufferDP1A2Queue4))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 5. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue5, sizeof(writeBufferDP1Queue5))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue5, sizeof(writeBufferDP1A1Queue5))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue5, sizeof(writeBufferDP1A2Queue5))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Data should look like:
    // <-- Queue Pos 1    --><-- Queue Pos 2    --><-- Queue Pos 3    --><-- Queue Pos 4    --><-- Queue Pos 5    -->
    // --------------------------------------------------------------------------------------------------------------
    // |     1    | 11 | 12 ||     2    | 21 | 22 ||     3    | 31 | 32 ||     4    | 41 | 42 ||     5    | 51 | 52 |
    // --------------------------------------------------------------------------------------------------------------
    // ^
    // |
    // read position

    // Read DP1 (expected queue pos 1).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue1, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    // Read DP1 (expected queue pos 2).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue2, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    // Read DP1 (expected queue pos 3).
    // Test for Ticket#3993
    // Read first attribute!!
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue3, readBuffer);
    // Discard this data packet (e.g. attribute filtering do not match with a criteria). 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    // Read DP1 (expected queue pos 4).
    // Read first attribute!!
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue4, readBuffer);
    // Accept this data packet (e.g. attribute filtering do match with a criteria). 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue4, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue4, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    // Read DP1 (expected queue pos 5).
    // Read now attribute!!
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue5, readBuffer);
    // Accept this data packet (e.g. attribute filtering do match with a criteria). 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue5, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue5, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));
}

TEST_F(DataHandleNewQueueSmokeTest, Issue4187_UnorderedWritesDataAndAttributes)
{

    // Write buffers
    uint32_t writeBufferDP1Queue1 = 1U;
    uint32_t writeBufferDP1Queue2 = 2U;
    uint32_t writeBufferDP1Queue3 = 3U;

    uint32_t writeBufferDP1A1Queue1 = 11U;
    uint32_t writeBufferDP1A2Queue1 = 12U;
    uint32_t writeBufferDP1A1Queue2 = 21U;
    uint32_t writeBufferDP1A2Queue3 = 32U;
    uint32_t writeBufferDP1A1Queue4 = 41U;

    // Read buffers
    uint32_t bytesRead;
    uint32_t readBuffer;


    // Write data in the queue. 
    // Write Position 1. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue1, sizeof(writeBufferDP1Queue1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue1, sizeof(writeBufferDP1A1Queue1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue1, sizeof(writeBufferDP1A2Queue1))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 2. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue2, sizeof(writeBufferDP1Queue2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue2, sizeof(writeBufferDP1A1Queue2))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 3. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue3, sizeof(writeBufferDP1Queue3))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue3, sizeof(writeBufferDP1A2Queue3))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 4. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue4, sizeof(writeBufferDP1A1Queue4))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 5. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Data should look like:
    // <-- Queue Pos 1    --><-- Queue Pos 2    --><-- Queue Pos 3    --><-- Queue Pos 4    --><-- Queue Pos 5    -->
    // --------------------------------------------------------------------------------------------------------------
    // |     1    | 11 | 12 ||     2    | 21 | XX ||     3    | XX | 32 ||     X    | 41 | XX ||     X    | XX | XX |
    // --------------------------------------------------------------------------------------------------------------
    // ^
    // |
    // read position

    // Read DP1 (expected queue pos 1).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue1, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    // Read DP1 (expected queue pos 2).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    //ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    // Read DP1 (expected queue pos 3).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue3, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    //ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue3, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    // Read DP1 (expected queue pos 4).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    //ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue4, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    //ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    // Read DP1 (expected queue pos 5).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    //ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_readData(dataPacketID1, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    //ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_readAttribute(dataPacketID1, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    //ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_readAttribute(dataPacketID1, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));
}

TEST_F(DataHandleNewQueueWitTransferSmokeTest, InitialTest)
{

    // Write buffers
    uint32_t writeBufferDP1Queue1 = 1U;
    uint32_t writeBufferDP1Queue2 = 2U;
    uint32_t writeBufferDP1Queue3 = 3U;
    uint32_t writeBufferDP1Queue4 = 4U;
    uint32_t writeBufferDP1Queue5 = 5U;

    uint32_t writeBufferDP1A1Queue1 = 11U;
    uint32_t writeBufferDP1A2Queue1 = 12U;
    uint32_t writeBufferDP1A1Queue2 = 21U;
    uint32_t writeBufferDP1A2Queue2 = 22U;
    uint32_t writeBufferDP1A1Queue3 = 31U;
    uint32_t writeBufferDP1A2Queue3 = 32U;
    uint32_t writeBufferDP1A1Queue4 = 41U;
    uint32_t writeBufferDP1A2Queue4 = 42U;
    uint32_t writeBufferDP1A1Queue5 = 51U;
    uint32_t writeBufferDP1A2Queue5 = 52U;

    // Read buffers
    uint32_t bytesRead;
    uint32_t readBuffer;


    // Write data in the queue. 
    // Write Position 1. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue1, sizeof(writeBufferDP1Queue1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue1, sizeof(writeBufferDP1A1Queue1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue1, sizeof(writeBufferDP1A2Queue1))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 2. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue2, sizeof(writeBufferDP1Queue2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue2, sizeof(writeBufferDP1A1Queue2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue2, sizeof(writeBufferDP1A2Queue2))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 3. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue3, sizeof(writeBufferDP1Queue3))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue3, sizeof(writeBufferDP1A1Queue3))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue3, sizeof(writeBufferDP1A2Queue3))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 4. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue4, sizeof(writeBufferDP1Queue4))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue4, sizeof(writeBufferDP1A1Queue4))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue4, sizeof(writeBufferDP1A2Queue4))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 5. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue5, sizeof(writeBufferDP1Queue5))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue5, sizeof(writeBufferDP1A1Queue5))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue5, sizeof(writeBufferDP1A2Queue5))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Data should look like:
    // <-- Queue Pos 1    --><-- Queue Pos 2    --><-- Queue Pos 3    --><-- Queue Pos 4    --><-- Queue Pos 5    -->
    // --------------------------------------------------------------------------------------------------------------
    // |     1    | 11 | 12 ||     2    | 21 | 22 ||     3    | 31 | 32 ||     4    | 41 | 42 ||     5    | 51 | 52 |
    // --------------------------------------------------------------------------------------------------------------
    // ^
    // |
    // read position

    // (transfers take place after each complete write operation)
    // Read DP 2 (expected queue pos 1).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue1, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));

    // Read DP1 (expected queue pos 2).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue2, readBuffer);

    // Read a second time. 
    // Expected the same result. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue2, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));

    // Read DP1 (expected queue pos 3).
    // Test for Ticket#3993
    // Read first attribute!!
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue3, readBuffer);
    // Discard this data packet (e.g. attribute filtering do not match with a criteria). 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));

    // Read DP1 (expected queue pos 4).
    // Read first attribute!!
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue4, readBuffer);
    // Accept this data packet (e.g. attribute filtering do match with a criteria). 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue4, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue4, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));

    // Read DP1 (expected queue pos 5).
    // Read now attribute!!
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue5, readBuffer);
    // Accept this data packet (e.g. attribute filtering do match with a criteria). 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue5, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue5, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandleNewQueueWitTransferSmokeTest, Issue4187_UnorderedWritesDataAndAttributes)
{

    // Write buffers
    uint32_t writeBufferDP1Queue1 = 1U;
    uint32_t writeBufferDP1Queue2 = 2U;
    uint32_t writeBufferDP1Queue3 = 3U;

    uint32_t writeBufferDP1A1Queue1 = 11U;
    uint32_t writeBufferDP1A2Queue1 = 12U;
    uint32_t writeBufferDP1A1Queue2 = 21U;
    uint32_t writeBufferDP1A2Queue3 = 32U;
    uint32_t writeBufferDP1A1Queue4 = 41U;

    // Read buffers
    uint32_t bytesRead;
    uint32_t readBuffer;

    // Write data in the queue. 
    // Write Position 1: WriteData + WriteAttribute1 + WriteAttribute2
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue1, sizeof(writeBufferDP1Queue1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue1, sizeof(writeBufferDP1A1Queue1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue1, sizeof(writeBufferDP1A2Queue1))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 2: WriteData + WriteAttribute1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue2, sizeof(writeBufferDP1Queue2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue2, sizeof(writeBufferDP1A1Queue2))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 3: WriteData + WriteAttribute2
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Queue3, sizeof(writeBufferDP1Queue3))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Queue3, sizeof(writeBufferDP1A2Queue3))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 4:WriteAttribute1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Queue4, sizeof(writeBufferDP1A1Queue4))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 5: Do not write.
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Data should look like:
    // <-- Queue Pos 1    --><-- Queue Pos 2    --><-- Queue Pos 3    --><-- Queue Pos 4    --><-- Queue Pos 5    -->
    // --------------------------------------------------------------------------------------------------------------
    // |     1    | 11 | 12 ||     2    | 21 | XX ||     3    | XX | 32 ||     X    | 41 | XX ||     X    | XX | XX |
    // --------------------------------------------------------------------------------------------------------------
    // ^
    // |
    // read position

    // (transfers take place after each complete write operation)
    // Read DP 2 (expected queue pos 1).
    // Available data: DP1, A1, A2
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue1, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));

    // Read DP2 (expected queue pos 2).
    // Available data: DP1, A1.
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    //ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));

    // Read DP2 (expected queue pos 3).
    // Available data: DP1, A2.
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Queue3, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    //ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Queue3, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));

    // Read DP2 (expected queue pos 4).
    // Available data: A1.
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Queue4, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));

    // Read DP1 (expected queue pos 5).
    // Available data: No data expected.
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(0U, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

/*
TEST_F(DataHandlerSmokeTest, shutdownWithoutInit) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_fini());
#endif
#endif 

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_fini(),
                    "");
#endif
}

TEST_F(DataHandlerSmokeTestDeathTest, startupAndShutdownWithZeroMemory) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_init(0u));
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_fini());
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_init(0u),
                    "");
    ASSERT_DEATH(
                    xme_core_dataHandler_fini(),
                    "");
#endif
}

TEST_F(DataHandlerSmokeTest, startupAndShutdownWithSomeMemory) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_fini());
}

TEST_F(DataHandlerSmokeTestDeathTest, startupTwiceAndShutdownWithSomeMemory) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE));
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE),
                    "");
#endif
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_fini());
}

// FIXME: test missing for initalizing system with less memory than needed!

TEST_F(DataHandlerSmokeTest, createPortUnitialized) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_createPort(component, type, topic, BUFFER * sizeof(char), XME_CORE_NO_ATTRIBUTE, 0, false, false, 0, &inport));
}

TEST_F(DataHandlerSmokeTestDeathTest, readingUnitialized) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_readData(inport, buffer, BUFFER * sizeof(char), &got));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readData(inport, buffer, BUFFER * sizeof(char), &got),
                    "");
#endif
}

TEST_F(DataHandlerSmokeTestDeathTest, writingUnitialized) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_writeData(inport, buffer, BUFFER * sizeof(char)));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeData(inport, buffer, BUFFER * sizeof(char)),
                    "");
#endif
}

TEST_F(DataHandlerSmokeTestDeathTest, transferingUnitialized) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_transferData(inport, outport));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_transferData(inport, outport),
                    "");
#endif
}

TEST_F(DataHandlerSmokeTestDeathTest, readAttributeUnitialized) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readAttribute(inport, key, buffer, BUFFER * sizeof(char), &got));
#endif
#endif 

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readAttribute(inport, key, buffer, BUFFER * sizeof(char), &got),
                    "");
#endif
}

TEST_F(DataHandlerSmokeTestDeathTest, writeAttributeUnitialized) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeAttribute(inport, key, buffer, BUFFER * sizeof(char)));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeAttribute(inport, key, buffer, BUFFER * sizeof(char)),
                    "");
#endif
}

TEST_F(DataHandlerSmokeTest, completeReadOperationPortUnitialized) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_completeReadOperation(inport));
}

TEST_F(DataHandlerSmokeTest, completeWriteOperationPortUnitialized) {
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_dataHandler_completeWriteOperation(inport));
}
*/

TEST_F(DataHandleNewQueueWitTransferSmokeTest_Issue4015, Test_Issue4015)
{

    // Write buffers
    uint32_t writeBufferDP1Item1 = 1U;
    uint32_t writeBufferDP1Item2 = 2U;

    uint32_t writeBufferDP1A1Item1 = 11U;
    uint32_t writeBufferDP1A2Item1 = 12U;
    uint32_t writeBufferDP1A1Item2 = 21U;
    uint32_t writeBufferDP1A2Item2 = 22U;

    // Read buffers
    uint32_t bytesRead;
    uint32_t readBuffer;

    // Write data in the data store. 
    // Write Position 1. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Item1, sizeof(writeBufferDP1Item1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Item1, sizeof(writeBufferDP1A1Item1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Item1, sizeof(writeBufferDP1A2Item1))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Write Position 2. 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1Item2, sizeof(writeBufferDP1Item2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 1U, &writeBufferDP1A1Item2, sizeof(writeBufferDP1A1Item2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, 2U, &writeBufferDP1A2Item2, sizeof(writeBufferDP1A2Item2))); 
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Data should look like in the target data store (dataPacketID2):
    // <-- Queue Pos 1    --><-- Queue Pos 2    -->
    // --------------------------------------------
    // |     1    | 11 | 12 ||     2    | 21 | 22 |
    // --------------------------------------------
    // ^
    // |
    // read position

    // (transfers take place after each complete write operation)
    // Read DP 2 (expected queue pos 1).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Item1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Item1, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Item1, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));

    // Read DP1 (expected queue pos 2).
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1Item2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 1U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A1Item2, readBuffer);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, 2U, &readBuffer, sizeof(readBuffer), &bytesRead)); 
    EXPECT_EQ(sizeof(readBuffer), bytesRead);
    EXPECT_EQ(writeBufferDP1A2Item2, readBuffer);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteFalseStatusSuccessWarningsFalseNoQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, false, XME_STATUS_SUCCESS, false));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 

    // Overwrite.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteFalseStatusSuccessWarningsTrueNoQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, false, XME_STATUS_SUCCESS, true));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 

    // Overwrite.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteFalseStatusErrorWarningsFalseNoQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, false, XME_STATUS_BUFFER_TOO_SMALL, false));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 

    // Overwrite.
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteFalseStatusErrorWarningsTrueNoQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, false, XME_STATUS_BUFFER_TOO_SMALL, true));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 

    // Overwrite.
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteTrueStatusSuccessWarningsFalseNoQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, (bool) true, XME_STATUS_SUCCESS, (bool) false));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 

    // Overwrite.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteTrueStatusSuccessWarningsTrueNoQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, (bool) true, XME_STATUS_SUCCESS, (bool) true));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 

    // Overwrite.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteTrueStatusErrorWarningsFalseNoQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, (bool) true, XME_STATUS_BUFFER_TOO_SMALL, (bool) false));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 

    // Overwrite.
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteTrueStatusErrorWarningsTrueNoQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, (bool) true, XME_STATUS_BUFFER_TOO_SMALL, (bool) true));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 

    // Overwrite.
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteDefaultNoQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 

    // Overwrite.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteFalseStatusSuccessWarningsFalseQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, false, XME_STATUS_SUCCESS, false));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataStoreID, 2U));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Queue Size = 2. Three writes are needed for demonstrate the feature. 
    // Write 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 2.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteFalseStatusSuccessWarningsTrueQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, false, XME_STATUS_SUCCESS, true));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataStoreID, 2U));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Queue Size = 2. Three writes are needed for demonstrate the feature. 
    // Write 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 2.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteFalseStatusErrorWarningsFalseQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, false, XME_STATUS_BUFFER_TOO_SMALL, false));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataStoreID, 2U));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Queue Size = 2. Three writes are needed for demonstrate the feature. 
    // Write 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 2.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_completeWriteOperation(dataStoreID));
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteFalseStatusErrorWarningsTrueQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, false, XME_STATUS_BUFFER_TOO_SMALL, true));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataStoreID, 2U));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Queue Size = 2. Three writes are needed for demonstrate the feature. 
    // Write 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 2.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_completeWriteOperation(dataStoreID));
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteTrueStatusSuccessWarningsFalseQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, (bool) true, XME_STATUS_SUCCESS, (bool) false));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataStoreID, 2U));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Queue Size = 2. Three writes are needed for demonstrate the feature. 
    // Write 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 2.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteTrueStatusSuccessWarningsTrueQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, (bool) true, XME_STATUS_SUCCESS, (bool) true));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataStoreID, 2U));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Queue Size = 2. Three writes are needed for demonstrate the feature. 
    // Write 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 2.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteTrueStatusErrorWarningsFalseQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, (bool) true, XME_STATUS_BUFFER_TOO_SMALL, (bool) false));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataStoreID, 2U));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Queue Size = 2. Three writes are needed for demonstrate the feature. 
    // Write 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 2.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteTrueStatusErrorWarningsTrueQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketOverwrite(dataStoreID, (bool) true, XME_STATUS_BUFFER_TOO_SMALL, (bool) true));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataStoreID, 2U));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Queue Size = 2. Three writes are needed for demonstrate the feature. 
    // Write 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 2.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));
}

TEST_F(DataHandlerBasicSmokeTest, Issue4014_OverwriteDefaultQueue)
{
    xme_core_dataManager_dataStoreID_t dataStoreID;
    uint32_t dataStoreWriteDP1 = 1U;
    uint32_t dataStoreWriteDP1A1 = 11U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(4U, &dataStoreID));
    EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataStoreID);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, 1U, dataStoreID)); // key 1
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataStoreID, 2U));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Queue Size = 2. Three writes are needed for demonstrate the feature. 
    // Write 1.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 2.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));

    // Write 3.
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID, &dataStoreWriteDP1, sizeof(dataStoreWriteDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataStoreID, 1U, &dataStoreWriteDP1A1, sizeof(dataStoreWriteDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID));
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
