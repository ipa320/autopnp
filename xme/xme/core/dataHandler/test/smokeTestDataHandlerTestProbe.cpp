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
 * $Id: smokeTestDataHandlerTestProbe.cpp 7717 2014-03-07 16:27:02Z ruiz $
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
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/broker/include/broker.h"
#include "xme/core/testUtils.h"
#include "xme/hal/include/mem.h"

#define DB_MASTER 1
#define DB_SHADOW 2

#define OFFSET_ZERO 0U

/******************************************************************************/
/***   Class Definition                                                     ***/
/******************************************************************************/

/******************************************************************************/

// NOTE: These are new tests associated to the new interface database.
// As soon as we reproduce old tests, old tests should be removed. 

class DataHandlerTestProbeSmokeTest : public ::testing::Test {

    protected:
        DataHandlerTestProbeSmokeTest()
            : masterFirstWrittenData(-1234),
            shadowFirstWrittenData(5678),
            masterSecondWrittenData(3456),
            shadowSecondWrittenData(-7890)
        {
            // Initialize the database.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            // Create data packet 1. 
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(8U, &dataPacketID1));
            EXPECT_EQ((xme_core_dataManager_dataPacketId_t)1, dataPacketID1);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketPersistency(dataPacketID1, (bool)true));

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
        }

        virtual
        ~DataHandlerTestProbeSmokeTest() {
            // Finalize the database. 
            xme_core_dataHandler_fini();
            xme_core_broker_fini();
        }

    xme_core_dataManager_dataPacketId_t dataPacketID1;
    int32_t masterFirstWrittenData;
    int32_t shadowFirstWrittenData;
    int32_t masterSecondWrittenData;
    int32_t shadowSecondWrittenData;
};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

TEST_F(DataHandlerTestProbeSmokeTest, InitialTest)
{
    // TODO: Remove when changing the API. 
    uint32_t bytesRead;
    uint32_t bytesWritten;


    // Purspose of the test:
    // A full lifecycle of read/write operations, combining both TestProbe and Applications.
    //
    // Preconditions:
    // As DataHandler design provides two interfaces (named DataHandler.h and DataHandlerAdvanced.h)
    // They will be used as follows:
    // - DataHandler.h will be used by the Applications. 
    // - DataHandlerTest.h will be used by the TestProbe. 
    //
    // Steps in this test:
    // Step 1:
    // Perform a write operation over a data packet from an application and read that value. 
    // Step 2:
    // Write from TestProbe the shadow database and activate the shadow data packet.
    // Step 3: 
    // Application: Read again the data packet. The read value should be the one written by the TestProbe. 
    // Step 4:
    // Read from TestProbe in shadow database (read in master database is allowed, but theoretically, not used). 
    // Take into account that TestProbe can use less bytes than the topic size (to check). 
    // Step 5:
    // Write from TestProbe in both databases different values. 
    // Step 6: 
    // Read from standard application. This operation will read the value written by the TestProbe in Step 5
    // from shadow database. 
    // Step 7:
    // Activate the master database (aka deactivate the shadow database). 
    // Step 8:
    // Read from standard application. This operation will read the value written by the TestProbe in Step 5
    // from master database. 
    // Step 9:
    // Read from TestProbe both from Master Database and from Shadow Database. 

    // Database Status (before Step 1)
    // 
    //      ------------------------
    // DP1  |     0     |     0    |
    //      ------------------------
    //       Master(1)*   Shadow(2)

    // Step 1:
    // Perform a write operation over a data packet from an application and read that value. 
    {
        int32_t readDataDP1;

        // Start write operation (DP1).
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));

        // Write (from Application)
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &masterFirstWrittenData, sizeof(masterFirstWrittenData)));

        // Complete write operation (DP1). 
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

        // Read DP1 (from Application).
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readDataDP1, sizeof(readDataDP1), &bytesRead));
        EXPECT_EQ(sizeof(readDataDP1), bytesRead);
        EXPECT_EQ(masterFirstWrittenData, readDataDP1);
    }

    // Database Status (before Step 2)
    // 
    //      ------------------------
    // DP1  |    1234   |     0    |
    //      ------------------------
    //       Master(1)*   Shadow(2)

    // Step 2:
    // Write from TestProbe the shadow database and activate the shadow data packet.
    {
        // Write (from TestProbe in Shadow Database (2))
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &shadowFirstWrittenData, sizeof(shadowFirstWrittenData), &bytesWritten)); 
        EXPECT_EQ(sizeof(masterFirstWrittenData), bytesWritten);

        // TestProbe: Activate the data packet to be used by the applications. 
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(DB_SHADOW, dataPacketID1));
    }

    // Database Status (before Step 3)
    // 
    //      ------------------------
    // DP1  |    1234   |   5678    |
    //      ------------------------
    //       Master(1)   Shadow(2)*

    // Step 3: 
    // Application: Read again the data packet. The read value should be the one written by the TestProbe. 
    {
        // Read data. 
        int32_t readDataDP1;

        // Read DP1 (from Application).
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readDataDP1, sizeof(readDataDP1), &bytesRead));
        EXPECT_EQ(sizeof(readDataDP1), bytesRead);
        EXPECT_EQ(shadowFirstWrittenData, readDataDP1);
    }

    // Database Status (before Step 4)
    // 
    //      ------------------------
    // DP1  |   -1234   |   5678    |
    //      ------------------------
    //       Master(1)   Shadow(2)*

    // Step 4:
    // Read from TestProbe in shadow database (read in master database is allowed, but theoretically, not used). 
    // Take into account that TestProbe can use less bytes than the topic size (to check in a separate test). 
    {
        // Read datas. 
        int32_t readDataDP1;

        // Read DP1 (from TestProbe from Shadow Database(2)).
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &readDataDP1, sizeof(readDataDP1), &bytesRead));
        EXPECT_EQ(sizeof(readDataDP1), bytesRead);
        EXPECT_EQ(shadowFirstWrittenData, readDataDP1);
    }

    // Database Status (before Step 5)
    // 
    //      ------------------------
    // DP1  |   -1234   |   5678    |
    //      ------------------------
    //       Master(1)   Shadow(2)*

    // Step 5:
    // Write from TestProbe in both databases different values. 
    {
        // Write (from TestProbe in Master Database (1)) -- This is the directManipulation. 
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(DB_MASTER, dataPacketID1, OFFSET_ZERO, &masterSecondWrittenData, sizeof(masterSecondWrittenData), &bytesWritten)); 
        EXPECT_EQ(sizeof(masterSecondWrittenData), bytesWritten);
        // Note: For write operation from TestProbe is not needed to lock/unlock the data packet. 
        //       The call will directly invoke the write operation.

        // Write (from TestProbe in Shadow Database (2)) -- This is the manipulation. 
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_writeDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &shadowSecondWrittenData, sizeof(shadowSecondWrittenData), &bytesWritten)); 
        EXPECT_EQ(sizeof(shadowSecondWrittenData), bytesWritten);
        // We do need not to activate again, because it was previously activated by the TestProbe in the first write.
    }

    // Database Status (before Step 6)
    // 
    //      ------------------------
    // DP1  |   3456    |   -7890   |
    //      ------------------------
    //       Master(1)   Shadow(2)*

    // Step 6: 
    // Read from standard application. This operation will read the value written by the TestProbe in Step 5
    // from shadow database. 
    {
        // Read datas. 
        int32_t readDataDP1;

        // Read DP1 (from Application).
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readDataDP1, sizeof(readDataDP1), &bytesRead));
        EXPECT_EQ(sizeof(readDataDP1), bytesRead);
        EXPECT_EQ(shadowSecondWrittenData, readDataDP1);
    }

    // Database Status (before Step 7)
    // 
    //      ------------------------
    // DP1  |   3456    |   -7890   |
    //      ------------------------
    //       Master(1)   Shadow(2)*

    // Step 7:
    // Activate the master database (aka deactivate the shadow database). 
    {
        // Activate the master database (index=1U) (from TestProbe)
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_activateDatabaseForDataPacket(DB_MASTER, dataPacketID1));
    }

    // Database Status (before Step 8)
    // 
    //      ------------------------
    // DP1  |   3456    |   -7890   |
    //      ------------------------
    //       Master(1)*   Shadow(2)

    // Step 8:
    // Read from standard application. This operation will read the value written by the TestProbe in Step 5
    // from master database. 
    {
        int32_t readDataDP1;

        // Read DP1 (from Application).
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID1, &readDataDP1, sizeof(readDataDP1), &bytesRead));
        EXPECT_EQ(sizeof(readDataDP1), bytesRead);
        EXPECT_EQ(masterSecondWrittenData, readDataDP1);
    }

    // Database Status (before Step 9)
    // 
    //      ------------------------
    // DP1  |   3456    |   -7890   |
    //      ------------------------
    //       Master(1)*   Shadow(2)

    // Step 9:
    // Read from TestProbe both from Master Database and from Shadow Database. 
    {
        int32_t readDataDP1a;
        int32_t readDataDP1b;

        // Read DP1 (from TestProbe from Master Database(1)).
        // Note that this is NOT the normal read operation from TestProbe. 
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_MASTER, dataPacketID1, OFFSET_ZERO, &readDataDP1a, sizeof(readDataDP1a), &bytesRead));
        EXPECT_EQ(sizeof(readDataDP1a), bytesRead);
        EXPECT_EQ(masterSecondWrittenData, readDataDP1a);

        // Read DP1 (from TestProbe from Shadow Database(2)).
        // Note that this is the normal read operation from TestProbe. 
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerTestProbe_readDataInDatabase(DB_SHADOW, dataPacketID1, OFFSET_ZERO, &readDataDP1b, sizeof(readDataDP1b), &bytesRead));
        EXPECT_EQ(sizeof(readDataDP1b), bytesRead);
        EXPECT_EQ(shadowSecondWrittenData, readDataDP1b);
    }
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
