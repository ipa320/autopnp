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
 * $Id: measurementTestDataHandler.cpp 7573 2014-02-24 23:07:44Z ruiz $
 */

/**
 * \file
 *         Data Handler Measurement test.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include <vector>

#include "xme/core/component.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/hal/include/mem.h"
#include "xme/core/testUtils.h"
#include "xme/core/broker/include/broker.h"
#include "xme/hal/include/time.h"
#include "xme/core/log.h"
#include "xme/hal/include/random.h"

#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define MAXITERATIONSINMEASUREMENT 10U
#define MAXDATASTORES 512U
#define MAXDATAPACKETSIZE 2048U
#define MULTIPLIERFORITERATIONS 10000U

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class DataHandlerTimeMeasurementInterfaceTest : public ::testing::Test 
{
    protected:
        DataHandlerTimeMeasurementInterfaceTest()
            : attributeKey1(1U)
            , attributeKey2(2U)
        {
            // Broker initialization.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));

            // Data Handler initialization.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setNumberOfDatabases(2U));

            //
            // Publisher: 
            //  -----------------------
            //  |    DP1    | a1 | a2 |
            //  -----------------------
            //
            // Subscriber: 
            //  -----------------------
            //  |    DP2    | a1 | a2 |
            //  -----------------------
            // 
            // Internal Non-Persistent Data Store (not associated to publications/subscriptions): 
            //  -----------------------
            //  |    DP3    | a1 | a2 |
            //  -----------------------
            //
            // Internal Persistent Data Store (not associated to publications/subscriptions): 
            //  -----------------------
            //  |    DP4    | a1 | a2 |
            //  -----------------------
            //
            // Internal Queue Data Store (not associated to publications/subscriptions): 
            //  ----------------------------------------------
            //  |    DP5    | a1 | a2 ||    DP5    | a1 | a2 |
            //  ----------------------------------------------
            //  <------- Q1 ----------><------- Q2 ---------->


            // Data Store creation: Publisher
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &dataPacketID1));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, dataPacketID1));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey2, dataPacketID1));

            // Data Store creation: Subscriber
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &dataPacketID2));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, dataPacketID2));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey2, dataPacketID2));

            // Data Store creation: Internal Non-Persistent
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &dataPacketID3));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, dataPacketID3));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey2, dataPacketID3));

            // Data Store creation: Internal Persistent
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &dataPacketID4));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, dataPacketID4));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey2, dataPacketID4));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketPersistent(dataPacketID4));

            // Data Store creation: Internal Queue
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &dataPacketID5));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, dataPacketID5));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey2, dataPacketID5));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(dataPacketID5, 2U));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

            // Broker: Add transfer entry DP1->DP2
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(dataPacketID1, dataPacketID2));


        }

        virtual
        ~DataHandlerTimeMeasurementInterfaceTest() {
            xme_core_broker_fini();
            xme_core_dataHandler_fini();
        }

    xme_core_dataManager_dataPacketId_t dataPacketID1;
    xme_core_dataManager_dataPacketId_t dataPacketID2;
    xme_core_dataManager_dataPacketId_t dataPacketID3;
    xme_core_dataManager_dataPacketId_t dataPacketID4;
    xme_core_dataManager_dataPacketId_t dataPacketID5;

    uint32_t attributeKey1; 
    uint32_t attributeKey2; 
};

class DataHandlerStressTransferMeasurementInterfaceTest : public ::testing::TestWithParam<uint16_t> 
{
    protected:
        DataHandlerStressTransferMeasurementInterfaceTest()
            : attributeKey1(1U)
            , attributeKey2(2U)
        {
            uint16_t i;
            uint16_t numOfDataPackets = GetParam();

            // Broker initialization.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));

            // Data Handler initialization.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            //
            // Publisher: 
            //  -----------------------
            //  |    DP1    | a1 | a2 |
            //  -----------------------
            //
            // Subscriber: 
            //  -----------------------
            //  |    DP2    | a1 | a2 |
            //  -----------------------
            // 

            for (i = 0U; i < numOfDataPackets; i++)
            {
                // Data Store creation: Publisher
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket((uint32_t) MAXDATAPACKETSIZE, &dataPacketID1));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute((uint32_t) MAXDATAPACKETSIZE, attributeKey1, dataPacketID1));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute((uint32_t) MAXDATAPACKETSIZE, attributeKey2, dataPacketID1));

                // Data Store creation: Subscriber
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket((uint32_t) MAXDATAPACKETSIZE, &dataPacketID2));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute((uint32_t) MAXDATAPACKETSIZE, attributeKey1, dataPacketID2));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute((uint32_t) MAXDATAPACKETSIZE, attributeKey2, dataPacketID2));

                // Broker: Add transfer entry DP1->DP2
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(dataPacketID1, dataPacketID2));
            }

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
        }

        virtual
        ~DataHandlerStressTransferMeasurementInterfaceTest() {
            xme_core_broker_fini();
            xme_core_dataHandler_fini();
        }

    xme_core_dataManager_dataPacketId_t dataPacketID1;
    xme_core_dataManager_dataPacketId_t dataPacketID2;

    uint32_t attributeKey1; 
    uint32_t attributeKey2; 
};

class DataHandlerStressReadWriteMeasurementInterfaceTest : public ::testing::TestWithParam<uint16_t> 
{
    protected:
        DataHandlerStressReadWriteMeasurementInterfaceTest()
            : attributeKey1(1U)
            , attributeKey2(2U)
        {
            uint16_t i;
            uint16_t numOfDataPackets = GetParam();

            // Broker initialization.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));

            // Data Handler initialization.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            // Internal Persistent Data Store (not associated to publications/subscriptions): 
            //  -----------------------
            //  |    DP3    | a1 | a2 |
            //  -----------------------
            //

            for (i = 0U; i < numOfDataPackets; i++)
            {
                // Data Store creation: Internal Persistent
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket((uint32_t) MAXDATAPACKETSIZE, &dataPacketID1));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute((uint32_t) MAXDATAPACKETSIZE, attributeKey1, dataPacketID1));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute((uint32_t) MAXDATAPACKETSIZE, attributeKey2, dataPacketID1));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketPersistent(dataPacketID1));

                if (i == 0U)
                {
                    startPersistentDataStore = dataPacketID1;
                }
                else if (i == (numOfDataPackets - 1))
                {
                    endPersistentDataStore = dataPacketID1;
                }
            }

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
        }

        virtual
        ~DataHandlerStressReadWriteMeasurementInterfaceTest() {
            xme_core_broker_fini();
            xme_core_dataHandler_fini();
        }

    xme_core_dataManager_dataPacketId_t dataPacketID1;

    xme_core_dataManager_dataPacketId_t startPersistentDataStore;
    xme_core_dataManager_dataPacketId_t endPersistentDataStore;


    uint32_t attributeKey1; 
    uint32_t attributeKey2; 
};

/******************************************************************************/
/***   Unit Tests                                                           ***/
/******************************************************************************/

TEST_F(DataHandlerTimeMeasurementInterfaceTest, MeasureTransferWithOnlyData) 
{
    uint32_t writeBufferDP1 = 10U;

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t microSeconds;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1, sizeof(writeBufferDP1))); 

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] Transfer time (only data): %d us.\n", microSeconds);
}

TEST_F(DataHandlerTimeMeasurementInterfaceTest, MeasureTransferWithDataAndAttributes) 
{
    uint32_t writeBufferDP1 = 10U;
    uint32_t writeBufferDP1A1 = 101U;
    uint32_t writeBufferDP1A2 = 102U;

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t microSeconds;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1, sizeof(writeBufferDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey1, &writeBufferDP1A1, sizeof(writeBufferDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] Transfer time (data + attributes): %d us.\n", microSeconds);
}

TEST_F(DataHandlerTimeMeasurementInterfaceTest, MeasureTransferStressTest) 
{
    uint32_t writeBufferDP1 = 10U;
    uint32_t writeBufferDP1A1 = 101U;
    uint32_t writeBufferDP1A2 = 102U;

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t totalMicroseconds = 0U;
    uint32_t i, j;
    uint64_t meanMeasurement;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1, sizeof(writeBufferDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey1, &writeBufferDP1A1, sizeof(writeBufferDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 

    for (i = 1U; i < MULTIPLIERFORITERATIONS; i = i * 10U)
    {
        for (j = 1U; j < (MAXITERATIONSINMEASUREMENT * i); j++)
        {
            uint64_t microSeconds;

            timeHandle1 = xme_hal_time_getCurrentTime();
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));
            timeHandle2 = xme_hal_time_getCurrentTime();
            EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
            EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
            timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
            microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
            totalMicroseconds = totalMicroseconds + microSeconds;
        }
    
        meanMeasurement = totalMicroseconds / (MAXITERATIONSINMEASUREMENT*i);
        XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] Transfer time (stress test with %d iterations) - same data packet: %d us (mean measurement).\n", 
            MAXITERATIONSINMEASUREMENT * i, meanMeasurement);
    }
}

TEST_F(DataHandlerTimeMeasurementInterfaceTest, MeasureWriteAndReadDataForNonPersistentDataStore) 
{
    uint32_t writeBufferDP3 = 10U;
    uint32_t readBufferDP3;

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t microSeconds;
    uint32_t bytesRead;

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID3, &writeBufferDP3, sizeof(writeBufferDP3))); 
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] WriteData time (non-persistent internal data store): %d us.\n", microSeconds);

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID3, &readBufferDP3, sizeof(readBufferDP3), &bytesRead)); 
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] ReadData time (non-persistent internal data store): %d us.\n", microSeconds);
}

TEST_F(DataHandlerTimeMeasurementInterfaceTest, MeasureWriteAndReadDataAndAttributeForNonPersistentDataStore) 
{
    uint32_t writeBufferDP3 = 10U;
    uint32_t writeBufferDP3A1 = 101U;
    uint32_t writeBufferDP3A2 = 102U;
    uint32_t readBufferDP3;

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t microSeconds;
    uint32_t bytesRead;

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID3, &writeBufferDP3, sizeof(writeBufferDP3))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID3, attributeKey1, &writeBufferDP3A1, sizeof(writeBufferDP3A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID3, attributeKey2, &writeBufferDP3A2, sizeof(writeBufferDP3A2))); 
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] WriteData+Attribute time (non-persistent internal data store): %d us.\n", microSeconds);

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID3, &readBufferDP3, sizeof(readBufferDP3), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID3, attributeKey1, &readBufferDP3, sizeof(readBufferDP3), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID3, attributeKey2, &readBufferDP3, sizeof(readBufferDP3), &bytesRead)); 
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] ReadData+Attribute time (non-persistent internal data store): %d us.\n", microSeconds);
}

TEST_F(DataHandlerTimeMeasurementInterfaceTest, MeasureWriteAndReadDataForPersistentDataStore) 
{
    uint32_t writeBufferDP4 = 10U;
    uint32_t readBufferDP4;

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t microSeconds;
    uint32_t bytesRead;

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID4, &writeBufferDP4, sizeof(writeBufferDP4))); 
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] WriteData time (persistent internal data store): %d us.\n", microSeconds);

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID4, &readBufferDP4, sizeof(readBufferDP4), &bytesRead)); 
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] ReadData time (persistent internal data store): %d us.\n", microSeconds);
}

TEST_F(DataHandlerTimeMeasurementInterfaceTest, MeasureWriteAndReadAttributeForPersistentDataStore) 
{
    uint32_t writeBufferDP4 = 10U;
    uint32_t writeBufferDP4A1 = 101U;
    uint32_t writeBufferDP4A2 = 102U;
    uint32_t readBufferDP4;

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t microSeconds;
    uint32_t bytesRead;

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID4, &writeBufferDP4, sizeof(writeBufferDP4))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID4, attributeKey1, &writeBufferDP4A1, sizeof(writeBufferDP4A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID4, attributeKey2, &writeBufferDP4A2, sizeof(writeBufferDP4A2))); 
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] WriteData+Attribute time (persistent internal data store): %d us.\n", microSeconds);

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID4, &readBufferDP4, sizeof(readBufferDP4), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID4, attributeKey1, &readBufferDP4, sizeof(readBufferDP4), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID4, attributeKey2, &readBufferDP4, sizeof(readBufferDP4), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID4));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] ReadData+Attribute time (persistent internal data store): %d us.\n", microSeconds);
}

TEST_F(DataHandlerTimeMeasurementInterfaceTest, MeasureWriteAndReadDataForQueueDataStore) 
{
    uint32_t writeBufferDP5a = 10U;
    uint32_t writeBufferDP5b = 11U;
    uint32_t readBufferDP5;

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t microSeconds;
    uint32_t bytesRead;

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID5, &writeBufferDP5a, sizeof(writeBufferDP5a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID5));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] WriteData time (queue - 1st write): %d us.\n", microSeconds);
    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID5, &writeBufferDP5b, sizeof(writeBufferDP5b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID5));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] WriteData time (queue - 2nd write): %d us.\n", microSeconds);

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID5, &readBufferDP5, sizeof(readBufferDP5), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID5));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] ReadData time (queue - 1st read): %d us.\n", microSeconds);
    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID5, &readBufferDP5, sizeof(readBufferDP5), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID5));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] ReadData time (queue - 2nd read): %d us.\n", microSeconds);
}

TEST_F(DataHandlerTimeMeasurementInterfaceTest, MeasureWriteAndReadAttributeForQueueDataStore) 
{
    uint32_t writeBufferDP5a = 10U;
    uint32_t writeBufferDP5A1a = 101U;
    uint32_t writeBufferDP5A2a = 102U;
    uint32_t writeBufferDP5b = 11U;
    uint32_t writeBufferDP5A1b = 111U;
    uint32_t writeBufferDP5A2b = 112U;
    uint32_t readBufferDP5;

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t microSeconds;
    uint32_t bytesRead;

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID5, &writeBufferDP5a, sizeof(writeBufferDP5a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID5, attributeKey1, &writeBufferDP5A1a, sizeof(writeBufferDP5A1a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID5, attributeKey2, &writeBufferDP5A2a, sizeof(writeBufferDP5A2a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID5));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] WriteData+Attribute time (queue - 1st write): %d us.\n", microSeconds);
    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID5, &writeBufferDP5b, sizeof(writeBufferDP5b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID5, attributeKey1, &writeBufferDP5A1b, sizeof(writeBufferDP5A1b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID5, attributeKey2, &writeBufferDP5A2b, sizeof(writeBufferDP5A2b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID5));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] WriteData+Attribute time (queue - 2nd write): %d us.\n", microSeconds);

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID5, &readBufferDP5, sizeof(readBufferDP5), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID5, attributeKey1, &readBufferDP5, sizeof(readBufferDP5), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID5, attributeKey2, &readBufferDP5, sizeof(readBufferDP5), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID5));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] ReadData+Attribute time (queue - 1st read): %d us.\n", microSeconds);

    timeHandle1 = xme_hal_time_getCurrentTime();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID5));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID5, &readBufferDP5, sizeof(readBufferDP5), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID5, attributeKey1, &readBufferDP5, sizeof(readBufferDP5), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID5, attributeKey2, &readBufferDP5, sizeof(readBufferDP5), &bytesRead)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID5));
    timeHandle2 = xme_hal_time_getCurrentTime();
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
    EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
    timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
    microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
    XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] ReadData+Attribute time (queue - 2nd read): %d us.\n", microSeconds);
}

INSTANTIATE_TEST_CASE_P
(
    DataPacketInstantiation,
    DataHandlerStressTransferMeasurementInterfaceTest,
    ::testing::Values(8U, 32U, 128U, 512U)
);

TEST_P(DataHandlerStressTransferMeasurementInterfaceTest, MeasureTransferStressTestWithRandomDataStores) 
{
    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t totalMicroseconds = 0U;
    uint32_t i, j;
    uint64_t meanMeasurement;
    xme_core_dataManager_dataPacketId_t dataPacketID;
    uint16_t numOfDataPackets = GetParam();

    xme_hal_random_init();
    xme_hal_random_registerThread();

    for (i = 1U; i < MULTIPLIERFORITERATIONS; i = i * 10U)
    {
        for (j = 1U; j < (MAXITERATIONSINMEASUREMENT * i); j++)
        {
            uint64_t microSeconds;
            int writeBufferDP[] = {4};
            int writeBufferDPA1[] = {1};
            int writeBufferDPA2[] = {2};
            uint16_t randomValue = xme_hal_random_randRange(1U, numOfDataPackets);

            if (randomValue % 2 == 0)
            {
                // Only even data stores enable transfer. 
                randomValue = randomValue - 1;
            }
        
            dataPacketID = (xme_core_dataManager_dataPacketId_t) randomValue;

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID));
            randomValue = xme_hal_random_randRange(4U, MAXDATAPACKETSIZE);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID, &writeBufferDP, randomValue)); 
            randomValue = xme_hal_random_randRange(4U, MAXDATAPACKETSIZE);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID, attributeKey1, &writeBufferDPA1, randomValue)); 
            randomValue = xme_hal_random_randRange(4U, MAXDATAPACKETSIZE);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID, attributeKey2, &writeBufferDPA2, randomValue)); 

            timeHandle1 = xme_hal_time_getCurrentTime();
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID));
            timeHandle2 = xme_hal_time_getCurrentTime();
            EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
            EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
            timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
            microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
            totalMicroseconds = totalMicroseconds + microSeconds;
            //meanMeasurement = totalMicroseconds / i;
            //XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] Transfer time: %d; Mean time: %d; Total transfers: %d (DataStoreID=%d).\n", microSeconds, meanMeasurement, i, dataPacketID);
        }

        meanMeasurement = totalMicroseconds / (MAXITERATIONSINMEASUREMENT * i);
        XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] Transfer time (stress test with %d iterations and %d data packets): %d us (mean measurement).\n", 
            MAXITERATIONSINMEASUREMENT * i, numOfDataPackets, meanMeasurement);
    }

    xme_hal_random_deregisterThread();
    xme_hal_random_fini();
}

INSTANTIATE_TEST_CASE_P
(
    DataPacketInstantiation,
    DataHandlerStressReadWriteMeasurementInterfaceTest,
    ::testing::Values(8U, 32U, 128U, 512U)
);

TEST_P(DataHandlerStressReadWriteMeasurementInterfaceTest, MeasureWriteStressTestWithRandomDataStores) 
{
    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t totalMicroseconds = 0U;
    uint32_t i, j;
    uint64_t meanMeasurement;
    xme_core_dataManager_dataPacketId_t dataPacketID;
    uint16_t numOfDataPackets = GetParam();

    for (i = 1U; i < MULTIPLIERFORITERATIONS; i = i * 10U)
    {
        for (j = 1U; j < (MAXITERATIONSINMEASUREMENT * i); j++)
        {
            int writeBufferDP[] = {4};
            int writeBufferDPA1[] = {1};
            int writeBufferDPA2[] = {2};
            uint64_t microSeconds;
            uint16_t randomValue;

            xme_hal_random_init();
            xme_hal_random_registerThread();
            randomValue = xme_hal_random_randRange((uint16_t) startPersistentDataStore, (uint16_t) endPersistentDataStore);
            dataPacketID = (xme_core_dataManager_dataPacketId_t) randomValue;
            xme_hal_random_deregisterThread();
            xme_hal_random_fini();

            xme_hal_random_init();
            xme_hal_random_registerThread();
            timeHandle1 = xme_hal_time_getCurrentTime();
            randomValue = xme_hal_random_randRange(4U, MAXDATAPACKETSIZE);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID, &writeBufferDP, randomValue)); 
            randomValue = xme_hal_random_randRange(4U, MAXDATAPACKETSIZE);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID, attributeKey1, &writeBufferDPA1, randomValue)); 
            randomValue = xme_hal_random_randRange(4U, MAXDATAPACKETSIZE);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID, attributeKey2, &writeBufferDPA2, randomValue)); 
            timeHandle2 = xme_hal_time_getCurrentTime();
            EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
            EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
            timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
            microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
            totalMicroseconds = totalMicroseconds + microSeconds;
            //meanMeasurement = totalMicroseconds / i;
            //XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] Write time: %d; Mean time: %d; Total writes: %d (DataStoreID=%d).\n", microSeconds, meanMeasurement, i, dataPacketID);
            xme_hal_random_deregisterThread();
            xme_hal_random_fini();
        }

        meanMeasurement = totalMicroseconds / (MAXITERATIONSINMEASUREMENT * i);
        XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] Write time (stress test with %d iterations and %d internal data packets): %d us (mean measurement).\n", 
            MAXITERATIONSINMEASUREMENT * i, numOfDataPackets ,meanMeasurement);
    }

}

TEST_P(DataHandlerStressReadWriteMeasurementInterfaceTest, MeasureReadStressTestWithRandomDataStores) 
{
    uint32_t bytesRead;
    int readBufferDP[MAXDATAPACKETSIZE];

    xme_hal_time_timeHandle_t timeHandle1, timeHandle2;
    xme_hal_time_timeInterval_t timeInterval;
    uint64_t totalMicroseconds = 0U;
    uint32_t i, j;
    uint64_t meanMeasurement;
    xme_core_dataManager_dataPacketId_t dataPacketID;
    uint16_t numOfDataPackets = GetParam();

    for (i = 0U; i < numOfDataPackets; i++)
    {
        int writeBufferDP[] = {4};
        int writeBufferDPA1[] = {1};
        int writeBufferDPA2[] = {2};
        dataPacketID = startPersistentDataStore + (xme_core_dataManager_dataPacketId_t) i;

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID, &writeBufferDP, MAXDATAPACKETSIZE)); 
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID, attributeKey1, &writeBufferDPA1, MAXDATAPACKETSIZE)); 
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID, attributeKey2, &writeBufferDPA2, MAXDATAPACKETSIZE)); 
    }

    xme_hal_random_init();
    xme_hal_random_registerThread();

    for (i = 1U; i < MULTIPLIERFORITERATIONS; i = i * 10U)
    {
        for (j = 1U; j < (MAXITERATIONSINMEASUREMENT * i); j++)
        {
            uint64_t microSeconds;
            uint16_t randomValue = xme_hal_random_randRange((uint16_t) startPersistentDataStore, (uint16_t) endPersistentDataStore);

            dataPacketID = (xme_core_dataManager_dataPacketId_t) randomValue;

            timeHandle1 = xme_hal_time_getCurrentTime();
            randomValue = xme_hal_random_randRange(4U, MAXDATAPACKETSIZE);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID, &readBufferDP, randomValue, &bytesRead)); 
            randomValue = xme_hal_random_randRange(4U, MAXDATAPACKETSIZE);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID, attributeKey1, &readBufferDP, randomValue, &bytesRead)); 
            randomValue = xme_hal_random_randRange(4U, MAXDATAPACKETSIZE);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID, attributeKey2, &readBufferDP, randomValue, &bytesRead)); 
            timeHandle2 = xme_hal_time_getCurrentTime();
            EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle1));
            EXPECT_TRUE(xme_hal_time_isValidTimeHandle(timeHandle2));
            timeInterval = xme_hal_time_getTimeIntervalBetween(timeHandle1, timeHandle2);
            microSeconds = xme_hal_time_timeIntervalInMicroseconds(timeInterval);
            totalMicroseconds = totalMicroseconds + microSeconds;
            //meanMeasurement = totalMicroseconds / i;
            //XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] Read time: %d; Mean time: %d; Total reads: %d (DataStoreID=%d).\n", microSeconds, meanMeasurement, i, dataPacketID);
        }

        meanMeasurement = totalMicroseconds / (MAXITERATIONSINMEASUREMENT * i);
        XME_LOG(XME_LOG_NOTE, "[DataHandlerMeasurementTest] Read time (stress test with %d iterations and %d internal data packets): %d us (mean measurement).\n", 
            MAXITERATIONSINMEASUREMENT * i, numOfDataPackets ,meanMeasurement);
    }

    xme_hal_random_deregisterThread();
    xme_hal_random_fini();
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
