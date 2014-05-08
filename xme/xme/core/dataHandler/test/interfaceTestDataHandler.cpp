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
 * $Id: interfaceTestDataHandler.cpp 7844 2014-03-14 14:11:49Z ruiz $
 */

/**
 * \file
 *         Data Handler test.
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

#include <stdint.h>
#include <string.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
typedef enum {
    ATTRIBUTE_KEY_UNDEFINED = XME_CORE_ATTRIBUTE_KEY_USER,
    ATTRIBUTE_KEY_TEST1,
    ATTRIBUTE_KEY_TEST2
} test_attributes_t;

typedef struct attribute_test1 {
        unsigned int value;
} attribute_test1_t;

typedef struct attribute_test2 {
        double value;
} attribute_test2_t;

class Port {
    public:
        Port(xme_core_dataManager_dataPacketId_t anIdentifier, xme_core_component_portType_t aType =
                 XME_CORE_COMPONENT_PORTTYPE_INVALID,
             xme_core_topic_t aTopic = XME_CORE_TOPIC_INVALID_TOPIC, unsigned int aBufferSize = 0u) :
                        portHandle(anIdentifier), type(aType), topic(aTopic), bufferSize(
                                        aBufferSize), bytesRead(0u), buffer(NULL) {
            if (bufferSize > 0u) {
                buffer = new xme_core_topic_t[this->bufferSize];
                xme_hal_mem_set(buffer, 0, this->bufferSize * sizeof(xme_core_topic_t));
            }
        }

        virtual
        ~Port() {
            if (buffer) {
                delete[] (xme_core_topic_t*) buffer;
            }
        }
    public:
        xme_core_dataManager_dataPacketId_t portHandle;
        xme_core_component_portType_t type;
        xme_core_topic_t topic;
        unsigned int bufferSize;
        unsigned int bytesRead;
        xme_core_topic_t* buffer;
};

class Application {
    public:
        Application() :
                        identifier((xme_core_component_t) 5) {
            Port *first = new Port(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                   XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                   (xme_core_topic_t) 1, 10u);

            Port *second = new Port(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                    XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                    (xme_core_topic_t) 2, 100u);

            Port *third = new Port(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                   XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                   (xme_core_topic_t) 1, 10u);

            Port *fourth = new Port(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                    XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                    (xme_core_topic_t) 1, 100u);

            for (unsigned int i = 0u; i < third->bufferSize; ++i) {
                ((xme_core_topic_t*) third->buffer)[i] = (xme_core_topic_t) 0xdeadbeef;
            }

            for (unsigned int i = 0u; i < fourth->bufferSize; ++i) {
                ((xme_core_topic_t*) fourth->buffer)[i] = (xme_core_topic_t) 0xc0ffebabe;
            }

            inport.push_back(first);
            inport.push_back(second);
            outport.push_back(third);
            outport.push_back(fourth);
        }

        virtual
        ~Application() {
            for (unsigned int i = 0u; i < inport.size(); ++i) {
                if (inport[i]) delete inport[i];
            }

            for (unsigned int i = 0u; i < outport.size(); ++i) {
                if (outport[i]) delete outport[i];
            }
        }

        //private:
        xme_core_component_t identifier;
        std::vector <Port*> inport;
        std::vector <Port*> outport;
};

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
#define DATAHANDLER_TEST_MEMORYVALUE 13

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class DataHandlerInterfaceSimpleTest : public ::testing::Test {
    protected:
        DataHandlerInterfaceSimpleTest() :
            componentID(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT), 
            type(XME_CORE_COMPONENT_PORTTYPE_INVALID), 
            topic(XME_CORE_TOPIC_INVALID_TOPIC), 
            bufferSize(0u), 
            queueSize(1u), 
            overwrite(false), 
            persistent(false), 
            historyDepth(0), 
            portHandle(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)
        {
            attributeKey1 = 1U;
            attributeKey2 = 2U;
            dataPacketID1 = (xme_core_dataManager_dataPacketId_t) 1U;
            dataPacketID4 = (xme_core_dataManager_dataPacketId_t) 4U;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());
        }

        virtual
        ~DataHandlerInterfaceSimpleTest() {
            xme_core_dataHandler_fini();
        }

    xme_core_component_t componentID;
    xme_core_component_portType_t type;
    xme_core_topic_t topic;
    int bufferSize;
    int queueSize;
    bool overwrite;
    bool persistent;
    int historyDepth;
    xme_core_dataManager_dataPacketId_t portHandle;
    xme_core_dataManager_dataPacketId_t pnpPortHandle;

    xme_core_dataManager_memoryRegionID_t defaultMemoryRegion;
    xme_core_dataManager_dataStoreID_t dataPacketID1;
    xme_core_dataManager_dataStoreID_t dataPacketID4;
    uint32_t attributeKey1;
    uint32_t attributeKey2;
};

class DataHandlerInterfaceTest : public ::testing::Test {
    protected:
        DataHandlerInterfaceTest() :
            app() 
        {
            // Data Handler initialization
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            // Port creation
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(app.inport[0]->bufferSize * sizeof(xme_core_topic_t), &app.inport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(app.inport[1]->bufferSize * sizeof(xme_core_topic_t), &app.inport[1]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(app.outport[0]->bufferSize * sizeof(xme_core_topic_t), &app.outport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(app.outport[1]->bufferSize * sizeof(xme_core_topic_t), &app.outport[1]->portHandle));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

#if 0
            // Old API. 
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(
                                      app.identifier, app.inport[0]->type, app.inport[0]->topic,
                                      app.inport[0]->bufferSize * sizeof(xme_core_topic_t),
                                      XME_CORE_NO_ATTRIBUTE, 1, false, false, 0,
                                      &app.inport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(
                                      app.identifier, app.inport[1]->type, app.inport[1]->topic,
                                      app.inport[1]->bufferSize * sizeof(xme_core_topic_t),
                                      XME_CORE_NO_ATTRIBUTE, 1, false, false, 0,
                                      &app.inport[1]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(
                                      app.identifier, app.outport[0]->type, app.outport[0]->topic,
                                      app.outport[0]->bufferSize * sizeof(xme_core_topic_t),
                                      XME_CORE_NO_ATTRIBUTE, 1, false, false, 0,
                                      &app.outport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(
                                      app.identifier, app.outport[1]->type, app.outport[1]->topic,
                                      app.outport[1]->bufferSize * sizeof(xme_core_topic_t),
                                      XME_CORE_NO_ATTRIBUTE, 1, false, false, 0,
                                      &app.outport[1]->portHandle));
#endif
        }

        virtual
        ~DataHandlerInterfaceTest() {
            xme_core_dataHandler_fini();
        }

        Application app;
};

typedef DataHandlerInterfaceTest DataHandlerInterfaceTestDeathTest;

class DataHandlerNewInterfaceTest : public ::testing::Test 
{
    protected:
        DataHandlerNewInterfaceTest()
            : attributeKey1(1U)
            , attributeKey2(2U)
            , attributeKey3(3U)
            , attributeKey4(4U)
        {
            // Broker initialization.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));

            // Data Handler initialization.
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());

            //
            // Publisher: 
            //  ----------------------------
            //  |    DP1    | a1 | a2 | a3 | (does not contain attribute 4)
            //  ----------------------------
            //
            // Subscriber: 
            //  ----------------------------
            //  |    DP2    | a2 | a3 | a4 | (does not contain attribute 1)
            //  ----------------------------


            // Data Store creation: Publication
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &dataPacketID1));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, dataPacketID1));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey2, dataPacketID1));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey3, dataPacketID1));

            // Data Store creation: Subscription
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &dataPacketID2));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey2, dataPacketID2));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey3, dataPacketID2));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey4, dataPacketID2));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

            // Broker: Add transfer entry DP1->DP2
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_addDataPacketTransferEntry(dataPacketID1, dataPacketID2));


        }

        virtual
        ~DataHandlerNewInterfaceTest() {
            xme_core_dataHandler_fini();
        }

    xme_core_dataManager_dataPacketId_t dataPacketID1;
    xme_core_dataManager_dataPacketId_t dataPacketID2;

    uint32_t attributeKey1; 
    uint32_t attributeKey2; 
    uint32_t attributeKey3; 
    uint32_t attributeKey4; 
};

// xme_core_dataHandler_createMemoryRegion()


TEST_F(DataHandlerInterfaceSimpleTest, createMemoryRegionWithNullParameter)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createMemoryRegion(NULL));
}

// xme_core_dataHandler_setNumberOfCopiesInMemoryRegion()

TEST_F(DataHandlerInterfaceSimpleTest, SetNumberOfCopiesWithInvalidMemoryRegionParameter)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setNumberOfCopiesInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID, 1U));
}

TEST_F(DataHandlerInterfaceSimpleTest, SetNumberOfCopiesToZero)
{
    xme_core_dataManager_memoryRegionID_t memoryRegionID;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createMemoryRegion(&memoryRegionID));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_setNumberOfCopiesInMemoryRegion(memoryRegionID, 0U));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_setNumberOfCopiesInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT, 0U));
}

// xme_core_dataHandler_setDatabaseSizeInMemoryRegion()

TEST_F(DataHandlerInterfaceSimpleTest, SetMemorySizeWithInvalidMemoryRegionParameter)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDatabaseSizeInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID, 256U));
}

TEST_F(DataHandlerInterfaceSimpleTest, SetMemorySizeWithInvalidParameters)
{
    xme_core_dataManager_memoryRegionID_t memoryRegionID = (xme_core_dataManager_memoryRegionID_t) 5U;
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDatabaseSizeInMemoryRegion(memoryRegionID, 256U));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createMemoryRegion(&memoryRegionID));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_setDatabaseSizeInMemoryRegion(memoryRegionID, 0U));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_setDatabaseSizeInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT, 0U));
}


// xme_core_dataHandler_createDataPacket()

TEST_F(DataHandlerInterfaceSimpleTest, createDataPacketWithNullDataPacketIDPointer)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createDataPacket(4U, NULL));
}

TEST_F(DataHandlerInterfaceSimpleTest, createDataPacketWithZeroBufferSize)
{
    // TODO: Not sure whether a buffer size of 0U is valid or not;
    //       thinking about GK requesting packets without data (i.e., events),
    //       we might want to accept a buffer size of zero.
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createDataPacket(0U, &portHandle));
}

// xme_core_dataHandler_setDataPacketQueueSize()

TEST_F(DataHandlerInterfaceSimpleTest, setDataPacketQueueSizeWithInvalidDataPacketID)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketQueueSize(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 42U));
}

TEST_F(DataHandlerInterfaceSimpleTest, setDataPacketQueueSizeWithQueueSizeZero)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_setDataPacketQueueSize(1U, 0U));
}

TEST_F(DataHandlerInterfaceSimpleTest, setDataPacketQueueSizeWithArbitraryDataPacketID)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketQueueSize(42U, 42U));
}

// xme_core_dataHandler_setDataPacketPersistent()

TEST_F(DataHandlerInterfaceSimpleTest, setDataPacketPersistentWithInvalidDataPacketID)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketPersistent(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DataHandlerInterfaceSimpleTest, setDataPacketPersistentWithArbitraryDataPacketID)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketPersistent(42U));
}


// xme_core_dataHandler_setDataPacketOverwrite()

TEST_F(DataHandlerInterfaceSimpleTest, setDataPacketOverwriteWithInvalidDataPacketID)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketOverwrite(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, true, XME_STATUS_SUCCESS, true));
}

TEST_F(DataHandlerInterfaceSimpleTest, setDataPacketOverwriteWithArbitraryDataPacketID)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketOverwrite(42U, true, XME_STATUS_SUCCESS, true));
}

// xme_core_dataHandler_createAttribute()

TEST_F(DataHandlerInterfaceSimpleTest, createAttributeWithInvalidDataPacketID)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_createAttribute(4U, 1U, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DataHandlerInterfaceSimpleTest, createAttributeWithArbitraryDataPacketID)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_createAttribute(4U, 1U, 42U));
}

TEST_F(DataHandlerInterfaceSimpleTest, setDatabaseSizeWithSizeZero)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_setDatabaseSize(0U));
}

TEST_F(DataHandlerInterfaceSimpleTest, writeDataWithInvalidDataPacketID)
{
    uint32_t buffer = 0U;

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, &buffer, sizeof(buffer)));
}

// FIXME: This Unit Test should disappear when dealing with different memory regions!. 
TEST_F(DataHandlerInterfaceSimpleTest, writeDataWithArbitraryDataPacketIDAndNoConfiguredDatabase)
{
#ifdef DEBUG
    uint32_t buffer = 0U;
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_writeData(42U, &buffer, sizeof(buffer)));
#endif
}

TEST_F(DataHandlerInterfaceSimpleTest, writeDataWithArbitraryDataPacketIDAndConfiguredDatabase)
{
    uint32_t buffer = 0U;
    
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeData(42U, &buffer, sizeof(buffer)));
}

TEST_F(DataHandlerInterfaceSimpleTest, writeDataWithZeroBufferSize)
{
    uint32_t buffer = 0U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeData(portHandle, &buffer, 0U));
}

TEST_F(DataHandlerInterfaceSimpleTest, writeAttributeWithInvalidDataPacketID)
{
    uint32_t buffer = 0U;

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey1, &buffer, sizeof(buffer)));
}

TEST_F(DataHandlerInterfaceSimpleTest, writeAttributeWithArbitraryDataPacketID)
{
#ifdef DEBUG
    uint32_t buffer = 0U;

    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_writeAttribute(42U, attributeKey1, &buffer, sizeof(buffer)));
#endif
}

TEST_F(DataHandlerInterfaceSimpleTest, writeAttributeWithZeroBufferSizeAndNoAttributeCreation)
{
    uint32_t buffer = 0U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeAttribute(portHandle, attributeKey1, &buffer, 0U));
}

TEST_F(DataHandlerInterfaceSimpleTest, writeAttributeWithZeroBufferSizeAndWitAttributeCreation)
{
    uint32_t buffer = 0U;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeAttribute(portHandle, attributeKey1, &buffer, 0U));
}

TEST_F(DataHandlerInterfaceSimpleTest, readDataWithInvalidDataPacketID)
{
    uint32_t buffer = 0U;
    uint32_t readBytes;

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, &buffer, sizeof(buffer), &readBytes));
}

TEST_F(DataHandlerInterfaceSimpleTest, readDataWithArbitraryDataPacketID)
{
#ifdef DEBUG
    uint32_t buffer = 0U;
    uint32_t readBytes;

    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_readData(42U, &buffer, sizeof(buffer), &readBytes));
#endif
}

TEST_F(DataHandlerInterfaceSimpleTest, readDataWithReadBytesParameterSetToNull)
{
    uint32_t buffer = 0U;

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readData(42U, &buffer, sizeof(buffer), NULL));
}

TEST_F(DataHandlerInterfaceSimpleTest, readDataWithZeroBufferSize)
{
    uint32_t buffer = 0U;
    uint32_t readBytes;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readData(portHandle, &buffer, 0U, &readBytes));
}

TEST_F(DataHandlerInterfaceSimpleTest, readAttributeWithInvalidDataPacketID)
{
    uint32_t buffer = 0U;
    uint32_t readBytes;

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey1, &buffer, sizeof(buffer), &readBytes));
}

TEST_F(DataHandlerInterfaceSimpleTest, readAttributeWithReadBytesParameterSetToNull)
{
    uint32_t buffer = 0U;

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey1, &buffer, sizeof(buffer), NULL));
}

TEST_F(DataHandlerInterfaceSimpleTest, readAttributeWithArbitraryDataPacketID)
{
#ifdef DEBUG
    uint32_t buffer = 0U;
    uint32_t readBytes;

    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_readAttribute(42U, attributeKey1, &buffer, sizeof(buffer), &readBytes));
#endif
}

TEST_F(DataHandlerInterfaceSimpleTest, readAttributeWithZeroBufferSizeAndNoAttributeCreation)
{
    uint32_t buffer = 0U;
    uint32_t readBytes;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(portHandle, attributeKey1, &buffer, 0U, &readBytes));
}

TEST_F(DataHandlerInterfaceSimpleTest, readAttributeWithZeroBufferSizeAndWitAttributeCreation)
{
    uint32_t buffer = 0U;
    uint32_t readBytes;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(portHandle, attributeKey1, &buffer, 0U, &readBytes));
}

TEST_F(DataHandlerInterfaceSimpleTest, readDataAndAttributeFromANonWrittenDataStore)
{
    uint32_t buffer = 0U;
    uint32_t readBytes;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readData(portHandle, &buffer, sizeof(buffer), &readBytes));
    ASSERT_EQ(readBytes, 0U);
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readAttribute(portHandle, attributeKey1, &buffer, sizeof(buffer), &readBytes));
    ASSERT_EQ(readBytes, 0U);
}


#if 0
// Old API: No more parameters like component, port type, and topic. 
// Buffer size is already included in ::createPortWithInvalidValues
TEST_F(DataHandlerInterfaceSimpleTest, createPortWithInvalidComponentID) {
    type = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
    topic = XME_CORE_TOPIC_ROUTES_LOCAL_ANNOUNCEMENT;
    bufferSize = 4;
    ASSERT_EQ(XME_STATUS_SUCCESS,   // this is valid due to merge of different interfaces, FIXME: please check!
              xme_core_dataHandler_createPort(componentID, type, topic, bufferSize,
                                              XME_CORE_NO_ATTRIBUTE,
                                              queueSize, overwrite,
                                              persistent, historyDepth, &portHandle));
}

TEST_F(DataHandlerInterfaceSimpleTest, createPortWithInvalidPortType) {
    componentID = (xme_core_component_t) 15u;
    topic = XME_CORE_TOPIC_ROUTES_LOCAL_ANNOUNCEMENT;
    bufferSize = 4;
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_createPort(componentID, type, topic, bufferSize,
                                              XME_CORE_NO_ATTRIBUTE, queueSize, overwrite,
                                              persistent, historyDepth, &portHandle));
}

TEST_F(DataHandlerInterfaceSimpleTest, createPortWithInvalidTopic) {
    componentID = (xme_core_component_t) 15u;
    type = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
    bufferSize = 4;
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_createPort(componentID, type, topic, bufferSize,
                                              XME_CORE_NO_ATTRIBUTE, queueSize, overwrite,
                                              persistent, historyDepth, &portHandle));
}

TEST_F(DataHandlerInterfaceSimpleTest, createPortWithInvalidBufferSize) {
    componentID = (xme_core_component_t) 15u;
    type = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
    topic = XME_CORE_TOPIC_ROUTES_LOCAL_ANNOUNCEMENT;
    bufferSize = 0u;
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_createPort(componentID, type, topic, bufferSize,
                                              XME_CORE_NO_ATTRIBUTE, queueSize, overwrite,
                                              persistent, historyDepth, &portHandle));
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataOfZeroElementsToInputPortWhichIsNotAllowed) {
#if 0
#ifdef DEBUG
    unsigned int elements = 0u;
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                                   sizeof(app.outport[0]->buffer[0]) * elements));
#endif
#endif

#ifdef DEBUG
    unsigned int elements = 0u;
    ASSERT_DEATH(
                    xme_core_dataHandler_writeData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                                   sizeof(app.outport[0]->buffer[0]) * elements),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataOfZeroElementsToOutputPort) {
#if 0
#ifdef DEBUG
    unsigned int elements = 0u;
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                                   app.outport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements));
#endif
#endif

#ifdef DEBUG
    unsigned int elements = 0u;
    ASSERT_DEATH(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                                   app.outport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, writeDataOfOneElement) {
    unsigned int elements = 1u;

    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle, app.outport[0]->buffer,
                                             sizeof(app.outport[0]->buffer[0]) * elements));
}

TEST_F(DataHandlerInterfaceTest, writeDataOfTenElements) {
    // write 10 elements to dataHandler, transfer it, and read as much possible
    unsigned int elements = 10u;
    
    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle, app.outport[0]->buffer,
                                             sizeof(app.outport[0]->buffer[0]) * elements));
}

TEST_F(DataHandlerInterfaceTest, writeDataOfBufferSizeElementsThanBufferCanKeep) {
    unsigned int elements = app.outport[0]->bufferSize;
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle, app.outport[0]->buffer,
                                             sizeof(app.outport[0]->buffer[0]) * elements));
}

TEST_F(DataHandlerInterfaceTest, writeDataOfMoreElementsThanBufferCanKeep) {
    unsigned int elements = app.outport[0]->bufferSize + 1;

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle, app.outport[0]->buffer,
                                             sizeof(app.outport[0]->buffer[0]) * elements));
}

TEST_F(DataHandlerInterfaceTest, transferDataWithBothInvalidPortHandles) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_transferData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DataHandlerInterfaceTest, transferDataWithSinkInvalidPortHandle) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_transferData(app.outport[0]->portHandle,
                                                XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DataHandlerInterfaceTest, transferDataWithSourceInvalidPortHandle) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_transferData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                app.inport[0]->portHandle));
}

TEST_F(DataHandlerInterfaceTest, transferDataBothPortsAreTheSame) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_transferData(app.outport[0]->portHandle,
                                                app.outport[0]->portHandle));
}

TEST_F(DataHandlerInterfaceTest, transferDataPortSourceAndSinkAreInverted) {
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED,
              xme_core_dataHandler_transferData(app.inport[0]->portHandle,
                                                app.outport[0]->portHandle));
}

TEST_F(DataHandlerInterfaceTest, transferDataPortFromSourceToSink) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[0]->portHandle,
                                                app.inport[0]->portHandle));
}

TEST_F(DataHandlerInterfaceTestDeathTest, readDataWithInvalidValues) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL,
                                                  0u, NULL));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL,
                                                  0u, NULL),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, readDataFromANonReadablePort) {
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED,
              xme_core_dataHandler_readData(app.outport[0]->portHandle, app.outport[0]->buffer,
                                            app.outport[0]->bufferSize,
                                            &app.outport[0]->bytesRead));
}

TEST_F(DataHandlerInterfaceTestDeathTest, readDataFromAPortButWriteInANULLBuffer) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readData(app.inport[0]->portHandle, NULL,
                                                  app.outport[0]->bufferSize,
                                                  &app.outport[0]->bytesRead));
#endif
#endif

#ifdef DEBUG 
    ASSERT_DEATH(
                    xme_core_dataHandler_readData(app.inport[0]->portHandle, NULL,
                                                  app.outport[0]->bufferSize,
                                                  &app.outport[0]->bytesRead),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, readDataFromAPortButBufferSizeIsZero) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle, app.inport[0]->buffer, 0u,
                                            &app.inport[0]->bytesRead));
}

TEST_F(DataHandlerInterfaceTestDeathTest, readDataFromAPortButWrittenBytesWillBeStoredInNull) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                                  app.inport[0]->bufferSize, NULL));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                                  app.inport[0]->bufferSize, NULL),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, readDataFromAPortAndStoreDataInABuffer) 
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                            app.inport[0]->bufferSize, &app.inport[0]->bytesRead));
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeWithInvalidAttributes) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                       XME_CORE_ATTRIBUTE_KEY_UNDEFINED, NULL, 0u,
                                                       NULL));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                       XME_CORE_ATTRIBUTE_KEY_UNDEFINED, NULL, 0u,
                                                       NULL),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeFromAnInvalidPort) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readAttribute ( XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer,
                                                         app.inport[0]->bufferSize,
                                                         &app.inport[0]->bytesRead ));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readAttribute ( XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer,
                                                         app.inport[0]->bufferSize,
                                                         &app.inport[0]->bytesRead ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeWithInvalidKey) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readAttribute(app.inport[0]->portHandle,
                                                       XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
                                                       app.inport[0]->buffer,
                                                       app.inport[0]->bufferSize,
                                                       &app.inport[0]->bytesRead));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readAttribute(app.inport[0]->portHandle,
                                                       XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
                                                       app.inport[0]->buffer,
                                                       app.inport[0]->bufferSize,
                                                       &app.inport[0]->bytesRead),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeIntoANULLBuffer) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         NULL,
                                                         app.inport[0]->bufferSize,
                                                         &app.inport[0]->bytesRead ));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         NULL,
                                                         app.inport[0]->bufferSize,
                                                         &app.inport[0]->bytesRead ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeWithZeroBufferSize) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer, 0u,
                                                         &app.inport[0]->bytesRead ));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer, 0u,
                                                         &app.inport[0]->bytesRead ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeWillSaveReadSizeInNULL) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer,
                                                         app.inport[0]->bufferSize, NULL ));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer,
                                                         app.inport[0]->bufferSize, NULL ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, readAttributeFromPublisherPort) {
    
    // FIXME: In new API, we do not differentiate between publisher and subscriber port. Data Handler is currently port-type agnostic. 
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
              xme_core_dataHandler_readAttribute ( app.outport[0]->portHandle,
                                                   XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                   app.outport[0]->buffer,
                                                   app.outport[0]->bufferSize,
                                                   &app.outport[0]->bytesRead ));

    
#if 0
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED,
              xme_core_dataHandler_readAttribute ( app.outport[0]->portHandle,
                                                   XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                   app.outport[0]->buffer,
                                                   app.outport[0]->bufferSize,
                                                   &app.outport[0]->bytesRead ));
#endif
}

TEST_F(DataHandlerInterfaceTest, readAttributeFromValidPortButNoAttributesAreStored) 
{
    // FIXME: New Data Handler do not look for the content of the data, simply check
    //        if the attribute is defined, and if not, NOT_FOUND is returned. 
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
              xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                   XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                   app.inport[0]->buffer,
                                                   app.inport[0]->bufferSize,
                                                   &app.inport[0]->bytesRead ));

#if 0
    // Old API.
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE,
              xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                   XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                   app.inport[0]->buffer,
                                                   app.inport[0]->bufferSize,
                                                   &app.inport[0]->bytesRead ));
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataWithInvalidValues) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL,
                                                   0u));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL,
                                                   0u),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataIntoInvalidPort) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                   app.outport[0]->buffer,
                                                   app.outport[0]->bufferSize));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                   app.outport[0]->buffer,
                                                   app.outport[0]->bufferSize),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataFromAnInvalidBuffer) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle, NULL,
                                                   app.outport[0]->bufferSize));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle, NULL,
                                                   app.outport[0]->bufferSize),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataOfZeroSize) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                                   app.outport[0]->buffer, 0u));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                                   app.outport[0]->buffer, 0u),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, writeDataIntoASubscriberPort) 
{
    // FIXME: In new API, we do not differentiate between publisher and subscriber port. Data Handler is currently port-type agnostic. 
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                             app.inport[0]->bufferSize));

#if 0
// Old API
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED,
              xme_core_dataHandler_writeData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                             app.inport[0]->bufferSize));
#endif
}

TEST_F(DataHandlerInterfaceTest, writeDataToAValidPort) 
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle, app.outport[0]->buffer,
                                             app.outport[0]->bufferSize));
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeWithInvalidValues) 
{
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                        XME_CORE_ATTRIBUTE_KEY_UNDEFINED, NULL, 0u));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                        XME_CORE_ATTRIBUTE_KEY_UNDEFINED, NULL, 0u),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeWithInvalidPort) 
{
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeAttribute ( XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                          XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                          app.outport[0]->buffer,
                                                          app.outport[0]->bufferSize ));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeAttribute ( XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                          XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                          app.outport[0]->buffer,
                                                          app.outport[0]->bufferSize ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeWithInvalidKey) 
{
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                        xme_core_dataHandler_writeAttribute(app.outport[0]->portHandle,
                                                            XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
                                                            app.outport[0]->buffer,
                                                            app.outport[0]->bufferSize));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeAttribute(app.outport[0]->portHandle,
                                                        XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
                                                        app.outport[0]->buffer,
                                                        app.outport[0]->bufferSize),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeIntoNULLBuffer) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandler_writeAttribute ( app.outport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), NULL, app.outport[0]->bufferSize ));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeAttribute ( app.outport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), NULL, app.outport[0]->bufferSize ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeWithZeroBufferSize) {
#if 0
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(
                        xme_core_dataHandler_writeAttribute ( app.outport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), app.outport[0]->buffer, 0u ));
#endif
#endif

#ifdef DEBUG
    ASSERT_DEATH(
                    xme_core_dataHandler_writeAttribute ( app.outport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), app.outport[0]->buffer, 0u ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, writeAttributeIntoASubscriberPort) {

    // FIXME: The new data handler simply do not check the permission, but the existence of the attribute. 
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
              xme_core_dataHandler_writeAttribute ( app.inport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), app.inport[0]->buffer, app.inport[0]->bufferSize ));

#if 0
// Old API
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED,
              xme_core_dataHandler_writeAttribute ( app.inport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), app.inport[0]->buffer, app.inport[0]->bufferSize ));
#endif
}

/* FIXME: Should this test stay here, then we have to change it thus attributes are supported in our test case
 * otherwise we have to move this to an integration test
 TEST_F(DataHandlerInterfaceTest, writeAttributeIntoAValidPort) {
 ASSERT_EQ(XME_STATUS_SUCCESS,
 xme_core_dataHandler_writeAttribute ( app.outport[0]->portHandle,
 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
 app.outport[0]->buffer,
 app.outport[0]->bufferSize ));
 }
 */

TEST_F(DataHandlerInterfaceTest, completeReadOperationWithInvalidValues) 
{
    // FIXME: In the new data handler, this is not a INVALID_PARAMETER, but a INVALID_CONFIGURATION.
    //        Should we check 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_startReadOperation(
                              XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_completeReadOperation(
                              XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DataHandlerInterfaceTest, completeReadOperationPortWithInvalidComponent) 
{
#if 0
// Old API
    ASSERT_EQ(XME_STATUS_NOT_FOUND,   // This is allowed, because internal components are marked as invalid, FIXMEs
              xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));
#endif
}

TEST_F(DataHandlerInterfaceTest, prepareInputPortWithSubscriberPort) 
{
    // We test both write and read from the output port. 
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_startReadOperation(app.outport[0]->portHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_completeReadOperation(app.outport[0]->portHandle));

    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_startWriteOperation(app.outport[0]->portHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

#if 0
// Old API. Additionally, this was not a subscriber port. 
    //FIXME: HERE WE HAVE TO MOCK THE EXECUTION MANAGER
    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));
#endif 
}

TEST_F(DataHandlerInterfaceTest, prepareInputPortWithPublisherPort) 
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_startReadOperation(app.inport[0]->portHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_startWriteOperation(app.inport[0]->portHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_completeWriteOperation(app.inport[0]->portHandle));

#if 0
// Old API.
    //FIXME: HERE WE HAVE TO MOCK THE EXECUTION MANAGER
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));
#endif
}

TEST_F(DataHandlerInterfaceTest, completeWriteOperationPortWithInvalidPort) {
    
    // FIXME: In the new data handler, this is not a INVALID_PARAMETER, but a INVALID_CONFIGURATION.
    //        Should we check 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_startWriteOperation(
                              XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_completeWriteOperation(
                              XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DataHandlerInterfaceTest, completeWriteOperationPortWithSubscriberPort) 
{
    // Here the right behaviour is to allow data write even if the port is not
    // defined in the broker. 
    ASSERT_EQ(XME_STATUS_SUCCESS,   // This is given by broker, because port is not found
              xme_core_dataHandler_completeWriteOperation(app.inport[0]->portHandle));
#if 0
    // Old test.
    ASSERT_EQ(XME_STATUS_NOT_FOUND,   // This is given by broker, because port is not found
              xme_core_dataHandler_completeWriteOperation(app.inport[0]->portHandle));
#endif
}

TEST_F(DataHandlerInterfaceTest, completeWriteOperationPortWithPublisherPort) 
{
    // Here the right behaviour is to allow data write even if the port is not
    // defined in the broker. 
    ASSERT_EQ(XME_STATUS_SUCCESS,   // This is given by broker, because port is not found
              xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

#if 0
    // Old test. 
    ASSERT_EQ(XME_STATUS_NOT_FOUND,   // This is given by broker, because port is not found
              xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));
#endif
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST_F(DataHandlerNewInterfaceTest, Issue4075TransferOfAttributes) 
{
    uint32_t writeBufferDP1 = 10U;
    uint32_t writeBufferDP1A1 = 101U;
    uint32_t writeBufferDP1A2 = 102U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A1;
    uint32_t readBufferDP2A2;
    uint32_t readBufferDP2A3;
    uint32_t readBufferDP2A4;

    uint32_t bytesRead;

    // Write DP1 (Publisher)
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1, sizeof(writeBufferDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey1, &writeBufferDP1A1, sizeof(writeBufferDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 
    // Do not write attribute 3: Use Case 3. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test starts. 
    // Read DP1.
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead));
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1, readBufferDP2);

    // Use Case 1: The attribute 1 (defined in DP1) is not transfered because it is not defined in DP2.
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey1, &readBufferDP2A1, sizeof(readBufferDP2A1), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);

    // Use Case 2: The attribute 2 (defined in DP1 and DP2) is transferred and the obtained value is the same.
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A1), bytesRead);
    EXPECT_EQ(writeBufferDP1A2, readBufferDP2A2);

    // Use Case 3: The attribute 3 (defined in DP1 and DP2) is not written in DP1, so that is not transferred to DP2.
    bytesRead = 42U;
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey3, &readBufferDP2A3, sizeof(readBufferDP2A3), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);

    // Use Case 4: The attribute 4 (defined only in DP2) takes no value.
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey4, &readBufferDP2A4, sizeof(readBufferDP2A4), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4075TransferOfAttributesWithPreviousSuccessTransfer) 
{
    uint32_t writeBufferDP2 = 20U;
    uint32_t writeBufferDP2A2 = 202U;
    uint32_t writeBufferDP2A3 = 203U;
    uint32_t writeBufferDP2A4 = 204U;

    uint32_t writeBufferDP1 = 10U;
    uint32_t writeBufferDP1A1 = 101U;
    uint32_t writeBufferDP1A2 = 102U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A1;
    uint32_t readBufferDP2A2;
    uint32_t readBufferDP2A3;
    uint32_t readBufferDP2A4;

    uint32_t bytesRead;

    // Write in target read port (DP2)
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID2, &writeBufferDP2, sizeof(writeBufferDP2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID2, attributeKey2, &writeBufferDP2A2, sizeof(writeBufferDP2A2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID2, attributeKey3, &writeBufferDP2A3, sizeof(writeBufferDP2A3))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID2, attributeKey4, &writeBufferDP2A4, sizeof(writeBufferDP2A4))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID2));

    // Start writing the second write. 
    // Write DP1 (Publisher)
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1, sizeof(writeBufferDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey1, &writeBufferDP1A1, sizeof(writeBufferDP1A1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 
    // Do not write attribute 3: Use Case 3. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test starts.
    // Read DP1.
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1, readBufferDP2);

    // Use Case 1: The attribute 1 (defined in DP1) is not transfered because it is not defined in DP2.
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey1, &readBufferDP2A1, sizeof(readBufferDP2A1), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);

    // Use Case 2: The attribute 2 (defined in DP1 and DP2) is transferred and the obtained value is the same.
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A1), bytesRead);
    EXPECT_EQ(writeBufferDP1A2, readBufferDP2A2);

    // Use Case 3: The attribute 3 (defined in DP1 and DP2) is not written in DP1, so that is not transferred to DP2.
    bytesRead = 42U;
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey3, &readBufferDP2A3, sizeof(readBufferDP2A3), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);

    // Use Case 4: The attribute 4 (defined only in DP2) takes no value.
    bytesRead = 42U;
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey4, &readBufferDP2A4, sizeof(readBufferDP2A4), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079WriteOnlyDataAndTransfer) 
{
    uint32_t writeBufferDP1 = 10U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;

    uint32_t bytesRead;

    // Test #1: Publisher:: SWO / WriteData / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1, sizeof(writeBufferDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #1: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1, readBufferDP2);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079WriteOnlyAttributeAndTransfer) 
{
    uint32_t writeBufferDP1A2 = 101U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;

    uint32_t bytesRead;

    // Test #2: Publisher:: SWO / WriteAttribute / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #2: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A2), bytesRead);
    EXPECT_EQ(writeBufferDP1A2, readBufferDP2A2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079WriteDataAndOneAttributeAndTransfer) 
{
    uint32_t writeBufferDP1 = 10U;
    uint32_t writeBufferDP1A2 = 102U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;

    uint32_t bytesRead;

    // Test #3: Publisher:: SWO / WriteData / WriteAttribute / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1, sizeof(writeBufferDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #3: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1, readBufferDP2);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A2), bytesRead);
    EXPECT_EQ(writeBufferDP1A2, readBufferDP2A2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079WriteDataAndTwoAttributesAndTransfer) 
{
    uint32_t writeBufferDP1 = 10U;
    uint32_t writeBufferDP1A2 = 102U;
    uint32_t writeBufferDP1A3 = 103U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;
    uint32_t readBufferDP2A3;

    uint32_t bytesRead;

    // Test #4: Publisher:: SWO / WriteData / WriteAttribute / WriteAttribute / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1, sizeof(writeBufferDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey3, &writeBufferDP1A3, sizeof(writeBufferDP1A3))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #4: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1, readBufferDP2);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A2), bytesRead);
    EXPECT_EQ(writeBufferDP1A2, readBufferDP2A2);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey3, &readBufferDP2A3, sizeof(readBufferDP2A3), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A3), bytesRead);
    EXPECT_EQ(writeBufferDP1A3, readBufferDP2A3);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079WriteDataTwiceAndTransferData) 
{
    uint32_t writeBufferDP1a = 10U;
    uint32_t writeBufferDP1b = 11U;

    uint32_t readBufferDP2;

    uint32_t bytesRead;

    // Test #5: Publisher:: SWO / WriteData / WriteData / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1a, sizeof(writeBufferDP1a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1b, sizeof(writeBufferDP1b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #5: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1b, readBufferDP2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079WriteDataTwiceAndAfterOneAttributeAndTransferData) 
{
    uint32_t writeBufferDP1a = 10U;
    uint32_t writeBufferDP1b = 11U;
    uint32_t writeBufferDP1A2 = 102U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;

    uint32_t bytesRead;

    // Test #6: Publisher:: SWO / WriteData / WriteData / WriteAttribute / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1a, sizeof(writeBufferDP1a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1b, sizeof(writeBufferDP1b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #6: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1b, readBufferDP2);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A2), bytesRead);
    EXPECT_EQ(writeBufferDP1A2, readBufferDP2A2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079WriteDataTwiceAndBetweenOneAttributeAndTransferData) 
{
    uint32_t writeBufferDP1a = 10U;
    uint32_t writeBufferDP1b = 11U;
    uint32_t writeBufferDP1A2 = 102U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;

    uint32_t bytesRead;

    // Test #7: Publisher:: SWO / WriteData / WriteAttribute / WriteData / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1a, sizeof(writeBufferDP1a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1b, sizeof(writeBufferDP1b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #7: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1b, readBufferDP2);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A2), bytesRead);
    EXPECT_EQ(writeBufferDP1A2, readBufferDP2A2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079WriteDataTwiceAndFirstOneAttributeAndTransferData) 
{
    uint32_t writeBufferDP1a = 10U;
    uint32_t writeBufferDP1b = 11U;
    uint32_t writeBufferDP1A2 = 102U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;

    uint32_t bytesRead;

    // Test #8: Publisher:: SWO / WriteAttribute / WriteData / WriteData / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2, sizeof(writeBufferDP1A2))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1a, sizeof(writeBufferDP1a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1b, sizeof(writeBufferDP1b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #8: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1b, readBufferDP2);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A2), bytesRead);
    EXPECT_EQ(writeBufferDP1A2, readBufferDP2A2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079WriteDataAndOverwriteAttributeAndTransfer) 
{
    uint32_t writeBufferDP1 = 10U;
    uint32_t writeBufferDP1A2a = 101U;
    uint32_t writeBufferDP1A2b = 102U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;

    uint32_t bytesRead;

    // Test #9: Publisher:: SWO / WriteData / WriteAttribute / WriteAttribute / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1, sizeof(writeBufferDP1))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2a, sizeof(writeBufferDP1A2a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2b, sizeof(writeBufferDP1A2b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #9: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1, readBufferDP2);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A2), bytesRead);
    EXPECT_EQ(writeBufferDP1A2b, readBufferDP2A2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079OverwriteDataAndOverwriteAttributeAndTransfer) 
{
    uint32_t writeBufferDP1a = 10U;
    uint32_t writeBufferDP1b = 11U;
    uint32_t writeBufferDP1A2a = 102U;
    uint32_t writeBufferDP1A2b = 112U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;

    uint32_t bytesRead;

    // Test #10: Publisher:: SWO / WriteData / WriteAttribute / WriteData / WriteAttribute / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1a, sizeof(writeBufferDP1a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2a, sizeof(writeBufferDP1A2a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &writeBufferDP1b, sizeof(writeBufferDP1b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2b, sizeof(writeBufferDP1A2b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #10: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2), bytesRead);
    EXPECT_EQ(writeBufferDP1b, readBufferDP2);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A2), bytesRead);
    EXPECT_EQ(writeBufferDP1A2b, readBufferDP2A2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4079OverwriteOnlyAttributeAndTransfer) 
{
    uint32_t writeBufferDP1A2a = 101U;
    uint32_t writeBufferDP1A2b = 102U;

    uint32_t readBufferDP2;
    uint32_t readBufferDP2A2;

    uint32_t bytesRead;

    // Test #11: Publisher:: SWO / WriteData / WriteData / CWO
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2a, sizeof(writeBufferDP1A2a))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(dataPacketID1, attributeKey2, &writeBufferDP1A2b, sizeof(writeBufferDP1A2b))); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    // Test #11: Subscriber:: Check values. 
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readData(dataPacketID2, &readBufferDP2, sizeof(readBufferDP2), &bytesRead)); 
    EXPECT_EQ(0U, bytesRead);
    bytesRead = 42U;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(dataPacketID2, attributeKey2, &readBufferDP2A2, sizeof(readBufferDP2A2), &bytesRead)); 
    EXPECT_EQ(sizeof(readBufferDP2A2), bytesRead);
    EXPECT_EQ(writeBufferDP1A2b, readBufferDP2A2);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

TEST_F(DataHandlerInterfaceSimpleTest, DataPacketCreationInDifferentConfigureAndSizeOfSecondSmaller)
{
    char writeBufferDP[42] = "12345678901234567890123456789012345678901";
    char readBufferDP[42];

    size_t firstPortSize = 42U;
    size_t secondPortSize = 40U;

    uint32_t bytesRead;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(firstPortSize, &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(secondPortSize, &pnpPortHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Check that it is possible to read/write from both packages. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(portHandle, &writeBufferDP, firstPortSize));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(pnpPortHandle, &writeBufferDP, secondPortSize));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(portHandle, &readBufferDP, firstPortSize, &bytesRead)); 
    EXPECT_EQ(firstPortSize, bytesRead);
    EXPECT_EQ(0, strcmp(writeBufferDP, readBufferDP));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(pnpPortHandle, &readBufferDP, secondPortSize, &bytesRead)); 
    EXPECT_EQ(secondPortSize, bytesRead);
    EXPECT_EQ(0, strcmp(writeBufferDP, readBufferDP));
}

TEST_F(DataHandlerInterfaceSimpleTest, DataPacketCreationInDifferentConfigureAndSizeOfSecondEqual)
{
    char writeBufferDP[42] = "12345678901234567890123456789012345678901";
    char readBufferDP[42];

    size_t firstPortSize = 42U;
    size_t secondPortSize = 42U;

    uint32_t bytesRead;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(firstPortSize, &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(secondPortSize, &pnpPortHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Check that it is possible to read/write from both packages. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(portHandle, &writeBufferDP, firstPortSize));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(pnpPortHandle, &writeBufferDP, secondPortSize));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(portHandle, &readBufferDP, firstPortSize, &bytesRead)); 
    EXPECT_EQ(firstPortSize, bytesRead);
    EXPECT_EQ(0, strcmp(writeBufferDP, readBufferDP));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(pnpPortHandle, &readBufferDP, secondPortSize, &bytesRead)); 
    EXPECT_EQ(secondPortSize, bytesRead);
    EXPECT_EQ(0, strcmp(writeBufferDP, readBufferDP));
}

TEST_F(DataHandlerInterfaceSimpleTest, DataPacketCreationInDifferentConfigureAndSizeOfSecondBigger)
{
    char writeBufferDP[44] = "1234567890123456789012345678901234567890123";
    char readBufferDP1[42];
    char readBufferDP2[44];

    size_t firstPortSize = 42U;
    size_t secondPortSize = 44U;

    uint32_t bytesRead;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(firstPortSize, &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(secondPortSize, &pnpPortHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Check that it is possible to read/write from both packages. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(portHandle, &writeBufferDP, firstPortSize));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(pnpPortHandle, &writeBufferDP, secondPortSize));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(portHandle, &readBufferDP1, firstPortSize, &bytesRead)); 
    EXPECT_EQ(firstPortSize, bytesRead);
    EXPECT_EQ(0, strncmp(writeBufferDP, readBufferDP1, firstPortSize));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(pnpPortHandle, &readBufferDP2, secondPortSize, &bytesRead)); 
    EXPECT_EQ(secondPortSize, bytesRead);
    EXPECT_EQ(0, strcmp(writeBufferDP, readBufferDP2));
}

TEST_F(DataHandlerInterfaceSimpleTest, DynamicalThreeDataPacketCreationInDifferentMemoryRegion)
{
    char writeBufferDP[44] = "1234567890123456789012345678901234567890123";
    char readBufferDP1[42];
    char readBufferDP2[44];

    size_t firstPortSize = 42U;
    size_t secondPortSize = 44U;
    size_t thirdPortSize = 44U;

    uint32_t bytesRead;

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(firstPortSize, &portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(secondPortSize, &pnpPortHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(thirdPortSize, &pnpPortHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Check that it is possible to read/write from both packages. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(portHandle, &writeBufferDP, firstPortSize));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(pnpPortHandle, &writeBufferDP, secondPortSize));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(portHandle, &readBufferDP1, firstPortSize, &bytesRead)); 
    EXPECT_EQ(firstPortSize, bytesRead);
    EXPECT_EQ(0, strncmp(writeBufferDP, readBufferDP1, firstPortSize));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(pnpPortHandle, &readBufferDP2, secondPortSize, &bytesRead)); 
    EXPECT_EQ(secondPortSize, bytesRead);
    EXPECT_EQ(0, strcmp(writeBufferDP, readBufferDP2));
}

TEST_F(DataHandlerInterfaceSimpleTest, DoubleInit)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init()); 
}

TEST_F(DataHandlerInterfaceSimpleTest, CreateMemoryRegionWithInvalidParameter)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createMemoryRegion(NULL)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, SetNumberOfMemoryRegionWithInvalidParameter)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setNumberOfCopiesInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID, 0U)); 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setNumberOfCopiesInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID, 4U)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, SetNumberOfMemoryRegionWithoutDHInit)
{
#ifdef DEBUG
    xme_core_dataHandler_fini();
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_setNumberOfCopiesInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT, 2U));
#endif
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDatabaseSizeInMemoryRegionWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDatabaseSizeInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID, 0U)); 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDatabaseSizeInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID, 4096U)); 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_setDatabaseSizeInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT, 0U)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDatabaseSizeInMemoryRegionWithoutDHInit)
{
    xme_core_dataHandler_fini();
    ASSERT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_dataHandler_setDatabaseSizeInMemoryRegion(XME_CORE_DATAMANAGER_MEMORYREGIONID_DEFAULT, 4096U)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, CreateDataPacketWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createDataPacket(0U, NULL)); 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createDataPacket(4U, NULL)); 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createDataPacket(0U, &portHandle)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, CreateDataPacketWithoutDHInit)
{
#ifdef DEBUG
    xme_core_dataHandler_fini();
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_createDataPacket(4U, &portHandle));
#endif
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDataPacketPersistentWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketPersistent(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDataPacketPersistentWithoutDHInit)
{
#ifdef DEBUG
    xme_core_dataHandler_fini();
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_setDataPacketPersistent(dataPacketID1));
#endif
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDataPacketPersistentWithNonRegisteredDataPacket)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketPersistent(dataPacketID1)); 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketPersistent(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDataPacketPersistentWithValidDataPacket)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketPersistent(portHandle)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDataPacketQueueSizeWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketQueueSize(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 0U)); 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketQueueSize(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 4U)); 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_setDataPacketQueueSize(dataPacketID1, 0U)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDataPacketQueueSizeWithoutDHInit)
{
#ifdef DEBUG
    xme_core_dataHandler_fini();
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_setDataPacketQueueSize(dataPacketID1, 4U));
#endif
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDataPacketQueueSizeWithNonRegisteredDataPacket)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketQueueSize(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 1U)); 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_setDataPacketQueueSize(dataPacketID1, 4U)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, SetDataPacketQueueSizeWithValidDataPacket)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(portHandle, 1U)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketQueueSize(portHandle, 2U)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, CreateAttributeWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_createAttribute(0U, XME_CORE_ATTRIBUTE_KEY_UNDEFINED, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)); 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createAttribute(0U, XME_CORE_ATTRIBUTE_KEY_UNDEFINED, dataPacketID1)); 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_createAttribute(0U, attributeKey1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)); 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createAttribute(0U, attributeKey1, dataPacketID1)); 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_createAttribute(4U, XME_CORE_ATTRIBUTE_KEY_UNDEFINED, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)); 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_createAttribute(4U, XME_CORE_ATTRIBUTE_KEY_UNDEFINED, dataPacketID1)); 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_createAttribute(4U, attributeKey1, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, CreateAttributeWithoutDHInit)
{
#ifdef DEBUG
    xme_core_dataHandler_fini();
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_createAttribute(4U, attributeKey1, dataPacketID1));
#endif
}

TEST_F(DataHandlerInterfaceSimpleTest, CreateAttributeWithNonRegisteredDataPacket)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_createAttribute(4U, attributeKey1, dataPacketID1)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, CreateAttributeWithValidDataPacket)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
}

TEST_F(DataHandlerInterfaceSimpleTest, ConfigureWithoutInit)
{
#ifdef DEBUG
    xme_core_dataHandler_fini();
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_configure());
#endif
}


TEST_F(DataHandlerInterfaceSimpleTest, WriteDataWithInvalidParameters)
{
    char* buffer = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    // Invalid data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, &buffer, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, &buffer, 40U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL, 40U));

    // Valid data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeData(portHandle, &buffer, 0U));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(portHandle, &buffer, 40U));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeData(portHandle, NULL, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeData(portHandle, NULL, 40U));

    // Non registered data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeData(dataPacketID4, &buffer, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeData(dataPacketID4, &buffer, 40U));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeData(dataPacketID4, NULL, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeData(dataPacketID4, NULL, 40U));
}

TEST_F(DataHandlerInterfaceSimpleTest, WriteAttributeWithInvalidParameters)
{
    char* buffer = NULL;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    // Invalid data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey2, &buffer, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey2, &buffer, 40U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey2, NULL, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey2, NULL, 40U));

    // Valid data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeAttribute(portHandle, attributeKey2, &buffer, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeAttribute(portHandle, attributeKey2, &buffer, 40U));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(portHandle, attributeKey1, &buffer, 40U));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeAttribute(portHandle, attributeKey2, NULL, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeAttribute(portHandle, attributeKey2, NULL, 40U));

    // Non registered data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeAttribute(dataPacketID4, attributeKey2, &buffer, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_writeAttribute(dataPacketID4, attributeKey2, &buffer, 40U));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeAttribute(dataPacketID4, attributeKey2, NULL, 0U));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_writeAttribute(dataPacketID4, attributeKey2, NULL, 40U));
}

TEST_F(DataHandlerInterfaceSimpleTest, ReadDataWithInvalidParameters)
{
    uint32_t dataToWrite = 512U;
    uint32_t buffer;
    uint32_t bytesRead;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(dataToWrite), &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketPersistent(portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(portHandle, &dataToWrite, sizeof(dataToWrite)));

    // Invalid data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, &buffer, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, &buffer, 4U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL, 4U, &bytesRead));

    // Valid data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readData(portHandle, &buffer, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(portHandle, &buffer, 4U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readData(portHandle, NULL, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readData(portHandle, NULL, 4U, &bytesRead));

    // Non registered data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readData(dataPacketID4, &buffer, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readData(dataPacketID4, &buffer, 4U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readData(dataPacketID4, NULL, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readData(dataPacketID4, NULL, 4U, &bytesRead));
}

TEST_F(DataHandlerInterfaceSimpleTest, ReadAttributeWithInvalidParameters)
{
    uint32_t dataToWrite = 512U;
    uint32_t buffer;
    uint32_t bytesRead;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(dataToWrite), &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_setDataPacketPersistent(portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(portHandle, attributeKey1, &dataToWrite, sizeof(dataToWrite)));

    // Invalid data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey2, &buffer, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey2, &buffer, 4U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey2, NULL, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey2, NULL, 4U, &bytesRead));

    // Valid data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(portHandle, attributeKey2, &buffer, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(portHandle, attributeKey2, &buffer, 4U, &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(portHandle, attributeKey1, &buffer, 4U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(portHandle, attributeKey2, NULL, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(portHandle, attributeKey2, NULL, 4U, &bytesRead));

    // Non registered data packet id combinations. 
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(dataPacketID4, attributeKey2, &buffer, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_readAttribute(dataPacketID4, attributeKey2, &buffer, 4U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(dataPacketID4, attributeKey2, NULL, 0U, &bytesRead));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_readAttribute(dataPacketID4, attributeKey2, NULL, 4U, &bytesRead));
}

TEST_F(DataHandlerInterfaceSimpleTest, ReadWriteTestBufferSize)
{
    char dataToWrite[5] = {1,2,3,4,5};
    char dataToRead[5];
    uint32_t bytesRead;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(dataToWrite) - 1, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(dataToWrite) - 1, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    // Check buffers for write operation. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(portHandle, &dataToWrite, sizeof(dataToWrite)));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(portHandle, attributeKey1, &dataToWrite, sizeof(dataToWrite)));

    // Check buffers for write operation. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(portHandle, &dataToRead, sizeof(dataToRead), &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(portHandle, attributeKey1, &dataToRead, sizeof(dataToRead), &bytesRead));
}

TEST_F(DataHandlerInterfaceSimpleTest, StartReadOperationWithUninitializedDatabase)
{
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_startReadOperation(dataPacketID1));
#endif
}
    
TEST_F(DataHandlerInterfaceSimpleTest, StartReadOperationWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_startReadOperation(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_startReadOperation(dataPacketID4));
}

TEST_F(DataHandlerInterfaceSimpleTest, StartReadOperationWithValidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
}

TEST_F(DataHandlerInterfaceSimpleTest, CompleteReadOperationWithUninitializedDatabase)
{
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_completeReadOperation(dataPacketID1));
#endif
}
    
TEST_F(DataHandlerInterfaceSimpleTest, CompleteReadOperationWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_completeReadOperation(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_completeReadOperation(dataPacketID4));
}

TEST_F(DataHandlerInterfaceSimpleTest, CompleteReadOperationWithValidParameters)
{
    uint32_t dataToWrite = 512U;
    uint32_t buffer;
    uint32_t bytesRead;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(portHandle, &dataToWrite, sizeof(dataToWrite)));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(portHandle, &buffer, 4U, &bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));
}

TEST_F(DataHandlerInterfaceSimpleTest, StartWriteOperationWithUninitializedDatabase)
{
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_startWriteOperation(dataPacketID1));
#endif
}
    
TEST_F(DataHandlerInterfaceSimpleTest, StartWriteOperationWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_startWriteOperation(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_startWriteOperation(dataPacketID4));
}

TEST_F(DataHandlerInterfaceSimpleTest, StartWriteOperationWithValidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
}

TEST_F(DataHandlerInterfaceSimpleTest, CompleteWriteOperationWithUninitializedDatabase)
{
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_completeWriteOperation(dataPacketID1));
#endif
}
    
TEST_F(DataHandlerInterfaceSimpleTest, CompleteWriteOperationWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_completeWriteOperation(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_dataHandler_completeWriteOperation(dataPacketID4));
}

TEST_F(DataHandlerInterfaceSimpleTest, CompleteWriteOperationWithValidParameters)
{
    uint32_t dataToWrite = 512U;

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    // Note this use case. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(portHandle, &dataToWrite, sizeof(dataToWrite)));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));
}

TEST_F(DataHandlerInterfaceSimpleTest, StartWriteOperationWhileWriteLocked)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));

    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED, xme_core_dataHandler_startWriteOperation(dataPacketID1));
}

TEST_F(DataHandlerInterfaceSimpleTest, StartReadOperationWhileReadLocked)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(1024U, &portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(4U, attributeKey1, portHandle)); 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure()); 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));

    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED, xme_core_dataHandler_startReadOperation(dataPacketID1));
}

TEST_F(DataHandlerNewInterfaceTest, Issue4267_ConsumptionWithAndWithoutSubscriber)
{
    // This data store is targeted to be a publisher. 
    xme_core_dataManager_dataStoreID_t dataStoreID3;
    uint32_t dataToWrite = 512U;
    uint32_t buffer;
    uint32_t bytesRead;

    // Data Store creation: Publication
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataPacket(sizeof(uint32_t), &dataStoreID3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey1, dataStoreID3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey2, dataStoreID3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createAttribute(sizeof(uint32_t), attributeKey3, dataStoreID3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_configure());

    // Write in both publications ports. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataPacketID1, &dataToWrite, sizeof(dataToWrite)));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataPacketID1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startWriteOperation(dataStoreID3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(dataStoreID3, &dataToWrite, sizeof(dataToWrite)));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(dataStoreID3));

    // Check overwrite. 
    // Theoretically, dataPacketID1 and dataPacketID3 do not have data. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID1));
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readData(dataPacketID1, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(0U, bytesRead);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataStoreID3));
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_dataHandler_readData(dataStoreID3, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(0U, bytesRead);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataStoreID3));

    // ... but, dataPacketID2 has the result of the transfer. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_startReadOperation(dataPacketID2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(dataPacketID2, &buffer, sizeof(buffer), &bytesRead));
    ASSERT_EQ(sizeof(dataToWrite), bytesRead);
    ASSERT_EQ(buffer, dataToWrite);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(dataPacketID2));
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
