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
 * $Id: integrationTestDataHandler.cpp 7664 2014-03-04 08:47:41Z geisinger $
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
#include <cstring>
#include <vector>

#include "xme/core/component.h"
#include "xme/hal/include/mem.h"

#include "xme/core/broker/include/broker.h"

#include "xme/core/dataManagerTypes.h"
#include "xme/core/dataHandler/include/dataHandler.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define DATAHANDLER_TEST_MEMORYVALUE 15

/**
 * \enum test_attributes_t
 * \brief Defines a set of keys that will be used as attribute keys. 
 */
typedef enum
{
    ATTRIBUTE_KEY_UNDEFINED = XME_CORE_ATTRIBUTE_KEY_UNDEFINED, ///< undefined attribute key.
    ATTRIBUTE_KEY_TEST1 = XME_CORE_ATTRIBUTE_KEY_USER, ///< assign first user attribute key. 
    ATTRIBUTE_KEY_TEST2, ///< second attribute key. 
    ATTRIBUTE_KEY_TEST3 ///< third attribute key. 
} test_attributes_t;

/**
 * \struct attribute_test1_t
 * \brief The struct containing the first attribute.
 */
typedef struct attribute_test1
{
    unsigned int value;
} attribute_test1_t;

/**
 * \struct attribute_test2_t
 * \brief The struct containing the second attribute. 
 */
typedef struct attribute_test2
{
    double value;
}attribute_test2_t;

/**
 * \struct attribute_test3_t
 * \brief The struct containing the third attribute. 
 */
typedef struct attribute_test3
{
    int value;
}attribute_test3_t;

xme_core_attribute_descriptor_t attributeKeys1[] = {{XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), sizeof(attribute_test1_t)}};
xme_core_attribute_descriptor_t attributeKeys2[] = {{XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2), sizeof(attribute_test2_t)}};
xme_core_attribute_descriptor_t attributeKeys3[] = {{XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), sizeof(attribute_test1_t)},
                                                    {XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2), sizeof(attribute_test2_t)}};
xme_core_attribute_descriptor_t attributeKeys4[] = {{XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), sizeof(attribute_test1_t)},
                                                    {XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST3), sizeof(attribute_test3_t)}};

// Publisher and subscriber descriptor with key1.
xme_core_attribute_descriptor_list_t publisherDescriptor1 = {sizeof(attributeKeys1)/sizeof(attributeKeys1[0]), attributeKeys1};
xme_core_attribute_descriptor_list_t subscriberDescriptor1 = {sizeof(attributeKeys1)/sizeof(attributeKeys1[0]), attributeKeys1};

// Publisher and subscriber descriptor with key2.
xme_core_attribute_descriptor_list_t publisherDescriptor2 = {sizeof(attributeKeys2)/sizeof(attributeKeys2[0]), attributeKeys2};
xme_core_attribute_descriptor_list_t subscriberDescriptor2 = {sizeof(attributeKeys2)/sizeof(attributeKeys2[0]), attributeKeys2};

// Publisher and subscriber descriptor with key1 and key2.
xme_core_attribute_descriptor_list_t publisherDescriptor3 = {sizeof(attributeKeys3)/sizeof(attributeKeys3[0]), attributeKeys3};
xme_core_attribute_descriptor_list_t subscriberDescriptor3 = {sizeof(attributeKeys3)/sizeof(attributeKeys3[0]), attributeKeys3};

// Subscriber descriptor with key1 and key3.
xme_core_attribute_descriptor_list_t subscriberDescriptor4 = {sizeof(attributeKeys4)/sizeof(attributeKeys4[0]), attributeKeys4};

class Port {
    public:
        Port
        (
            xme_core_dataManager_dataPacketId_t anIdentifier,
            xme_core_component_portType_t aType = XME_CORE_COMPONENT_PORTTYPE_INVALID,
            xme_core_topic_t aTopic = XME_CORE_TOPIC_INVALID_TOPIC,
            unsigned int aBufferSize = 0u
        ) 
        : identifier(anIdentifier)
        , type(aType)
        , topic(aTopic)
        , portHandle(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID)
        , bufferSize(aBufferSize)
        , bytesRead(0u)
        , buffer(NULL) 
        {
            if (bufferSize > 0u) {
                buffer = new xme_core_topic_t[this->bufferSize];
                xme_hal_mem_set(buffer, 0, bufferSize * sizeof(xme_core_topic_t));
            }
        }

        virtual
        ~Port() {
            if (buffer) {
               // delete[] (xme_core_topic_t*) buffer;
            }
        }
    public:
        xme_core_dataManager_dataPacketId_t identifier;
        xme_core_component_portType_t type;
        xme_core_topic_t topic;
        xme_core_dataManager_dataPacketId_t portHandle;
        unsigned int bufferSize;
        unsigned int bytesRead;
        xme_core_topic_t* buffer;
};

class Application {
    public:
        Application(xme_core_component_t srcComponentID, xme_core_component_t dstComponentID) 
        : componentID1 (srcComponentID)
        , componentID2 (dstComponentID)
        {
            // Previous examples
            Port *first = new Port(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                   XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                   (xme_core_topic_t) 1, 
                                   10u);

            Port *second = new Port(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                    XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                    (xme_core_topic_t) 2, 
                                    100u);

            Port *third = new Port(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                   XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                   (xme_core_topic_t) 1, 
                                   10u);

            Port *fourth = new Port(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                    XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                    (xme_core_topic_t) 2, 
                                    100u);

            // New examples for attribute testing. 
            Port *pub1 = new Port((xme_core_dataManager_dataPacketId_t) 11,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                  (xme_core_topic_t) 1,
                                  8u);
            Port *sub1 = new Port((xme_core_dataManager_dataPacketId_t) 12,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                  (xme_core_topic_t) 1,
                                  8u);

            Port *pub2 = new Port((xme_core_dataManager_dataPacketId_t) 21,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                  (xme_core_topic_t) 2,
                                  16u);
            Port *sub2 = new Port((xme_core_dataManager_dataPacketId_t) 22,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                  (xme_core_topic_t) 2,
                                  16u);

            Port *pub3 = new Port((xme_core_dataManager_dataPacketId_t) 31,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                  (xme_core_topic_t) 3,
                                  32u);
            Port *sub3 = new Port((xme_core_dataManager_dataPacketId_t) 32,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                  (xme_core_topic_t) 3,
                                  32u);

            Port *pub4 = new Port((xme_core_dataManager_dataPacketId_t) 41,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                  (xme_core_topic_t) 4,
                                  64u);
            Port *sub4 = new Port((xme_core_dataManager_dataPacketId_t) 42,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                  (xme_core_topic_t) 4,
                                  64u);

            Port *pub5 = new Port((xme_core_dataManager_dataPacketId_t) 51,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                  (xme_core_topic_t) 5,
                                  128u);
            Port *sub5 = new Port((xme_core_dataManager_dataPacketId_t) 52,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                  (xme_core_topic_t) 5,
                                  128u);

            Port *pub6 = new Port((xme_core_dataManager_dataPacketId_t) 61,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                                  (xme_core_topic_t) 6,
                                  256u);
            Port *sub6 = new Port((xme_core_dataManager_dataPacketId_t) 62,
                                  XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                                  (xme_core_topic_t) 6,
                                  256u);

            // Now we populate the buffer with some data, to be read by the publisher. 
            for (unsigned int i = 0u; i < third->bufferSize; ++i) {
                ((xme_core_topic_t*) third->buffer)[i] = (xme_core_topic_t) 0xdeadbeef;
            }

            for (unsigned int i = 0u; i < fourth->bufferSize; ++i) {
                ((xme_core_topic_t*) fourth->buffer)[i] = (xme_core_topic_t) 0xc0ffebabe;
            }

            // New attribute tests. 
            for (unsigned int i = 0u; i < pub1->bufferSize; ++i) {
                ((xme_core_topic_t*) pub1->buffer)[i] = (xme_core_topic_t) 0xaaaaaaa0;
            }

            for (unsigned int i = 0u; i < pub2->bufferSize; ++i) {
                ((xme_core_topic_t*) pub2->buffer)[i] = (xme_core_topic_t) 0xbbbbbbb0;
            }

            for (unsigned int i = 0u; i < pub3->bufferSize; ++i) {
                ((xme_core_topic_t*) pub3->buffer)[i] = (xme_core_topic_t) 0xccccccc0;
            }

            for (unsigned int i = 0u; i < pub4->bufferSize; ++i) {
                ((xme_core_topic_t*) pub4->buffer)[i] = (xme_core_topic_t) 0xddddddd0;
            }

            for (unsigned int i = 0u; i < pub4->bufferSize; ++i) {
                ((xme_core_topic_t*) pub5->buffer)[i] = (xme_core_topic_t) 0xeeeeeee0;
            }

            for (unsigned int i = 0u; i < pub4->bufferSize; ++i) {
                ((xme_core_topic_t*) pub6->buffer)[i] = (xme_core_topic_t) 0xfffffff0;
            }

            // Add to the vector all these port definitions. 
            inport.push_back(first);
            inport.push_back(second);
            outport.push_back(third);
            outport.push_back(fourth);

            // publishers & subscribers
            outport.push_back(pub1);
            inport.push_back(sub1);
            outport.push_back(pub2);
            inport.push_back(sub2);
            outport.push_back(pub3);
            inport.push_back(sub3);
            outport.push_back(pub4);
            inport.push_back(sub4);
            outport.push_back(pub5);
            inport.push_back(sub5);
            outport.push_back(pub6);
            inport.push_back(sub6);
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
        xme_core_component_t componentID1;
        xme_core_component_t componentID2;
        std::vector <Port*> inport;
        std::vector <Port*> outport;
};

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class DataHandlerIntegrationTest : public ::testing::Test {
    protected:
        DataHandlerIntegrationTest() : app((xme_core_component_t)1, (xme_core_component_t)2) {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
            if(XME_STATUS_SUCCESS == xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE))
            {
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID2,
                                                      app.inport[0]->type,
                                                      app.inport[0]->topic,
                                                      app.inport[0]->bufferSize * sizeof(xme_core_topic_t),
                                                      subscriberDescriptor1, 2U, // queue size 2
                                                      false, true, 0,
                                                      &app.inport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID2,
                                                      app.inport[1]->type,
                                                      app.inport[1]->topic,
                                                      app.inport[1]->bufferSize * sizeof(xme_core_topic_t),
                                                      subscriberDescriptor2, 1,
                                                      false, true, 0,
                                                      &app.inport[1]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.outport[0]->type,
                                                      app.outport[0]->topic,
                                                      app.outport[0]->bufferSize * sizeof(xme_core_topic_t),
                                                      publisherDescriptor1, 2U, // queue size 2
                                                      false, true, 0,
                                                      &app.outport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.outport[1]->type,
                                                      app.outport[1]->topic,
                                                      app.outport[1]->bufferSize * sizeof(xme_core_topic_t),
                                                      publisherDescriptor2, 1,
                                                      false, true, 0,
                                                      &app.outport[1]->portHandle));
            }
            else
            {
                throw;
            }
        }

        virtual
        ~DataHandlerIntegrationTest() {
            xme_core_dataHandler_fini();
            xme_core_broker_fini();
        }

        Application app;
};

class DataHandlerAttributeIntegrationTest : public ::testing::Test {
    protected:
        DataHandlerAttributeIntegrationTest() : app((xme_core_component_t)1, (xme_core_component_t)2) {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_broker_init(NULL));
            if(XME_STATUS_SUCCESS == xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE))
            {

            // USE CASE 1: publication and subscription share the same attributes. 
            // publication1 (index in outport = 2) (attribute key = 1)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.outport[2]->type,
                                                      app.outport[2]->topic,
                                                      app.outport[2]->bufferSize * sizeof(xme_core_topic_t),
                                                      publisherDescriptor1, 1,
                                                      false, true, 0,
                                                      &app.outport[2]->portHandle));
            // subscription1 (index in inport = 2) (attribute key = 1)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.inport[2]->type,
                                                      app.inport[2]->topic,
                                                      app.inport[2]->bufferSize * sizeof(xme_core_topic_t),
                                                      subscriberDescriptor1, 1,
                                                      false, true, 0,
                                                      &app.inport[2]->portHandle));

            // USE CASE 2: publication and subscription do not share the same key. 
            // publication2 (index in outport = 3) (attribute key = 1)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.outport[3]->type,
                                                      app.outport[3]->topic,
                                                      app.outport[3]->bufferSize * sizeof(xme_core_topic_t),
                                                      publisherDescriptor1, 1, 
                                                      false, true, 0,
                                                      &app.outport[3]->portHandle));
            // subscription2 (index in inport = 3) (attribute key = 2)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.inport[3]->type,
                                                      app.inport[3]->topic,
                                                      app.inport[3]->bufferSize * sizeof(xme_core_topic_t),
                                                      subscriberDescriptor2, 1,
                                                      false, true, 0,
                                                      &app.inport[3]->portHandle));

            // USE CASE 3: publication has two attributes (key 1 and 2) and subscriber has only one (key 2). 
            // publication3 (index in outport = 4) (attribute key = 1,2)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.outport[4]->type,
                                                      app.outport[4]->topic,
                                                      app.outport[4]->bufferSize * sizeof(xme_core_topic_t),
                                                      publisherDescriptor3, 1,
                                                      false, true, 0,
                                                      &app.outport[4]->portHandle));
            // subscription3 (index in inport = 4) (attribute key = 2)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.inport[4]->type,
                                                      app.inport[4]->topic,
                                                      app.inport[4]->bufferSize * sizeof(xme_core_topic_t),
                                                      subscriberDescriptor2, 1,
                                                      false, true, 0,
                                                      &app.inport[4]->portHandle));

            // USE CASE 4: publication has one attribute (key 1) and subscriber has two keys (key 1 and 2). 
            // publication4 (index in outport = 5) (attribute key = 1)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.outport[5]->type,
                                                      app.outport[5]->topic,
                                                      app.outport[5]->bufferSize * sizeof(xme_core_topic_t),
                                                      publisherDescriptor1, 1,
                                                      false, true, 0,
                                                      &app.outport[5]->portHandle));

            // subscription4 (index in inport = 5) (attribute key = 1,2)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.inport[5]->type,
                                                      app.inport[5]->topic,
                                                      app.inport[5]->bufferSize * sizeof(xme_core_topic_t),
                                                      subscriberDescriptor3, 1,
                                                      false, true, 0,
                                                      &app.inport[5]->portHandle));

            // USE CASE 5: publication has two attributes (key 1 and 2) and subscriber has two keys (key 1 and 2). 
            // publication5 (index in outport = 6) (attribute key = 1,2)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.outport[6]->type,
                                                      app.outport[6]->topic,
                                                      app.outport[6]->bufferSize * sizeof(xme_core_topic_t),
                                                      publisherDescriptor3, 1,
                                                      false, true, 0,
                                                      &app.outport[6]->portHandle));

            // subscription5 (index in inport = 6) (attribute key = 1,2)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.inport[6]->type,
                                                      app.inport[6]->topic,
                                                      app.inport[6]->bufferSize * sizeof(xme_core_topic_t),
                                                      subscriberDescriptor3, 1,
                                                      false, true, 0,
                                                      &app.inport[6]->portHandle));

            // USE CASE 6: publication has two attributes (key 1 and 2) and subscriber has two keys (key 1 and 3).
            //             one of the keys matches and the other no. 
            // publication6 (index in outport = 7) (attribute key = 1,2)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.outport[7]->type,
                                                      app.outport[7]->topic,
                                                      app.outport[7]->bufferSize * sizeof(xme_core_topic_t),
                                                      publisherDescriptor3, 1,
                                                      false, true, 0,
                                                      &app.outport[7]->portHandle));

            // subscription6 (index in inport = 7) (attribute key = 1,3)
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.componentID1,
                                                      app.inport[7]->type,
                                                      app.inport[7]->topic,
                                                      app.inport[7]->bufferSize * sizeof(xme_core_topic_t),
                                                      subscriberDescriptor4, 1,
                                                      false, true, 0,
                                                      &app.inport[7]->portHandle));
            }
            else
            {
                throw;
            }
        }

        virtual
        ~DataHandlerAttributeIntegrationTest() {
            xme_core_dataHandler_fini();
            xme_core_broker_fini();
        }

        Application app;
};

TEST_F(DataHandlerIntegrationTest, writeTenElementsTransferItAndReadAsMuchPossible)
{
    unsigned int elements = 10u;
    ASSERT_GE(app.outport[0]->bufferSize, elements);

    // Configure Broker to perform the transfer between outport[0] and inport[0] automatically
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(app.outport[0]->portHandle,
                                                         app.inport[0]->portHandle));

    // Write data
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             elements * sizeof(app.outport[0]->buffer[0])));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

    // Transfer implicitly done by Broker

    // Read data
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                            app.inport[0]->buffer,
                                            app.inport[0]->bufferSize * sizeof(app.inport[0]->buffer[0]),
                                            &app.inport[0]->bytesRead));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));

    // Check data
    ASSERT_EQ(elements, app.inport[0]->bytesRead / sizeof(app.inport[0]->buffer[0]));
    for(unsigned int i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[0]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[0]->buffer)[i]);
    }
}

TEST_F(DataHandlerIntegrationTest, writeFiveElementsTransferAndReadAsMuchAsPossible)
{
    unsigned int elements = 5u;
    ASSERT_GE(app.outport[0]->bufferSize, elements);

    // Configure Broker to perform the transfer between outport[0] and inport[0] automatically
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(app.outport[0]->portHandle,
                                                         app.inport[0]->portHandle));

    // Write data
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             elements * sizeof(app.outport[0]->buffer[0])));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

    // Transfer implicitly done by Broker

    // Read data
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                            app.inport[0]->buffer,
                                            app.inport[0]->bufferSize * sizeof(app.inport[0]->buffer[0]),
                                            &app.inport[0]->bytesRead));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));

    // Check data
    app.inport[0]->bytesRead = app.inport[0]->bufferSize ;/// sizeof(int);
    ASSERT_EQ(10u, app.inport[0]->bytesRead);
    for(unsigned int i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[0]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[0]->buffer)[i])
                                  << "Got following i = " << i;
    }
}

TEST_F(DataHandlerIntegrationTest, writeTransferReadTest)
{
  // Write 10 elements to dataHandler and read 10 elements back
  unsigned int elements = 10u;

  // init test layout
  for(unsigned i = 0u; i < app.outport[0]->bufferSize; ++i)
  {
    if (i%2==0)    ((xme_core_topic_t*)app.outport[0]->buffer)[i] = (xme_core_topic_t) 0xdeadbeef;
    else        ((xme_core_topic_t*)app.outport[0]->buffer)[i] = (xme_core_topic_t) 0x3ace3ace;
  }

  // Configure Broker to perform the transfer between outport[0] and inport[0] automatically
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_broker_addDataPacketTransferEntry(app.outport[0]->portHandle,
                                                       app.inport[0]->portHandle));

  // Write data
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                           app.outport[0]->buffer,
                                           elements * sizeof(app.outport[0]->buffer[0])));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

  // Transfer implicitly done by Broker

  // Read data
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                          app.inport[0]->buffer,
                                          elements * sizeof(app.inport[0]->buffer[0]),
                                          &app.inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));

  // Check data
  ASSERT_EQ(elements, app.inport[0]->bytesRead / sizeof(app.inport[0]->buffer[0]));
  for(unsigned i = 0u; i < elements; ++i)
  {
      ASSERT_EQ(((xme_core_topic_t*)app.outport[0]->buffer)[i],
                ((xme_core_topic_t*)app.inport[0]->buffer)[i]);
  }
}

TEST_F(DataHandlerIntegrationTest, writeFiveElementsTransferAndReadFiveElementsBack)
{
  // Write 5 elements to dataHandler and read 5 elements back
  unsigned elements = 5u;

  // Configure Broker to perform the transfer between outport[0] and inport[0] automatically
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_broker_addDataPacketTransferEntry(app.outport[0]->portHandle,
                                                       app.inport[0]->portHandle));

  // Write data
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                           app.outport[0]->buffer,
                                           elements * sizeof(app.outport[0]->buffer[0])));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

  // Transfer implicitly done by Broker

  // Read data
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                          app.inport[0]->buffer,
                                          elements * sizeof(app.inport[0]->buffer[0]),
                                          &app.inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));

  // Check data
  ASSERT_EQ(elements, app.inport[0]->bytesRead / sizeof(app.inport[0]->buffer[0]));
  for(unsigned i = 0u; i < elements; ++i)
  {
      ASSERT_EQ(((xme_core_topic_t*)app.outport[0]->buffer)[i],
                ((xme_core_topic_t*)app.inport[0]->buffer)[i]);
  }
}

TEST_F(DataHandlerIntegrationTest, writeTransferReadAttributeTest)
{
    unsigned int elements = 5u;

    attribute_test1_t inputTest1 = {50};
    attribute_test2_t inputTest2 = {0.42};
    attribute_test1_t outputTest1;
    attribute_test2_t outputTest2;

    unsigned int read;

    // Configure Broker to perform the transfer between out/inport[0] and out/inport[1] automatically
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(app.outport[0]->portHandle,
                                                         app.inport[0]->portHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(app.outport[1]->portHandle,
                                                         app.inport[1]->portHandle));

    // Write data and attributes
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             elements * sizeof(app.outport[0]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[0]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                  &inputTest1,
                                                  sizeof(inputTest1)));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[1]->portHandle,
                                             app.outport[1]->buffer,
                                             elements * sizeof(app.outport[0]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[1]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                  &inputTest2,
                                                  sizeof(inputTest2)));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[1]->portHandle));

    // Read data and attributes
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                            app.inport[0]->buffer,
                                            elements * sizeof(app.inport[0]->buffer[0]),
                                            &app.inport[0]->bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[0]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                 &outputTest1,
                                                 sizeof(outputTest1),
                                                 &read));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[1]->portHandle,
                                            app.inport[1]->buffer,
                                            elements * sizeof(app.inport[0]->buffer[0]),
                                            &app.inport[1]->bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[1]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                 &outputTest2,
                                                 sizeof(outputTest2),
                                                 &read));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(app.inport[1]->portHandle));

    // Check data
    ASSERT_EQ(elements, app.inport[0]->bytesRead / sizeof(app.inport[0]->buffer[0]));
    for(unsigned i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[0]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[0]->buffer)[i]);
    }

    ASSERT_EQ(elements, app.inport[1]->bytesRead / sizeof(app.inport[0]->buffer[0]));
    for(unsigned i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[1]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[1]->buffer)[i]);
    }

    ASSERT_EQ(inputTest1.value, outputTest1.value);
    ASSERT_EQ(inputTest2.value, outputTest2.value);
}

TEST_F(DataHandlerAttributeIntegrationTest, issue2871)
{
    unsigned int elements;

    attribute_test1_t inputTest1 = {16};
    attribute_test2_t inputTest2 = {0.32};
    attribute_test1_t outputTest11;
    attribute_test1_t outputTest21;
    attribute_test2_t outputTest22;
    attribute_test1_t outputTest31;
    attribute_test2_t outputTest32;
    attribute_test1_t outputTest41;
    attribute_test2_t outputTest42;
    attribute_test1_t outputTest51;
    attribute_test2_t outputTest52;
    attribute_test1_t outputTest61;
    attribute_test2_t outputTest62;
    attribute_test3_t outputTest63;

    unsigned int read;

    // USE CASE 1: Publisher and consumer share the same attributes. 
    elements = 8u;


    // publisher 1 (index=2)
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[2]->portHandle,
                                             app.outport[2]->buffer,
                                             elements * sizeof(app.outport[2]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[2]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                  &inputTest1,
                                                  sizeof(inputTest1)));

    // Do the transfer
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[2]->portHandle,
                                                app.inport[2]->portHandle));

    // Check the data. 
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[2]->portHandle,
                                            app.inport[2]->buffer,
                                            elements * sizeof(app.inport[2]->buffer[0]),
                                            &app.inport[2]->bytesRead));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[2]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                 &outputTest11,
                                                 sizeof(outputTest11),
                                                 &read));

    ASSERT_EQ(elements, app.inport[2]->bytesRead / sizeof(app.inport[2]->buffer[0]));
    for(unsigned i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[2]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[2]->buffer)[i]);
    }

    ASSERT_EQ(inputTest1.value, outputTest11.value);

    // USE CASE 2: Publisher and consumer DO NOT share the same attributes. 
    elements = 16u;

    // publisher 2 (index=3)
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[3]->portHandle,
                                             app.outport[3]->buffer,
                                             elements * sizeof(app.outport[3]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[3]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                  &inputTest1,
                                                  sizeof(inputTest1)));

    // Do the transfer
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[3]->portHandle,
                                                app.inport[3]->portHandle));

    // Check the data. 
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[3]->portHandle,
                                            app.inport[3]->buffer,
                                            elements * sizeof(app.inport[3]->buffer[0]),
                                            &app.inport[3]->bytesRead));

    // Note that the subscription does not containt key1. 
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE,
              xme_core_dataHandler_readAttribute(app.inport[3]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                 &outputTest21,
                                                 sizeof(outputTest21),
                                                 &read));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[3]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                 &outputTest22,
                                                 sizeof(outputTest22),
                                                 &read));

    ASSERT_EQ(elements, app.inport[3]->bytesRead / sizeof(app.inport[3]->buffer[0]));

    // Topic check. 
    for(unsigned i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[3]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[3]->buffer)[i]);
    }

    ASSERT_EQ(0u, outputTest22.value);

    // USE CASE 3: Publisher has two attributes (with keys 1 and 2) and consumer has only one attribute (with key 1). 
    elements = 32u;

    // publisher 3 (index=4)
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[4]->portHandle,
                                             app.outport[4]->buffer,
                                             elements * sizeof(app.outport[4]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[4]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                  &inputTest1,
                                                  sizeof(inputTest1)));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[4]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                  &inputTest2,
                                                  sizeof(inputTest2)));

    // Do the transfer
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[4]->portHandle,
                                                app.inport[4]->portHandle));

    // Check the data in subscriber. 
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[4]->portHandle,
                                            app.inport[4]->buffer,
                                            elements * sizeof(app.inport[4]->buffer[0]),
                                            &app.inport[4]->bytesRead));

    // Note that the subscription does not containt key1. 
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE,
              xme_core_dataHandler_readAttribute(app.inport[4]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                 &outputTest31,
                                                 sizeof(outputTest31),
                                                 &read));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[4]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                 &outputTest32,
                                                 sizeof(outputTest32),
                                                 &read));

    ASSERT_EQ(elements, app.inport[4]->bytesRead / sizeof(app.inport[4]->buffer[0]));

    // Topic check. 
    for(unsigned i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[4]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[4]->buffer)[i]);
    }

    ASSERT_EQ(inputTest2.value, outputTest32.value);

    // USE CASE 4: Publisher has one attribute (with key 1) and consumer has two attributes (with keys 1 and 2). 
    elements = 64u;

    // publisher 4 (index=5)
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[5]->portHandle,
                                             app.outport[5]->buffer,
                                             elements * sizeof(app.outport[5]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[5]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                  &inputTest1,
                                                  sizeof(inputTest1)));

    // Do the transfer
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[5]->portHandle,
                                                app.inport[5]->portHandle));

    // Check the data in subscriber. 
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[5]->portHandle,
                                            app.inport[5]->buffer,
                                            elements * sizeof(app.inport[5]->buffer[0]),
                                            &app.inport[5]->bytesRead));

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[5]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                 &outputTest41,
                                                 sizeof(outputTest41),
                                                 &read));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[5]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                 &outputTest42,
                                                 sizeof(outputTest42),
                                                 &read));

    ASSERT_EQ(elements, app.inport[5]->bytesRead / sizeof(app.inport[5]->buffer[0]));

    // Topic check. 
    for(unsigned i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[5]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[5]->buffer)[i]);
    }

    ASSERT_EQ(inputTest1.value, outputTest41.value);
    ASSERT_EQ(0u, outputTest42.value);

    // USE CASE 5: Publisher and subscriber have two attributes (with keys 1 and 2). 
    elements = 128u;

    // publisher 5 (index=6)
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[6]->portHandle,
                                             app.outport[6]->buffer,
                                             elements * sizeof(app.outport[6]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[6]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                  &inputTest1,
                                                  sizeof(inputTest1)));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[6]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                  &inputTest2,
                                                  sizeof(inputTest2)));

    // Do the transfer
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[6]->portHandle,
                                                app.inport[6]->portHandle));

    // Check the data in subscriber. 
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[6]->portHandle,
                                            app.inport[6]->buffer,
                                            elements * sizeof(app.inport[6]->buffer[0]),
                                            &app.inport[6]->bytesRead));

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[6]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                 &outputTest51,
                                                 sizeof(outputTest51),
                                                 &read));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[6]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                 &outputTest52,
                                                 sizeof(outputTest52),
                                                 &read));

    ASSERT_EQ(elements, app.inport[6]->bytesRead / sizeof(app.inport[6]->buffer[0]));

    // Topic check. 
    for(unsigned i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[6]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[6]->buffer)[i]);
    }

    ASSERT_EQ(inputTest1.value, outputTest51.value);
    ASSERT_EQ(inputTest2.value, outputTest52.value);

    // USE CASE 6: Publisher has two attributes (with keys 1 and 2) and subscriber has two attributes (with keys 1 and 3). 
    elements = 256u;

    // publisher 6 (index=7)
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[7]->portHandle,
                                             app.outport[7]->buffer,
                                             elements * sizeof(app.outport[7]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[7]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                  &inputTest1,
                                                  sizeof(inputTest1)));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[7]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                  &inputTest2,
                                                  sizeof(inputTest2)));

    // Do the transfer
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[7]->portHandle,
                                                app.inport[7]->portHandle));

    // Check the data in subscriber. 
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[7]->portHandle,
                                            app.inport[7]->buffer,
                                            elements * sizeof(app.inport[7]->buffer[0]),
                                            &app.inport[7]->bytesRead));

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[7]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                 &outputTest61,
                                                 sizeof(outputTest61),
                                                 &read));
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE,
              xme_core_dataHandler_readAttribute(app.inport[7]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2),
                                                 &outputTest62,
                                                 sizeof(outputTest62),
                                                 &read));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[7]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST3),
                                                 &outputTest63,
                                                 sizeof(outputTest63),
                                                 &read));

    ASSERT_EQ(elements, app.inport[7]->bytesRead / sizeof(app.inport[7]->buffer[0]));

    // Topic check. 
    for(unsigned i = 0u; i < elements; ++i)
    {
        ASSERT_EQ(((xme_core_topic_t*)app.outport[7]->buffer)[i],
                  ((xme_core_topic_t*)app.inport[7]->buffer)[i]);
    }

    ASSERT_EQ(inputTest1.value, outputTest61.value);
    ASSERT_EQ(0, outputTest63.value);
}

TEST_F(DataHandlerIntegrationTest, simpleQueueTest)
{
    uint32_t i;
    uint32_t bufferSize;
    uint32_t elementSize;

    attribute_test1_t inputTest1 = { 50 };
    attribute_test1_t inputTest2 = { 4242 };
    attribute_test1_t outputTest1;
    attribute_test1_t outputTest2;

    uint32_t read = 0U;

    // Configure Broker to perform the transfer between outport[0] and inport[0] automatically
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(app.outport[0]->portHandle,
                                                         app.inport[0]->portHandle));

    // Write first data element

    bufferSize = app.outport[0]->bufferSize;
    elementSize = sizeof(*(app.outport[0]->buffer));

    (void) xme_hal_mem_set(app.outport[0]->buffer, 0x55, bufferSize * elementSize);

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             bufferSize * elementSize // FIXME: This is counter-intuitive!
                                             ));

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[0]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                  &inputTest1,
                                                  sizeof(inputTest1)));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

    // Write second data element

    (void) xme_hal_mem_set(app.outport[0]->buffer, 0xAA, app.outport[0]->bufferSize * sizeof(app.outport[0]->buffer));

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             bufferSize * elementSize // FIXME: This is counter-intuitive!
                                             ));

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeAttribute(app.outport[0]->portHandle,
                                                  XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                  &inputTest2,
                                                  sizeof(inputTest2)));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

    // Transfer implicitly done by Broker

    // Read the first data element back

    bufferSize = app.inport[0]->bufferSize;
    elementSize = sizeof(*(app.inport[0]->buffer));

    (void) xme_hal_mem_set(app.inport[0]->buffer, 0x11, bufferSize * elementSize);
    app.inport[0]->bytesRead = 0U;

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                            app.inport[0]->buffer,
                                            bufferSize * elementSize, // FIXME: This is counter-intuitive!
                                            &app.inport[0]->bytesRead));
    EXPECT_EQ(20U, app.inport[0]->bytesRead);

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[0]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                 &outputTest1,
                                                 sizeof(outputTest1),
                                                 &read));
    EXPECT_EQ(sizeof(outputTest1), read);

    for (i = 0; i < bufferSize * elementSize; i++)
    {
#if defined(USE_QUEUE)
        EXPECT_EQ(0x55U, ((unsigned char*) app.inport[0]->buffer)[i]);
#else
        EXPECT_EQ(0xAAU, ((unsigned char*) app.inport[0]->buffer)[i]);
#endif
    }

#if defined(USE_QUEUE)
    EXPECT_EQ(outputTest1.value, inputTest1.value);
#else
    EXPECT_EQ(outputTest1.value, inputTest2.value);
#endif

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));

    // Read the second data element back

    (void) xme_hal_mem_set(app.inport[0]->buffer, 0x44, bufferSize * elementSize);
    app.inport[0]->bytesRead = 0U;

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                            app.inport[0]->buffer,
                                            bufferSize * elementSize, // FIXME: This is counter-intuitive!
                                            &app.inport[0]->bytesRead));
    EXPECT_EQ(20u, app.inport[0]->bytesRead);

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readAttribute(app.inport[0]->portHandle,
                                                 XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                 &outputTest2,
                                                 sizeof(outputTest2),
                                                 &read));
    EXPECT_EQ(sizeof(outputTest2), read);

    for (i = 0; i < bufferSize * elementSize; i++)
    {
        EXPECT_EQ(0xAAU, ((unsigned char*) app.inport[0]->buffer)[i]);
    }

    EXPECT_EQ(outputTest2.value, inputTest2.value);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));
}

TEST_F(DataHandlerIntegrationTest, overflowQueueInOutputPortWithoutTransferTest)
{
    // No overflows should happen when excessive data packets are written to
    // an output port that has no input port connected

    uint32_t bufferSize = app.outport[0]->bufferSize;
    uint32_t elementSize = sizeof(*(app.outport[0]->buffer));

    (void) xme_hal_mem_set(app.outport[0]->buffer, 0x55, bufferSize * elementSize);

    // Write first data element

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             bufferSize * elementSize // FIXME: This is counter-intuitive!
                                             ));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

    // Write second data element (should still fit)

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             bufferSize * elementSize // FIXME: This is counter-intuitive!
                                             ));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

    // Write third data element (should still succeed, because data packets have been invalidated)

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             bufferSize * elementSize // FIXME: This is counter-intuitive!
                                             ));

    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));
}

TEST_F(DataHandlerIntegrationTest, overflowQueueInOutputAndInputPortWithTransferTest)
{
    // An overflow should happen when excessive data packets are written to
    // an output port that is connected to an input port with inappropriate
    // queue length. The overflow will happen in the destination port, not
    // in the source port.

    uint32_t i;
    uint32_t bufferSize;
    uint32_t elementSize;

    // Configure Broker to perform the transfer between outport[0] and inport[0] automatically
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_broker_addDataPacketTransferEntry(app.outport[0]->portHandle,
                                                         app.inport[0]->portHandle));

    // Write first data element

    bufferSize = app.outport[0]->bufferSize;
    elementSize = sizeof(*(app.outport[0]->buffer));

    (void) xme_hal_mem_set(app.outport[0]->buffer, 0x55, bufferSize * elementSize);

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             bufferSize * elementSize // FIXME: This is counter-intuitive!
                                             ));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

    // Write second data element (should still fit)

    (void) xme_hal_mem_set(app.outport[0]->buffer, 0x66, bufferSize * elementSize);

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             bufferSize * elementSize // FIXME: This is counter-intuitive!
                                             ));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

    // Write third data element (should still succeed, but will invalidate the oldest data packet (0x55))

    (void) xme_hal_mem_set(app.outport[0]->buffer, 0x77, bufferSize * elementSize);

    printf("The following warning is EXPECTED:\n");
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             bufferSize * elementSize // FIXME: This is counter-intuitive!
                                             ));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));

    // Transfer implicitly done by Broker

    // Read the first data element back

    bufferSize = app.inport[0]->bufferSize;
    elementSize = sizeof(*(app.inport[0]->buffer));

    (void) xme_hal_mem_set(app.inport[0]->buffer, 0x11, bufferSize * elementSize);
    app.inport[0]->bytesRead = 0U;

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                            app.inport[0]->buffer,
                                            bufferSize * elementSize, // FIXME: This is counter-intuitive!
                                            &app.inport[0]->bytesRead));
    EXPECT_EQ(20U, app.inport[0]->bytesRead);

    for (i = 0; i < bufferSize * elementSize; i++)
    {
#if defined(USE_QUEUE)
        EXPECT_EQ(0x66U, ((unsigned char*) app.inport[0]->buffer)[i]);
#else
        EXPECT_EQ(0xAAU, ((unsigned char*) app.inport[0]->buffer)[i]);
#endif
    }

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));
}

// TODO: Test with one output port connected to two input ports with different queue sizes, of which one overflows

TEST_F(DataHandlerIntegrationTest, CompleteTest)
{
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
