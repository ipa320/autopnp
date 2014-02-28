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
 * $Id: integrationTestDataHandler.cpp 5123 2013-09-19 14:30:10Z camek $
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

#include "xme/core/dataManagerTypes.h"
#include "xme/core/dataHandler/include/dataHandler.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define DATAHANDLER_TEST_MEMORYVALUE 15

typedef enum
{
    ATTRIBUTE_KEY_UNDEFINED = XME_CORE_ATTRIBUTE_KEY_USER,
    ATTRIBUTE_KEY_TEST1,
    ATTRIBUTE_KEY_TEST2
} test_attributes_t;

typedef struct attribute_test1
{
    unsigned int value;
}attribute_test1_t;

typedef struct attribute_test2
{
    double value;
}attribute_test2_t;

xme_core_attribute_descriptor_t attributeTest1[] = {{XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),sizeof(attribute_test1_t)}};
xme_core_attribute_descriptor_t attributeTest2[] = {{XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST2), sizeof(attribute_test2_t)}};
xme_core_attribute_descriptor_list_t descriptorTest1 = {sizeof(attributeTest1)/sizeof(attributeTest1[0]), attributeTest1};
xme_core_attribute_descriptor_list_t descriptorTest2 = {sizeof(attributeTest2)/sizeof(attributeTest2[0]), attributeTest2};
xme_core_dataManager_dataPacketId_t xme_core_exec_CycleCounter;

class Port {
    public:
        Port(xme_core_dataManager_dataPacketId_t anIdentifier,
             xme_core_component_portType_t aType = XME_CORE_COMPONENT_PORTTYPE_INVALID,
             xme_core_topic_t aTopic = XME_CORE_TOPIC_INVALID_TOPIC,
             unsigned int aBufferSize = 0u) :
                        identifier(anIdentifier), type(aType), topic(aTopic),
                        portHandle(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID),
                        bufferSize(aBufferSize), bytesRead(0u), buffer(NULL) {
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
                                    (xme_core_topic_t) 2, 100u);

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
/***   Static variables                                                     ***/
/******************************************************************************/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class DataHandlerIntegrationTest : public ::testing::Test {
    protected:
        DataHandlerIntegrationTest() : app() {
            if(XME_STATUS_SUCCESS == xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE))
            {

            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.identifier,
                                                      app.inport[0]->type,
                                                      app.inport[0]->topic,
                                                      app.inport[0]->bufferSize * sizeof(xme_core_topic_t),
                                                      descriptorTest1, 1,
                                                      false, true, 0,
                                                      &app.inport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.identifier,
                                                      app.inport[1]->type,
                                                      app.inport[1]->topic,
                                                      app.inport[1]->bufferSize * sizeof(xme_core_topic_t),
                                                      descriptorTest2, 1,
                                                      false, true, 0,
                                                      &app.inport[1]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.identifier,
                                                      app.outport[0]->type,
                                                      app.outport[0]->topic,
                                                      app.outport[0]->bufferSize * sizeof(xme_core_topic_t),
                                                      descriptorTest1, 1,
                                                      false, true, 0,
                                                      &app.outport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.identifier,
                                                      app.outport[1]->type,
                                                      app.outport[1]->topic,
                                                      app.outport[1]->bufferSize * sizeof(xme_core_topic_t),
                                                      descriptorTest2, 1,
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
        }

        Application app;
};

TEST_F(DataHandlerIntegrationTest, writeTenElementsTransferItAndReadAsMuchPossible)
{
    unsigned int elements = 10u;
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             elements * sizeof(app.outport[0]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[0]->portHandle,
                                                app.inport[0]->portHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                            app.inport[0]->buffer,
                                            app.inport[0]->bufferSize * sizeof(app.inport[0]->buffer[0]),
                                            &app.inport[0]->bytesRead));
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
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                             app.outport[0]->buffer,
                                             elements * sizeof(app.outport[0]->buffer[0])));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[0]->portHandle,
                                                app.inport[0]->portHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                            app.inport[0]->buffer,
                                            app.inport[0]->bufferSize * sizeof(app.inport[0]->buffer[0]),
                                            &app.inport[0]->bytesRead));
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
  unsigned int elements;

  // init test layout
  for(unsigned i = 0u; i < app.outport[0]->bufferSize; ++i)
  {
    if (i%2==0)    ((xme_core_topic_t*)app.outport[0]->buffer)[i] = (xme_core_topic_t) 0xdeadbeef;
    else        ((xme_core_topic_t*)app.outport[0]->buffer)[i] = (xme_core_topic_t) 0x3ace3ace;
  }

  // write 10 elements to dataHandler, transfer it, and read 10 elements back
  elements = 10u;
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                           app.outport[0]->buffer,
                                           elements * sizeof(app.outport[0]->buffer[0])));
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_transferData(app.outport[0]->portHandle,
                                              app.inport[0]->portHandle));
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                          app.inport[0]->buffer,
                                          elements * sizeof(app.inport[0]->buffer[0]),
                                          &app.inport[0]->bytesRead));
  ASSERT_EQ(elements, app.inport[0]->bytesRead / sizeof(app.inport[0]->buffer[0]));
  for(unsigned i = 0u; i < elements; ++i)
  {
      ASSERT_EQ(((xme_core_topic_t*)app.outport[0]->buffer)[i],
                ((xme_core_topic_t*)app.inport[0]->buffer)[i]);
  }
}

TEST_F(DataHandlerIntegrationTest, writeFiveElementsTransferAndReadFiveElementsBack)
{
  unsigned elements = 5u;
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                           app.outport[0]->buffer,
                                           elements * sizeof(app.outport[0]->buffer[0])));
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_transferData(app.outport[0]->portHandle,
                                              app.inport[0]->portHandle));
  ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_dataHandler_readData(app.inport[0]->portHandle,
                                          app.inport[0]->buffer,
                                          elements * sizeof(app.inport[0]->buffer[0]),
                                          &app.inport[0]->bytesRead));

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

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[0]->portHandle,
                                                app.inport[0]->portHandle));
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_transferData(app.outport[1]->portHandle,
                                                app.inport[1]->portHandle));

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

TEST_F(DataHandlerIntegrationTest, CompleteTest)
{
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
