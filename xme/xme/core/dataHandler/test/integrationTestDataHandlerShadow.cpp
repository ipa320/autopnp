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
 * $Id: integrationTestDataHandlerShadow.cpp 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Data Handler Shadow test.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/component.h"

#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerTestsystemInterface.h"
#include "xme/core/dataHandler/include/dataHandlerRTEInterface.h"


#include "xme/hal/include/mem.h"

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

class Port {
    public:
        Port(xme_core_dataManager_dataPacketId_t anIdentifier,
             xme_core_component_portType_t aType = XME_CORE_COMPONENT_PORTTYPE_INVALID,
             xme_core_topic_t aTopic = XME_CORE_TOPIC_INVALID_TOPIC,
             xme_core_attribute_descriptor_list_t anAttribute = XME_CORE_NO_ATTRIBUTE,
             unsigned int aBufferSize = 0u) :
                        identifier(anIdentifier), type(aType), topic(aTopic), attribute(anAttribute),
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
        xme_core_attribute_descriptor_list_t attribute;
        xme_core_dataManager_dataPacketId_t portHandle;
        unsigned int bufferSize;
        unsigned int bytesRead;
        xme_core_topic_t* buffer;
};

class ApplicationShadow {
    public:
        ApplicationShadow(xme_core_component_t componentIdentifier) :
                        identifier(componentIdentifier) {}

        void createInputPort(xme_core_dataManager_dataPacketId_t id,
                               xme_core_component_portType_t type,
                               xme_core_topic_t topic,
                               xme_core_attribute_descriptor_list_t attribute,
                               unsigned int bufferSize,
                               unsigned int defaultValue = 0u){
            inport.push_back(createPort(id, type, topic, attribute, bufferSize, defaultValue));
        }

        void createOutputPort(xme_core_dataManager_dataPacketId_t id,
                               xme_core_component_portType_t type,
                               xme_core_topic_t topic,
                               xme_core_attribute_descriptor_list_t attribute,
                               unsigned int bufferSize,
                               unsigned int defaultValue = 0u){
            outport.push_back(createPort(id, type, topic, attribute, bufferSize, defaultValue));
        }

        virtual
        ~ApplicationShadow() {
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

    private:
        Port* createPort(xme_core_dataManager_dataPacketId_t id,
                          xme_core_component_portType_t type,
                          xme_core_topic_t topic,
                          xme_core_attribute_descriptor_list_t attribute,
                          unsigned int bufferSize,
                          unsigned int defaultValue = 0u){
            Port *port = new Port(id, type, topic, attribute, bufferSize);

            for (unsigned int i = 0u; i < bufferSize; ++i) {
                ((xme_core_topic_t*) port->buffer)[i] = (xme_core_topic_t) defaultValue;
            }
            return port;
        }
};

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class DataHandlerShadowIntegrationTest : public ::testing::Test {
    protected:
        DataHandlerShadowIntegrationTest() {
            if (XME_STATUS_SUCCESS != xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE)) {
                throw;
            }
        }

        void addApplicationShadow(ApplicationShadow *app) {
            applications.push_back(app);
        }

        void createAllPorts(){
            /* first all input ports */
            for (unsigned int i = 0u; i < applications.size(); ++i) {
                for (unsigned int j = 0u; j < applications[i]->inport.size(); ++j) {
                    // FIXME: here we need a possibility to call either createPort or createRTE...
                    EXPECT_EQ(XME_STATUS_SUCCESS,
                              xme_core_dataHandler_createPort(applications[i]->identifier,
                                                              applications[i]->inport[j]->type,
                                                              applications[i]->inport[j]->topic,
                                                              applications[i]->inport[j]->bufferSize * sizeof(xme_core_topic_t),
                                                              applications[i]->inport[j]->attribute, 1, false, true, 0,
                                                              &applications[i]->inport[j]->portHandle));
                }
            }

            /* then all output ports */
            for (unsigned int i = 0u; i < applications.size(); ++i) {
                for (unsigned int j = 0u; j < applications[i]->outport.size(); ++j) {
                    EXPECT_EQ(XME_STATUS_SUCCESS,
                              xme_core_dataHandler_createPort(applications[i]->identifier,
                                                              applications[i]->outport[j]->type,
                                                              applications[i]->outport[j]->topic,
                                                              applications[i]->outport[j]->bufferSize * sizeof(xme_core_topic_t),
                                                              applications[i]->outport[j]->attribute, 1, false, true, 0,
                                                              &applications[i]->outport[j]->portHandle));

                }
            }
        }

        virtual
        ~DataHandlerShadowIntegrationTest() {
            xme_core_dataHandler_fini();
        }

        std::vector<ApplicationShadow*> applications;
};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

TEST_F(DataHandlerShadowIntegrationTest, writeElementsAndReadWithOffset)
{
    srand((unsigned int)time(NULL));
    ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());

    appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0,
                           XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, (xme_core_topic_t) 1,
                           XME_CORE_NO_ATTRIBUTE, 2 /* has 2 data elements */);

    addApplicationShadow(appA);
    // setUp our system
    createAllPorts();

    //set test data
    xme_core_topic_t normalData1[2];   //normal data of appA->port0 (has 2 data elements)
    normalData1[0] = (xme_core_topic_t) 0xda7a001a;
    normalData1[1] = (xme_core_topic_t) 0xda7a001b;

    //write data to output ports of AppA
    appA->outport[0]->buffer[0] = normalData1[0];
    appA->outport[0]->buffer[1] = normalData1[1];

    //write data into internal database
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeData(appA->outport[0]->portHandle,
                                                   appA->outport[0]->buffer,
                                                   2 * sizeof(appA->outport[0]->buffer[0])));
    //reset buffer
    xme_hal_mem_set(appA->outport[0]->buffer, 0, 2 * sizeof(appA->outport[0]->buffer[0]));

    //read data from SHADOW database, written data to internal database should be read because normal database is mirrored to shadow
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readDataWithOffset(appA->outport[0]->portHandle, sizeof(appA->outport[0]->buffer[0]),
                                                            appA->outport[0]->buffer,
                                                            1 * sizeof(appA->outport[0]->buffer[0]),
                                                            &appA->outport[0]->bytesRead));

    // test the result
    EXPECT_EQ(normalData1[1], appA->outport[0]->buffer[0]);
    delete appA;
}

TEST_F(DataHandlerShadowIntegrationTest, writeElementsAndReadWithOffsetNearBufferBoarder)
{
    srand((unsigned int)time(NULL));
    ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());

    appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0,
                           XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, (xme_core_topic_t) 1,
                           XME_CORE_NO_ATTRIBUTE, 2 /* has 2 data elements */);

    addApplicationShadow(appA);
    // setUp our system
    createAllPorts();

    //set test data
    xme_core_topic_t normalData1[2];   //normal data of appA->port0 (has 2 data elements)
    normalData1[0] = (xme_core_topic_t) 0xda7a001a;
    normalData1[1] = (xme_core_topic_t) 0x0000001a;

    //write data to output ports of AppA
    appA->outport[0]->buffer[0] = normalData1[0];
    appA->outport[0]->buffer[1] = normalData1[1];

    //write data into internal database
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeData(appA->outport[0]->portHandle,
                                                   appA->outport[0]->buffer,
                                                   2 * sizeof(appA->outport[0]->buffer[0])));
    //reset buffer
    xme_hal_mem_set(appA->outport[0]->buffer, 0, 2 * sizeof(appA->outport[0]->buffer[0]));

    //read data from SHADOW database, written data to internal database should be read because normal database is mirrored to shadow
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readDataWithOffset(appA->outport[0]->portHandle, sizeof(appA->outport[0]->buffer[0]),
                                                            appA->outport[0]->buffer,
                                                            1 * sizeof(appA->outport[0]->buffer[0]),
                                                            &appA->outport[0]->bytesRead));

    // test the result
    EXPECT_EQ(normalData1[1], appA->outport[0]->buffer[0]);
    delete appA;
}

/* FIXME: Issue 2930: This fails due to checksum vertification error! */
TEST_F(DataHandlerShadowIntegrationTest, DISABLED_writeElementsWithOffsetAndReadAll)
{
    srand((unsigned int)time(NULL));
    ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());

    appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0,
                           XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, (xme_core_topic_t) 1,
                           XME_CORE_NO_ATTRIBUTE, 2 /* has 2 data elements */);

    addApplicationShadow(appA);
    // setUp our system
    createAllPorts();

    //set test data
    xme_core_topic_t normalData1[2];   //normal data of appA->port0 (has 2 data elements)
    ((xme_core_topic_t*) normalData1)[0] = (xme_core_topic_t) 0xda7a001a;
    ((xme_core_topic_t*) normalData1)[1] = (xme_core_topic_t) 0xda7a001b;

    //write data to output ports of AppA
    appA->outport[0]->buffer[0] = (normalData1)[0];

    //write data into internal database
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeDataWithOffset(appA->outport[0]->portHandle, 4,
                                                             appA->outport[0]->buffer,
                                                             1 * sizeof(appA->outport[0]->buffer[0])));

    //reset buffer
    xme_hal_mem_set(appA->outport[0]->buffer, 0, 2 * sizeof(appA->outport[0]->buffer[0]));

    //read data from SHADOW database, written data to internal database should be read because normal database is mirrored to shadow
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readData(appA->outport[0]->portHandle,
                                                  appA->outport[0]->buffer, 2 * sizeof(appA->outport[0]->buffer[0]),
                                                  &appA->outport[0]->bytesRead));

    EXPECT_EQ(((xme_core_topic_t* ) normalData1)[0],
              ((xme_core_topic_t* )appA->outport[0]->buffer)[1]);
    delete appA;
}

/* FIXME: Issue 2930: This fails due to checksum vertification error! */
TEST_F(DataHandlerShadowIntegrationTest, DISABLED_writeElementsWithOffsetAndReadAllNearBufferBoarder)
{
    srand((unsigned int)time(NULL));
    ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());

    appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0,
                           XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, (xme_core_topic_t) 1,
                           XME_CORE_NO_ATTRIBUTE, 2 /* has 2 data elements */);

    addApplicationShadow(appA);
    // setUp our system
    createAllPorts();

    //set test data
    xme_core_topic_t normalData1[2];   //normal data of appA->port0 (has 2 data elements)
    ((xme_core_topic_t*) normalData1)[0] = (xme_core_topic_t) 0xda7a001a;
    ((xme_core_topic_t*) normalData1)[1] = (xme_core_topic_t) 0x1a000000;

    //write data to output ports of AppA
    appA->outport[0]->buffer[0] = (normalData1)[0];

    //write data into internal database
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeDataWithOffset(appA->outport[0]->portHandle, 7, appA->outport[0]->buffer,
                                             1 * sizeof(appA->outport[0]->buffer[0])));

    //reset buffer
    xme_hal_mem_set(appA->outport[0]->buffer, 0, 2 * sizeof(appA->outport[0]->buffer[0]));

    //read data from SHADOW database, written data to internal database should be read because normal database is mirrored to shadow
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readData(appA->outport[0]->portHandle,
                                                  appA->outport[0]->buffer, 2 * sizeof(appA->outport[0]->buffer[0]),
                                                  &appA->outport[0]->bytesRead));

    EXPECT_EQ(((xme_core_topic_t*) normalData1)[1], ((xme_core_topic_t* )appA->outport[0]->buffer)[1]);
    delete appA;
}

TEST_F(DataHandlerShadowIntegrationTest, writeAttributesAndReadWithOffset)
{
    srand((unsigned int)time(NULL));
    ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());

    //set up an attribute (metadata)
    xme_core_attribute_descriptor_t metadata_elements[2];
    metadata_elements[0].key = (xme_core_attribute_key_t) 1;
    metadata_elements[0].size = (size_t) 4;
    metadata_elements[1].key = (xme_core_attribute_key_t) 2;
    metadata_elements[1].size = (size_t) 2;

    xme_core_attribute_descriptor_list_t metadata;
    metadata.element = metadata_elements;
    metadata.length = 2;

    appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0,
                           XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, (xme_core_topic_t) 1,
                           metadata, 2 /* has 2 data elements */);

    addApplicationShadow(appA);
    // setUp our system
    createAllPorts();

    //set test attributes
    uint8_t normalAttributeElement1[4] = { 0x61, 0x66, 0x66, 0x65 };
    uint8_t attributeReadBuffer[8];

    //write data into internal database
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeAttribute(appA->outport[0]->portHandle,
                                                        (xme_core_attribute_key_t ) 1,
                                                        (void* ) normalAttributeElement1, 4u));

    //reset buffer
    xme_hal_mem_set(attributeReadBuffer, 0, 8);

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readAttributeWithOffset(appA->outport[0]->portHandle,
                                                                 (xme_core_attribute_key_t ) 1,
                                                                 2,
                                                                 (void* ) attributeReadBuffer, 4u,
                                                                 &appA->outport[0]->bytesRead));

    EXPECT_EQ(normalAttributeElement1[2], attributeReadBuffer[0]);
    EXPECT_EQ(normalAttributeElement1[3], attributeReadBuffer[1]);
    EXPECT_EQ(0x00, attributeReadBuffer[2]);
    EXPECT_EQ(0x00, attributeReadBuffer[3]);

    delete appA;
}

TEST_F(DataHandlerShadowIntegrationTest, writeAttributesAndReadWithOffsetNearBufferBoarder)
{
    srand((unsigned int)time(NULL));
    ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());

    //set up an attribute (metadata)
    xme_core_attribute_descriptor_t metadata_elements[2];
    metadata_elements[0].key = (xme_core_attribute_key_t) 1;
    metadata_elements[0].size = (size_t) 4;
    metadata_elements[1].key = (xme_core_attribute_key_t) 2;
    metadata_elements[1].size = (size_t) 2;

    xme_core_attribute_descriptor_list_t metadata;
    metadata.element = metadata_elements;
    metadata.length = 2;

    appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0,
                           XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, (xme_core_topic_t) 1,
                           metadata, 2 /* has 2 data elements */);

    addApplicationShadow(appA);
    // setUp our system
    createAllPorts();

    //set test attributes
    uint8_t normalAttributeElement1[4] = { 0x61, 0x66, 0x66, 0x65 };
    uint8_t attributeReadBuffer[8];

    //write data into internal database
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeAttribute(appA->outport[0]->portHandle,
                                                        (xme_core_attribute_key_t ) 1,
                                                        (void* ) normalAttributeElement1, 4u));

    //reset buffer
    xme_hal_mem_set(attributeReadBuffer, 0, 8);

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readAttributeWithOffset(appA->outport[0]->portHandle,
                                                                 (xme_core_attribute_key_t ) 1,
                                                                 3,
                                                                 (void* ) attributeReadBuffer, 4u,
                                                                 &appA->outport[0]->bytesRead));

    EXPECT_EQ(normalAttributeElement1[3], attributeReadBuffer[0]);
    EXPECT_EQ(0x00, attributeReadBuffer[1]);
    EXPECT_EQ(0x00, attributeReadBuffer[2]);
    EXPECT_EQ(0x00, attributeReadBuffer[3]);

    delete appA;
}

/* FIXME: Issue 2930: This fails due to checksum vertification error! */
TEST_F(DataHandlerShadowIntegrationTest, DISABLED_writeAttributeWithOffsetAndReadAll)
{
    srand((unsigned int)time(NULL));
    ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());

    //set up an attribute (metadata)
    xme_core_attribute_descriptor_t metadata_elements[2];
    metadata_elements[0].key = (xme_core_attribute_key_t) 1;
    metadata_elements[0].size = (size_t) 4;
    metadata_elements[1].key = (xme_core_attribute_key_t) 2;
    metadata_elements[1].size = (size_t) 2;
    xme_core_attribute_descriptor_list_t metadata;
    metadata.element = metadata_elements;
    metadata.length = 2;

    appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0,
                           XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, (xme_core_topic_t) 1,
                           metadata, 2 /* has 2 data elements */);

    addApplicationShadow(appA);
    // setUp our system
    createAllPorts();

    //set test attributes
    uint8_t normalAttributeElement1[4] = { 0x61, 0x66, 0x66, 0x65 };
    uint8_t attributeReadBuffer[8];

    //write data into internal database
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeAttributeWithOffset(appA->outport[0]->portHandle,
                                                        (xme_core_attribute_key_t ) 1, 2,
                                                        (void* ) normalAttributeElement1, 4u));

    //reset buffer
    xme_hal_mem_set(attributeReadBuffer, 0, 8);

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readAttribute(appA->outport[0]->portHandle,
                                                       (xme_core_attribute_key_t ) 1,
                                                       (void* ) attributeReadBuffer, 4u,
                                                       &appA->outport[0]->bytesRead));

    EXPECT_EQ(0x00, attributeReadBuffer[0]);
    EXPECT_EQ(0x00, attributeReadBuffer[1]);
    EXPECT_EQ(normalAttributeElement1[0], attributeReadBuffer[2]);
    EXPECT_EQ(normalAttributeElement1[1], attributeReadBuffer[3]);

    delete appA;
}

/* FIXME: Issue 2930: This fails due to checksum vertification error! */
TEST_F(DataHandlerShadowIntegrationTest, DISABLED_writeAttributeWithOffsetAndReadAllNearBufferBoarder)
{
    srand((unsigned int)time(NULL));
    ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());

    //set up an attribute (metadata)
    xme_core_attribute_descriptor_t metadata_elements[2];
    metadata_elements[0].key = (xme_core_attribute_key_t) 1;
    metadata_elements[0].size = (size_t) 4;
    metadata_elements[1].key = (xme_core_attribute_key_t) 2;
    metadata_elements[1].size = (size_t) 2;
    xme_core_attribute_descriptor_list_t metadata;
    metadata.element = metadata_elements;
    metadata.length = 2;

    appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0,
                           XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION, (xme_core_topic_t) 1,
                           metadata, 2 /* has 2 data elements */);

    addApplicationShadow(appA);
    // setUp our system
    createAllPorts();

    //set test attributes
    uint8_t normalAttributeElement1[4] = { 0x61, 0x66, 0x66, 0x65 };
    uint8_t attributeReadBuffer[8];

    //write data into internal database
    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeAttributeWithOffset(appA->outport[0]->portHandle,
                                                        (xme_core_attribute_key_t ) 1, 3,
                                                        (void* ) normalAttributeElement1, 4u));

    //reset buffer
    xme_hal_mem_set(attributeReadBuffer, 0, 8);

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readAttribute(appA->outport[0]->portHandle,
                                                       (xme_core_attribute_key_t ) 1,
                                                       (void* ) attributeReadBuffer, 4u,
                                                       &appA->outport[0]->bytesRead));

    EXPECT_EQ(0x00, attributeReadBuffer[0]);
    EXPECT_EQ(0x00, attributeReadBuffer[1]);
    EXPECT_EQ(0x00, attributeReadBuffer[2]);
    EXPECT_EQ(normalAttributeElement1[0], attributeReadBuffer[3]);

    delete appA;
}


TEST_F(DataHandlerShadowIntegrationTest, DISABLED_ManipulateDataInShadowDatabaseTest)
//void testShadowDatabaseTestsystemInterface(bool testRTEPorts)
{
  unsigned int bytesRead;

  srand((unsigned int)time(NULL));

  //set up an attribute (metadata)
    xme_core_attribute_descriptor_t metadata_elements[2];
    metadata_elements[0].key = (xme_core_attribute_key_t) 1;
    metadata_elements[0].size = (size_t) 4;
    metadata_elements[1].key = (xme_core_attribute_key_t) 2;
    metadata_elements[1].size = (size_t) 2;
    xme_core_attribute_descriptor_list_t metadata;
    metadata.element = metadata_elements;
    metadata.length = 2;

  /*
                     ########
   ########          # appB #
   #      #--T1[2]-->########
   # appA #
   #      #--T2[1]-->########
   ########  (attr)  # appC #
                     ########
  */

  // 1st application
  ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());
  appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0, XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                        (xme_core_topic_t) 1, XME_CORE_NO_ATTRIBUTE, 2 /* has 2 data elements */);
  /* appA->inportCount = 0;
     appA->outportCount = 2; */
  appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0, XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                        (xme_core_topic_t) 2, metadata, 1 /* has 1 data elements */);

  addApplicationShadow(appA);

  // 2nd application
  ApplicationShadow *appB = new ApplicationShadow((xme_core_component_t) rand());
  /*appB->inportCount = 1;
  appB->outportCount = 0;*/
  appB->createInputPort((xme_core_dataManager_dataPacketId_t) 0, XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                       (xme_core_topic_t) 1, XME_CORE_NO_ATTRIBUTE, 2);
  addApplicationShadow(appB);

  // 3rd application
  ApplicationShadow *appC = new ApplicationShadow((xme_core_component_t) rand());
  //appC->inportCount = 1;
  //appC->outportCount = 0;
  appC->createInputPort((xme_core_dataManager_dataPacketId_t) 0, XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                       (xme_core_topic_t) 2, metadata, 1);
  addApplicationShadow(appC);

  // setUp our system
  createAllPorts();

  //set test data
  xme_core_topic_t* normalData1 = new xme_core_topic_t[2];        //normal data of appA->port0 (has 2 data elements)
  xme_core_topic_t* normalData2 = new xme_core_topic_t[1];        //normal data of appA->port1 (has 1 data element)
  xme_core_topic_t* manipulateData1 = new xme_core_topic_t[1];    //manipulated data of appA->port0
  xme_core_topic_t* manipulateData2 = new xme_core_topic_t[1];    //manipulated data of appA->port1
  ((xme_core_topic_t*) normalData1)[0] = (xme_core_topic_t) 0xda7a001a;
  ((xme_core_topic_t*) normalData1)[1] = (xme_core_topic_t) 0xda7a001b;
  ((xme_core_topic_t*) normalData2)[0] = (xme_core_topic_t) 0xda7a002a;
  ((xme_core_topic_t*) manipulateData1)[0] = (xme_core_topic_t) 0x12341111;
  ((xme_core_topic_t*) manipulateData2)[0] = (xme_core_topic_t) 0x56782222;

  //set test attributes
  uint8_t normalAttributeElement1[4] = {0xab, 0xba, 0xba, 0xab};
  uint8_t normalAttributeElement2[2] = {0xef, 0xfe};
  uint8_t manipulatedAttributeElement1[4] = {0x12, 0x34, 0x56, 0x78};
  uint8_t manipulatedAttributeElement2[2] = {0x56, 0x44};
  uint8_t attributeReadBuffer[8];

  //write data to output ports of AppA
  appA->outport[0]->buffer[0] = (normalData1)[0];
  appA->outport[0]->buffer[1] = (normalData1)[1];
  appA->outport[1]->buffer[0] = (normalData2)[0];
  //set receiver ports to zero
  xme_hal_mem_set(appB->inport[0]->buffer, 0, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]));
  xme_hal_mem_set(appC->inport[0]->buffer, 0, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]));


  //write data into internal database
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(appA->outport[0]->portHandle, appA->outport[0]->buffer, 2 * sizeof(appA->outport[0]->buffer[0])));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(appA->outport[1]->portHandle, appA->outport[1]->buffer, 1 * sizeof(appA->outport[0]->buffer[0])));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(appA->outport[1]->portHandle, (xme_core_attribute_key_t) 1, (void*) normalAttributeElement1, 4u));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(appA->outport[1]->portHandle, (xme_core_attribute_key_t) 2, (void*) normalAttributeElement2, 2u));
  //transfer data to matching subscriber ports inside the internal database
  //if(!testRTEPorts) {
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[0]->portHandle, appB->inport[0]->portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[1]->portHandle, appC->inport[0]->portHandle));
  //}

  //read data from SHADOW database, written data to internal database should be read because normal database is mirrored to shadow
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ(2u, appB->inport[0]->bytesRead / sizeof(appB->inport[0]->buffer[0]));
  EXPECT_EQ(1u, appC->inport[0]->bytesRead / sizeof(appC->inport[0]->buffer[0]));
  EXPECT_EQ(appA->outport[0]->buffer[0], appB->inport[0]->buffer[0]);
  EXPECT_EQ(appA->outport[0]->buffer[1], appB->inport[0]->buffer[1]);
  EXPECT_EQ(appA->outport[1]->buffer[0], appC->inport[0]->buffer[0]);

  //read an attribute from INTERNAL database
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(normalAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(normalAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(normalAttributeElement1[3], attributeReadBuffer[3]);

  //read attributes from SHADOW database, written attributes to internal database should be read because normal database is mirrored to shadow
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(normalAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(normalAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(normalAttributeElement1[3], attributeReadBuffer[3]);
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 2, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(2u, bytesRead);
  EXPECT_EQ(normalAttributeElement2[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement2[1], attributeReadBuffer[1]);

  //manipulate data and attributes in shadow database  (but let shadow deactivated for now)
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_writeData(appA->outport[0]->portHandle, manipulateData1, 1 * sizeof(appB->inport[0]->buffer[0]) ));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_writeData(appA->outport[1]->portHandle, manipulateData2, 1 * sizeof(appB->inport[0]->buffer[0]) ));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_writeAttribute(appA->outport[1]->portHandle, (xme_core_attribute_key_t) 1, (void*) manipulatedAttributeElement1, 4u));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_writeAttribute(appA->outport[1]->portHandle, (xme_core_attribute_key_t) 2, (void*) manipulatedAttributeElement2, 2u));

  //transfer
  //if(!testRTEPorts) {
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[0]->portHandle, appB->inport[0]->portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[1]->portHandle, appC->inport[0]->portHandle));
  //}

  //read data from INTERNAL database (not manipulated, because shadow not active yet)
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));

  //1) compare if manipulated value is as expected and original value is untouched (because shadow is still inactive)
  //1.a) compare: Here old unmanipulated value should be read, because we read from normal database and shadow is still inactive
  EXPECT_EQ( normalData1[0],                     appB->inport[0]->buffer[0]);
  EXPECT_EQ( normalData1[1],                    appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);
  EXPECT_NE( normalData1[0],                     appB->inport[0]->buffer[1]);

  //read data from SHADOW database
  xme_hal_mem_set(appB->inport[0]->buffer, 0, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]));
  xme_hal_mem_set(appC->inport[0]->buffer, 0, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));

  //1.b) compare: Here the manipulated value should be read, because read from shadow is called (however, shadow is still inactive)
  EXPECT_EQ( manipulateData1[0],                appB->inport[0]->buffer[0]);
  EXPECT_EQ( (xme_core_topic_t) 0x00000000,     appB->inport[0]->buffer[1]);
  EXPECT_EQ( manipulateData2[0],                appC->inport[0]->buffer[0]);
  EXPECT_NE( manipulateData1[0],                appB->inport[0]->buffer[1]);

  //read an attribute from INTERNAL database
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(normalAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(normalAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(normalAttributeElement1[3], attributeReadBuffer[3]);

  //read attributes from SHADOW database, written attributes to internal database should be read because normal database is mirrored to shadow
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(manipulatedAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(manipulatedAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(manipulatedAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(manipulatedAttributeElement1[3], attributeReadBuffer[3]);
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 2, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(2u, bytesRead);
  EXPECT_EQ(normalAttributeElement2[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement2[1], attributeReadBuffer[1]);
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 2, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(2u, bytesRead);
  EXPECT_EQ(manipulatedAttributeElement2[0], attributeReadBuffer[0]);
  EXPECT_EQ(manipulatedAttributeElement2[1], attributeReadBuffer[1]);


  //activate COMPLETE SHADOW database (param=0 means forever until deactivate)
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_activateShadowDatabase());
  //we should not need another transfer here
  //read data over interface of internal database, however SHADOW data should be read now
  xme_hal_mem_set(appB->inport[0]->buffer, 0, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]));
  xme_hal_mem_set(appC->inport[0]->buffer, 0, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  //compare: Here the manipulated value should be read while reading from normal database, because shadow database is active now
  EXPECT_EQ( manipulateData1[0],                appB->inport[0]->buffer[0]);
  EXPECT_EQ( (xme_core_topic_t) 0x00000000,     appB->inport[0]->buffer[1]);
  EXPECT_EQ( manipulateData2[0],                appC->inport[0]->buffer[0]);
  EXPECT_NE( normalData1[0],                    appB->inport[0]->buffer[0]);
  EXPECT_NE( manipulateData1[0],                appB->inport[0]->buffer[1]);
  //read attributes over interface of internal database, however manipulated SHADOW attributes should be read now
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(manipulatedAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(manipulatedAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(manipulatedAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(manipulatedAttributeElement1[3], attributeReadBuffer[3]);
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 2, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(2u, bytesRead);
  EXPECT_EQ(manipulatedAttributeElement2[0], attributeReadBuffer[0]);
  EXPECT_EQ(manipulatedAttributeElement2[1], attributeReadBuffer[1]);


  //deactivate COMPLETE SHADOW database again
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_deactivateShadowDatabase());
  //read data over interface of internal database, now the original unmanipulated data should be read
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  //compare: Here the manipulated value should be read while reading from normal database, because shadow database is active now
  EXPECT_EQ( normalData1[0],                    appB->inport[0]->buffer[0]);
  EXPECT_EQ( normalData1[1],                    appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);
  EXPECT_NE( manipulateData1[0],                appB->inport[0]->buffer[0]);
  //read data from shadow database (still manipulated data should be read because no transfer up to now)
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ( manipulateData1[0],                appB->inport[0]->buffer[0]);
  EXPECT_EQ( (xme_core_topic_t) 0x00000000,        appB->inport[0]->buffer[1]);
  //transfer
  //if(!testRTEPorts) {
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[0]->portHandle, appB->inport[0]->portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[1]->portHandle, appC->inport[0]->portHandle));
  //}

  //read data from shadow database (now the manipulated data should be gone, because transfer was called)
  //TODO: not he case actually. Ask JHF to determine the desired behavior.
  /*
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ( normalData1[0],                    appB->inport[0]->buffer[0]);
  EXPECT_EQ( normalData1[1],                    appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);
  */

  //test manipulation of only a single shadow database element (appA->outport0 --> appB->inport0)
  xme_hal_mem_set(appB->inport[0]->buffer, 0, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]));
  xme_hal_mem_set(appC->inport[0]->buffer, 0, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]));
  //if(!testRTEPorts) {
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_activateShadowElement(appB->inport[0]->portHandle));
  //}

  //ok if a publication port is given
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_activateShadowElement(appA->outport[0]->portHandle));
  //transfer
  //if(!testRTEPorts) {
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[0]->portHandle, appB->inport[0]->portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[1]->portHandle, appC->inport[0]->portHandle));
  //}

  //read, appB should get manipulated data, appC should get normal data
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ( manipulateData1[0],                appB->inport[0]->buffer[0]);
  EXPECT_EQ( (xme_core_topic_t) 0x00000000,     appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);
  //deactivate again
  //if(!testRTEPorts) {
    EXPECT_EQ(XME_STATUS_SUCCESS,xme_core_dataHandlerShadow_deactivateShadowElement(appB->inport[0]->portHandle));
  //}
  EXPECT_EQ(XME_STATUS_SUCCESS,             xme_core_dataHandlerShadow_deactivateShadowElement(appA->outport[0]->portHandle));
  //transfer
  //if(!testRTEPorts) {
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[0]->portHandle, appB->inport[0]->portHandle));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_transferData(appA->outport[1]->portHandle, appC->inport[0]->portHandle));
  //}
  //read normal data now
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ( normalData1[0],                    appB->inport[0]->buffer[0]);
  EXPECT_EQ( normalData1[1],                     appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);

  //TODO: This test doesn't test if an activation of shadow is automatically deactivated after the given amount of cycles. For this, we would need some mock of the ExecutionManager.
}

// FIXME: to run this test we have to add a check to port creation.
#if 0
TEST_F(DataHandlerShadowIntegrationTest, ManipulateDataInShadowDatabaseRTEPortTest)
{
  unsigned int bytesRead;

  srand((unsigned int)time(NULL));

  //set up an attribute (metadata)
    xme_core_attribute_descriptor_t metadata_elements[2];
    metadata_elements[0].key = (xme_core_attribute_key_t) 1;
    metadata_elements[0].size = (size_t) 4;
    metadata_elements[1].key = (xme_core_attribute_key_t) 2;
    metadata_elements[1].size = (size_t) 2;
    xme_core_attribute_descriptor_list_t metadata;
    metadata.element = metadata_elements;
    metadata.length = 2;

  /*
                     ########
   ########          # appB #
   #      #--T1[2]-->########
   # appA #
   #      #--T2[1]-->########
   ########  (attr)  # appC #
                     ########
  */

  // 1st application
  ApplicationShadow *appA = new ApplicationShadow((xme_core_component_t) rand());
  appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0, XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                        (xme_core_topic_t) 1, XME_CORE_NO_ATTRIBUTE, 2 /* has 2 data elements */);
  /* appA->inportCount = 0;
     appA->outportCount = 2; */
  appA->createOutputPort((xme_core_dataManager_dataPacketId_t) 0, XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                        (xme_core_topic_t) 2, metadata, 1 /* has 1 data elements */);

  addApplicationShadow(appA);

  // 2nd application
  ApplicationShadow *appB = new ApplicationShadow((xme_core_component_t) rand());
  /*appB->inportCount = 1;
  appB->outportCount = 0;*/
  appB->createInputPort((xme_core_dataManager_dataPacketId_t) 0, XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                       (xme_core_topic_t) 1, XME_CORE_NO_ATTRIBUTE, 2);
  addApplicationShadow(appB);

  // 3rd application
  ApplicationShadow *appC = new ApplicationShadow((xme_core_component_t) rand());
  //appC->inportCount = 1;
  //appC->outportCount = 0;
  appC->createInputPort((xme_core_dataManager_dataPacketId_t) 0, XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                       (xme_core_topic_t) 2, metadata, 1);
  addApplicationShadow(appC);

  // setUp our system
  createAllPorts();

  //set test data
  xme_core_topic_t* normalData1 = new xme_core_topic_t[2];      //normal data of appA->port0 (has 2 data elements)
  xme_core_topic_t* normalData2 = new xme_core_topic_t[1];      //normal data of appA->port1 (has 1 data element)
  xme_core_topic_t* manipulateData1 = new xme_core_topic_t[1];  //manipulated data of appA->port0
  xme_core_topic_t* manipulateData2 = new xme_core_topic_t[1];  //manipulated data of appA->port1
  ((xme_core_topic_t*) normalData1)[0] = (xme_core_topic_t) 0xda7a001a;
  ((xme_core_topic_t*) normalData1)[1] = (xme_core_topic_t) 0xda7a001b;
  ((xme_core_topic_t*) normalData2)[0] = (xme_core_topic_t) 0xda7a002a;
  ((xme_core_topic_t*) manipulateData1)[0] = (xme_core_topic_t) 0x12341111;
  ((xme_core_topic_t*) manipulateData2)[0] = (xme_core_topic_t) 0x56782222;

  //set test attributes
  uint8_t normalAttributeElement1[4] = {0xab, 0xba, 0xba, 0xab};
  uint8_t normalAttributeElement2[2] = {0xef, 0xfe};
  uint8_t manipulatedAttributeElement1[4] = {0x12, 0x34, 0x56, 0x78};
  uint8_t manipulatedAttributeElement2[2] = {0x56, 0x44};
  uint8_t attributeReadBuffer[8];

  //write data to output ports of AppA
  appA->outport[0]->buffer[0] = (normalData1)[0];
  appA->outport[0]->buffer[1] = (normalData1)[1];
  appA->outport[1]->buffer[0] = (normalData2)[0];
  //set receiver ports to zero
  xme_hal_mem_set(appB->inport[0]->buffer, 0, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]));
  xme_hal_mem_set(appC->inport[0]->buffer, 0, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]));


  //write data into internal database
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(appA->outport[0]->portHandle, appA->outport[0]->buffer, 2 * sizeof(appA->outport[0]->buffer[0])));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeData(appA->outport[1]->portHandle, appA->outport[1]->buffer, 1 * sizeof(appA->outport[0]->buffer[0])));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(appA->outport[1]->portHandle, (xme_core_attribute_key_t) 1, (void*) normalAttributeElement1, 4u));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_writeAttribute(appA->outport[1]->portHandle, (xme_core_attribute_key_t) 2, (void*) normalAttributeElement2, 2u));

  //read data from SHADOW database, written data to internal database should be read because normal database is mirrored to shadow
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ(2u, appB->inport[0]->bytesRead / sizeof(appB->inport[0]->buffer[0]));
  EXPECT_EQ(1u, appC->inport[0]->bytesRead / sizeof(appC->inport[0]->buffer[0]));
  EXPECT_EQ(appA->outport[0]->buffer[0], appB->inport[0]->buffer[0]);
  EXPECT_EQ(appA->outport[0]->buffer[1], appB->inport[0]->buffer[1]);
  EXPECT_EQ(appA->outport[1]->buffer[0], appC->inport[0]->buffer[0]);

  //read an attribute from INTERNAL database
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(normalAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(normalAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(normalAttributeElement1[3], attributeReadBuffer[3]);

  //read attributes from SHADOW database, written attributes to internal database should be read because normal database is mirrored to shadow
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(normalAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(normalAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(normalAttributeElement1[3], attributeReadBuffer[3]);
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 2, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(2u, bytesRead);
  EXPECT_EQ(normalAttributeElement2[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement2[1], attributeReadBuffer[1]);

  //manipulate data and attributes in shadow database  (but let shadow deactivated for now)
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_writeData(appA->outport[0]->portHandle, manipulateData1, 1 * sizeof(appA->outport[0]->buffer[0]) ));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_writeData(appA->outport[1]->portHandle, manipulateData2, 1 * sizeof(appA->outport[0]->buffer[0]) ));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_writeAttribute(appA->outport[1]->portHandle, (xme_core_attribute_key_t) 1, (void*) manipulatedAttributeElement1, 4u));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_writeAttribute(appA->outport[1]->portHandle, (xme_core_attribute_key_t) 2, (void*) manipulatedAttributeElement2, 2u));


  //read data from INTERNAL database (not manipulated, because shadow not active yet)
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));

  //1) compare if manipulated value is as expected and original value is untouched (because shadow is still inactive)
  //1.a) compare: Here old unmanipulated value should be read, because we read from normal database and shadow is still inactive
  EXPECT_EQ( normalData1[0],                    appB->inport[0]->buffer[0]);
  EXPECT_EQ( normalData1[1],                    appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);
  EXPECT_NE( normalData1[0],                    appB->inport[0]->buffer[1]);

  //read data from SHADOW database
  xme_hal_mem_set(appB->inport[0]->buffer, 0, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]));
  xme_hal_mem_set(appC->inport[0]->buffer, 0, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));

  //1.b) compare: Here the manipulated value should be read, because read from shadow is called (however, shadow is still inactive)
  EXPECT_EQ( manipulateData1[0],                appB->inport[0]->buffer[0]);
  EXPECT_EQ( (xme_core_topic_t) 0x00000000,     appB->inport[0]->buffer[1]);
  EXPECT_EQ( manipulateData2[0],                appC->inport[0]->buffer[0]);
  EXPECT_NE( manipulateData1[0],                appB->inport[0]->buffer[1]);

  //read an attribute from INTERNAL database
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(normalAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(normalAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(normalAttributeElement1[3], attributeReadBuffer[3]);

  //read attributes from SHADOW database, written attributes to internal database should be read because normal database is mirrored to shadow
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(manipulatedAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(manipulatedAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(manipulatedAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(manipulatedAttributeElement1[3], attributeReadBuffer[3]);
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 2, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(2u, bytesRead);
  EXPECT_EQ(normalAttributeElement2[0], attributeReadBuffer[0]);
  EXPECT_EQ(normalAttributeElement2[1], attributeReadBuffer[1]);
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 2, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(2u, bytesRead);
  EXPECT_EQ(manipulatedAttributeElement2[0], attributeReadBuffer[0]);
  EXPECT_EQ(manipulatedAttributeElement2[1], attributeReadBuffer[1]);


  //activate COMPLETE SHADOW database (param=0 means forever until deactivate)
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_activateShadowDatabase(0u));
  //we should not need another transfer here
  //read data over interface of internal database, however SHADOW data should be read now
  xme_hal_mem_set(appB->inport[0]->buffer, 0, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]));
  xme_hal_mem_set(appC->inport[0]->buffer, 0, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  //compare: Here the manipulated value should be read while reading from normal database, because shadow database is active now
  EXPECT_EQ( manipulateData1[0],                appB->inport[0]->buffer[0]);
  EXPECT_EQ( (xme_core_topic_t) 0x00000000,     appB->inport[0]->buffer[1]);
  EXPECT_EQ( manipulateData2[0],                appC->inport[0]->buffer[0]);
  EXPECT_NE( normalData1[0],                    appB->inport[0]->buffer[0]);
  EXPECT_NE( manipulateData1[0],                appB->inport[0]->buffer[1]);
  //read attributes over interface of internal database, however manipulated SHADOW attributes should be read now
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 1, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(4u, bytesRead);
  EXPECT_EQ(manipulatedAttributeElement1[0], attributeReadBuffer[0]);
  EXPECT_EQ(manipulatedAttributeElement1[1], attributeReadBuffer[1]);
  EXPECT_EQ(manipulatedAttributeElement1[2], attributeReadBuffer[2]);
  EXPECT_EQ(manipulatedAttributeElement1[3], attributeReadBuffer[3]);
  xme_hal_mem_set(attributeReadBuffer, 0, 8);
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readAttribute(appC->inport[0]->portHandle, (xme_core_attribute_key_t) 2, (void*) attributeReadBuffer, 8u, &bytesRead));
  EXPECT_EQ(2u, bytesRead);
  EXPECT_EQ(manipulatedAttributeElement2[0], attributeReadBuffer[0]);
  EXPECT_EQ(manipulatedAttributeElement2[1], attributeReadBuffer[1]);


  //deactivate COMPLETE SHADOW database again
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_deactivateShadowDatabase());
  //read data over interface of internal database, now the original unmanipulated data should be read
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  //compare: Here the manipulated value should be read while reading from normal database, because shadow database is active now
  EXPECT_EQ( normalData1[0],                    appB->inport[0]->buffer[0]);
  EXPECT_EQ( normalData1[1],                    appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);
  EXPECT_NE( manipulateData1[0],                appB->inport[0]->buffer[0]);
  //read data from shadow database (still manipulated data should be read because no transfer up to now)
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ( manipulateData1[0],                appB->inport[0]->buffer[0]);
  EXPECT_EQ( (xme_core_topic_t) 0x00000000,     appB->inport[0]->buffer[1]);

  //read data from shadow database (now the manipulated data should be gone, because transfer was called)
  //TODO: not he case actually. Ask JHF to determine the desired behavior.
  /*
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ( normalData1[0],                    appB->inport[0]->buffer[0]);
  EXPECT_EQ( normalData1[1],                    appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);
  */

  //test manipulation of only a single shadow database element (appA->outport0 --> appB->inport0)
  xme_hal_mem_set(appB->inport[0]->buffer, 0, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]));
  xme_hal_mem_set(appC->inport[0]->buffer, 0, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]));

  //ok if a publication port is given
  EXPECT_EQ(XME_STATUS_SUCCESS,             xme_core_dataHandlerShadow_activateShadowElement(appA->outport[0]->portHandle ,0u));

  //read, appB should get manipulated data, appC should get normal data
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ( manipulateData1[0],                appB->inport[0]->buffer[0]);
  EXPECT_EQ( (xme_core_topic_t) 0x00000000,     appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);

  //deactivate again
  EXPECT_EQ(XME_STATUS_SUCCESS,             xme_core_dataHandlerShadow_deactivateShadowElement(appA->outport[0]->portHandle));

//read normal data now
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appB->inport[0]->portHandle, appB->inport[0]->buffer, appB->inport[0]->bufferSize * sizeof(appB->inport[0]->buffer[0]), &appB->inport[0]->bytesRead));
  EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_readData(appC->inport[0]->portHandle, appC->inport[0]->buffer, appC->inport[0]->bufferSize * sizeof(appC->inport[0]->buffer[0]), &appC->inport[0]->bytesRead));
  EXPECT_EQ( normalData1[0],                    appB->inport[0]->buffer[0]);
  EXPECT_EQ( normalData1[1],                    appB->inport[0]->buffer[1]);
  EXPECT_EQ( normalData2[0],                    appC->inport[0]->buffer[0]);

  //TODO: This test doesn't test if an activation of shadow is automatically deactivated after the given amount of cycles. For this, we would need some mock of the ExecutionManager.
}
#endif
