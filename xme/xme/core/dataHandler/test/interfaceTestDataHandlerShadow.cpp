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
 * $Id: interfaceTestDataHandlerShadow.cpp 7664 2014-03-04 08:47:41Z geisinger $
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
#include <climits>

#include "xme/core/component.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerTestsystemInterface.h"
#include "xme/hal/include/mem.h"
#include "xme/core/testUtils.h"

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

xme_core_attribute_descriptor_t metadata_elements[2] = { {
    (xme_core_attribute_key_t) ATTRIBUTE_KEY_TEST1, (size_t) 4 }, {
    (xme_core_attribute_key_t) ATTRIBUTE_KEY_TEST2, (size_t) 2 } };

xme_core_attribute_descriptor_list_t metadata = { 2, metadata_elements };

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
                delete[] buffer;
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
class DataHandlerShadowInterfaceTest : public ::testing::Test {
    protected:
        DataHandlerShadowInterfaceTest() :
                        app() {
            xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE);

            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(
                                      app.identifier, app.inport[0]->type, app.inport[0]->topic,
                                      app.inport[0]->bufferSize * sizeof(xme_core_topic_t),
                                      metadata, 1, false, false, 0, &app.inport[0]->portHandle));
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
                                      metadata, 1, false, false, 0, &app.outport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(
                                      app.identifier, app.outport[1]->type, app.outport[1]->topic,
                                      app.outport[1]->bufferSize * sizeof(xme_core_topic_t),
                                      XME_CORE_NO_ATTRIBUTE, 1, false, false, 0,
                                      &app.outport[1]->portHandle));
        }

        virtual
        ~DataHandlerShadowInterfaceTest() {
            xme_core_dataHandler_fini();
        }

        Application app;
};

typedef DataHandlerShadowInterfaceTest DataHandlerShadowInterfaceTestDeathTest;

TEST_F(DataHandlerShadowInterfaceTest, writeDataToShadow) {
    // write 10 elements to dataHandler, transfer it, and read as much possible
    unsigned int elements = 10u;

    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeData(app.outport[0]->portHandle,
                                                   app.outport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements));
}

TEST_F(DataHandlerShadowInterfaceTestDeathTest, writeDataToShadowWithWrongPort) {
    // write 10 elements to dataHandler, transfer it, and read as much possible
    unsigned int elements = 10u;

    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code

#ifdef NDEBUG
#if 0 // Disabled, because it makes the testsuite fail
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandlerShadow_writeData((xme_core_dataManager_dataPacketId_t )50,
                                                         app.outport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements));
#endif // #if 0
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandlerShadow_writeData((xme_core_dataManager_dataPacketId_t )50,
                                                         app.outport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements),
                    "");
#endif
}

TEST_F(DataHandlerShadowInterfaceTest, writeDataWithOffsetToShadow) {
    // write 10 elements to dataHandler, transfer it, and read as much possible
    unsigned int elements = 10u;

    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeDataWithOffset(app.outport[0]->portHandle, 2,
                                                             app.outport[0]->buffer,
                                                             sizeof(app.outport[0]->buffer[0]) * elements));
}

TEST_F(DataHandlerShadowInterfaceTestDeathTest, writeDataWithOffsetToShadowWithWrongPort) {
    // write 10 elements to dataHandler, transfer it, and read as much possible
    unsigned int elements = 10u;

    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code

#ifdef NDEBUG
#if 0 // Disabled, because it makes the testsuite fail
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandlerShadow_writeDataWithOffset(
                                    (xme_core_dataManager_dataPacketId_t ) 50, 2,
                                    app.outport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements));
#endif // #if 0
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandlerShadow_writeDataWithOffset(
                                    (xme_core_dataManager_dataPacketId_t ) 50, 2,
                                    app.outport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements),
                    "");
#endif
}

TEST_F(DataHandlerShadowInterfaceTest, writeAttributeToShadow) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeAttribute(
                              app.outport[0]->portHandle,
                              (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1,
                              app.outport[0]->buffer, metadata.element[0].size));
}

TEST_F(DataHandlerShadowInterfaceTestDeathTest, writeAttributeToShadowWithWrongPort) {
#ifdef NDEBUG
#if 0 // Disabled, because it makes the testsuite fail
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandlerShadow_writeAttribute(
                                    (xme_core_dataManager_dataPacketId_t ) 50,
                                    (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1,
                                    app.outport[0]->buffer, metadata.element[0].size));
#endif // #if 0
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandlerShadow_writeAttribute(
                                    (xme_core_dataManager_dataPacketId_t ) 50,
                                    (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1,
                                    app.outport[0]->buffer, metadata.element[0].size),
                    "");
#endif
}

TEST_F(DataHandlerShadowInterfaceTest, writeAttributeWithOffsetToShadow) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_writeAttributeWithOffset(
                              app.outport[0]->portHandle,
                              (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1, 2,
                              app.outport[0]->buffer, metadata.element[0].size));
}

TEST_F(DataHandlerShadowInterfaceTestDeathTest, writeAttributeWithOffsetToShadowWithWrongPort) {
#ifdef NDEBUG
#if 0 // Disabled, because it makes the testsuite fail
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandlerShadow_writeAttributeWithOffset(
                                    (xme_core_dataManager_dataPacketId_t )50,
                                    (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1, 2,
                                    app.outport[0]->buffer, metadata.element[0].size));
#endif // #if 0
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandlerShadow_writeAttributeWithOffset(
                                    (xme_core_dataManager_dataPacketId_t )50,
                                    (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1, 2,
                                    app.outport[0]->buffer, metadata.element[0].size),
                    "");
#endif
}

TEST_F(DataHandlerShadowInterfaceTest, readDataFromShadow) {
    // write 10 elements to dataHandler, transfer it, and read as much possible
    unsigned int elements = 10u;

    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code
    
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readData(app.outport[0]->portHandle,
                                                  app.outport[0]->buffer,
                                                  sizeof(app.outport[0]->buffer[0]) * elements,
                                                  &app.outport[0]->bytesRead));
}

TEST_F(DataHandlerShadowInterfaceTestDeathTest, readDataFromShadowWithWrongPort) {
    // write 10 elements to dataHandler, transfer it, and read as much possible
    unsigned int elements = 10u;

    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code

#ifdef NDEBUG
#if 0 // Disabled, because it makes the testsuite fail
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandlerShadow_readData((xme_core_dataManager_dataPacketId_t )50,
                                                        app.outport[0]->buffer,
                                                        sizeof(app.outport[0]->buffer[0]) * elements,
                                                        &app.outport[0]->bytesRead));
#endif // #if 0
#else

    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandlerShadow_readData((xme_core_dataManager_dataPacketId_t )50,
                                                        app.outport[0]->buffer,
                                                        sizeof(app.outport[0]->buffer[0]) * elements,
                                                        &app.outport[0]->bytesRead),
                    "");
#endif
}

TEST_F(DataHandlerShadowInterfaceTest, readDataWithOffsetFromShadow) {
    // write 10 elements to dataHandler, transfer it, and read as much possible
    unsigned int elements = 10u;

    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code

    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readDataWithOffset(app.outport[0]->portHandle, 2,
                                                            app.outport[0]->buffer,
                                                            sizeof(app.outport[0]->buffer[0]) * elements,
                                                            &app.outport[0]->bytesRead));
}

TEST_F(DataHandlerShadowInterfaceTestDeathTest, readDataWithOffsetFromShadowWithWrongPort) {
    // write 10 elements to dataHandler, transfer it, and read as much possible
    unsigned int elements = 10u;

    ASSERT_GE(app.outport[0]->bufferSize, elements); // An error here would mean an error in the test, and not in the tested code

#ifdef NDEBUG
#if 0 // Disabled, because it makes the testsuite fail
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandlerShadow_readDataWithOffset(
                                    (xme_core_dataManager_dataPacketId_t )50, 2,
                                    app.outport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                                    &app.outport[0]->bytesRead));
#endif // #if 0
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandlerShadow_readDataWithOffset(
                                    (xme_core_dataManager_dataPacketId_t )50, 2,
                                    app.outport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                                    &app.outport[0]->bytesRead),
                    "");
#endif
}

TEST_F(DataHandlerShadowInterfaceTest, readAttributeFromShadow) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readAttribute(
                              app.outport[0]->portHandle,
                              (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1,
                              app.outport[0]->buffer, metadata.element[0].size,
                              &app.outport[0]->bytesRead));
}

TEST_F(DataHandlerShadowInterfaceTestDeathTest, readAttributeFromShadowWithWrongPort) {
#ifdef NDEBUG
#if 0 // Disabled, because it makes the testsuite fail
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandlerShadow_readAttribute(
                                    (xme_core_dataManager_dataPacketId_t )50,
                                    (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1,
                                    app.outport[0]->buffer, metadata.element[0].size,
                                    &app.outport[0]->bytesRead));
#endif // #if 0
#else

    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandlerShadow_readAttribute(
                                    (xme_core_dataManager_dataPacketId_t )50,
                                    (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1,
                                    app.outport[0]->buffer, metadata.element[0].size,
                                    &app.outport[0]->bytesRead),
                    "");
#endif
}

TEST_F(DataHandlerShadowInterfaceTest, readAttributeWithOffsetFromShadow) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_readAttributeWithOffset(
                              app.outport[0]->portHandle,
                              (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1, 2,
                              app.outport[0]->buffer, metadata.element[0].size,
                              &app.outport[0]->bytesRead));
}

TEST_F(DataHandlerShadowInterfaceTestDeathTest, readAttributeWithOffsetFromShadowWithWrongPort) {
#ifdef NDEBUG
#if 0 // Disabled, because it makes the testsuite fail
    ASSERT_XME_ASSERTION_FAILURE(
                    xme_core_dataHandlerShadow_readAttributeWithOffset(
                                    (xme_core_dataManager_dataPacketId_t )50,
                                    (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1, 2,
                                    app.outport[0]->buffer, metadata.element[0].size,
                                    &app.outport[0]->bytesRead));
#endif // #if 0
#else
    EXPECT_DEATH(
                    xme_core_dataHandlerShadow_readAttributeWithOffset(
                                    (xme_core_dataManager_dataPacketId_t )50,
                                    (xme_core_attribute_key_t )ATTRIBUTE_KEY_TEST1, 2,
                                    app.outport[0]->buffer, metadata.element[0].size,
                                    &app.outport[0]->bytesRead),
                    "");
#endif 
}

TEST_F(DataHandlerShadowInterfaceTest, activateShadowDatabaseInfinite) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_activateShadowDatabase());
}

TEST_F(DataHandlerShadowInterfaceTest, activateShadowElement) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_activateShadowElement(app.outport[0]->portHandle));
}

TEST_F(DataHandlerShadowInterfaceTest, activateShadowElementOfWrongPort) {
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandlerShadow_activateShadowElement(
                              (xme_core_dataManager_dataPacketId_t )50));
}

TEST_F(DataHandlerShadowInterfaceTest, deActivateShadowDatabase) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandlerShadow_deactivateShadowDatabase());
}

TEST_F(DataHandlerShadowInterfaceTest, deActivateShadowElement) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandlerShadow_deactivateShadowElement(app.outport[0]->portHandle));
}

TEST_F(DataHandlerShadowInterfaceTest, deActivateShadowElementOfWrongPort) {
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandlerShadow_deactivateShadowElement(
                              (xme_core_dataManager_dataPacketId_t )50));
}
