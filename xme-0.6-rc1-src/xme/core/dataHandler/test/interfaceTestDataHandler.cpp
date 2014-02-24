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
 * $Id: interfaceTestDataHandler.cpp 5123 2013-09-19 14:30:10Z camek $
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
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/hal/include/mem.h"

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

xme_core_dataManager_dataPacketId_t xme_core_exec_CycleCounter;

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
/***   Static variables                                                     ***/
/******************************************************************************/
#define DATAHANDLER_TEST_MEMORYVALUE 13

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class DataHandlerInterfaceSimpleTest : public ::testing::Test {
    protected:
        DataHandlerInterfaceSimpleTest() :
                        componentID(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT), type(
                                        XME_CORE_COMPONENT_PORTTYPE_INVALID), topic(
                                        XME_CORE_TOPIC_INVALID_TOPIC), bufferSize(0u), queueSize(
                                        0u), overwrite(false), persistent(false), historyDepth(0), portHandle(
                                        XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) {
            xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE);
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
};

class DataHandlerInterfaceTest : public ::testing::Test {
    protected:
        DataHandlerInterfaceTest() :
                        app() {
            xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE);

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
        }

        virtual
        ~DataHandlerInterfaceTest() {
            xme_core_dataHandler_fini();
        }

        Application app;
};

typedef DataHandlerInterfaceSimpleTest DataHandlerInterfaceSimpleTestDeathTest;
typedef DataHandlerInterfaceTest DataHandlerInterfaceTestDeathTest;

TEST_F(DataHandlerInterfaceSimpleTest, createPortWithInvalidValues) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_createPort(componentID, type, topic, bufferSize,
                                              XME_CORE_NO_ATTRIBUTE, queueSize, overwrite,
                                              persistent, historyDepth, &portHandle));
}

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
    unsigned int elements = 0u;
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(
                    xme_core_dataHandler_writeData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                                   sizeof(app.outport[0]->buffer[0]) * elements),
                    "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                                   sizeof(app.outport[0]->buffer[0]) * elements),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataOfZeroElementsToOutputPort) {
    unsigned int elements = 0u;
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                                   app.outport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements),
                    "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
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
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL,
                                                  0u, NULL),
                    "");
}

TEST_F(DataHandlerInterfaceTest, readDataFromANonReadablePort) {
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED,
              xme_core_dataHandler_readData(app.outport[0]->portHandle, app.outport[0]->buffer,
                                            app.outport[0]->bufferSize,
                                            &app.outport[0]->bytesRead));
}

TEST_F(DataHandlerInterfaceTestDeathTest, readDataFromAPortButWriteInANULLBuffer) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readData(app.inport[0]->portHandle, NULL,
                                                  app.outport[0]->bufferSize,
                                                  &app.outport[0]->bytesRead),
                    "");
}

TEST_F(DataHandlerInterfaceTest, readDataFromAPortButBufferSizeIsZero) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle, app.inport[0]->buffer, 0u,
                                            &app.inport[0]->bytesRead));
}

TEST_F(DataHandlerInterfaceTestDeathTest, readDataFromAPortButWrittenBytesWillBeStoredInNull) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                                  app.inport[0]->bufferSize, NULL),
                    "");
}

TEST_F(DataHandlerInterfaceTest, readDataFromAPortAndStoreDataInABuffer) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_readData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                            app.inport[0]->bufferSize, &app.inport[0]->bytesRead));
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeWithInvalidAttributes) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                       XME_CORE_ATTRIBUTE_KEY_UNDEFINED, NULL, 0u,
                                                       NULL),
                    "");
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeFromAnInvalidPort) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readAttribute ( XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer,
                                                         app.inport[0]->bufferSize,
                                                         &app.inport[0]->bytesRead ),
                    "");
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeWithInvalidKey) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(
                    xme_core_dataHandler_readAttribute(app.inport[0]->portHandle,
                                                       XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
                                                       app.inport[0]->buffer,
                                                       app.inport[0]->bufferSize,
                                                       &app.inport[0]->bytesRead),
                    "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readAttribute(app.inport[0]->portHandle,
                                                       XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
                                                       app.inport[0]->buffer,
                                                       app.inport[0]->bufferSize,
                                                       &app.inport[0]->bytesRead),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeIntoANULLBuffer) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         NULL,
                                                         app.inport[0]->bufferSize,
                                                         &app.inport[0]->bytesRead ),
                    "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         NULL,
                                                         app.inport[0]->bufferSize,
                                                         &app.inport[0]->bytesRead ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeWithZeroBufferSize) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer, 0u,
                                                         &app.inport[0]->bytesRead ),
                    "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer, 0u,
                                                         &app.inport[0]->bytesRead ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, readAttributeWillSaveReadSizeInNULL) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer,
                                                         app.inport[0]->bufferSize, NULL ),
                    "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                         XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                         app.inport[0]->buffer,
                                                         app.inport[0]->bufferSize, NULL ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, readAttributeFromPublisherPort) {
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED,
              xme_core_dataHandler_readAttribute ( app.outport[0]->portHandle,
                                                   XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                   app.outport[0]->buffer,
                                                   app.outport[0]->bufferSize,
                                                   &app.outport[0]->bytesRead ));
}

TEST_F(DataHandlerInterfaceTest, readAttributeFromValidPortButNoAttributesAreStored) {
    ASSERT_EQ(XME_STATUS_NO_SUCH_VALUE,
              xme_core_dataHandler_readAttribute ( app.inport[0]->portHandle,
                                                   XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                   app.inport[0]->buffer,
                                                   app.inport[0]->bufferSize,
                                                   &app.inport[0]->bytesRead ));
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataWithInvalidValues) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, NULL,
                                                   0u),
                    "");
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataIntoInvalidPort) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeData(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                   app.outport[0]->buffer,
                                                   app.outport[0]->bufferSize),
                    "");
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataFromAnInvalidBuffer) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle, NULL,
                                                   app.outport[0]->bufferSize),
                    "");
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeDataOfZeroSize) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                                   app.outport[0]->buffer, 0u),
                    "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeData(app.outport[0]->portHandle,
                                                   app.outport[0]->buffer, 0u),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, writeDataIntoASubscriberPort) {
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED,
              xme_core_dataHandler_writeData(app.inport[0]->portHandle, app.inport[0]->buffer,
                                             app.inport[0]->bufferSize));
}

TEST_F(DataHandlerInterfaceTest, writeDataToAValidPort) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_writeData(app.outport[0]->portHandle, app.outport[0]->buffer,
                                             app.outport[0]->bufferSize));
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeWithInvalidValues) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                        XME_CORE_ATTRIBUTE_KEY_UNDEFINED, NULL, 0u),
                    "");
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeWithInvalidPort) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeAttribute ( XME_CORE_DATAMANAGER_DATAPACKETID_INVALID,
                                                          XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1),
                                                          app.outport[0]->buffer,
                                                          app.outport[0]->bufferSize ),
                    "");
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeWithInvalidKey) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(
                        xme_core_dataHandler_writeAttribute(app.outport[0]->portHandle,
                                                            XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
                                                            app.outport[0]->buffer,
                                                            app.outport[0]->bufferSize),
                        "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeAttribute(app.outport[0]->portHandle,
                                                        XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
                                                        app.outport[0]->buffer,
                                                        app.outport[0]->bufferSize),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeIntoNULLBuffer) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(xme_core_dataHandler_writeAttribute ( app.outport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), NULL, app.outport[0]->bufferSize ),
                       "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeAttribute ( app.outport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), NULL, app.outport[0]->bufferSize ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTestDeathTest, writeAttributeWithZeroBufferSize) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(
                        xme_core_dataHandler_writeAttribute ( app.outport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), app.outport[0]->buffer, 0u ),
                        "");
#else
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeAttribute ( app.outport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), app.outport[0]->buffer, 0u ),
                    "");
#endif
}

TEST_F(DataHandlerInterfaceTest, writeAttributeIntoASubscriberPort) {
    ASSERT_EQ(XME_STATUS_PERMISSION_DENIED,
              xme_core_dataHandler_writeAttribute ( app.inport[0]->portHandle, XME_CORE_ATTRIBUTES(ATTRIBUTE_KEY_TEST1), app.inport[0]->buffer, app.inport[0]->bufferSize ));
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

TEST_F(DataHandlerInterfaceTest, completeReadOperationWithInvalidValues) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_completeReadOperation(
                              XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DataHandlerInterfaceTest, completeReadOperationPortWithInvalidComponent) {
    ASSERT_EQ(XME_STATUS_NOT_FOUND,   // This is allowed, because internal components are marked as invalid, FIXMEs
              xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));
}

TEST_F(DataHandlerInterfaceTest, prepareInputPortWithSubscriberPort) {
//FIXME: HERE WE HAVE TO MOCK THE EXECUTION MANAGER
    //    ASSERT_EQ(XME_STATUS_SUCCESS,
//              xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));
}

TEST_F(DataHandlerInterfaceTest, prepareInputPortWithPublisherPort) {
    //FIXME: HERE WE HAVE TO MOCK THE EXECUTION MANAGER
//    ASSERT_EQ(XME_STATUS_SUCCESS,
//              xme_core_dataHandler_completeReadOperation(app.inport[0]->portHandle));
}

TEST_F(DataHandlerInterfaceTest, completeWriteOperationPortWithInvalidPort) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_completeWriteOperation(
                              XME_CORE_DATAMANAGER_DATAPACKETID_INVALID));
}

TEST_F(DataHandlerInterfaceTest, completeWriteOperationPortWithSubscriberPort) {
    ASSERT_EQ(XME_STATUS_NOT_FOUND,   // This is given by broker, because port is not found
              xme_core_dataHandler_completeWriteOperation(app.inport[0]->portHandle));
}

TEST_F(DataHandlerInterfaceTest, completeWriteOperationPortWithPublisherPort) {
    ASSERT_EQ(XME_STATUS_NOT_FOUND,   // This is given by broker, because port is not found
              xme_core_dataHandler_completeWriteOperation(app.outport[0]->portHandle));
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
