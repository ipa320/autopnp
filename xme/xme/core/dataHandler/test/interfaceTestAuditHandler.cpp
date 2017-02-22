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
 * $Id: interfaceTestAuditHandler.cpp 7664 2014-03-04 08:47:41Z geisinger $
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
#include "xme/core/dataHandler/include/auditHandler.h"
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

xme_core_attribute_descriptor_t auditTestMetadata_elements[2] = {
    {(xme_core_attribute_key_t) ATTRIBUTE_KEY_TEST1, (size_t) 4 },
    {(xme_core_attribute_key_t) ATTRIBUTE_KEY_TEST2, (size_t) 2 } };

xme_core_attribute_descriptor_list_t auditTestMetadata = { 2, auditTestMetadata_elements };

class Port {
    public:
        Port(xme_core_dataManager_dataPacketId_t anIdentifier,
             xme_core_component_portType_t aType = XME_CORE_COMPONENT_PORTTYPE_INVALID,
             xme_core_topic_t aTopic = XME_CORE_TOPIC_INVALID_TOPIC,
             unsigned int aBufferSize = 0u) :
                        portHandle(anIdentifier), type(aType), topic(aTopic),
                        bufferSize(aBufferSize), bytesRead(0u), buffer(NULL) {
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

class AuditHandlerInterfaceTest : public ::testing::Test {
    protected:
        AuditHandlerInterfaceTest() : app() {
            xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE);

            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.identifier,
                                                      app.inport[0]->type,
                                                      app.inport[0]->topic,
                                                      app.inport[0]->bufferSize * sizeof(xme_core_topic_t),
                                                      auditTestMetadata, 1,
                                                      false, false, 0,
                                                      &app.inport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.identifier,
                                                      app.inport[1]->type,
                                                      app.inport[1]->topic,
                                                      app.inport[1]->bufferSize * sizeof(xme_core_topic_t),
                                                      XME_CORE_NO_ATTRIBUTE, 1,
                                                      false, false, 0,
                                                      &app.inport[1]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.identifier,
                                                      app.outport[0]->type,
                                                      app.outport[0]->topic,
                                                      app.outport[0]->bufferSize * sizeof(xme_core_topic_t),
                                                      XME_CORE_NO_ATTRIBUTE, 1,
                                                      false, false, 0,
                                                      &app.outport[0]->portHandle));
            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.identifier,
                                                      app.outport[1]->type,
                                                      app.outport[1]->topic,
                                                      app.outport[1]->bufferSize * sizeof(xme_core_topic_t),
                                                      XME_CORE_NO_ATTRIBUTE, 1,
                                                      false, false, 0,
                                                      &app.outport[1]->portHandle));
        }

        virtual
        ~AuditHandlerInterfaceTest() {
            xme_core_dataHandler_fini();
        }

        Application app;
};

TEST_F(AuditHandlerInterfaceTest, dumpDataHandler) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_dataHandlerDump());
}

TEST_F(AuditHandlerInterfaceTest, generateAuditEntry) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_generateAuditEntry());
}

TEST_F(AuditHandlerInterfaceTest, signalAlarm) {
    ASSERT_EQ(XME_STATUS_SUCCESS,
              xme_core_dataHandler_signalAlarm());
}
