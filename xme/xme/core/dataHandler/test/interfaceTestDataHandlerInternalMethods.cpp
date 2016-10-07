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
 * $Id: interfaceTestDataHandlerInternalMethods.cpp 7664 2014-03-04 08:47:41Z geisinger $
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

#include "xme/defines.h"
#include "xme/core/component.h"
#include "xme/core/dataHandler/include/dataHandlerInternalTypes.h"
#include "xme/core/dataHandler/include/dataHandlerInternalMethods.h"
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

xme_core_attribute_descriptor_t internalTestMetadata_elements[2] = {
    {(xme_core_attribute_key_t) ATTRIBUTE_KEY_TEST1, (size_t) 4 },
    {(xme_core_attribute_key_t) ATTRIBUTE_KEY_TEST2, (size_t) 2 } };

xme_core_attribute_descriptor_list_t internalTestMetadata = { 2, internalTestMetadata_elements };

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
class DataHandlerInternalInterfaceSimpleTest : public ::testing::Test {
    protected:
        DataHandlerInternalInterfaceSimpleTest() :
                        componentID(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT),
                        type(XME_CORE_COMPONENT_PORTTYPE_INVALID),
                        topic(XME_CORE_TOPIC_INVALID_TOPIC),
                        bufferSize(0u), queueSize(0u), overwrite(false),
                        persistent(false), historyDepth(0),
                        portHandle(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID) {
            xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE);
        }

        virtual
        ~DataHandlerInternalInterfaceSimpleTest() {
            xme_core_dataHandler_fini();
        }

        xme_core_component_t componentID;
        xme_core_component_portType_t type;
        xme_core_topic_t topic;
        int bufferSize;
        int queueSize;bool overwrite;bool persistent;
        int historyDepth;
        xme_core_dataManager_dataPacketId_t portHandle;
};

class DataHandlerInternalInterfaceTest : public ::testing::Test {
    protected:
        DataHandlerInternalInterfaceTest() : app() {
            xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE);

            EXPECT_EQ(XME_STATUS_SUCCESS,
                      xme_core_dataHandler_createPort(app.identifier,
                                                      app.inport[0]->type,
                                                      app.inport[0]->topic,
                                                      app.inport[0]->bufferSize * sizeof(xme_core_topic_t),
                                                      internalTestMetadata, 1,
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
        ~DataHandlerInternalInterfaceTest() {
            xme_core_dataHandler_fini();
        }

        Application app;
};

TEST_F(DataHandlerInternalInterfaceSimpleTest, checkDataHandlerImageForConsistency) {
    xme_core_dataHandler_imageDataStructure_t image = NULL;
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_checkImageForConsistancy(image));
}

TEST_F(DataHandlerInternalInterfaceSimpleTest, createDataEntryWithoutInitializing) {
    xme_core_dataHandler_setup_t values;
    memset(&values, 0U, sizeof(values));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_createDataEntry(&values, &portHandle));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveInvalidState) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_INVALID));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveReadAuditElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_READ_AUDITELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveWriteAuditElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_WRITE_AUDITELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveReadSecurityAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_READ_SECURITYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveWriteSecurityAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(
                              app.inport[0]->portHandle, 0u, app.inport[0]->buffer,
                              sizeof(app.outport[0]->buffer[0]) * elements, &app.inport[0]->bytesRead,
                              DATAHANDLER_STATE_WRITE_SECURITYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveReadSafetyAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_READ_SAFETYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveWriteSafetyAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_WRITE_SAFETYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveWriteShadowData) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_WRITE_SHADOWELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveReadShadowAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_READ_SHADOWATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveWriteShadowAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveWriteShadowElementWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(
                              app.inport[0]->portHandle, 0u, app.inport[0]->buffer,
                              sizeof(app.outport[0]->buffer[0]) * elements, &app.inport[0]->bytesRead,
                              DATAHANDLER_STATE_WRITE_SHADOWELEMENT_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveReadShadowAttributeWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(
                              app.inport[0]->portHandle, 0u, app.inport[0]->buffer,
                              sizeof(app.outport[0]->buffer[0]) * elements, &app.inport[0]->bytesRead,
                              DATAHANDLER_STATE_READ_SHADOWATTRIBUTE_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveWriteShadowAttributeWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(
                              app.inport[0]->portHandle, 0u, app.inport[0]->buffer,
                              sizeof(app.outport[0]->buffer[0]) * elements, &app.inport[0]->bytesRead,
                              DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveWriteTopicElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_WRITE_TOPICELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveReadTopicAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_READ_TOPICATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveWriteTopicAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         DATAHANDLER_STATE_WRITE_TOPICATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readDataWithDirctiveCurrentlyUnknown) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readDataWithDirective(app.inport[0]->portHandle, 0u,
                                                         app.inport[0]->buffer,
                                                         sizeof(app.outport[0]->buffer[0]) * elements,
                                                         &app.inport[0]->bytesRead,
                                                         (xme_core_dataHandler_states_t ) 100));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirctiveInvalidState) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_INVALID));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveReadAuditElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_READ_AUDITELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveWriteAuditElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_WRITE_AUDITELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveReadSecurityAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_READ_SECURITYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveWriteSecurityAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead,
                              DATAHANDLER_STATE_WRITE_SECURITYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveReadSafetyAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_READ_SAFETYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveWriteSafetyAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_WRITE_SAFETYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveWriteShadowData) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_WRITE_SHADOWELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveReadShadowData) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_READ_SHADOWELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveWriteShadowAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveWriteShadowElementWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead,
                              DATAHANDLER_STATE_WRITE_SHADOWELEMENT_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveReadShadowElementWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead,
                              DATAHANDLER_STATE_READ_SHADOWELEMENT_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveWriteShadowAttributeWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead,
                              DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveReadeTopicElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_READ_TOPICELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveWriteTopicElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_WRITE_TOPICELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveWriteTopicAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, DATAHANDLER_STATE_WRITE_TOPICATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, readAttributesWithDirectiveCurrentlyUnknown) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_readAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              &app.inport[0]->bytesRead, (xme_core_dataHandler_states_t ) 100));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirctiveInvalidState) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_INVALID));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveReadAuditElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_READ_AUDITELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveWriteAuditElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_WRITE_AUDITELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveReadSecurityAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(
                              app.inport[0]->portHandle, 0u, app.inport[0]->buffer,
                              sizeof(app.outport[0]->buffer[0]) * elements, DATAHANDLER_STATE_READ_SECURITYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveWriteSecurityAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_writeDataWithDirective(
                              app.inport[0]->portHandle, 0u, app.inport[0]->buffer,
                              sizeof(app.outport[0]->buffer[0]) * elements, DATAHANDLER_STATE_WRITE_SECURITYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveReadSafetyAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_READ_SAFETYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveWriteSafetyAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_WRITE_SAFETYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveReadShadowData) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_READ_SHADOWELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveReadShadowAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_READ_SHADOWATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveWriteShadowAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveReadShadowElementWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(
                              app.inport[0]->portHandle, 0u, app.inport[0]->buffer,
                              sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_SHADOWELEMENT_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveReadShadowAttributeWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(
                              app.inport[0]->portHandle, 0u, app.inport[0]->buffer,
                              sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_SHADOWATTRIBUTE_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveWriteShadowAttributeWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(
                              app.inport[0]->portHandle, 0u, app.inport[0]->buffer,
                              sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveReadTopicElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_READ_TOPICELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveWriteTopicAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          DATAHANDLER_STATE_READ_TOPICATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeDataWithDirectiveCurrentlyUnknown) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeDataWithDirective(app.inport[0]->portHandle, 0u,
                                                          app.inport[0]->buffer,
                                                          sizeof(app.outport[0]->buffer[0]) * elements,
                                                          (xme_core_dataHandler_states_t ) 100));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirctiveInvalidState) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_INVALID));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveReadAuditElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_AUDITELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveWriteAuditElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_WRITE_AUDITELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveReadSecurityAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_SECURITYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveWriteSecurityAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_WRITE_SECURITYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveReadSafetyAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_SAFETYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveWriteSafetyAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_UNSUPPORTED,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_WRITE_SAFETYATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveReadShadowData) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_SHADOWELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveWriteShadowData) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_WRITE_SHADOWELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveReadShadowAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_SHADOWATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveReadShadowElementWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_SHADOWELEMENT_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveWriteShadowElementWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_WRITE_SHADOWELEMENT_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveReadShadowAttributeWithOffset) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_SHADOWATTRIBUTE_WITHOFFSET));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveReadTopicElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_TOPICELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveWriteTopicElement) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_WRITE_TOPICELEMENT));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveReadTopicAttribute) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              DATAHANDLER_STATE_READ_TOPICATTRIBUTE));
}

TEST_F(DataHandlerInternalInterfaceTest, writeAttributeWithDirectiveCurrentlyUnknown) {
    unsigned int elements = 0u;
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
              xme_core_dataHandler_writeAttributeWithDirective(
                              app.inport[0]->portHandle,
                              (xme_core_attribute_key_t ) ATTRIBUTE_KEY_TEST1, 0u,
                              app.inport[0]->buffer, sizeof(app.outport[0]->buffer[0]) * elements,
                              (xme_core_dataHandler_states_t ) 100));
}

TEST_F(DataHandlerInternalInterfaceTest, loadImage) {
    ASSERT_EQ(NULL, xme_core_dataHandler_loadImage());
}
