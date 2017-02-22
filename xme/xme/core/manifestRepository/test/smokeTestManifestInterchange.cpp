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
 * $Id: smokeTestManifestInterchange.cpp 7778 2014-03-11 18:44:41Z geisinger $
 */

/**
 * \file
 *         Manifest interchange smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/manifestRepository/include/manifestInterchange.h"

#include "xme/core/directory/include/attribute.h"
#include "xme/core/directory/include/attributeInternalMethods.h"
#include "xme/core/log.h"
#include "xme/core/testUtils.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"
#include "xme/hal/include/xml.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class ManifestInterchangeSmokeTest: public xme::testing::Test
{
protected:
    struct visitorInfo_s
    {
        xme_core_componentManifest_t componentManifest;
        unsigned int currentFunctionIndex;
        unsigned int currentPortIndex;
    };

    ManifestInterchangeSmokeTest()
    {
    }

    virtual ~ManifestInterchangeSmokeTest()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_init());
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_directory_attribute_fini();
    }
    
    static char
    enterElementCallback
    (
        const xme_hal_xml_elementHandle_t element,
        const xme_hal_xml_attributeHandle_t firstAttribute,
        void* userData
    )
    {
        xme_hal_xml_attributeHandle_t attribute;
        const char* elementName = xme_hal_xml_getElementName(element);
        visitorInfo_s* visitorInfo = (visitorInfo_s*) userData;

        printf("Element %s enter!\n", elementName);

        for (attribute = firstAttribute; XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute; attribute = xme_hal_xml_getNextAttribute(attribute))
        {
            printf
            (
                " - Attribute %s = %s\n",
                xme_hal_xml_getAttributeName(attribute), xme_hal_xml_getAttributeValue(attribute)
            );
        }

        if (0 == strcmp("component", elementName))
        {
            for (attribute = firstAttribute; XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute; attribute = xme_hal_xml_getNextAttribute(attribute))
            {
                const char* attributeName = xme_hal_xml_getAttributeName(attribute);

                if (0 == strcmp("componentType", attributeName))
                {
                    unsigned int componentType = XME_CORE_COMPONENT_TYPE_INVALID;
                    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_xml_getAttributeValueAsUnsignedInt(attribute, &componentType));
                    EXPECT_NE(XME_CORE_COMPONENT_TYPE_INVALID, (xme_core_componentType_t) componentType);
                    visitorInfo->componentManifest.componentType = (xme_core_componentType_t) componentType;
                }
                else if (0 == strcmp("componentName", attributeName))
                {
                    const char* componentName = xme_hal_xml_getAttributeValue(attribute);
                    EXPECT_TRUE(NULL != componentName);
                    EXPECT_NE(0U, strlen(componentName));
                    (void) xme_hal_safeString_strncpy(visitorInfo->componentManifest.name, componentName, sizeof(visitorInfo->componentManifest.name));
                }
                else if (0 == strcmp("componentID", attributeName))
                {
                    // Not yet implemented
                    //EXPECT_TRUE(false);
                }
                else if (0 == strcmp("componentInitCB", attributeName))
                {
                    // Not yet implemented
                    //EXPECT_TRUE(false);
                }
                else if (0 == strcmp("componentFiniCB", attributeName))
                {
                    // Not yet implemented
                    //EXPECT_TRUE(false);
                }
                else if (0 == strcmp("componentInitializationString", attributeName))
                {
                    // Not yet implemented
                    //EXPECT_TRUE(false);
                }
                else if (0 == strcmp("componentConstraint", attributeName))
                {
                    // Not yet implemented
                    //EXPECT_TRUE(false);
                }
            }
        }
        else if (0 == strcmp("function", elementName))
        {
            // Get the next free function manifest
            xme_core_functionManifest_t* functionManifest;
            size_t maxFunctions = sizeof(visitorInfo->componentManifest.functionManifests) / sizeof(visitorInfo->componentManifest.functionManifests[0]);
            visitorInfo->currentFunctionIndex++;

            EXPECT_LE(visitorInfo->currentFunctionIndex, maxFunctions);
            //XME_CHECK_MSG(visitorInfo->currentFunctionIndex < maxFunctions, 1, XME_LOG_WARNING, "Too many functions defined in manifest!\n");

            functionManifest = &visitorInfo->componentManifest.functionManifests[visitorInfo->currentFunctionIndex - 1U];

            for (attribute = firstAttribute; XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute; attribute = xme_hal_xml_getNextAttribute(attribute))
            {
                const char* attributeName = xme_hal_xml_getAttributeName(attribute);

                if (0 == strcmp("functionID", attributeName))
                {
                    unsigned int functionID = XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT;
                    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_xml_getAttributeValueAsUnsignedInt(attribute, &functionID));
                    EXPECT_NE(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, (xme_core_component_functionId_t) functionID);
                    functionManifest->functionId = (xme_core_component_functionId_t) functionID;
                }
                else if (0 == strcmp("functionWCETInNS", attributeName))
                {
                    uint64_t functionWCETInNS = 0U;
                    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_xml_getAttributeValueAsUnsignedInt64(attribute, &functionWCETInNS));
                    functionManifest->wcet = (xme_hal_time_timeInterval_t) functionWCETInNS;
                }
                else if (0 == strcmp("functionInitCB", attributeName))
                {
                    // Not yet implemented
                    //EXPECT_TRUE(false);
                }
                else if (0 == strcmp("functionWrapperExecuteCB", attributeName))
                {
                    // Not yet implemented
                    //EXPECT_TRUE(false);
                }
                else if (0 == strcmp("functionFiniCB", attributeName))
                {
                    // Not yet implemented
                    //EXPECT_TRUE(false);
                }
            }
        }
        else if (0 == strcmp("publication", elementName) || 0 == strcmp("subscription", elementName))
        {
            // Get the next free port manifest
            xme_core_componentPortManifest_t* portManifest;
            size_t maxPorts = sizeof(visitorInfo->componentManifest.portManifests) / sizeof(visitorInfo->componentManifest.portManifests[0]);
            visitorInfo->currentPortIndex++;

            EXPECT_LE(visitorInfo->currentPortIndex, maxPorts);
            //XME_CHECK_MSG(visitorInfo->currentPortIndex < maxPorts, 1, XME_LOG_WARNING, "Too many ports defined in manifest!\n");

            portManifest = &visitorInfo->componentManifest.portManifests[visitorInfo->currentPortIndex - 1U];

            if (0 == strcmp("publication", elementName))
            {
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
            }
            else if (0 == strcmp("subscription", elementName))
            {
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
            }
            else
            {
                EXPECT_TRUE(false);
            }

            for (attribute = firstAttribute; XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute; attribute = xme_hal_xml_getNextAttribute(attribute))
            {
                const char* attributeName = xme_hal_xml_getAttributeName(attribute);

                if (0 == strcmp("portName", attributeName))
                {
                    // Not yet implemented
                    //EXPECT_TRUE(false);
                }
                else if (0 == strcmp("topicID", attributeName))
                {
                    unsigned int topicID = XME_CORE_TOPIC_INVALID_TOPIC;
                    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_xml_getAttributeValueAsUnsignedInt(attribute, &topicID));
                    EXPECT_NE(XME_CORE_TOPIC_INVALID_TOPIC, (int) topicID); // FIXME: Define XME_CORE_TOPIC_INVALID_TOPIC as unsigned!
                    portManifest->topic = (xme_core_topic_t) topicID;
                }
                else if (0 == strcmp("lowerConnectionBound", attributeName))
                {
                    unsigned int lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_xml_getAttributeValueAsUnsignedInt(attribute, &lowerConnectionBound));
                    portManifest->lowerConnectionBound = lowerConnectionBound;
                }
                else if (0 == strcmp("upperConnectionBound", attributeName))
                {
                    unsigned int upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_xml_getAttributeValueAsUnsignedInt(attribute, &upperConnectionBound));
                    portManifest->upperConnectionBound = upperConnectionBound;
                }
                else if (0 == strcmp("queueSize", attributeName))
                {
                    unsigned int queueSize = 0U;
                    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_xml_getAttributeValueAsUnsignedInt(attribute, &queueSize));
                    EXPECT_NE(0U, queueSize);
                    portManifest->queueSize = queueSize; // FIXME: Possible truncation, queue size should be more than just uint8_t
                }
            }
        }
        else if (0 == strcmp("attributeDefinition", elementName))
        {
            // Get the current port manifest
            xme_core_componentPortManifest_t* portManifest = &visitorInfo->componentManifest.portManifests[visitorInfo->currentPortIndex - 1U];
            xme_core_attribute_key_t key = XME_CORE_ATTRIBUTE_KEY_UNDEFINED;
            const char* value = NULL;
            xme_core_directory_attribute_datatype_t datatype = XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID;

            // Create a new attribute set if it does not already exist
            if (XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET == portManifest->attrSet)
            {
                portManifest->attrSet = xme_core_directory_attribute_createAttributeSet();
                EXPECT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, portManifest->attrSet);
            }

            // First find the attribute key
            for (attribute = firstAttribute; XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute; attribute = xme_hal_xml_getNextAttribute(attribute))
            {
                const char* attributeName = xme_hal_xml_getAttributeName(attribute);

                if (0 == strcmp("attributeKey", attributeName))
                {
                    unsigned int attributeKey = XME_CORE_ATTRIBUTE_KEY_UNDEFINED;
                    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_xml_getAttributeValueAsUnsignedInt(attribute, &attributeKey));
                    EXPECT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, attributeKey);
                    key = (xme_core_attribute_key_t) attributeKey;
                    break;
                }
            }
            EXPECT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, key);

            // Check whether the key is registered
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeKeyDatatype(key, &datatype));
            EXPECT_NE(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID, datatype);

            // Now parse the value
            for (attribute = firstAttribute; XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute; attribute = xme_hal_xml_getNextAttribute(attribute))
            {
                const char* attributeName = xme_hal_xml_getAttributeName(attribute);

                if (0 == strcmp("attributeValue", attributeName))
                {
                    value = xme_hal_xml_getAttributeValue(attribute);
                    break;
                }
            }
            EXPECT_TRUE(NULL != value);
            EXPECT_STRNE("", value);

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_addPredefinedAttributeDefinition(portManifest->attrSet, key, value, 1, sizeof(value), datatype, true));
        }

        return 1;
    }

    static char
    leaveElementCallback
    (
        const xme_hal_xml_elementHandle_t element,
        void* userData
    )
    {
        XME_UNUSED_PARAMETER(userData);

        printf("Element %s leave!\n", xme_hal_xml_getElementName(element));

        return 1;
    }
};

TEST_F(ManifestInterchangeSmokeTest, parseManifest)
{
    xme_hal_xml_documentHandle_t doc;
    xme_hal_xml_visitorCallbacks_t callbacks;
    xme_status_t status;
    visitorInfo_s visitorInfo;

    const char* xml =
        "<?xml version=\"1.0\" standalone=\"no\" ?>\n"
        "<manifest>\n"
        "    <component\n"
        "        componentType=\"1\"\n"
        "        componentName=\"sensor\"\n"
        "        componentID=\"11\"\n"
        "        componentWrapperInitCB=\"compWrapper_init\"\n"
        "        componentWrapperReceivePortCB=\"compWrapper_receivePort\"\n"
        "        componentWrapperFiniCB=\"compWrapper_fini\"\n"
        "        componentInitCB=\"comp_init\"\n"
        "        componentFiniCB=\"comp_fini\"\n"
        "        componentInitializationString=\"initializationString\"\n"
        "        componentConstraint=\"constraint\"\n"
        "        >\n"
        "        <function\n"
        "            functionID=\"1\"\n"
        "            functionWCETInNS=\"10000000\"\n"
        "            functionInitCB=\"func_init\"\n"
        "            functionWrapperExecuteCB=\"funcWrapper_exec\"\n"
        "            functionFiniCB=\"func_fini\"\n"
        "            />\n"
        "        <publication\n"
        "            portName=\"outSensorValue\"\n"
        "            topicID=\"4097\"\n"
        "            >\n"
        "            <attributeDefinition\n"
        "                attributeKey=\"5000\"\n"
        "                attributeValue=\"123\"\n"
        "                />\n"
        "        </publication>\n"
        "        <extension extensionType=\"race\">\n"
        "            <raceManifest\n"
        "                asil=\"2\"\n"
        "                failOperational=\"1\"\n"
        "                applicationType=\"abc\"\n"
        "                relatedOtherLawAppli=\"42\"\n"
        "                priority=\"5\"\n"
        "                />\n"
        "        </extension>\n"
        "    </component>\n"
        "</manifest>";

    // register attribute key 5000
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_registerKey((xme_core_attribute_key_t) 5000, XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC));

    // Parse the document
    doc = xme_hal_xml_createDocument();
    ASSERT_TRUE(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != doc);

    status = xme_hal_xml_parseDocument(doc, xml);
    ASSERT_EQ(XME_STATUS_SUCCESS, status);

    xme_hal_mem_set(&visitorInfo.componentManifest, 0, sizeof(visitorInfo.componentManifest));
    visitorInfo.currentFunctionIndex = 0U;
    visitorInfo.currentPortIndex = 0U;

    xme_hal_mem_set(&callbacks, 0, sizeof(callbacks));
    callbacks.enterElement = enterElementCallback;
    callbacks.leaveElement = leaveElementCallback;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_xml_acceptDocumentVisitor(doc, callbacks, &visitorInfo));

    // Check the resulting manifest
    // TODO

    // Free the XML document
    xme_hal_xml_freeDocument(doc);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
