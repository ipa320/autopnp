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
 * $Id: manifestInterchange.c 6223 2013-12-20 15:21:34Z geisinger $
 */

/**
 * \file
 *         Manifest interchange.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/manifestRepository/include/manifestInterchange.h"

#include "xme/core/log.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/xml.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

static char
enterElementCallback
(
    const xme_hal_xml_elementHandle_t element,
    const xme_hal_xml_attributeHandle_t firstAttribute,
    void* userData
)
{
    xme_hal_xml_attributeHandle_t attribute;

    XME_UNUSED_PARAMETER(userData);

    XME_LOG(XME_LOG_ALWAYS, "Element %s enter!\n", xme_hal_xml_getElementName(element));

    for (attribute = firstAttribute; XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute; attribute = xme_hal_xml_getNextAttribute(attribute))
    {
        XME_LOG
        (
            XME_LOG_ALWAYS, " - Attribute %s = %s\n",
            xme_hal_xml_getAttributeName(attribute), xme_hal_xml_getAttributeValue(attribute)
        );
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

    XME_LOG(XME_LOG_ALWAYS, "Element %s leave!\n", xme_hal_xml_getElementName(element));

    return 1;
}

xme_status_t
xme_core_manifestInterchange_parseManifest(const char* filename, xme_core_componentManifest_t* outComponentManifest)
{
    xme_hal_xml_documentHandle_t doc;
    xme_hal_xml_elementHandle_t root;
    xme_hal_xml_visitorCallbacks_t callbacks;
    xme_status_t status;

    const char* xml =
        "<?xml version=\"1.0\" standalone=\"no\" ?>\n"
        "<manifest>\n"
        "    <component\n"
        "        componentType=\"1\"\n"
        "        componentName=\"sensor\"\n"
        "        componentID=\"11\"\n"
        "        >\n"
        "        <function\n"
        "            functionId=\"1\"\n"
        "            function_wcet_in_us=\"10000\"\n"
        "            />\n"
        "        <publication\n"
        "            portName=\"outSensorValue\"\n"
        "            portIndex=\"1\"\n"
        "            topicID=\"4097\"\n"
        "            >\n"
        "            <attributeDefinition\n"
        "                attributeKey=\"5000\"\n"
        "                attributeValue=\"123\"\n"
        "                />\n"
        "        </publication>\n"
        "    </component>\n"
        "</manifest>";

    // TODO: Use filename
    XME_UNUSED_PARAMETER(filename);
    //XME_CHECK(NULL != filename, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != outComponentManifest, XME_STATUS_INVALID_PARAMETER);

    doc = xme_hal_xml_createDocument();
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != doc, XME_STATUS_OUT_OF_RESOURCES);

    status = xme_hal_xml_parseDocument(doc, xml);
    XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "Error parsing manifest XML!\n");

    root = xme_hal_xml_getRootElement(doc);
    XME_CHECK_MSG(XME_HAL_XML_INVALID_ELEMENT_HANDLE != root, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "Unable to retrieve root element!\n");

    xme_hal_mem_set(&callbacks, 0, sizeof(callbacks));
    callbacks.enterElement = enterElementCallback;
    callbacks.leaveElement = leaveElementCallback;
    xme_hal_xml_acceptElementVisitor(root, callbacks, outComponentManifest);

    return XME_STATUS_SUCCESS;
}
