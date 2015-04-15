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
 * $Id: xml_arch.cpp 7726 2014-03-10 10:45:38Z geisinger $
 */

/**
 * \file
 *         XML abstraction (architecture specific part: generic-OS
 *         implementation).
 */

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
// Ensure that PRIxXX and SCNxXX macros are defined by <inttypes.h>
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif // #ifndef __STDC_FORMAT_MACROS

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/xml.h"

#include "xme/hal/include/safeString.h"

#include "tinyxml2.h"

using namespace tinyxml2;

/******************************************************************************/
/***   Class definitions                                                    ***/
/******************************************************************************/
class CallbackVisitor : public XMLVisitor
{
public:
    CallbackVisitor(const xme_hal_xml_visitorCallbacks_t& visitorCallbacks, void* userData)
    : _visitorCallbacks(visitorCallbacks)
    , _userData(userData)
    {
    }
    
    virtual ~CallbackVisitor()
    {
    }

    /**
     * \brief Triggers the callback function for entering an XML document.
     *
     * \param[in] doc Document being entered.
     *
     * \retval true if no visitor callback for entering XML documents has been
     *         set or if the visitor callback returned a non-zero value.
     * \retval false otherwise.
     */
    virtual bool VisitEnter(const XMLDocument& doc)
    {
        XME_CHECK(NULL != _visitorCallbacks.enterDocument, true);
        return _visitorCallbacks.enterDocument((xme_hal_xml_documentHandle_t) &doc, _userData) ? true : false;
    }

    /**
     * \brief Triggers the callback function for leaving an XML document.
     *
     * \param[in] doc Document being entered.
     *
     * \retval true if no visitor callback for leaving XML documents has been
     *         set or if the visitor callback returned a non-zero value.
     * \retval false otherwise.
     */
    virtual bool VisitExit(const XMLDocument& doc)
    {
        XME_CHECK(NULL != _visitorCallbacks.leaveDocument, true);
        return _visitorCallbacks.leaveDocument((xme_hal_xml_documentHandle_t) &doc, _userData) ? true : false;
    }

    /**
     * \brief Triggers the callback function for entering an XML element.
     *
     * \param[in] element Element being entered.
     * \param[in] firstAttribute First attribute of the element being entered.
     *            NULL if no such attribute exists.
     *
     * \retval true if no visitor callback for entering XML elements has been
     *         set or if the visitor callback returned a non-zero value.
     * \retval false otherwise.
     */
    virtual bool VisitEnter(const XMLElement& element, const XMLAttribute* firstAttribute)
    {
        XME_CHECK(NULL != _visitorCallbacks.enterElement, true);
        return _visitorCallbacks.enterElement((xme_hal_xml_elementHandle_t) &element, (xme_hal_xml_attributeHandle_t) firstAttribute, _userData) ? true : false;
    }

    /**
     * \brief Triggers the callback function for leaving an XML element.
     *
     * \param[in] element Element being left.
     *
     * \retval true if no visitor callback for entering XML elements has been
     *         set or if the visitor callback returned a non-zero value.
     * \retval false otherwise.
     */
    virtual bool VisitExit(const XMLElement& element)
    {
        XME_CHECK(NULL != _visitorCallbacks.leaveElement, true);
        return _visitorCallbacks.leaveElement((xme_hal_xml_elementHandle_t) &element, _userData) ? true : false;
    }

    /**
     * \brief Triggers the callback function for visiting an XML declaration.
     *
     * \param[in] declaration Declaration being visited.
     *
     * \retval true if no visitor callback for XML declarations has been set
     *         or if the visitor callback returned a non-zero value.
     * \retval false otherwise.
     */
    virtual bool Visit(const XMLDeclaration& declaration)
    {
        XME_CHECK(NULL != _visitorCallbacks.visitDeclaration, true);
        return _visitorCallbacks.visitDeclaration((xme_hal_xml_declarationHandle_t) &declaration, _userData) ? true : false;
    }

    /**
     * \brief Triggers the callback function for visiting an XML text node.
     *
     * \param[in] text Text node being visited.
     *
     * \retval true if no visitor callback for XML text nodes has been set
     *         or if the visitor callback returned a non-zero value.
     * \retval false otherwise.
     */
    virtual bool Visit(const XMLText& text)
    {
        XME_CHECK(NULL != _visitorCallbacks.visitTextNode, true);
        return _visitorCallbacks.visitTextNode((xme_hal_xml_declarationHandle_t) &text, _userData) ? true : false;
    }

    /**
     * \brief Triggers the callback function for visiting an XML comment node.
     *
     * \param[in] comment Comment node being visited.
     *
     * \retval true if no visitor callback for XML comment nodes has been set
     *         or if the visitor callback returned a non-zero value.
     * \retval false otherwise.
     */
    virtual bool Visit(const XMLComment& comment)
    {
        XME_CHECK(NULL != _visitorCallbacks.visitCommentNode, true);
        return _visitorCallbacks.visitCommentNode((xme_hal_xml_declarationHandle_t) &comment, _userData) ? true : false;
    }

    /**
     * \brief Triggers the callback function for visiting an unknown XML node.
     *
     * \param[in] unknown Unknown node being visited.
     *
     * \retval true if no visitor callback for unknown XML nodes has been set
     *         or if the visitor callback returned a non-zero value.
     * \retval false otherwise.
     */
    virtual bool Visit( const XMLUnknown& unknown)
    {
        XME_CHECK(NULL != _visitorCallbacks.visitUnknownNode, true);
        return _visitorCallbacks.visitUnknownNode((xme_hal_xml_declarationHandle_t) &unknown, _userData) ? true : false;
    }

protected:
    xme_hal_xml_visitorCallbacks_t _visitorCallbacks; ///< Callbacks for visiting the items of the given node.
    void* _userData; ///< User-defined data to pass to the callback functions.
};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

xme_status_t
xme_hal_xml_init(void)
{
    // Nothing to do
    return XME_STATUS_SUCCESS;
}

void
xme_hal_xml_fini(void)
{
    // Nothing to do
}

xme_hal_xml_documentHandle_t
xme_hal_xml_createDocument(void)
{
    return new XMLDocument();
}

xme_status_t
xme_hal_xml_clearDocument
(
    xme_hal_xml_documentHandle_t documentHandle
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);
    
    document->Clear();
    
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_xml_freeDocument
(
    xme_hal_xml_documentHandle_t documentHandle
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);
    
    delete document;
    
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_xml_loadDocumentFromFile
(
    xme_hal_xml_documentHandle_t documentHandle,
    const char* filename
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    XMLError error = document->LoadFile(filename);
    switch (error)
    {
        case XML_NO_ERROR:
            return XME_STATUS_SUCCESS;
        case XML_ERROR_FILE_NOT_FOUND:
            return XME_STATUS_NOT_FOUND;
        case XML_ERROR_FILE_COULD_NOT_BE_OPENED:
        case XML_ERROR_FILE_READ_ERROR:
            return XME_STATUS_PERMISSION_DENIED;
        case XML_ERROR_EMPTY_DOCUMENT:
            return XME_STATUS_NO_SUCH_VALUE;
        case XML_ERROR_PARSING:
        case XML_ERROR_MISMATCHED_ELEMENT:
        case XML_ERROR_PARSING_ATTRIBUTE:
        case XML_ERROR_PARSING_ELEMENT:
        case XML_ERROR_PARSING_CDATA:
        case XML_ERROR_PARSING_DECLARATION:
        case XML_ERROR_PARSING_COMMENT:
        case XML_ERROR_PARSING_TEXT:
        case XML_ERROR_PARSING_UNKNOWN:
            return XME_STATUS_INVALID_CONFIGURATION;
        default:
            XME_ASSERT(!"Unknown error loading XML document from file!");
            return XME_STATUS_INTERNAL_ERROR;
    }
}

xme_status_t
xme_hal_xml_loadDocumentFromStream
(
    xme_hal_xml_documentHandle_t documentHandle,
    FILE* stream
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    XMLError error = document->LoadFile(stream);
    switch (error)
    {
        case XML_NO_ERROR:
            return XME_STATUS_SUCCESS;
        case XML_ERROR_FILE_READ_ERROR:
            return XME_STATUS_PERMISSION_DENIED;
        case XML_ERROR_EMPTY_DOCUMENT:
            return XME_STATUS_NO_SUCH_VALUE;
        case XML_ERROR_PARSING:
        case XML_ERROR_MISMATCHED_ELEMENT:
        case XML_ERROR_PARSING_ATTRIBUTE:
        case XML_ERROR_PARSING_ELEMENT:
        case XML_ERROR_PARSING_CDATA:
        case XML_ERROR_PARSING_DECLARATION:
        case XML_ERROR_PARSING_COMMENT:
        case XML_ERROR_PARSING_TEXT:
        case XML_ERROR_PARSING_UNKNOWN:
            return XME_STATUS_INVALID_CONFIGURATION;
        default:
            XME_ASSERT(!"Unknown error loading XML document from stream!");
            return XME_STATUS_INTERNAL_ERROR;
    }
}

xme_status_t
xme_hal_xml_parseDocument
(
    xme_hal_xml_documentHandle_t documentHandle,
    const char* xml
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    XMLError error = document->Parse(xml);
    switch (error)
    {
        case XML_NO_ERROR:
            return XME_STATUS_SUCCESS;
        case XML_ERROR_EMPTY_DOCUMENT:
            return XME_STATUS_NO_SUCH_VALUE;
        case XML_ERROR_PARSING:
        case XML_ERROR_MISMATCHED_ELEMENT:
        case XML_ERROR_PARSING_ATTRIBUTE:
        case XML_ERROR_PARSING_ELEMENT:
        case XML_ERROR_PARSING_CDATA:
        case XML_ERROR_PARSING_DECLARATION:
        case XML_ERROR_PARSING_COMMENT:
        case XML_ERROR_PARSING_TEXT:
        case XML_ERROR_PARSING_UNKNOWN:
            return XME_STATUS_INVALID_CONFIGURATION;
        default:
            XME_ASSERT(!"Unknown error parsing XML document from string!");
            return XME_STATUS_INTERNAL_ERROR;
    }
}

xme_status_t
xme_hal_xml_saveDocumentToFile
(
    xme_hal_xml_documentHandle_t documentHandle,
    const char* filename
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    XMLError error = document->SaveFile(filename);
    switch (error)
    {
        case XML_NO_ERROR:
            return XME_STATUS_SUCCESS;
        case XML_ERROR_FILE_COULD_NOT_BE_OPENED:
            return XME_STATUS_PERMISSION_DENIED;
        default:
            XME_ASSERT(!"Unknown error saving XML document to file!");
            return XME_STATUS_INTERNAL_ERROR;
    }
}

xme_status_t
xme_hal_xml_saveDocumentToStream
(
    xme_hal_xml_documentHandle_t documentHandle,
    FILE* stream
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    XMLError error = document->SaveFile(stream);
    switch (error)
    {
        case XML_NO_ERROR:
            return XME_STATUS_SUCCESS;
        default:
            XME_ASSERT(!"Unknown error saving XML document to stream!");
            return XME_STATUS_INTERNAL_ERROR;
    }
}

xme_status_t
xme_hal_xml_getErrorMessage
(
    xme_hal_xml_documentHandle_t documentHandle,
    char* const buffer,
    size_t size
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    // Return XME_STATUS_NO_SUCH_VALUE and empty string if no error
    XME_CHECK_REC
    (
        document->Error(),
        XME_STATUS_NO_SUCH_VALUE,
        {
            (void) xme_hal_safeString_strncpy(buffer, "", size);
        }
    );

    const char* str1 = document->GetErrorStr1();
    const char* str2 = document->GetErrorStr2();

    // Format:
    //  - "Error code 123: string 1, string 2"
    //  - "Error code 123: string 1"
    //  - "Error code 123: string 2" (if no str1)
    //  - "Error code 123"
    (void) xme_hal_safeString_snprintf
    (
        buffer, size, "Error code %d%s%s%s%s",
        document->ErrorID(),
        (NULL != str1 || NULL != str2) ? ": " : "",
        str1,
        (NULL != str1 && NULL != str2) ? ", " : "",
        str2
    );

    return XME_STATUS_SUCCESS;
}

void
xme_hal_xml_resetErrorMessage
(
    xme_hal_xml_documentHandle_t documentHandle
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_CHECK_RVAL_VOID);

    document->SetError(XML_NO_ERROR, NULL, NULL);
}

xme_hal_xml_elementHandle_t
xme_hal_xml_getRootElement
(
    xme_hal_xml_documentHandle_t documentHandle
)
{
    XMLDocument* document = static_cast<XMLDocument*>(documentHandle);
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_HAL_XML_INVALID_ELEMENT_HANDLE);

    return (xme_hal_xml_elementHandle_t) document->RootElement();
}

xme_status_t
xme_hal_xml_acceptDocumentVisitor
(
    xme_hal_xml_documentHandle_t documentHandle,
    xme_hal_xml_visitorCallbacks_t visitorCallbacks,
    void* userData
)
{
    XMLDocument* document = (XMLDocument*) documentHandle;
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    CallbackVisitor cbv(visitorCallbacks, userData);
    return document->Accept(&cbv) ? XME_STATUS_SUCCESS : XME_STATUS_ABORTED;
}

xme_status_t
xme_hal_xml_acceptElementVisitor
(
    xme_hal_xml_elementHandle_t elementHandle,
    xme_hal_xml_visitorCallbacks_t visitorCallbacks,
    void* userData
)
{
    XMLNode* node = (XMLNode*) elementHandle;
    XME_CHECK(NULL != node, XME_STATUS_INVALID_HANDLE);

    XMLElement* element = node->ToElement();
    XME_CHECK(XME_HAL_XML_INVALID_ELEMENT_HANDLE != element, XME_STATUS_INVALID_HANDLE);

    CallbackVisitor cbv(visitorCallbacks, userData);
    return element->Accept(&cbv) ? XME_STATUS_SUCCESS : XME_STATUS_ABORTED;
}

const char*
xme_hal_xml_getElementName
(
    xme_hal_xml_elementHandle_t elementHandle
)
{
    XMLNode* node = (XMLNode*) elementHandle;
    XME_CHECK(NULL != node, NULL);

    XMLElement* element = node->ToElement();
    XME_CHECK(XME_HAL_XML_INVALID_ELEMENT_HANDLE != element, NULL);

    return element->Value();
}

xme_hal_xml_attributeHandle_t
xme_hal_xml_getNextAttribute
(
    xme_hal_xml_attributeHandle_t attributeHandle
)
{
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE);

    return (xme_hal_xml_attributeHandle_t) attribute->Next();
}

const char*
xme_hal_xml_getAttributeName
(
    xme_hal_xml_attributeHandle_t attributeHandle
)
{
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, NULL);

    return attribute->Name();
}

const char*
xme_hal_xml_getAttributeValue
(
    xme_hal_xml_attributeHandle_t attributeHandle
)
{
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, NULL);

    return attribute->Value();
}

xme_status_t
xme_hal_xml_getAttributeValueAsBool
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    int8_t* const value
)
{
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != value, XME_STATUS_INVALID_PARAMETER);

    bool b = false;
    XMLError error = attribute->QueryBoolValue(&b);
    XME_CHECK(XML_NO_ERROR == error, XME_STATUS_NO_SUCH_VALUE);

    *value = b ? 1 : 0;
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_xml_getAttributeValueAsInt
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    int* const value
)
{
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != value, XME_STATUS_INVALID_PARAMETER);

    return (XML_NO_ERROR == attribute->QueryIntValue(value)) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
xme_hal_xml_getAttributeValueAsUnsignedInt
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    unsigned int* const value
)
{
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != value, XME_STATUS_INVALID_PARAMETER);

    return (XML_NO_ERROR == attribute->QueryUnsignedValue(value)) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
xme_hal_xml_getAttributeValueAsInt64
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    int64_t* const value
)
{
    int scanned;
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != value, XME_STATUS_INVALID_PARAMETER);

#ifdef WIN32
    scanned = sscanf_s(attribute->Value(), "%" SCNi64, value);
#else
    scanned = sscanf(attribute->Value(), "%" SCNi64, value);
#endif

    return (1 == scanned) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
xme_hal_xml_getAttributeValueAsUnsignedInt64
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    uint64_t* const value
)
{
    int scanned;
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != value, XME_STATUS_INVALID_PARAMETER);

#ifdef WIN32
    scanned = sscanf_s(attribute->Value(), "%" SCNu64, value);
#else
    scanned = sscanf(attribute->Value(), "%" SCNu64, value);
#endif

    return (1 == scanned) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
xme_hal_xml_getAttributeValueAsFloat
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    float* const value
)
{
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != value, XME_STATUS_INVALID_PARAMETER);

    return (XML_NO_ERROR == attribute->QueryFloatValue(value)) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
xme_hal_xml_getAttributeValueAsDouble
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    double* const value
)
{
    XMLAttribute* attribute = static_cast<XMLAttribute*>(attributeHandle);
    XME_CHECK(XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE != attribute, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != value, XME_STATUS_INVALID_PARAMETER);

    return (XML_NO_ERROR == attribute->QueryDoubleValue(value)) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

XME_EXTERN_C_END
