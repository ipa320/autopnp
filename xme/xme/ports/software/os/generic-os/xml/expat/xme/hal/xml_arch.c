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
 * $Id: xml_arch.c 7726 2014-03-10 10:45:38Z geisinger $
 */

/**
 * \file
 *         XML abstraction (architecture specific part: generic-OS
 *         implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/xml.h"

#include "xme/hal/include/fileio.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"
#include "xme/hal/include/table.h"

#include "expat.h"

#ifdef _MSC_VER
#define sscanf sscanf_s
#define strcasecmp _stricmp
#endif

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def XME_HAL_XML_CHUNK_SIZE
 *
 * \brief Size in bytes of the chunk to use for parsing an XML file.
 */
#define XME_HAL_XML_CHUNK_SIZE 512U

// FIXME: Move to Options.cmake
#define XME_HAL_XML_MAX_ATTRIBUTE_INFOS 16

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

struct xme_hal_xml_attributeInfo_s;

// TODO: Documentation
typedef struct
{
    XML_Parser parser; ///< The XML parser.
    XME_HAL_TABLE(struct xme_hal_xml_attributeInfo_s, attributeInfos, XME_HAL_XML_MAX_ATTRIBUTE_INFOS);
    char* filename; ///< If non-null, the filename of the XML document to load.
    FILE* stream; ///< If non-null, the stream to load.
    char* xml; ///< If non-null, the XML document to load.
    size_t xmlLen; ///< Length of XML document to load.
} xme_hal_xml_documentInfo_t;

// TODO: Documentation
typedef struct
{
    xme_hal_xml_documentInfo_t* documentInfo; ///< Document information.
    xme_hal_xml_visitorCallbacks_t* visitorCallbacks; ///< Callback information.
    void* userData; ///< Application-specific user data.
    uint16_t skip; ///< Level until which to skip element parsing, zero to disable.
    uint16_t level; ///< Current element parsing "depth".
} xme_hal_xml_parserInfo_t;

// TODO: Documentation
typedef struct
{
    const XML_Char* name; ///< Name of the element.
} xme_hal_xml_elementInfo_t;

// TODO: Documentation
typedef struct xme_hal_xml_attributeInfo_s
{
    xme_hal_xml_documentInfo_t* documentInfo; ///< The XML document.
    const XML_Char** atts; ///< Names and values of the attributes.
} xme_hal_xml_attributeInfo_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
// TODO: Documentation
void XMLCALL
_xme_hal_xml_startElementHandler
(
    void* userData,
    const XML_Char* name,
    const XML_Char** atts
);

// TODO: Documentation
void XMLCALL
_xme_hal_xml_endElementHandler
(
    void* userData,
    const XML_Char* name
);

// TODO: Documentation
xme_status_t
_xme_hal_xml_visitDocumentStream
(
    xme_hal_xml_documentInfo_t* document,
    xme_hal_fileio_fileHandle_t stream
);

// TODO: Documentation
xme_status_t
_xme_hal_xml_toBool
(
    const char* string,
    int8_t* value
);

// TODO: Documentation
xme_status_t
_xme_hal_xml_toInt
(
    const char* string,
    int* value
);

// TODO: Documentation
xme_status_t
_xme_hal_xml_toUnsignedInt
(
    const char* string,
    unsigned int* value
);

// TODO: Documentation
xme_status_t
_xme_hal_xml_toInt64
(
    const char* string,
    int64_t* value
);

// TODO: Documentation
xme_status_t
_xme_hal_xml_toUnsignedInt64
(
    const char* string,
    uint64_t* value
);

// TODO: Documentation
xme_status_t
_xme_hal_xml_toFloat
(
    const char* string,
    float* value
);

// TODO: Documentation
xme_status_t
_xme_hal_xml_toDouble
(
    const char* string,
    double* value
);

// TODO: Documentation
xme_hal_xml_attributeInfo_t*
_xme_hal_xml_createAttributeInfo
(
    xme_hal_xml_documentInfo_t* document
);

// TODO: Documentation
void
_xme_hal_xml_clearAttributeInfo
(
    xme_hal_xml_documentInfo_t* document
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void XMLCALL
_xme_hal_xml_startElementHandler
(
    void* userData,
    const XML_Char* name,
    const XML_Char** atts
)
{
    xme_hal_xml_parserInfo_t* parserInfo = (xme_hal_xml_parserInfo_t*) userData;
    XME_ASSERT_NORVAL(NULL != parserInfo);

    if (!parserInfo->skip)
    {
        if (NULL != parserInfo->visitorCallbacks->enterElement)
        {
            xme_hal_xml_elementInfo_t elementInfo;
            xme_hal_xml_attributeInfo_t attributeInfo;

            elementInfo.name = name;
            attributeInfo.documentInfo = parserInfo->documentInfo;
            attributeInfo.atts = atts;

            if (0 == parserInfo->visitorCallbacks->enterElement(&elementInfo, (NULL != atts[0]) ? &attributeInfo : NULL, parserInfo->userData))
            {
                // We can't easily skip processing of siblings, hence just don't visit the current element
                parserInfo->skip = parserInfo->level;
            }
        }
    }

    parserInfo->level++;
}

void XMLCALL
_xme_hal_xml_endElementHandler
(
    void* userData,
    const XML_Char* name
)
{
    xme_hal_xml_parserInfo_t* parserInfo = (xme_hal_xml_parserInfo_t*) userData;
    XME_ASSERT_NORVAL(NULL != parserInfo);

    parserInfo->level--;
    XME_ASSERT_NORVAL(parserInfo->level >= 1U);

    if (!parserInfo->skip)
    {
        if (NULL != parserInfo->visitorCallbacks->leaveElement)
        {
            xme_hal_xml_elementInfo_t elementInfo;
            elementInfo.name = name;

            // We can't easily skip processing of siblings, hence ignore the return value
            (void) parserInfo->visitorCallbacks->leaveElement(&elementInfo, parserInfo->userData);
        }
    }

    // Free attribute infos
    _xme_hal_xml_clearAttributeInfo(parserInfo->documentInfo);

    if (parserInfo->skip == parserInfo->level)
    {
        parserInfo->skip = 0U;
    }
}

xme_status_t
_xme_hal_xml_visitDocumentStream
(
    xme_hal_xml_documentInfo_t* document,
    xme_hal_fileio_fileHandle_t stream
)
{
    xme_status_t status;

    XME_ASSERT(XME_HAL_FILEIO_INVALID_FILE_HANDLE != stream);

    for (;;)
    {
        uint32_t bytesRead;
        enum XML_Status xmlStatus;

        void* buffer = XML_GetBuffer(document->parser, XME_HAL_XML_CHUNK_SIZE);
        XME_ASSERT(NULL != buffer);

        bytesRead = xme_hal_fileio_fread(buffer, 1UL, XME_HAL_XML_CHUNK_SIZE, stream);

        xmlStatus = XML_ParseBuffer(document->parser, bytesRead, bytesRead == 0UL);
        XME_ASSERT(XML_STATUS_ERROR == xmlStatus || XML_STATUS_OK == xmlStatus);

        if (XML_STATUS_ERROR == xmlStatus)
        {
            status = XME_STATUS_INVALID_CONFIGURATION;
            break;
        }

        if (0UL == bytesRead)
        {
            /*uint64_t position = 0ULL;
            xme_status_t st = xme_hal_fileio_ftell(stream, &position);
            XME_ASSERT(XME_STATUS_SUCCESS == st);
            if (0ULL == position)
            {
                status = XME_STATUS_NO_SUCH_VALUE;
            }*/
            break;
        }
    }

    return status;
}

xme_status_t
_xme_hal_xml_toBool
(
    const char* string,
    int8_t* value
)
{
    int intValue = 0;
    if (XME_STATUS_SUCCESS == _xme_hal_xml_toInt(string, &intValue))
    {
        *value = intValue ? 1 : 0;
        return XME_STATUS_SUCCESS;
    }
    if (0 == strcasecmp(string, "true"))
    {
        *value = 1;
        return XME_STATUS_SUCCESS;
    }
    if (0 == strcasecmp(string, "false"))
    {
        *value = 0;
        return XME_STATUS_SUCCESS;
    }

    return XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
_xme_hal_xml_toInt
(
    const char* string,
    int* value
)
{
    return (sscanf(string, "%d", value) == 1) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
_xme_hal_xml_toUnsignedInt
(
    const char* string,
    unsigned int* value
)
{
    return (sscanf(string, "%u", value) == 1) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
_xme_hal_xml_toInt64
(
    const char* string,
    int64_t* value
)
{
    return (sscanf(string, "%" SCNi64, value) == 1) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
_xme_hal_xml_toUnsignedInt64
(
    const char* string,
    uint64_t* value
)
{
    return (sscanf(string, "%" SCNu64, value) == 1) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
_xme_hal_xml_toFloat
(
    const char* string,
    float* value
)
{
    return (sscanf(string, "%f", value) == 1) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_status_t
_xme_hal_xml_toDouble
(
    const char* string,
    double* value
)
{
    return (sscanf(string, "%lf", value) == 1) ? XME_STATUS_SUCCESS : XME_STATUS_NO_SUCH_VALUE;
}

xme_hal_xml_attributeInfo_t*
_xme_hal_xml_createAttributeInfo
(
    xme_hal_xml_documentInfo_t* document
)
{
    xme_hal_xml_attributeInfo_t* attributeInfo = NULL;
    xme_hal_table_rowHandle_t handle = XME_HAL_TABLE_ADD_ITEM(document->attributeInfos);
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != handle, attributeInfo);

    attributeInfo = XME_HAL_TABLE_ITEM_FROM_HANDLE(document->attributeInfos, handle);
    XME_ASSERT_RVAL(NULL != attributeInfo, attributeInfo);

    attributeInfo->documentInfo = document;

    return attributeInfo;
}

void
_xme_hal_xml_clearAttributeInfo
(
    xme_hal_xml_documentInfo_t* document
)
{
    XME_HAL_TABLE_CLEAR(document->attributeInfos);
}

// -------------------------------------------------------------------------- //

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
    XML_Parser parser;
    xme_hal_xml_documentInfo_t* document;

    parser = XML_ParserCreate(NULL);
    XME_CHECK(NULL != parser, XME_HAL_XML_INVALID_DOCUMENT_HANDLE);

    document = (xme_hal_xml_documentInfo_t*) xme_hal_mem_alloc(sizeof(xme_hal_xml_documentInfo_t));
    XME_CHECK_REC
    (
        NULL != document,
        XME_HAL_XML_INVALID_DOCUMENT_HANDLE,
        {
            XML_ParserFree(parser);
        }
    );

    XME_HAL_TABLE_INIT(document->attributeInfos);

    document->parser = parser;
    document->filename = NULL;
    document->stream = NULL;
    document->xml = NULL;
    document->xmlLen = 0U;

    return document;
}

xme_status_t
xme_hal_xml_clearDocument
(
    xme_hal_xml_documentHandle_t documentHandle
)
{
    xme_hal_xml_documentInfo_t* document = (xme_hal_xml_documentInfo_t*) documentHandle;
    XML_Bool result;
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    _xme_hal_xml_clearAttributeInfo(document);

    result = XML_ParserReset(document->parser, NULL);
    XME_ASSERT(XML_TRUE == result);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_xml_freeDocument
(
    xme_hal_xml_documentHandle_t documentHandle
)
{
    xme_hal_xml_documentInfo_t* document = (xme_hal_xml_documentInfo_t*) documentHandle;
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    _xme_hal_xml_clearAttributeInfo(document);

    XML_ParserFree(document->parser);
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(0U == XME_HAL_TABLE_ITEM_COUNT(document->attributeInfos)));
    XME_HAL_TABLE_FINI(document->attributeInfos);
    xme_hal_mem_free(document->filename);
    xme_hal_mem_free(document->xml);
    xme_hal_mem_free(document);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_xml_loadDocumentFromFile
(
    xme_hal_xml_documentHandle_t documentHandle,
    const char* filename
)
{
    xme_hal_xml_documentInfo_t* document = (xme_hal_xml_documentInfo_t*) documentHandle;
    size_t len;

    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    XME_CHECK(xme_hal_fileio_fileExists(filename), XME_STATUS_NOT_FOUND);

    _xme_hal_xml_clearAttributeInfo(document);

    xme_hal_mem_free(document->filename);
    document->filename = NULL;
    document->stream = NULL;
    xme_hal_mem_free(document->xml);
    document->xml = NULL;
    document->xmlLen = 0U;

    len = strlen(filename);
    document->filename = (char*) xme_hal_mem_alloc(len + 1);
    XME_CHECK(NULL != document->filename, XME_STATUS_OUT_OF_RESOURCES);
    (void) xme_hal_safeString_strncpy(document->filename, filename, len + 1);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_xml_loadDocumentFromStream
(
    xme_hal_xml_documentHandle_t documentHandle,
    FILE* stream
)
{
    xme_hal_xml_documentInfo_t* document = (xme_hal_xml_documentInfo_t*) documentHandle;

    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    _xme_hal_xml_clearAttributeInfo(document);

    xme_hal_mem_free(document->filename);
    document->filename = NULL;
    document->stream = stream;
    xme_hal_mem_free(document->xml);
    document->xml = NULL;
    document->xmlLen = 0U;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_xml_parseDocument
(
    xme_hal_xml_documentHandle_t documentHandle,
    const char* xml
)
{
    xme_hal_xml_documentInfo_t* document = (xme_hal_xml_documentInfo_t*) documentHandle;

    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    _xme_hal_xml_clearAttributeInfo(document);

    xme_hal_mem_free(document->filename);
    document->filename = NULL;
    document->stream = NULL;
    xme_hal_mem_free(document->xml);
    document->xml = NULL;
    document->xmlLen = strlen(xml);

    document->xml = (char*) xme_hal_mem_alloc(document->xmlLen + 1);
    XME_CHECK(NULL != document->xml, XME_STATUS_OUT_OF_RESOURCES);
    (void) xme_hal_safeString_strncpy(document->xml, xml, document->xmlLen + 1);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_xml_saveDocumentToFile
(
    xme_hal_xml_documentHandle_t documentHandle,
    const char* filename
)
{
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    XME_UNUSED_PARAMETER(filename);

    return XME_STATUS_UNSUPPORTED;
}

xme_status_t
xme_hal_xml_saveDocumentToStream
(
    xme_hal_xml_documentHandle_t documentHandle,
    FILE* stream
)
{
    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    XME_UNUSED_PARAMETER(stream);

    return XME_STATUS_UNSUPPORTED;
}

xme_status_t
xme_hal_xml_getErrorMessage
(
    xme_hal_xml_documentHandle_t documentHandle,
    char* const buffer,
    size_t size
)
{
    xme_hal_xml_documentInfo_t* document = (xme_hal_xml_documentInfo_t*) documentHandle;
    enum XML_Error code;

    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    code = XML_GetErrorCode(document->parser);

    // Return XME_STATUS_NO_SUCH_VALUE and empty string if no error
    XME_CHECK_REC
    (
        XML_ERROR_NONE != code,
        XME_STATUS_NO_SUCH_VALUE,
        {
            (void) xme_hal_safeString_strncpy(buffer, "", size);
        }
    );

    // Format:
    //  - "Error code 123: string (line 123, column 42)"
    (void) xme_hal_safeString_snprintf
    (
        buffer, size, "Error code %d: %s (line %d, column %d)",
        code, XML_ErrorString(code),
        XML_GetCurrentLineNumber(document->parser),
        XML_GetCurrentColumnNumber(document->parser)
    );

    return XME_STATUS_SUCCESS;
}

void
xme_hal_xml_resetErrorMessage
(
    xme_hal_xml_documentHandle_t documentHandle
)
{
    // FIXME: Not available in Expat
    XME_UNUSED_PARAMETER(documentHandle);
}

xme_hal_xml_elementHandle_t
xme_hal_xml_getRootElement
(
    xme_hal_xml_documentHandle_t documentHandle
)
{
    XME_UNUSED_PARAMETER(documentHandle);

    // Not supported in Expat
    return XME_HAL_XML_INVALID_ELEMENT_HANDLE;
}

xme_status_t
xme_hal_xml_acceptDocumentVisitor
(
    xme_hal_xml_documentHandle_t documentHandle,
    xme_hal_xml_visitorCallbacks_t visitorCallbacks,
    void* userData
)
{
    xme_hal_xml_documentInfo_t* document = (xme_hal_xml_documentInfo_t*) documentHandle;
    xme_hal_xml_parserInfo_t parserInfo;
    xme_status_t status = XME_STATUS_SUCCESS;
    XML_Bool xmlStatus;

    XME_CHECK(XME_HAL_XML_INVALID_DOCUMENT_HANDLE != documentHandle, XME_STATUS_INVALID_HANDLE);

    _xme_hal_xml_clearAttributeInfo(document);

    parserInfo.documentInfo = document;
    parserInfo.visitorCallbacks = &visitorCallbacks;
    parserInfo.userData = userData;
    parserInfo.skip = 0U;
    parserInfo.level = 1U;

    xmlStatus = XML_ParserReset(document->parser, NULL);
    XME_ASSERT(XML_TRUE == xmlStatus);

    XML_SetUserData(document->parser, &parserInfo);
    XML_SetElementHandler(document->parser, &_xme_hal_xml_startElementHandler, &_xme_hal_xml_endElementHandler);

    if (NULL != visitorCallbacks.enterDocument)
    {
        visitorCallbacks.enterDocument(documentHandle, userData);
    }

    if (NULL != document->filename)
    {
        // Load from file
        xme_hal_fileio_fileHandle_t fileHandle;

        fileHandle = xme_hal_fileio_fopen(document->filename, XME_HAL_FILEIO_MODE_READONLY_BINARY);
        XME_CHECK(XME_HAL_FILEIO_INVALID_FILE_HANDLE != fileHandle, XME_STATUS_PERMISSION_DENIED);

        status = _xme_hal_xml_visitDocumentStream(document, fileHandle);

        xme_hal_fileio_fclose(fileHandle);
    }
    else if (NULL != document->stream)
    {
        // Load from stream
        status = _xme_hal_xml_visitDocumentStream(document, xme_hal_fileio_fromStream(document->stream));
    }
    else if (NULL != document->xml)
    {
        // Load from buffer
        enum XML_Status xmlStatus = XML_Parse(document->parser, document->xml, document->xmlLen, 1);
        XME_ASSERT(XML_STATUS_ERROR == xmlStatus || XML_STATUS_OK == xmlStatus);

        if (XML_STATUS_ERROR == xmlStatus) {
            status = XME_STATUS_INVALID_CONFIGURATION;
        }
        else if (0UL == document->xmlLen)
        {
            //status = XME_STATUS_NO_SUCH_VALUE;
        }
    }

    if (NULL != visitorCallbacks.leaveDocument)
    {
        visitorCallbacks.leaveDocument(documentHandle, userData);
    }

    // FIXME: Return XME_STATUS_ABORTED on abort
    return status;
}

xme_status_t
xme_hal_xml_acceptElementVisitor
(
    xme_hal_xml_elementHandle_t elementHandle,
    xme_hal_xml_visitorCallbacks_t visitorCallbacks,
    void* userData
)
{
    XME_CHECK(XME_HAL_XML_INVALID_ELEMENT_HANDLE != elementHandle, XME_STATUS_INVALID_HANDLE);

    XME_UNUSED_PARAMETER(visitorCallbacks);
    XME_UNUSED_PARAMETER(userData);

    // Not supported directly in Expat, but can be implemented on demand
    // by calling just a subset of the callbacks while parsing the file
    return XME_STATUS_UNSUPPORTED;
}

const char*
xme_hal_xml_getElementName
(
    xme_hal_xml_elementHandle_t elementHandle
)
{
    xme_hal_xml_elementInfo_t* elementInfo = (xme_hal_xml_elementInfo_t*) elementHandle;
    XME_CHECK(NULL != elementInfo, NULL);

    return elementInfo->name;
}

xme_hal_xml_attributeHandle_t
xme_hal_xml_getNextAttribute
(
    xme_hal_xml_attributeHandle_t attributeHandle
)
{
    // attributeInfo->atts points to an array of nul-terminated strings,
    // where the odd items are the attribute names and the even items
    // are the attribute values. Hence the next attribute is the one
    // at index 2 (if it is non-NULL).
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;
    xme_hal_xml_attributeInfo_t* newAttributeInfo;

    XME_CHECK(NULL != attributeInfo->atts, XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE);
    XME_CHECK(NULL != attributeInfo->atts[2], XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE);

    // There exists a next attribute. In order to not affect the initial
    // attribute handle, we have to create a new handle here and return
    // that instead.
    newAttributeInfo = _xme_hal_xml_createAttributeInfo(attributeInfo->documentInfo);
    XME_ASSERT_RVAL(NULL != newAttributeInfo, XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE);

    newAttributeInfo->atts = attributeInfo->atts + 2;

    return newAttributeInfo;
}

const char*
xme_hal_xml_getAttributeName
(
    xme_hal_xml_attributeHandle_t attributeHandle
)
{
    // attributeHandle->atts points to an array of nul-terminated strings,
    // where the odd items are the attribute names and the even items
    // are the attribute values. Hence the attribute name is the
    // element at index 0.
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;

    return attributeInfo->atts[0];
}

const char*
xme_hal_xml_getAttributeValue
(
    xme_hal_xml_attributeHandle_t attributeHandle
)
{
    // attributeHandle->atts points to an array of nul-terminated strings,
    // where the odd items are the attribute names and the even items
    // are the attribute values. Hence the attribute value is the
    // element at index 1.
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;

    XME_ASSERT_RVAL(NULL != attributeInfo->atts[0], NULL);

    return attributeInfo->atts[1];
}

xme_status_t
xme_hal_xml_getAttributeValueAsBool
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    int8_t* const value
)
{
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;

    XME_CHECK(NULL != attributeInfo->atts[0], XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != attributeInfo->atts[1], XME_STATUS_INVALID_PARAMETER);

    return _xme_hal_xml_toBool(attributeInfo->atts[1], value);
}

xme_status_t
xme_hal_xml_getAttributeValueAsInt
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    int* const value
)
{
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;

    XME_CHECK(NULL != attributeInfo->atts[0], XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != attributeInfo->atts[1], XME_STATUS_INVALID_PARAMETER);

    return _xme_hal_xml_toInt(attributeInfo->atts[1], value);
}

xme_status_t
xme_hal_xml_getAttributeValueAsUnsignedInt
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    unsigned int* const value
)
{
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;

    XME_CHECK(NULL != attributeInfo->atts[0], XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != attributeInfo->atts[1], XME_STATUS_INVALID_PARAMETER);

    return _xme_hal_xml_toUnsignedInt(attributeInfo->atts[1], value);
}

xme_status_t
xme_hal_xml_getAttributeValueAsInt64
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    int64_t* const value
)
{
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;

    XME_CHECK(NULL != attributeInfo->atts[0], XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != attributeInfo->atts[1], XME_STATUS_INVALID_PARAMETER);

    return _xme_hal_xml_toInt64(attributeInfo->atts[1], value);
}

xme_status_t
xme_hal_xml_getAttributeValueAsUnsignedInt64
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    uint64_t* const value
)
{
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;

    XME_CHECK(NULL != attributeInfo->atts[0], XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != attributeInfo->atts[1], XME_STATUS_INVALID_PARAMETER);

    return _xme_hal_xml_toUnsignedInt64(attributeInfo->atts[1], value);
}

xme_status_t
xme_hal_xml_getAttributeValueAsFloat
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    float* const value
)
{
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;

    XME_CHECK(NULL != attributeInfo->atts[0], XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != attributeInfo->atts[1], XME_STATUS_INVALID_PARAMETER);

    return _xme_hal_xml_toFloat(attributeInfo->atts[1], value);
}

xme_status_t
xme_hal_xml_getAttributeValueAsDouble
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    double* const value
)
{
    xme_hal_xml_attributeInfo_t* attributeInfo = (xme_hal_xml_attributeInfo_t*) attributeHandle;

    XME_CHECK(NULL != attributeInfo->atts[0], XME_STATUS_INVALID_HANDLE);
    XME_CHECK(NULL != attributeInfo->atts[1], XME_STATUS_INVALID_PARAMETER);

    return _xme_hal_xml_toDouble(attributeInfo->atts[1], value);
}
