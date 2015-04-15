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
 * $$
 */

/**
 * \file
 * \brief XML abstraction.
 */

#ifndef XME_HAL_XML_H
#define XME_HAL_XML_H

/**
 * \defgroup hal_xml XML abstraction
 * @{
 *
 * \brief XML abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <inttypes.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \brief Handle representing an invalid XML document.
 */
#define XME_HAL_XML_INVALID_DOCUMENT_HANDLE (NULL)

/**
 * \brief Handle representing an invalid XML element.
 */
#define XME_HAL_XML_INVALID_ELEMENT_HANDLE (NULL)

/**
 * \brief Handle representing an invalid XML attribute.
 */
#define XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE (NULL)

/**
 * \brief Handle representing an invalid XML declaration.
 */
#define XME_HAL_XML_INVALID_DECLARATION_HANDLE (NULL)

/**
 * \brief Handle representing an invalid XML text node.
 */
#define XME_HAL_XML_INVALID_TEXT_NODE_HANDLE (NULL)

/**
 * \brief Handle representing an invalid XML comment node.
 */
#define XME_HAL_XML_INVALID_COMMENT_NODE_HANDLE (NULL)

/**
 * \brief Handle representing an invalid unknown XML node.
 */
#define XME_HAL_XML_INVALID_UNKNOWN_NODE_HANDLE (NULL)

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \brief Handle to an XML document.
 */
typedef void* xme_hal_xml_documentHandle_t;

/**
 * \brief Handle to an XML element.
 */
typedef void* xme_hal_xml_elementHandle_t;

/**
 * \brief Handle to an XML attribute.
 */
typedef void* xme_hal_xml_attributeHandle_t;

/**
 * \brief Handle to an XML declaration.
 */
typedef void* xme_hal_xml_declarationHandle_t;

/**
 * \brief Handle to an XML text node.
 */
typedef void* xme_hal_xml_textNodeHandle_t;

/**
 * \brief Handle to an XML comment node.
 */
typedef void* xme_hal_xml_commentNodeHandle_t;

/**
 * \brief Handle to an unknown XML node.
 */
typedef void* xme_hal_xml_unknownNodeHandle_t;

/**
 * \brief Visitor callback for entering or leaving an XML document.
 *
 * \param[in] document Handle of the XML document being entered or left.
 * \param[in,out] userData Address of user-defined memory passed to the
 *                xme_hal_xml_acceptElementVisitor() function.
 *
 * \return If the function returns a non-zero value, parsing will continue.
 *         If the function returns zero, no elements in this document
 *         will be visited (if the document has been entered) or
 *         xme_hal_xml_acceptElementVisitor() returns with status
 *         XME_STATUS_ABORTED (if the document has been left).
 */
typedef char (*xme_hal_xml_visitDocumentCallback_t) (const xme_hal_xml_documentHandle_t document, void* userData);

/**
 * \brief Visitor callback for entering an XML element.
 *
 * \param[in] element Handle of the XML element being entered.
 * \param[in] firstAttribute Handle of the first attribute within the element.
 *            XME_HAL_XML_INVALID_ELEMENT_HANDLE if the element does not have
 *            any attributes.
 * \param[in,out] userData Address of user-defined memory passed to the
 *                xme_hal_xml_acceptElementVisitor() function.
 *
 * \return If the function returns a non-zero value, recursive parsing will
 *         continue. If the function returns zero, no children of this
 *         element will be visited. Whether or not siblings of the rejected
 *         element will be visited is implementation-defined.
 */
typedef char (*xme_hal_xml_visitEnterElementCallback_t) (const xme_hal_xml_elementHandle_t element, const xme_hal_xml_attributeHandle_t firstAttribute, void* userData);

/**
 * \brief Visitor callback for leaving an XML element.
 *
 * \param[in] element Handle of the XML element being left.
 * \param[in,out] userData Address of user-defined memory passed to the
 *                xme_hal_xml_acceptElementVisitor() function.
 *
 * \return If the function returns a non-zero value, recursive parsing will
 *         continue. If the function returns zero, the implementation
 *         defines whether or not siblings of the element being leftwill be
 *         visited.
 */
typedef char (*xme_hal_xml_visitLeaveElementCallback_t) (const xme_hal_xml_elementHandle_t element, void* userData);

/**
 * \brief Visitor callback for visiting an XML declaration.
 *
 * \details Unknown nodes are identified by the tokens "<?...?>".
 *
 * \param[in] declaration Handle of the XML declaration being entered.
 * \param[in,out] userData Address of user-defined memory passed to the
 *                xme_hal_xml_acceptElementVisitor() function.
 *
 * \return If the function returns a non-zero value, recursive parsing will
 *         continue. If the function returns zero, no children of this
 *         declaration will be visited. Whether or not siblings of the
 *         rejected declaration will be visited is implementation-defined.
 */
typedef char (*xme_hal_xml_visitDeclarationCallback_t) (const xme_hal_xml_declarationHandle_t declaration, void* userData);

/**
 * \brief Visitor callback for visiting an XML text node.
 *
 * \param[in] textNode Handle of the XML text node being entered.
 * \param[in,out] userData Address of user-defined memory passed to the
 *                xme_hal_xml_acceptElementVisitor() function.
 *
 * \return If the function returns a non-zero value, recursive parsing will
 *         continue. If the function returns zero, no children of this
 *         text node will be visited. Whether or not siblings of the
 *         rejected text node will be visited is implementation-defined.
 */
typedef char (*xme_hal_xml_visitTextNodeCallback_t) (const xme_hal_xml_textNodeHandle_t textNode, void* userData);

/**
 * \brief Visitor callback for visiting an XML comment node.
 *
 * \details Comment nodes are identified by the tokens "<!--...-->".
 *
 * \param[in] commentNode Handle of the XML comment node being entered.
 * \param[in,out] userData Address of user-defined memory passed to the
 *                xme_hal_xml_acceptElementVisitor() function.
 *
 * \return If the function returns a non-zero value, recursive parsing will
 *         continue. If the function returns zero, no children of this
 *         comment node will be visited. Whether or not siblings of the
 *         rejected comment node will be visited is implementation-defined.
 */
typedef char (*xme_hal_xml_visitCommentNodeCallback_t) (const xme_hal_xml_commentNodeHandle_t commentNode, void* userData);

/**
 * \brief Visitor callback for visiting an unknown XML node.
 *
 * \details Unknown nodes are identified by the tokens "<!...>".
 *
 * \param[in] unknownNode Handle of the unknown XML node being entered.
 * \param[in,out] userData Address of user-defined memory passed to the
 *                xme_hal_xml_acceptElementVisitor() function.
 *
 * \return If the function returns a non-zero value, recursive parsing will
 *         continue. If the function returns zero, no children of this
 *         unknown node will be visited. Whether or not siblings of the
 *         rejected unknown node will be visited is implementation-defined.
 */
typedef char (*xme_hal_xml_visitUnknownNodeCallback_t) (const xme_hal_xml_unknownNodeHandle_t unknownNode, void* userData);

/**
 * \brief Structure with callback functions for visiting an XME document.
 */
typedef struct
{
    xme_hal_xml_visitDocumentCallback_t enterDocument; ///< Callback function to call when entering an XML document.
    xme_hal_xml_visitDocumentCallback_t leaveDocument; ///< Callback function to call when leaving an XML document.
    xme_hal_xml_visitEnterElementCallback_t enterElement; ///< Callback function to call when entering an XML element.
    xme_hal_xml_visitLeaveElementCallback_t leaveElement; ///< Callback function to call when leaving an XML element.
    xme_hal_xml_visitDeclarationCallback_t visitDeclaration; ///< Callback function to call when visiting an XML declaration.
    xme_hal_xml_visitTextNodeCallback_t visitTextNode; ///< Callback function to call when visiting an XML text node.
    xme_hal_xml_visitCommentNodeCallback_t visitCommentNode; ///< Callback function to call when visiting an XML comment node.
    xme_hal_xml_visitUnknownNodeCallback_t visitUnknownNode; ///< Callback function to call when visiting an unknown XML node.
} xme_hal_xml_visitorCallbacks_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the XML abstraction.
 *
 * \retval XME_STATUS_SUCCESS on successful initialization.
 * \retval XME_STATUS_OUT_OF_RESOURCES if not enough resources were available.
 */
xme_status_t
xme_hal_xml_init(void);

/**
 * \brief Frees resources occupied by the XML abstraction.
 */
void
xme_hal_xml_fini(void);

/**
 * \brief Creates a new XML document handle.
 *
 * \details The document handle can subsequently be used for loading XML
 *          content from a file (xme_hal_xml_loadDocumentFromFile()),
 *          loading XML content from a stream (xme_hal_xml_loadDocumentFromStream())
 *          or parsing XML content from a string (xme_hal_xml_parseDocument()).
 *          Call xme_hal_freeDocument() to free the document after use.
 *
 * \return XML document handle.
 *
 * \see xme_hal_freeDocument()
 */
xme_hal_xml_documentHandle_t
xme_hal_xml_createDocument(void);

/**
 * \brief Clears the XML document represented by the given handle.
 *
 * \note The given handle is still valid after calling this function.
 *
 * \param[in] documentHandle Handle of the document to clear.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if documentHandle was invalid.
 */
xme_status_t
xme_hal_xml_clearDocument
(
    xme_hal_xml_documentHandle_t documentHandle
);

/**
 * \brief Frees the XML document represented by the given handle.
 *
 * \note The given handle must have been previously allocated using
 *       xme_hal_xml_createDocument().
 *
 * \note The given handle is not valid any more after calling this function.
 *
 * \param[in] documentHandle Handle of the document to free.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if documentHandle was invalid.
 *
 * \see xme_hal_xml_createDocument()
 */
xme_status_t
xme_hal_xml_freeDocument
(
    xme_hal_xml_documentHandle_t documentHandle
);

/**
 * \brief Loads XML content identified by a filename into an XML document.
 *
 * \note The given handle must have been previously allocated using
 *       xme_hal_xml_createDocument().
 *
 * \param[in] documentHandle Handle of the document to fill with the content
 *            loaded from the file.
 * \param[in] filename Name of the file to load XML content from.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_NO_SUCH_VALUE if the XML content was loaded
 *         successfully, but the file was empty.
 * \retval XME_STATUS_NOT_FOUND if the file with the given name could not be
 *         found.
 * \retval XME_STATUS_PERMISSION_DENIED if reading from the file failed;
 *         check file system permissions.
 * \reval  XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to load the file.
 * \retval XME_STATUS_INVALID_CONFIGURATION if parsing the XML content failed
 *         due to syntax errors in the file. Use xme_hal_xml_getErrorMessage()
 *         to obtain more information.
 * \retval XME_STATUS_INVALID_HANDLE if documentHandle was invalid.
 */
xme_status_t
xme_hal_xml_loadDocumentFromFile
(
    xme_hal_xml_documentHandle_t documentHandle,
    const char* filename
);

/**
 * \brief Loads XML content identified by an open stream into an XML document.
 *
 * \note The given handle must have been previously allocated using
 *       xme_hal_xml_createDocument().
 *
 * \param[in] documentHandle Handle of the document to fill with the content
 *            loaded from the stream.
 * \param[in] stream Stream to load XML content from. The stream must have
 *            been previously opened with appropriate permissions.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_NO_SUCH_VALUE if the XML content was loaded
 *         successfully, but the file was empty.
 * \retval XME_STATUS_PERMISSION_DENIED if reading from the stream failed;
 *         check stream permissions.
 * \reval  XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to load the stream.
 * \retval XME_STATUS_INVALID_CONFIGURATION if parsing the XML content failed
 *         due to syntax errors in the stream. Use xme_hal_xml_getErrorMessage()
 *         to obtain more information.
 * \retval XME_STATUS_INVALID_HANDLE if documentHandle was invalid.
 */
xme_status_t
xme_hal_xml_loadDocumentFromStream
(
    xme_hal_xml_documentHandle_t documentHandle,
    FILE* stream
);

/**
 * \brief Parses XML content from a string into an XML document.
 *
 * \note The given handle must have been previously allocated using
 *       xme_hal_xml_createDocument().
 *
 * \param[in] documentHandle Handle of the document to fill with the content
 *            parsed from the string.
 * \param[in] xml C string to parse XML content from.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_NO_SUCH_VALUE if the XML content was parsed
 *         successfully, but the string was empty.
 * \reval  XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to parse the XML.
 * \retval XME_STATUS_INVALID_CONFIGURATION if parsing the XML content failed
 *         due to syntax errors in the string. Use xme_hal_xml_getErrorMessage()
 *         to obtain more information.
 * \retval XME_STATUS_INVALID_HANDLE if documentHandle was invalid.
 */
xme_status_t
xme_hal_xml_parseDocument
(
    xme_hal_xml_documentHandle_t documentHandle,
    const char* xml
);

/**
 * \brief Saves the given XML document to the file identified by the given
 *        name.
 *
 * \note The given handle must have been previously allocated using
 *       xme_hal_xml_createDocument().
 *
 * \param[in] documentHandle Handle of the document to save to the file.
 * \param[in] filename Name of the file where to save the document to.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_UNSUPPORTED if the XML abstraction does not support
 *         saving an XML document to file.
 * \retval XME_STATUS_PERMISSION_DENIED if the file could not be opened for
 *         writing; check file system permissions.
 * \reval  XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to save the file.
 */
xme_status_t
xme_hal_xml_saveDocumentToFile
(
    xme_hal_xml_documentHandle_t documentHandle,
    const char* filename
);

/**
 * \brief Saves the given XML document to the given stream.
 *
 * \note The given handle must have been previously allocated using
 *       xme_hal_xml_createDocument().
 *
 * \param[in] documentHandle Handle of the document to save to the stream.
 * \param[in] stream Stream where to save the document to.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_UNSUPPORTED if the XML abstraction does not support
 *         saving an XML document to file.
 * \reval  XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to save the stream.
 */
// FIXME: Check for errors writing to stream (this is currently not supported by TinyXML2)!
xme_status_t
xme_hal_xml_saveDocumentToStream
(
    xme_hal_xml_documentHandle_t documentHandle,
    FILE* stream
);

/**
 * \brief Copies the error message of the previous failing operation in
 *        conjunction with the given XML document into the given buffer.
 *
 * \details If no error ocurred since the document was created or since the
 *          last error reset using xme_hal_xml_resetErrorMessage(), an empty
 *          string is copied into the buffer (if buffer is not NULL and size
 *          is larger than zero) and XME_STATUS_NO_SUCH_VALUE is returned.
 *
 * \note If size is to small to store the whole error message, the message
 *       is truncated and XME_STATUS_SUCCESS is returned.
 *
 * \param[in] documentHandle Handle of the document to retrieve the error
 *            message for.
 * \param[in] buffer Buffer where to copy the error message to.
 * \param[in] size Size of the buffer.
 *
 * \retval XME_STATUS_SUCCESS if an error occurred previously and the error
 *         message has been copied into the buffer (possibly truncated).
 * \retval XME_STATUS_NO_SUCH_VALUE if there was no previous error since the
 *         creation of the document or the last error reset.
 * \retval XME_STATUS_INVALID_HANDLE if documentHandle was invalid.
 */
xme_status_t
xme_hal_xml_getErrorMessage
(
    xme_hal_xml_documentHandle_t documentHandle,
    char* const buffer,
    size_t size
);

/**
 * \brief Resets any pending error message for the given XML document.
 *
 * \param[in] documentHandle Handle of the document to reset the error
 *            information for.
 */
void
xme_hal_xml_resetErrorMessage
(
    xme_hal_xml_documentHandle_t documentHandle
);

/**
 * \brief Retrieves the root element for the given document.
 *
 * \param[in] documentHandle Handle of the document to retrieve root element
 *            handle for.
 *
 * \deprectaed This function is deprecated. Iterate over the document instead
 *             by calling xme_hal_xml_acceptDocumentVisitor() directly.
 *
 * \return Root element handle for the given document handle. If the given
 *         document handle is invalid or no root element exists in the
 *         document, XME_HAL_XML_INVALID_ELEMENT_HANDLE is returned.
 */
xme_hal_xml_elementHandle_t
xme_hal_xml_getRootElement
(
    xme_hal_xml_documentHandle_t documentHandle
);

/**
 * \brief Arranges for a visitor to visit an XML document.
 *
 * \param[in] documentHandle Handle of the document to visit.
 * \param[in] visitorCallbacks Structure with visitor callback functions that
 *            are called during the visit. Set unused items to NULL to prevent
 *            it from being called during the visit.
 * \param[in] userData Address of user-defined memory that is passed to the
 *            visitor callback functions.
 *
 * \retval XME_STATUS_SUCCESS if the document has been successfully visited.
 * \retval XME_STATUS_ABORTED if the leave document visitor callback was set
 *         and it returned zero.
 * \retval XME_STATUS_INVALID_HANDLE if documentHandle was invalid.
 * \retval XME_STATUS_INVALID_CONFIGURATION if a parsing error occurred.
 */
xme_status_t
xme_hal_xml_acceptDocumentVisitor
(
    xme_hal_xml_documentHandle_t documentHandle,
    xme_hal_xml_visitorCallbacks_t visitorCallbacks,
    void* userData
);

/**
 * \brief Arranges for a visitor to visit an XML element.
 *
 * \param[in] elementHandle Handle of the element to visit.
 * \param[in] visitorCallbacks Structure with visitor callback functions that
 *            are called during the visit. Set unused items to NULL to prevent
 *            it from being called during the visit.
 *            Notice that the enterDocument and leaveDocument callbacks are
 *            never called when visiting elements using this function.
 * \param[in] userData Address of user-defined memory that is passed to the
 *            visitor callback functions.
 *
 * \retval XME_STATUS_SUCCESS if the document has been successfully visited.
 * \retval XME_STATUS_ABORTED if the leave element visitor callback was set
 *         and it returned zero.
 * \retval XME_STATUS_INVALID_HANDLE if elementHandle was invalid.
 * \retval XME_STATUS_INVALID_CONFIGURATION if a parsing error occurred.
 */
xme_status_t
xme_hal_xml_acceptElementVisitor
(
    xme_hal_xml_elementHandle_t elementHandle,
    xme_hal_xml_visitorCallbacks_t visitorCallbacks,
    void* userData
);

/**
 * \brief Returns the name of the element represented by the given handle.
 *
 * \param[in] elementHandle Handle of the element to return the name for.
 *
 * \return Name of the element represented by the given handle.
 *         If elementHandle was invalid, NULL is returned.
 */
const char*
xme_hal_xml_getElementName
(
    xme_hal_xml_elementHandle_t elementHandle
);

/**
 * \brief Returns the handle of the next attribute after the one represented
 *        by the given handle.
 *
 * \param[in] attributeHandle Handle of the attribute to return the next
 *            attribute handle for.
 *
 * \return Handle of the next attribute following the one represented by
 *         attributeHandle. If attributeHandle was invalid or was the last
 *         attribute in the respective element, XME_HAL_XML_INVALID_ATTRIBUTE_HANDLE
 *         is returned.
 */
xme_hal_xml_attributeHandle_t
xme_hal_xml_getNextAttribute
(
    xme_hal_xml_attributeHandle_t attributeHandle
);

/**
 * \brief Returns the name of the attribute represented by the given handle.
 *
 * \param[in] attributeHandle Handle of the attribute to return the name for.
 *
 * \return Name of the attribute represented by the given handle.
 *         If attributeHandle was invalid, NULL is returned.
 */
const char*
xme_hal_xml_getAttributeName
(
    xme_hal_xml_attributeHandle_t attributeHandle
);

/**
 * \brief Returns the value of the attribute represented by the given handle.
 *
 * \param[in] attributeHandle Handle of the attribute to return the value for.
 *
 * \return String representation of the value of the attribute represented by
 *         the given handle. If attributeHandle was invalid, NULL is returned.
 */
const char*
xme_hal_xml_getAttributeValue
(
    xme_hal_xml_attributeHandle_t attributeHandle
);

/**
 * \brief Returns the attribute value as a Boolean value.
 *
 * \details A value of zero is considered false, all other integer numbers
 *          are considered true. In addition, the string literals "true"
 *          and "false" (case sensitive) are recognized.
 *
 * \param[in] attributeHandle Attribute handle to retrieve the value for.
 * \param[in,out] value Address of a variable to initialize with the value of
 *                the attribute cast to bool (zero or one).
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if attributeHandle was invalid.
 * \retval XME_STATUS_INVALID_PARAMETER if value was NULL.
 * \retval XME_STATUS_NO_SUCH_VALUE if the attribute value could not be cast
 *         to bool.
 */
xme_status_t
xme_hal_xml_getAttributeValueAsBool
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    int8_t* const value
);

/**
 * \brief Returns the attribute value as a signed integer value.
 *
 * \param[in] attributeHandle Attribute handle to retrieve the value for.
 * \param[in,out] value Address of a variable to initialize with the value of
 *                the attribute cast to signed integer.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if attributeHandle was invalid.
 * \retval XME_STATUS_INVALID_PARAMETER if value was NULL.
 * \retval XME_STATUS_NO_SUCH_VALUE if the attribute value could not be cast
 *         to signed integer.
 */
xme_status_t
xme_hal_xml_getAttributeValueAsInt
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    int* const value
);

/**
 * \brief Returns the attribute value as an unsigned integer value.
 *
 * \param[in] attributeHandle Attribute handle to retrieve the value for.
 * \param[in,out] value Address of a variable to initialize with the value of
 *                the attribute cast to unsigned integer.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if attributeHandle was invalid.
 * \retval XME_STATUS_INVALID_PARAMETER if value was NULL.
 * \retval XME_STATUS_NO_SUCH_VALUE if the attribute value could not be cast
 *         to unsigned integer.
 */
xme_status_t
xme_hal_xml_getAttributeValueAsUnsignedInt
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    unsigned int* const value
);

/**
 * \brief Returns the attribute value as a 64-bit integer value.
 *
 * \param[in] attributeHandle Attribute handle to retrieve the value for.
 * \param[in,out] value Address of a variable to initialize with the value of
 *                the attribute cast to 64-bit integer.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if attributeHandle was invalid.
 * \retval XME_STATUS_INVALID_PARAMETER if value was NULL.
 * \retval XME_STATUS_NO_SUCH_VALUE if the attribute value could not be cast
 *         to 64-bit integer.
 */
xme_status_t
xme_hal_xml_getAttributeValueAsInt64
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    int64_t* const value
);

/**
 * \brief Returns the attribute value as a 64-bit unsigned integer value.
 *
 * \param[in] attributeHandle Attribute handle to retrieve the value for.
 * \param[in,out] value Address of a variable to initialize with the value of
 *                the attribute cast to 64-bit unsigned integer.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if attributeHandle was invalid.
 * \retval XME_STATUS_INVALID_PARAMETER if value was NULL.
 * \retval XME_STATUS_NO_SUCH_VALUE if the attribute value could not be cast
 *         to 64-bit unsigned integer.
 */
xme_status_t
xme_hal_xml_getAttributeValueAsUnsignedInt64
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    uint64_t* const value
);

/**
 * \brief Returns the attribute value as a single-precidion floating-point
 *        value.
 *
 * \param[in] attributeHandle Attribute handle to retrieve the value for.
 * \param[in,out] value Address of a variable to initialize with the value of
 *                the attribute cast to single-precision floating-point.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if attributeHandle was invalid.
 * \retval XME_STATUS_INVALID_PARAMETER if value was NULL.
 * \retval XME_STATUS_NO_SUCH_VALUE if the attribute value could not be cast
 *         to single-precision floating-point.
 */
xme_status_t
xme_hal_xml_getAttributeValueAsFloat
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    float* const value
);

/**
 * \brief Returns the attribute value as a double-precidion floating-point
 *        value.
 *
 * \param[in] attributeHandle Attribute handle to retrieve the value for.
 * \param[in,out] value Address of a variable to initialize with the value of
 *                the attribute cast to double-precision floating-point.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if attributeHandle was invalid.
 * \retval XME_STATUS_INVALID_PARAMETER if value was NULL.
 * \retval XME_STATUS_NO_SUCH_VALUE if the attribute value could not be cast
 *         to double-precision floating-point.
 */
xme_status_t
xme_hal_xml_getAttributeValueAsDouble
(
    xme_hal_xml_attributeHandle_t attributeHandle,
    double* const value
);

/**
 * @}
 */

XME_EXTERN_C_END

#endif // #ifndef XME_HAL_XML_H
