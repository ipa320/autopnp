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
 * $Id: attribute.h 5215 2013-09-27 14:29:25Z ruiz $
 */

/**
 * \file
 *         Attribute abstraction.
 *
 */

#ifndef XME_CORE_DIRECTORY_ATTRIBUTE_H
#define XME_CORE_DIRECTORY_ATTRIBUTE_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"
#include "xme/hal/include/table.h"
#include "xme/defines.h"

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_core_directory_attributeSetHandle_t
 *
 * \brief  Locally valid handle for a topic attribute set specification.
 *         This handle refers to an object that a set of key/value pairs that
 *         describe the attributes associated with a topic. The topic attributes
 *         can also be used to filter topics.
 */
typedef enum
{
    XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET = 0, ///< Empty topic attribute.
    XME_CORE_ATTRIBUTE_MAX_ATTRIBUTE_SET_HANDLE = XME_MAX_SYSTEM_VALUE ///< Largest possible topic attribute set handle.
} xme_core_directory_attributeSetHandle_t;

/**
 * \typedef xme_core_directory_attributeHandle_t
 *
 * \brief  Locally valid handle for a topic attribute specification.
 *         This handle refers to an object that represents a single attribute
 *         definition. It is used in conjunction with attribute set iterators.
 */
typedef enum
{
    XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE = 0, ///< Invalid attribute handle.
    XME_CORE_ATTRIBUTE_MAX_ATTRIBUTE_HANDLE = XME_MAX_SYSTEM_VALUE ///< Largest possible topic attribute handle.
} xme_core_directory_attributeHandle_t;

#if 0
/**
 * \typedef xme_core_md_topicFilterHandle_t
 *
 * \brief  Locally valid handle to a topic filter specification.
 *         This handle refers to an object that defines key/value pairs.
 */
typedef uint16_t xme_core_md_topicFilterHandle_t;
#endif 

/**
 * \enum xme_core_directory_attribute_filterOperation_t
 *
 * \brief  Operators for topic attributes comparisons.
 */
typedef enum
{
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID = 0, ///< Invalid filter operation. 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,       ///< Filter operation is equality. 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER,     ///< Filter operation is greater (only accepting values greater than the value). 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER_OR_EQUAL, ///< Filter operation is greater (only accepting values equal or greater than the value). 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS,     ///< Filter operation is smaller (only accepting values smaller than the value). 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS_OR_EQUAL, ///< Filter operation is greater (only accepting values equal or smaller than the value). 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_UNEQUAL, ///< Filter operation is inequality. 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EXISTS, ///< Filter operation is inequality. 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_NOT_EXISTS ///< Filter operation is inequality. 
} xme_core_directory_attribute_filterOperation_t;

/**
 * \enum xme_core_directory_attribute_datatype_t
 *
 * \brief  Datatype associated to the datatype.
*/
typedef enum
{
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID = 0, ///< Invalid datatype.
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING, ///< String datatype.
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, ///< Numeric datatype (positive or negative integer).
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED, ///< Unsigned datatype (positive integer).
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL, ///< Decimal datatype (double-precision floating-point number).
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY, ///< Array datatype.
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX ///< Complex datatype.
} xme_core_directory_attribute_datatype_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the attribute component.
 * \details Exactly one component of this type must be present on every node.
 *          The attribute component stores all attributes of the corresponding
 *          node. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute component has been propoerly
 *         initialized. 
 * \retval XME_STATUS_OUT_OF_RESOURCES if initialization failed. 
 */
xme_status_t
xme_core_directory_attribute_init(void);

/**
 * \brief  Frees all resources occupied by the meta data component.
 * \details The associated variables are freed. 
 */
void
xme_core_directory_attribute_fini(void);

/**
 * \brief  Allocates a new locally valid topic attribute set handle.
 *         Topic attribute set handles can be used to specify <key,value> pairs
 *         for topics as well as filter criteria for topic subscriptions.
 *
 * \return Newly allocated topic attribute set handle.
 */
xme_core_directory_attributeSetHandle_t
xme_core_directory_attribute_createAttributeSet(void);

/**
 * \brief  Removes an entry from the attribute repository. 
 * \details Removes a complete entry from the attribute repository, freeing 
 *          the associated memory. 
 *
 * \param attributeSetHandle the attribute set handle. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute set is successfully removed 
 *         from the repository.
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_NOT_FOUND if the provided handle is not part of the handles. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot complete the removal operation. 
 */
xme_status_t
xme_core_directory_attribute_removeAttributeSet
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief  Checks if an attribute set handle is valid.
 *
 * \param  attributeSetHandle Attribute set handle.
 *
 * \retval true if the attribute set handle is a valid handle.
 * \retval false if the attribute set handle is not a valid handle.
 */
bool
xme_core_directory_attribute_isAttributeSetHandleValid
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Adds a new attribute definition to the list.
 *
 * \details Creates a new entry for a new attribute definition.
 *          If the key is already present, it should be checked
 *          the 'overwrite' boolean variable. In case there is an
 *          existing key with the same key, the value is overwritten.
 *          On the contrary, an XME_STATUS_ALREADY_EXISTS is returned.
 *          The operation will be completed if all values are included.
 *          The valueSize determines the number of values to add.
 *          In case of numValues is 1, a primitive datatype is supposed.
 *          In case of numValues is more than 1, an array datatype is supposed.
 *
 * \param attributeSetHandle the handle associated to the attribute set.
 * \param key the attribute key.
 * \param value the value to add.
 * \param numValues the number of values contained in the value parameter.
 * \param valueSize the value size expressed in bytes. If datatype is
 *        XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING, valueSize specifies
 *        the maximum number of characters to copy. If the nul-terminated
 *        string is shorter, only characters up to the first nul character
 *        are considered. Specify zero for this parameter to consider the
 *        complete string until the first nul character.
 * \param datatype the datatype used to specify the value. May be one of the
 *        following constants:
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING: String datatype.
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC: Numeric datatype
 *           (positive or negative integer).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED: Unsigned datatype
 *           (positive integer).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL: Decimal datatype
 *           (double-precision floating-point number).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY: Array datatype.
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX: Complex datatype.
 * \param overwrite a boolean determining the overwriting of the value 
 *        in case of an existing key.
 *
 * \retval XME_STATUS_SUCCESS if the new attribute is successfully stored.
 * \retval XME_STATUS_INVALID_HANDLE if the handle is not a valid handle.
 * \retval XME_STATUS_OUT_OF_RESOURCES if allocation of memory for attribute
 *         is not possible.
 * \retval XME_STATUS_INVALID_PARAMETER if key was undefined, value was NULL,
 *         numValues was NULL, datatype was invalid or valueSize was zero if
 *         datatype was not XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING.
 * \retval XME_STATUS_ALREADY_EXISTS if there is already an entry for that
 *         attribute and the corresponding 'overwrite' flag is set to false.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot complete the operation.
 */
// TODO: What about strings? How is valueSize interpreted?
xme_status_t
xme_core_directory_attribute_addPredefinedAttributeDefinition
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key,
    const void* const value,
    uint16_t numValues,
    size_t valueSize,
    xme_core_directory_attribute_datatype_t datatype,
    bool overwrite
);

/**
 * \brief Removes a attribute definition from the list.
 * \details Looks up for the provided attribute definition and
 *          removes it from the list of associated attributes. 
 *
 * \param attributeSetHandle the handle associated to the attribute set. 
 * \param key the attribute key. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute is successfully removed. 
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_NOT_FOUND if there are not entries for the corresponding key. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot complete the operation. 
 */
xme_status_t
xme_core_directory_attribute_removePredefinedAttributeDefinition
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key
);

/**
 * \brief Adds a new attribute filter to the attribute set.
 * \details Creates a new entry for a new attribute filter.
 *          If the key is already present, it should be checked
 *          the 'overwrite' boolean variable. In case there is an
 *          existing key with the same key, the value is overwritten.
 *          On the contrary, an XME_STATUS_ALREADY_EXISTS is returned.
 *          The operation will be completed if all values are included.
 *          The valueSize determines the number of values to add.
 *          In case of numValues is 1, a primitive datatype is supposed.
 *          In case of numValues is more than 1, an array datatype is supposed.
 *
 * \param attributeSetHandle the handle associated to the attribute set.
 * \param key the attribute key.
 * \param value the value to add.
 * \param numValues the number of values contained in the value parameter.
 * \param valueSize the value size expressed in bytes. If datatype is
 *        XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING, valueSize specifies
 *        the maximum number of characters to copy. If the nul-terminated
 *        string is shorter, only characters up to the first nul character
 *        are considered. Specify zero for this parameter to consider the
 *        complete string until the first nul character.
 * \param datatype the datatype used to specify the value. May be one of the
 *        following constants:
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING: String datatype.
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC: Numeric datatype
 *           (positive or negative integer).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED: Unsigned datatype
 *           (positive integer).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL: Decimal datatype
 *           (double-precision floating-point number).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY: Array datatype.
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX: Complex datatype.
 * \param operation the operation that is going to be used in the filter.
 * \param overwrite a boolean determining the overwriting of the value
 *        in case of an existing key.
 *
 * \retval XME_STATUS_SUCCESS if the new attribute is successfully stored.
 * \retval XME_STATUS_INVALID_HANDLE if the handle is not a valid handle.
 * \retval XME_STATUS_OUT_OF_RESOURCES if allocation of memory for attribute
 *         is not possible.
 * \retval XME_STATUS_INVALID_PARAMETER if key was undefined, value was NULL,
 *         numValues was NULL, datatype was invalid or valueSize was zero if
 *         datatype was not XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING.
 * \retval XME_STATUS_ALREADY_EXISTS if there is already an entry for that
 *         attribute and the corresponding 'overwrite' flag is set to false.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot complete the operation.
 */
xme_status_t
xme_core_directory_attribute_addPredefinedAttributeFilter
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key,
    const void* const value,
    uint16_t numValues,
    size_t valueSize,
    xme_core_directory_attribute_datatype_t datatype,
    xme_core_directory_attribute_filterOperation_t operation,
    bool overwrite
);

/**
 * \brief Removes a attribute filter from the list.
 * \details Looks up for the provided attribute filter and
 *          removes it from the list of associated attributes. 
 *
 * \param attributeSetHandle the handle associated to the attribute set. 
 * \param key the attribute key. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute is successfully removed. 
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_NOT_FOUND if there are not entries for the corresponding key. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot complete the operation. 
 */
xme_status_t
xme_core_directory_attribute_removePredefinedAttributeFilter
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key
);

/**
 * \brief  Gets the number of attributes in the attribute set.
 *
 * \param attributeSetHandle the attribute set handle. 
 *
 * \return the number of attributes in the attribute set.
 */
uint16_t
xme_core_directory_attribute_getAttributeSetCount
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Gets the attribute key datatype.
 *
 * \param key the attribute key. 
 * \param datatype the output attribute datatype. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute datatype was successfully obtained. 
 *         The result is stored at datatype variable. 
 * \retval XME_STATUS_INVALID_PARAMETER if datatype was NULL.
 * \retval XME_STATUS_INVALID_HANDLE if the attribute set handle is not a valid handle. 
 * \retval XME_STATUS_NOT_FOUND if there are no attributes with the corresponding key. 
 */
xme_status_t
xme_core_directory_attribute_getAttributeKeyDatatype
(
    xme_core_attribute_key_t key,
    xme_core_directory_attribute_datatype_t* datatype
);

/**
 * \brief Inits the iterator over the given attribute set handle. 
 *
 * \param attributeSetHandle the attribute set handle. 
 *
 * \retval XME_STATUS_SUCCESS if the iterator has been successfully initialized. 
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_INTERNAL_ERROR if the iterator initialization was not success. 
 */
xme_status_t
xme_core_directory_attribute_initIterator
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Ends the iterator over the given attribute set handle. 
 *
 * \param attributeSetHandle the attribute set handle. 
 *
 * \retval XME_STATUS_SUCCESS if the iterator has been successfully finalized. 
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_INTERNAL_ERROR if the iterator finalization was not success. 
 */
xme_status_t
xme_core_directory_attribute_finiIterator
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);


/**
 * \brief Determines if there are more attributes to explore. 
 *
 * \param attributeSetHandle the attribute set handle. 
 *
 * \retval true if there are more attributes in the attribute set to iterate.
 * \retval false if there are no more attributes to iterate. 
 */
bool
xme_core_directory_attribute_hasNextAttributeHandle
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Get the next attribute handle in the iterator. 
 * \details Get the next attribute handle in the iterator and increments
 *          the position to which is pointing the iterator. 
 *
 * \param[in] attributeSetHandle the attribute set handle. 
 * \param[out] attributeHandle the output attribute handle. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute handle was successfully obtained.
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot complete the operation. 
 */
xme_status_t
xme_core_directory_attribute_getNextAttributeHandle
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_directory_attributeHandle_t* attributeHandle
);

/**
 * \brief Gets the attribute key. 
 * \details Given the attribute set handle and the attribute handle,
 *          it provides the attribute key. 
 *
 * \param[in] attributeSetHandle the attribute set handle. 
 * \param[in] attributeHandle the attribute handle. 
 * \param[out] key the attribute key. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute key is obtained and
 *         stored in 'key' variable.
 * \retval XME_STATUS_INVALID_PARAMETER if key was NULL.
 * \retval XME_STATUS_INVALID_HANDLE if one of the given handles was invalid.
 */
xme_status_t
xme_core_directory_attribute_getAttributeKey
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_directory_attributeHandle_t attributeHandle,
    xme_core_attribute_key_t* key
);

/**
 * \brief Copies the attribute value into the given buffer.
 *
 * \details The size of the output buffer is indicated by the bufferSize
 *          parameter. If the given buffer was too small, the buffer content
 *          is truncated and XME_STATUS_BUFFER_TOO_SMALL is returned.
 *          If valueSize is set to a non-NULL value, the value it points to
 *          is set to the minimum buffer size required to successfully copy
 *          the complete value.
 *
 * \note    For safety reasons, string values (i.e., attributes of type
 *          XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING) are always
 *          zero-terminated. This means that the buffer size needs to be at
 *          least one character larger than the number of characters in the
 *          string value. If the buffer is too small, the string is truncated
 *          in a way such that the last character in the buffer will be a zero.
 *
 * \note    If the buffer is larger than is required to store the value, the
 *          content of the unused buffer elements is indeterminate.
 *
 * \param[in] attributeSetHandle Attribute set handle.
 * \param[in] key Attribute key identifying the value to copy.
 * \param[out] buffer Pointer to a block of memory of at least the given size
 *             where the value is copied to.
 * \param[in] bufferSize Size of the buffer.
 * \param[out] valueSize If this parameter is non-NULL, the value it points to
 *             is set to the size of the value in bytes independent of how many
 *             bytes are actually copied to the destination buffer. Notice that
 *             for string values, this (i.e., attributes of type
 *             XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING), this includes the
 *             terminating nul character.
 *
 * \retval XME_STATUS_SUCCESS if the buffer was large enough and the attribute
 *         value was succesfully copied to the destination buffer.
 * \retval XME_STATUS_BUFFER_TOO_SMALL if the given buffer was too small to
 *         store the complete value. If valueSize was non-NULL, the value that
 *         it points to indicates the minimum buffer size required to copy the
 *         complete value.
 * \retval XME_STATUS_INVALID_HANDLE if either the attributeSetHandle or the
 *         key were invalid.
 * \retval XME_STATUS_UNSUPPORTED if the attribute's data type is not supported.
 */
xme_status_t
xme_core_directory_attribute_getAttributeValue
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key,
    void* buffer,
    uint16_t bufferSize,
    uint16_t* valueSize
);

/**
 * \brief  Checks if the provided attribute set is composed of filter attributes.
 *
 * \param attributeSetHandle the attribute set handle. 
 *
 * \retval true if the attribute set is composed of filter attributes.
 * \retval false if the attribute set is not fully composed of filter attributes.
 */
bool
xme_core_directory_attribute_isFilter
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief  Checks if the provided attribute set is composed of definition attributes.
 *
 * \param attributeSetHandle the attribute set handle. 
 *
 * \retval true if the attribute set is composed of definition attributes.
 * \retval false if the attribute set is not fully composed of definition attributes.
 */
bool
xme_core_directory_attribute_isDefinition
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief  Determines whether an attribute definition matches an attribute
 *         filter.
 *
 * \details An attribute definition matches an attribute filter if for every
 *          filter condition there exists a definition (except if the filter
 *          condition has the operator "not exists") and, if the filter
 *          operator is binary, the filter value matches.
 *
 * \note    Both the filterAttributeSetHandle and the defAttributeSetHandle
 *          parameters accept a value of XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET
 *          as well as attribute handles that point to empty attribute sets
 *          (both values lead to the same effects).
 *          If XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET or an empty attribute set
 *          is specified for filterAttributeSetHandle, all attribute
 *          definitions match.
 *          If XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET or an empty attribute set
 *          is specified for defAttributeSetHandle, then this matches only an
 *          filterAttributeSetHandle set to XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET
 *          or to an empty attribute set.
 *          Otherwise, all attribute filter definitions in
 *          filterAttributeSetHandle need to be backed up by matching attribute
 *          definitions in defAttributeSetHandle in order to return a positive
 *          result.
 *
 * \param[in] filterAttributeSetHandle Attribute filter set handle.
 * \param[in] defAttributeSetHandle Attribute definition set handle.
 * \param[out] matches Address of a variable that is set to the result
 *             of the matching operation. Is is set to a nonzero value
 *             if the definition matches the filter and to zero otherwise.
 *             This parameter should not be NULL.
 *
 * \retval XME_STATUS_SUCCESS if the fact whether the definition matches or
 *         not could be successfully determined.
 * \retval XME_STATUS_INVALID_PARAMETER if matches was NULL.
 * \retval XME_STATUS_INVALID_HANDLE if at least one of the attribute set
 *         handles was invalid, for example if filterAttributeSetHandle
 *         indicates an attribute definition or defAttributeSetHandle
 *         indicates an attribute filter. Notice that as indicated above,
 *         values such as XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET or empty
 *         attribute sets are not considered invalid.
 */
xme_status_t
xme_core_directory_attribute_isFilterMatchingDefinition
(
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle, 
    xme_core_directory_attributeSetHandle_t defAttributeSetHandle,
    int* matches
);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DIRECTORY_ATTRIBUTE_H
