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
 * $Id: attribute.h 5630 2013-10-25 13:52:29Z ruiz $
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
#include "xme/defines.h"

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_core_directory_attributeSetHandle_t
 *
 * \brief Locally valid handle for a topic attribute set specification.
 *        An attribute set is a collection of key/value pairs, where each pair
 *        represents an attribute definition (publication) or filter (subscription).
 */
typedef enum
{
    XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET = 0, ///< Empty topic attribute.
    XME_CORE_ATTRIBUTE_MAX_ATTRIBUTE_SET_HANDLE = XME_MAX_SYSTEM_VALUE ///< Largest possible topic attribute set handle.
} xme_core_directory_attributeSetHandle_t;

/**
 * \typedef xme_core_directory_attributeHandle_t
 *
 * \brief Locally valid handle for a topic attribute specification.
 *        This handle refers to an object that represents a single attribute
 *        definition. It is used in conjunction with attribute set iterators.
 */
typedef enum
{
    XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE = 0, ///< Invalid attribute handle.
    XME_CORE_ATTRIBUTE_MAX_ATTRIBUTE_HANDLE = XME_MAX_SYSTEM_VALUE ///< Largest possible topic attribute handle.
} xme_core_directory_attributeHandle_t;

/**
 * \enum xme_core_directory_attribute_filterOperation_t
 *
 * \brief Operations for topic attribute filters.
 */
typedef enum
{
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID = 0, ///< Invalid filter operation. 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,       ///< Filter operation is equality. 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER,     ///< Filter operation is greater (only accepting values greater than the value). 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER_OR_EQUAL, ///< Filter operation is greater or equal (only accepting values equal or greater than the value). 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS,     ///< Filter operation is less (only accepting values less than the value). 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS_OR_EQUAL, ///< Filter operation is less or equal (only accepting values equal or less than the value). 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_UNEQUAL,   ///< Filter operation is inequality. 
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EXISTS,    ///< Filter operation for existance (checks whether a topic data packet has a value for a given attribute).
    XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_NOT_EXISTS ///< Filter operation for non-existance (checks whether a topic data packet does not have a value for a given attribute).
} xme_core_directory_attribute_filterOperation_t;

/**
 * \enum xme_core_directory_attribute_datatype_t
 *
 * \brief Datatype of an attribute value.
*/
typedef enum
{
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID = 0, ///< Invalid datatype.
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING, ///< String datatype.
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, ///< Numeric datatype (positive or negative integer). Maximum value range is int64_t.
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED, ///< Unsigned datatype (positive integer). Maximum value range is uint64_t.
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL, ///< Decimal datatype (double-precision floating-point number).
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY, ///< Array datatype.
    XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX ///< Complex datatype.
} xme_core_directory_attribute_datatype_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the attribute component.
 *
 * \details Exactly one component of this type must be present on every node.
 *          The attribute component stores all attributes of the corresponding
 *          node. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute component has been properly
 *         initialized. 
 * \retval XME_STATUS_OUT_OF_RESOURCES if initialization failed. 
 */
xme_status_t
xme_core_directory_attribute_init(void);

/**
 * \brief Frees all resources occupied by the attribute component.
 */
void
xme_core_directory_attribute_fini(void);

/**
 * \brief Register an attribute descriptor list for a given topic key.
 *
 * \details The registered attribute descriptor lists can be retrieved
 *          later via xme_core_directory_attribute_getAttributeDescriptorList.
 *          Only a single list is allowed per topic.
 *          The attribute descriptors of the given list will be copied
 *          into the registry.
 *
 * \param topic Topic for which to register the list.
 * \param attributeDescriptorList The registered attribute descriptor list.
 * \param overwrite When this is true an already existing entry will be
 *        overwritten. When false and an entry already exists, then 
 *        XME_STATUS_ALREADY_EXIST is returned and the operation is canceled.
 *
 * \retval XME_STATUS_SUCCESS when operation completed without any problems.
 * \retval XME_STATUS_INVALID_PARAMETER when given topic is XME_CORE_TOPIC_INVALID_TOPIC
 *         or given attributeDescriptorList is NULL.
 * \retval XME_STATUS_ALREADY_EXIST when overwrite is false and an entry for the given
 *         topic already exists.
 * \retval XME_STATUS_OUT_OF_RESOURCES when there is not enough memory to register
 *         the list.
 */
xme_status_t
xme_core_directory_attribute_registerAttributeDescriptorList
(
    xme_core_topic_t topic,
    const xme_core_attribute_descriptor_list_t* const attributeDescriptorList,
    bool overwrite
);

/**
 * \brief Get the attribute descriptor list for a given topic.
 *
 * \details The attribute descriptor list defines which attributes
 *          can be attached to a given topic.
 *          Attribute descriptor lists need to registered via
 *          xme_core_directory_attribute_registerAttributeDescriptorList.
 *
 * \param[in] topic Given topic for which to find a registered attribute descriptor list.
 * \param[out] attributeDescriptorList On a success the pointer will contain the found
 *             attribute descriptor list.
 *
 * \retval XME_STATUS_SUCCESS when operation completed without any problems.
 * \retval XME_STATUS_INVALID_PARAMETER when given topic is XME_CORE_TOPIC_INVALID_TOPIC, or
 *         given attributeDescriptorList is NULL.
 * \retval XME_STATUS_NOT_FOUND when no registered list has been found for the given topic.
 */
xme_status_t
xme_core_directory_attribute_getAttributeDescriptorList
(
    xme_core_topic_t topic,
    xme_core_attribute_descriptor_list_t* attributeDescriptorList
);

/**
 * \brief Allocates a new attribute set and returns a locally valid handle to it.
 *        Attribute sets contain <key,value> pairs that define attribute definitions
 *        (for publications) or attribute filters (for subscriptions).
 *
 * \return Handle to newly created attribute set, or XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET
 *         when creation failed.
 */
xme_core_directory_attributeSetHandle_t
xme_core_directory_attribute_createAttributeSet(void);

/**
 * \brief Removes an entry from the attribute set repository.
 *
 * \details Removes a complete entry from the attribute repository, freeing 
 *          the associated memory. 
 *
 * \param attributeSetHandle The attribute set handle. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute set is successfully removed 
 *         from the repository.
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_NOT_FOUND if the provided handle cannot be found in the repository.
 * \retval XME_STATUS_INTERNAL_ERROR if the removal operation cannot be completed for some other reason.
 */
xme_status_t
xme_core_directory_attribute_removeAttributeSet
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Checks if an attribute set handle is valid.
 * \details An attribute set handle is valid if it corresponds 
 *          to a valid row handle of the table containing all
 *          attribute sets. This is specially important when 
 *          we should calculate at logical route manager level
 *          the matching between attribute definitions and 
 *          attribute filters. 
 *
 * \param attributeSetHandle Attribute set handle.
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
 * \brief Adds a new attribute definition to the given attribute set.
 *
 * \details Creates a new entry for an attribute definition.
 *          If the key is already present in the set, behaviour depends on the 'overwrite' flag.
 *          When overwrite is set and there is an existing entry with the same key,
 *          the existing value is overwritten. Otherwise, XME_STATUS_ALREADY_EXISTS is
 *          returned.
 * \note If all input parameters are valid and the key corresponds to the right data type
 *       (if previously registered), the attribute definition storage will take place. 
 *
 * \param attributeSetHandle The handle associated to the attribute set.
 * \param key The attribute key.
 * \param value The value to add.
 * \param numValues The number of values contained in the value parameter.
 *        Only meaningful for datatype XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY.
 *        For other types this must be set to 1.
 *        Must never be 0.
 * \param valueSize The value size expressed in bytes. If datatype is
 *        XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING, valueSize specifies
 *        the maximum number of characters to copy. If the null-terminated
 *        string is shorter, only characters up to the first null character
 *        are considered. Specify zero for this parameter to consider the
 *        complete string until the first null character.
 * \param datatype The datatype used to specify the value. May be one of the
 *        following constants:
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING: String datatype.
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC: Numeric datatype
 *           (positive or negative integer). Maximum value range is int64_t.
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED: Unsigned datatype
 *           (positive integer). Maximum value range is uint64_t.
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL: Decimal datatype
 *           (double-precision floating-point number).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY: Array datatype (still unsupported).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX: Complex datatype (still unsupported).
 * \param overwrite A boolean determining the overwriting of the value 
 *        in case of an existing key.
 *
 * \retval XME_STATUS_SUCCESS if the new attribute is successfully stored.
 * \retval XME_STATUS_INVALID_HANDLE if the given handle is not a valid handle.
 * \retval XME_STATUS_OUT_OF_RESOURCES if allocation of memory for attribute
 *         is not possible.
 * \retval XME_STATUS_INVALID_PARAMETER if key was undefined, value was NULL,
 *         numValues was NULL, datatype was invalid or valueSize was zero if
 *         datatype was not XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING.
 * \retval XME_STATUS_ALREADY_EXISTS if there is already an entry for that
 *         attribute and the 'overwrite' flag is set to false.
 * \retval XME_STATUS_INTERNAL_ERROR if the operation cannot be completed.
 */
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
 * \brief Removes an attribute definition from the given attribute set.
 *
 * \details Looks up for the provided attribute definition and
 *          removes it from the list of associated attributes. 
 *
 * \param attributeSetHandle The handle associated to the attribute set. 
 * \param key The attribute key. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute is successfully removed. 
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_NOT_FOUND if there are no entries for the corresponding key. 
 * \retval XME_STATUS_INTERNAL_ERROR if the operation cannot be completed. 
 */
xme_status_t
xme_core_directory_attribute_removePredefinedAttributeDefinition
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key
);

/**
 * \brief Adds a new attribute filter to the attribute set.
 *
 * \details Creates a new entry for an attribute filter.
 *          If the key is already present, behaviour depends on the 'overwrite' flag.
 *          When overwrite is set and there is an existing entry with the same key,
 *          the existing value is overwritten. Otherwise, XME_STATUS_ALREADY_EXISTS is
 *          returned.
 * \note If all input parameters are valid and the key corresponds to the right data type
 *       (if previously registered), the attribute definition storage will take place. 
 *
 * \param attributeSetHandle The handle associated to the attribute set.
 * \param key The attribute key.
 * \param value The value to add.
 * \param numValues The number of values contained in the value parameter.
 *        Only meaningful for datatype XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY.
 *        For other types this must be set to 1.
 *        Must never be 0.
 * \param valueSize The value size expressed in bytes. If datatype is
 *        XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING, valueSize specifies
 *        the maximum number of characters to copy. If the null-terminated
 *        string is shorter, only characters up to the first null character
 *        are considered. Specify zero for this parameter to consider the
 *        complete string until the first null character.
 * \param datatype The datatype used to specify the value. May be one of the
 *        following constants:
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING: String datatype.
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC: Numeric datatype
 *           (positive or negative integer).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED: Unsigned datatype
 *           (positive integer).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL: Decimal datatype
 *           (double-precision floating-point number).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY: Array datatype (still unsupported).
 *         - XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX: Complex datatype (still unsupported).
 * \param operation The operation that is going to be used in the filter.
 * \param overwrite A boolean determining the overwriting of the value
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
 * \retval XME_STATUS_INTERNAL_ERROR if the operation cannot be completed.
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
 * \brief Removes an attribute filter from the given attribute set.
 *
 * \details Looks up for the provided attribute filter 
 *          matching the attribute key and removes it 
 *          from the list of associated attributes. 
 *
 * \param attributeSetHandle The handle associated to the attribute set. 
 * \param key The attribute key to remove. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute matching a key is successfully removed. 
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_NOT_FOUND if there are not attribute entries for the corresponding key. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot complete the operation. 
 */
xme_status_t
xme_core_directory_attribute_removePredefinedAttributeFilter
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key
);

/**
 * \brief Gets the number of attributes (definitions and filters) in the attribute set.
 *
 * \param attributeSetHandle The attribute set handle. 
 *
 * \return The number of attributes in the attribute set.
 */
uint16_t
xme_core_directory_attribute_getAttributeSetCount
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Gets the datatype of the attribute with the given key.
 *
 * \param[in] key The attribute key. 
 * \param[out] datatype The output parameter where the determined datatype will be written to. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute datatype was successfully obtained. 
 *         The result is stored in the datatype variable. 
 * \retval XME_STATUS_INVALID_PARAMETER if datatype was NULL.
 * \retval XME_STATUS_NOT_FOUND if the key is not registered in the attribute key map. 
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
 * \param attributeSetHandle The attribute set handle. 
 *
 * \retval XME_STATUS_SUCCESS if the iterator has been successfully initialized. 
 * \retval XME_STATUS_INVALID_HANDLE if the provided handle is not a valid handle. 
 * \retval XME_STATUS_INTERNAL_ERROR if the iterator initialization was not successful,
 *         for example because the iterator is already in use.
 */
xme_status_t
xme_core_directory_attribute_initIterator
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Ends the iterator over the given attribute set handle. 
 *
 * \param attributeSetHandle The attribute set handle. 
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
 * \brief Determines if there are more attributes to explore in the iterator
 *        of the given attribute set.
 *
 * \param attributeSetHandle The attribute set handle. 
 *
 * \retval true if there are more attributes in the attribute set to iterate.
 * \retval false if there are no more attributes to iterate, or the iterator
 *         is not initialized. 
 */
bool
xme_core_directory_attribute_hasNextAttributeHandle
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Get the next attribute handle in the iterator. 
 *
 * \details Get the next attribute handle in the iterator and increments
 *          the iterator position.
 *
 * \param[in] attributeSetHandle The attribute set handle. 
 * \param[out] attributeHandle The output attribute handle. 
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
 *
 * \details Given the attribute set handle and the attribute handle,
 *          it provides the attribute key. 
 *
 * \param[in] attributeSetHandle The attribute set handle. 
 * \param[in] attributeHandle The attribute handle. 
 * \param[out] key The attribute key. 
 *
 * \retval XME_STATUS_SUCCESS if the attribute key is obtained and
 *         stored in 'key' variable.
 * \retval XME_STATUS_INVALID_PARAMETER if key was NULL.
 * \retval XME_STATUS_INVALID_HANDLE if oen of the given handle was invalid.
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
 *          Only valueSize bytes are actually written in the buffer.
 *
 * \note For safety reasons, string values (i.e., attributes of type
 *       XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING) are always
 *       zero-terminated. This means that the buffer size needs to be at
 *       least one character larger than the number of characters in the
 *       string value. If the buffer is too small, the string is truncated
 *       in a way such that the last character in the buffer will be a zero.
 *
 * \note If the buffer is larger than is required to store the value, the
 *       content of the unused buffer elements is indeterminate.
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
 *             terminating null character.
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
// TODO: What about getting array attribute values? Don't we also need a
//       uint16_t* numValues output parameter?
xme_status_t
xme_core_directory_attribute_getAttributeValue
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key,
    void* buffer,
    uint16_t bufferSize,
    uint32_t* valueSize
);

/**
 * \brief Checks if the provided attribute set is only composed of filter attributes.
 *
 * \param attributeSetHandle The attribute set handle. 
 *
 * \retval true if the attribute set is only composed of attribute filters.
 * \retval false if the attribute set is not fully composed of attribute filters.
 */
bool
xme_core_directory_attribute_isFilter
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Checks if the provided attribute set is only composed of definition attributes.
 *
 * \param attributeSetHandle The attribute set handle. 
 *
 * \retval true if the attribute set is only composed of attribute definitions.
 * \retval false if the attribute set is not fully composed of attribute definitions.
 */
bool
xme_core_directory_attribute_isDefinition
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
);

/**
 * \brief Determines whether an attribute definition matches an attribute
 *        filter.
 *
 * \details An attribute definition matches an attribute filter if for every
 *          filter condition there exists a definition (except if the filter
 *          condition has the operator "not exists") and, if the filter
 *          operator is binary, the filter value matches.
 *
 * \note Both the filterAttributeSetHandle and the defAttributeSetHandle
 *       parameters accept a value of XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET
 *       as well as attribute handles that point to empty attribute sets
 *       (both values lead to the same effects).
 *       If XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET or an empty attribute set
 *       is specified for filterAttributeSetHandle, all attribute
 *       definitions match.
 *       If XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET or an empty attribute set
 *       is specified for defAttributeSetHandle, then this matches only an
 *       filterAttributeSetHandle set to XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET
 *       or to an empty attribute set. // TODO: What about non-existance filters?
 *       Otherwise, all attribute filter definitions in filterAttributeSetHandle
 *       need to be backed up by matching attribute definitions in
 *       defAttributeSetHandle in order to return a positive result.
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
