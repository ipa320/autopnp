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
 * $Id: attributeInternalMethods.h 6459 2014-01-23 16:50:54Z geisinger $
 */

/**
 * \file
 *         Attribute internal methods.
 *
 */

#ifndef XME_CORE_DIRECTORY_ATTRIBUTEINTERNALMETHODS_H
#define XME_CORE_DIRECTORY_ATTRIBUTEINTERNALMETHODS_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/directory/include/attributeInternalTypes.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Checks if a given key is already registered. 
 * 
 * \param key The key to check.
 *
 * \retval true if the provided key is registered. 
 * \retval false if the provided key is not registered. 
 */
bool
xme_core_directory_attribute_isKeyRegistered
(
    xme_core_attribute_key_t key
);

/**
 * \brief Register a new key in the register. 
 *
 * \details To register a new key, we need to provide the attribute key 
 *          and the datatype associated to that key. This allows the association
 *          of a key to a datatype. 
 *          Keys must be globally unique (not only per topic).
 *
 * \note Before registering a key, it is necessary to check if the key 
 *       is already registered using ::xme_core_directory_attribute_isKeyRegistered
 * 
 * \param attributeKey The key to register.
 * \param datatype The datatype associated to the attribute key. 
 * 
 * \retval XME_STATUS_SUCCESS if the key was successfully registered. 
 * \retval XME_STATUS_ALREADY_EXISTS if the entry for that key already exists. 
 * \retval XME_STATUS_OUT_OF_RESOURCES if the key cannot be registered due to out of resources. 
 * \retval XME_STATUS_INVALID_PARAMETER if provided parameters are invalid. 
 * \retval XME_STATUS_INTERNAL_ERROR if there was an error during operation. 
 */
xme_status_t
xme_core_directory_registerKey
(
    xme_core_attribute_key_t attributeKey, 
    xme_core_directory_attribute_datatype_t datatype
);

/**
 * \brief Checks if a given attribute value is matching 
 *        comparing the filter with the definition. 
 *
 * \details For a given pair of filter and definition attributes, 
 *          with the same key, it is checked if the attribute
 *          value in definition matches with the attribute filter
 *          value in the filter.
 *
 * \note Both attributes should have the same key. 
 *
 * \param filterAttribute The filter attribute. 
 * \param defAttribute The definition attribute. 
 * \param datatype The datatype to compare. 
 *
 * \retval true if the definition attribute matches with filter attribute. 
 * \retval false if the definition attribute does not match with filter attribute. 
 */
bool
xme_core_directory_attribute_matchesAttributeValue
(
    const xme_core_directory_attribute_t* const filterAttribute,
    const xme_core_directory_attribute_t* const defAttribute,
    xme_core_directory_attribute_datatype_t datatype
);

/**
 * \brief Compares two string values. 
 * 
 * \param operation The comparison operation. 
 * \param filterValue The pointer to the filter string. 
 * \param filterValueSize The size of the string.
 * \param defValue The pointer to the definition string. 
 * \param defValueSize The size of the string. 
 *
 * \retval true if string values match wrt operation. 
 * \retval false if string values do not match wrt operation. 
 */
bool
xme_core_directory_attribute_compareStringValues
(
    xme_core_directory_attribute_filterOperation_t operation,
    const char* filterValue,
    uint32_t filterValueSize,
    const char* defValue,
    uint16_t defValueSize
);

/**
 * \brief Compares two signed numeric values. 
 * 
 * \param operation The comparison operation. 
 * \param filterValue The signed numeric filter value. 
 * \param defValue The signed numeric defintion value. 
 *
 * \retval true if signed numeric values match wrt operation. 
 * \retval false if signed numeric values do not match wrt operation. 
 */
bool
xme_core_directory_attribute_compareNumericValues
(
    xme_core_directory_attribute_filterOperation_t operation,
    int64_t filterValue,
    int64_t defValue
);

/**
 * \brief Compares two unsigned numeric values. 
 * 
 * \param operation The comparison operation. 
 * \param filterValue The unsigned numeric filter value. 
 * \param defValue The unsigned numeric defintion value. 
 *
 * \retval true if unsigned numeric values match wrt operation. 
 * \retval false if unsigned numeric values do not match wrt operation. 
 */
bool
xme_core_directory_attribute_compareUnsignedValues
(
    xme_core_directory_attribute_filterOperation_t operation,
    uint64_t filterValue,
    uint64_t defValue
);

/**
 * \brief Compares two decimal values. 
 * 
 * \param operation The comparison operation. 
 * \param filterValue The decimal filter value. 
 * \param defValue The decimal defintion value. 
 *
 * \retval true if decimal values match wrt operation. 
 * \retval false if decimal values do not match wrt operation. 
 */
bool
xme_core_directory_attribute_compareDecimalValues
(
    xme_core_directory_attribute_filterOperation_t operation,
    double filterValue,
    double defValue
);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DIRECTORY_ATTRIBUTEINTERNALMETHODS_H
