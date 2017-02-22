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
 * $Id: attributeInternalTypes.h 5630 2013-10-25 13:52:29Z ruiz $
 */

/**
 * \file
 *         Attribute internal types abstraction.
 *
 */

#ifndef XME_CORE_DIRECTORY_ATTRIBUTEINTERNALTYPES_H
#define XME_CORE_DIRECTORY_ATTRIBUTEINTERNALTYPES_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/directory/include/attribute.h"
#include "xme/hal/include/table.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_core_directory_attribute_category_t
 *
 * \brief The category of the attribute (definition, filter).
*/
typedef enum
{
    XME_CORE_DIRECTORY_ATTRIBUTE_CATEGORY_INVALID = 0, ///< Invalid category.
    XME_CORE_DIRECTORY_ATTRIBUTE_CATEGORY_PREDEFINED_DEFINITION, ///< Definition category (for publishers).
    XME_CORE_DIRECTORY_ATTRIBUTE_CATEGORY_PREDEFINED_FILTER ///< Filter category (for subscribers).
} xme_core_directory_attribute_category_t;


/**
 * \union xme_core_directory_attribute_value_t
 *
 * \brief  union of specific supported typed data.
 */
typedef union
{
    char* str; ///< String property value.
    int64_t num; ///< Numeric property value (positive or negative integer).
    uint64_t uns; ///< Unsigned property value (positive integer).
    double dec; ///< Decimal property value (double-precision floating-point number).
} xme_core_directory_attribute_value_t;

/**
 * \struct xme_core_directory_attribute_valueData_t
 *
 * \brief  The specific data storage for value data, categorized by
 *         datatype.
 */
typedef struct xme_core_directory_attribute_valueData_t
{
    xme_core_directory_attribute_value_t value; ///< The union of specific supported typed data.
    struct xme_core_directory_attribute_valueData_t* next; ///< The next element (in case of arrays of attributes).
} xme_core_directory_attribute_valueData_t;

/**
 * \struct xme_core_directory_attribute_attributeFilterData_t
 *
 * \brief Structure containing attribute filter specific data.
 */
typedef struct
{
    xme_core_directory_attribute_filterOperation_t operation; ///< The filter operation.
} xme_core_directory_attribute_attributeFilterData_t;

/**
 * \struct xme_core_directory_attribute_attributeDefinitionData_t
 *
 * \brief Structure containing attribute definition specific data.
 */
typedef struct
{
    char _dummy; ///< Just a dummy item; attribute definitions contain no specific data.
} xme_core_directory_attribute_attributeDefinitionData_t;

/**
 * \union xme_core_directory_attribute_parameter_t
 *
 * \brief The category of the attribute. 
 *
 * \details The category determines the different types of attributes. 
 */
typedef union
{
    xme_core_directory_attribute_attributeDefinitionData_t definition; ///< The attribute value is labeled as a definition. 
    xme_core_directory_attribute_attributeFilterData_t filter; ///< The attribute value is labeled as a filter. 
} xme_core_directory_attribute_parameter_t;

/**
 * \struct xme_core_directory_attribute_t
 *
 * \brief An entry in an attribute set.
 */
typedef struct
{
    xme_core_attribute_key_t key; ///< Property key.
    uint16_t numValues; ///< The number of values (for array data type, otherwise must be set to 1).
    uint32_t valueSize; ///< The size of the value (for arrays, the size of the whole data). 
    xme_core_directory_attribute_valueData_t valueData; ///< The data associated to the value(s). 
    xme_core_directory_attribute_parameter_t parameter; ///< The parameter associated to the attribute category. In filters, the required parameter is the operation. 
    xme_core_directory_attribute_category_t category; ///< The attribute category. 
} xme_core_directory_attribute_t;

/**
 * \brief This table stores all attributes associated to a given topic. // TODO: I don't think "to a given topic" is correct. Different publication/subscriptions might have different attribute sets despite having the same topic, right?
 */
typedef struct 
{
    XME_HAL_TABLE(xme_core_directory_attribute_t, xme_core_directory_attributeSet, XME_CORE_DIRECTORY_ATTRIBUTE_MAX_TOPIC_ATTRIBUTE_ITEMS); ///< The attribute set table. 
    xme_core_directory_attribute_t* iterator; ///< A pointer to the next element in the iterator. 
    bool iteratorInUse; ///< Boolean value to label if the iterator is already in use. 
} xme_core_directory_attribute_attributeSet_t;

/**
 * \brief A linked list storing all attribute sets. 
 */
typedef XME_HAL_TABLE
(
    xme_core_directory_attribute_attributeSet_t, 
    xme_core_directory_attributes_t, 
    XME_CORE_DIRECTORY_ATTRIBUTE_MAX_ATTRIBUTE_SETS
); 

/**
 * \brief The structure for storing the key map with all associated fields.
 */
typedef struct
{
    xme_core_attribute_key_t key; ///< The key. 
    xme_core_directory_attribute_datatype_t datatype; ///< The datatype associated to the key. 
    void* name; ///< The associated displayed name for the given key. 
} xme_core_directory_attribute_keyMapEntry_t;

/**
 * \brief This table stores all attributes associated to a given topic. // TODO: Same doc as for xme_core_directory_attribute_attributeSet_t -> copy&paste error?
 */
typedef XME_HAL_TABLE
(
    xme_core_directory_attribute_keyMapEntry_t, 
    xme_core_directory_attribute_keymap_t, 
    XME_CORE_DIRECTORY_ATTRIBUTE_MAX_ATTRIBUTE_KEYS
);

#endif // #ifndef XME_CORE_DIRECTORY_ATTRIBUTEINTERNALTYPES_H
