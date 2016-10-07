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
 * $Id: attribute.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Attribute abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/directory/include/attributeInternalMethods.h"

#include "xme/core/log.h"

#include "xme/hal/include/math.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

#include <float.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \brief attributeDescriptorListRegistry_t
 *
 * \details Entry for table attributeDescriptorListRegistry.
 */
typedef struct
{
    xme_core_topic_t topic; ///< Topic for which attributeDescriptorList is registered.
    xme_core_attribute_descriptor_list_t attributeDescriptorList; ///< The attribute descriptor list itself.
    xme_core_attribute_descriptor_t attributeDescriptors[255]; ///< Storage for attribute descriptors referenced by attributeDescriptorList.
    // TODO: How can we dynamically allocate only the necessary amount of memory fo attributeDescriptors (when dynamic allocation is available?)
} attributeDescriptorListRegistry_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief AttributeDescriptorList registry.
 *
 * \details Contains all registered attribute descriptor lists.
 */
static XME_HAL_TABLE
(
    attributeDescriptorListRegistry_t,
    attributeDescriptorListRegistry,
    XME_CORE_DIRECTORY_ATTRIBUTE_MAX_REGISTRY_SIZE
);

/**
 * \var xme_core_directory_attributes
 *
 * \brief Table with all the attribute sets.
 */
xme_core_directory_attributes_t xme_core_directory_attributes;

/**
 * \var xme_core_directory_attribute_keymap
 *
 * \brief Table with the hash table for keys and associated data.
 */
xme_core_directory_attribute_keymap_t xme_core_directory_attribute_keymap;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_directory_attribute_init(void)
{
    // Inits the structure for storing all attributes associated to 
    // the node. In the case of PnPManager, this should include
    // eventually all attributes received from other nodes. 
    XME_HAL_TABLE_INIT(xme_core_directory_attributes);

    XME_HAL_TABLE_INIT(xme_core_directory_attribute_keymap);

    XME_HAL_TABLE_INIT(attributeDescriptorListRegistry);

    return XME_STATUS_SUCCESS;
}

void
xme_core_directory_attribute_fini(void)
{
    XME_HAL_TABLE_FINI(xme_core_directory_attributes);
    XME_HAL_TABLE_FINI(xme_core_directory_attribute_keymap);
    XME_HAL_TABLE_FINI(attributeDescriptorListRegistry);
}

xme_status_t
xme_core_directory_attribute_registerAttributeDescriptorList
(
    xme_core_topic_t topic,
    const xme_core_attribute_descriptor_list_t* const attributeDescriptorList,
    bool overwrite
)
{
    xme_hal_table_rowHandle_t handle;
    attributeDescriptorListRegistry_t* item;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    item = NULL;

    XME_CHECK(XME_CORE_TOPIC_INVALID_TOPIC != topic, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != attributeDescriptorList, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        attributeDescriptorListRegistry,
        xme_hal_table_rowHandle_t,
        handle,
        attributeDescriptorListRegistry_t,
        item,
        (topic == item->topic)
    );

    XME_CHECK(NULL == item || overwrite, XME_STATUS_ALREADY_EXIST);

    // When item is NULL then add it to table
    if (NULL == item)
    {
        handle = XME_HAL_TABLE_ADD_ITEM(attributeDescriptorListRegistry);
        XME_CHECK(handle != XME_HAL_TABLE_INVALID_ROW_HANDLE, XME_STATUS_OUT_OF_RESOURCES);
 
        item = XME_HAL_TABLE_ITEM_FROM_HANDLE(attributeDescriptorListRegistry, handle);
        XME_ASSERT(NULL != item);
    }

    // Item is not NULL here
    item->topic = topic;

    // Copy attribute descriptors
    (void) xme_hal_mem_copy
    (
        &(item->attributeDescriptors),
        attributeDescriptorList->element,
        attributeDescriptorList->length * sizeof(item->attributeDescriptors[0])
    );

    item->attributeDescriptorList.length = attributeDescriptorList->length;
    item->attributeDescriptorList.element = item->attributeDescriptors;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_attribute_getAttributeDescriptorList
(
    xme_core_topic_t topic,
    xme_core_attribute_descriptor_list_t* attributeDescriptorList
)
{
    xme_hal_table_rowHandle_t handle;
    attributeDescriptorListRegistry_t* item;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    item = NULL;

    XME_CHECK(NULL != attributeDescriptorList, XME_STATUS_INVALID_PARAMETER);
    
    attributeDescriptorList->element = NULL;
    attributeDescriptorList->length = 0;

    XME_CHECK(XME_CORE_TOPIC_INVALID_TOPIC != topic, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        attributeDescriptorListRegistry,
        xme_hal_table_rowHandle_t,
        handle,
        attributeDescriptorListRegistry_t,
        item,
        (topic == item->topic)
    );

    XME_CHECK(NULL != item, XME_STATUS_NOT_FOUND);

    *attributeDescriptorList = item->attributeDescriptorList;

    return XME_STATUS_SUCCESS;
}

xme_core_directory_attributeSetHandle_t
xme_core_directory_attribute_createAttributeSet(void)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;

    // Allocate an unique attribute set handle
    xme_core_directory_attributeSetHandle_t newAttributeSetHandle = (xme_core_directory_attributeSetHandle_t) XME_HAL_TABLE_ADD_ITEM(xme_core_directory_attributes);
    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != newAttributeSetHandle, XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET);

    // Initialize the recently created attribute set. 
    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, newAttributeSetHandle);
    XME_CHECK(NULL != attributeSet, XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET);
    XME_HAL_TABLE_INIT(attributeSet->xme_core_directory_attributeSet);
    attributeSet->iterator = NULL;
    attributeSet->iteratorInUse = false;
    
    return newAttributeSetHandle;
}

xme_status_t
xme_core_directory_attribute_removeAttributeSet
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;

    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);

    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, XME_STATUS_NOT_FOUND); 

    // Free memory for the attribute set. 
    XME_HAL_TABLE_FINI(attributeSet->xme_core_directory_attributeSet);

    // Return the result of item removal. 
    return XME_HAL_TABLE_REMOVE_ITEM(xme_core_directory_attributes, (xme_hal_table_rowHandle_t) attributeSetHandle);
}

bool
xme_core_directory_attribute_isAttributeSetHandleValid
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, false);

    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, false);

    return true;
}

// TODO: Create an internal function that handles both
//       xme_core_directory_attribute_addPredefinedAttributeDefinition()
//       and xme_core_directory_attribute_addPredefinedAttributeFilter(),
//       then replace this function with a call to that function in order
//       to reduce code duplication.
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
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    xme_core_directory_attribute_t* attribute;
    xme_hal_table_rowHandle_t internalHandle;

    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    attributeSet = NULL;
    attribute = NULL;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != key, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != value, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(0 != numValues, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(1 == numValues, XME_STATUS_UNSUPPORTED); // This check corresponds to the current assumptions (arrays not supported yet).
    XME_CHECK(0 != valueSize || XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING == datatype, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID != datatype, XME_STATUS_INVALID_PARAMETER);

    // TODO: Decide if this block should be done at this point. 
    XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY != datatype, XME_STATUS_UNSUPPORTED);
    XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX != datatype, XME_STATUS_UNSUPPORTED);

    // 1. Lookup for the corresponding attributeSet. 
    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, XME_STATUS_INVALID_HANDLE);

    // 2. Check if the category is the same when there is at least one attribute. 
    if (0 < xme_core_directory_attribute_getAttributeSetCount(attributeSetHandle))
    {
        XME_CHECK(xme_core_directory_attribute_isDefinition(attributeSetHandle), XME_STATUS_INVALID_PARAMETER);
    }

    // 3. Register the attribute key if it is not already registered.
    if (!xme_core_directory_attribute_isKeyRegistered(key))
    {
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_directory_registerKey(key, datatype), XME_STATUS_INTERNAL_ERROR);
    }
    else
    {
        xme_core_directory_attribute_datatype_t registeredDatatype;
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_directory_attribute_getAttributeKeyDatatype(key, &registeredDatatype), XME_STATUS_INTERNAL_ERROR);

        XME_CHECK(registeredDatatype == datatype, XME_STATUS_INVALID_PARAMETER);
    }

    // 4. Find if there is an existing key in the current attribute set. 
    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    XME_HAL_TABLE_GET_NEXT
    (
        attributeSet->xme_core_directory_attributeSet, 
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_t, 
        attribute,
        attribute->key == key
    );
    
    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == internalHandle)
    {
        // The key was not found, so we can safely create a new entry. 
        internalHandle = XME_HAL_TABLE_ADD_ITEM(attributeSet->xme_core_directory_attributeSet);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, XME_STATUS_OUT_OF_RESOURCES);

        attribute = XME_HAL_TABLE_ITEM_FROM_HANDLE(attributeSet->xme_core_directory_attributeSet, internalHandle);
        XME_ASSERT(NULL != attribute);

        attribute->key = key;
        attribute->valueSize = valueSize;
    }
    else
    {
        // There is an existing key. 
        // So, we have to check two conditions:
        // - if the existing attributeSet is a definition. 
        // - the 'overwrite' parameter. 
        XME_CHECK(xme_core_directory_attribute_isDefinition(attributeSetHandle), XME_STATUS_INVALID_PARAMETER); // TODO: This has already been checked before
        XME_CHECK(overwrite, XME_STATUS_ALREADY_EXIST);

        // Now, we can ensure that we can overwrite the existing parameter with the new values. 
        attribute = XME_HAL_TABLE_ITEM_FROM_HANDLE(attributeSet->xme_core_directory_attributeSet, internalHandle);
    }

    XME_ASSERT(NULL != attribute);
    attribute->category = XME_CORE_DIRECTORY_ATTRIBUTE_CATEGORY_PREDEFINED_DEFINITION;
    attribute->numValues = numValues;

    switch(datatype)
    {
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING:
            // TODO: How to internally store multiple string values?
            //       Probably in an array of char pointers?
            if (NULL != attribute->valueData.value.str)
            {
                xme_hal_mem_free(attribute->valueData.value.str);
            }
            attribute->valueSize = strlen((char*) value);
            if (0U != valueSize && attribute->valueSize > valueSize)
            {
                attribute->valueSize = valueSize;
            }
            attribute->valueSize += 1U; // Account for terminating nul character
            attribute->valueData.value.str = (char*) xme_hal_mem_alloc(attribute->valueSize);
            (void) xme_hal_safeString_strncpy(attribute->valueData.value.str, (const char*) value, attribute->valueSize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC:
            (void) xme_hal_mem_copy(&attribute->valueData.value.num, value, valueSize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED:
            (void) xme_hal_mem_copy(&attribute->valueData.value.uns, value, valueSize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL:
            (void) xme_hal_mem_copy(&attribute->valueData.value.dec, value, valueSize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY:
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX:
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID:
        default:
            return XME_STATUS_UNSUPPORTED;
    }

    // TODO: Existing attribute valueSize might differ from parameter value size!
    //       This might cause error when interpreting the value later
    // TODO: Perhaps always memset the attribute->valueData to 0?

    XME_ASSERT(attribute->valueSize > 0U);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_attribute_removePredefinedAttributeDefinition
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    xme_core_directory_attribute_t* attribute;
    xme_hal_table_rowHandle_t internalHandle;

    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    attributeSet = NULL;
    attribute = NULL;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != key, XME_STATUS_INVALID_PARAMETER);

    // 1. Lookup for the corresponding attributeSet. 
    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, XME_STATUS_INVALID_HANDLE);

    // 2. Check if the corresponding handle corresponds to a definition attribute. 
    //    The return status is an invalid handle, because that handle does not correspond to a definition attribute. 
    XME_CHECK(xme_core_directory_attribute_isDefinition(attributeSetHandle), XME_STATUS_INVALID_HANDLE);

    // 3. Find if there is an attribute with the given key in the current attribute set. 
    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    XME_HAL_TABLE_GET_NEXT
    (
        attributeSet->xme_core_directory_attributeSet, 
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_t, 
        attribute,
        attribute->key == key
    );
    
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, XME_STATUS_NOT_FOUND);

    // We can safely remove that key. 
    return XME_HAL_TABLE_REMOVE_ITEM(attributeSet->xme_core_directory_attributeSet, internalHandle);
}

// TODO: Create an internal function that handles both
//       xme_core_directory_attribute_addPredefinedAttributeDefinition()
//       and xme_core_directory_attribute_addPredefinedAttributeFilter(),
//       then replace this function with a call to that function in order
//       to reduce code duplication.
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
)
{
    // TODO: Same TODOs as xme_core_directory_attribute_addPredefinedAttributeDefinition apply

    xme_core_directory_attribute_attributeSet_t* attributeSet;
    xme_core_directory_attribute_t* attribute;
    xme_hal_table_rowHandle_t internalHandle;

    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    attributeSet = NULL;
    attribute = NULL;

    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != key, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != value, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(0 != numValues, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(1 == numValues, XME_STATUS_UNSUPPORTED); // This check corresponds to the current assumptions.
    XME_CHECK(0 != valueSize || XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING == datatype, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID != datatype, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID != operation, XME_STATUS_INVALID_PARAMETER);

    // TODO: Decide if this block should be done at this point. 
    XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY != datatype, XME_STATUS_UNSUPPORTED);
    XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX != datatype, XME_STATUS_UNSUPPORTED);

    // 1. Lookup for the corresponding attributeSet. 
    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_attributes, 
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_attributeSet_t, 
        attributeSet,
        internalHandle == (xme_hal_table_rowHandle_t) attributeSetHandle
    );

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, XME_STATUS_INVALID_HANDLE);

    // 2. Check if the category is the same when there is at least one attribute. 
    if (0 < xme_core_directory_attribute_getAttributeSetCount(attributeSetHandle))
    {
        XME_CHECK(xme_core_directory_attribute_isFilter(attributeSetHandle), XME_STATUS_INVALID_PARAMETER);
    }

    // 3. Check if the key is already registered in the key map.
    if (!xme_core_directory_attribute_isKeyRegistered(key))
    {
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_directory_registerKey(key, datatype), XME_STATUS_INTERNAL_ERROR);
    }
    else
    {
        xme_core_directory_attribute_datatype_t registeredDatatype;
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_directory_attribute_getAttributeKeyDatatype(key, &registeredDatatype), XME_STATUS_INTERNAL_ERROR);

        XME_CHECK(registeredDatatype == datatype, XME_STATUS_INVALID_PARAMETER);
    }

    // 4. Find if there is an existing key in the current attribute set. 
    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    XME_HAL_TABLE_GET_NEXT
    (
        attributeSet->xme_core_directory_attributeSet, 
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_t, 
        attribute,
        attribute->key == key
    );

    if (XME_HAL_TABLE_INVALID_ROW_HANDLE == internalHandle)
    {
        // The key was not found, so we can safely create a new entry. 
        internalHandle = XME_HAL_TABLE_ADD_ITEM(attributeSet->xme_core_directory_attributeSet);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, XME_STATUS_OUT_OF_RESOURCES);

        attribute = XME_HAL_TABLE_ITEM_FROM_HANDLE(attributeSet->xme_core_directory_attributeSet, internalHandle);
        XME_ASSERT(NULL != attribute);

        attribute->key = key;
        attribute->valueSize = valueSize;
    }
    else
    {
        // There is an existing key. 
        // So, we have to check two conditions:
        // - if the existing attributeSet is a filter. 
        // - the 'overwrite' parameter. 
        XME_CHECK(xme_core_directory_attribute_isFilter(attributeSetHandle), XME_STATUS_INVALID_PARAMETER);
        XME_CHECK(overwrite, XME_STATUS_ALREADY_EXIST);

        // Now, we can ensure that we can overwrite the existing parameter with the new values. 
        attribute = XME_HAL_TABLE_ITEM_FROM_HANDLE(attributeSet->xme_core_directory_attributeSet, internalHandle);
    }

    attribute->category = XME_CORE_DIRECTORY_ATTRIBUTE_CATEGORY_PREDEFINED_FILTER;
    attribute->numValues = numValues;
    attribute->parameter.filter.operation = operation;

    switch(datatype)
    {
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING:
            // TODO: How to internally store multiple string values?
            //       Probably in an array of char pointers?
            if (NULL != attribute->valueData.value.str)
            {
                // Free old memory
                xme_hal_mem_free(attribute->valueData.value.str);
            }
            attribute->valueSize = strlen((char*) value);
            if (0U != valueSize && attribute->valueSize > valueSize)
            {
                attribute->valueSize = valueSize;
            }
            attribute->valueSize += 1U; // Account for terminating nul character
            attribute->valueData.value.str = (char*) xme_hal_mem_alloc(attribute->valueSize);
            (void) xme_hal_safeString_strncpy(attribute->valueData.value.str, (const char*) value, attribute->valueSize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC:
            (void) xme_hal_mem_copy(&attribute->valueData.value.num, value, valueSize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED:
            (void) xme_hal_mem_copy(&attribute->valueData.value.uns, value, valueSize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL:
            (void) xme_hal_mem_copy(&attribute->valueData.value.dec, value, valueSize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID:
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY:
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX:
        default:
            return XME_STATUS_UNSUPPORTED;
    }

    XME_ASSERT(attribute->valueSize > 0U);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_attribute_removePredefinedAttributeFilter
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    xme_core_directory_attribute_t* attribute;
    xme_hal_table_rowHandle_t internalHandle;

    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    attributeSet = NULL;
    attribute = NULL;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);
    XME_CHECK(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != key, XME_STATUS_INVALID_PARAMETER);

    // 1. Lookup for the corresponding attributeSet. 
    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, XME_STATUS_INVALID_HANDLE);

    // 2. Check if the corresponding handle corresponds to a filter attribute. 
    //    The return status is an invalid handle, because that handle does not correspond to a filter attribute. 
    XME_CHECK(xme_core_directory_attribute_isFilter(attributeSetHandle), XME_STATUS_INVALID_HANDLE);

    // 3. Find if there is an existing key in the current attribute set. 
    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    XME_HAL_TABLE_GET_NEXT
    (
        attributeSet->xme_core_directory_attributeSet, 
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_t, 
        attribute,
        attribute->key == key
    );
    
    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, XME_STATUS_NOT_FOUND);

    // We can safely remove that key. 
    return XME_HAL_TABLE_REMOVE_ITEM(attributeSet->xme_core_directory_attributeSet, internalHandle);
}

uint16_t
xme_core_directory_attribute_getAttributeSetCount
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, 0);
        
    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, 0);

    return XME_HAL_TABLE_ITEM_COUNT(attributeSet->xme_core_directory_attributeSet);
}

xme_status_t
xme_core_directory_attribute_getAttributeKeyDatatype
(
    xme_core_attribute_key_t key,
    xme_core_directory_attribute_datatype_t* datatype
)
{
    xme_core_directory_attribute_keyMapEntry_t* keyMapEntry;
    xme_hal_table_rowHandle_t internalHandle;

    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    keyMapEntry = NULL;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != key, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != datatype, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_attribute_keymap, 
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_keyMapEntry_t, 
        keyMapEntry,
        keyMapEntry->key == key
    );

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, XME_STATUS_INVALID_PARAMETER);

    *datatype = keyMapEntry->datatype;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_attribute_initIterator
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;

    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);

    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, XME_STATUS_INVALID_HANDLE);

    XME_CHECK(!attributeSet->iteratorInUse, XME_STATUS_INTERNAL_ERROR);

    if (xme_core_directory_attribute_getAttributeSetCount(attributeSetHandle) > 0)
    {
        // Get the first element of the attribute set. 
        XME_HAL_TABLE_ITERATE_BEGIN
        ( 
            attributeSet->xme_core_directory_attributeSet,
            xme_hal_table_rowHandle_t, 
            internalHandle, 
            xme_core_directory_attribute_t, 
            attribute
        );
        {
            XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, XME_STATUS_INVALID_HANDLE); // TODO: I think XME_STATUS_INTERNAL_ERROR is more appropriate

            attributeSet->iterator = attribute;

            break;
        }
        XME_HAL_TABLE_ITERATE_END();
    }
    else
    {
        attributeSet->iterator = NULL;
    }

    attributeSet->iteratorInUse = true;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_attribute_finiIterator
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);

    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, XME_STATUS_INVALID_HANDLE);

    XME_CHECK(attributeSet->iteratorInUse, XME_STATUS_INTERNAL_ERROR);

    attributeSet->iterator = NULL;
    attributeSet->iteratorInUse = false;

    return XME_STATUS_SUCCESS;
}

bool
xme_core_directory_attribute_hasNextAttributeHandle
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;

    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, false);

    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, false);

    XME_CHECK(attributeSet->iteratorInUse, false);

    return (NULL != attributeSet->iterator);
}

xme_status_t
xme_core_directory_attribute_getNextAttributeHandle
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_directory_attributeHandle_t* attributeHandle
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    bool found;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);

    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, XME_STATUS_INVALID_HANDLE);

    // Get the element that matches with the iterator. 
    found = false;

    XME_HAL_TABLE_ITERATE_BEGIN
    ( 
        attributeSet->xme_core_directory_attributeSet,
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_t, 
        attribute
    );
    {
        if (found)
        {
            // We should place the found check here, to get the following iterator.
            attributeSet->iterator = attribute;

            // the operation was completed: the attribute handle is set, and the iterator
            // advanced to the following position. 
            return XME_STATUS_SUCCESS;
        }
            
        if (attribute == attributeSet->iterator)
        {
            found = true;
            *attributeHandle = (xme_core_directory_attributeHandle_t) internalHandle; // TODO: Add a test this looks to me like it might be wrong (shouldn't this be in the if(found) block?). All tests of getnext only return XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE...
            attributeSet->iterator = NULL;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    if (found)
    {
        // If we get out of the loop, this means that we have found the iterator
        // but not established the next item, because it was already the last
        // item of the attributes. So we should just establish the iterator to NULL. 
        attributeSet->iterator = NULL;
    }
    else 
    {
        // On the contrary, we should obtain an error. 
        *attributeHandle = (xme_core_directory_attributeHandle_t) XME_HAL_TABLE_INVALID_ROW_HANDLE;
        return XME_STATUS_INTERNAL_ERROR;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_attribute_getAttributeKey
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_directory_attributeHandle_t attributeHandle,
    xme_core_attribute_key_t* key
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    xme_core_directory_attribute_t* attribute;

    XME_CHECK(NULL != key, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);

    // Retrieve the attribute set element
    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);
    XME_ASSERT(NULL != attributeSet);

    // Retrieve the attribute element
    attribute = XME_HAL_TABLE_ITEM_FROM_HANDLE(attributeSet->xme_core_directory_attributeSet, attributeHandle);
    XME_CHECK(NULL != attribute, XME_STATUS_INVALID_HANDLE);

    *key = attribute->key;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_directory_attribute_getAttributeValue
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle,
    xme_core_attribute_key_t key,
    void* buffer,
    uint16_t bufferSize,
    uint32_t* valueSize
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    xme_core_directory_attribute_t* attribute;
    xme_hal_table_rowHandle_t internalHandle;
    xme_core_directory_attribute_datatype_t datatype;
    uint32_t copySize;

    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    attributeSet = NULL;
    attribute = NULL;

    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, XME_STATUS_INVALID_HANDLE);

    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, XME_STATUS_INVALID_HANDLE);

    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    // Get the element that matches the attribute handle
    XME_HAL_TABLE_GET_NEXT
    ( 
        attributeSet->xme_core_directory_attributeSet,
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_t, 
        attribute, 
        attribute->key == key
    );

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, XME_STATUS_INVALID_HANDLE);

    // Get the datatype of the key
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_directory_attribute_getAttributeKeyDatatype(key, &datatype), XME_STATUS_INVALID_HANDLE);

    if (NULL != valueSize)
    {
        *valueSize = attribute->valueSize;
    }

    copySize = XME_HAL_MATH_MIN((uint32_t)bufferSize, attribute->valueSize);
    if (0 == copySize)
    {
        // Early abort if copySize is zero
        return (copySize < attribute->valueSize) ? XME_STATUS_BUFFER_TOO_SMALL : XME_STATUS_SUCCESS;
    }

    switch (datatype)
    {
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING:
            (void) xme_hal_safeString_strncpy((char*) buffer, attribute->valueData.value.str, bufferSize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC:
            (void) xme_hal_mem_copy(buffer, &attribute->valueData.value.num, copySize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED:
            (void) xme_hal_mem_copy(buffer, &attribute->valueData.value.uns, copySize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL:
            (void) xme_hal_mem_copy(buffer, &attribute->valueData.value.dec, copySize);
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID:
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY:
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX:
        default:
            return XME_STATUS_UNSUPPORTED;
    }

    return (copySize < attribute->valueSize) ? XME_STATUS_BUFFER_TOO_SMALL : XME_STATUS_SUCCESS;
}

bool
xme_core_directory_attribute_isFilter
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    bool allFilters;

    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, false);

    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, false);

    XME_CHECK(0 < xme_core_directory_attribute_getAttributeSetCount(attributeSetHandle), false);

    // Explore all items. 
    allFilters = true;
    XME_HAL_TABLE_ITERATE_BEGIN
    ( 
        attributeSet->xme_core_directory_attributeSet,
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_t, 
        attribute
    );
    {
        if (XME_CORE_DIRECTORY_ATTRIBUTE_CATEGORY_PREDEFINED_FILTER != attribute->category) // Add here all categories matching filters. 
        {
            allFilters = false; // TODO: Early exit here (remove later check, and remove allFilters variable)
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    XME_CHECK(allFilters, false);

    return true;
}

bool
xme_core_directory_attribute_isDefinition
(
    xme_core_directory_attributeSetHandle_t attributeSetHandle
)
{
    xme_core_directory_attribute_attributeSet_t* attributeSet;
    bool allDefs;

    XME_CHECK(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET != attributeSetHandle, false);

    attributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, attributeSetHandle);

    XME_CHECK(NULL != attributeSet, false);

    XME_CHECK(0 < xme_core_directory_attribute_getAttributeSetCount(attributeSetHandle), false);

    // Explore all items. 
    allDefs = true;
    XME_HAL_TABLE_ITERATE_BEGIN
    ( 
        attributeSet->xme_core_directory_attributeSet,
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_t, 
        attribute
    );
    {
        if (attribute->category != XME_CORE_DIRECTORY_ATTRIBUTE_CATEGORY_PREDEFINED_DEFINITION) // Add here all categories matching definitions. 
        {
            allDefs = false;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    XME_CHECK(allDefs, false);

    return true;
}

xme_status_t
xme_core_directory_attribute_isFilterMatchingDefinition
(
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle,
    xme_core_directory_attributeSetHandle_t defAttributeSetHandle,
    int* matches
)
{
    xme_core_directory_attribute_attributeSet_t* defAttributeSet = NULL;
    xme_core_directory_attribute_attributeSet_t* filterAttributeSet = NULL;
    bool isFilter;
    bool isDefinition;

    XME_CHECK(NULL != matches, XME_STATUS_INVALID_PARAMETER);

    // First check whether the attribute definition is either empty or actually
    // an attribute definition, because we don't want to return SUCCESS for
    // this case
    isFilter = xme_core_directory_attribute_isFilter(defAttributeSetHandle);
    isDefinition = xme_core_directory_attribute_isDefinition(defAttributeSetHandle);
    XME_ASSERT(!(isFilter && isDefinition));
    XME_CHECK(isDefinition || !isFilter, XME_STATUS_INVALID_HANDLE);

    // Early return if filter is empty (i.e., if it accepts any definition)
    if (XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET == filterAttributeSetHandle ||
        0 == xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle))
    {
        XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(!xme_core_directory_attribute_isFilter(filterAttributeSetHandle)));
        XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(!xme_core_directory_attribute_isDefinition(filterAttributeSetHandle)));

        *matches = true;
        return XME_STATUS_SUCCESS;
    }

    // If we get here, filterAttributeSetHandle must specify an attribute
    // filter. This means that it contains at least one filter condition.
    // Notice that the filter condition could be something like "not exists
    // attribute a", hence we cannot directly decide whether an empty
    // attribute definition matches or does not match.
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(xme_core_directory_attribute_isFilter(filterAttributeSetHandle)));
    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(!xme_core_directory_attribute_isDefinition(filterAttributeSetHandle))); // TODO: Why do we CHECK if defAttributeSetHandle is a definition, but only ASSERT that filterAttributeSetHandle is a filter?

    // 1. Find the corresponding attribute sets
    defAttributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, defAttributeSetHandle);
    XME_CHECK(NULL != defAttributeSet, XME_STATUS_INVALID_HANDLE);

    filterAttributeSet = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attributes, filterAttributeSetHandle);
    XME_CHECK(NULL != filterAttributeSet, XME_STATUS_INVALID_HANDLE);

    // 2. Match the keys for filters and definitions
    XME_HAL_TABLE_ITERATE_BEGIN
    (
        filterAttributeSet->xme_core_directory_attributeSet,
        xme_core_directory_attributeHandle_t,
        filterHandle,
        xme_core_directory_attribute_t,
        filterAttribute
    );
    {
        bool found = false;

        XME_ASSERT(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != filterAttribute->key);

        // TODO: Use XME_HAL_TABLE_GET_NEXT here, this saves some iterations (there can only be one attribute value definition per key, right?)
        XME_HAL_TABLE_ITERATE_BEGIN
        (
            defAttributeSet->xme_core_directory_attributeSet,
            xme_core_directory_attributeHandle_t,
            defHandle,
            xme_core_directory_attribute_t,
            defAttribute
        );
        {
            XME_ASSERT(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != defAttribute->key);

            if (defAttribute->key == filterAttribute->key)
            {
                // Definition and filter key match
                xme_core_directory_attribute_datatype_t datatype;
                xme_status_t status;

                found = true;

                status = xme_core_directory_attribute_getAttributeKeyDatatype(defAttribute->key, &datatype);
                XME_ASSERT(XME_STATUS_SUCCESS == status);
                XME_ASSERT(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID != datatype);

                // FIXME: Only some data types are currently supported
                XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY != datatype, XME_STATUS_UNSUPPORTED);
                XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX != datatype, XME_STATUS_UNSUPPORTED);

                *matches = xme_core_directory_attribute_matchesAttributeValue(filterAttribute, defAttribute, datatype);

                if (!(*matches))
                {
                    // One of the filter conditions does not match. Since all
                    // conditions are conjoint, we can stop checking at this
                    // point.
                    return XME_STATUS_SUCCESS;
                }
            }
        }
        XME_HAL_TABLE_ITERATE_END();

        if (!found)
        {
            // Special handling for "not exists" operator
            if (XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_NOT_EXISTS != filterAttribute->parameter.filter.operation)
            {
                *matches = false;
                return XME_STATUS_SUCCESS;
            }
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    // We can have two different cases here:
    // - When there are no attributes in filter, the result is intended to match.
    // - When all attributes matches, the result is true. 
    *matches = true;

    return XME_STATUS_SUCCESS;
}

bool
xme_core_directory_attribute_matchesAttributeValue
(
    const xme_core_directory_attribute_t* const filterAttribute,
    const xme_core_directory_attribute_t* const defAttribute,
    xme_core_directory_attribute_datatype_t datatype
)
{
    bool retVal;

    // Check the number of values
    // FIXME: Currently only one value supported
    XME_CHECK(1U == filterAttribute->numValues, false);
    XME_CHECK(1U == defAttribute->numValues, false);

    // Check the value sizes
    // TODO: Is a value size of zero really not allowed?
    //       Consider "flags" without a value, e.g., "i-am-safety-critical"!
    XME_CHECK(filterAttribute->valueSize > 0U, false);
    XME_CHECK(defAttribute->valueSize > 0U, false);

    // Check the filter operation
    XME_ASSERT_RVAL(XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID != filterAttribute->parameter.filter.operation, false);

    switch(datatype)
    {
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING:
            retVal = xme_core_directory_attribute_compareStringValues
                (
                    filterAttribute->parameter.filter.operation,
                    filterAttribute->valueData.value.str, 
                    filterAttribute->valueSize, 
                    defAttribute->valueData.value.str,
                    defAttribute->valueSize
                );
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC:
            retVal = xme_core_directory_attribute_compareNumericValues
                (
                    filterAttribute->parameter.filter.operation,
                    filterAttribute->valueData.value.num, 
                    defAttribute->valueData.value.num
                );
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED:
            retVal = xme_core_directory_attribute_compareUnsignedValues
                (
                    filterAttribute->parameter.filter.operation,
                    filterAttribute->valueData.value.uns, 
                    defAttribute->valueData.value.uns
                );
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL:
            retVal = xme_core_directory_attribute_compareDecimalValues
                (
                    filterAttribute->parameter.filter.operation,
                    filterAttribute->valueData.value.dec, 
                    defAttribute->valueData.value.dec
                );
            break;
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY:
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX:
        case XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID:
        default:
            return false;
    }

    return retVal;
}

bool
xme_core_directory_attribute_compareStringValues
(
    xme_core_directory_attribute_filterOperation_t operation,
    const char* filterValue,
    uint32_t filterValueSize,
    const char* defValue,
    uint16_t defValueSize
)
{
    int result;

    XME_ASSERT_RVAL(NULL != filterValue, false);
    XME_ASSERT_RVAL(NULL != defValue, false);

    XME_UNUSED_PARAMETER(filterValueSize); // TODO: Do we ever plan to use these? Please add comment
    XME_UNUSED_PARAMETER(defValueSize);

    result = strcmp(defValue, filterValue);

    switch(operation)
    {
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL:
            return (0 == result);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER:
            return (0 < result);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER_OR_EQUAL:
            return (0 <= result);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS:
            return (0 > result);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS_OR_EQUAL:
            return (0 >= result);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_UNEQUAL:
            return (0 != result);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_NOT_EXISTS:
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EXISTS:
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID:
        default:
            return false;
    }
}

bool
xme_core_directory_attribute_compareNumericValues
(
    xme_core_directory_attribute_filterOperation_t operation,
    int64_t filterValue,
    int64_t defValue
)
{
    switch(operation)
    {
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL:
            return (defValue == filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER:
            return (defValue > filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER_OR_EQUAL:
            return (defValue >= filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS:
            return (defValue < filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS_OR_EQUAL:
            return (defValue <= filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_UNEQUAL:
            return (defValue != filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_NOT_EXISTS:
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EXISTS:
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID:
        default:
            return false;
    }
}

bool
xme_core_directory_attribute_compareUnsignedValues
(
    xme_core_directory_attribute_filterOperation_t operation,
    uint64_t filterValue,
    uint64_t defValue
)
{
    switch(operation)
    {
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL:
            return (defValue == filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER:
            return (defValue > filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER_OR_EQUAL:
            return (defValue >= filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS:
            return (defValue < filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS_OR_EQUAL:
            return (defValue <= filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_UNEQUAL:
            return (defValue != filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_NOT_EXISTS:
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EXISTS:
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID:
        default:
            return false;
    }
}

bool
xme_core_directory_attribute_compareDecimalValues
(
    xme_core_directory_attribute_filterOperation_t operation,
    double filterValue,
    double defValue
)
{
    switch(operation)
    {
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL:
            // This kind of comparison is not ideal, but better than plain '=='
            return (fabs(defValue - filterValue) <= DBL_EPSILON);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER:
            return (defValue > filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER_OR_EQUAL:
            return (defValue >= filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS:
            return (defValue < filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS_OR_EQUAL:
            return (defValue <= filterValue);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_UNEQUAL:
            // This kind of comparison is not ideal, but better than plain '!='
            return (fabs(defValue - filterValue) > DBL_EPSILON);
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_NOT_EXISTS:
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EXISTS:
        case XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID:
        default:
            return false;
    }
}

////////////////////////////////////////////////////////////////////////////////////

bool
xme_core_directory_attribute_isKeyRegistered
(
    xme_core_attribute_key_t key
)
{
    xme_core_directory_attribute_keyMapEntry_t* keyMapEntry;
    xme_hal_table_rowHandle_t internalHandle;

    internalHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    keyMapEntry = NULL;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != key, false);

    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_attribute_keymap, 
        xme_hal_table_rowHandle_t, 
        internalHandle, 
        xme_core_directory_attribute_keyMapEntry_t, 
        keyMapEntry,
        keyMapEntry->key == key
    );

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != internalHandle, false);

    // if the row handle is not invalid, then it exists at least an entry. 
    return true;
}

xme_status_t
xme_core_directory_registerKey
(
    xme_core_attribute_key_t attributeKey, 
    xme_core_directory_attribute_datatype_t datatype
)
{
    xme_core_directory_attribute_keyMapEntry_t* keyMapEntry;
    xme_hal_table_rowHandle_t handle;

    handle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
    keyMapEntry = NULL;
    
    XME_CHECK(XME_CORE_ATTRIBUTE_KEY_UNDEFINED != attributeKey, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID != datatype, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        xme_core_directory_attribute_keymap, 
        xme_hal_table_rowHandle_t, 
        handle, 
        xme_core_directory_attribute_keyMapEntry_t, 
        keyMapEntry,
        keyMapEntry->key == attributeKey
    );

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE == handle, XME_STATUS_ALREADY_EXIST);

    // Create a new entry for the key map. 
    handle = XME_HAL_TABLE_ADD_ITEM(xme_core_directory_attribute_keymap);

    // Initialize the recently created attribute set. 
    keyMapEntry = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_core_directory_attribute_keymap, handle);
    XME_ASSERT(NULL != keyMapEntry);
    keyMapEntry->key = attributeKey;
    keyMapEntry->datatype = datatype;
    keyMapEntry->name = NULL; // TODO: Add comment that we currently are not using names for keys are we?)

    return XME_STATUS_SUCCESS;
}

