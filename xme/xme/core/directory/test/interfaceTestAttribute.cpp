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
 * $Id: interfaceTestAttribute.cpp 5630 2013-10-25 13:52:29Z ruiz $
 */

/**
 * \file
 *         Attribute abstraction interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/directory/include/attribute.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class AttributeInterfaceTest: public ::testing::Test
{
protected:
    AttributeInterfaceTest()
    : defAttributeSetHandle1(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , defAttributeSetHandle2(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , defAttributeSetHandle3(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle1(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle2(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle3(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , genericAttributeSetHandle1(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , genericAttributeSetHandle2(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , genericAttributeSetHandle3(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , value1(1U)
    , value2(2U)
    , attributeHandle(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE)
    , attributeKey(XME_CORE_ATTRIBUTE_KEY_UNDEFINED)
    , attributeValue(0U)
    , attributeSize(0U)
    {
    }

    virtual ~AttributeInterfaceTest()
    {
    }

    virtual void SetUp()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_init());

        attributeDescriptorList1.length = 0;

        attributeDescriptorList2.length = 0;
        attributeDescriptorList2.element = NULL;

        attributeDescriptorList3.length = 1;
        attributeDescriptorList3.element = attributeDesriptors3;
        attributeDesriptors3[0].key = (xme_core_attribute_key_t)1;
        attributeDesriptors3[0].size = 4;

        attributeDescriptorList4.length = 2;
        attributeDescriptorList4.element = attributeDesriptors4;
        attributeDesriptors4[0].key = (xme_core_attribute_key_t)1;
        attributeDesriptors4[0].size = 4;
        attributeDesriptors4[1].key = (xme_core_attribute_key_t)2;
        attributeDesriptors4[1].size = 8;

        attributeDescriptorListOut1.length = 0;
        attributeDescriptorListOut1.element = NULL;
        attributeDescriptorListOut2.length = 0;
        attributeDescriptorListOut2.element = NULL;

        topic1 = (xme_core_topic_t)1;
        topic2 = (xme_core_topic_t)2;
        topic3 = (xme_core_topic_t)3;
    }

    virtual void TearDown()
    {
        xme_core_directory_attribute_fini();
    }

    xme_core_directory_attributeSetHandle_t defAttributeSetHandle1;
    xme_core_directory_attributeSetHandle_t defAttributeSetHandle2;
    xme_core_directory_attributeSetHandle_t defAttributeSetHandle3;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle1;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle2;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle3;

    xme_core_directory_attributeSetHandle_t genericAttributeSetHandle1;
    xme_core_directory_attributeSetHandle_t genericAttributeSetHandle2;
    xme_core_directory_attributeSetHandle_t genericAttributeSetHandle3;

    uint32_t value1;
    uint32_t value2;

    xme_core_directory_attributeHandle_t attributeHandle;

    xme_core_attribute_key_t attributeKey;
    uint32_t attributeValue;
    uint32_t attributeSize;

    xme_core_attribute_descriptor_list_t attributeDescriptorList1;
    xme_core_attribute_descriptor_list_t attributeDescriptorList2;
    xme_core_attribute_descriptor_list_t attributeDescriptorList3;
    xme_core_attribute_descriptor_t attributeDesriptors3[1];
    xme_core_attribute_descriptor_list_t attributeDescriptorList4;
    xme_core_attribute_descriptor_t attributeDesriptors4[2];

    xme_core_attribute_descriptor_list_t attributeDescriptorListOut1;
    xme_core_attribute_descriptor_list_t attributeDescriptorListOut2;
    
    xme_core_topic_t topic1;
    xme_core_topic_t topic2;
    xme_core_topic_t topic3;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
TEST_F(AttributeInterfaceTest, getAttributeDescriptorLists)
{
    uint8_t i;

    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_attribute_getAttributeDescriptorList(XME_CORE_TOPIC_INVALID_TOPIC, &attributeDescriptorList1)
    );

    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_attribute_getAttributeDescriptorList(topic1, NULL)
    );

    // Invalid parameters
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_attribute_getAttributeDescriptorList(XME_CORE_TOPIC_INVALID_TOPIC, NULL)
    );

    // Valid topic, but not registered yet
    EXPECT_EQ
    (
        XME_STATUS_NOT_FOUND,
        xme_core_directory_attribute_getAttributeDescriptorList(topic1, &attributeDescriptorList1)
    );

    // Valid topic, but not registered yet
    EXPECT_EQ
    (
        XME_STATUS_NOT_FOUND,
        xme_core_directory_attribute_getAttributeDescriptorList(topic2, &attributeDescriptorList1)
    );

    // Valid topic, but not registered yet
    EXPECT_EQ
    (
        XME_STATUS_NOT_FOUND,
        xme_core_directory_attribute_getAttributeDescriptorList(topic3, &attributeDescriptorList1)
    );

    // Register some topics
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic2, &attributeDescriptorList3, false)
    );
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic3, &attributeDescriptorList4, false)
    );

    // topic1 is still not registered
    EXPECT_EQ
    (
        XME_STATUS_NOT_FOUND,
        xme_core_directory_attribute_getAttributeDescriptorList(topic1, &attributeDescriptorList1)
    );

    // topic2 is registered
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_getAttributeDescriptorList(topic2, &attributeDescriptorListOut1)
    );

    // Check if returned attribute list matches the registered one
    EXPECT_EQ(attributeDescriptorList3.length, attributeDescriptorListOut1.length);
    for (i = 0; i < attributeDescriptorList3.length; i++)
    {
        EXPECT_EQ(attributeDescriptorList3.element[i].key, attributeDescriptorListOut1.element[i].key);
        EXPECT_EQ(attributeDescriptorList3.element[i].size, attributeDescriptorListOut1.element[i].size);
    }

    // topic3 is registered
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_getAttributeDescriptorList(topic3, &attributeDescriptorListOut2)
    );

    // Check if returned attribute list matches the registered one
    EXPECT_EQ(attributeDescriptorList3.length, attributeDescriptorListOut1.length);
    for (i = 0; i < attributeDescriptorList4.length; i++)
    {
        EXPECT_EQ(attributeDescriptorList4.element[i].key, attributeDescriptorListOut2.element[i].key);
        EXPECT_EQ(attributeDescriptorList4.element[i].size, attributeDescriptorListOut2.element[i].size);
    }
}

TEST_F(AttributeInterfaceTest, registerAttributeDescriptorLists)
{
    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_attribute_registerAttributeDescriptorList(XME_CORE_TOPIC_INVALID_TOPIC, &attributeDescriptorList1, false)
    );

    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_attribute_registerAttributeDescriptorList(XME_CORE_TOPIC_INVALID_TOPIC, &attributeDescriptorList1, true)
    );

    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic1, NULL, false)
    );

    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic1, NULL, true)
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic1, &attributeDescriptorList1, false)
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic2, &attributeDescriptorList2, false)
    );

    // Trying to registering second attribute descriptor list, when overwrite is false
    EXPECT_EQ
    (
        XME_STATUS_ALREADY_EXIST,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic1, &attributeDescriptorList2, false)
    );

    // Trying to registering second attribute descriptor list, when overwrite is true
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic1, &attributeDescriptorList2, true)
    );

    // Trying to registering second attribute descriptor list, when overwrite is false
    EXPECT_EQ
    (
        XME_STATUS_ALREADY_EXIST,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic2, &attributeDescriptorList2, false)
    );

    // Trying to registering second attribute descriptor list, when overwrite is true
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic2, &attributeDescriptorList2, true)
    );

    // Registering list with overwrite set to true and no pre-existing entry
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic3, &attributeDescriptorList2, true)
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic3, &attributeDescriptorList3, true)
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_attribute_registerAttributeDescriptorList(topic3, &attributeDescriptorList4, true)
    );
}

//----------------------------------------------------------------------------//
//     AttributeInterfaceTest: Create/remove handles                          //
//----------------------------------------------------------------------------//

TEST_F(AttributeInterfaceTest, createAndRemoveAttributeSets)
{
    genericAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();
    genericAttributeSetHandle2 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, genericAttributeSetHandle1);
    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, genericAttributeSetHandle2);
    EXPECT_NE(genericAttributeSetHandle1, genericAttributeSetHandle2);

    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(genericAttributeSetHandle1));
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(genericAttributeSetHandle2));
    ASSERT_FALSE(xme_core_directory_attribute_isAttributeSetHandleValid(genericAttributeSetHandle3));

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_directory_attribute_removeAttributeSet(genericAttributeSetHandle3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(genericAttributeSetHandle2));

    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(genericAttributeSetHandle1));
    ASSERT_FALSE(xme_core_directory_attribute_isAttributeSetHandleValid(genericAttributeSetHandle2));
    ASSERT_FALSE(xme_core_directory_attribute_isAttributeSetHandleValid(genericAttributeSetHandle3));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(genericAttributeSetHandle1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_directory_attribute_removeAttributeSet(genericAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, addPredefinedAttributeDefinitionsToAnAttributeSetWithDifferentInvalidParameters)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    // All invalid. It is supposed that the first check is the invalid handle. // TODO: In general I think this is a bad idea (assuming a ceratin order of checks). Perhaps just check against not XME_STATUS_SUCCESS?
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE,
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            genericAttributeSetHandle1,
            XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
            NULL,
            0,
            0,
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID,
            false
        )
    );

    // Handle set to an empty attribute set
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE,
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            genericAttributeSetHandle1,
            (xme_core_attribute_key_t) 1,
            &value1,
            1,
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
            false
        )
    );

    // Invalid handle
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // Invalid attribute key
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
            &value1,
            1, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // Invalid value
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            NULL,
            1, 
            sizeof(uint32_t), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // Invalid number of values
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            0,
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // Invalid size of value
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1,
            0,
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // Invalid datatype
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1,
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID, 
            false
        )
    );
}

TEST_F(AttributeInterfaceTest, addPredefinedAttributeDefinitionsToAnAttributeSetWithUnsupportedParameters)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    // Numeric data with length 2 are unsupported
    EXPECT_EQ(XME_STATUS_UNSUPPORTED, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            2, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // Array data with length 2 are unsupported
    EXPECT_EQ(XME_STATUS_UNSUPPORTED, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            2, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY, 
            false
        )
    );

    // Complex data with length 1 are unsupported
    EXPECT_EQ(XME_STATUS_UNSUPPORTED, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX, 
            false
        )
    );
}

TEST_F(AttributeInterfaceTest, addAndRemovePredefinedAttributeDefinitionsToAnAttributeSetWithValidParameters)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();
    defAttributeSetHandle2 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle2);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle2));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle2, 
            (xme_core_attribute_key_t) 2,
            &value1,
            1, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // defAttributeSetHandle1 only contains attribute with key 1
    EXPECT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 2
        )
    );

    // defAttributeSetHandle1 only contains attribute with key 2
    EXPECT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle2, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Remove defAttributeSetHandle1
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle2));

    // Remove defAttributeSetHandle2
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle2, 
            (xme_core_attribute_key_t) 2
        )
    );

    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle1));

    ASSERT_FALSE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle2));

    ASSERT_FALSE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));
    ASSERT_FALSE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle2));
}

TEST_F(AttributeInterfaceTest, addPredefinedAttributeFiltersToAnAttributeSetWithDifferentInvalidParameters)
{
    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));

    // All invalid. It is supposed that the first check is the invalid handle.
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            genericAttributeSetHandle1,
            XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
            NULL,
            0, 
            0, 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID,
            false
        )
    );

    // Handle set to an empty attribute set
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            genericAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Invalid handle
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Invalid attribute key
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            XME_CORE_ATTRIBUTE_KEY_UNDEFINED,
            &value1,
            1, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Invalid value
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            NULL,
            1,
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Invalid number of values
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            0, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Invalid size of value
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            0,
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Invalid datatype
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1,
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_INVALID, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Invalid operation
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_INVALID,
            false
        )
    );
}

TEST_F(AttributeInterfaceTest, addPredefinedAttributeFiltersToAnAttributeSetWithUnsupportedParameters)
{
    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));

    // Numeric data with length 2 are unsupported
    EXPECT_EQ(XME_STATUS_UNSUPPORTED, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            2,
            sizeof(value1),
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Array data with length 2 are unsupported
    EXPECT_EQ(XME_STATUS_UNSUPPORTED, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            2, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_ARRAY, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Complex data with length 1 are unsupported
    EXPECT_EQ(XME_STATUS_UNSUPPORTED, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_COMPLEX, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );
}

TEST_F(AttributeInterfaceTest, addAndRemovePredefinedAttributeFiltersToAnAttributeSetWithValidParameters)
{
    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();
    filterAttributeSetHandle2 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle2);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle2, 
            (xme_core_attribute_key_t) 2,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // filterAttributeSetHandle1 only contains attribute with key 1
    EXPECT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 2
        )
    );

    // filterAttributeSetHandle2 only contains attribute with key 2
    EXPECT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            filterAttributeSetHandle2, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Remove filterAttributeSetHandle1
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2));

    // Remove filterAttributeSetHandle2
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            filterAttributeSetHandle2, 
            (xme_core_attribute_key_t) 2
        )
    );

    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle1));

    ASSERT_FALSE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle2));

    ASSERT_FALSE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));
    ASSERT_FALSE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2));
}

TEST_F(AttributeInterfaceTest, addTwoAttributesWhereFirstOneIsDefinitionAndSecondOneIsFilter)
{
    // Create an attribute set
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    EXPECT_FALSE(xme_core_directory_attribute_isDefinition(defAttributeSetHandle1));
    EXPECT_FALSE(xme_core_directory_attribute_isFilter(defAttributeSetHandle1));

    // Add an attribute definition
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    EXPECT_TRUE(xme_core_directory_attribute_isDefinition(defAttributeSetHandle1));
    EXPECT_FALSE(xme_core_directory_attribute_isFilter(defAttributeSetHandle1));

    // Verify that adding an attribute filter fails
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 2,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    EXPECT_TRUE(xme_core_directory_attribute_isDefinition(defAttributeSetHandle1));
    EXPECT_FALSE(xme_core_directory_attribute_isFilter(defAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, addTwoAttributesWhereFirstOneIsFilterAndSecondOneIsDefinition)
{
    // Create an attribute set
    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));

    EXPECT_FALSE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle1));
    EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle1));

    // Add an attribute filter
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle1));
    EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle1));

    // Verify that adding an attribute definition fails
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 2,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle1));
    EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, getAttributeSetCountForInvalidHandles)
{
    // Invalid handles contain no attributes
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET));
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, getAttributeSetCountWithAdditionAndRemovalOfAttributesInOppositeOrderOfInsertion)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();
    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));

    // No attributes yet
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // One attribute here
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 2,
            &value2,
            1, 
            sizeof(value2), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // Two attributes here
    EXPECT_EQ(2U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 2
        )
    );

    // One attribute left
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // No attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, getAttributeSetCountWithAdditionAndRemovalOfAttributesInOrderOfInsertion)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();
    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));

    // No attributes yet
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // One attribute here
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 2,
            &value2,
            1, 
            sizeof(value2), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // Two attributes here
    EXPECT_EQ(2U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // One attribute left
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 2
        )
    );

    // No attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, addTwoAttributeDefinitionsWithOverwriteFlagSetToTrue)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    // No attributes yet
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // One attribute here
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    // Add the same attribute (with the same key) definition. 
    // The result should be SUCCESS because of the overwrite flag is set to true. 
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value2,
            1, 
            sizeof(value2), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            true
        )
    );

    // Note that only one attribute is contained in the attribute set
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    // Attempt to remove the attribute as a filter, this should fail
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE,
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Still one attribute left
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    // Now remove the attribute
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // No attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Still no attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 
}

TEST_F(AttributeInterfaceTest, addTwoAttributeFiltersWithOverwriteFlagSetToTrue)
{
    // Second with the filter first. 
    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));

    // No attributes yet
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // One attribute here
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    // Add the same attribute (with the same key) filter.
    // The result should be SUCCESS because of the overwrite flag is set to true.
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value2,
            1, 
            sizeof(value2), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            true
        )
    );

    // Note that only one attribute is contained in the attribute set
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    // Attempt to remove the attribute as a definition, this should fail
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE,
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Still one attribute left
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    // Now remove the attribute
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // No attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, 
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Still no attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 
}

TEST_F(AttributeInterfaceTest, addTwoAttributeDefinitionsWithOverwriteFlagSetToFalse)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    // No attributes yet
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // One attribute here
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    // Add the same attribute (with the same key) definition. 
    // The result should be ALREADY_EXISTS because of the overwrite flag is set to false. 
    EXPECT_EQ(XME_STATUS_ALREADY_EXIST, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value2,
            1, 
            sizeof(value2), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );

    // Note that only one attribute is contained in the attribute set
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    // Attempt to remove the attribute as a filter, this should fail
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE,
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Still one attribute left
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    // Now remove the attribute
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // No attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, 
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Still no attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 
}

TEST_F(AttributeInterfaceTest, addTwoAttributeFiltersWithOverwriteFlagSetToFalse)
{
    // Second with the filter first. 
    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));

    // No attributes yet
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // One attribute here
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    // Add the same attribute (with the same key) filter.
    // The result should be ALREADY_EXISTS because of the overwrite flag is set to false. 
    EXPECT_EQ(XME_STATUS_ALREADY_EXIST, 
        xme_core_directory_attribute_addPredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value2,
            1, 
            sizeof(value2), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
            false
        )
    );

    // Note that only one attribute is contained in the attribute set
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    // Attempt to remove the attribute as a definition, this should fail
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE,
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Still one attribute left
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    // Now remove the attribute
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // No attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, 
        xme_core_directory_attribute_removePredefinedAttributeFilter
        (
            filterAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1
        )
    );

    // Still no attributes left
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(filterAttributeSetHandle1)); 
}

TEST_F(AttributeInterfaceTest, doubleInitializationOfAnIterator)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_initIterator(defAttributeSetHandle1));
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_directory_attribute_initIterator(defAttributeSetHandle1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, doubleFinalizationOfAnIterator)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_initIterator(defAttributeSetHandle1));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, finiIteratorUninitialized)
{
    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, iterationOverAnEmptyAttributeSet)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_directory_attribute_initIterator(genericAttributeSetHandle1));

    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
    ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

    // Attribute set with no attributes
    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_initIterator(defAttributeSetHandle1));
    {
        // Ensure that attribute handle gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;

        EXPECT_FALSE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle)); 
        EXPECT_EQ(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));

    EXPECT_EQ(0U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 
}

TEST_F(AttributeInterfaceTest, iterationOverAnAttributeSetWithOneAttribute)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_directory_attribute_initIterator(genericAttributeSetHandle1));

    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    // Attribute set with one attribute
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_initIterator(defAttributeSetHandle1));
    {
        // Ensure that attribute handle and key gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;
        attributeKey = (xme_core_attribute_key_t) 43;
        attributeSize = 0x1234;
        attributeValue = 0xFEDCBA98U;

        EXPECT_TRUE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle));
        ASSERT_NE(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeKey(defAttributeSetHandle1, attributeHandle, &attributeKey));
        ASSERT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, attributeKey);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeValue(defAttributeSetHandle1, attributeKey, &attributeValue, sizeof(attributeValue), &attributeSize));

        EXPECT_EQ((xme_core_attribute_key_t) 1, attributeKey);
        EXPECT_EQ(value1, attributeValue);

        // Ensure that attribute handle gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;

        EXPECT_FALSE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle));
        EXPECT_EQ(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));

    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 
}

TEST_F(AttributeInterfaceTest, iterationOverAnAttributeSetWithTwoAttributes)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_directory_attribute_initIterator(genericAttributeSetHandle1));

    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    // Attribute set with two attributes
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 3,
            &value2,
            1, 
            sizeof(value2), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );
    EXPECT_EQ(2U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_initIterator(defAttributeSetHandle1));
    {
        // Ensure that attribute handle and key gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;
        attributeKey = (xme_core_attribute_key_t) 43;
        attributeSize = 0x1234;
        attributeValue = 0xFEDCBA98U;

        EXPECT_TRUE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle));
        ASSERT_NE(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeKey(defAttributeSetHandle1, attributeHandle, &attributeKey));
        ASSERT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, attributeKey);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeValue(defAttributeSetHandle1, attributeKey, &attributeValue, sizeof(attributeValue), &attributeSize));

        EXPECT_EQ((xme_core_attribute_key_t) 1, attributeKey);
        EXPECT_EQ(value1, attributeValue);

        // Ensure that attribute handle and key gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;
        attributeKey = (xme_core_attribute_key_t) 43;
        attributeSize = 0x1234;
        attributeValue = 0xFEDCBA98U;

        EXPECT_TRUE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle));
        ASSERT_NE(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeKey(defAttributeSetHandle1, attributeHandle, &attributeKey));
        ASSERT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, attributeKey);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeValue(defAttributeSetHandle1, attributeKey, &attributeValue, sizeof(attributeValue), &attributeSize));

        EXPECT_EQ((xme_core_attribute_key_t) 3, attributeKey);
        EXPECT_EQ(value2, attributeValue);

        // Ensure that attribute handle gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;

        EXPECT_FALSE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle)); 
        EXPECT_EQ(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));

    EXPECT_EQ(2U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 
}

TEST_F(AttributeInterfaceTest, iterationOverAnAttributeSetWithThreeAttributes)
{
    ASSERT_EQ(XME_STATUS_INVALID_HANDLE, xme_core_directory_attribute_initIterator(genericAttributeSetHandle1));

    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    // Attribute set with three attributes
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 1,
            &value1,
            1, 
            sizeof(value1), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 3,
            &value2,
            1, 
            sizeof(value2), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );
    EXPECT_EQ(XME_STATUS_SUCCESS, 
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1, 
            (xme_core_attribute_key_t) 2,
            &value2,
            1, 
            sizeof(value2), 
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC, 
            false
        )
    );
    EXPECT_EQ(3U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_initIterator(defAttributeSetHandle1));
    {
        // Ensure that attribute handle and key gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;
        attributeKey = (xme_core_attribute_key_t) 43;
        attributeSize = 0x1234;
        attributeValue = 0xFEDCBA98U;

        EXPECT_TRUE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle));
        ASSERT_NE(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeKey(defAttributeSetHandle1, attributeHandle, &attributeKey));
        ASSERT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, attributeKey);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeValue(defAttributeSetHandle1, attributeKey, &attributeValue, sizeof(attributeValue), &attributeSize));

        EXPECT_EQ((xme_core_attribute_key_t) 1, attributeKey);
        EXPECT_EQ(value1, attributeValue);

        // Ensure that attribute handle and key gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;
        attributeKey = (xme_core_attribute_key_t) 43;
        attributeSize = 0x1234;
        attributeValue = 0xFEDCBA98U;

        EXPECT_TRUE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle));
        ASSERT_NE(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeKey(defAttributeSetHandle1, attributeHandle, &attributeKey));
        ASSERT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, attributeKey);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeValue(defAttributeSetHandle1, attributeKey, &attributeValue, sizeof(attributeValue), &attributeSize));

        EXPECT_EQ((xme_core_attribute_key_t) 3, attributeKey);
        EXPECT_EQ(value2, attributeValue);

        // Ensure that attribute handle and key gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;
        attributeKey = (xme_core_attribute_key_t) 43;
        attributeSize = 0x1234;
        attributeValue = 0xFEDCBA98U;

        EXPECT_TRUE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle));
        ASSERT_NE(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeKey(defAttributeSetHandle1, attributeHandle, &attributeKey));
        ASSERT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, attributeKey);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeValue(defAttributeSetHandle1, attributeKey, &attributeValue, sizeof(attributeValue), &attributeSize));

        EXPECT_EQ((xme_core_attribute_key_t) 2, attributeKey);
        EXPECT_EQ(value2, attributeValue);

        // Ensure that attribute handle gets reset
        attributeHandle = (xme_core_directory_attributeHandle_t) 42;

        EXPECT_FALSE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
        EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle)); 
        EXPECT_EQ(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
    }
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));

    EXPECT_EQ(3U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 
}

TEST_F(AttributeInterfaceTest, getPrimitiveValueWithTooSmallBufferSize)
{
    unsigned int d;

    const xme_core_directory_attribute_datatype_t datatypes[] = {
        XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
        XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
        XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL
    };

    int64_t numericValue = 0x0123456789ABCDEFL;
    uint64_t unsignedValue = 0x0123456789ABCDEFUL;
    double decimalValue = 0.1234567890123456789;

    const void* valuePointers[] = {
        &numericValue,
        &unsignedValue,
        &decimalValue
    };

    const size_t valueSizes[] = {
        sizeof(numericValue),
        sizeof(unsignedValue),
        sizeof(decimalValue)
    };

    const size_t maxSize = 8U;

    uint8_t key = 0;

    for (d = 0U; d < sizeof(datatypes)/sizeof(datatypes[0U]); d++)
    {
        uint8_t values[maxSize];
        unsigned int i, j;
        uint16_t size;

        ASSERT_LE(valueSizes[d], maxSize);

        for (i = 0; i < sizeof(values); i++)
        {
            values[i] = 0x55U;
        }

        defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

        // Add one attribute
        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeDefinition
            (
                defAttributeSetHandle1,
                (xme_core_attribute_key_t) ++key,
                valuePointers[d],
                1,
                valueSizes[d],
                datatypes[d],
                false
            )
        );
        EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));

        for (i = 1U; i <= valueSizes[d]; i++)
        {
            uint8_t buf[maxSize];

            for (j = 0U; j < sizeof(values); j++)
            {
                buf[j] = values[j];
            }

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_initIterator(defAttributeSetHandle1));
            {
                // Ensure that attribute handle and key gets reset
                attributeHandle = (xme_core_directory_attributeHandle_t) 42;
                attributeKey = (xme_core_attribute_key_t) 43;
                attributeSize = 0x1234;
                size = maxSize - i; // i bytes too small

                EXPECT_TRUE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle));
                ASSERT_NE(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
                EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeKey(defAttributeSetHandle1, attributeHandle, &attributeKey));
                ASSERT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, attributeKey);
                EXPECT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_directory_attribute_getAttributeValue(defAttributeSetHandle1, attributeKey, &buf, size, &attributeSize));
                EXPECT_EQ(valueSizes[d], attributeSize);

                for (j = 0; j < size; j++)
                {
                    EXPECT_EQ(((uint8_t*) valuePointers[d])[j], buf[j]);
                }

                for (j = 1U; j <= i; j++)
                {
                    EXPECT_EQ(values[sizeof(values) - j], buf[sizeof(values) - j]); // this field must not be overwritten
                }
            }
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));
        }

        EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1)); 

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_removePredefinedAttributeDefinition
            (
                defAttributeSetHandle1,
                (xme_core_attribute_key_t) key
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle1));
    }
}

TEST_F(AttributeInterfaceTest, getStringValueWithTooSmallBufferSize)
{
    char string[] = "This is a test string value.";
    const size_t bufSize = 29UL;

    size_t totalSize = strlen(string) + 1U;

    char values[bufSize];
    unsigned int i, j;
    uint32_t size;

    ASSERT_EQ(totalSize, bufSize);

    for (i = 0; i < sizeof(values); i++)
    {
        values[i] = 0x55U;
    }

    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    // Add one attribute
    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_addPredefinedAttributeDefinition
        (
            defAttributeSetHandle1,
            (xme_core_attribute_key_t) 1,
            string,
            1,
            totalSize,
            XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
            false
        )
    );
    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));

    for (i = 1U; i <= totalSize; i++)
    {
        char buf[bufSize];

        for (j = 0U; j < sizeof(values); j++)
        {
            buf[j] = values[j];
        }

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_initIterator(defAttributeSetHandle1));
        {
            // Ensure that attribute handle and key gets reset
            attributeHandle = (xme_core_directory_attributeHandle_t) 42;
            attributeKey = (xme_core_attribute_key_t) 43;
            attributeSize = 0x1234;
            size = totalSize - i; // i bytes too small

            EXPECT_TRUE(xme_core_directory_attribute_hasNextAttributeHandle(defAttributeSetHandle1));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getNextAttributeHandle(defAttributeSetHandle1, &attributeHandle));
            ASSERT_NE(XME_CORE_ATTRIBUTE_INVALID_ATTRIBUTE_HANDLE, attributeHandle);
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_getAttributeKey(defAttributeSetHandle1, attributeHandle, &attributeKey));
            ASSERT_NE(XME_CORE_ATTRIBUTE_KEY_UNDEFINED, attributeKey);
            EXPECT_EQ(XME_STATUS_BUFFER_TOO_SMALL, xme_core_directory_attribute_getAttributeValue(defAttributeSetHandle1, attributeKey, &buf, size, &attributeSize));
            EXPECT_EQ(totalSize, attributeSize);

            if (size > 0U)
            {
                for (j = 0; j < size - 1U; j++)
                {
                    EXPECT_EQ(string[j], buf[j]);
                }

                // Ensure NULL-termination
                EXPECT_EQ(0, buf[size - 1U]);
            }

            for (j = 1U; j <= i; j++)
            {
                EXPECT_EQ(values[sizeof(values) - j], buf[sizeof(values) - j]); // this field must not be overwritten
            }
        }
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_finiIterator(defAttributeSetHandle1));
    }

    EXPECT_EQ(1U, xme_core_directory_attribute_getAttributeSetCount(defAttributeSetHandle1));

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_removePredefinedAttributeDefinition
        (
            defAttributeSetHandle1,
            (xme_core_attribute_key_t) 1
        )
    );

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle1));
}

TEST_F(AttributeInterfaceTest, isFilterMatchingDefinitionWithEmptyFilterAndEmptyDefinition)
{
    int matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, // empty
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, // empty
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

TEST_F(AttributeInterfaceTest, isFilterMatchingDefinitionWithEmptyFilterAndBlankDefinition)
{
    int matches = false;

    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, // empty
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!!matches);

    xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle1);
}

TEST_F(AttributeInterfaceTest, isFilterMatchingDefinitionWithBlankFilterAndEmptyDefinition)
{
    int matches = false;

    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle1, // blank
            XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, // empty
            &matches
        )
    );

    EXPECT_TRUE(!!matches);

    xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle1);
}

TEST_F(AttributeInterfaceTest, isFilterMatchingDefinitionWithBlankFilterAndBlankDefinition)
{
    int matches = false;

    defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();
    filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle1, // blank
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!!matches);

    xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle1);
    xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle1);
}

// TODO: Add here tests that invalidate the set values and then read the original values back.
//       This ensures that the API internally copies the values.

// TODO: Add here tests for the function ::xme_core_directory_attribute_getAttributeKeyDatatype
//       Note: this function can work without a handle, so we should review the signature. 

// TODO: Add here tests for adding data of different datatypes. 
//       Note: the solution here is to create a new class with mixed datatypes. 

// TODO: Add here tests for the funtion ::xme_core_directory_attribute_isFilterAcceptingDefinition (to be defined what is accepting a definition for a filter)

// TODO: Add here tests for the funtion ::xme_core_directory_attribute_compareAttributes (to be defined how do we implement this)

// TODO: Add test for adding numeric filters and definitions that do not take the full 64-bit size

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
