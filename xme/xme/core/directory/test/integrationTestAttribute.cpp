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
 * $Id: integrationTestAttribute.cpp 5416 2013-10-08 08:32:31Z ruiz $
 */

/**
 * \file
 *         Attribute abstraction integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/directory/include/attribute.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class AttributeIntegrationTest: public ::testing::Test
{
protected:
    AttributeIntegrationTest()
    : defAttributeSetHandle1(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , defAttributeSetHandle2(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , defAttributeSetHandle3(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , defAttributeSetHandle4a(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , defAttributeSetHandle4b(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , defAttributeSetHandle4c(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle1(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle2a(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle2b(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle2c(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle2d(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle2e(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle2f(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle3a(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle3b(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle3c(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle3d(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle3e(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle3f(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle4(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle5a(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle5b(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , filterAttributeSetHandle5c(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET)
    , numericKey((xme_core_attribute_key_t) 1)
    , unsignedKey((xme_core_attribute_key_t) 2)
    , decimalKey((xme_core_attribute_key_t) 3)
    , stringKey((xme_core_attribute_key_t) 4)
    , numericValue(1)
    , unsignedValue(2U)
    , decimalValue(3.0)
    , stringValue("String")
    , matches(false)
    {
    }

    virtual ~AttributeIntegrationTest()
    {
    }

    virtual void SetUp()
    {
        // We initialize the attribute sets as follows:
        //
        // defAttributeSetHandle1:
        //  - (empty)
        //
        // defAttributeSetHandle2:
        //  - numericKey with numericValue
        //
        // defAttributeSetHandle3:
        //  - numericKey with numericValue
        //  - unsignedKey with unsignedValue
        //  - decimalKey with decimalValue
        //  - stringKey with stringValue
        //
        // defAttributeSetHandle4a:
        //  - stringKey with stringValue, valueSize 0
        ASSERT_GT(strlen(stringValue), 0U);
        //
        // defAttributeSetHandle4a:
        //  - stringKey with stringValue, valueSize 3
        ASSERT_GT(strlen(stringValue), 3U);
        //
        // defAttributeSetHandle4a:
        //  - stringKey with stringValue, valueSize 10
        ASSERT_LT(strlen(stringValue), 10U);
        //
        // filterAttributeSetHandle1:
        //  - (empty)
        //
        // filterAttributeSetHandle2a:
        //  - numericKey with numericValue and operator ==
        //
        // filterAttributeSetHandle2b:
        //  - numericKey with numericValue and operator !=
        //
        // filterAttributeSetHandle2c:
        //  - numericKey with numericValue and operator <=
        //
        // filterAttributeSetHandle2d:
        //  - numericKey with numericValue and operator >=
        //
        // filterAttributeSetHandle2e:
        //  - numericKey with numericValue and operator <
        //
        // filterAttributeSetHandle2f:
        //  - numericKey with numericValue and operator >
        //
        // filterAttributeSetHandle3a:
        //  - numericKey with numericValue and operator ==
        //  - unsignedKey with unsignedValue and operator ==
        //
        // filterAttributeSetHandle3b:
        //  - numericKey with numericValue and operator ==
        //  - unsignedKey with unsignedValue and operator !=
        //
        // filterAttributeSetHandle3c:
        //  - numericKey with numericValue and operator ==
        //  - unsignedKey with unsignedValue and operator <=
        //
        // filterAttributeSetHandle3d:
        //  - numericKey with numericValue and operator ==
        //  - unsignedKey with unsignedValue and operator >=
        //
        // filterAttributeSetHandle3e:
        //  - numericKey with numericValue and operator ==
        //  - unsignedKey with unsignedValue and operator <
        //
        // filterAttributeSetHandle3f:
        //  - numericKey with numericValue and operator ==
        //  - unsignedKey with unsignedValue and operator >
        //
        // filterAttributeSetHandle4:
        //  - numericKey with numericValue and operator ==
        //  - unsignedKey with unsignedValue and operator ==
        //  - decimalKey with decimalValue and operator ==
        //  - stringKey with stringValue and operator ==
        //
        // filterAttributeSetHandle5a:
        //  - stringKey with stringValue, valueSize 0
        ASSERT_GT(strlen(stringValue), 0U);
        //
        // filterAttributeSetHandle5b:
        //  - stringKey with stringValue, valueSize 3
        ASSERT_GT(strlen(stringValue), 3U);
        //
        // filterAttributeSetHandle5c:
        //  - stringKey with stringValue, valueSize 10
        ASSERT_LT(strlen(stringValue), 10U);

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_init());

        // defAttributeSetHandle1
        defAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle1);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle1));

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(defAttributeSetHandle1));
        EXPECT_FALSE(xme_core_directory_attribute_isFilter(defAttributeSetHandle1));

        // defAttributeSetHandle2
        defAttributeSetHandle2 = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle2);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle2));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeDefinition
            (
                defAttributeSetHandle2,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                false
            )
        );

        EXPECT_TRUE(xme_core_directory_attribute_isDefinition(defAttributeSetHandle2));
        EXPECT_FALSE(xme_core_directory_attribute_isFilter(defAttributeSetHandle2));

        // defAttributeSetHandle3
        defAttributeSetHandle3 = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle3);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle3));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeDefinition
            (
                defAttributeSetHandle3,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeDefinition
            (
                defAttributeSetHandle3,
                unsignedKey,
                &unsignedValue,
                1,
                sizeof(unsignedValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeDefinition
            (
                defAttributeSetHandle3,
                decimalKey,
                &decimalValue,
                1,
                sizeof(decimalValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeDefinition
            (
                defAttributeSetHandle3,
                stringKey,
                stringValue,
                1,
                strlen(stringValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                false
            )
        );

        EXPECT_TRUE(xme_core_directory_attribute_isDefinition(defAttributeSetHandle3));
        EXPECT_FALSE(xme_core_directory_attribute_isFilter(defAttributeSetHandle3));

        // defAttributeSetHandle4a
        defAttributeSetHandle4a = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle4a);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle4a));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeDefinition
            (
                defAttributeSetHandle4a,
                stringKey,
                stringValue,
                1,
                0U, // valueSize 0
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                false
            )
        );

        EXPECT_TRUE(xme_core_directory_attribute_isDefinition(defAttributeSetHandle4a));
        EXPECT_FALSE(xme_core_directory_attribute_isFilter(defAttributeSetHandle4a));

        // defAttributeSetHandle4b
        defAttributeSetHandle4b = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle4b);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle4b));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeDefinition
            (
                defAttributeSetHandle4b,
                stringKey,
                stringValue,
                1,
                3U, // valueSize 3
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                false
            )
        );

        EXPECT_TRUE(xme_core_directory_attribute_isDefinition(defAttributeSetHandle4b));
        EXPECT_FALSE(xme_core_directory_attribute_isFilter(defAttributeSetHandle4b));

        // defAttributeSetHandle4c
        defAttributeSetHandle4c = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, defAttributeSetHandle4c);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(defAttributeSetHandle4c));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeDefinition
            (
                defAttributeSetHandle4c,
                stringKey,
                stringValue,
                1,
                10U, // valueSize 10
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                false
            )
        );

        EXPECT_TRUE(xme_core_directory_attribute_isDefinition(defAttributeSetHandle4c));
        EXPECT_FALSE(xme_core_directory_attribute_isFilter(defAttributeSetHandle4c));

        // filterAttributeSetHandle1
        filterAttributeSetHandle1 = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle1);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle1));

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle1));
        EXPECT_FALSE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle1));

        // filterAttributeSetHandle2a
        filterAttributeSetHandle2a = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle2a);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2a));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle2a,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle2a));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle2a));

        // filterAttributeSetHandle2b
        filterAttributeSetHandle2b = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle2b);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2b));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle2b,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_UNEQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle2b));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle2b));

        // filterAttributeSetHandle2c
        filterAttributeSetHandle2c = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle2c);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2c));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle2c,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS_OR_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle2c));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle2c));

        // filterAttributeSetHandle2d
        filterAttributeSetHandle2d = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle2d);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2d));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle2d,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER_OR_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle2d));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle2d));

        // filterAttributeSetHandle2e
        filterAttributeSetHandle2e = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle2e);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2e));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle2e,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle2e));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle2e));

        // filterAttributeSetHandle2f
        filterAttributeSetHandle2f = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle2f);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle2f));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle2f,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle2f));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle2f));

        // filterAttributeSetHandle3a
        filterAttributeSetHandle3a = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle3a);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle3a));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3a,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3a,
                unsignedKey,
                &unsignedValue,
                1,
                sizeof(unsignedValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle3a));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle3a));

        // filterAttributeSetHandle3b
        filterAttributeSetHandle3b = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle3b);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle3b));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3b,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3b,
                unsignedKey,
                &unsignedValue,
                1,
                sizeof(unsignedValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_UNEQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle3b));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle3b));

        // filterAttributeSetHandle3c
        filterAttributeSetHandle3c = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle3c);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle3c));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3c,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3c,
                unsignedKey,
                &unsignedValue,
                1,
                sizeof(unsignedValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS_OR_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle3c));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle3c));

        // filterAttributeSetHandle3d
        filterAttributeSetHandle3d = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle3d);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle3d));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3d,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3d,
                unsignedKey,
                &unsignedValue,
                1,
                sizeof(unsignedValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER_OR_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle3d));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle3d));

        // filterAttributeSetHandle3e
        filterAttributeSetHandle3e = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle3e);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle3e));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3e,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3e,
                unsignedKey,
                &unsignedValue,
                1,
                sizeof(unsignedValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_LESS,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle3e));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle3e));

        // filterAttributeSetHandle3f
        filterAttributeSetHandle3f = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle3f);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle3f));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3f,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle3f,
                unsignedKey,
                &unsignedValue,
                1,
                sizeof(unsignedValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_GREATER,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle3f));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle3f));

        // filterAttributeSetHandle4
        filterAttributeSetHandle4 = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle4);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle4));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle4,
                numericKey,
                &numericValue,
                1,
                sizeof(numericValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_NUMERIC,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle4,
                unsignedKey,
                &unsignedValue,
                1,
                sizeof(unsignedValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_UNSIGNED,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle4,
                decimalKey,
                &decimalValue,
                1,
                sizeof(decimalValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_DECIMAL,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle4,
                stringKey,
                stringValue,
                1,
                strlen(stringValue),
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle4));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle4));

        // filterAttributeSetHandle5a
        filterAttributeSetHandle5a = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle5a);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle5a));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle5a,
                stringKey,
                stringValue,
                1,
                0U, // valueSize 0
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle5a));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle5a));

        // filterAttributeSetHandle5b
        filterAttributeSetHandle5b = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle5b);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle5b));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle5b,
                stringKey,
                stringValue,
                1,
                3U, // valueSize 3
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle5b));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle5b));

        // filterAttributeSetHandle5c
        filterAttributeSetHandle5c = xme_core_directory_attribute_createAttributeSet();
        ASSERT_NE(XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET, filterAttributeSetHandle5c);
        ASSERT_TRUE(xme_core_directory_attribute_isAttributeSetHandleValid(filterAttributeSetHandle5c));

        EXPECT_EQ(XME_STATUS_SUCCESS,
            xme_core_directory_attribute_addPredefinedAttributeFilter
            (
                filterAttributeSetHandle5c,
                stringKey,
                stringValue,
                1,
                10U, // valueSize 10
                XME_CORE_DIRECTORY_ATTRIBUTE_DATATYPE_STRING,
                XME_CORE_DIRECTORY_ATTRIBUTE_FILTEROPERATION_EQUAL,
                false
            )
        );

        EXPECT_FALSE(xme_core_directory_attribute_isDefinition(filterAttributeSetHandle5c));
        EXPECT_TRUE(xme_core_directory_attribute_isFilter(filterAttributeSetHandle5c));
    }

    virtual void TearDown()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle5c));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle5b));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle5a));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle4));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle3f));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle3e));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle3d));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle3c));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle3b));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle3a));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle2f));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle2e));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle2d));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle2c));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle2b));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle2a));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(filterAttributeSetHandle1));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle4c));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle4b));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle4a));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle3));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle2));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_removeAttributeSet(defAttributeSetHandle1));

        xme_core_directory_attribute_fini();
    }

    xme_core_directory_attributeSetHandle_t defAttributeSetHandle1;
    xme_core_directory_attributeSetHandle_t defAttributeSetHandle2;
    xme_core_directory_attributeSetHandle_t defAttributeSetHandle3;
    xme_core_directory_attributeSetHandle_t defAttributeSetHandle4a;
    xme_core_directory_attributeSetHandle_t defAttributeSetHandle4b;
    xme_core_directory_attributeSetHandle_t defAttributeSetHandle4c;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle1;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle2a;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle2b;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle2c;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle2d;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle2e;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle2f;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle3a;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle3b;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle3c;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle3d;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle3e;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle3f;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle4;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle5a;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle5b;
    xme_core_directory_attributeSetHandle_t filterAttributeSetHandle5c;

    xme_core_attribute_key_t numericKey;
    xme_core_attribute_key_t unsignedKey;
    xme_core_attribute_key_t decimalKey;
    xme_core_attribute_key_t stringKey;

    int64_t numericValue;
    uint16_t unsignedValue;
    double decimalValue;
    const char* stringValue;

    int matches;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     AttributeIntegrationTest                                               //
//----------------------------------------------------------------------------//

// filterAttributeSetHandle1 with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter1AndVoidDefinition)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle1, // blank
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter1AndNumericDefinition)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle1, // blank
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter1AndVariousDefinitions)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle1, // blank
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// filterAttributeSetHandle2a with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2aAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2a, // numeric '=='
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2aAndNumericDefinition)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2a, // numeric '=='
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2aAndVariousDefinitions)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2a, // numeric '=='
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// filterAttributeSetHandle2b with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2bAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2b, // numeric '!='
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2bAndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2b, // numeric '!='
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2bAndVariousDefinitions)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2b, // numeric '!='
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

// filterAttributeSetHandle2c with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2cAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2c, // numeric '<='
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2cAndNumericDefinition)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2c, // numeric '<='
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2cAndVariousDefinitions)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2c, // numeric '<='
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// filterAttributeSetHandle2d with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2dAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2d, // numeric '>='
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2dAndNumericDefinition)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2d, // numeric '>='
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2dAndVariousDefinitions)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2d, // numeric '>='
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// filterAttributeSetHandle2e with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2eAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2e, // numeric '<'
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2eAndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2e, // numeric '<'
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2eAndVariousDefinitions)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2e, // numeric '<'
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

// filterAttributeSetHandle2f with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2fAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2f, // numeric '>'
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2fAndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2f, // numeric '>'
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter2fAndVariousDefinitions)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle2f, // numeric '>'
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

// filterAttributeSetHandle3a with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3aAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3a, // numeric '==', unsigned '=='
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3aAndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3a, // numeric '==', unsigned '=='
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3aAndVariousDefinitions)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3a, // numeric '==', unsigned '=='
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// filterAttributeSetHandle3b with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3bAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3b, // numeric '==', unsigned '!='
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3bAndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3b, // numeric '==', unsigned '!='
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3bAndVariousDefinitions)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3b, // numeric '==', unsigned '!='
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

// filterAttributeSetHandle3c with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3cAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3c, // numeric '==', unsigned '<='
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3cAndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3c, // numeric '==', unsigned '<='
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3cAndVariousDefinitions)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3c, // numeric '==', unsigned '<='
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// filterAttributeSetHandle3d with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3dAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3d, // numeric '==', unsigned '>='
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3dAndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3d, // numeric '==', unsigned '>='
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3dAndVariousDefinitions)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3d, // numeric '==', unsigned '>='
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// filterAttributeSetHandle3e with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3eAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3e, // numeric '==', unsigned '<'
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3eAndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3e, // numeric '==', unsigned '<'
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3eAndVariousDefinitions)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3e, // numeric '==', unsigned '<'
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

// filterAttributeSetHandle3f with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3fAndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3f, // numeric '==', unsigned '>'
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3fAndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3f, // numeric '==', unsigned '>'
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter3fAndVariousDefinitions)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle3f, // numeric '==', unsigned '>'
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

// filterAttributeSetHandle4 with defAttributeSetHandle{1,2,3}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter4AndVoidDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle4, // numeric '==', unsigned '==', decimal '==', string '=='
            defAttributeSetHandle1, // blank
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithFilter4AndNumericDefinition)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle4, // numeric '==', unsigned '==', decimal '==', string '=='
            defAttributeSetHandle2, // numeric
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinition3WithFilter4AndVariousDefinitions)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle4, // numeric '==', unsigned '==', decimal '==', string '=='
            defAttributeSetHandle3, // various
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// filterAttributeSetHandle5a with defAttributeSetHandle4{a,b,c}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithStringFilterAutodetectAndDefinitionAutodetect)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle5a, // string, valueSize 0 --> "String"
            defAttributeSetHandle4a, // string, valueSize 0 --> "String"
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithStringFilterAutodetectAndDefinitionTruncated)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle5a, // string, valueSize 0 --> "String"
            defAttributeSetHandle4b, // string, valueSize 3 --> "Str"
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithStringFilterAutodetectAndDefinitionTooLong)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle5a, // string, valueSize 0 --> "String"
            defAttributeSetHandle4c, // string, valueSize 10 --> "String"
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// filterAttributeSetHandle5b with defAttributeSetHandle4{a,b,c}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithStringFilterTruncatedAndDefinitionAutodetect)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle5b, // string, valueSize 3 --> "Str"
            defAttributeSetHandle4a, // string, valueSize 0 --> "String"
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithStringFilterTruncatedAndDefinitionTruncated)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle5b, // string, valueSize 3 --> "Str"
            defAttributeSetHandle4b, // string, valueSize 3 --> "Str"
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithStringFilterTruncatedAndDefinitionTooLong)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle5b, // string, valueSize 3 --> "Str"
            defAttributeSetHandle4c, // string, valueSize 10 --> "String"
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

// filterAttributeSetHandle5c with defAttributeSetHandle4{a,b,c}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithStringFilterTooLongAndDefinitionAutodetect)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle5c, // string, valueSize 10 --> "String"
            defAttributeSetHandle4a, // string, valueSize 0 --> "String"
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithStringFilterTooLongAndDefinitionTruncated)
{
    matches = true;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle5c, // string, valueSize 10 --> "String"
            defAttributeSetHandle4b, // string, valueSize 3 --> "Str"
            &matches
        )
    );

    EXPECT_TRUE(!matches);
}

TEST_F(AttributeIntegrationTest, isFilterMatchingDefinitionWithStringFilterTooLongAndDefinitionTooLong)
{
    matches = false;

    EXPECT_EQ(XME_STATUS_SUCCESS,
        xme_core_directory_attribute_isFilterMatchingDefinition
        (
            filterAttributeSetHandle5c, // string, valueSize 10 --> "String"
            defAttributeSetHandle4c, // string, valueSize 10 --> "String"
            &matches
        )
    );

    EXPECT_TRUE(!!matches);
}

// TODO: Add test for existance and non-existance operators

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
