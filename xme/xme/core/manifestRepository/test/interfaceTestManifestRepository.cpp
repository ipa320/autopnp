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
 * $Id: interfaceTestManifestRepository.cpp 5517 2013-10-16 14:46:14Z ruiz $
 */

/**
 * \file
 *         Manifest Repository interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/manifestRepository/include/manifestRepository.h"

#include "xme/core/testUtils.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/*****************************************************************************/
class ManifestRepositoryInterfaceTest: public ::testing::Test
{
protected:

    ManifestRepositoryInterfaceTest()
    : componentType0(XME_CORE_COMPONENT_TYPE_INVALID)
	, componentType1((xme_core_componentType_t)1)
    , componentType2((xme_core_componentType_t)2)
    , componentType3((xme_core_componentType_t)3)
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());

		// INVALID COMPONENT MANIFEST
        componentManifest0 = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));
        componentManifest0->componentType = componentType0;
        (void) xme_hal_safeString_strncpy(componentManifest0->name, "testComponent", sizeof(componentManifest0->name));

        // FIRST COMPONENT MANIFEST
        componentManifest1 = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));
        componentManifest1->componentType = componentType1;
        (void) xme_hal_safeString_strncpy(componentManifest1->name, "testComponent", sizeof(componentManifest1->name));

        //Function
        functionManifest1 = &componentManifest1->functionManifests[0];
        functionManifest1->functionId = (xme_core_component_functionId_t) 1;
        (void) xme_hal_safeString_strncpy(functionManifest1->name, "testFunction", sizeof(functionManifest1->name));
        functionManifest1->wcet = 100000000;
        functionManifest1->alphaCurve.alphaCurve = 0;
        functionManifest1->completion = true;

        //Port
        portManifest1 = &componentManifest1->portManifests[0];
        portManifest1->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
        portManifest1->topic = XME_CORE_TOPIC(4098); 
        portManifest1->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;

        // SECOND COMPONENT MANIFEST
        componentManifest2 = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));
        componentManifest2->componentType = componentType2;
        (void) xme_hal_safeString_strncpy(componentManifest2->name, "testComponent2", sizeof(componentManifest2->name));

        //Function
        functionManifest2 = &componentManifest2->functionManifests[0];
        functionManifest2->functionId = (xme_core_component_functionId_t) 2;
        (void) xme_hal_safeString_strncpy(functionManifest2->name, "testFunction2", sizeof(functionManifest2->name));
        functionManifest2->wcet = 200000000;
        functionManifest2->alphaCurve.alphaCurve = 0;
        functionManifest2->completion = true;

        //Port
        portManifest2 = &componentManifest2->portManifests[0];
        portManifest2->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
        portManifest2->topic = XME_CORE_TOPIC(4098); 
        portManifest2->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;

		// EMPTY COMPONENT MANIFEST
        componentManifest3 = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));
        componentManifest3->componentType = componentType3;
        (void) xme_hal_safeString_strncpy(componentManifest3->name, "testComponent", sizeof(componentManifest3->name));

    }

    virtual ~ManifestRepositoryInterfaceTest()
    {
        xme_core_manifestRepository_fini();
    }

    xme_core_componentType_t componentType0;
    xme_core_componentType_t componentType1;
    xme_core_componentType_t componentType2;
    xme_core_componentType_t componentType3;

    xme_core_componentManifest_t* componentManifest0;
	xme_core_componentManifest_t* componentManifest1;
    xme_core_componentManifest_t* componentManifest2;
    xme_core_componentManifest_t* componentManifest3;
    xme_core_componentManifest_t* outComponentManifest;

    xme_core_functionManifest_t* functionManifest1;
    xme_core_functionManifest_t* functionManifest2;
    
    xme_core_componentPortManifest_t* portManifest1;
    xme_core_componentPortManifest_t* portManifest2;
};

// xme_core_manifestRepository_addComponentManifest()

TEST_F(ManifestRepositoryInterfaceTest, addComponentManifestWithNullComponent)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_manifestRepository_addComponentManifest(NULL, true));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_manifestRepository_addComponentManifest(NULL, false));
}


TEST_F(ManifestRepositoryInterfaceTest, getFunctionAndPortCountWithInvalidComponentType)
{
    outComponentManifest = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_manifestRepository_addComponentManifest(componentManifest0, true));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_manifestRepository_findComponentManifest(componentType0, outComponentManifest));

    ASSERT_EQ(0, xme_core_manifestRepository_getFunctionCount(outComponentManifest));
    ASSERT_EQ(0, xme_core_manifestRepository_getPortCount(outComponentManifest));
}

TEST_F(ManifestRepositoryInterfaceTest, getFunctionAndPortCountWithEmptyComponentManifest)
{
    outComponentManifest = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest3, true));

    ASSERT_EQ(XME_STATUS_SUCCESS, 
        xme_core_manifestRepository_findComponentManifest(componentType3, outComponentManifest));

    ASSERT_EQ(0, xme_core_manifestRepository_getFunctionCount(outComponentManifest));
    ASSERT_EQ(0, xme_core_manifestRepository_getPortCount(outComponentManifest));
}

TEST_F(ManifestRepositoryInterfaceTest, getFunctionAndPortCount)
{
    outComponentManifest = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));

    ASSERT_EQ(XME_STATUS_SUCCESS, 
        xme_core_manifestRepository_findComponentManifest(componentType1, outComponentManifest));

    ASSERT_EQ(1, xme_core_manifestRepository_getFunctionCount(outComponentManifest));
    ASSERT_EQ(1, xme_core_manifestRepository_getPortCount(outComponentManifest));

    //Add two more functions
    componentManifest1->functionManifests[1].functionId = (xme_core_component_functionId_t) 11;
    (void) xme_hal_safeString_strncpy(componentManifest1->functionManifests[1].name, "testFunction1new", sizeof(componentManifest1->functionManifests[1].name));
    componentManifest1->functionManifests[1].wcet = 100000000;
    componentManifest1->functionManifests[1].alphaCurve.alphaCurve = 0;
    componentManifest1->functionManifests[1].completion = true;

    componentManifest1->functionManifests[2].functionId = (xme_core_component_functionId_t) 12;
    (void) xme_hal_safeString_strncpy(componentManifest1->functionManifests[2].name, "testFunction2new", sizeof(componentManifest1->functionManifests[2].name));
    componentManifest1->functionManifests[2].wcet = 100000000;
    componentManifest1->functionManifests[2].alphaCurve.alphaCurve = 0;
    componentManifest1->functionManifests[2].completion = true;

    // Add three more Ports
    componentManifest1->portManifests[1].portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
    componentManifest1->portManifests[1].topic = XME_CORE_TOPIC(4098); 
    componentManifest1->portManifests[1].attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;

    componentManifest1->portManifests[2].portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
    componentManifest1->portManifests[2].topic = XME_CORE_TOPIC(4098); 
    componentManifest1->portManifests[2].attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;

    componentManifest1->portManifests[3].portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
    componentManifest1->portManifests[3].topic = XME_CORE_TOPIC(4098); 
    componentManifest1->portManifests[3].attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));

    ASSERT_EQ(XME_STATUS_SUCCESS, 
        xme_core_manifestRepository_findComponentManifest(componentType1, outComponentManifest));

    ASSERT_EQ(3, xme_core_manifestRepository_getFunctionCount(outComponentManifest));
    ASSERT_EQ(4, xme_core_manifestRepository_getPortCount(outComponentManifest));

    xme_hal_mem_free(outComponentManifest);
}

TEST_F(ManifestRepositoryInterfaceTest, addComponentManifestWithValidComponentAndReplaceToTrue)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest2, true));
}

TEST_F(ManifestRepositoryInterfaceTest, addComponentManifestWithValidComponentAndReplaceToFalse)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, false));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest2, false));
}

TEST_F(ManifestRepositoryInterfaceTest, addComponentManifestByAddingTwoTimesTheSameComponentAndReplaceSetToFalse)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, false));
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, false));
}

TEST_F(ManifestRepositoryInterfaceTest, addComponentManifestByAddingTwoTimesTheSameComponentAndReplaceSetToTrue)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));
}

//// xme_core_manifestRepository_removeComponentManifest
TEST_F(ManifestRepositoryInterfaceTest, removeManifestWithInvalidComponent)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_manifestRepository_removeComponentManifest(XME_CORE_COMPONENT_TYPE_INVALID));
}

TEST_F(ManifestRepositoryInterfaceTest, removeManifestWithValidComponentAndEmptyRepository)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
        xme_core_manifestRepository_removeComponentManifest(componentType1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
        xme_core_manifestRepository_removeComponentManifest(componentType2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
        xme_core_manifestRepository_removeComponentManifest(componentType3));
}

TEST_F(ManifestRepositoryInterfaceTest, removeManifestWithValidComponentAndRepositoryWithOneComponentManifest)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_removeComponentManifest(componentType1));
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
        xme_core_manifestRepository_removeComponentManifest(componentType2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
        xme_core_manifestRepository_removeComponentManifest(componentType3));
}

TEST_F(ManifestRepositoryInterfaceTest, removeManifestWithValidComponentAndRepositoryWithTwoComponentManifests)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest2, true));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_removeComponentManifest(componentType1));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_removeComponentManifest(componentType2));
    ASSERT_EQ(XME_STATUS_NOT_FOUND,
        xme_core_manifestRepository_removeComponentManifest(componentType3));
}

// xme_core_manifestRepository_findComponentManifest()

TEST_F(ManifestRepositoryInterfaceTest, findComponentManifestWithInvalidComponent)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_manifestRepository_findComponentManifest(XME_CORE_COMPONENT_TYPE_INVALID, outComponentManifest));
}

TEST_F(ManifestRepositoryInterfaceTest, findComponentManifestWithNULLOutputParameter)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, 
        xme_core_manifestRepository_findComponentManifest(XME_CORE_COMPONENT_TYPE_INVALID, NULL));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_manifestRepository_findComponentManifest(componentType1, NULL));
}

TEST_F(ManifestRepositoryInterfaceTest, findComponentManifestWithValidParameters)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_manifestRepository_findComponentManifest(componentType1, outComponentManifest));
}

TEST_F(ManifestRepositoryInterfaceTest, findComponentManifestWithValidParametersAndComponentAdded)
{
    outComponentManifest = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));

    ASSERT_EQ(XME_STATUS_SUCCESS, 
        xme_core_manifestRepository_findComponentManifest(componentType1, outComponentManifest));
    //ASSERT_NE(NULL, *outComponentManifest);
    xme_hal_mem_free(outComponentManifest);
}

TEST_F(ManifestRepositoryInterfaceTest, findComponentManifestWithValidParametersAndComponentNotAdded)
{
    outComponentManifest = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));

    ASSERT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_manifestRepository_findComponentManifest(componentType2, outComponentManifest));
    //ASSERT_EQ(NULL, *outComponentManifest);
    ASSERT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_manifestRepository_findComponentManifest(componentType3, outComponentManifest));
    //ASSERT_EQ(NULL, *outComponentManifest);
    xme_hal_mem_free(outComponentManifest);
}

TEST_F(ManifestRepositoryInterfaceTest, findComponentManifestWithValidParametersAndComponentAddedAndLaterRemoved)
{
    outComponentManifest = (xme_core_componentManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_componentManifest_t));

    // add manifest
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_addComponentManifest(componentManifest1, true));

    // test findComponentManifest
    ASSERT_EQ(XME_STATUS_SUCCESS, 
        xme_core_manifestRepository_findComponentManifest(componentType1, outComponentManifest));
    //ASSERT_EQ(NULL, *outComponentManifest);
    ASSERT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_manifestRepository_findComponentManifest(componentType2, outComponentManifest));
    //ASSERT_EQ(NULL, *outComponentManifest);
    ASSERT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_manifestRepository_findComponentManifest(componentType3, outComponentManifest));
    //ASSERT_EQ(NULL, *outComponentManifest);

    // remove manifest
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_manifestRepository_removeComponentManifest(componentType1));

    // test again findComponentManifest
    ASSERT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_manifestRepository_findComponentManifest(componentType1, outComponentManifest));
    //ASSERT_EQ(NULL, *outComponentManifest);
    ASSERT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_manifestRepository_findComponentManifest(componentType2, outComponentManifest));
    //ASSERT_EQ(NULL, *outComponentManifest);
    ASSERT_EQ(XME_STATUS_NOT_FOUND, 
        xme_core_manifestRepository_findComponentManifest(componentType3, outComponentManifest));
    //ASSERT_EQ(NULL, *outComponentManifest);
    xme_hal_mem_free(outComponentManifest);

}

TEST_F(ManifestRepositoryInterfaceTest, iterateOverEmptyManifestRepository)
{
    xme_core_manifestRepository_iterator_t iter;

    EXPECT_NO_XME_ASSERTION_FAILURES(iter = xme_core_manifestRepository_initIterator());
    EXPECT_EQ((xme_core_manifestRepository_iterator_t) XME_HAL_TABLE_INVALID_ROW_HANDLE, iter);

    EXPECT_FALSE(xme_core_manifestRepository_hasNext(iter));

    EXPECT_EQ(NULL, xme_core_manifestRepository_next(&iter));
    EXPECT_EQ((xme_core_manifestRepository_iterator_t) XME_HAL_TABLE_INVALID_ROW_HANDLE, iter);

    EXPECT_NO_XME_ASSERTION_FAILURES(xme_core_manifestRepository_finiIterator(iter));
}

TEST_F(ManifestRepositoryInterfaceTest, iterateOverNonEmptyManifestRepository)
{
    xme_core_componentManifest_t inputManifest = { };
    xme_core_componentManifest_t* manifest;
    xme_core_manifestRepository_iterator_t iter;

    inputManifest.componentType = (xme_core_componentType_t) ((XME_CORE_COMPONENT_TYPE_INVALID) + 1);

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&inputManifest, false));

    EXPECT_NO_XME_ASSERTION_FAILURES(iter = xme_core_manifestRepository_initIterator());
    EXPECT_EQ((xme_core_manifestRepository_iterator_t) XME_HAL_TABLE_INVALID_ROW_HANDLE, iter);

    EXPECT_TRUE(xme_core_manifestRepository_hasNext(iter));

    manifest = xme_core_manifestRepository_next(&iter);
    EXPECT_TRUE(NULL != manifest);
    EXPECT_EQ(0, xme_fallback_memcmp(&inputManifest, manifest, sizeof(inputManifest)));

    EXPECT_FALSE(xme_core_manifestRepository_hasNext(iter));

    EXPECT_EQ(NULL, xme_core_manifestRepository_next(&iter));
    EXPECT_EQ((xme_core_manifestRepository_iterator_t) XME_HAL_TABLE_INVALID_ROW_HANDLE, iter);

    EXPECT_NO_XME_ASSERTION_FAILURES(xme_core_manifestRepository_finiIterator(iter));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
