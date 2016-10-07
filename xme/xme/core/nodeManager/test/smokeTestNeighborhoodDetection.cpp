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
 * $Id: smokeTestNeighborhoodDetection.cpp 4995 2013-09-05 18:43:44Z ruiz $
 */

/**
 * \file
 *         Neighborhood Detection smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/nodeManager/include/neighborhoodDetection.h"
#include "xme/hal/include/mem.h"
#include "xme/core/directory/include/nodeRegistryController.h"
#include "xme/hal/include/net.h"
#include "xme/hal/include/sync.h"


/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
class NeighborhoodDetectionSmokeTest: public ::testing::Test
{
protected:

    // constructor
    NeighborhoodDetectionSmokeTest()
    {
        //Do nothing
    }

    virtual ~NeighborhoodDetectionSmokeTest()
    {
    }

    // SetUp before the first test case
    virtual void SetUpTestCase()
    {

    }

    virtual void TearDownTestCase()
    {
        xme_core_nodeManager_neighborhoodDetection_fini();
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
TEST(NeighborhoodDetectionSmokeTest, InitFunctionWithNULLParameter)
{
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_nodeManager_neighborhoodDetection_init(NULL));
    xme_core_nodeManager_neighborhoodDetection_fini();
}

TEST(NeighborhoodDetectionSmokeTest, InitFunctionWithInvalidNodeParameter)
{
    xme_core_node_nodeId_t node = (xme_core_node_nodeId_t) 3;
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_nodeManager_neighborhoodDetection_init(&node));
    xme_core_nodeManager_neighborhoodDetection_fini();
}

TEST(NeighborhoodDetectionSmokeTest, InitFunctionWithCorrectNodeParameter)
{
    xme_core_node_nodeId_t node = (xme_core_node_nodeId_t) 1;
    xme_com_interface_address_t nodeIntf1;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
    // Register the nodes
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(node, "first node", (xme_core_node_guid_t)1));
    // Add network interfaces
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("192.168.2.1:32211", &nodeIntf1));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(node, nodeIntf1));
    //Initialize the network abstraction
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_sync_init());
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_init());
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeManager_neighborhoodDetection_init(&node));
    xme_core_nodeManager_neighborhoodDetection_fini();
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
