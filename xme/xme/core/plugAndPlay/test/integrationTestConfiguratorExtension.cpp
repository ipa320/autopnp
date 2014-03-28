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
 * $Id: integrationTestConfiguratorExtension.cpp 7751 2014-03-10 16:48:38Z wiesmueller $
 */

/**
 * \file
 *         Configurator Extension integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/plugAndPlay/include/configuratorExtension.h"

#include "xme/core/plugAndPlay/include/plugAndPlayManager.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief Array used by callback functions defined below.
 */
static uint8_t callbackOutput[10];

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Implementation of xme_core_pnp_configExt_configuratorCallback_t that
 *        increments callbackOutput[0].
 */
void
callback0(xme_hal_graph_graph_t* const configGraph)
{
    XME_UNUSED_PARAMETER(configGraph);

    callbackOutput[0]++;
}

/**
 * \brief Implementation of xme_core_pnp_configExt_configuratorCallback_t that
 *        increments callbackOutput[1].
 */
void
callback1(xme_hal_graph_graph_t* const configGraph)
{
    XME_UNUSED_PARAMETER(configGraph);

    callbackOutput[1]++;
}

/**
 * \brief Implementation of xme_core_pnp_configExt_configuratorCallback_t that
 *        calls addComponent.
 */
void
callbackAddComponent(xme_hal_graph_graph_t* const configGraph)
{
    XME_UNUSED_PARAMETER(configGraph);

    xme_core_pnp_configExt_addComponent((xme_core_componentType_t) 1, NULL, (xme_core_node_nodeId_t) 1);
}

/**
 * \brief Implementation of xme_core_pnp_configExt_configuratorCallback_t that
 *        calls removeComponent.
 */
void
callbackRemoveComponent(xme_hal_graph_graph_t* const configGraph)
{
    XME_UNUSED_PARAMETER(configGraph);

    xme_core_pnp_configExt_removeComponent((xme_core_component_t) 1, (xme_core_node_nodeId_t) 1);
}

/**
 * \brief Implementation of xme_core_pnp_configExt_configuratorCallback_t that
 *        calls removeLink.
 */
void
callbackRemoveLink(xme_hal_graph_graph_t* const configGraph)
{
    XME_UNUSED_PARAMETER(configGraph);

    xme_core_pnp_configExt_removeLink((xme_hal_graph_edgeId_t) 1);
}

XME_EXTERN_C_END

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
class ConfigExtIntegrationeTest: public ::testing::Test
{
protected:
    ConfigExtIntegrationeTest()
    {

    }

    virtual ~ConfigExtIntegrationeTest()
    {

    }

    virtual void SetUp()
    {
        uint8_t i = 0;

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_configExt_init());

        for (i = 0; i < (sizeof(callbackOutput) / sizeof(callbackOutput[0])); i++)
        {
            callbackOutput[i] = 0;
        }
    }

    virtual void TearDown()
    {
        xme_core_pnp_configExt_fini();
        xme_core_pnp_pnpManager_fini();
        xme_core_pnp_lrm_fini();
    }

    // Announce some simple test component ports to the LRM.
    void announcePorts()
    {
        ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_pnp_lrm_announcePort
            (
                (xme_core_node_nodeId_t)1,
                (xme_core_component_t)1,
                (xme_core_componentType_t)1,
                XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                0u,
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
                XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID,
                (xme_core_topic_t)1,
                XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
                XME_CORE_INVALID_TRANSACTION_ID
            ));

        ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_pnp_lrm_announcePort
            (
                (xme_core_node_nodeId_t)1,
                (xme_core_component_t)2,
                (xme_core_componentType_t)2,
                XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                0u,
                0u,
                XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED,
                (xme_core_topic_t)1,
                XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET,
                XME_CORE_INVALID_TRANSACTION_ID
            ));
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
//TEST_F(ConfigExtIntegrationeTest, checkRerunFlagAddComponent)
//{
//    xme_core_pnp_lrm_logicalRoutes_t graph;
//
//    xme_hal_graph_init(&graph);
//
//    EXPECT_EQ(XME_STATUS_SUCCESS,
//              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callbackAddComponent, NULL));
//
//    EXPECT_TRUE(xme_core_pnp_configExt_executeConfigurators(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, &graph));
//}

// Same as test checkRemoveComponent, but without the configurator
// Used to verify that the component removed in checkRemoveComponent is even part of the graph.
TEST_F(ConfigExtIntegrationeTest, verifyCheckRemoveComponentWithoutConfigurator)
{
    xme_core_pnp_lrm_logicalRoutes_t graph;
    bool comp1Found = false;

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_hal_graph_init(&graph));

    announcePorts();

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &graph));

    ASSERT_FALSE(
        xme_core_pnp_configExt_executeConfigurators(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, &graph));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &graph));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_hal_graph_initVertexIterator(&graph));
    while (xme_hal_graph_hasNextVertex(&graph))
    {
        xme_hal_graph_vertexId_t vertexId = xme_hal_graph_nextVertex(&graph);
        xme_core_pnp_dataLinkGraph_vertexData_t* vertexData;

        xme_hal_graph_getVertexData(&graph, vertexId, (void**)&vertexData);

        if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vertexData->vertexType &&
            vertexData->vertexData.componentPortVertex.componentId == (xme_core_component_t)1u)
        {
            comp1Found = true;
        }
    }
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_hal_graph_finiVertexIterator(&graph));

    EXPECT_TRUE(comp1Found);
}

TEST_F(ConfigExtIntegrationeTest, checkRemoveComponent)
{
    xme_core_pnp_lrm_logicalRoutes_t graph;

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_hal_graph_init(&graph));

    announcePorts();

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &graph));

    ASSERT_EQ(XME_STATUS_SUCCESS,
            xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callbackRemoveComponent, NULL));

    ASSERT_TRUE(
        xme_core_pnp_configExt_executeConfigurators(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, &graph));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_lrm_getLogicalRoutes(XME_CORE_INVALID_TRANSACTION_ID, &graph));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_hal_graph_initVertexIterator(&graph));
    while (xme_hal_graph_hasNextVertex(&graph))
    {
        xme_hal_graph_vertexId_t vertexId = xme_hal_graph_nextVertex(&graph);
        xme_core_pnp_dataLinkGraph_vertexData_t* vertexData;

        xme_hal_graph_getVertexData(&graph, vertexId, (void**)&vertexData);

        if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vertexData->vertexType)
        {
            EXPECT_NE((xme_core_component_t)1u, vertexData->vertexData.componentPortVertex.componentId); // Assert that the logical route graph does not contain the removed component
        }
    }
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_hal_graph_finiVertexIterator(&graph));
}

//TEST_F(ConfigExtIntegrationeTest, checkRerunFlagRemoveLink)
//{
//    xme_core_pnp_lrm_logicalRoutes_t graph;
//
//    xme_hal_graph_init(&graph);
//
//    EXPECT_EQ(XME_STATUS_SUCCESS,
//              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callbackRemoveLink, NULL));
//
//    EXPECT_FALSE(xme_core_pnp_configExt_executeConfigurators(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, &graph));
//}
//
//TEST_F(ConfigExtIntegrationeTest, checkRerunFlagMixed)
//{
//    xme_core_pnp_lrm_logicalRoutes_t graph;
//
//    xme_hal_graph_init(&graph);
//
//    EXPECT_EQ(XME_STATUS_SUCCESS,
//              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callbackAddComponent, NULL));
//    EXPECT_EQ(XME_STATUS_SUCCESS,
//              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callbackRemoveLink, NULL));
//
//    EXPECT_TRUE(xme_core_pnp_configExt_executeConfigurators(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, &graph));
//}

TEST_F(ConfigExtIntegrationeTest, addAndRunAndRemoveAndRun)
{
    xme_core_pnp_configExt_configuratorHandle_t handle0;
    xme_core_pnp_configExt_configuratorHandle_t handle1;
    xme_core_pnp_lrm_logicalRoutes_t graph;

    xme_hal_graph_init(&graph);

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callback0, &handle0));

    EXPECT_FALSE(xme_core_pnp_configExt_executeConfigurators(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, &graph));

    EXPECT_EQ(1, callbackOutput[0]);
    EXPECT_EQ(0, callbackOutput[1]);

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callback1, &handle1));

    EXPECT_FALSE(xme_core_pnp_configExt_executeConfigurators(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, &graph));

    EXPECT_EQ(2, callbackOutput[0]);
    EXPECT_EQ(1, callbackOutput[1]);

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_pnp_configExt_removeConfigurator(handle0));

    EXPECT_FALSE(xme_core_pnp_configExt_executeConfigurators(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, &graph));

    EXPECT_EQ(2, callbackOutput[0]);
    EXPECT_EQ(2, callbackOutput[1]);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
