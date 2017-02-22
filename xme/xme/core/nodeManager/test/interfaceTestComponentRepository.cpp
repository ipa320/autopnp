/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: interfaceTestComponentRepository.cpp 7780 2014-03-12 09:12:38Z wiesmueller $
 */

/**
 * \file
 *         Component repository interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpClientInterface.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpManagerInterface.h"

#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"
#include "xme/core/directory/include/attribute.h"
#include "xme/core/directory/include/topicRegistry.h"
#include "xme/core/log.h"
#include "xme/core/manifestRepository/include/manifestRepository.h"
#include "xme/core/testUtils.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/time.h"

#include <gtest/gtest.h>

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

uint32_t xme_core_nodeMgr_compRep_test_brokerRegisterFunctionCallCount; ///< Counting calls of xme_core_broker_registerFunction() in mocks.c.

uint32_t xme_core_nodeMgr_compRep_test_brokerRemoveFunctionCallCount; ///< Counting calls of xme_core_broker_removeFunction() in mocks.c.

uint32_t xme_core_nodeMgr_compRep_test_dataHandlerCreatePortCallCount; ///< Counting calls of xme_core_dataHandler_createDataPacket() in mocks.c.

static uint32_t xme_core_nodeMgr_compRep_test_componentWrapperReceivePortCallCount; ///< Counting calls of componentWrapperReceivePort().

static uint32_t xme_core_nodeMgr_compRep_test_componentInitCallCount; ///< Counting calls of componentInit().

static xme_status_t componentInitReturnValue; ///< Return status of componentInit().

XME_EXTERN_C_END

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
typedef struct
{
    uint16_t value;
} topic1_t;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
componentWrapperReceivePort
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint8_t componentInternalPortId
)
{
    XME_UNUSED_PARAMETER(dataPacketId);
    XME_UNUSED_PARAMETER(componentInternalPortId);

    xme_core_nodeMgr_compRep_test_componentWrapperReceivePortCallCount++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
componentInit
(
    void* const config,
    const char* initializationString
)
{
    XME_UNUSED_PARAMETER(config);
    XME_UNUSED_PARAMETER(initializationString);

    xme_core_nodeMgr_compRep_test_componentInitCallCount++;

    return componentInitReturnValue;
}

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
/**
 * \brief Initializes component repository and adds some manifests to
 *        the manifest repository.
 *        The component repository is initially empty.
 */
class EmptyComponentRepositoryTest: public xme::testing::Test
{
protected:

    xme_core_componentType_t componentTypeNotExisting;
    xme_core_componentType_t componentType1;

    xme_core_topic_t topic1;

    // constructor
    EmptyComponentRepositoryTest()
    {
        //Do nothing
    }

    virtual ~EmptyComponentRepositoryTest()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_node_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_topicRegistry_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_init());

        componentTypeNotExisting = (xme_core_componentType_t)6999;
        componentType1 = (xme_core_componentType_t)7000;

        topic1 = (xme_core_topic_t)7000;

        registerTopics();
        addComponentType1Manifest();
        resetCallCounts();
    }
    
    virtual void AssertionCheckedTearDown()
    {
        xme_core_nodeMgr_compRep_fini();
        xme_core_manifestRepository_fini();
        xme_core_directory_attribute_fini();
        xme_core_directory_topicRegistry_fini();
        xme_core_node_fini();
        xme_core_dataHandler_fini();
    }

    void resetCallCounts()
    {
        xme_core_nodeMgr_compRep_test_brokerRegisterFunctionCallCount = 0u;
        xme_core_nodeMgr_compRep_test_brokerRemoveFunctionCallCount = 0u;
        xme_core_nodeMgr_compRep_test_componentInitCallCount = 0u;
        xme_core_nodeMgr_compRep_test_dataHandlerCreatePortCallCount = 0u;
        xme_core_nodeMgr_compRep_test_componentWrapperReceivePortCallCount = 0u;
    }

    void registerTopics()
    {
        xme_core_attribute_descriptor_list_t attributeDescriptorList;
        xme_core_attribute_descriptor_t elements[2];

        attributeDescriptorList.element = elements;
        attributeDescriptorList.length = sizeof(elements) / sizeof(elements[0]);
        elements[0].key = (xme_core_attribute_key_t)1234u;
        elements[0].size = sizeof(uint8_t);
        elements[1].key = (xme_core_attribute_key_t)1235u;
        elements[1].size = sizeof(uint8_t);

        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_topicRegistry_registerTopicSize(topic1, sizeof(topic1_t), false));
        ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_directory_attribute_registerAttributeDescriptorList(topic1, &attributeDescriptorList, false));
    }

    void addComponentType1Manifest()
    {
        xme_core_componentManifest_t componentManifest;

        (void)xme_hal_mem_set(&componentManifest, 0u, sizeof(xme_core_componentManifest_t));

        componentManifest.componentInit = NULL;
        componentManifest.componentType = componentType1;
        componentManifest.componentWrapperReceivePort = NULL;
        componentManifest.configStructSize = 0u;
        (void)xme_hal_safeString_strncpy(componentManifest.name, "componentType1", sizeof(componentManifest.name));
        componentManifest.componentWrapperReceivePort = componentWrapperReceivePort;
        
        {
            xme_core_componentPortManifest_t* portManifest = &componentManifest.portManifests[0];

            portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
            portManifest->persistent = false;
            portManifest->queueSize = 11u;
            portManifest->topic = topic1;
            portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
            portManifest->lowerConnectionBound = 0u;
            portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
        }

        {
            xme_core_componentPortManifest_t* portManifest = &componentManifest.portManifests[1];

            portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
            portManifest->persistent = false;
            portManifest->queueSize = 12u;
            portManifest->topic = topic1;
            portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
            portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
            portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
        }

        {
            xme_core_functionManifest_t* functionManifest = &componentManifest.functionManifests[0];

            functionManifest->completion = true;
            functionManifest->functionFini = NULL;
            functionManifest->functionId = (xme_core_component_functionId_t)1;
            functionManifest->functionInit = NULL;
            functionManifest->functionWrapperExecute = NULL;
            (void)xme_hal_safeString_strncpy(functionManifest->name, "function1", sizeof(functionManifest->name));
            functionManifest->optionalPortIndicesLength = 0u;
            functionManifest->requiredPortIndicesLength = 2u;
            functionManifest->requiredPortIndices[0] = 0;
            functionManifest->requiredPortIndices[1] = 1;
            functionManifest->wcet = xme_hal_time_timeIntervalFromMilliseconds(10ull);
        }

        {
            xme_core_functionManifest_t* functionManifest = &componentManifest.functionManifests[1];

            functionManifest->completion = true;
            functionManifest->functionFini = NULL;
            functionManifest->functionId = (xme_core_component_functionId_t)2;
            functionManifest->functionInit = NULL;
            functionManifest->functionWrapperExecute = NULL;
            (void)xme_hal_safeString_strncpy(functionManifest->name, "function2", sizeof(functionManifest->name));
            functionManifest->optionalPortIndicesLength = 1u;
            functionManifest->optionalPortIndices[0] = 0;
            functionManifest->requiredPortIndicesLength = 1u;
            functionManifest->requiredPortIndices[0] = 0;
            functionManifest->wcet = xme_hal_time_timeIntervalFromMilliseconds(20ull);
        }

        xme_core_manifestRepository_addComponentManifest(&componentManifest, false);
    }

};

/**
 * \brief As EmptyComponentRepositoryTest, but it creates some initial
 *        components in the repository.
 */
class PrefilledComponentRepositoryTest: public EmptyComponentRepositoryTest
{
protected:

    xme_core_node_nodeId_t localNodeID;

    xme_core_component_t componentID1;
    xme_core_component_t componentID2;
    xme_core_component_t componentID3;
    xme_core_component_t componentID4;
    xme_core_component_t componentID5;
    xme_core_component_t componentID6;
    xme_core_component_t componentID7;
    xme_core_component_t componentID8;
    xme_core_component_t componentID9;

    xme_core_nodeMgr_compRep_componentHandle_t componentHandle1;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle2;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle3;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle5;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle6;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle7;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle8;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle9;

    xme_core_componentType_t componentType2;
    xme_core_componentType_t componentType3;
    xme_core_componentType_t componentType4;

    // constructor
    PrefilledComponentRepositoryTest()
    {
        //Do nothing
    }

    virtual ~PrefilledComponentRepositoryTest()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        EmptyComponentRepositoryTest::AssertionCheckedSetUp();

        localNodeID = (xme_core_node_nodeId_t)12;
        componentID1 = (xme_core_component_t)1001u;
        componentID2 = (xme_core_component_t)1002u;
        componentID3 = (xme_core_component_t)1003u;
        componentID4 = (xme_core_component_t)1004u;
        componentID5 = (xme_core_component_t)1005u;
        componentID6 = (xme_core_component_t)1006u;
        componentID7 = (xme_core_component_t)1007u;
        componentID8 = (xme_core_component_t)1008u;
        componentID9 = (xme_core_component_t)1009u;

        componentType2 = (xme_core_componentType_t)(componentType1 + 1);
        componentType3 = (xme_core_componentType_t)(componentType2 + 1);
        componentType4 = (xme_core_componentType_t)(componentType3 + 1);

        createComponentTypeManifests();

        {
            xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

            builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
            ASSERT_NE((void*)NULL, builder);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID1);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle1));

            builder = xme_core_nodeMgr_compRep_createBuilder((xme_core_node_nodeId_t)2, componentType1);
            ASSERT_NE((void*)NULL, builder);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID2);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle2));

            builder = xme_core_nodeMgr_compRep_createBuilder(localNodeID, componentType1);
            ASSERT_NE((void*)NULL, builder);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID3);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle3));

            // Build incomplete!
            builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
            ASSERT_NE((void*)NULL, builder);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID4);

            // Announced component
            builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
            ASSERT_NE((void*)NULL, builder);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID5);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle5));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle5));

            // Created component
            builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
            ASSERT_NE((void*)NULL, builder);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID6);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle6));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle6));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_createAndRegisterComponent(componentHandle6));

            // Announced component
            builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType2);
            ASSERT_NE((void*)NULL, builder);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID7);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle7));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle7));

            // Announced component
            builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType3);
            ASSERT_NE((void*)NULL, builder);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID8);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle8));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle8));

            // Announced component
            builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType4);
            ASSERT_NE((void*)NULL, builder);
            xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID9);
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle9));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle9));
        }

        xme_core_node_setCurrentNodeId(localNodeID);

        resetCallCounts();

        componentInitReturnValue = XME_STATUS_SUCCESS;
    }
    
    virtual void AssertionCheckedTearDown()
    {
        EmptyComponentRepositoryTest::AssertionCheckedTearDown();
    }

    void
    createComponentTypeManifests(void)
    {
        xme_core_componentManifest_t componentManifest;
    
        // Component type 2
        //
        // Everything is 0/NULL, except for type
        {
            xme_hal_mem_set(&componentManifest, 0, sizeof(xme_core_componentManifest_t));
            componentManifest.componentType = componentType2;

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }

        // Component type 3
        //
        // Has two functions and two ports
        {
            xme_hal_mem_set(&componentManifest, 0, sizeof(xme_core_componentManifest_t));
            componentManifest.componentType = componentType3;
            xme_hal_safeString_strncpy(componentManifest.name, "ComponentType3", sizeof(componentManifest.name));
            
            componentManifest.componentWrapperReceivePort = componentWrapperReceivePort;

            {
                xme_core_functionManifest_t* funcManifest = &(componentManifest.functionManifests[0]);
                funcManifest->functionId = (xme_core_component_functionId_t)1;
                funcManifest->optionalPortIndicesLength = 0;
                funcManifest->requiredPortIndicesLength = 2;
                funcManifest->requiredPortIndices[0] = 0;
                funcManifest->requiredPortIndices[1] = 1;
                funcManifest->wcet = 1;
            }

            {
                xme_core_functionManifest_t* funcManifest = &(componentManifest.functionManifests[1]);
                funcManifest->functionId = (xme_core_component_functionId_t)1;
                funcManifest->optionalPortIndicesLength = 0;
                funcManifest->requiredPortIndicesLength = 2;
                funcManifest->requiredPortIndices[0] = 0;
                funcManifest->requiredPortIndices[1] = 1;
                funcManifest->wcet = 1;
            }

            {
                xme_core_componentPortManifest_t* portManifest = &(componentManifest.portManifests[0]);
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                portManifest->topic = topic1;
            }

            {
                xme_core_componentPortManifest_t* portManifest = &(componentManifest.portManifests[1]);
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                portManifest->topic = topic1;
            }

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }

        // Component type 4
        //
        // Has two functions and four ports
        {
            xme_hal_mem_set(&componentManifest, 0, sizeof(xme_core_componentManifest_t));
            componentManifest.componentType = componentType4;
            xme_hal_safeString_strncpy(componentManifest.name, "Test3", sizeof(componentManifest.name));

            componentManifest.componentWrapperReceivePort = componentWrapperReceivePort;
            componentManifest.componentInit = componentInit;

            {
                xme_core_functionManifest_t* funcManifest = &(componentManifest.functionManifests[0]);
                funcManifest->functionId = (xme_core_component_functionId_t)1;
                funcManifest->requiredPortIndices[0] = 0;
                funcManifest->requiredPortIndices[1] = 1;
                funcManifest->requiredPortIndicesLength = 2;
                funcManifest->wcet = 42;
            }

            {
                xme_core_functionManifest_t* funcManifest = &(componentManifest.functionManifests[1]);
                funcManifest->functionId = (xme_core_component_functionId_t)2;
                funcManifest->requiredPortIndices[0] = 2;
                funcManifest->requiredPortIndices[1] = 3;
                funcManifest->requiredPortIndicesLength = 2;
                funcManifest->wcet = 79;
            }

            {
                xme_core_componentPortManifest_t* portManifest = &(componentManifest.portManifests[0]);
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                portManifest->topic = topic1;
                portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
            }

            {
                xme_core_componentPortManifest_t* portManifest = &(componentManifest.portManifests[1]);
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                portManifest->topic = topic1;
                portManifest->lowerConnectionBound = 0u;
                portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
            }

            {
                xme_core_componentPortManifest_t* portManifest = &(componentManifest.portManifests[2]);
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                portManifest->topic = topic1;
                portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
            }

            {
                xme_core_componentPortManifest_t* portManifest = &(componentManifest.portManifests[3]);
                portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                portManifest->topic = topic1;
                portManifest->lowerConnectionBound = 0u;
                portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
            }

            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
TEST_F(EmptyComponentRepositoryTest, createBuilder)
{
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

    // SUCCESS: NULL component handle on build
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL)); // NULL component handle is allowed

    // SUCCESS: Check valid component handle and resulting default instance structure
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle)); // NULL component handle is allowed
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, componentHandle);

    // Checking structure
    {
        xme_core_nodeMgr_compRep_componentHandle_t portHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
        xme_core_nodeMgr_compRep_componentHandle_t functionHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
        const xme_core_exec_functionDescriptor_t* functionDescriptor = NULL;

        EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, xme_core_nodeMgr_compRep_getComponentID(componentHandle));
        EXPECT_EQ(componentType1, xme_core_nodeMgr_compRep_getComponentType(componentHandle));
        EXPECT_EQ(componentHandle, xme_core_nodeMgr_compRep_getForeignComponentHandle(componentHandle));
        EXPECT_EQ((char*)NULL, xme_core_nodeMgr_compRep_getInitializationString(componentHandle));
        EXPECT_EQ(XME_CORE_NODE_LOCAL_NODE_ID, xme_core_nodeMgr_compRep_getNodeID(componentHandle));
        EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_STATE_PREPARED, xme_core_nodeMgr_compRep_getState(componentHandle));

        portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, 0u);
        EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, portHandle);
        EXPECT_EQ(11u, xme_core_nodeMgr_compRep_getQueueSize(portHandle));
        EXPECT_EQ(componentHandle, xme_core_nodeMgr_compRep_getComponentInstanceOfPort(portHandle));
        EXPECT_EQ(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, xme_core_nodeMgr_compRep_getDataPacketID(portHandle));
        EXPECT_EQ(componentHandle, xme_core_nodeMgr_compRep_getComponentInstanceOfPort(portHandle));

        portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, 1u);
        EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, portHandle);
        EXPECT_EQ(12u, xme_core_nodeMgr_compRep_getQueueSize(portHandle));
        EXPECT_EQ(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, xme_core_nodeMgr_compRep_getDataPacketID(portHandle));
        EXPECT_EQ(componentHandle, xme_core_nodeMgr_compRep_getComponentInstanceOfPort(portHandle));

        functionHandle = xme_core_nodeMgr_compRep_getFunction(componentHandle, 0u);
        EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, functionHandle);
        EXPECT_EQ(0u, xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle));
        functionDescriptor = xme_core_nodeMgr_compRep_getFunctionDescriptor(functionHandle);
        EXPECT_NE((xme_core_exec_functionDescriptor_t*)NULL, functionDescriptor);
        EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionDescriptor->componentId);
        EXPECT_EQ((void*)NULL, functionDescriptor->fini);
        EXPECT_EQ((xme_core_component_functionId_t)1u, functionDescriptor->functionId);
        EXPECT_EQ((void*)NULL, functionDescriptor->init);
        EXPECT_EQ(xme_hal_time_timeIntervalFromMilliseconds(10ull), functionDescriptor->wcet_ns);
        EXPECT_EQ(componentHandle, xme_core_nodeMgr_compRep_getComponentInstanceOfFunction(functionHandle));

        functionHandle = xme_core_nodeMgr_compRep_getFunction(componentHandle, 1u);
        EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, functionHandle);
        EXPECT_EQ(0u, xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle));
        functionDescriptor = xme_core_nodeMgr_compRep_getFunctionDescriptor(functionHandle);
        EXPECT_NE((xme_core_exec_functionDescriptor_t*)NULL, functionDescriptor);
        EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, functionDescriptor->componentId);
        EXPECT_EQ((void*)NULL, functionDescriptor->fini);
        EXPECT_EQ((xme_core_component_functionId_t)2u, functionDescriptor->functionId);
        EXPECT_EQ((void*)NULL, functionDescriptor->init);
        EXPECT_EQ(xme_hal_time_timeIntervalFromMilliseconds(20ull), functionDescriptor->wcet_ns);
        EXPECT_EQ(componentHandle, xme_core_nodeMgr_compRep_getComponentInstanceOfFunction(functionHandle));
    }

    // ERROR: Two components with same ID
    {
        xme_core_component_t componentID = (xme_core_component_t)1234;

        builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
        EXPECT_NE((void*)NULL, builder);
        xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, NULL));

        builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
        EXPECT_NE((void*)NULL, builder);
        xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID);
        EXPECT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_nodeMgr_compRep_build(builder, NULL));
    }

    // ERROR: Builder NULL
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_nodeMgr_compRep_build(NULL, NULL));

    // ERROR: Unknown manifest
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentTypeNotExisting);
    EXPECT_EQ((void*)NULL, builder);
}

TEST_F(EmptyComponentRepositoryTest, nonBuilderSettersMustNotHaveAnyEffectWhenBuildInProgress)
{
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = (xme_core_nodeMgr_compRep_componentHandle_t)1u;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    ASSERT_NE((void*)NULL, builder);

    // This test is a bit tricky, as we rely on the component instance under construction
    // to have a specific handle value

    xme_core_nodeMgr_compRep_setComponentID(componentHandle, (xme_core_component_t)2222);
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, xme_core_nodeMgr_compRep_getComponentID(componentHandle));

    xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle);
    EXPECT_EQ(-1, xme_core_nodeMgr_compRep_getState(componentHandle));
}

TEST_F(EmptyComponentRepositoryTest, builderSetQueueSize)
{
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_portHandle_t portHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;

    // SUCCESS: Writing both queue sizes to valid values
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 0u, 1u);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 1u, 2u);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, 0u);
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, portHandle);
    EXPECT_EQ(1u, xme_core_nodeMgr_compRep_getQueueSize(portHandle));
    
    portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, 1u);
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, portHandle);
    EXPECT_EQ(2u, xme_core_nodeMgr_compRep_getQueueSize(portHandle));

    // SUCCESS: Writing same queue size twice + default from manifest
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 1u, 12u);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 1u, 255u);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, 0u);
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, portHandle);
    EXPECT_EQ(11u, xme_core_nodeMgr_compRep_getQueueSize(portHandle)); // From manifest
    
    portHandle = xme_core_nodeMgr_compRep_getPort(componentHandle, 1u);
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, portHandle);
    EXPECT_EQ(255u, xme_core_nodeMgr_compRep_getQueueSize(portHandle)); // Last written value

    // ERROR: Unknown portTypeID
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 0u, 1u);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 1u, 2u);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 2u, 3u); // Error!
    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_nodeMgr_compRep_build(builder, NULL));

    // ERROR: Queue size 0
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 0u, 1u);
    xme_core_nodeMgr_compRep_builderSetQueueSize(builder, 1u, 0u); // Error!
    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_nodeMgr_compRep_build(builder, NULL));

    // ERROR: Builder NULL
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_core_nodeMgr_compRep_builderSetQueueSize(NULL, 0u, 1u));
}

TEST_F(EmptyComponentRepositoryTest, builderSetComponentID)
{
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;

    // SUCCESS: Valid component ID
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, (xme_core_component_t)1234);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    EXPECT_EQ((xme_core_component_t)1234, xme_core_nodeMgr_compRep_getComponentID(componentHandle));

    // SUCCESS: Invalid component ID
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, xme_core_nodeMgr_compRep_getComponentID(componentHandle));

    // SUCCESS: Writing twice
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, (xme_core_component_t)1234);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, (xme_core_component_t)5678);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    EXPECT_EQ((xme_core_component_t)5678, xme_core_nodeMgr_compRep_getComponentID(componentHandle));

    // Check if function descriptors also get the component ID
    {
        xme_core_nodeMgr_compRep_componentHandle_t functionHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
        const xme_core_exec_functionDescriptor_t* functionDescriptor = NULL;

        functionHandle = xme_core_nodeMgr_compRep_getFunction(componentHandle, 0u);
        EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, functionHandle);
        functionDescriptor = xme_core_nodeMgr_compRep_getFunctionDescriptor(functionHandle);
        EXPECT_NE((xme_core_exec_functionDescriptor_t*)NULL, functionDescriptor);
        EXPECT_EQ((xme_core_component_t)5678, functionDescriptor->componentId);

        functionHandle = xme_core_nodeMgr_compRep_getFunction(componentHandle, 1u);
        EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, functionHandle);
        functionDescriptor = xme_core_nodeMgr_compRep_getFunctionDescriptor(functionHandle);
        EXPECT_NE((xme_core_exec_functionDescriptor_t*)NULL, functionDescriptor);
        EXPECT_EQ((xme_core_component_t)5678, functionDescriptor->componentId);
    }

    // ERROR: Builder NULL
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_core_nodeMgr_compRep_builderSetComponentID(NULL, (xme_core_component_t)1));
}

TEST_F(EmptyComponentRepositoryTest, builderSetExecutionPeriod)
{
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_functionHandle_t functionHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_hal_time_timeInterval_t timeInterval1 = xme_hal_time_timeIntervalFromMilliseconds(10ull);
    xme_hal_time_timeInterval_t timeInterval2 = xme_hal_time_timeIntervalFromMilliseconds(10ull);

    // SUCCESS: Set execution period
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, 0u, timeInterval1);
    xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, 0u, timeInterval2);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    functionHandle = xme_core_nodeMgr_compRep_getFunction(componentHandle, 0u);
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, functionHandle);
    EXPECT_EQ(timeInterval2, xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle)); // Last written

    functionHandle = xme_core_nodeMgr_compRep_getFunction(componentHandle, 1u);
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, functionHandle);
    EXPECT_EQ(0u, xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle)); // Default

    // SUCCESS: Set execution period to 0
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, 0u, 0u);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    functionHandle = xme_core_nodeMgr_compRep_getFunction(componentHandle, 0u);
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, functionHandle);
    EXPECT_EQ(0u, xme_core_nodeMgr_compRep_getExecutionPeriod(functionHandle)); // Last written

    // ERROR: Unknown functionTypeID
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, 0u, timeInterval1);
    xme_core_nodeMgr_compRep_builderSetExecutionPeriod(builder, 2u, timeInterval1); // Error!
    EXPECT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_nodeMgr_compRep_build(builder, NULL));

    // ERROR: Builder NULL
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_core_nodeMgr_compRep_builderSetExecutionPeriod(NULL, 0u, timeInterval1));
}

TEST_F(EmptyComponentRepositoryTest, builderSetForeignComponentHandle)
{
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_componentHandle_t foreignComponentHandle1 =
        (xme_core_nodeMgr_compRep_componentHandle_t)12;
    xme_core_nodeMgr_compRep_componentHandle_t foreignComponentHandle2 =
        (xme_core_nodeMgr_compRep_componentHandle_t)13;

    // SUCCESS
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(builder, foreignComponentHandle1);
    xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(builder, foreignComponentHandle2);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    EXPECT_EQ(foreignComponentHandle2, xme_core_nodeMgr_compRep_getForeignComponentHandle(componentHandle));

    // SUCCESS: Invalid foreign component handle
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(builder, XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, xme_core_nodeMgr_compRep_getForeignComponentHandle(componentHandle));

    // SUCCESS: Default
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    EXPECT_EQ(componentHandle, xme_core_nodeMgr_compRep_getForeignComponentHandle(componentHandle));
    // By default the foreign handle is the same as the local handle
    // This means that when using @xme_core_pnp_pnpManager_plugInNewComponent()@ the pnpClient will reuse
    // the component entry by default

    // ERROR: Builder NULL
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_core_nodeMgr_compRep_builderSetForeignComponentHandle(NULL, foreignComponentHandle1));
}

TEST_F(EmptyComponentRepositoryTest, builderSetInitializationString)
{
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;

    // SUCCESS: Default
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    EXPECT_EQ((char*)NULL, xme_core_nodeMgr_compRep_getInitializationString(componentHandle));

    // SUCCESS: Some value
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetInitializationString(builder, "zonk");
    xme_core_nodeMgr_compRep_builderSetInitializationString(builder, "initString");
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    EXPECT_STREQ("initString", xme_core_nodeMgr_compRep_getInitializationString(componentHandle));

    // SUCCESS: NULL
    builder = xme_core_nodeMgr_compRep_createBuilder(XME_CORE_NODE_LOCAL_NODE_ID, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetInitializationString(builder, NULL);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle));

    EXPECT_EQ((char*)NULL, xme_core_nodeMgr_compRep_getInitializationString(componentHandle));

    // ERROR: Builder NULL
    ASSERT_NO_XME_ASSERTION_FAILURES(xme_core_nodeMgr_compRep_builderSetInitializationString(NULL, ""));

    // Truncation is also an error, but the maximum allowed length is not well-defined yet
}

TEST_F(EmptyComponentRepositoryTest, alreadyExists)
{
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle1 = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle2 = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle3= XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;

    // SUCCESS
    builder = xme_core_nodeMgr_compRep_createBuilder((xme_core_node_nodeId_t)2, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, (xme_core_component_t)1);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle1));
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, componentHandle1);

    // SUCCESS (same component ID but different node ID as component1)
    builder = xme_core_nodeMgr_compRep_createBuilder((xme_core_node_nodeId_t)3, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, (xme_core_component_t)1);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_build(builder, &componentHandle2));
    EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, componentHandle2);
    EXPECT_NE(componentHandle1, componentHandle2);

    // ALREADY_EXIST (same as component1)
    builder = xme_core_nodeMgr_compRep_createBuilder((xme_core_node_nodeId_t)2, componentType1);
    EXPECT_NE((void*)NULL, builder);
    xme_core_nodeMgr_compRep_builderSetComponentID(builder, (xme_core_component_t)1);
    EXPECT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_nodeMgr_compRep_build(builder, &componentHandle3));
    EXPECT_EQ(componentHandle1, componentHandle3);
}

TEST_F(EmptyComponentRepositoryTest, emptyIterator)
{
    xme_core_nodeMgr_compRep_componentIteratorInit();

    EXPECT_FALSE(xme_core_nodeMgr_compRep_componentIteratorHasNext());
    EXPECT_FALSE(xme_core_nodeMgr_compRep_componentIteratorHasNext());

    xme_core_nodeMgr_compRep_componentIteratorFini();
}


TEST_F(EmptyComponentRepositoryTest, callingGettersOnInvalidHandles)
{
    EXPECT_EQ(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, xme_core_nodeMgr_compRep_getComponentID(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, xme_core_nodeMgr_compRep_getComponentInstanceOfFunction(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, xme_core_nodeMgr_compRep_getComponentInstanceOfPort(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(XME_CORE_COMPONENT_TYPE_INVALID, xme_core_nodeMgr_compRep_getComponentType(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, xme_core_nodeMgr_compRep_getDataPacketID(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(0ull, xme_core_nodeMgr_compRep_getExecutionPeriod(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, xme_core_nodeMgr_compRep_getForeignComponentHandle(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, xme_core_nodeMgr_compRep_getFunction(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, 0u));
    EXPECT_EQ((void*)NULL, xme_core_nodeMgr_compRep_getFunctionDescriptor(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ((void*)NULL, xme_core_nodeMgr_compRep_getInitializationString(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(XME_CORE_NODE_INVALID_NODE_ID, xme_core_nodeMgr_compRep_getNodeID(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, xme_core_nodeMgr_compRep_getPort(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, 0u));
    EXPECT_EQ(0u, xme_core_nodeMgr_compRep_getQueueSize(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
    EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_STATE_INVALID, xme_core_nodeMgr_compRep_getState(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE));
}

TEST_F(PrefilledComponentRepositoryTest, iterator)
{
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;
    bool isVisitedComponent1 = false;
    bool isVisitedComponent2 = false;
    bool isVisitedComponent3 = false;
    bool isVisitedComponent4 = false;
    bool isVisitedComponent5 = false;
    bool isVisitedComponent6 = false;
    bool isVisitedComponent7 = false;
    bool isVisitedComponent8 = false;
    bool isVisitedComponent9 = false;

    xme_core_nodeMgr_compRep_componentIteratorInit();

    while (xme_core_nodeMgr_compRep_componentIteratorHasNext())
    {
        xme_core_nodeMgr_compRep_componentIteratorHasNext();
        xme_core_component_t componentID = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT;

        componentHandle = xme_core_nodeMgr_compRep_componentIteratorNext();
        EXPECT_NE(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, componentHandle);

        componentID = xme_core_nodeMgr_compRep_getComponentID(componentHandle);
        if (componentID1 == componentID) { isVisitedComponent1 = true; }
        else if (componentID2 == componentID) { isVisitedComponent2 = true; }
        else if (componentID3 == componentID) { isVisitedComponent3 = true; }
        else if (componentID4 == componentID) { isVisitedComponent4 = true; }
        else if (componentID5 == componentID) { isVisitedComponent5 = true; }
        else if (componentID6 == componentID) { isVisitedComponent6 = true; }
        else if (componentID7 == componentID) { isVisitedComponent7 = true; }
        else if (componentID8 == componentID) { isVisitedComponent8 = true; }
        else if (componentID9 == componentID) { isVisitedComponent9 = true; }
        else { EXPECT_FALSE(true); }
    }

    EXPECT_TRUE(isVisitedComponent1);
    EXPECT_TRUE(isVisitedComponent2);
    EXPECT_TRUE(isVisitedComponent3);
    EXPECT_FALSE(isVisitedComponent4); // Build not finished, should not appear in iteration!
    EXPECT_TRUE(isVisitedComponent5);
    EXPECT_TRUE(isVisitedComponent6);
    EXPECT_TRUE(isVisitedComponent7);
    EXPECT_TRUE(isVisitedComponent8);
    EXPECT_TRUE(isVisitedComponent9);

    // Repeated next calls at the end of iteration must result in invalid handle
    componentHandle = xme_core_nodeMgr_compRep_componentIteratorNext();
    EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, componentHandle);

    componentHandle = xme_core_nodeMgr_compRep_componentIteratorNext();
    EXPECT_EQ(XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE, componentHandle);

    xme_core_nodeMgr_compRep_componentIteratorFini();
}

TEST_F(PrefilledComponentRepositoryTest, getComponentInstance)
{
    xme_core_nodeMgr_compRep_componentHandle_t componentHandleA;
    xme_core_nodeMgr_compRep_componentHandle_t componentHandleB;

    // ERROR: Invalid parameter
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_nodeMgr_compRep_getComponentInstance(XME_CORE_NODE_LOCAL_NODE_ID, componentID1, NULL));

    // ERROR: Does not exist
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_nodeMgr_compRep_getComponentInstance(XME_CORE_NODE_LOCAL_NODE_ID, componentID2, &componentHandleA));
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_nodeMgr_compRep_getComponentInstance((xme_core_node_nodeId_t)404, componentID3, &componentHandleA));
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_nodeMgr_compRep_getComponentInstance(XME_CORE_NODE_LOCAL_NODE_ID, (xme_core_component_t)404, &componentHandleA));

    // SUCCESS: Also verify if both localNodeID and XME_CORE_NODE_LOCAL_NODE_ID are treated as equal
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_getComponentInstance(XME_CORE_NODE_LOCAL_NODE_ID, componentID1, &componentHandleA));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_getComponentInstance(localNodeID, componentID1, &componentHandleB));
    EXPECT_EQ(componentHandleA, componentHandleB);
    
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_getComponentInstance((xme_core_node_nodeId_t)2, componentID2, &componentHandleA));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_getComponentInstance(XME_CORE_NODE_LOCAL_NODE_ID, componentID3, &componentHandleA));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_getComponentInstance(localNodeID, componentID3, &componentHandleB));
    EXPECT_EQ(componentHandleA, componentHandleB);
}

TEST_F(PrefilledComponentRepositoryTest, destroyComponentInstance)
{
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle = XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE;

    xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle1);
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_nodeMgr_compRep_getComponentInstance(XME_CORE_NODE_LOCAL_NODE_ID, componentID1, &componentHandle));

    xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle2);
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_nodeMgr_compRep_getComponentInstance((xme_core_node_nodeId_t)2, componentID2, &componentHandle));

    xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle3);
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_nodeMgr_compRep_getComponentInstance(localNodeID, componentID3, &componentHandle));
}

TEST_F(PrefilledComponentRepositoryTest, setState)
{
    // SUCCESS: Component was previously in state PREPARED
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle1));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle3));

    // ERROR: Component was previously in state BUILD_IN_PROGRESS
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_nodeMgr_compRep_setStateToAnnounced((xme_core_nodeMgr_compRep_componentHandle_t)4));
    // This is tricky because we rely on component 4 on having a specific handle value, we do this
    // because we did not finish the build and the repository therefore did not return a handle for us

    // ERROR: Component was previously in state ANNOUNCED
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle5));
    
    // ERROR: Component was previously in state CREATED
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_nodeMgr_compRep_setStateToAnnounced(componentHandle6));
}

TEST_F(PrefilledComponentRepositoryTest, createAndRegisterComponent)
{
    // ERROR: State is BUILD_IN_PROGRESS
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_nodeMgr_compRep_createAndRegisterComponent((xme_core_nodeMgr_compRep_componentHandle_t)4));
    // This is tricky because we rely on component 4 on having a specific handle value, we do this
    // because we did not finish the build and the repository therefore did not return a handle for us

    // ERROR: State is CREATED
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_core_nodeMgr_compRep_createAndRegisterComponent(componentHandle6));

    // SUCCESS: State is ANNOUNCED
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_createAndRegisterComponent(componentHandle7));

    EXPECT_EQ(0U, xme_core_nodeMgr_compRep_test_componentInitCallCount);
    EXPECT_EQ(0U, xme_core_nodeMgr_compRep_test_brokerRegisterFunctionCallCount);
    EXPECT_EQ(0U, xme_core_nodeMgr_compRep_test_dataHandlerCreatePortCallCount);
    EXPECT_EQ(0U, xme_core_nodeMgr_compRep_test_componentWrapperReceivePortCallCount);
    EXPECT_EQ(0U, xme_core_nodeMgr_compRep_test_brokerRemoveFunctionCallCount);

    resetCallCounts();

    // SUCCESS: State is ANNOUNCED
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_createAndRegisterComponent(componentHandle8));

    EXPECT_EQ(0U, xme_core_nodeMgr_compRep_test_componentInitCallCount);
    EXPECT_EQ(2U, xme_core_nodeMgr_compRep_test_brokerRegisterFunctionCallCount);
    EXPECT_EQ(2U, xme_core_nodeMgr_compRep_test_dataHandlerCreatePortCallCount);
    EXPECT_EQ(2U, xme_core_nodeMgr_compRep_test_componentWrapperReceivePortCallCount);
    EXPECT_EQ(0U, xme_core_nodeMgr_compRep_test_brokerRemoveFunctionCallCount);

    resetCallCounts();

    // SUCCESS: State is ANNOUNCED
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_createAndRegisterComponent(componentHandle9));

    EXPECT_EQ(1U, xme_core_nodeMgr_compRep_test_componentInitCallCount);
    EXPECT_EQ(2U, xme_core_nodeMgr_compRep_test_brokerRegisterFunctionCallCount);
    EXPECT_EQ(4U, xme_core_nodeMgr_compRep_test_dataHandlerCreatePortCallCount);
    EXPECT_EQ(4U, xme_core_nodeMgr_compRep_test_componentWrapperReceivePortCallCount);
    EXPECT_EQ(0U, xme_core_nodeMgr_compRep_test_brokerRemoveFunctionCallCount);
}

TEST_F(PrefilledComponentRepositoryTest, createAndRegisterComponentErrorRecovery)
{
    // Inject failure
    componentInitReturnValue = XME_STATUS_INTERNAL_ERROR;

    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_nodeMgr_compRep_createAndRegisterComponent(componentHandle9));

    EXPECT_EQ(xme_core_nodeMgr_compRep_test_brokerRegisterFunctionCallCount, xme_core_nodeMgr_compRep_test_brokerRemoveFunctionCallCount);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
