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
 * $Id: integrationTestPnPManager.cpp 7683 2014-03-05 14:57:53Z ruiz $
 */

/**
 * \file
 *         Plug and Play Manager integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/plugAndPlay/include/plugAndPlayManager.h"

#include "xme/defines.h"

#include "xme/core/node.h"
#include "xme/core/testUtils.h"
#include "xme/core/topic.h"
#include "xme/core/manifestRepository/include/manifestRepository.h"
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpManagerInterface.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"
#include "xme/core/plugAndPlay/include/networkConfigurationCalculator.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/random.h"
#include "xme/hal/include/time.h"

#include "xme/core/topicData.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

#define COMPONENTTYPE1 (XME_CORE_COMPONENT_TYPE_USER + 1U)
#define COMPONENTTYPE2 (XME_CORE_COMPONENT_TYPE_USER + 2U)
#define COMPONENTTYPE3 (XME_CORE_COMPONENT_TYPE_USER + 3U)
#define COMPONENTTYPE4 (XME_CORE_COMPONENT_TYPE_USER + 4U)

#define NODE2 (xme_core_node_nodeId_t) 2U
#define NODE3 (xme_core_node_nodeId_t) 3U


/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class PnPManagerIntegrationTest: public xme::testing::Test
{
protected:
    PnPManagerIntegrationTest()
    : invalidComponentId(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT)
    , componentId1((xme_core_component_t)1)
    , componentId2((xme_core_component_t)2)
    , componentId3((xme_core_component_t)3)
    , componentId4((xme_core_component_t)4)
    , componentId5((xme_core_component_t)5)
    , invalidNodeId(XME_CORE_NODE_INVALID_NODE_ID)
    , localNodeId((xme_core_node_nodeId_t)1)
    , nodeId2((xme_core_node_nodeId_t)2)
    , nodeId3((xme_core_node_nodeId_t)3)
    , invalidComponentType(XME_CORE_COMPONENT_TYPE_INVALID)
    , componentTypeSensor(XME_CORE_COMPONENT_TYPE_SENSOR)
    , componentTypeMonitor(XME_CORE_COMPONENT_TYPE_MONITOR)
    {
    }

    virtual ~PnPManagerIntegrationTest()
    {
    }

    virtual void AssertionCheckedSetUp()
    { 
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_init());

        guid1 = (xme_core_node_guid_t) xme_hal_random_rand();
        guid2 = (xme_core_node_guid_t) (((uint64_t)guid1) - 1);
        guid3 = (xme_core_node_guid_t) (((uint64_t)guid1) + 1);

        // Register the nodes
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(localNodeId, "localNodeId", guid1));
        {
            xme_com_interface_address_t nodeInterface;
            
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33221", &nodeInterface));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(localNodeId, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(localNodeId));


        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId2, "nodeId2", guid2));
        {
            xme_com_interface_address_t nodeInterface;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33222", &nodeInterface));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId2, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId2));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId3, "nodeId3", guid3));
        {
            xme_com_interface_address_t nodeInterface;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33223", &nodeInterface));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId3, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId3));

        outGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

        createComponentTypeManifests();
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_nodeMgr_compRep_fini();
        xme_core_pnp_pnpManager_fini();
        xme_core_manifestRepository_fini();
        xme_core_pnp_lrm_fini();
        xme_core_directory_nodeRegistryController_fini();
    }

    void
    createComponentTypeManifests(void)
    {
        xme_core_componentManifest_t componentManifest;
    
        // Create and add component type manifest for component 'sensor' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, XME_CORE_COMPONENT_TYPE_SENSOR));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    
        // Create and add component type manifest for component 'monitor' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, XME_CORE_COMPONENT_TYPE_MONITOR));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    }

    xme_status_t
    createComponentTypeManifest
    (
        xme_core_componentManifest_t* componentManifest,
        xme_core_componentType_t componentType
    )
    {
        uint32_t functionArrayLength;
        uint32_t portArrayLength;

        XME_CHECK
        (
            NULL != componentManifest,
            XME_STATUS_INVALID_PARAMETER
        );

        // Initialize the structure with all zeros
        xme_hal_mem_set(componentManifest, 0U, sizeof(xme_core_componentManifest_t));

        componentManifest->componentType = componentType;

        functionArrayLength = sizeof(componentManifest->functionManifests) / sizeof(componentManifest->functionManifests[0]);

        // Function 'printSensorValue'
        {
            if (0 >= functionArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0 == functionArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component defines more functions (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        1,
                        functionArrayLength
                    );
                }
            }
            else
            {
                xme_core_functionManifest_t* functionManifest;
            
                functionManifest = &componentManifest->functionManifests[0];
                functionManifest->functionId = (xme_core_component_functionId_t)1;
                functionManifest->wcet = xme_hal_time_timeIntervalFromMilliseconds(200);
                functionManifest->alphaCurve.alphaCurve = 0;
                functionManifest->completion = true;
            }

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT; i ++)
                {
                    componentManifest->functionManifests[i].functionId = (xme_core_component_functionId_t)0;
                    componentManifest->functionManifests[i].wcet = xme_hal_time_timeIntervalFromMilliseconds(0);
                    componentManifest->functionManifests[i].alphaCurve.alphaCurve = 0;
                    componentManifest->functionManifests[i].completion = true;
                }
            }
        }
    
        portArrayLength = sizeof(componentManifest->portManifests) / sizeof(componentManifest->portManifests[0]);

        // Subscription 'sensorValueIn'
        {
            if (0 >= portArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0 == portArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component defines more ports (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        1,
                        portArrayLength
                    );
                }
            }
            else
            {
                xme_core_componentPortManifest_t* portManifest;
            
                portManifest = &componentManifest->portManifests[0];
                if (componentType == XME_CORE_COMPONENT_TYPE_MONITOR) // monitor
                {
                    portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                    portManifest->lowerConnectionBound = 0;
                    portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                } 
                else
                {
                    portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                    portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                }
                portManifest->topic = XME_CORE_TOPIC(4098); 
                portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                portManifest->queueSize = 1u;
            }

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT; i ++)
                {
                    componentManifest->portManifests[i].portType = XME_CORE_COMPONENT_PORTTYPE_INVALID;
                    componentManifest->portManifests[i].topic = XME_CORE_TOPIC_INVALID_TOPIC;
                    componentManifest->portManifests[i].lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    componentManifest->portManifests[i].upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    componentManifest->portManifests[i].attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                }
            }
        }
    
        return XME_STATUS_SUCCESS;
    }

    xme_core_component_t invalidComponentId;
    xme_core_component_t componentId1;
    xme_core_component_t componentId2;
    xme_core_component_t componentId3;
    xme_core_component_t componentId4;
    xme_core_component_t componentId5;

    xme_core_node_nodeId_t invalidNodeId;
    xme_core_node_nodeId_t localNodeId;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeId3;

    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;
    xme_core_node_guid_t guid3;

    xme_core_componentType_t invalidComponentType;
    xme_core_componentType_t componentTypeSensor;
    xme_core_componentType_t componentTypeMonitor;

    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph;
};

class PnPManagerLoginIntegrationTest: public xme::testing::Test
{
    protected:
    // constructor
    PnPManagerLoginIntegrationTest()
        :nodeId1((xme_core_node_nodeId_t) 1)
        ,nodeId2((xme_core_node_nodeId_t) 2)
        ,nodeIdInvalid(XME_CORE_NODE_INVALID_NODE_ID)
        ,guid0(0)
        ,guid1(512)
        ,guid2(1024)
    {
    }

    virtual ~PnPManagerLoginIntegrationTest()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
//        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createSensorComponentTypeManifest());
//        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createMonitorComponentTypeManifest());

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_init());


        pnpLoginResponse1 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));
        pnpLoginResponse2 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));
        pnpLoginResponse3 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));
        pnpLoginResponse4 = (xme_core_topic_login_pnpLoginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_pnpLoginResponse_t));

        pnpLoginResponse1->nodeId = nodeId1;
        pnpLoginResponse2->nodeId = nodeId1;
        pnpLoginResponse3->nodeId = nodeId2;
        pnpLoginResponse4->nodeId = XME_CORE_NODE_INVALID_NODE_ID;

        ip = "192.168.0.1:33331";
        port = 33331u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse1->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse1->portAddress, &portNetworkByteOrder, 2);

        ip = "192.168.0.2:33332";
        port = 33332u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse2->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse2->portAddress, &portNetworkByteOrder, 2);

        ip = "192.168.0.3:33333";
        port = 33333u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse3->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse3->portAddress, &portNetworkByteOrder, 2);

        ip = "192.168.0.4:33334";
        port = 33334u;
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipNetworkByteOrder, &portNetworkByteOrder));
        xme_hal_mem_copy(&pnpLoginResponse4->ipAddress, &ipNetworkByteOrder, 4);
        xme_hal_mem_copy(&pnpLoginResponse4->portAddress, &portNetworkByteOrder, 2);

        loginResponse = (xme_core_topic_login_loginResponse_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_login_loginResponse_t));
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_nodeMgr_compRep_fini();
        xme_core_pnp_pnpManager_fini();
        xme_core_manifestRepository_fini();
        xme_core_pnp_lrm_fini();
        xme_core_directory_nodeRegistryController_fini();
    }

    xme_core_node_nodeId_t nodeId1;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeIdInvalid;

    xme_core_node_guid_t guid0;
    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;

    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse1;
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse2;
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse3;
    xme_core_topic_login_pnpLoginResponse_t* pnpLoginResponse4;

    xme_core_topic_login_loginResponse_t* loginResponse;
    
    const char* ip;
    uint16_t port;
    uint32_t ipNetworkByteOrder;
    uint16_t portNetworkByteOrder;
    xme_com_interface_address_t interfaceAddress;
};

class PnPManagerIntegrationTestComponentInstanceManifestTests: public xme::testing::Test
{
protected:
    PnPManagerIntegrationTestComponentInstanceManifestTests()
    {
    }

    virtual ~PnPManagerIntegrationTestComponentInstanceManifestTests()
    {
    }

    virtual void AssertionCheckedSetUp()
    {
        xme_core_pnp_pnpManager_init(NULL);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
//        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createSensorComponentTypeManifest());
//        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_createMonitorComponentTypeManifest());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_init());

        componentInstanceManifest1 = (xme_core_topic_pnp_componentInstanceManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnp_componentInstanceManifest_t));
        componentInstanceManifest2 = (xme_core_topic_pnp_componentInstanceManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnp_componentInstanceManifest_t));
        componentInstanceManifest3 = (xme_core_topic_pnp_componentInstanceManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnp_componentInstanceManifest_t));
        componentInstanceManifest4 = (xme_core_topic_pnp_componentInstanceManifest_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnp_componentInstanceManifest_t));

        // Component Instance Manifest 1 (invalid node)
        componentInstanceManifest1->nodeId = XME_CORE_NODE_INVALID_NODE_ID;

        // Component Instance Manifest 2 (only node established)
        componentInstanceManifest2->nodeId = (xme_core_node_nodeId_t) 2;

        // Component Instance Manifest 3 (with one component)
        componentInstanceManifest3->nodeId = (xme_core_node_nodeId_t) 3;
        componentInstanceManifest3->components[0].componentId = (xme_core_component_t) 1;
        componentInstanceManifest3->components[0].componentType = XME_CORE_COMPONENT_TYPE_SENSOR;
        
        // Component Instance Manifest 4 (with two components)
        componentInstanceManifest4->nodeId = (xme_core_node_nodeId_t) 4;
        componentInstanceManifest4->components[0].componentId = (xme_core_component_t) 1;
        componentInstanceManifest4->components[0].componentType = XME_CORE_COMPONENT_TYPE_SENSOR;
        componentInstanceManifest4->components[1].componentId = (xme_core_component_t) 2;
        componentInstanceManifest4->components[1].componentType = XME_CORE_COMPONENT_TYPE_MONITOR;

        createComponentTypeManifests();

        invalidInterfaceAddress.addressType = XME_COM_INTERFACE_ADDRESS_TYPE_INVALID;

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33221", &validInterfaceAddress));
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_nodeMgr_compRep_fini();
        xme_core_pnp_pnpManager_fini();
        xme_core_manifestRepository_fini();
        xme_core_pnp_lrm_fini();
        xme_core_directory_nodeRegistryController_fini();

        xme_hal_mem_free(componentInstanceManifest1);
        xme_hal_mem_free(componentInstanceManifest2);
        xme_hal_mem_free(componentInstanceManifest3);
        xme_hal_mem_free(componentInstanceManifest4);
    }

    void
    createComponentTypeManifests(void)
    {
        xme_core_componentManifest_t componentManifest;
    
        // Create and add component type manifest for component 'sensor' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, XME_CORE_COMPONENT_TYPE_SENSOR));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    
        // Create and add component type manifest for component 'monitor' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, XME_CORE_COMPONENT_TYPE_MONITOR));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    }

    xme_status_t
    createComponentTypeManifest
    (
        xme_core_componentManifest_t* componentManifest,
        xme_core_componentType_t componentType
    )
    {
        uint32_t functionArrayLength;
        uint32_t portArrayLength;
    
        XME_CHECK
        (
            NULL != componentManifest,
            XME_STATUS_INVALID_PARAMETER
        );
    
        // Initialize the structure with all zeros
        xme_hal_mem_set(componentManifest, 0U, sizeof(xme_core_componentManifest_t));

        componentManifest->componentType = componentType;

        functionArrayLength = sizeof(componentManifest->functionManifests) / sizeof(componentManifest->functionManifests[0]);

        // Function 'printSensorValue'
        {
            if (0 >= functionArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0 == functionArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component defines more functions (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        1,
                        functionArrayLength
                    );
                }
            }
            else
            {
                xme_core_functionManifest_t* functionManifest;
            
                functionManifest = &componentManifest->functionManifests[0];
                functionManifest->functionId = (xme_core_component_functionId_t)1;
                functionManifest->wcet = xme_hal_time_timeIntervalFromMilliseconds(200);
                functionManifest->alphaCurve.alphaCurve = 0;
                functionManifest->completion = true;
            }

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT; i ++)
                {
                    componentManifest->functionManifests[i].functionId = (xme_core_component_functionId_t)0;
                    componentManifest->functionManifests[i].wcet = xme_hal_time_timeIntervalFromMilliseconds(0);
                    componentManifest->functionManifests[i].alphaCurve.alphaCurve = 0;
                    componentManifest->functionManifests[i].completion = true;
                }
            }
        }
    
        portArrayLength = sizeof(componentManifest->portManifests) / sizeof(componentManifest->portManifests[0]);

        // Subscription 'sensorValueIn'
        {
            if (0 >= portArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0 == portArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component defines more ports (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        1,
                        portArrayLength
                    );
                }
            }
            else
            {
                xme_core_componentPortManifest_t* portManifest;
            
                portManifest = &componentManifest->portManifests[0];
                if (componentType == XME_CORE_COMPONENT_TYPE_MONITOR) // monitor
                {
                    portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                    portManifest->lowerConnectionBound = 0;
                    portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                } 
                else
                {
                    portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                    portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                }
                portManifest->topic = XME_CORE_TOPIC(4098); 
                portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                portManifest->queueSize = 1u;
            }

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT; i ++)
                {
                    componentManifest->portManifests[i].portType = XME_CORE_COMPONENT_PORTTYPE_INVALID;
                    componentManifest->portManifests[i].topic = XME_CORE_TOPIC_INVALID_TOPIC;
                    componentManifest->portManifests[i].attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                }
            }
        }
    
        return XME_STATUS_SUCCESS;
    }



    xme_core_topic_pnp_componentInstanceManifest_t* componentInstanceManifest1;
    xme_core_topic_pnp_componentInstanceManifest_t* componentInstanceManifest2;
    xme_core_topic_pnp_componentInstanceManifest_t* componentInstanceManifest3;
    xme_core_topic_pnp_componentInstanceManifest_t* componentInstanceManifest4;

    xme_com_interface_address_t invalidInterfaceAddress;
    xme_com_interface_address_t validInterfaceAddress;
};

class PnPManagerPlugOutIntegrationTest: public xme::testing::Test
{
    // 
    // Initial Configuration
    // --------------------------------------------------------------------------------------------------------
    // | Node | ComponentID | PortType   | ComponentType | Topic    | TargetComponentIDs | SourceComponentIDs |
    // --------------------------------------------------------------------------------------------------------
    // |    1 |           1 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C2 |                    |
    // |    1 |           2 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C2 |
    // --------------------------------------------------------------------------------------------------------
    // |    2 |           1 | PUBLISHER  |   CT_USER + 3 | USER + 2 |               N3C1 |                    |
    // |    2 |           2 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C2 |
    // --------------------------------------------------------------------------------------------------------
    // |    3 |           1 | SUBSCRIBER |   CT_USER + 4 | USER + 2 |                    |               N2C1 |
    // |    3 |           2 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C2 |                    |
    // --------------------------------------------------------------------------------------------------------
    // 
    // Operations: 
    // - Remove component N3C1 (Subscriber of topic 2). 
    //
    // --------------------------------------------------------------------------------------------------------
    // | Node | ComponentID | PortType   | ComponentType | Topic    | TargetComponentIDs | SourceComponentIDs |
    // --------------------------------------------------------------------------------------------------------
    // |    1 |           1 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C2 |                    |
    // |    1 |           2 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C2 |
    // --------------------------------------------------------------------------------------------------------
    // |    2 |           1 | PUBLISHER  |   CT_USER + 3 | USER + 2 |               N3C1 |                    | <-- remove graph
    // |    2 |           2 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C2 |
    // --------------------------------------------------------------------------------------------------------
    // |  *3* |         *1* | SUBSCRIBER |   CT_USER + 4 | USER + 2 |                    |               N2C1 | <-- remove graph
    // |    3 |           2 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C2 |                    |
    // --------------------------------------------------------------------------------------------------------
    //
    // - Remove component N3C2 (Publisher of topic 1). 
    //
    // --------------------------------------------------------------------------------------------------------
    // | Node | ComponentID | PortType   | ComponentType | Topic    | TargetComponentIDs | SourceComponentIDs |
    // --------------------------------------------------------------------------------------------------------
    // |    1 |           1 | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C2 |                    |
    // |    1 |           2 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C2 | <-- remove graph (1 link to N3C2)
    // --------------------------------------------------------------------------------------------------------
    // |    - |           - |      -     |             - |        - |                  - |                  - |
    // |    2 |           2 | SUBSCRIBER |   CT_USER + 2 | USER + 1 |                    |         N1C1, N3C2 | <-- remove graph (1 link to N3C2)
    // --------------------------------------------------------------------------------------------------------
    // |    - |           - |      -     |             - |        - |                  - |                  - |
    // |  *3* |         *2* | PUBLISHER  |   CT_USER + 1 | USER + 1 |         N1C2, N2C2 |                    | <-- remove graph (2 links)
    // --------------------------------------------------------------------------------------------------------
    
protected:
    PnPManagerPlugOutIntegrationTest()
    {
        // Declaration of three nodes. 
        invalidNodeId = (xme_core_node_nodeId_t) XME_CORE_NODE_INVALID_NODE_ID;
        localNodeId = (xme_core_node_nodeId_t) XME_CORE_NODE_LOCAL_NODE_ID;
        nodeId2 = (xme_core_node_nodeId_t) NODE2;
        nodeId3 = (xme_core_node_nodeId_t) NODE3;

        // Declaration of the two user topics. 
        userTopic1 = XME_CORE_TOPIC((uint16_t)XME_CORE_TOPIC_USER + 1U);
        userTopic2 = XME_CORE_TOPIC((uint16_t)XME_CORE_TOPIC_USER + 2U);

        // Declaration of component IDs
        componentId1 = (xme_core_component_t) 1U;
        componentId2 = (xme_core_component_t) 2U;

        // Declaration of component types (one subscriber and publisher per topic). 
        invalidComponentType = (xme_core_componentType_t) XME_CORE_COMPONENT_TYPE_INVALID;
        componentTypePublisherTopic1 = (xme_core_componentType_t) COMPONENTTYPE1;
        componentTypeSubscriberTopic1 = (xme_core_componentType_t) COMPONENTTYPE2;
        componentTypePublisherTopic2 = (xme_core_componentType_t) COMPONENTTYPE3;
        componentTypeSubscriberTopic2 = (xme_core_componentType_t) COMPONENTTYPE4;

        // Declaration of components
        invalidComponentId= (xme_core_component_t) XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT;

        functionIdentifier = 1U;
    }

    virtual ~PnPManagerPlugOutIntegrationTest()
    {
    }

    virtual void AssertionCheckedSetUp()
    { 
        // Init all needed components: manifest repository, logical route manager, plug and play manager,
        // node registry controller and component repository. 
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_lrm_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_init());
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_nodeMgr_compRep_init());

        // Generate three different generated IDs for register nodes. 
        guid1 = (xme_core_node_guid_t) xme_hal_random_rand();
        guid2 = (xme_core_node_guid_t) (((uint64_t)guid1) - 1);
        guid3 = (xme_core_node_guid_t) (((uint64_t)guid1) + 1);

        // Register the nodes
        // Node 1 (Local Node)
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(localNodeId, "localNodeId", guid1));
        {
            xme_com_interface_address_t nodeInterface;
            
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33221", &nodeInterface));
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(localNodeId, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(localNodeId));

        // Node 2
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId2, "nodeId2", guid2));
        {
            xme_com_interface_address_t nodeInterface;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33222", &nodeInterface));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId2, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId2));

        // Node 3
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode(nodeId3, "nodeId3", guid3));
        {
            xme_com_interface_address_t nodeInterface;
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress("127.0.0.1:33223", &nodeInterface));

            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_addInterface(nodeId3, nodeInterface));
        }
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode(nodeId3));

        // Reserve memory for the runtime graph. 
        outGraph = (xme_core_topic_pnpManager_runtime_graph_model_t*) xme_hal_mem_alloc(sizeof(xme_core_topic_pnpManager_runtime_graph_model_t));

        // Create component manifests. 
        createComponentTypeManifests();

        // Announce manifests. 
        announceManifests();

        // Consume announcements runtime graphs. 
        consumeRuntimeGraphs();

        // Establish all the routes. 
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished());
    }

    virtual void AssertionCheckedTearDown()
    {
        xme_core_nodeMgr_compRep_fini();
        xme_core_pnp_pnpManager_fini();
        xme_core_manifestRepository_fini();
        xme_core_pnp_lrm_fini();
        xme_core_directory_nodeRegistryController_fini();
    }

    void
    createComponentTypeManifests(void)
    {
        xme_core_componentManifest_t componentManifest;
    
        // Create and add component type manifest for component 'publisher of topic 1' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, componentTypePublisherTopic1));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    
        // Create and add component type manifest for component 'subscriber of topic 1' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, componentTypeSubscriberTopic1));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }

        // Create and add component type manifest for component 'publisher of topic 2' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, componentTypePublisherTopic2));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    
        // Create and add component type manifest for component 'subscriber of topic 2' to manifest repository
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, createComponentTypeManifest(&componentManifest, componentTypeSubscriberTopic2));
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_addComponentManifest(&componentManifest, false));
        }
    }

    void
    announceManifests(void)
    {
        EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
        EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
        EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
        EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

        // Announce components in local node.
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(localNodeId, componentId1, componentTypePublisherTopic1));
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(localNodeId, componentId2, componentTypeSubscriberTopic1));

        // Announce components in node 2.
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(nodeId2, componentId1, componentTypePublisherTopic2));
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(nodeId2, componentId2, componentTypeSubscriberTopic1));

        // Announce components in node 3. 
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(nodeId3, componentId1, componentTypeSubscriberTopic2));
        EXPECT_EQ(XME_STATUS_SUCCESS, announceNewComponentInPnPManager(nodeId3, componentId2, componentTypePublisherTopic1));

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateConfiguration());
    }

    xme_status_t
    announceNewComponentInPnPManager
    (
        xme_core_node_nodeId_t nodeID,
        xme_core_component_t componentID,
        xme_core_componentType_t componentType
    )
    {
        xme_status_t status = XME_STATUS_INTERNAL_ERROR;
        xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;
        xme_core_nodeMgr_compRep_componentHandle_t componentHandle;

        XME_CHECK(XME_CORE_COMPONENT_TYPE_INVALID != componentType, XME_STATUS_INVALID_PARAMETER);
        XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeID, XME_STATUS_INVALID_PARAMETER);

        builder = xme_core_nodeMgr_compRep_createBuilder(nodeID, componentType);
        XME_CHECK(NULL != builder, XME_STATUS_INTERNAL_ERROR);
        xme_core_nodeMgr_compRep_builderSetComponentID(builder, componentID);
    
        status = xme_core_nodeMgr_compRep_build(builder, &componentHandle);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        return xme_core_pnp_pnpManager_announceNewComponentOnNode(componentHandle);
    }

    void
    consumeRuntimeGraphs(void)
    {
        uint8_t i = 0U;

        // Consume all runtime graphs. 
        while (xme_core_pnp_pnpManager_hasNewRuntimeGraphs())
        {
            ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraph(outGraph));
            i++;
        }

        EXPECT_EQ(3U, i);
    }

    xme_status_t
    createComponentTypeManifest
    (
        xme_core_componentManifest_t* componentManifest,
        xme_core_componentType_t componentType
    )
    {
        uint32_t functionArrayLength;
        uint32_t portArrayLength;

        XME_CHECK
        (
            NULL != componentManifest,
            XME_STATUS_INVALID_PARAMETER
        );

        // Initialize the structure with all zeros
        xme_hal_mem_set(componentManifest, 0U, sizeof(xme_core_componentManifest_t));

        componentManifest->componentType = componentType;

        functionArrayLength = sizeof(componentManifest->functionManifests) / sizeof(componentManifest->functionManifests[0]);

        // Function 'printSensorValue'
        {
            if (0 >= functionArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0 == functionArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component defines more functions (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        1,
                        functionArrayLength
                    );
                }
            }
            else
            {
                xme_core_functionManifest_t* functionManifest;
            
                functionManifest = &componentManifest->functionManifests[0];
                functionManifest->functionId = (xme_core_component_functionId_t) functionIdentifier++;
                functionManifest->wcet = xme_hal_time_timeIntervalFromMilliseconds(200);
                functionManifest->alphaCurve.alphaCurve = 0;
                functionManifest->completion = true;
            }

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT; i ++)
                {
                    componentManifest->functionManifests[i].functionId = (xme_core_component_functionId_t)0;
                    componentManifest->functionManifests[i].wcet = xme_hal_time_timeIntervalFromMilliseconds(0);
                    componentManifest->functionManifests[i].alphaCurve.alphaCurve = 0;
                    componentManifest->functionManifests[i].completion = true;
                }
            }
        }
    
        portArrayLength = sizeof(componentManifest->portManifests) / sizeof(componentManifest->portManifests[0]);

        // Subscription 'sensorValueIn'
        {
            if (0 >= portArrayLength) // Check generated by tool (which does not know about the array size)
            {
                if (0 == portArrayLength) // Only trigger warning once
                {
                    XME_LOG
                    (
                        XME_LOG_WARNING,
                        "%s:%d Component defines more ports (%d) than can be stored in the manifest data structure (%d).\n",
                        __FILE__,
                        __LINE__,
                        1,
                        portArrayLength
                    );
                }
            }
            else
            {
                xme_core_componentPortManifest_t* portManifest;
            
                portManifest = &componentManifest->portManifests[0];
                portManifest->queueSize = 1u;
                switch(componentType)
                {
                    case (xme_core_componentType_t) COMPONENTTYPE1:
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                        portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                        portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                        portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                        portManifest->topic = userTopic1;
                        break;
                    case (xme_core_componentType_t) COMPONENTTYPE2:
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                        portManifest->lowerConnectionBound = 0;
                        portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                        portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                        portManifest->topic = userTopic1;
                        break;
                    case (xme_core_componentType_t) COMPONENTTYPE3:
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                        portManifest->lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                        portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                        portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                        portManifest->topic = userTopic2;
                        break;
                    case (xme_core_componentType_t) COMPONENTTYPE4:
                        portManifest->portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                        portManifest->lowerConnectionBound = 0;
                        portManifest->upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED;
                        portManifest->attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                        portManifest->topic = userTopic2;
                        break;
                    default:
                        break;
                }
            }

            // Set the rest of values to zero. 
            {
                for (int i = 1; i < XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT; i ++)
                {
                    componentManifest->portManifests[i].portType = XME_CORE_COMPONENT_PORTTYPE_INVALID;
                    componentManifest->portManifests[i].topic = XME_CORE_TOPIC_INVALID_TOPIC;
                    componentManifest->portManifests[i].lowerConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    componentManifest->portManifests[i].upperConnectionBound = XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID;
                    componentManifest->portManifests[i].attrSet = XME_CORE_ATTRIBUTE_EMPTY_ATTRIBUTE_SET;
                }
            }
        }
    
        return XME_STATUS_SUCCESS;
    }

    xme_core_component_t invalidComponentId;
    xme_core_component_t componentId1;
    xme_core_component_t componentId2;

    xme_core_node_nodeId_t invalidNodeId;
    xme_core_node_nodeId_t localNodeId;
    xme_core_node_nodeId_t nodeId2;
    xme_core_node_nodeId_t nodeId3;

    xme_core_node_guid_t guid1;
    xme_core_node_guid_t guid2;
    xme_core_node_guid_t guid3;

    xme_core_componentType_t invalidComponentType;
    xme_core_componentType_t componentTypePublisherTopic1;
    xme_core_componentType_t componentTypeSubscriberTopic1;
    xme_core_componentType_t componentTypePublisherTopic2;
    xme_core_componentType_t componentTypeSubscriberTopic2;

    xme_core_topic_t userTopic1;
    xme_core_topic_t userTopic2;

    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph;

    uint8_t functionIdentifier;
};



/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     PnPManager: instantiateComponentOnNode                                 //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, NULL, invalidNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, NULL, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, NULL, localNodeId, componentId1));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, NULL, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(invalidComponentType, NULL, nodeId2, componentId1));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, invalidNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, invalidNodeId, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeValidParameters)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithTwoSensorsOfSameTypeOnTheSameNodeAndValidComponentId)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, componentId1));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, componentId2));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithTwoSensorsOfSameTypeOnTheSameNodeAndInvalidComponentId)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, invalidComponentId));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInTwoNodesAndInvalidComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId3, invalidComponentId));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInTwoNodesAndValidComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId2, componentId1));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId3, componentId2));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInTwoNodesAndSameComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId2, componentId1));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId3, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInSameNodeAndSameComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId2, componentId1));
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId2, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithOneSensorInLocalNodeAndSameComponentID)
{
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, componentId1));
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, componentId1));
}

TEST_F(PnPManagerIntegrationTest, instantiateComponentOnNodeWithoutManifests)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_removeComponentManifest(componentTypeSensor));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_manifestRepository_removeComponentManifest(componentTypeMonitor));

    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, componentId1++));
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, NULL, localNodeId, componentId1++));
}

//----------------------------------------------------------------------------//
//     PnPManager: hasNextRuntimeGraph + getNextRuntimeGraph                  //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerIntegrationTest, hasNextRuntimeGraphTestPopulationWithComponentIdEstablished)
{
    // One sensor, two monitors. 
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, componentId1));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, NULL, nodeId2, componentId2));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, NULL, nodeId3, componentId3));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
}

TEST_F(PnPManagerIntegrationTest, hasNextRuntimeGraphTestPopulationWithComponentIdNotEstablished)
{
    // One monitor, two sensors
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, NULL, nodeId2, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId3, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
}

TEST_F(PnPManagerIntegrationTest, getNextRuntimeGraphTwoSensorsTwoMonitors)
{
    // One monitor, two sensors
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, NULL, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, NULL, nodeId3, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(invalidNodeId, outGraph));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, outGraph)); // N1 publication for N2 and N3.
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, outGraph)); // no more expected.

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, outGraph)); // N2 subscription for N1 & N2, N2 publication for N2 and N3.
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, outGraph)); // no more expected. 

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, outGraph)); // N3 subscription for N1 & N2.
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, outGraph)); // no more expected.
}

//----------------------------------------------------------------------------//
//     PnPManager: updateInstanceFromRuntimeGraph                             //
//----------------------------------------------------------------------------//

#if 0
TEST_F(PnPManagerIntegrationTest, updateInstanceFromRuntimeGraphTestingOfSeveralSituations)
{
    // One monitor, two sensors
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, localNodeId, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeSensor, NULL, nodeId2, invalidComponentId));
    ASSERT_EQ(XME_STATUS_SUCCESS,
        xme_core_pnp_pnpManager_instantiateComponentOnNode(componentTypeMonitor, NULL, nodeId2, invalidComponentId));

    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(invalidNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(invalidNodeId, outGraph));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, outGraph)); // N1 publication for N2.

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(NULL, XME_STATUS_SUCCESS));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(NULL, XME_STATUS_INTERNAL_ERROR));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(outGraph, XME_STATUS_SUCCESS));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(outGraph, XME_STATUS_INTERNAL_ERROR));

    xme_core_pnp_pnpManager_fini();
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));

    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(NULL, XME_STATUS_SUCCESS));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(NULL, XME_STATUS_INTERNAL_ERROR));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(outGraph, XME_STATUS_SUCCESS));
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph(outGraph, XME_STATUS_INTERNAL_ERROR));
}
#endif

//----------------------------------------------------------------------------//
//     PnPManagerIntegrationTest: Processing Component Instance Manifest      //
//----------------------------------------------------------------------------//

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processNullComponentInstanceManifest)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announceComponentInstanceManifest(NULL));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processInvalidNodeComponentInstanceManifest)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest1));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processValidNodeComponentInstanceManifestWithoutComponents)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest2));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processValidNodeComponentInstanceManifestWithOneComponentInstantiated)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest3));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processValidNodeComponentInstanceManifestWithTwoComponentsInstantiated)
{
    ASSERT_EQ(XME_STATUS_NOT_FOUND, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest4));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest4));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, processSeveralComponentInstanceManifests)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerNode((xme_core_node_nodeId_t)4));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest2)); // without components
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest3)); // with one component
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_announceComponentInstanceManifest(componentInstanceManifest4)); // with two components
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode((xme_core_node_nodeId_t)2));
    EXPECT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode((xme_core_node_nodeId_t)3));
    EXPECT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode((xme_core_node_nodeId_t)4));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, RegisterInterfaceAddressWithInvalidParameters)
{
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_registerInterfaceAddress(XME_CORE_NODE_INVALID_NODE_ID, invalidInterfaceAddress));
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpManager_registerInterfaceAddress(XME_CORE_NODE_INVALID_NODE_ID, validInterfaceAddress));
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_pnp_pnpManager_registerInterfaceAddress((xme_core_node_nodeId_t)1, invalidInterfaceAddress));
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_pnp_pnpManager_registerInterfaceAddress((xme_core_node_nodeId_t)2, validInterfaceAddress));
}

TEST_F(PnPManagerIntegrationTestComponentInstanceManifestTests, RegisterInterfaceAddressWithoutNodeRegistrationAndWithNodeRegistration)
{
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_pnp_pnpManager_registerInterfaceAddress((xme_core_node_nodeId_t)1, validInterfaceAddress));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_directory_nodeRegistryController_registerNode((xme_core_node_nodeId_t)1, "nodeId1", (xme_core_node_guid_t)1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_registerInterfaceAddress((xme_core_node_nodeId_t)1, validInterfaceAddress));
}

TEST_F(PnPManagerPlugOutIntegrationTest, UnannounceComponents)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_unannounceComponentOnNode(nodeId3, componentId1));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, outGraph));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, outGraph));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    // Remove the instance. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_removeComponentInstance(nodeId3, componentId1));

    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_unannounceComponentOnNode(nodeId3, componentId2));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(localNodeId));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(localNodeId, outGraph));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId2));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId2, outGraph));
    ASSERT_TRUE(xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(nodeId3));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_getNextRuntimeGraphForNode(nodeId3, outGraph));
    ASSERT_FALSE(xme_core_pnp_pnpManager_hasNewRuntimeGraphs());
    // Remove the instance. 
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_removeComponentInstance(nodeId3, componentId2));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
