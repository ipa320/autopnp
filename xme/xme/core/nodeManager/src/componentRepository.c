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
 * $Id: componentRepository.c 7838 2014-03-14 12:38:35Z geisinger $
 */

/**
 * \file
 *         Component Repository.
 */

/**
 * \addtogroup core_compRepository
 * @{
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpClientInterface.h"
#include "xme/core/nodeManager/include/componentRepositoryPnpManagerInterface.h"

#include "xme/core/broker/include/brokerPnpManagerInterface.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"
#include "xme/core/directory/include/topicRegistry.h"
#include "xme/core/log.h"
#include "xme/core/manifestRepository/include/manifestRepository.h"

#include "xme/hal/include/mem.h"

#include <inttypes.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define XME_CORE_NODEMGR_COMPREP_STATE_BUILD_IN_PROGRESS -1 ///< Internal state for components that are still under construction, by a component builder.

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef componentInstance_t
 *
 * \brief Component instance data structure.
 */
typedef struct
{
    xme_hal_table_rowHandle_t handle; ///< Corresponding handle in componentInstanceTable.
    xme_core_node_nodeId_t nodeID; ///< The node ID of the node on which this component instance is located.
    xme_core_component_t componentID; ///< The component ID.
    xme_core_componentType_t componentType; ///< The component type.
    char* initializationString; ///< Component-specific initialization data. Memory is allocated by the component repository.
    xme_core_nodeMgr_compRep_state_t state; ///< Current state of the component. Some operations are only allowed for certain states.
    xme_core_component_config_t* componentConfig;  ///< Component-specific configuration strucutre data. Memory is allocated by the component repository.
    xme_hal_singlyLinkedList_t(XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT) ports; ///< List of contained port instances.
    xme_hal_singlyLinkedList_t(XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT) functions; ///< List of contained function instances.
    xme_hal_table_rowHandle_t foreignComponentHandle; ///< Used by plug and play manager to remember component handles from instance manifests.
} componentInstance_t;

/**
 * \typedef portInstance_t
 *
 * \brief Port instance data structure.
 */
typedef struct
{
    xme_hal_table_rowHandle_t handle; ///< Corresponding handle in portInstanceTable.
    const componentInstance_t* componentInstance; ///< Poiner to component instance which contains this port.
    xme_core_dataManager_dataPacketId_t dataPacketID; ///< The data packet ID of this port.
    uint8_t queueSize; ///< The queue size of this port.
        // TODO: Check data type (8bit vs. 16bit); See Issue #4210, see also issue #3943
} portInstance_t;

/**
 * \typedef functionInstance_t
 *
 * \brief Function instance data structure.
 */
typedef struct
{
    xme_hal_table_rowHandle_t handle; ///< Corresponding handle in functionInstanceTable.
    const componentInstance_t* componentInstance; ///< Poiner to component instance which contains this function.
    xme_hal_time_timeInterval_t executionPeriod; ///< Execution period of the function.
    xme_core_exec_functionDescriptor_t functionDescriptor; ///< The descriptor of the function.
} functionInstance_t;

/**
 * \brief Data structure of component builder.
 */
struct xme_core_nodeMgr_compRep_componentBuilder_s
{
    char error; ///< Indicates if any error occurred during one of the builder's setter calls.
    componentInstance_t* componentInstance; ///< The component instance under construction.
};

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief Table for storing the component instances.
 */
static XME_HAL_TABLE
(
    componentInstance_t,
    componentInstanceTable,
    XME_CORE_NODEMGR_COMPREP_COMPONENT_TABLE_MAX
);

/**
 * \brief Table for storing the port instances.
 */
static XME_HAL_TABLE
(
    portInstance_t,
    portInstanceTable,
    XME_CORE_NODEMGR_COMPREP_PORT_TABLE_MAX
);

/**
 * \brief Table for storing the function instances.
 */
static XME_HAL_TABLE
(
    functionInstance_t,
    functionInstanceTable,
    XME_CORE_NODEMGR_COMPREP_FUNCTION_TABLE_MAX
);

/**
 * \brief Current row handle in componentInstanceTable of component iterator.
 */
static xme_hal_table_rowHandle_t
componentIteratorCurrentHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief Helper function for xme_core_nodeMgr_compRep_createBuilder().
 *        Initializes ports according to the given component type manifest.
 *
 * \details All ports are allocated dynamically and are NOT added to the table.
 *
 * \param[in] componentManifest Given component manifest.
 * \param[out] componentInstance Given component instance.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successful.
 * \retval XME_STATUS_OUT_OF_RESOURCES if memory allocation failed.
 */
static xme_status_t
initPortInstances
(
    const xme_core_componentManifest_t* const componentManifest,
    componentInstance_t* const componentInstance
);

/**
 * \brief Helper function for xme_core_nodeMgr_compRep_createBuilder().
 *        Initializes functions according to the given component type manifest.
 *
 * \details All functions are allocated dynamically and are NOT added to the table.
 *
 * \param[in] componentInstance Given component instance.
 * \param[in] componentConfig Pointer to component configuration structure.
 * \param[out] componentInstance Given component instance.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successful.
 * \retval XME_STATUS_OUT_OF_RESOURCES if memory allocation failed.
 */
static xme_status_t
initFunctionInstances
(
    const xme_core_componentManifest_t* const componentManifest,
    xme_core_component_config_t* componentConfig,
    componentInstance_t* const componentInstance
);

/**
 * \brief Internal function for setting the component ID.
 *        Always use this to set the ID, as it will also update the function
 *        descriptors (which also contain the component ID).
 *
 * \param[out] componentInstance Given component instance.
 * \param[in] componentID The new component ID.
 */
static void
xme_core_nodeMgr_compRep_setComponentIDInternal
(
    componentInstance_t* componentInstance,
    xme_core_component_t componentID
);

/**
 * \brief Helper function for xme_core_nodeMgr_compRep_createAndRegisterComponent().
 *        Creates data packets in data handler.
 *
 * \param[in] componentInstance Given component instance.
 * \param[in] componentManifest Manifest of given component.
 *
 * \retval XME_STATUS_SUCCESS if the operation is successful.
 * \retval XME_STATUS_NOT_FOUND if the topic size of attribute descriptors could
 *         not be found.
 * \retval XME_STATUS_INTERNAL_ERROR if any function call to other components
 *         fails.
 */
static xme_status_t
createPorts
(
    const componentInstance_t* const componentInstance,
    const xme_core_componentManifest_t* const componentManifest
);

/**
 * \brief Helper function for xme_core_nodeMgr_compRep_createAndRegisterComponent().
 *        Registers functions in broker.
 *
 * \param[in] componentInstance Given component instance.
 * \param[in] componentManifest Manifest of given component.
 *
 * \retval XME_STATUS_SUCCESS if the operation is successful.
 * \retval XME_STATUS_INTERNAL_ERROR if any function call to other components
 *         fails.
 */
static xme_status_t
registerFunctions
(
    const componentInstance_t* const componentInstance,
    const xme_core_componentManifest_t* const componentManifest
);

/**
 * \brief Deregister functions and destroy ports of gicen component instance.
 *        Performs no actions for non-local components.
 *
 * \param[in] componentInstance Given component instance.
 */
static void
deregisterComponent
(
    const componentInstance_t* const componentInstance
);

/**
 * \brief Sets componentIteratorCurrentHandle to next value in iteration.
 *        No checks are performed.
 */
static void
componentIteratorNextInternal(void);

/**
 * \brief Returns whether the given node IDs are equal.
 *
 * \details Two node IDs are considered equal if they have the same value
 *          or if one of them is XME_CORE_NODE_LOCAL_NODE_ID and the
 *          other one matches the local node's currently assigned node ID.
 *
 * \param[in] nodeID1 The first node identifier. 
 * \param[in] nodeID2 The second node identifier. 
 *
 * \return The result of comparison. If the result is true, both nodes are equals. 
 */
static bool
areEqualNodeIDs
(
    xme_core_node_nodeId_t nodeID1,
    xme_core_node_nodeId_t nodeID2
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_nodeMgr_compRep_init(void)
{
    XME_HAL_TABLE_INIT(componentInstanceTable);
    XME_HAL_TABLE_INIT(portInstanceTable);
    XME_HAL_TABLE_INIT(functionInstanceTable);

    return XME_STATUS_SUCCESS;
}

void
xme_core_nodeMgr_compRep_fini(void)
{
    XME_HAL_TABLE_ITERATE_BEGIN(componentInstanceTable,
        xme_hal_table_rowHandle_t, handle, componentInstance_t, item);
    {
        // Destroy will free resources for us (free allocated memory etc.)
        xme_core_nodeMgr_compRep_destroyComponentInstance(handle);
    }
    XME_HAL_TABLE_ITERATE_END();

    // Assert that the previous destroy calls removed everything
    {
        XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(0u == XME_HAL_TABLE_ITEM_COUNT(componentInstanceTable)));
        XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(0u == XME_HAL_TABLE_ITEM_COUNT(portInstanceTable)));
        XME_ASSERT_NORVAL(XME_ASSERT_NO_SIDE_EFFECTS(0u == XME_HAL_TABLE_ITEM_COUNT(functionInstanceTable)));
    }

    XME_HAL_TABLE_FINI(componentInstanceTable);
    XME_HAL_TABLE_FINI(portInstanceTable);
    XME_HAL_TABLE_FINI(functionInstanceTable);
}

xme_core_nodeMgr_compRep_componentBuilder_t*
xme_core_nodeMgr_compRep_createBuilder
(
    xme_core_node_nodeId_t nodeID,
    xme_core_componentType_t componentType
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_nodeMgr_compRep_componentBuilder_t* builder = NULL;

    XME_CHECK_MSG(XME_CORE_COMPONENT_TYPE_INVALID != componentType, NULL, XME_LOG_ERROR, "[componentBuilder] Given component type is invalid.\n");

    builder = (xme_core_nodeMgr_compRep_componentBuilder_t*)
        xme_hal_mem_alloc(sizeof(xme_core_nodeMgr_compRep_componentBuilder_t));

    XME_CHECK_MSG(NULL != builder, NULL, XME_LOG_ERROR, "[componentBuilder] Not enough memory for creating builder.\n");

    {
        xme_hal_table_rowHandle_t componentHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
        xme_core_componentManifest_t componentManifest;

        componentHandle = XME_HAL_TABLE_ADD_ITEM(componentInstanceTable);
        XME_CHECK_MSG_REC(XME_HAL_TABLE_INVALID_ROW_HANDLE != componentHandle, NULL, { xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle); xme_hal_mem_free(builder); }, XME_LOG_ERROR, "[componentBuilder] Creation of builder failed. Not enough memory for new component instance.\n");

        builder->componentInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);
        XME_ASSERT_RVAL(NULL != builder->componentInstance, NULL);

        (void)xme_hal_mem_set(builder->componentInstance, 0u, sizeof(componentInstance_t));

        builder->componentInstance->handle = componentHandle;
        builder->componentInstance->state = XME_CORE_NODEMGR_COMPREP_STATE_BUILD_IN_PROGRESS;
        builder->componentInstance->nodeID = nodeID;
        builder->componentInstance->componentID = XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT;
        builder->componentInstance->foreignComponentHandle = componentHandle;
        builder->componentInstance->componentType = componentType;
        XME_HAL_SINGLYLINKEDLIST_INIT(builder->componentInstance->ports);
        XME_HAL_SINGLYLINKEDLIST_INIT(builder->componentInstance->functions);

        // Get component manifest
        status = xme_core_manifestRepository_findComponentManifest(builder->componentInstance->componentType, &componentManifest);
        XME_ASSERT_RVAL(XME_STATUS_SUCCESS == status || XME_STATUS_NOT_FOUND == status, NULL);
        XME_CHECK_MSG_REC(XME_STATUS_NOT_FOUND != status, NULL, { xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle); xme_hal_mem_free(builder); }, XME_LOG_ERROR, "[componentBuilder] Creation of builder failed. No manifest found for component type %d.\n", builder->componentInstance->componentType);

        // Build ports
        status = initPortInstances(&componentManifest, builder->componentInstance);
        XME_CHECK_REC(XME_STATUS_SUCCESS == status, NULL, { xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle); });

        // Allocate memory for component configuration
        if (0U < componentManifest.configStructSize)
        {
            builder->componentInstance->componentConfig = (xme_core_component_config_t*)
                xme_hal_mem_alloc(componentManifest.configStructSize);
            XME_CHECK_MSG_REC(NULL != builder->componentInstance->componentConfig, NULL, { xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle); xme_hal_mem_free(builder); }, XME_LOG_ERROR,  "[componentBuilder] Creation of builder failed. Not enough memory for configuration structure of component.\n");
	    bzero(builder->componentInstance->componentConfig, componentManifest.configStructSize);
        }
        else
        {
            builder->componentInstance->componentConfig = NULL;
        }

        // Build functions
        status = initFunctionInstances(&componentManifest, builder->componentInstance->componentConfig, builder->componentInstance);
        XME_CHECK_REC(XME_STATUS_SUCCESS == status, NULL, { xme_core_nodeMgr_compRep_destroyComponentInstance(componentHandle); xme_hal_mem_free(builder); });
    }

    builder->error = false;

    return builder;
}

static xme_status_t
initPortInstances
(
    const xme_core_componentManifest_t* const componentManifest,
    componentInstance_t* const componentInstance
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    uint16_t portCount = 0u;
    uint16_t portIndex = 0u;

    portCount = xme_core_manifestRepository_getPortCount(componentManifest);
    for (portIndex = 0u; portIndex < portCount; portIndex++)
    {
        xme_core_componentPortManifest_t portManifest = componentManifest->portManifests[portIndex];

        {
            xme_hal_table_rowHandle_t portHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            portInstance_t* portInstance = NULL;

            portHandle = XME_HAL_TABLE_ADD_ITEM(portInstanceTable);
            XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != portHandle, XME_STATUS_OUT_OF_RESOURCES);

            portInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(portInstanceTable, portHandle);
            XME_ASSERT(NULL != portInstance);

            xme_hal_mem_set(portInstance, 0u, sizeof(portInstance_t));

            portInstance->handle = portHandle;
            portInstance->componentInstance = componentInstance;
            portInstance->queueSize = portManifest.queueSize;

            status = xme_hal_singlyLinkedList_addItem(&(componentInstance->ports), portInstance);
            XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_OUT_OF_RESOURCES);
        }
    }

    return XME_STATUS_SUCCESS;
}

static xme_status_t
initFunctionInstances
(
    const xme_core_componentManifest_t* const componentManifest,
    xme_core_component_config_t* componentConfig,
    componentInstance_t* const componentInstance
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    uint16_t functionCount = 0u;
    uint16_t functionIndex = 0u;

    functionCount = xme_core_manifestRepository_getFunctionCount(componentManifest);
    for (functionIndex = 0u; functionIndex < functionCount; functionIndex++)
    {
        xme_hal_table_rowHandle_t functionHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
        functionInstance_t* functionInstance = NULL;
        xme_core_functionManifest_t functionManifest = componentManifest->functionManifests[functionIndex];

        functionHandle = XME_HAL_TABLE_ADD_ITEM(functionInstanceTable);
        XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != functionHandle, XME_STATUS_OUT_OF_RESOURCES);

        functionInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(functionInstanceTable, functionHandle);
        XME_ASSERT(NULL != functionInstance);

        xme_hal_mem_set(functionInstance, 0u, sizeof(functionInstance_t));

        functionInstance->handle = functionHandle;
        functionInstance->componentInstance = componentInstance;

        xme_hal_mem_set(&(functionInstance->functionDescriptor), 0u, sizeof(xme_core_exec_functionDescriptor_t));

        functionInstance->functionDescriptor.task = functionManifest.functionWrapperExecute;
        functionInstance->functionDescriptor.taskArgs = componentConfig;
        functionInstance->functionDescriptor.componentId = componentInstance->componentID;
        functionInstance->functionDescriptor.functionId = functionManifest.functionId;
        functionInstance->functionDescriptor.init = functionManifest.functionInit;
        functionInstance->functionDescriptor.fini = functionManifest.functionFini;
        functionInstance->functionDescriptor.wcet_ns = functionManifest.wcet;

        status = xme_hal_singlyLinkedList_addItem(&(componentInstance->functions), functionInstance);
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_OUT_OF_RESOURCES);
    }

    return XME_STATUS_SUCCESS;
}

void
xme_core_nodeMgr_compRep_builderSetComponentID
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    xme_core_component_t componentID
)
{
    if (NULL == componentBuilder)
    {
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetComponentID(): Given builder is NULL.\n");
        return;
    }

    xme_core_nodeMgr_compRep_setComponentIDInternal(componentBuilder->componentInstance, componentID);
}

void
xme_core_nodeMgr_compRep_builderSetInitializationString
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    const char* const initializationString
)
{
    size_t strLen = 0u;
    size_t maxLen = 256u;

    if (NULL == componentBuilder)
    {
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetInitializationString(): Given builder is NULL.\n");
        return;
    }

    xme_hal_mem_free(componentBuilder->componentInstance->initializationString);

    if (NULL == initializationString)
    {
        componentBuilder->componentInstance->initializationString = NULL;
        return;
    }

    strLen = xme_hal_safeString_strnlen(initializationString, maxLen);
    if (maxLen == strLen && initializationString[maxLen - 1] != '\0')
    {
        componentBuilder->error = true;
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetInitializationString(): Initialization string truncated. Maximum length (including null-terminator) is %d.\n", maxLen);
        return;
    }

    componentBuilder->componentInstance->initializationString = (char*)xme_hal_mem_alloc(strLen + 1u);
    if (NULL == componentBuilder->componentInstance->initializationString)
    {
        componentBuilder->error = true;
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetInitializationString(): Not enough memory to set initialization string.\n");
        return;
    }

    (void)xme_hal_safeString_strncpy
    (
        &(componentBuilder->componentInstance->initializationString[0]),
        initializationString,
        strLen + 1
    );
}

void
xme_core_nodeMgr_compRep_builderSetQueueSize
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    uint16_t portTypeID,
    uint8_t queueSize
)
{
    portInstance_t* portInstance = NULL;

    if (NULL == componentBuilder)
    {
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetQueueSize(): Given builder is NULL.\n");
        return;
    }

    if (0u == queueSize)
    {
        componentBuilder->error = true;
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetQueueSize(): Invalid queue size 0. Queue size must be at least 1.\n");
        return;
    }

    portInstance = (portInstance_t*)
        xme_hal_singlyLinkedList_itemFromIndex(&(componentBuilder->componentInstance->ports), portTypeID);
    
    if (NULL == portInstance)
    {
        componentBuilder->error = true;
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetQueueSize(): Invalid portTypeID %" PRIu16 ".\n", portTypeID);
        return;
    }

    portInstance->queueSize = queueSize;
}

void
xme_core_nodeMgr_compRep_builderSetExecutionPeriod
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    uint16_t functionTypeID,
    xme_hal_time_timeInterval_t executionPeriod
)
{
    functionInstance_t* functionInstance = NULL;

    if (NULL == componentBuilder)
    {
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetExecutionPeriod(): Given builder is NULL.\n");
        return;
    }

    functionInstance = (functionInstance_t*)
        xme_hal_singlyLinkedList_itemFromIndex(&(componentBuilder->componentInstance->functions), functionTypeID);
    
    if (NULL == functionInstance)
    {
        componentBuilder->error = true;
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetExecutionPeriod(): Called for invalid functionTypeID %" PRIu16 ".\n", functionTypeID);
        return;
    }

    functionInstance->executionPeriod = executionPeriod;
}

void
xme_core_nodeMgr_compRep_builderSetForeignComponentHandle
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    xme_core_nodeMgr_compRep_componentHandle_t foreignComponentHandle
)
{
    if (NULL == componentBuilder)
    {
        XME_LOG(XME_LOG_ERROR, "[componentBuilder] builderSetForeignComponentHandle(): Given builder is NULL.\n");
        return;
    }

    componentBuilder->componentInstance->foreignComponentHandle =
        (xme_hal_table_rowHandle_t)foreignComponentHandle;
}

xme_status_t
xme_core_nodeMgr_compRep_build
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    xme_core_nodeMgr_compRep_componentHandle_t* outComponentHandle
)
{
    XME_CHECK_MSG(NULL != componentBuilder, XME_STATUS_INVALID_PARAMETER, XME_LOG_ERROR, "[componentBuilder] build(): Given builder is NULL.\n");

    // Check if any error occurred during construction
    XME_CHECK(!(componentBuilder->error), XME_STATUS_INTERNAL_ERROR);

    // Check if component instance with given nodeID and componentID already exists
    if (XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentBuilder->componentInstance->componentID)
    {
        xme_hal_table_rowHandle_t rowHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
        componentInstance_t* item = NULL;

        XME_HAL_TABLE_GET_NEXT
        (
            componentInstanceTable,
            xme_hal_table_rowHandle_t, rowHandle,
            componentInstance_t, item,
            (
                areEqualNodeIDs(item->nodeID, componentBuilder->componentInstance->nodeID) &&
                item->componentID == componentBuilder->componentInstance->componentID &&
                XME_CORE_NODEMGR_COMPREP_STATE_PREPARED <= item->state
            )
        );

        XME_CHECK_REC
        (
            XME_HAL_TABLE_INVALID_ROW_HANDLE == rowHandle,
            XME_STATUS_ALREADY_EXIST,
            {
                if (NULL != outComponentHandle)
                {
                    *outComponentHandle = rowHandle;
                }

                xme_core_nodeMgr_compRep_destroyComponentInstance(componentBuilder->componentInstance->handle);
                xme_hal_mem_free(componentBuilder);
            }
        );
    }

    componentBuilder->componentInstance->state = XME_CORE_NODEMGR_COMPREP_STATE_PREPARED;
    
    if (NULL != outComponentHandle)
    {
        *outComponentHandle = componentBuilder->componentInstance->handle;
    }

    xme_hal_mem_free(componentBuilder);

    return XME_STATUS_SUCCESS;
}

void
xme_core_nodeMgr_compRep_cancelBuild
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder
)
{
    XME_CHECK(NULL != componentBuilder, XME_CHECK_RVAL_VOID);

    xme_core_nodeMgr_compRep_destroyComponentInstance(componentBuilder->componentInstance->handle);

    xme_hal_mem_free(componentBuilder);
}

xme_status_t
xme_core_nodeMgr_compRep_createAndRegisterComponent
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_componentManifest_t componentManifest;

    componentInstance_t* componentInstance =
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK_MSG(NULL != componentInstance, XME_STATUS_NOT_FOUND, XME_LOG_ERROR, "[componentRepository] Registration of component failed. Given component handle is unknown.\n");
    XME_CHECK_MSG(XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED == componentInstance->state , XME_STATUS_INVALID_CONFIGURATION, XME_LOG_DEBUG, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Component instance state is not ANNOUNCED. State was: %d.\n", componentInstance->componentID, componentInstance->componentType, componentInstance->state);

    status = xme_core_manifestRepository_findComponentManifest(componentInstance->componentType, &componentManifest);
    XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_NOT_FOUND, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Component type manifest not found.\n", componentInstance->componentID, componentInstance->componentType);

    if (NULL != componentManifest.componentWrapperInit)
    {
        status = componentManifest.componentWrapperInit();
        XME_CHECK_MSG
        (
            XME_STATUS_SUCCESS == status,
            status,
            XME_LOG_ERROR,
            "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Component wrapper init function returned status %d.\n",
            componentInstance->componentID, componentInstance->componentType, status
        );
    }

    status = createPorts(componentInstance, &componentManifest);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    status = registerFunctions(componentInstance, &componentManifest);
    XME_CHECK_REC(XME_STATUS_SUCCESS == status, status, { deregisterComponent(componentInstance); });

    if (NULL != componentManifest.componentInit)
    {
        status = componentManifest.componentInit(componentInstance->componentConfig, componentInstance->initializationString);
        XME_CHECK_MSG_REC(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, { deregisterComponent(componentInstance); }, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Component initialization function returned error status %d.\n", componentInstance->componentID, componentInstance->componentType, status);
    }

    componentInstance->state = XME_CORE_NODEMGR_COMPREP_STATE_CREATED;

    return XME_STATUS_SUCCESS;
}

static xme_status_t
createPorts
(
    const componentInstance_t* const componentInstance,
    const xme_core_componentManifest_t* const componentManifest
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_hal_linkedList_index_t portIndex = 0u;
    xme_hal_linkedList_index_t portCount = 0u;

    portCount = xme_hal_singlyLinkedList_getItemCount(&(componentInstance->ports));

    XME_CHECK_MSG(0u == portCount || NULL != componentManifest->componentWrapperReceivePort, XME_STATUS_INVALID_CONFIGURATION, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Component wrapper receive port function in manifest is NULL (must not be NULL when there is at least one port).\n", componentInstance->componentID, componentInstance->componentType);

    for (portIndex = 0u; portIndex < portCount; portIndex++)
    {
        xme_core_attribute_descriptor_list_t attributeDescriptorList;
        uint16_t topicSize = 0u;
        xme_core_componentPortManifest_t portManifest = componentManifest->portManifests[portIndex];
        portInstance_t* portInstance = (portInstance_t*)
            xme_hal_singlyLinkedList_itemFromIndex(&(componentInstance->ports), portIndex);
        XME_ASSERT(NULL != portInstance);

        status = xme_core_directory_attribute_getAttributeDescriptorList
        (
            portManifest.topic,
            &attributeDescriptorList
        );
        XME_ASSERT(XME_STATUS_SUCCESS == status || XME_STATUS_NOT_FOUND == status);
        XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_NOT_FOUND, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. No attribute descriptor list found for topic %" PRIu16 ". Topic may not be registered in topic registry.\n", componentInstance->componentID, componentInstance->componentType, portManifest.topic);

        status = xme_core_directory_topicRegistry_getTopicSize(portManifest.topic, &topicSize);
        XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_NOT_FOUND, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Size not found for topic %" PRIu16 ". Topic may not be registered in topic registry.\n", componentInstance->componentID, componentInstance->componentType, portManifest.topic);

        // Search for previously created port of a component instance of the same type, we will reuse that port
        // TODO: Update this after issue #3861 is resolved
        portInstance->dataPacketID = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
        {
            xme_hal_table_rowHandle_t rowHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
            componentInstance_t* item;

            XME_HAL_TABLE_GET_NEXT
            (
                componentInstanceTable,
                xme_hal_table_rowHandle_t, rowHandle,
                componentInstance_t, item,
                (
                    areEqualNodeIDs(item->nodeID, XME_CORE_NODE_LOCAL_NODE_ID) &&
                    item->componentType == componentInstance->componentType &&
                    XME_CORE_NODEMGR_COMPREP_STATE_CREATED == item->state
                )
            );

            if (XME_HAL_TABLE_INVALID_ROW_HANDLE != rowHandle)
            {
                // Reuse existing port
                portInstance_t* existingPortInstance = NULL;

                existingPortInstance = (portInstance_t*)xme_hal_singlyLinkedList_itemFromIndex(&(item->ports), portIndex);
                XME_ASSERT(NULL != existingPortInstance);

                portInstance->dataPacketID = existingPortInstance->dataPacketID;
            }
        }

        if (XME_CORE_DATAMANAGER_DATAPACKETID_INVALID == portInstance->dataPacketID)
        {
            status = xme_core_dataHandler_createDataPacket(topicSize, &(portInstance->dataPacketID));
            XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Creation of data packet (portIndex: %d) failed with status %d.\n", componentInstance->componentID, componentInstance->componentType, portIndex, status);

            {
                uint8_t i = 0u;

                for (i = 0u; i < attributeDescriptorList.length; i++)
                {
                    status = xme_core_dataHandler_createAttribute
                    (
                        attributeDescriptorList.element[i].size,
                        attributeDescriptorList.element[i].key,
                        portInstance->dataPacketID
                    );
                    XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Addition of attribute to data packet (portIndex: %d) failed with status %d.\n", componentInstance->componentID, componentInstance->componentType, portIndex, status);
                }
            }

            status = xme_core_dataHandler_setDataPacketQueueSize(portInstance->dataPacketID, portInstance->queueSize); // Using queueSize from instance, not from manifest!
            XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Setting of queue size of data packet failed with status %d.\n", componentInstance->componentID, componentInstance->componentType, status);

            if (portManifest.persistent)
            {
                status = xme_core_dataHandler_setDataPacketPersistent(portInstance->dataPacketID);
                XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Setting data packet as persistent failed with status %d.\n", componentInstance->componentID, componentInstance->componentType, status);
            }

            status = xme_core_dataHandler_setDataPacketOverwrite(portInstance->dataPacketID, true, XME_STATUS_BUFFER_TOO_SMALL, false);
            XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Setting of overwrite behaviour failed with status %d.\n", componentInstance->componentID, componentInstance->componentType, status);

            // Set port in component wrapper
            // TODO: Consolidate port index data types: Component wrapper uses uint8_t, other places use uint16_t (issue #4210)
            XME_CHECK_MSG(UCHAR_MAX >= portIndex, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Port count over %d not supported.\n", componentInstance->componentID, componentInstance->componentType, UCHAR_MAX);
            status = componentManifest->componentWrapperReceivePort(portInstance->dataPacketID, (uint8_t)portIndex); // NULL != componentWrapperReceivePort has alread been checked
            XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Call to component wrapper's receivePort returned error status %d.\n", componentInstance->componentID, componentInstance->componentType, status);
        }

        status = xme_core_dataHandler_configure();
        XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Configuration of data handler returned error status %d.\n", componentInstance->componentID, componentInstance->componentType, status);
    }

    return XME_STATUS_SUCCESS;
}

static xme_status_t
registerFunctions
(
    const componentInstance_t* const componentInstance,
    const xme_core_componentManifest_t* const componentManifest
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    uint16_t functionCount = 0u;
    uint16_t functionIndex = 0u;

    functionCount = xme_core_manifestRepository_getFunctionCount(componentManifest);
    for (functionIndex = 0u; functionIndex < functionCount; functionIndex++)
    {
        xme_core_functionManifest_t functionManifest = componentManifest->functionManifests[functionIndex];
        functionInstance_t* functionInstance = (functionInstance_t*)
            xme_hal_singlyLinkedList_itemFromIndex(&(componentInstance->functions), functionIndex);
        XME_ASSERT(NULL != functionInstance);

        // Register function with broker
        status = xme_core_broker_registerFunction
        (
            functionInstance->functionDescriptor.componentId,
            functionInstance->functionDescriptor.functionId,
            functionInstance->functionDescriptor.taskArgs
        );
        XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Registration of function (functionId = %d) in broker failed.\n", componentInstance->componentID, componentInstance->componentType, functionInstance->functionDescriptor.functionId);

        // Register mandatory input ports in broker
        {
            uint16_t requiredPortIndex = 0u;

            for (requiredPortIndex = 0u; requiredPortIndex < functionManifest.requiredPortIndicesLength; requiredPortIndex++)
            {
                uint16_t portIndex = functionManifest.requiredPortIndices[requiredPortIndex];
                xme_core_component_portType_t portType = componentManifest->portManifests[portIndex].portType;

                // Only register input ports
                if (XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION == portType ||
                    XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == portType ||
                    XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == portType)
                {
                    xme_core_nodeMgr_compRep_portHandle_t portHandle =
                        xme_core_nodeMgr_compRep_getPort(componentInstance->handle, portIndex);

                    status = xme_core_broker_addDataPacketToFunction
                    (
                        xme_core_nodeMgr_compRep_getDataPacketID(portHandle),
                        functionInstance->functionDescriptor.componentId,
                        functionInstance->functionDescriptor.functionId,
                        functionInstance->functionDescriptor.taskArgs,
                        true
                    );
                    XME_CHECK_MSG(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR, XME_LOG_ERROR, "[componentRepository] Registration of component (ID %" PRIu32 ", type %d) failed. Registration of required input port (portIndex = %d) in broker failed.\n", componentInstance->componentID, componentInstance->componentType, portIndex);
                }
            }
        }
    }

    return XME_STATUS_SUCCESS;
}

static void
deregisterComponent
(
    const componentInstance_t* const componentInstance
)
{
    xme_core_componentManifest_t componentManifest;
    xme_status_t status;

    if (xme_core_node_getCurrentNodeId() != componentInstance->nodeID &&
        XME_CORE_NODE_LOCAL_NODE_ID != componentInstance->nodeID)
    {
        return;
    }

    // Finalize component wrapper
    if (componentInstance->state >= XME_CORE_NODEMGR_COMPREP_STATE_CREATED)
    {
        status = xme_core_manifestRepository_findComponentManifest(componentInstance->componentType, &componentManifest);
        if (XME_STATUS_SUCCESS == status)
        {
            if (NULL != componentManifest.componentWrapperFini)
            {
                componentManifest.componentWrapperFini();
            }
        }
        else
        {
            XME_LOG_IF(XME_STATUS_SUCCESS != status, XME_LOG_ERROR, "[componentRepository] Deregistration of component (ID %" PRIu32 ", type %d) incomplete. Component type manifest not found.\n", componentInstance->componentID, componentInstance->componentType);
        }
    }

    // TODO (Issue #4309): Data packets cannot be destroyed yet

    // Deregister functions
    {
        xme_hal_linkedList_index_t functionIndex = 0u;

        while (true)
        {
            functionInstance_t* functionInstance = (functionInstance_t*)
                xme_hal_singlyLinkedList_itemFromIndex(&(componentInstance->functions), functionIndex);

            if (NULL == functionInstance) { break; }

            (void)xme_core_broker_removeFunction(componentInstance->componentID, functionInstance->functionDescriptor.functionId);
            // We do not care here if the function has not been registered before (can happen during error recovery during registration
            // of a component)

            functionIndex++;
        }
    }
}

void
xme_core_nodeMgr_compRep_destroyComponentInstance
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;

    componentInstance_t* componentInstance =
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    if (NULL == componentInstance) return;

    deregisterComponent(componentInstance);

    xme_hal_mem_free(componentInstance->initializationString);
    xme_hal_mem_free(componentInstance->componentConfig);

    {
        xme_hal_linkedList_index_t count = xme_hal_singlyLinkedList_getItemCount(&(componentInstance->ports));
        xme_hal_linkedList_index_t i = 0;
        portInstance_t* portInstance = NULL;

        for (i = 0; i < count; i++)
        {
            portInstance = (portInstance_t*)
                xme_hal_singlyLinkedList_itemFromIndex(&(componentInstance->ports), i);
            XME_ASSERT_NORVAL(NULL != portInstance);

            status = XME_HAL_TABLE_REMOVE_ITEM(portInstanceTable, portInstance->handle);
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
        }

        xme_hal_singlyLinkedList_fini(&(componentInstance->ports));
    }

    {
        xme_hal_linkedList_index_t count = xme_hal_singlyLinkedList_getItemCount(&(componentInstance->functions));
        xme_hal_linkedList_index_t i = 0;
        functionInstance_t* functionInstance = NULL;

        for (i = 0; i < count; i++)
        {
            functionInstance = (functionInstance_t*)
                xme_hal_singlyLinkedList_itemFromIndex(&(componentInstance->functions), i);
            XME_ASSERT_NORVAL(NULL != functionInstance);

            status = XME_HAL_TABLE_REMOVE_ITEM(functionInstanceTable, functionInstance->handle);
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
        }

        xme_hal_singlyLinkedList_fini(&(componentInstance->functions));
    }

    status = XME_HAL_TABLE_REMOVE_ITEM(componentInstanceTable, (xme_hal_table_rowHandle_t)componentHandle);
    XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
}

xme_status_t
xme_core_nodeMgr_compRep_getComponentInstance
(
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID,
    xme_core_nodeMgr_compRep_componentHandle_t* const outComponentHandle
)
{
    componentInstance_t* componentInstance = NULL;
    xme_hal_table_rowHandle_t rowHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    XME_CHECK(NULL != outComponentHandle, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_GET_NEXT
    (
        componentInstanceTable,
        xme_hal_table_rowHandle_t, rowHandle,
        xme_core_nodeMgr_compRep_componentInstance_t, componentInstance,
        (
            areEqualNodeIDs(componentInstance->nodeID, nodeID) &&
            componentInstance->componentID == componentID
        )
    );

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != rowHandle, XME_STATUS_NOT_FOUND);

    *outComponentHandle = rowHandle;

    return XME_STATUS_SUCCESS;
}

xme_core_nodeMgr_compRep_componentHandle_t
xme_core_nodeMgr_compRep_getComponentInstanceOfPort
(
    xme_core_nodeMgr_compRep_portHandle_t portHandle
)
{
    portInstance_t* portInstance = NULL;

    portInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(portInstanceTable, portHandle);

    XME_CHECK(NULL != portInstance, XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE);

    return portInstance->componentInstance->handle;
}

xme_core_nodeMgr_compRep_componentHandle_t
xme_core_nodeMgr_compRep_getComponentInstanceOfFunction
(
    xme_core_nodeMgr_compRep_functionHandle_t functionHandle
)
{
    functionInstance_t* functionInstance = NULL;

    functionInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(functionInstanceTable, functionHandle);

    XME_CHECK(NULL != functionInstance, XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE);

    return functionInstance->componentInstance->handle;
}

static void
componentIteratorNextInternal(void)
{
    componentInstance_t* currentComponentInstance = NULL;

    XME_HAL_TABLE_GET_NEXT
    (
        componentInstanceTable,
        xme_hal_table_rowHandle_t, componentIteratorCurrentHandle,
        xme_core_nodeMgr_compRep_componentInstance_t, currentComponentInstance,
        (XME_CORE_NODEMGR_COMPREP_STATE_BUILD_IN_PROGRESS < currentComponentInstance->state)
    );
}

void
xme_core_nodeMgr_compRep_componentIteratorInit(void)
{
    componentIteratorCurrentHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;

    componentIteratorNextInternal();
}

bool
xme_core_nodeMgr_compRep_componentIteratorHasNext(void)
{
    return componentIteratorCurrentHandle != XME_HAL_TABLE_INVALID_ROW_HANDLE;
}

xme_core_nodeMgr_compRep_componentHandle_t
xme_core_nodeMgr_compRep_componentIteratorNext(void)
{
    xme_hal_table_rowHandle_t next;

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != componentIteratorCurrentHandle, XME_HAL_TABLE_INVALID_ROW_HANDLE);

    next = componentIteratorCurrentHandle;

    componentIteratorNextInternal();

    return next;
}

void
xme_core_nodeMgr_compRep_componentIteratorFini(void)
{
    componentIteratorCurrentHandle = XME_HAL_TABLE_INVALID_ROW_HANDLE;
}

xme_core_node_nodeId_t
xme_core_nodeMgr_compRep_getNodeID
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    componentInstance_t* componentInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, XME_CORE_NODE_INVALID_NODE_ID);

    return componentInstance->nodeID;
}

xme_core_component_t
xme_core_nodeMgr_compRep_getComponentID
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    componentInstance_t* componentInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT);

    return componentInstance->componentID;
}

xme_core_componentType_t
xme_core_nodeMgr_compRep_getComponentType
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    componentInstance_t* componentInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, XME_CORE_COMPONENT_TYPE_INVALID);

    return componentInstance->componentType;
}

const char*
xme_core_nodeMgr_compRep_getInitializationString
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    componentInstance_t* componentInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, NULL);

    return componentInstance->initializationString;
}

xme_core_nodeMgr_compRep_state_t
xme_core_nodeMgr_compRep_getState
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    componentInstance_t* componentInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, XME_CORE_NODEMGR_COMPREP_STATE_INVALID);

    return componentInstance->state;
}

xme_core_nodeMgr_compRep_portHandle_t
xme_core_nodeMgr_compRep_getPort
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    uint16_t portTypeID
)
{
    componentInstance_t* componentInstance = NULL;
    portInstance_t* portInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE);

    portInstance = (portInstance_t*)
        xme_hal_singlyLinkedList_itemFromIndex(&(componentInstance->ports), portTypeID);

    XME_CHECK(NULL != portInstance, 0u);

    return portInstance->handle;
}

xme_core_dataManager_dataPacketId_t
xme_core_nodeMgr_compRep_getDataPacketID
(
    xme_core_nodeMgr_compRep_portHandle_t portHandle
)
{
    portInstance_t* portInstance = NULL;

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != portHandle, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);

    portInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(portInstanceTable, portHandle);

    XME_CHECK(NULL != portInstance, XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);

    return portInstance->dataPacketID;
}

uint16_t
xme_core_nodeMgr_compRep_getQueueSize
(
    xme_core_nodeMgr_compRep_portHandle_t portHandle
)
{
    portInstance_t* portInstance = NULL;

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != portHandle, 0u);

    portInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(portInstanceTable, portHandle);

    XME_CHECK(NULL != portInstance, 0u);

    return portInstance->queueSize;
}

xme_hal_time_timeInterval_t
xme_core_nodeMgr_compRep_getExecutionPeriod
(
    xme_core_nodeMgr_compRep_functionHandle_t functionHandle
)
{
    functionInstance_t* functionInstance = NULL;

    XME_CHECK(XME_HAL_TABLE_INVALID_ROW_HANDLE != functionHandle, 0u);

    functionInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(functionInstanceTable, functionHandle);

    XME_CHECK(NULL != functionInstance, 0u);

    return functionInstance->executionPeriod;
}

xme_core_nodeMgr_compRep_functionHandle_t
xme_core_nodeMgr_compRep_getFunction
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    uint16_t functionTypeID
)
{
    componentInstance_t* componentInstance = NULL;
    functionInstance_t* functionInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE);

    functionInstance = (functionInstance_t*)
        xme_hal_singlyLinkedList_itemFromIndex(&(componentInstance->functions), functionTypeID);

    XME_CHECK(NULL != functionInstance, XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE);

    return functionInstance->handle;
}

const xme_core_exec_functionDescriptor_t*
xme_core_nodeMgr_compRep_getFunctionDescriptor
(
    xme_core_nodeMgr_compRep_functionHandle_t functionHandle
)
{
    functionInstance_t* functionInstance = NULL;

    functionInstance = XME_HAL_TABLE_ITEM_FROM_HANDLE(functionInstanceTable, functionHandle);

    XME_CHECK(NULL != functionInstance, NULL);

    return &(functionInstance->functionDescriptor);
}

static void
xme_core_nodeMgr_compRep_setComponentIDInternal
(
    componentInstance_t* componentInstance,
    xme_core_component_t componentID
)
{
    componentInstance->componentID = componentID;

    {
        xme_hal_linkedList_index_t functionCount = 0u;
        xme_hal_linkedList_index_t functionIndex = 0u;

        functionCount = xme_hal_singlyLinkedList_getItemCount(&(componentInstance->functions));
        for (functionIndex = 0u; functionIndex < functionCount; functionIndex++)
        {
            functionInstance_t* functionInstance = (functionInstance_t*)
                xme_hal_singlyLinkedList_itemFromIndex(&(componentInstance->functions), functionIndex);
            XME_ASSERT_NORVAL(NULL != functionInstance);
            
            functionInstance->functionDescriptor.componentId = componentID;
        }
    }
}

void
xme_core_nodeMgr_compRep_setComponentID
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    xme_core_component_t componentID
)
{
    componentInstance_t* componentInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, XME_CHECK_RVAL_VOID);

    if (XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED != componentInstance->state && XME_CORE_NODEMGR_COMPREP_STATE_PREPARED != componentInstance->state)
    {
        XME_LOG(XME_LOG_ERROR, "[componentRepository] setComponentID(): Trying to set component ID in state %d which is not allowed.\n", componentInstance->state);
        return;
    }
    
    xme_core_nodeMgr_compRep_setComponentIDInternal(componentInstance, componentID);
}

void
xme_core_nodeMgr_compRep_setQueueSize
(
    xme_core_nodeMgr_compRep_portHandle_t portHandle,
    uint8_t queueSize
)
{
    portInstance_t* portInstance = NULL;

    portInstance = (portInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(portInstanceTable, portHandle);

    XME_CHECK(NULL != portInstance, XME_CHECK_RVAL_VOID);

    if (XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED != portInstance->componentInstance->state && XME_CORE_NODEMGR_COMPREP_STATE_PREPARED != portInstance->componentInstance->state)
    {
        XME_LOG(XME_LOG_ERROR, "[componentRepository] setQueueSize(): Trying to set queue size in state %d which is not allowed.\n", portInstance->componentInstance->state);
        return;
    }
    
    portInstance->queueSize = queueSize;
}

xme_status_t
xme_core_nodeMgr_compRep_setStateToAnnounced
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    componentInstance_t* componentInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, XME_STATUS_NOT_FOUND);

    XME_CHECK_MSG(XME_CORE_NODEMGR_COMPREP_STATE_PREPARED == componentInstance->state, XME_STATUS_INVALID_CONFIGURATION, XME_LOG_ERROR, "[componentRepository] setStateToAnnounced(): Must only be called when state is PREPARED (%d). Actual state of component (componentID = %d, componentType = %d) was: %d.\n", XME_CORE_NODEMGR_COMPREP_STATE_PREPARED, componentInstance->componentID, componentInstance->componentType, componentInstance->state);
    
    componentInstance->state = XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED;

    return XME_STATUS_SUCCESS;
}

xme_core_nodeMgr_compRep_componentHandle_t
xme_core_nodeMgr_compRep_getForeignComponentHandle
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
)
{
    componentInstance_t* componentInstance = NULL;

    componentInstance = (componentInstance_t*)
        XME_HAL_TABLE_ITEM_FROM_HANDLE(componentInstanceTable, componentHandle);

    XME_CHECK(NULL != componentInstance, XME_CORE_NODEMGR_COMPREP_INVALID_HANDLE);

    return componentInstance->foreignComponentHandle;
}

static bool
areEqualNodeIDs
(
    xme_core_node_nodeId_t nodeID1,
    xme_core_node_nodeId_t nodeID2
)
{
    if (nodeID1 == nodeID2)
    {
        return (bool) true;
    }

    if (XME_CORE_NODE_LOCAL_NODE_ID == nodeID1)
    {
        return (nodeID2 == xme_core_node_getCurrentNodeId());
    }

    if (XME_CORE_NODE_LOCAL_NODE_ID == nodeID2)
    {
        return (nodeID1 == xme_core_node_getCurrentNodeId());
    }

    return (bool) false;
}

/**
 * @}
 */
