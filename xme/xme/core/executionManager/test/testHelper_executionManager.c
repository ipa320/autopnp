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
 * $Id: testHelper_executionManager.c 6459 2014-01-23 16:50:54Z geisinger $
 */

#define MODULE_ACRONYM "mHelperMod: "
#include "xme/core/executionManager/test/testHelper_executionManager.h"
#include "xme/core/executionManager/test/mHelperApplication/include/mHelperApplicationFunctionWrapper.h"
#include "xme/core/executionManager/test/mHelperApplication/include/mHelperApplicationFunction.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/executionManager/include/internDescriptorTable.h"
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/broker/include/broker.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"

#include "xme/core/component.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/sync.h"
#include "xme/hal/include/tls.h"
#include "xme/hal/include/context.h"
#include "xme/hal/include/sched.h"
#include "xme/defines.h"

/* Counters to check how many executions have happened */
uint32_t executionCount[N_FUNCTIONS_SIMULATED] = {0};
uint32_t cycleEndEventCount = 0;

/* Pointers to callbacks for registered functions */
xme_core_exec_initCallback_t functionPointers[N_FUNCTIONS_SIMULATED];

xme_status_t onCycleEnd ( void* );
xme_core_exec_configStruct_t testConfig = {&(onCycleEnd)};

xme_status_t onCycleEnd ( void* param )
{
    XME_UNUSED_PARAMETER(param);

    return XME_STATUS_SUCCESS;
}

void
preinitXme(void)
{
    xme_core_broker_initStruct_t brokerParams;

    xme_hal_sync_init();
    xme_hal_tls_init();
    xme_hal_context_init();
    xme_hal_sched_init();

    xme_core_broker_init(&brokerParams);
    //xme_core_dataHandler_init(15);
    xme_core_dataHandler_init();
}

xme_core_exec_componentDescriptor_t*
createHelper
(
    xme_core_component_t                cid,
    xme_core_component_functionId_t     fid,
    xme_hal_time_timeInterval_t         wcet
)
{
    xme_core_exec_componentDescriptor_t* component1;
    xme_core_exec_functionDescriptor_t* fun11;

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "registering a component\n");

    /* Compoment descriptor init */
    component1 = (xme_core_exec_componentDescriptor_t*) xme_hal_mem_alloc(
            sizeof(xme_core_exec_componentDescriptor_t));
    if ((void *) 0 == component1)
        return NULL;
    component1->componentId = cid;
    component1->init = (xme_core_exec_initCallback_t) NULL;
    component1->fini = (xme_core_exec_finiCallback_t) NULL;
    component1->initParam = (void *) 0;
    component1->initWcet_ns = 0 * ((xme_hal_time_timeInterval_t) 1000);
    XME_HAL_SINGLYLINKEDLIST_INIT(component1->functions);

    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "Component ID = %d\n", cid);

    /* Function descriptor init */
    fun11 = (xme_core_exec_functionDescriptor_t*) xme_hal_mem_alloc(
            sizeof(xme_core_exec_functionDescriptor_t));

    if ((void *) 0 == fun11)
        return NULL;

    fun11->componentId = component1->componentId;
    fun11->functionId = fid;
    fun11->task = &test_mHelperApplication_mHelperApplicationFunctionWrapper_execute;
    fun11->taskArgs = fun11;
    fun11->wcet_ns = wcet;
    fun11->init = &test_mHelperApplication_mHelperApplicationFunction_init;
    fun11->fini = &test_mHelperApplication_mHelperApplicationFunction_fini;
    fun11->initWcet_ns = 0;

    if (!(XME_STATUS_SUCCESS
            == xme_hal_singlyLinkedList_addItem(
                    (void*) &(component1->functions), fun11 )))
    {
        return NULL;
    }
    return component1;
}


/**
 * \brief   A function to create a "helper function" instance with specified
 *          CID, FID.
 *     odo    Variant ID will be fused in to testHelper later.
 */
xme_status_t
createHelperInstance
(
    xme_core_component_t                cid,
    xme_core_component_functionId_t     fid,
    xme_hal_time_timeInterval_t         wcet
)
{
    xme_core_exec_componentDescriptor_t* component1;
    xme_core_exec_functionDescriptor_t* function1;
    xme_hal_sched_taskHandle_t task = XME_HAL_SCHED_INVALID_TASK_HANDLE;

    XME_ASSERT(cid+fid < N_FUNCTIONS_SIMULATED);
    XME_ASSERT(cid > 0);
    XME_ASSERT(fid > 0);
    XME_ASSERT(wcet > 0);

    component1= createHelper(cid, fid, wcet);

    /* Register component with all functions */
    xme_core_exec_componentRepository_registerComponent(component1);

    /* Initialize the registered component and create a function exec unit*/
    if (NULL != component1->initParam)
    {
        XME_CHECK_MSG(
                XME_STATUS_SUCCESS == component1->init(component1->initParam),
                XME_STATUS_INTERNAL_ERROR, XME_LOG_FATAL,
                MODULE_ACRONYM "could not initialize component %d\n",
                component1->componentId);
    }

    function1 = (xme_core_exec_functionDescriptor_t*)
    xme_hal_singlyLinkedList_itemFromIndex(
                                &component1->functions,
                                0
                        );

    /* Create an execution unit for each function */
    if (XME_STATUS_NOT_FOUND
            == xme_core_exec_dispatcher_getRunnable(function1->componentId, function1->functionId, &task))
    {
        XME_CHECK_MSG(
            XME_STATUS_SUCCESS ==
                xme_core_exec_dispatcher_createFunctionExecutionUnit(
                        xme_hal_singlyLinkedList_itemFromIndex(&(component1->functions), 0),
                        false),
            XME_STATUS_INTERNAL_ERROR,
            XME_LOG_FATAL,
            MODULE_ACRONYM "error creating testHelper execution unit!\n");
    }

    return XME_STATUS_SUCCESS;
}


/**
 *  \brief  Returns execution count for each function
 */
uint32_t getExecutionCount
(
    xme_core_component_t cid,
    xme_core_component_functionId_t fid
)
{
    return executionCount[cid+fid];
}


void clearExecutionCounters( void )
{
    int i;
    for(i=0; i<N_FUNCTIONS_SIMULATED; i++)
        executionCount[i]=0;
}
