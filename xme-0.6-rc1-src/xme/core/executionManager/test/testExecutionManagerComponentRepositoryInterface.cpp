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
 * $Id: testExecutionManagerComponentRepositoryInterface.cpp 4430 2013-07-31 08:01:56Z rupanov $
 */

/**
 * \file
 *         Execution Manager component repository interface tests.
 */

#include <gtest/gtest.h>
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/test/testHelper_executionManager.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/defines.h"


class ExecutionManagerComponentRepositoryInterfaceTest : public ::testing::Test {
 protected:
  virtual void SetUp(void)
  {
     EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_init());
  }

  virtual void TearDown(void)
  {
      EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_fini());
  }


};

TEST_F(ExecutionManagerComponentRepositoryInterfaceTest, InitFiniBasic)
{
    EXPECT_EQ(true, true);
}

TEST_F(ExecutionManagerComponentRepositoryInterfaceTest, GetComponent)
{
    xme_core_exec_componentDescriptor_t* components[1];

    xme_core_exec_componentDescriptor_t* tmpComp;

    /* set up components */
    components[0] = createHelper((xme_core_component_t)42, (xme_core_component_functionId_t)1, 1000);

    /* register the component */
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[0]));

    /* normal getComponent */
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_getComponent((xme_core_component_t)42, &tmpComp));
    EXPECT_EQ(components[0], tmpComp);

    /* non-existing component */
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_exec_componentRepository_getComponent((xme_core_component_t)1, &tmpComp));

    /* invalid id */
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_exec_componentRepository_getComponent(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, &tmpComp));
}


TEST_F(ExecutionManagerComponentRepositoryInterfaceTest, GetComponentFunction)
{
    xme_core_exec_componentDescriptor_t* components[3];
    xme_core_exec_functionDescriptor_t* functions[2];
    xme_core_exec_functionDescriptor_t* functions2[2];

    xme_core_exec_functionDescriptor_t* tmpFun;


    xme_core_component_functionId_t functionIds[2] = {(xme_core_component_functionId_t)1, (xme_core_component_functionId_t)5};

    /* set up components */
    components[0] = createHelper((xme_core_component_t)42, functionIds[0], 1000);
    EXPECT_TRUE(NULL != components[0]);

    functions[0] = (xme_core_exec_functionDescriptor_t*)xme_hal_singlyLinkedList_itemFromIndex(&(components[0]->functions), 0);
    EXPECT_TRUE(NULL != functions[0]);

    components[1] = createHelper((xme_core_component_t)73, functionIds[1], 1000);
    EXPECT_TRUE(NULL != components[1]);

    functions[1] = (xme_core_exec_functionDescriptor_t*)xme_hal_singlyLinkedList_itemFromIndex(&(components[1]->functions), 0);
    EXPECT_TRUE(NULL != functions[1]);

    /* unregistered component */
    components[2] = createHelper((xme_core_component_t)33, functionIds[1], 1000);


    /* set up components */
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[0]));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[1]));

    /* corner cases */
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_exec_componentRepository_getComponentFunction(NULL,functionIds[1], &tmpFun));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_exec_componentRepository_getComponentFunction(components[0], XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, &tmpFun));

    /* normal get */
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_getComponentFunction(components[0], functionIds[0], &(functions2[0])));
    EXPECT_EQ(functions[0], functions2[0]);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_getComponentFunction(components[1], functionIds[1], &(functions2[1])));
    EXPECT_EQ(functions[1], functions2[1]);

    /* unregistered component: still valid */
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_getComponentFunction(components[2], functionIds[1], &tmpFun));
    EXPECT_TRUE(NULL != tmpFun);

    /* function not there */
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_exec_componentRepository_getComponentFunction(components[0], (xme_core_component_functionId_t)9, &tmpFun));
}


TEST_F(ExecutionManagerComponentRepositoryInterfaceTest, GetFunction)
{
    xme_core_exec_componentDescriptor_t* components[3];
    xme_core_component_t componentIds[3] = {(xme_core_component_t)42, (xme_core_component_t)73, (xme_core_component_t)33};
    xme_core_exec_functionDescriptor_t* functions[2];
    xme_core_exec_functionDescriptor_t* functions2[2];
    xme_core_component_functionId_t functionIds[2] = {(xme_core_component_functionId_t)1, (xme_core_component_functionId_t)5};

    xme_core_exec_functionDescriptor_t* tmpFun;

    /* set up components */
    components[0] = createHelper(componentIds[0], functionIds[0], 1000);
    EXPECT_TRUE(NULL != components[0]);

    functions[0] = (xme_core_exec_functionDescriptor_t*)xme_hal_singlyLinkedList_itemFromIndex(&(components[0]->functions), 0);
    EXPECT_TRUE(NULL != functions[0]);

    components[1] = createHelper(componentIds[1], functionIds[1], 1000);
    EXPECT_TRUE(NULL != components[1]);

    functions[1] = (xme_core_exec_functionDescriptor_t*)xme_hal_singlyLinkedList_itemFromIndex(&(components[1]->functions), 0);
    EXPECT_TRUE(NULL != functions[1]);

    /* unregistered component */
    components[2] = createHelper(componentIds[2], functionIds[1], 1000);

    /* set up components */
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[0]));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[1]));

    /* corner cases */
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_exec_componentRepository_getFunction(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT,functionIds[1], &tmpFun));
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_exec_componentRepository_getFunction(componentIds[0], XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, &tmpFun));

    /* normal get */
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_getFunction(componentIds[0], functionIds[0], &(functions2[0])));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_getFunction(componentIds[1], functionIds[1], &(functions2[1])));
    EXPECT_EQ(functions[0], functions2[0]);
    EXPECT_EQ(functions[1], functions2[1]);

    /* unregistered component: now invalid */
    EXPECT_EQ(XME_STATUS_NO_SUCH_VALUE, xme_core_exec_componentRepository_getFunction(componentIds[2], functionIds[1], &tmpFun));

    /* function not there */
    EXPECT_EQ(XME_STATUS_NOT_FOUND, xme_core_exec_componentRepository_getFunction(componentIds[0], (xme_core_component_functionId_t)9, &tmpFun));
}

TEST_F(ExecutionManagerComponentRepositoryInterfaceTest, RegisterSameComponentTwice)
{
    xme_core_exec_componentDescriptor_t* components[2];

    /* normal component */
    components[0] = createHelper((xme_core_component_t)1, (xme_core_component_functionId_t)1, 1000);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[0]));

    /* normal component #2 - same id */
    components[1] = createHelper((xme_core_component_t)1, (xme_core_component_functionId_t)1, 3000);
    EXPECT_NE(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[1]));
}

TEST_F(ExecutionManagerComponentRepositoryInterfaceTest, RegisterComponent)
{
    xme_core_exec_componentDescriptor_t* components[10];
    /* normal component */
    components[0] = createHelper((xme_core_component_t)1, (xme_core_component_functionId_t)1, 1000);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[0]));

    EXPECT_NE(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(NULL));

    /* invalid component id */
    components[1] = createHelper(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, (xme_core_component_functionId_t)1, 1000);
    EXPECT_NE(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[1]));

    /* invalid function id */
    components[2] = createHelper((xme_core_component_t)2, XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT, 1000);
    EXPECT_NE(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[0]));

    /* functions list empty */
    components[4] = createHelper((xme_core_component_t)4, (xme_core_component_functionId_t)1, 1000);
    XME_HAL_SINGLYLINKEDLIST_INIT(components[4]->functions);
    EXPECT_NE(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[4]));

    /* screwed up function descriptor: no wcet */
    components[5] = createHelper((xme_core_component_t)5, (xme_core_component_functionId_t)1, 1000);
    ((xme_core_exec_functionDescriptor_t*) XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(components[5]->functions, 0))->wcet_ns = 0;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[5]));

    /* screwed up function descriptor: task=NULL */
    components[6] = createHelper((xme_core_component_t)6, (xme_core_component_functionId_t)1, 1000);
    ((xme_core_exec_functionDescriptor_t*) XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(components[6]->functions, 0))->task = NULL;
    EXPECT_NE(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[6]));

    /* screwed up function descriptor: invalid function ID */
    components[7] = createHelper((xme_core_component_t)7, (xme_core_component_functionId_t)1, 1000);
    ((xme_core_exec_functionDescriptor_t*) XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(components[7]->functions, 0))->functionId = XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT;
    EXPECT_NE(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[7]));

    /* screwed up function descriptor: NULL in the list */
    components[8] = createHelper((xme_core_component_t)8, (xme_core_component_functionId_t)1, 1000);
    XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(components[8]->functions, NULL);
    EXPECT_NE(XME_STATUS_SUCCESS, xme_core_exec_componentRepository_registerComponent(components[8]));

}
