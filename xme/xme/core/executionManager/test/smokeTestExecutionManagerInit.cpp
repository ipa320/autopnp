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
 * $Id: smokeTestExecutionManagerInit.cpp 4254 2013-07-17 13:13:22Z geisinger $
 */

/**
 * \file
 *         Execution Manager smoke tests (init).
 */


#include <gtest/gtest.h>
#include "testFunctionsExecutionManager.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"

#include "xme/core/component.h"
#include "xme/hal/include/sync.h"
#include "xme/hal/include/sched.h"

/*****************************************************************************/
/***        "int main()" interface                                         ***/
/*****************************************************************************/
// Normal init/fini
TEST(ExecutionManagerSmokeTestInitFini, InitFini)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_init(&testConfig));
    ASSERT_EQ(XME_STATUS_ALREADY_EXIST, xme_core_exec_init(&testConfig));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_exec_fini());
    ASSERT_EQ(XME_STATUS_UNEXPECTED, xme_core_exec_fini());
    ASSERT_EQ(XME_STATUS_UNEXPECTED, xme_core_exec_init(&testConfig));
}

