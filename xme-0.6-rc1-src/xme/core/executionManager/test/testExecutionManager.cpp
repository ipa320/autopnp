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
 * $Id: testExecutionManager.cpp 4597 2013-08-07 14:18:28Z ruiz $
 */

/**
 * \file
 *         Execution Manager test entry point.
 */

#include <gtest/gtest.h>
#include "testHelper_executionManager.h"
#include "xme/core/dataHandler/include/dataHandler.h"

#include "xme/hal/include/sched.h"
#include "xme/hal/include/sync.h"
#include "xme/hal/include/tls.h"

int main(int argc, char **argv)
{
    int result = -1;
    ::testing::InitGoogleTest(&argc, argv);

    preinitXme();

    result = RUN_ALL_TESTS();
    return result;
}
