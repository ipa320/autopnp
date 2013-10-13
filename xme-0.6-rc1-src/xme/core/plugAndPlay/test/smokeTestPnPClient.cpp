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
 * $Id: smokeTestPnPClient.cpp 4070 2013-07-08 15:51:07Z ruiz $
 */

/**
 * \file
 *         Plug and Play Client integration tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"

#include "xme/defines.h"
#include "xme/core/node.h"
#include "xme/core/topic.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class PnPClientSmokeTest: public ::testing::Test
{

};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
