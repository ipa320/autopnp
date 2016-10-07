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
 * $Id: interfaceTestPnPClient.cpp 7807 2014-03-13 10:18:07Z geisinger $
 */

/**
 * \file
 *         Plug and Play Client interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"

#include "xme/core/testUtils.h"

#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class PnPClientInterfaceTest: public xme::testing::Test
{
protected:
    PnPClientInterfaceTest()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpClient_init());
    }

    virtual ~PnPClientInterfaceTest()
    {
        xme_core_pnp_pnpClient_fini();
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

TEST_F(PnPClientInterfaceTest, processGraphWithNullIsShutdown)
{
#ifdef DEBUG
    xme_core_topic_pnpManager_runtime_graph_model_t graphModel;
    xme_hal_mem_set(&graphModel, 0, sizeof(graphModel));

    EXPECT_XME_ASSERTION_FAILURE(xme_core_pnp_pnpClient_processGraph(&graphModel, NULL));
#endif
}

TEST_F(PnPClientInterfaceTest, processGraphWithNullGraph)
{
    char isShutdownDesired = false;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpClient_processGraph(NULL, &isShutdownDesired));
    EXPECT_FALSE(isShutdownDesired);

    isShutdownDesired = true;
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_pnp_pnpClient_processGraph(NULL, &isShutdownDesired));
    EXPECT_FALSE(isShutdownDesired);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
