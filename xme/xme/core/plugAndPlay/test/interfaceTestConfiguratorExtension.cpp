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
 * $Id: interfaceTestConfiguratorExtension.cpp 5838 2013-11-18 16:48:51Z wiesmueller $
 */

/**
 * \file
 *         Configurator Extension interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/plugAndPlay/include/configuratorExtension.h"

#include "xme/core/plugAndPlay/include/plugAndPlayManager.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
class ConfigExtInterfaceTest: public ::testing::Test
{
protected:
    ConfigExtInterfaceTest()
    {

    }

    virtual ~ConfigExtInterfaceTest()
    {

    }

    virtual void SetUp()
    {
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_pnpManager_init(NULL));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_pnp_configExt_init());
    }

    virtual void TearDown()
    {
        xme_core_pnp_configExt_fini();
        xme_core_pnp_pnpManager_fini();
    }

};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Dummy callback, does nothing.
 */
void
callback(xme_hal_graph_graph_t* const configGraph)
{
    XME_UNUSED_PARAMETER(configGraph);

    // Do nothing
}

XME_EXTERN_C_END

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
TEST_F(ConfigExtInterfaceTest, addConfigurator)
{
    xme_core_pnp_configExt_configuratorHandle_t handle1;
    xme_core_pnp_configExt_configuratorHandle_t handle2;

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, NULL, NULL));

    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, NULL, &handle1));

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callback, NULL));

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callback, &handle1));

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callback, &handle2));

    EXPECT_NE(handle1, handle2);
}

TEST_F(ConfigExtInterfaceTest, removeConfigurator)
{
    xme_core_pnp_configExt_configuratorHandle_t handle = (xme_core_pnp_configExt_configuratorHandle_t) 1;

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE,
              xme_core_pnp_configExt_removeConfigurator(XME_HAL_TABLE_INVALID_ROW_HANDLE));

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE,
              xme_core_pnp_configExt_removeConfigurator(handle));

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_pnp_configExt_addConfigurator(XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES, callback, &handle));

    EXPECT_EQ(XME_STATUS_SUCCESS,
              xme_core_pnp_configExt_removeConfigurator(handle));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
