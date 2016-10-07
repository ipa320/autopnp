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
 * $Id: smokeTestInterface.cpp 4570 2013-08-06 10:57:02Z gulati $
 */

/**
 * \file
 *         Node Interface smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/com/interface.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class InterfaceSmokeTest: public ::testing::Test
{
protected:
    InterfaceSmokeTest()
    {
    }

    virtual ~InterfaceSmokeTest()
    {
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     InterfaceSmokeTest                                                     //
//----------------------------------------------------------------------------//

TEST_F(InterfaceSmokeTest, ipv4StringBufferSize)
{
    const char ipv4_1[] = "192.168.0.1:12345";
    const char ipv4_2[] = "255.255.255.255:65535";

    EXPECT_GT(XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE, sizeof(ipv4_1));
    EXPECT_EQ(XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE, sizeof(ipv4_2));
}

TEST_F(InterfaceSmokeTest, NullIPv4NullInterface_IPv4ToGeneric)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_com_interface_ipv4StringToGenericAddress(NULL, NULL));
}

TEST_F(InterfaceSmokeTest, NullIPv4ZeroSizeNullInterface_genericAddressToIPv4String)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_com_interface_genericAddressToIPv4String(NULL, NULL, 0));
}

TEST_F(InterfaceSmokeTest, ZeroIpZeroPortNullInterface_ipv4TogenericAddress)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_com_interface_ipv4ToGenericAddress(0, 0, 0, 0, 0, NULL));
}

TEST_F(InterfaceSmokeTest, NullIPv4ZeroSizeNullInterface_genericAddressToIPv4)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_com_interface_genericAddressToIPv4(NULL, NULL, NULL));
}
/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
