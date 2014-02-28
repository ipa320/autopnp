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
 * $Id: interfaceTestInterface.cpp 4570 2013-08-06 10:57:02Z gulati $
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
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class InterfaceInterfaceTest: public ::testing::Test
{
protected:
    InterfaceInterfaceTest()
    : ip("192.168.150.121:33221")
    , port(33221)
    {
        // Initialize interfaceAddress with non-zeroes
        for (size_t i = 0; i < sizeof(interfaceAddress); i++)
        {
            ((char*) &interfaceAddress)[i] = i+1;
        }
    }

    virtual ~InterfaceInterfaceTest()
    {

    }

    const char* ip;
    const uint16_t port;
    xme_com_interface_address_t interfaceAddress;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     NoodeInterfaceTest                                                     //
//----------------------------------------------------------------------------//

TEST_F(InterfaceInterfaceTest, ipv4ToGenericWithValidIPv4AndNullInterface)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_com_interface_ipv4StringToGenericAddress(ip, NULL));
}

TEST_F(InterfaceInterfaceTest, ipv4ToGenericWithInvalidIPv4PortAndNullInterface)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_com_interface_ipv4StringToGenericAddress(NULL, NULL));
}

TEST_F(InterfaceInterfaceTest, ipv4ToGenericWithInvalidIPv4PortAndValidInterface)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, xme_com_interface_ipv4StringToGenericAddress(NULL, &interfaceAddress));
}

TEST_F(InterfaceInterfaceTest, ipv4ToGenericAndGenericToIPv4WithValidIPv4AndPortAndInterface)
{
    char newIP[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE + 2] = { 0x55 };
    uint32_t ipResult = 0x12345678U;
    uint16_t portResult = 0xABCDU;

    // Convert from IP/port in host byte order to generic address
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4StringToGenericAddress(ip, &interfaceAddress));
    ASSERT_EQ((uint16_t)XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, interfaceAddress.addressType);

    // Large buffer
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&interfaceAddress, newIP, sizeof(newIP)));
    EXPECT_STREQ("192.168.150.121:33221", newIP);

    // Buffer just large enough
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&interfaceAddress, newIP, XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE));
    EXPECT_STREQ("192.168.150.121:33221", newIP);

    // Too small buffer
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&interfaceAddress, newIP, XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE - 1));
    EXPECT_STREQ("192.168.150.121:3322", newIP);

    // Convert back to IP and port in network byte order
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipResult, &portResult));
    EXPECT_EQ(0x7996A8C0U, ipResult); // IP "192.168.150.12" in network byte order
    EXPECT_EQ(0xC581U, portResult); // Port number 33221 in network byte order

    // Invalidate address
    interfaceAddress.addressType = XME_COM_INTERFACE_ADDRESS_TYPE_IPV6;
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_com_interface_genericAddressToIPv4String(&interfaceAddress, newIP, sizeof(newIP)));
}

TEST_F(InterfaceInterfaceTest, ipv4BinaryToGenericAndGenericToIPv4WithValidIPv4AndPortAndInterface)
{
    char newIP[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE + 2] = { 0x55 };
    uint32_t ipResult = 0x12345678U;
    uint16_t portResult = 0xABCDU;

    // Convert from IP/port in host byte order to generic address
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_ipv4ToGenericAddress(192U, 168U, 150U, 121U, port, &interfaceAddress));
    ASSERT_EQ((uint16_t)XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, interfaceAddress.addressType);

    // Large buffer
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&interfaceAddress, newIP, sizeof(newIP)));
    EXPECT_STREQ("192.168.150.121:33221", newIP);

    // Buffer just large enough
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&interfaceAddress, newIP, XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE));
    EXPECT_STREQ("192.168.150.121:33221", newIP);

    // Too small buffer
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4String(&interfaceAddress, newIP, XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE - 1));
    EXPECT_STREQ("192.168.150.121:3322", newIP);

    // Convert back to IP and port in network byte order
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_com_interface_genericAddressToIPv4(&interfaceAddress, &ipResult, &portResult));
    EXPECT_EQ(0x7996A8C0U, ipResult); // IP "192.168.150.12" in network byte order
    EXPECT_EQ(0xC581U, portResult); // Port number 33221 in network byte order

    // Invalidate address
    interfaceAddress.addressType = XME_COM_INTERFACE_ADDRESS_TYPE_IPV6;
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_com_interface_genericAddressToIPv4String(&interfaceAddress, newIP, sizeof(newIP)));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
