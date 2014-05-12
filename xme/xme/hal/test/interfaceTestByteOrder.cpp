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
 * $Id: interfaceTestByteOrder.cpp 4686 2013-08-13 15:52:13Z kukreja $
 */

/**
 * \file
 *         Byte Order interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/byteOrder.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class ByteOrderInterfaceTest: public ::testing::Test
{
protected:
    ByteOrderInterfaceTest()
    :hostshort(0x1234)                  //4660
    ,netshort(0x3412)                   //13330
    ,hostlong(0x1234ABCD)               //305441741
    ,netlong(0xCDAB3412)                //3450549266
    ,hostlonglong(0x12AB34CD56AB78CD)   //1345226970227243213
    ,netlonglong(0xCD78AB56CD34AB12)    //14805772164278823698
    {
    }

    virtual ~ByteOrderInterfaceTest()
    {    
    }

    uint16_t hostshort;
    uint16_t netshort;
    uint32_t hostlong;
    uint32_t netlong;
    uint64_t hostlonglong;
    uint64_t netlonglong;	
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     byteOrderInterfaceTest                                                 //
//----------------------------------------------------------------------------//

TEST_F(ByteOrderInterfaceTest, conversionBetweenHostAndNetworkFormats)
{
    EXPECT_EQ(netshort, xme_hal_byteOrder_htons(hostshort));
    EXPECT_EQ(hostshort, xme_hal_byteOrder_ntohs(netshort));
    EXPECT_EQ(netlong, xme_hal_byteOrder_htonl(hostlong));
    EXPECT_EQ(hostlong, xme_hal_byteOrder_ntohl(netlong));
    EXPECT_EQ(netlonglong, xme_hal_byteOrder_htonll(hostlonglong));
    EXPECT_EQ(hostlonglong, xme_hal_byteOrder_ntohll(netlonglong));
}

TEST_F(ByteOrderInterfaceTest, checkSystemByteOrder)
{
#if defined(_MSC_VER)
    #if defined(_M_IX86)
        EXPECT_EQ(XME_HAL_BYTEORDER_LITTLE_ENDIAN, xme_hal_byteOrder_getHostByteOrder());
    #elif defined(_M_AMD64)
        EXPECT_EQ(XME_HAL_BYTEORDER_LITTLE_ENDIAN, xme_hal_byteOrder_getHostByteOrder());
    #else
        #error "Unsupported Processor Architecure"
    #endif
#elif defined(__GNUC__)
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        EXPECT_EQ(XME_HAL_BYTEORDER_LITTLE_ENDIAN, xme_hal_byteOrder_getHostByteOrder());
    #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        EXPECT_EQ(XME_HAL_BYTEORDER_BIG_ENDIAN, xme_hal_byteOrder_getHostByteOrder());
    #elif __BYTE_ORDER__ == __ORDER_PDP_ENDIAN__
        EXPECT_EQ(XME_HAL_BYTEORDER_PDP_ENDIAN, xme_hal_byteOrder_getHostByteOrder());
    #else
        #error "Unsupported byte order!"
    #endif
#else
    #error "Unknown compiler, please add tests here!"
#endif
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
