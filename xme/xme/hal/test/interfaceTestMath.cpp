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
 * $Id: interfaceTestMath.cpp 2895 2013-04-12 18:04:27Z geisinger $
 */

/**
 * \file
 *         Math interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
// Ensure that stdint.h defines limit macros
#define __STDC_LIMIT_MACROS

#include <gtest/gtest.h>

#include "xme/hal/include/math.h"

#include <stdint.h>

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class MathInterfaceTest: public ::testing::Test
{
protected:
    MathInterfaceTest()
    {
    }

    virtual ~MathInterfaceTest()
    {
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     MathInterfaceTest                                                      //
//----------------------------------------------------------------------------//

TEST_F(MathInterfaceTest, isPowerOf2With8Bits)
{
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-8, 8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-4, 8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-2, 8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-1, 8));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 0, 8));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 1, 8));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 2, 8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 3, 8));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 4, 8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 5, 8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 6, 8));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 8, 8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(12, 8));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(16, 8));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX,   8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX+1, 8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX+2, 8));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX,   8));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX+1, 8));
    // Special case: casting 32769 to 8bit yields 1, which is a power-of-2
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX+2, 8));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX,   8));
//    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX+1, 8));
//    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX+2, 8));
}

TEST_F(MathInterfaceTest, isPowerOf2With16Bits)
{
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-8, 16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-4, 16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-2, 16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-1, 16));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 0, 16));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 1, 16));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 2, 16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 3, 16));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 4, 16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 5, 16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 6, 16));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 8, 16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(12, 16));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(16, 16));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX,   16));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX+1, 16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX+2, 16));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX,   16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX+1, 16));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX+2, 16));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX,   16));
//    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX+1, 16));
//    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX+2, 16));
}

TEST_F(MathInterfaceTest, isPowerOf2With32Bits)
{
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-8, 32));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-4, 32));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-2, 32));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-1, 32));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 0, 32));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 1, 32));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 2, 32));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 3, 32));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 4, 32));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 5, 32));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 6, 32));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 8, 32));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(12, 32));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(16, 32));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX,   32));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX+1, 32));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX+2, 32));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX,   32));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX+1, 32));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX+2, 32));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX,   32));
//    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX+1, 32));
//    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX+2, 32));
}

TEST_F(MathInterfaceTest, isPowerOf2With64Bits)
{
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-8, 64));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-4, 64));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-2, 64));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(-1, 64));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 0, 64));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 1, 64));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 2, 64));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 3, 64));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 4, 64));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 5, 64));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2( 6, 64));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2( 8, 64));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(12, 64));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(16, 64));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX,   64));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX+1, 64));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT8_MAX+2, 64));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX,   64));
    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX+1, 64));
    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT16_MAX+2, 64));

    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX,   64));
//    EXPECT_TRUE (XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX+1, 64));
//    EXPECT_FALSE(XME_HAL_MATH_IS_POWER_OF_2(INT32_MAX+2, 64));
}

TEST_F(MathInterfaceTest, min)
{
    EXPECT_EQ(0, XME_HAL_MATH_MIN(0, 0));
    EXPECT_EQ(1, XME_HAL_MATH_MIN(1, 1));
    EXPECT_EQ(1, XME_HAL_MATH_MIN(1, 2));
    EXPECT_EQ(1, XME_HAL_MATH_MIN(2, 1));
    EXPECT_EQ(-1, XME_HAL_MATH_MIN(-1, 2));
    EXPECT_DOUBLE_EQ(1.5, XME_HAL_MATH_MIN(1.5, 2.3));
    EXPECT_DOUBLE_EQ(-1.5, XME_HAL_MATH_MIN(-1.5, 2.3));
    EXPECT_EQ(5, 2 + XME_HAL_MATH_MIN(3, 4));
    EXPECT_EQ(8, XME_HAL_MATH_MIN(3, 4) + 5);
    EXPECT_DOUBLE_EQ(-1.6, XME_HAL_MATH_MIN(-1.5, XME_HAL_MATH_MIN(-1.6, 2.3)));
}

TEST_F(MathInterfaceTest, max)
{
    EXPECT_EQ(0, XME_HAL_MATH_MAX(0, 0));
    EXPECT_EQ(1, XME_HAL_MATH_MAX(1, 1));
    EXPECT_EQ(2, XME_HAL_MATH_MAX(1, 2));
    EXPECT_EQ(2, XME_HAL_MATH_MAX(2, 1));
    EXPECT_EQ(2, XME_HAL_MATH_MAX(-1, 2));
    EXPECT_DOUBLE_EQ(2.3, XME_HAL_MATH_MAX(1.5, 2.3));
    EXPECT_DOUBLE_EQ(2.3, XME_HAL_MATH_MAX(-1.5, 2.3));
    EXPECT_EQ(6, 2 + XME_HAL_MATH_MAX(3, 4));
    EXPECT_EQ(9, XME_HAL_MATH_MAX(3, 4) + 5);
    EXPECT_DOUBLE_EQ(2.3, XME_HAL_MATH_MAX(-1.5, XME_HAL_MATH_MAX(1.6, 2.3)));
}

TEST_F(MathInterfaceTest, ceilPowerofTwo)
{
    EXPECT_EQ((uint32_t)0, xme_hal_math_ceilPowerOfTwo(0));
    EXPECT_EQ((uint32_t)1, xme_hal_math_ceilPowerOfTwo(1));
    EXPECT_EQ((uint32_t)2, xme_hal_math_ceilPowerOfTwo(2));
    EXPECT_EQ((uint32_t)4, xme_hal_math_ceilPowerOfTwo(3));
    EXPECT_EQ((uint32_t)4, xme_hal_math_ceilPowerOfTwo(4));
    EXPECT_EQ((uint32_t)8, xme_hal_math_ceilPowerOfTwo(5));
    EXPECT_EQ((uint32_t)8, xme_hal_math_ceilPowerOfTwo(7));
    EXPECT_EQ((uint32_t)8, xme_hal_math_ceilPowerOfTwo(8));
    EXPECT_EQ((uint32_t)2048, xme_hal_math_ceilPowerOfTwo(1025));
    EXPECT_EQ((uint32_t)16384, xme_hal_math_ceilPowerOfTwo(16384));
    EXPECT_EQ((uint32_t)262144, xme_hal_math_ceilPowerOfTwo(262143));
    EXPECT_EQ((uint32_t)2147483648u, xme_hal_math_ceilPowerOfTwo(1073741825));
    EXPECT_EQ((uint32_t)2147483648u, xme_hal_math_ceilPowerOfTwo(2147483647));
    EXPECT_EQ((uint32_t)2147483648u, xme_hal_math_ceilPowerOfTwo(2147483648u));
    EXPECT_EQ((uint32_t)0, xme_hal_math_ceilPowerOfTwo(2147483649u));
}

TEST_F(MathInterfaceTest, floorPowerofTwo)
{
    EXPECT_EQ((uint32_t)0, xme_hal_math_floorPowerOfTwo(0));
    EXPECT_EQ((uint32_t)1, xme_hal_math_floorPowerOfTwo(1));
    EXPECT_EQ((uint32_t)2, xme_hal_math_floorPowerOfTwo(2));
    EXPECT_EQ((uint32_t)2, xme_hal_math_floorPowerOfTwo(3));
    EXPECT_EQ((uint32_t)4, xme_hal_math_floorPowerOfTwo(4));
    EXPECT_EQ((uint32_t)4, xme_hal_math_floorPowerOfTwo(5));
    EXPECT_EQ((uint32_t)4, xme_hal_math_floorPowerOfTwo(7));
    EXPECT_EQ((uint32_t)8, xme_hal_math_floorPowerOfTwo(8));
    EXPECT_EQ((uint32_t)512, xme_hal_math_floorPowerOfTwo(1023));
    EXPECT_EQ((uint32_t)16384, xme_hal_math_floorPowerOfTwo(16384));
    EXPECT_EQ((uint32_t)262144, xme_hal_math_floorPowerOfTwo(262145));
    EXPECT_EQ((uint32_t)1073741824, xme_hal_math_floorPowerOfTwo(1073741825));
    EXPECT_EQ((uint32_t)1073741824, xme_hal_math_floorPowerOfTwo(2147483647));
    EXPECT_EQ((uint32_t)2147483648u, xme_hal_math_floorPowerOfTwo(2147483648u));
    EXPECT_EQ((uint32_t)2147483648u, xme_hal_math_floorPowerOfTwo(2147483649u));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
