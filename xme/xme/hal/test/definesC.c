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
 * $Id: definesC.c 4311 2013-07-22 17:22:54Z geisinger $
 */

/**
 * \file
 *         A function to test SFINIT() in C (not C++) mode.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <float.h>
#include <math.h>
#include <stdbool.h>

#include "xme/defines.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
typedef struct
{
    bool b1;
    char c2;
    int i3;
    double d4;
    struct
    {
        int x;
        int y;
    }
    s5;
    long l6;
    union
    {
        double u1;
        long u2;
    } n7;
    unsigned int u8;
}
testStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  SFINIT() macro tests.
 *
 * \return Returns 0 if all tests passed. Otherwise, returns the one-based
 *         index of the test that failed.
 */
unsigned int
testDesignedStructFieldInitializer(void);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
unsigned int
testDesignedStructFieldInitializer(void)
{
    unsigned int index = 0;

    testStruct_t testStruct =
    {
        SFINIT(b1, true),
        SFINIT(c2, 'A'),
        SFINIT(i3, 0765),
        SFINIT(d4, 3.1415),
        SFINIT(s5, { SFINIT(x, -123), SFINIT(y, 456) }),
        SFINIT(l6, 0xABCDEF),
        SFINIT(n7.u1, 1.23456),
        SFINIT(u8, 0xABCDEF)
    };

    static testStruct_t staticTestStruct =
    {
        SFINIT(b1, true),
        SFINIT(c2, 'A'),
        SFINIT(i3, 0765),
        SFINIT(d4, 3.1415),
        SFINIT(s5, { SFINIT(x, -123), SFINIT(y, 456) }),
        SFINIT(l6, 0xABCDEF),
        SFINIT(n7.u1, 1.23456),
    };

    XME_CHECK(testStruct.b1, ++index);
    XME_CHECK('A' == testStruct.c2, ++index);
    XME_CHECK(0765 == testStruct.i3, ++index);
    XME_CHECK(fabs(3.1415 - testStruct.d4) <= DBL_EPSILON, ++index);
    XME_CHECK(-123 == testStruct.s5.x, ++index);
    XME_CHECK(456 == testStruct.s5.y, ++index);
    XME_CHECK(0xABCDEF == testStruct.l6, ++index);
    XME_CHECK(fabs(1.23456 - testStruct.n7.u1) < DBL_EPSILON, ++index);
    XME_CHECK(0xABCDEF == testStruct.u8, ++index);

    XME_CHECK(staticTestStruct.b1, ++index);
    XME_CHECK('A' == staticTestStruct.c2, ++index);
    XME_CHECK(0765 == staticTestStruct.i3, ++index);
    XME_CHECK(fabs(3.1415 - staticTestStruct.d4) <= DBL_EPSILON, ++index);
    XME_CHECK(-123 == staticTestStruct.s5.x, ++index);
    XME_CHECK(456 == staticTestStruct.s5.y, ++index);
    XME_CHECK(0xABCDEF == staticTestStruct.l6, ++index);
    XME_CHECK(fabs(1.23456 - staticTestStruct.n7.u1) <= DBL_EPSILON, ++index);
    XME_CHECK(0 == staticTestStruct.u8, ++index);

    return 0;
}
