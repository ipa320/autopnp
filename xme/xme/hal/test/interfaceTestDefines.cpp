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
 * $Id: interfaceTestDefines.cpp 5045 2013-09-11 17:14:43Z geisinger $
 */

/**
 * \file
 *         Defines interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/defines.h"
#include "xme/core/log.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/*!
 * \def    XME_ASSERT_SIDE_EFFECT_TEST
 *
 * \brief  Returns a nonzero value only if the given condition might contain a
 *         side effect as determined by a heutristic analysis of the condition.
 * 
 * \param  condition Condition to test. Must yield a Boolean value.
 *
 * \return Returns a nonzero value if the given condition might contain a side
 *         effect that is not suppressed by the use of the
 *         XME_ASSERT_NO_SIDE_EFFECTS() macro and zero otherwise.
 */
#define XME_ASSERT_SIDE_EFFECT_TEST(condition) \
    xme_assert_checkForSideEffects("XME_ASSERT_TEST", #condition, __FILE__, __LINE__, (condition) ? 0 : 0)

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class DefinesInterfaceTest: public ::testing::Test
{
protected:
    DefinesInterfaceTest()
    : a(1)
    , b(1)
    , c(2)
    {
    }

    virtual ~DefinesInterfaceTest()
    {
    }

    xme_status_t
    testCheck
    (
        bool condition
    )
    {
        XME_CHECK(condition, XME_STATUS_INVALID_PARAMETER);
        return XME_STATUS_SUCCESS;
    }

    xme_status_t
    testCheckRec
    (
        bool condition,
        bool* failed
    )
    {
        *failed = false;
        XME_CHECK_REC(condition, XME_STATUS_INVALID_PARAMETER, *failed = true);
        return XME_STATUS_SUCCESS;
    }

    xme_status_t
    testCheckMsg
    (
        bool condition,
        const char* msg
    )
    {
        XME_CHECK_MSG(condition, XME_STATUS_INVALID_PARAMETER, XME_LOG_ALWAYS, msg);
        return XME_STATUS_SUCCESS;
    }

    xme_status_t
    testCheckMsgRec
    (
        bool condition,
        bool* failed,
        const char* msg
    )
    {
        *failed = false;
        XME_CHECK_MSG_REC(condition, XME_STATUS_INVALID_PARAMETER, *failed = true, XME_LOG_ALWAYS, msg);
        return XME_STATUS_SUCCESS;
    }

    xme_status_t
    testAssert
    (
        bool condition
    )
    {
        XME_ASSERT(condition);
        return XME_STATUS_SUCCESS;
    }

    bool
    testAssertRval
    (
        bool condition
    )
    {
        XME_ASSERT_RVAL(condition, false);
        return true;
    }

    void
    testAssertNoRval
    (
        bool condition,
        bool* failed
    )
    {
        *failed = true;
        XME_ASSERT_NORVAL(condition);
        *failed = false;
    }

    int s(void)
    {
        return 2;
    }

    int s2(void)
    {
        return 2;
    }

    int s2s(void)
    {
        return 2;
    }

    int _s(void)
    {
        return 2;
    }

    int _s2(void)
    {
        return 2;
    }

    int _s2s(void)
    {
        return 2;
    }

    int s_(void)
    {
        return 2;
    }

    int s2_(void)
    {
        return 2;
    }

    int s2s_(void)
    {
        return 2;
    }

    int a;
    int b;
    int c;
};

/******************************************************************************/
/***   Helper functions                                                     ***/
/******************************************************************************/

// In order to test designated struct field initializers, we need to switch to
// C mode (C99 to be specific). Hence, the implementation of this function is
// in a .c file.
XME_EXTERN_C_BEGIN
unsigned int
testDesignedStructFieldInitializer(void);
XME_EXTERN_C_END

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     DefinesInterfaceTest                                                   //
//----------------------------------------------------------------------------//

TEST_F(DefinesInterfaceTest, ensureDebugCompile)
{
    // Issue a warning in case this is not compiled in debug mode,
    // because most of the tests will be no-operations in non-debug mode!

#ifndef DEBUG
    printf("Warning: xme_tests_core_defines_assert test suite will only work in debug builds!\n");
#endif // #ifndef DEBUG
}

TEST_F(DefinesInterfaceTest, zeroIsZero)
{
    EXPECT_EQ(0, 0);
}

TEST_F(DefinesInterfaceTest, maxSystemValueIsAtLeast16Bits)
{
    EXPECT_GE(XME_MAX_SYSTEM_VALUE, 0xFFFFU);
}

TEST_F(DefinesInterfaceTest, maxSystemValueIsConsistentWithType)
{
    xme_maxSystemValue_t v = (xme_maxSystemValue_t)(-1);
    EXPECT_GE(v, XME_MAX_SYSTEM_VALUE);
}

TEST_F(DefinesInterfaceTest, maxSystemValueStore)
{
    xme_maxSystemValue_t value = (xme_maxSystemValue_t)XME_MAX_SYSTEM_VALUE;
    EXPECT_EQ(XME_MAX_SYSTEM_VALUE, value);
}

TEST_F(DefinesInterfaceTest, statusSuccessIsZero)
{
    EXPECT_EQ(0, XME_STATUS_SUCCESS);
}

TEST_F(DefinesInterfaceTest, checkTrue)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, testCheck(1 == 1));
}

TEST_F(DefinesInterfaceTest, checkFalse)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, testCheck(0 == 1));
}

TEST_F(DefinesInterfaceTest, checkRecTrue)
{
    bool failed = true;
    ASSERT_EQ(XME_STATUS_SUCCESS, testCheckRec(1 == 1, &failed));
    EXPECT_FALSE(failed);
}

TEST_F(DefinesInterfaceTest, checkRecFalse)
{
    bool failed = false;
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, testCheckRec(0 == 1, &failed));
    EXPECT_TRUE(failed);
}

TEST_F(DefinesInterfaceTest, checkMsgTrue)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, testCheckMsg(1 == 1, "This text should NOT be displayed.\n"));
}

TEST_F(DefinesInterfaceTest, checkMsgFalse)
{
    EXPECT_EQ(XME_STATUS_INVALID_PARAMETER, testCheckMsg(0 == 1, "This text should be displayed.\n"));
}

TEST_F(DefinesInterfaceTest, checkMsgRecTrue)
{
    bool failed = true;
    ASSERT_EQ(XME_STATUS_SUCCESS, testCheckMsgRec(1 == 1, &failed, "This text should NOT be displayed.\n"));
    EXPECT_FALSE(failed);
}

TEST_F(DefinesInterfaceTest, checkMsgRecFalse)
{
    bool failed = false;
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, testCheckMsgRec(0 == 1, &failed, "This text should be displayed.\n"));
    EXPECT_TRUE(failed);
}

TEST_F(DefinesInterfaceTest, assertTrue)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, testAssert(1 == 1));
}

TEST_F(DefinesInterfaceTest, assertFalse)
{
#ifdef DEBUG
    try
    {
        EXPECT_EQ(XME_STATUS_UNEXPECTED, testAssert(0 == 1));
        EXPECT_TRUE(false);
    }
    catch (AssertionFailedException e)
    {
        // Assertion failure expected
    }
#else // #ifdef DEBUG
    // No assertions in release mode
    EXPECT_EQ(XME_STATUS_SUCCESS, testAssert(0 == 1));
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, assertRvalTrue)
{
    EXPECT_TRUE(testAssertRval(1 == 1));
}

TEST_F(DefinesInterfaceTest, assertRvalFalse)
{
#ifdef DEBUG
    try
    {
        EXPECT_FALSE(testAssertRval(0 == 1));
        EXPECT_TRUE(false);
    }
    catch (AssertionFailedException e)
    {
        // Assertion failure expected
    }
#else // #ifdef DEBUG
    // No assertions in release mode
    EXPECT_EQ(true, testAssertRval(0 == 1));
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, assertNoRvalTrue)
{
    bool failed = true;
    testAssertNoRval(1 == 1, &failed);
    EXPECT_FALSE(failed);
}

TEST_F(DefinesInterfaceTest, assertNoRvalFalse)
{
    bool failed;

#ifdef DEBUG
    try
    {
        failed = false;
        testAssertNoRval(0 == 1, &failed);
        EXPECT_TRUE(false);
    }
    catch (AssertionFailedException e)
    {
        // Assertion failure expected
        EXPECT_TRUE(failed);
    }
#else // #ifdef DEBUG
    // No assertions in release mode
    failed = true;
    testAssertNoRval(0 == 1, &failed);
    EXPECT_FALSE(failed);
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, structFieldInitializer)
{
     EXPECT_EQ(0U, testDesignedStructFieldInitializer());
}

TEST_F(DefinesInterfaceTest, assertSideEffectsCompareEquality)
{
#ifdef DEBUG
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == 2));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a == b));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a == 1));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a == 2));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2==3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2==3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2 == 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2 == 3));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2==3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2==3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2 == 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2 == 3));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1-2==3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 - 2==3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 - 2 == 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1-2 == 3));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1- -2==3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 - -2==3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 - -2 == 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1- -2 == 3));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == _s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == _s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == _s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s2_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s2s_()));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == _s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == _s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == _s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s2_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 == s2s_ ()));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == _s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == _s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == _s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s2_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s2s_())));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == _s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == _s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == _s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s2_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 == s2s_ ())));
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, assertSideEffectsCompareLess)
{
#ifdef DEBUG
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < 2));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a < b));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a < 1));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a < 2));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2<3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2<3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2 < 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2 < 3));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2<3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2<3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2 < 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2 < 3));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < _s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < _s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < _s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s2_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s2s_()));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < _s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < _s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < _s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s2_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 < s2s_ ()));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < _s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < _s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < _s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s2_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s2s_())));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < _s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < _s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < _s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s2_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 < s2s_ ())));
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, assertSideEffectsCompareLessEqual)
{
#ifdef DEBUG
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= 2));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a <= b));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a <= 1));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a <= 2));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2<=3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2<=3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2 <= 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2 <= 3));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2<=3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2<=3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2 <= 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2 <= 3));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= _s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= _s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= _s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s2_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s2s_()));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= _s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= _s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= _s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s2_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 <= s2s_ ()));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= _s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= _s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= _s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s2_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s2s_())));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= _s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= _s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= _s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s2_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 <= s2s_ ())));
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, assertSideEffectsCompareGreater)
{
#ifdef DEBUG
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > 2));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a > b));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a > 1));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a > 2));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2>3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2>3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2 > 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2 > 3));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2>3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2>3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2 > 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2 > 3));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > _s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > _s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > _s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s2_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s2s_()));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > _s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > _s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > _s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s2_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 > s2s_ ()));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > _s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > _s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > _s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s2_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s2s_())));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > _s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > _s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > _s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s2_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 > s2s_ ())));
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, assertSideEffectsCompareGreaterEqual)
{
#ifdef DEBUG
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= 2));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a >= b));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a >= 1));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(a >= 2));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2>=3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2>=3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2 >= 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2 >= 3));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2>=3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2>=3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + +2 >= 3));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ +2 >= 3));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= _s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= _s2()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= _s2s()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s2_()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s2s_()));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= _s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= _s2 ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= _s2s ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s2_ ()));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 >= s2s_ ()));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= _s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= _s2())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= _s2s())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s2_())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s2s_())));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= _s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= _s2 ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= _s2s ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s2_ ())));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 >= s2s_ ())));
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, assertSideEffectsIncrement)
{
#ifdef DEBUG
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2==a++));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2==a++));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2 == a++));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2 == a++));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2==++a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2==++a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + 2 == ++a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1+2 == ++a));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(a++));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1+a++));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + a++));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(++a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1+ ++a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + ++a));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1+2==a++)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 + 2==a++)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 + 2 == a++)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1+2 == a++)));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1+2==++a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 + 2==++a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 + 2 == ++a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1+2 == ++a)));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(a++)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1+a++)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 + a++)));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(++a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1+ ++a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 + ++a)));
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, assertSideEffectsDecrement)
{
#ifdef DEBUG
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1-2==a--));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 - 2==a--));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 - 2 == a--));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1-2 == a--));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1-2==--a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 - 2==--a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 - 2 == --a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1-2 == --a));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(a--));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1+a--));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 + a--));

    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(--a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1- --a));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(1 - --a));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1-2==a--)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 - 2==a--)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 - 2 == a--)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1-2 == a--)));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1-2==--a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 - 2==--a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 - 2 == --a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1-2 == --a)));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(a--)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1+a--)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 + a--)));

    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(--a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1- --a)));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(1 - --a)));
#endif // #ifdef DEBUG
}

TEST_F(DefinesInterfaceTest, assertSideEffectsAssignment)
{
#ifdef DEBUG
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(a = b));
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(XME_ASSERT_NO_SIDE_EFFECTS(a = b)));
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(   a = b)); // excess spaces on purpose
    EXPECT_EQ(0, XME_ASSERT_SIDE_EFFECT_TEST(   XME_ASSERT_NO_SIDE_EFFECTS(a = b))); // excess spaced on purpose
    EXPECT_NE(0, XME_ASSERT_SIDE_EFFECT_TEST(a = 1));
#endif // #ifdef DEBUG
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
