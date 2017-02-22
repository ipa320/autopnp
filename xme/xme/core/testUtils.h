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
 * $Id: testUtils.h 6686 2014-02-07 15:03:31Z geisinger $
 */

/**
 * \file
 *         Unit testing utilities.
 */

#ifndef XME_CORE_TESTUTILS_H
#define XME_CORE_TESTUTILS_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <gtest/gtest.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/**
 * \def EXPECT_XME_ASSERTION_FAILURE
 *
 * \brief Specifies that the given statement is supposed to throw a single
 *        assertion failure.
 *
 * \details If the assertion failure is not thrown, a failing googletest unit
 *          test will be provoked using EXPECT_EQ().
 *
 * \note Since assertion checks are NOPs in Release mode and hence
 *       assertion failures cannot be thrown in this mode, this macro
 *       will -- despite its name -- expect no assertion failures to be
 *       thrown when compiling in Release mode. This saves a developer
 *       from checking this condition when this macro is used.
 *
 * \param[in] statement The statement to test for assertion failures.
 *
 * \return Returns nothing, even if the statement returns a value.
 *
 * \see XME_ASSERT()
 */
#define EXPECT_XME_ASSERTION_FAILURE(statement) \
    EXPECT_XME_ASSERTION_FAILURE_COUNT(statement, 1)

/**
 * \def EXPECT_XME_ASSERTION_FAILURES
 *
 * \brief Specifies that the given statement is supposed to throw at least
 *        one assertion failure.
 *
 * \details If not at least one assertion failures is thrown, a failing
 *          googletest unit test will be provoked using EXPECT_GT().
 *
 * \note Since assertion checks are NOPs in Release mode and hence
 *       assertion failures cannot be thrown in this mode, this macro
 *       will -- despite its name -- expect no assertion failures to be
 *       thrown when compiling in Release mode. This saves a developer
 *       from checking this condition when this macro is used.
 *
 * \param[in] statement The statement to test for assertion failures.
 *
 * \return Returns nothing, even if the statement returns a value.
 *
 * \see XME_ASSERT()
 */
#if defined(DEBUG) || defined(DOXYGEN)
    // Debug mode: Assertion failures can happen

    #ifdef __cplusplus
        // C++ Debug mode

        #define EXPECT_XME_ASSERTION_FAILURES(statement) \
            do \
            { \
                unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
                xme_fallback_printf("--- Following assertion failures are EXPECTED and may be safely IGNORED:\n"); \
                \
                try \
                { \
                    statement; \
                } \
                catch (AssertionFailedException &afe) \
                { \
                    xme_fallback_printf("%s\n", afe.what()); \
                } \
                xme_fallback_printf("--- (unable to determine exact number of assertion failures)\n"); \
                EXPECT_GT(_xme_assert_assertionFailureCounter, __oldAssertionFailureCounter); \
            } \
            while (0)

    #else // #ifdef __cplusplus
        // Non-C++ Debug mode

        #define EXPECT_XME_ASSERTION_FAILURES(statement) \
            do \
            { \
                unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
                xme_fallback_printf("--- Following assertion failures are EXPECTED and may be safely IGNORED:\n"); \
                { \
                    statement; \
                } \
                xme_fallback_printf("---\n"); \
                EXPECT_GT(_xme_assert_assertionFailureCounter, __oldAssertionFailureCounter); \
            } \
            while (0)

    #endif // #ifdef __cplusplus

#else // #if defined(DEBUG) || defined(DOXYGEN)
    // Release mode: Assertion failures cannot happen
    #define EXPECT_XME_ASSERTION_FAILURES(statement) \
        do \
        { \
            EXPECT_NO_XME_ASSERTION_FAILURES(statement); \
        } \
        while (0)
#endif // #if defined(DEBUG) || defined(DOXYGEN)

/**
 * \def EXPECT_XME_ASSERTION_FAILURE_COUNT
 *
 * \brief Specifies that the given statement is supposed to throw the given
 *        number of assertion failures.
 *
 * \details If the specified number of assertion failures is not thrown, a
 *          failing googletest unit test will be provoked using EXPECT_EQ().
 *
 * \note Since assertion checks are NOPs in Release mode and hence
 *       assertion failures cannot be thrown in this mode, this macro
 *       will -- despite its name -- expect no assertion failures to be
 *       thrown when compiling in Release mode. This saves a developer
 *       from checking this condition when this macro is used.
 *
 * \param[in] statement The statement to test for assertion failures.
 * \param[in] count Expected number of assertion failures.
 *
 * \return Returns nothing, even if the statement returns a value.
 *
 * \see XME_ASSERT()
 */
#if defined(DEBUG) || defined(DOXYGEN)
    // Debug mode: Assertion failures can happen

    #ifdef __cplusplus
        // C++ Debug mode

        #define EXPECT_XME_ASSERTION_FAILURE_COUNT(statement, count) \
            do \
            { \
                unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
                xme_fallback_printf \
                ( \
                    (1 == count ? \
                        "--- Following single assertion failure is EXPECTED and may be safely IGNORED:\n" : \
                        "--- Following %u assertion failures are EXPECTED and may be safely IGNORED:\n" \
                    ), \
                    count \
                ); \
                \
                try \
                { \
                    statement; \
                } \
                catch (AssertionFailedException &afe) \
                { \
                    xme_fallback_printf("%s\n", afe.what()); \
                } \
                xme_fallback_printf("---%s\n", (1 == count) ? "" : " (unable to determine exact number of assertion failures)"); \
                /* Since AssertionFailedExceptions are fatal in that they  */ \
                /* break the control flow, we can only detect the presence */ \
                /* of at least one happening, but not the exact count      */ \
                EXPECT_GT(_xme_assert_assertionFailureCounter, __oldAssertionFailureCounter); \
            } \
            while (0)

    #else // #ifdef __cplusplus
        // Non-C++ Debug mode

        #define EXPECT_XME_ASSERTION_FAILURE_COUNT(statement, count) \
            do \
            { \
                unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
                xme_fallback_printf \
                ( \
                    (1 == count ? \
                        "--- Following single assertion failure is EXPECTED and may be safely IGNORED:\n" : \
                        "--- Following %u assertion failures are EXPECTED and may be safely IGNORED:\n" \
                    ), \
                    count \
                ); \
                { \
                    statement; \
                } \
                xme_fallback_printf("---\n"); \
                EXPECT_EQ(__oldAssertionFailureCounter + count, _xme_assert_assertionFailureCounter); \
            } \
            while (0)

    #endif // #ifdef __cplusplus

#else // #if defined(DEBUG) || defined(DOXYGEN)
    // Release mode: Assertion failures cannot happen
    #define EXPECT_XME_ASSERTION_FAILURE_COUNT(statement, count) \
        do \
        { \
            XME_UNUSED_PARAMETER(count); \
            EXPECT_NO_XME_ASSERTION_FAILURES(statement); \
        } \
        while (0)
#endif // #if defined(DEBUG) || defined(DOXYGEN)

/**
 * \def ASSERT_XME_ASSERTION_FAILURE
 *
 * \brief Specifies that the given statement is supposed to throw a single
 *        assertion failure.
 *
 * \details If the assertion failure is not thrown, a failing googletest unit
 *          test will be provoked using ASSERT_EQ().
 *
 * \note Since assertion checks are NOPs in Release mode and hence
 *       assertion failures cannot be thrown in this mode, this macro
 *       will -- despite its name -- expect no assertion failures to be
 *       thrown when compiling in Release mode. This saves a developer
 *       from checking this condition when this macro is used.
 *
 * \param[in] statement The statement to test for assertion failures.
 *
 * \return Returns nothing, even if the statement returns a value.
 *
 * \see XME_ASSERT()
 */
#define ASSERT_XME_ASSERTION_FAILURE(statement) \
    ASSERT_XME_ASSERTION_FAILURE_COUNT(statement, 1)

/**
 * \def ASSERT_XME_ASSERTION_FAILURES
 *
 * \brief Specifies that the given statement is supposed to throw at least
 *        one assertion failure.
 *
 * \details If not at least one assertion failures is thrown, a failing
 *          googletest unit test will be provoked using ASSERT_GT().
 *
 * \note Since assertion checks are NOPs in Release mode and hence
 *       assertion failures cannot be thrown in this mode, this macro
 *       will -- despite its name -- expect no assertion failures to be
 *       thrown when compiling in Release mode. This saves a developer
 *       from checking this condition when this macro is used.
 *
 * \param[in] statement The statement to test for assertion failures.
 *
 * \return Returns nothing, even if the statement returns a value.
 *
 * \see XME_ASSERT()
 */
#if defined(DEBUG) || defined(DOXYGEN)
    // Debug mode: Assertion failures can happen

    #ifdef __cplusplus
        // C++ Debug mode

        #define ASSERT_XME_ASSERTION_FAILURES(statement) \
            do \
            { \
                unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
                xme_fallback_printf("--- Following assertion failures are EXPECTED and may be safely IGNORED:\n"); \
                \
                try \
                { \
                    statement; \
                } \
                catch (AssertionFailedException &afe) \
                { \
                    xme_fallback_printf("%s\n", afe.what()); \
                } \
                xme_fallback_printf("---\n"); \
                EXPECT_GT(_xme_assert_assertionFailureCounter, __oldAssertionFailureCounter); \
            } \
            while (0)

    #else // #ifdef __cplusplus
        // Non-C++ Debug mode

        #define ASSERT_XME_ASSERTION_FAILURES(statement) \
            do \
            { \
                unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
                xme_fallback_printf("--- Following assertion failures are EXPECTED and may be safely IGNORED:\n"); \
                { \
                    statement; \
                } \
                xme_fallback_printf("---\n"); \
                ASSERT_GT(_xme_assert_assertionFailureCounter, __oldAssertionFailureCounter); \
            } \
            while (0)

    #endif // #ifdef __cplusplus

#else // #if defined(DEBUG) || defined(DOXYGEN)
    // Release mode: Assertion failures cannot happen
    #define ASSERT_XME_ASSERTION_FAILURES(statement) \
        do \
        { \
            EXPECT_NO_XME_ASSERTION_FAILURES(statement); \
        } \
        while (0)
#endif // #if defined(DEBUG) || defined(DOXYGEN)

/**
 * \def ASSERT_XME_ASSERTION_FAILURE_COUNT
 *
 * \brief Specifies that the given statement is supposed to throw the given
 *        number of assertion failures.
 *
 * \details If the specified number of assertion failures is not thrown, a
 *          failing googletest unit test will be provoked using ASSERT_EQ().
 *
 * \note Since assertion checks are NOPs in Release mode and hence
 *       assertion failures cannot be thrown in this mode, this macro
 *       will -- despite its name -- expect no assertion failures to be
 *       thrown when compiling in Release mode. This saves a developer
 *       from checking this condition when this macro is used.
 *
 * \param[in] statement The statement to test for assertion failures.
 * \param[in] count Expected number of assertion failures.
 *
 * \return Returns nothing, even if the statement returns a value.
 *
 * \see XME_ASSERT()
 */
#if defined(DEBUG) || defined(DOXYGEN)
    // Debug mode: Assertion failures can happen

    #ifdef __cplusplus
        // C++ Debug mode

        #define ASSERT_XME_ASSERTION_FAILURE_COUNT(statement, count) \
            do \
            { \
                unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
                xme_fallback_printf \
                ( \
                    (1 == count ? \
                        "--- Following single assertion failure is EXPECTED and may be safely IGNORED:\n" : \
                        "--- Following %u assertion failures are EXPECTED and may be safely IGNORED:\n" \
                    ), \
                    count \
                ); \
                \
                try \
                { \
                    statement; \
                } \
                catch (AssertionFailedException &afe) \
                { \
                    xme_fallback_printf("%s\n", afe.what()); \
                } \
                xme_fallback_printf("---%s\n", (1 == count) ? "" : " (unable to determine exact number of assertion failures)"); \
                /* Since AssertionFailedExceptions are fatal in that they  */ \
                /* break the control flow, we can only detect the presence */ \
                /* of at least one happening, but not the exact count      */ \
                EXPECT_GT(_xme_assert_assertionFailureCounter, __oldAssertionFailureCounter); \
            } \
            while (0)

    #else // #ifdef __cplusplus
        // Non-C++ Debug mode

        #define ASSERT_XME_ASSERTION_FAILURE_COUNT(statement, count) \
            do \
            { \
                unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
                xme_fallback_printf \
                ( \
                    (1 == count ? \
                        "--- Following single assertion failure is EXPECTED and may be safely IGNORED:\n" : \
                        "--- Following %u assertion failures are EXPECTED and may be safely IGNORED:\n" \
                    ), \
                    count \
                ); \
                { \
                    statement; \
                } \
                xme_fallback_printf("---\n"); \
                ASSERT_EQ(__oldAssertionFailureCounter + count, _xme_assert_assertionFailureCounter); \
            } \
            while (0)

    #endif // #ifdef __cplusplus

#else // #if defined(DEBUG) || defined(DOXYGEN)
    // Release mode: Assertion failures cannot happen
    #define ASSERT_XME_ASSERTION_FAILURE_COUNT(statement, count) \
        do \
        { \
            XME_UNUSED_PARAMETER(count); \
            ASSERT_NO_XME_ASSERTION_FAILURES(statement); \
        } \
        while (0)
#endif // #if defined(DEBUG) || defined(DOXYGEN)

/**
 * \def EXPECT_NO_XME_ASSERTION_FAILURES
 *
 * \brief Specifies that the given statement is not supposed to throw any
 *        assertion failures.
 *
 * \details If an assertion failure is thrown nevertheless, a failing
 *          googletest unit test will be provoked using EXPECT_EQ().
 *
 * \note Since assertion checks are NOPs in Release mode and hence
 *       assertion failures cannot be thrown in this mode, this macro
 *       should never cause a failing unit test when compiling in
 *       Release mode.
 *
 * \param[in] statement The statement to test for assertion failures.
 *
 * \return Returns nothing, even if the statement returns a value.
 *
 * \see XME_ASSERT()
 */
#define EXPECT_NO_XME_ASSERTION_FAILURES(statement) \
    do \
    { \
        unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
        { \
            statement; \
        } \
        EXPECT_EQ(__oldAssertionFailureCounter, _xme_assert_assertionFailureCounter); \
    } \
    while (0)

/**
 * \def ASSERT_NO_XME_ASSERTION_FAILURES
 *
 * \brief Specifies that the given statement is not supposed to throw any
 *        assertion failures.
 *
 * \details If an assertion failure is thrown nevertheless, a failing
 *          googletest unit test will be provoked using ASSERT_EQ().
 *
 * \note Since assertion checks are NOPs in Release mode and hence
 *       assertion failures cannot be thrown in this mode, this macro
 *       should never cause a failing unit test when compiling in
 *       Release mode.
 *
 * \param[in] statement The statement to test for assertion failures.
 *
 * \return Returns nothing, even if the statement returns a value.
 *
 * \see XME_ASSERT()
 */
#define ASSERT_NO_XME_ASSERTION_FAILURES(statement) \
    do \
    { \
        unsigned int __oldAssertionFailureCounter = _xme_assert_assertionFailureCounter; \
        { \
            statement; \
        } \
        ASSERT_EQ(__oldAssertionFailureCounter, _xme_assert_assertionFailureCounter); \
    } \
    while (0)

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/
#ifdef __cplusplus

namespace xme
{
namespace testing
{

/**
 * \class Test
 *
 * \brief Testing class with assertion-safe setup and teardown methods.
 *
 * \details This class overwrites the SetUp() and TearDown() methods of the
 *          GoolgeTest ::testing::Test class. The overwritten methods
 *          implement checking for XME assertion failures and mark the unit
 *          test as failed when assertion failures are raised.
 *          Hence using this class is only meaningful when you don't expect
 *          any assertion failures to be raised during setup and teardown.
 *
 * \note    When using this class, make sure you put your setup and teardown
 *          code in AssertionCheckedSetUp() and AssertionCheckedTearDown()
 *          as opposed to the normal SetUp() and TearDown() methods.
 */
class Test : public ::testing::Test
{
protected:
    /**
     * \brief Virtual destructor.
     */
    virtual ~Test()
    {
    }

    /**
     * \brief Called when the testsuite is to be set up.
     */
    virtual void AssertionCheckedSetUp()
    {
    }

    /**
     * \brief Called when the testsuite is to be torn down.
     */
    virtual void AssertionCheckedTearDown()
    {
    }

private:
    /**
     * \brief Overwritten setup method from base class.
     */
    virtual void SetUp()
    {
        EXPECT_NO_XME_ASSERTION_FAILURES(AssertionCheckedSetUp());
    }

    /**
     * \brief Overwritten teardown method from base class.
     */
    virtual void TearDown()
    {
        EXPECT_NO_XME_ASSERTION_FAILURES(AssertionCheckedTearDown());
    }
};

} // namespace testing
} // namespace xme

#endif // #ifdef __cplusplus

#endif // #ifndef XME_CORE_TESTUTILS_H
