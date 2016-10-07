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
 * $Id: defines_arch.c 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Generic definition and defines (architecture specific part:
 *         generic OS based implementation).
 */

#ifdef DEBUG
/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#if defined(linux)
#include <execinfo.h>
#endif // linux
/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

#if defined(linux)

void
xme_print_stacktrace(void)
{
	void *array[10];
	size_t size;
	char **strings;
	size_t i;

	size = backtrace(array, 10);
	strings = backtrace_symbols(array, size);

	fprintf(stderr, "Obtained %zd stack frames.\n", size);

	for (i = 0; i < size; i++)
		fprintf(stderr, "%s\n", strings[i]);

	xme_fallback_free(strings);
}
#else // linux

void
xme_print_stacktrace(void)
{
    // TODO: Clean up after merge
	//XXX: implement stacktrace for your platform, Dude!
}

#endif // linux

int
xme_assert_checkForSideEffects(const char* macro, const char* condition, const char* file, const unsigned long line, int emitWarning)
{
    // Check for "no side effects" flag
    {
        static const char compare[] = "XME_ASSERT_NO_SIDE_EFFECTS";
        static const int compareLength = (sizeof(compare)/sizeof(compare[0]))-1;
        const char* cond = condition;
        int i;

        /* Whitespace is automatically stripped by the preprocessor
        char c;
        while (c = *cond)
        {
            // Ignore whitespace and control characters (e.g., tab, newline)
            if (c > 0x20) break;
            cond++;
        }*/

        for (i = 0; i < compareLength; ++i)
        {
            if (compare[i] != *cond++)
            {
                break;
            }
        }

        if (compareLength == i)
        {
            //xme_fallback_printf("Developer warning: %s(%s) side effects ignored at %s:%d\n", macro, condition, file, line);
            return 0;
        }
    }

    // Check for assignments
    {
        const char* cond = condition;
        char c, h0 = 0, h1 = 0;
        while (0 != (c = *cond++))
        {
            // Ignore whitespace and control characters (e.g., tab, newline)
            if (c <= 0x20) continue;

            // An identifier may end with digits (0x30..0x39), letters
            // (0x41..0x5A, 0x61..0x7A) or underscore (0x5F). The right side
            // of an assignment might start with the same character classes.
            if
            (
                ( (c >= 0x30 && c <= 0x39) || (c >= 0x41 && c <= 0x5A) || (c >= 0x61 && c <= 0x7A) || (0x5F == c) ) &&
                ( '=' == h0 ) &&
                ( (h1 >= 0x30 && h1 <= 0x39) || (h1 >= 0x41 && h1 <= 0x5A) || (h1 >= 0x61 && h1 <= 0x7A) || (0x5F == h1) )
            )
            {
                if (emitWarning)
                {
                    xme_fallback_printf("Developer warning: %s(%s) with potential side effects (assignment) at %s:%lu\n", macro, condition, file, line);
                }
                return 1;
            }

            h1 = h0;
            h0 = c;
        }
    }

    // Check for function calls
    {
        // identifier is initially 0, because an opening bracket at the beginning
        // of the condition does not indicate a function call.
        char identifier = 0, c;
        const char* identifierName = NULL;
        const char* cond = condition;
        while (0 != (c = *cond++))
        {
            // Ignore whitespace and control characters (e.g., tab, newline)
            if (c <= 0x20)
            {
                identifierName = NULL;
                continue;
            }

            // Heuristics: if an opening bracket follows a token, it is
            // interpreted as a function call with potential side effects.
            // Ignore special operator 'sizeof(x)' and related macro 'offsetof(x)',
            // which are assumed to be side effect free (this will not work
            // for people who like to write it like this: 'sizeof (x)'!).
            // TODO: Introduce platform abstraction for strncmp() and strcmp()!
            if ('(' == c && identifier && (!identifierName || (identifierName
                && strncmp(identifierName, "sizeof(", 7) && strncmp(identifierName, "offsetof(", 9))))
            {
                if (emitWarning)
                {
                    xme_fallback_printf("Developer warning: %s(%s) with potential side effects (function call) at %s:%lu\n", macro, condition, file, line);
                }
                return 1;
            }

            // An identifier starts with letters (0x41..0x5A, 0x61..0x7A) or underscore (0x5F).
            // An identifier may continue with any of the above or digits (0x30..0x39).
            identifier = (identifier ? (c >= 0x30 && c <= 0x39) : 0) || (c >= 0x41 && c <= 0x5A) || (c >= 0x61 && c <= 0x7A) || (0x5F == c);
            identifierName = identifierName ? identifierName : (identifier ? (cond-1) : NULL);
        }
    }

    // Check for increment/decrement operators
    {
        const char* cond = condition;
        char c, h0 = 0;
        while (0 != (c = *cond++))
        {
            // We just need for find two subsequent '+' or '-' characters
            if ('+' == c || '-' == c)
            {
                if (0 == h0)
                {
                    h0 = c;
                }
                else if (h0 == c)
                {
                    if (emitWarning)
                    {
                        (void) xme_fallback_printf("Developer warning: %s(%s) with potential side effects (%s) at %s:%lu\n", macro, condition, '+' == h0 ? "increment" : "decrement", file, line);
                    }
                    return 1;
                }
                else
                {
                    h0 = 0;
                }
            }
            else
            {
                h0 = 0;
            }
        }
    }

    return 0;
}
#endif // #ifndef NDEBUG

