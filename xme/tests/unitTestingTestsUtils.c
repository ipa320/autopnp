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
 * $Id: unitTestingTestsUtils.c 3348 2013-05-17 12:28:31Z geisinger $
 */

/**
 * \file
 *         Testsuite unit tests utils.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "tests/unitTestingTestsUtils.h"

#if defined(__cplusplus)
#error "This translation unit must not be compiled in C++ mode for the test to be effective!"
#endif // #if defined(__cplusplus)

void xme_raiseAssertionFailure(void)
{
    XME_ASSERT_NORVAL(0);
}
