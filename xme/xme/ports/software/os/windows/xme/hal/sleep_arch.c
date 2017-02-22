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
 * $Id: sleep_arch.c 5201 2013-09-27 09:43:54Z wiesmueller $
 */

/**
 * \file
 *         Sleep abstraction (platform specific part: Windows).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/sleep.h"

#include "xme/hal/include/time.h"

#include <Windows.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
xme_hal_sleep_sleep(xme_hal_time_timeInterval_t sleepTime)
{
    // Windows natively offers sleeping at millisecond granularity only
    Sleep((DWORD)xme_hal_time_timeIntervalInMilliseconds(sleepTime));
}
