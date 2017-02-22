/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: task_arch.c 2533 2013-02-28 16:05:30Z ruiz $
 */

/**
 * \file
 *         Task abstraction (architecture specific part: Windows implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/task.h"

/******************************************************************************/
/***   Implementations                                                      ***/
/******************************************************************************/
xme_hal_taskId_t
xme_hal_task_getCurrentTaskId(void)
{
	return GetCurrentThreadId();
}
