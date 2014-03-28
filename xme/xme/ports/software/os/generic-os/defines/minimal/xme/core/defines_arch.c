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
 * $Id: defines_arch.c 4382 2013-07-26 14:19:08Z geisinger $
 */

/**
 * \file
 *         Generic definition and defines (architecture specific part:
 *         generic embedded implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
#ifndef NDEBUG
int
xme_assert_endlessLoop(void)
{
	for(;;);

	return 0;
}
#endif // #ifndef NDEBUG
