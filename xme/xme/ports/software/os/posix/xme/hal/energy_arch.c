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
 * $Id: energy_arch.c 6684 2014-02-07 14:34:26Z geisinger $
 */

/**
 * \file
 *         Energy management abstraction (platform specific part: Posix).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/energy.h"

#include <pthread.h>
#include <unistd.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
xme_hal_energy_sleep(void)
{
	// TODO (ticket #805): If only pthread_yield() is used here, this will result in the
	//                     system spinning idle, since the resource manager has not yet been
	//                     implemented, which causes a high CPU usage. This is why we let
	//                     the main thread sleep here for the time being.
    (void) usleep(1000000);

	pthread_yield();
}
