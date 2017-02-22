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
 * $Id: energy_arch.c 3345 2013-05-17 12:07:58Z geisinger $
 */

/**
 * \file
 *         Energy management abstraction (platform specific part: Windows).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/energy.h"

#include <Windows.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
xme_hal_energy_sleep()
{
	// TODO (ticket #805): If only SwitchToThread() is used here, this will result in the
	//                     system spinning idle, since the resource manager has not yet been
	//                     implemented, which causes a high CPU usage. This is why we let
	//                     the main thread sleep here for the time being.
	Sleep(1000);

	SwitchToThread();
}
