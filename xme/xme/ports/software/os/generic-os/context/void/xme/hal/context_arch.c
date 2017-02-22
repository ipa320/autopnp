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
 * $Id: context_arch.c 3010 2013-04-23 12:00:57Z geisinger $
 */

/**
 * \file
 *         Context abstraction (architecture specific part:
 *         generic OS-based implementation).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_hal_context_init(void)
{
	return XME_STATUS_SUCCESS;
}

void
xme_hal_context_fini(void)
{
	// Nothing to do
}

xme_status_t
xme_hal_context_setContext
(
	xme_hal_context_contextHandle_t context
)
{
	XME_UNUSED_PARAMETER(context);

	// Always return success
	return XME_STATUS_SUCCESS;
}

xme_hal_context_contextHandle_t
xme_hal_context_getContext(void)
{
	return XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE;
}
