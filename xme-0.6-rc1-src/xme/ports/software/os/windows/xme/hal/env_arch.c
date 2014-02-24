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
 * $Id: env_arch.c 3366 2013-05-21 07:26:05Z ruiz $
 */

/**
 * \file
 *         Energy management abstraction (platform specific part: Windows).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/env.h"
#include "xme/hal/include/mem.h"

#include "xme/core/log.h"

#include <Windows.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_hal_env_getCurrentExecutablePath
(
	char * const filePath, 
	size_t size
)
{
  xme_status_t status;
  TCHAR path[MAX_PATH];
  if(! GetModuleFileName(NULL, path, MAX_PATH))
  {
    XME_LOG(XME_LOG_DEBUG, "Problem in reading the path for the current executable: %s (%d)\n", path, GetLastError());
    status = XME_STATUS_NOT_FOUND;
  }
  else
  {
    xme_hal_mem_copy(filePath, path, MAX_PATH < size ? MAX_PATH : size);
    status = XME_STATUS_SUCCESS;
  }
  return status;
}
