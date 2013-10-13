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
 * $Id: env_arch.c 4871 2013-08-30 11:57:36Z camek $
 */

/**
 * \file
 *         Energy management abstraction (platform specific part: Windows).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/log.h"

#include "xme/hal/include/env.h"
#include "xme/hal/include/mem.h"

#include <unistd.h>

#define MAX_PATH 260

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_hal_env_getCurrentExecutablePath(char * const filePath, size_t size)
{
  xme_status_t status;
  char path[MAX_PATH];
  //size_t result;

 (void) xme_hal_mem_set (path, 0u, MAX_PATH);
#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
  // how about: sysctl CTL_KERN KERN_PROC KERN_PROC_PATHNAME -1
  if(-1 == readlink("/proc/curproc/file", path, MAX_PATH))
#elif defined(sun) || defined(__sun)
    // or should we use getexecname() ?
  if(-1 == readlink("/proc/self/path/a.out", path, MAX_PATH))
#elif defined(__linux__) || defined(linux) || defined(__linux_)
  if(-1 == readlink ("/proc/self/exe", path, MAX_PATH))
#elif defined(__PikeOS__)
  if(1)
#else
#error "Please implement me for specific OS!"
#endif
  {
    XME_LOG(XME_LOG_DEBUG, "Problem in reading the path for the current executable: %s\n", path);
    status = XME_STATUS_NOT_FOUND;
  }
  else
  {
      (void) xme_hal_mem_copy(filePath, path, MAX_PATH < size ? MAX_PATH : size);
    status = XME_STATUS_SUCCESS;
  }
  return status;
}
