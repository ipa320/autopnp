/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: env_arch.c 7297 2014-02-13 09:57:41Z geisinger $
 */

/**
 * \file
 * \brief Generic environment abstraction (platform specific part: POSIX).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/log.h"

#include "xme/hal/include/env.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

#include <unistd.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define MAX_PATH 260

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_hal_env_init(void)
{
    // Nothing to do
    return XME_STATUS_SUCCESS;
}

void
xme_hal_env_fini(void)
{
    // Nothing to do
}

xme_status_t
xme_hal_env_getCurrentExecutablePath(char* const filePath, size_t size)
{
    xme_status_t status;
    char path[MAX_PATH];

    XME_CHECK(NULL != filePath, XME_STATUS_INVALID_PARAMETER);

    (void) xme_hal_mem_set(path, 0u, MAX_PATH);

#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
    // how about: sysctl CTL_KERN KERN_PROC KERN_PROC_PATHNAME -1
    if (-1 == readlink("/proc/curproc/file", path, MAX_PATH))
#elif defined(sun) || defined(__sun)
    // or should we use getexecname() ?
    if (-1 == readlink("/proc/self/path/a.out", path, MAX_PATH))
#elif defined(__linux__) || defined(linux) || defined(__linux_)
    if (-1 == readlink ("/proc/self/exe", path, MAX_PATH))
#else
#error "Please implement me for this operating system!"
#endif
    {
        XME_LOG(XME_LOG_DEBUG, "Problem in reading the path for the current executable: %s\n", path);
        status = XME_STATUS_NOT_FOUND;
    }
    else
    {
        (void) xme_hal_safeString_strncpy(filePath, path, MAX_PATH < size ? MAX_PATH : size);
        status = XME_STATUS_SUCCESS;
    }

    return status;
}

void
xme_hal_env_setConsoleTitle
(
    const char* title
)
{
#ifdef XME_ENABLE_CONSOLE_COLORS
    // This does not work in all distributions and terminal types, but it's worth a try
    (void) xme_fallback_printf("\033]0;%s\007", title);
    (void) fflush(stdout);
#else
    // No really compatible way to do this
    XME_UNUSED_PARAMETER(title);
#endif
}
