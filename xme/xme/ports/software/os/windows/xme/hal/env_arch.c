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
 * $Id: env_arch.c 7278 2014-02-12 09:59:23Z geisinger $
 */

/**
 * \file
 * \brief Generic environment abstraction (platform specific part: Windows).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/env.h"
#include "xme/hal/include/safeString.h"

#include "xme/core/log.h"

#include <Windows.h>

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
xme_hal_env_getCurrentExecutablePath
(
    char* const filePath,
    size_t size
)
{
    xme_status_t status;
    TCHAR path[MAX_PATH];

    XME_CHECK(NULL != filePath, XME_STATUS_INVALID_PARAMETER);

    if (!GetModuleFileName(NULL, path, MAX_PATH))
    {
        XME_LOG(XME_LOG_DEBUG, "[Env] Problem reading file path for the current executable: %s (%d)\n", path, GetLastError());
        status = XME_STATUS_NOT_FOUND;
    }
    else
    {
        (void) xme_hal_safeString_strncpy(filePath, path, size);
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
    (void) SetConsoleTitle(title);
}
