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
 * $Id: mmap_arch.c 6626 2014-02-05 14:36:43Z geisinger $
 */

/**
 * \file
 *         Memory mapping abstraction (platform specific part: Windows).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/mmap.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_mmap_mapPrivateMemory
(
    void* addr,
    size_t length,
    uint8_t protection,
    uint32_t flags,
    void** const outAddr
)
{
    XME_UNUSED_PARAMETER(addr);
    XME_UNUSED_PARAMETER(length);
    XME_UNUSED_PARAMETER(protection);
    XME_UNUSED_PARAMETER(flags);
    XME_UNUSED_PARAMETER(outAddr);

    XME_ASSERT(!"Not yet implemented!");

    return XME_STATUS_UNSUPPORTED;
}

xme_status_t
xme_hal_mmap_mapSharedMemory
(
    void* addr,
    size_t length,
    uint8_t protection,
    int fileDescriptor,
    off_t offset,
    uint32_t flags,
    void** const outAddr
)
{
    XME_UNUSED_PARAMETER(addr);
    XME_UNUSED_PARAMETER(length);
    XME_UNUSED_PARAMETER(protection);
    XME_UNUSED_PARAMETER(fileDescriptor);
    XME_UNUSED_PARAMETER(offset);
    XME_UNUSED_PARAMETER(flags);
    XME_UNUSED_PARAMETER(outAddr);

    XME_ASSERT(!"Not yet implemented!");

    return XME_STATUS_UNSUPPORTED;
}

xme_status_t
xme_hal_mmap_syncMappedMemory
(
    void* addr,
    size_t length,
    uint32_t flags
)
{
    XME_UNUSED_PARAMETER(addr);
    XME_UNUSED_PARAMETER(length);
    XME_UNUSED_PARAMETER(flags);

    XME_ASSERT(!"Not yet implemented!");

    return XME_STATUS_UNSUPPORTED;
}

xme_status_t
xme_hal_mmap_unmapMemory
(
    void* addr,
    size_t length
)
{
    XME_UNUSED_PARAMETER(addr);
    XME_UNUSED_PARAMETER(length);

    XME_ASSERT(!"Not yet implemented!");

    return XME_STATUS_UNSUPPORTED;
}

uint32_t
xme_hal_mmap_getSystemPageSize(void)
{
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return (uint32_t) si.dwPageSize;
}