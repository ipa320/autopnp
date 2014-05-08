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
 *         Memory mapping abstraction (platform specific part: POSIX).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/mmap.h"

#include <errno.h>
#include <unistd.h>

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
    XME_CHECK(NULL != outAddr, XME_STATUS_INVALID_PARAMETER);

    *outAddr = mmap(addr, length, protection, flags | MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

    if (MAP_FAILED == *outAddr)
    {
        XME_CHECK(EINVAL != errno, XME_STATUS_INVALID_PARAMETER);
        XME_CHECK(EAGAIN != errno, XME_STATUS_PERMISSION_DENIED);
        XME_CHECK(ENFILE != errno, XME_STATUS_OUT_OF_RESOURCES);
        XME_CHECK(ENOMEM != errno, XME_STATUS_OUT_OF_RESOURCES);

        XME_ASSERT(0 == errno);
    }

    return XME_STATUS_SUCCESS;
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
    XME_CHECK(NULL != outAddr, XME_STATUS_INVALID_PARAMETER);

    *outAddr = mmap(addr, length, protection, flags | MAP_SHARED, fileDescriptor, offset);

    if (MAP_FAILED == *outAddr)
    {
        XME_CHECK(EINVAL != errno, XME_STATUS_INVALID_PARAMETER);
        XME_CHECK(EBADF != errno, XME_STATUS_INVALID_PARAMETER);
        XME_CHECK(EACCES != errno, XME_STATUS_PERMISSION_DENIED);
        XME_CHECK(EAGAIN != errno, XME_STATUS_PERMISSION_DENIED);
        XME_CHECK(EPERM != errno, XME_STATUS_PERMISSION_DENIED);
        XME_CHECK(ENFILE != errno, XME_STATUS_OUT_OF_RESOURCES);
        XME_CHECK(ENOMEM != errno, XME_STATUS_OUT_OF_RESOURCES);
        XME_CHECK(ENODEV != errno, XME_STATUS_UNSUPPORTED);

        XME_ASSERT(0 == errno);
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_mmap_syncMappedMemory
(
    void* addr,
    size_t length,
    uint32_t flags
)
{
    if (0 != msync(addr, length, flags))
    {
        XME_CHECK(EINVAL != errno, XME_STATUS_INVALID_PARAMETER);
        XME_CHECK(ENOMEM != errno, XME_STATUS_INVALID_CONFIGURATION);
        XME_CHECK(EBUSY != errno, XME_STATUS_WOULD_BLOCK);

        XME_ASSERT(0 == errno);
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_mmap_unmapMemory
(
    void* addr,
    size_t length
)
{
    if (0 != munmap(addr, length))
    {
        XME_CHECK(EINVAL != errno, XME_STATUS_INVALID_PARAMETER);

        XME_ASSERT(0 == errno);
    }

    return XME_STATUS_SUCCESS;
}

uint32_t
xme_hal_mmap_getSystemPageSize(void)
{
    return (uint32_t) sysconf(_SC_PAGE_SIZE);
}