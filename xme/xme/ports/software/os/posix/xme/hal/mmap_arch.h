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
 * $Id: mmap_arch.h 6626 2014-02-05 14:36:43Z geisinger $
 */

/**
 * \file
 *         Memory mapping abstraction (platform specific part: POSIX).
 */

#ifndef XME_HAL_MMAP_ARCH_H
#define XME_HAL_MMAP_ARCH_H

/**
 * \addtogroup hal_mmap
 * @{
 */

#ifndef XME_HAL_MMAP_H
    #error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_HAL_MMAP_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <sys/mman.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define XME_HAL_MMAP_PROTECTION_NONE PROT_NONE
#define XME_HAL_MMAP_PROTECTION_READ PROT_READ
#define XME_HAL_MMAP_PROTECTION_WRITE PROT_WRITE
#define XME_HAL_MMAP_PROTECTION_EXECUTE PROT_EXEC

#define XME_HAL_MMAP_FIXED MAP_FIXED
#define XME_HAL_MMAP_ANONYMOUS MAP_ANONYMOUS

#define XME_HAL_MAP_SYNC_ASYNC MS_ASYNC
#define XME_HAL_MAP_SYNC_SYNC MS_SYNC
#define XME_HAL_MAP_SYNC_INVALIDATE MS_INVALIDATE

/**
 * @}
 */

#endif // #ifndef XME_HAL_MMAP_ARCH_H
