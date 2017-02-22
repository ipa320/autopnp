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
 *         Memory mapping abstraction (platform specific part: Windows).
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
#include <Windows.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define XME_HAL_MMAP_PROTECTION_NONE 0 // TODO
#define XME_HAL_MMAP_PROTECTION_READ 0 // TODO
#define XME_HAL_MMAP_PROTECTION_WRITE 0 // TODO
#define XME_HAL_MMAP_PROTECTION_EXECUTE 0 // TODO

#define XME_HAL_MMAP_FIXED 0 // TODO
#define XME_HAL_MMAP_ANONYMOUS 0 // TODO

#define XME_HAL_MAP_SYNC_ASYNC 0 // TODO
#define XME_HAL_MAP_SYNC_SYNC 0 // TODO
#define XME_HAL_MAP_SYNC_INVALIDATE 0 // TODO

/**
 * @}
 */

#endif // #ifndef XME_HAL_MMAP_ARCH_H
