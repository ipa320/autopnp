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
 * $Id: sync_arch.h 6282 2014-01-09 16:43:46Z geisinger $
 */

/**
 * \file
 *         Synchronization abstraction (architecture specific part:
 *         POSIX implementation).
 */

#ifndef XME_HAL_SYNC_ARCH_H
#define XME_HAL_SYNC_ARCH_H

#ifndef XME_HAL_SYNC_H
	#error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_HAL_SYNC_H

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
// Documented in sync.h
// POSIX supports both recursive and non-recursive mutexes;
// the current implementation uses the non-recursive variant.
#define XME_HAL_SYNC_RECURSIVE_LOCKING (0)

#endif // #ifndef XME_HAL_NET_ARCH_H
