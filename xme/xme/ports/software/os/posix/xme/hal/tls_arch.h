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
 * $Id: tls_arch.h 3345 2013-05-17 12:07:58Z geisinger $
 */

/**
 * \file
 *         Thread-local storage abstraction (platform specific part: Posix).
 */

#ifndef XME_HAL_TLS_ARCH_H
#define XME_HAL_TLS_ARCH_H

#ifndef XME_HAL_TLS_H
	#error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_HAL_TLS_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <pthread.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \ingroup hal_tls
 *
 * \typedef xme_hal_tls_index_t
 *
 * \brief  Platform dependent thread-local storage index.
 */
typedef pthread_key_t xme_hal_tls_index_t;

#endif // #ifndef XME_HAL_TLS_ARCH_H
