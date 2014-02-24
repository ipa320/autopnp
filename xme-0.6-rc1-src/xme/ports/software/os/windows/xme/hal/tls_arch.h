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
 * $Id: tls_arch.h 4664 2013-08-13 09:06:08Z ruiz $
 */

/**
 * \file
 *         Thread-local storage abstraction (platform specific part: Windows).
 */

#ifndef XME_HAL_TLS_ARCH_H
#define XME_HAL_TLS_ARCH_H

/**
 * \addtogroup hal_tls
 * @{
 */

#ifndef XME_HAL_TLS_H
    #error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_HAL_TLS_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <Windows.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_hal_tls_index_t
 *
 * \brief  Platform dependent thread-local storage index.
 */
typedef DWORD xme_hal_tls_index_t;

/**
 * @}
 */

#endif // #ifndef XME_HAL_TLS_ARCH_H
