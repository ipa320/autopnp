/*
 * Copyright (c) 2011-2013, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: log_arch.h 6199 2013-12-19 20:46:46Z geisinger $
 */

/**
 * \file
 *         Logging system abstraction (architecture specific part: generic OS
 *                                     based implementation).
 *
 */

#ifndef XME_CORE_LOG_ARCH_H
#define XME_CORE_LOG_ARCH_H

#ifndef XME_CORE_LOG_H
    #error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_CORE_LOG_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

extern xme_core_log_logCallback_t xme_core_log_logCallback; ///< Logging callback function.

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_LOG_ARCH_H
