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
 * $Id: log_arch.h 2843 2013-04-03 17:13:38Z geisinger $
 */

/**
 * \file
 *         Logging system abstraction (architecture specific part:
 *         generic embedded implementation).
 */

#ifndef XME_CORE_LOG_ARCH_H
#define XME_CORE_LOG_ARCH_H

#ifndef XME_CORE_LOG_H
	#error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_CORE_LOG_H

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

extern xme_core_log_logCallback_t xme_core_log_logCallback;

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_LOG_ARCH_H
