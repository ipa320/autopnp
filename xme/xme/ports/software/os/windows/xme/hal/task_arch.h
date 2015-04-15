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
 * $Id: task_arch.h 2022 2012-12-13 19:29:49Z geisinger $
 */

/**
 * \file
 *         Task abstraction (architecture specific part: Windows implementation).
 */

#ifndef XME_HAL_TASK_ARCH_H
#define XME_HAL_TASK_ARCH_H

#ifndef XME_HAL_TASK_H
	#error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_HAL_TASK_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <Windows.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_hal_taskId_t
 *
 * \brief  Represents a platform dependent task identifier.
 *
 * \note   The value of this handle is platform dependent. Hence, only use it
 *         for comparing task handles from a specific node with each other
 *         (e.g., as a key in a data structure) and do not draw any further
 *         conclusions from the value itself.
 */
typedef DWORD xme_hal_taskId_t;

#endif // #ifndef XME_HAL_TASK_ARCH_H
