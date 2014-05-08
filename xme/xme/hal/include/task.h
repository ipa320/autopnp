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
 * $Id: task.h 7459 2014-02-18 10:25:58Z geisinger $
 */

/** 
 * \file
 * \brief Task abstraction.
 */

#ifndef XME_HAL_TASK_H
#define XME_HAL_TASK_H

/**
 * \defgroup hal_task Task abstraction
 * @{
 *
 * \brief  Provides an abstraction for task to be scheduled. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/task_arch.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

typedef uint32_t xme_hal_processId_t;
/**
 * \brief Sets the process-local ID
 *
 */
void
xme_hal_task_setCurrentProcessId(xme_hal_processId_t);

/**
 * \brief Retrieves the identification of the calling process.
 *
 * \return Returns the process identification.
 */
xme_hal_processId_t
xme_hal_task_getCurrentProcessId(void);



/**
 * \brief Retrieves the identification of the calling task.
 *
 * \return Returns the task identification.
 */
xme_hal_taskId_t
xme_hal_task_getCurrentTaskId(void);

XME_EXTERN_C_END


/**
 * @}
 */


#endif // #ifndef XME_HAL_TASK_H
