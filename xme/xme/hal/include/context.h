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
 * $Id: context.h 5596 2013-10-24 06:07:26Z ruiz $
 */

/**
 * \file
 * \brief Context abstraction.
 */

#ifndef XME_HAL_CONTEXT_H
#define XME_HAL_CONTEXT_H

/**
 * \defgroup hal_context Context abstraction
 * @{
 *
 * \brief Context abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

// Context ID boundary values. 
#define XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE ((uint32_t) 0U) ///< Defines the invalid context indentifier for a component context.
#define XME_HAL_CONTEXT_MAX_CONTEXT_HANDLE ((uint32_t) XME_MAX_SYSTEM_VALUE) ///< Defines the maximum context identifier for a component context. 

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_hal_context_contextHandle_t
 *
 * \brief The context handle.
 */
typedef uint32_t xme_hal_context_contextHandle_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the context abstraction.
 *
 * \retval XME_STATUS_SUCCESS if the context abstraction has been
 *         properly initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if context abstration initialization
 *         has failed.
 */
xme_status_t
xme_hal_context_init(void);

/**
 * \brief  Frees resources occupied by the context abstraction.
 */
void
xme_hal_context_fini(void);

/**
 * \brief  Sets the current context for the calling thread.
 *
 * \param  context New context.
 *
 * \retval XME_STATUS_SUCCESS if the context has been successfully set.
 * \retval XME_STATUS_OUT_OF_RESOURCES if the context could not be set.
 * \retval XME_STATUS_INVALID_CONFIGURATION if the context abstraction
 *         has not been properly initialized.
 */
xme_status_t
xme_hal_context_setContext(xme_hal_context_contextHandle_t context);

/**
 * \brief  Returns the currently set context for the calling thread.
 *
 * \return Returns the context currently set for the calling thread.
 *         If no successful call to xme_hal_context_setContext() has
 *         been issued before calling this function,
 *         XME_HAL_CONTEXT_INVALID_CONTEXT_HANDLE is returned.
 */
xme_hal_context_contextHandle_t
xme_hal_context_getContext(void);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_HAL_CONTEXT_H
