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
 * $Id: componentContext.h 3010 2013-04-23 12:00:57Z geisinger $
 */

/**
 * \file
 *         Component context abstraction.
 */

#ifndef XME_CORE_COMPONENTCONTEXT_H
#define XME_CORE_COMPONENTCONTEXT_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include "xme/hal/include/context.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/**
 * \def    XME_COMPONENT_CONTEXT
 *
 * \brief  Switches the component context temporarily to the one of the
 *         specified component and executes the code specified in body.
 *
 *         This macro must be called directly or indirectly when a component
 *         is called from the runtime system or from the execution context of
 *         another component. That component may then in turn call the runtime
 *         system. This macro ensures that in this case the runtime system can
 *         reliably detect which component made the specific call.
 *
 * \param  componentId Identifier of the component to use as context for
 *         evaluating the code in body.
 * \param  body Code to execute within the component context componentId.
 *         The code made directly or indirectly call functions from the
 *         runtime system and the runtime system will be aware that these
 *         calls are associated to the component componentId.
 */
#define XME_COMPONENT_CONTEXT(componentId, body) \
	do { \
		xme_core_component_t oldComponentId = (xme_core_component_t)xme_hal_context_getContext(); \
		xme_hal_context_setContext((xme_hal_context_contextHandle_t)componentId); \
		{ \
			body \
		} \
		xme_hal_context_setContext((xme_hal_context_contextHandle_t)oldComponentId); \
	} while (0)

#endif // #ifndef XME_CORE_COMPONENTCONTEXT_H
