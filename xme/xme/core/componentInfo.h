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
 * $Id: componentInfo.h 6308 2014-01-14 13:14:56Z geisinger $
 */

/**
 * \file
 *         Software component information.
 */

#ifndef XME_CORE_COMPONENTINFO_H
#define XME_CORE_COMPONENTINFO_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Returns a human-readable name of the given port type.
 *
 * \param[in] portType Port type to retrieve string representation for.
 *
 * \return Human-readable name of the given port type.
 */
const char*
xme_core_component_getPortTypeString(xme_core_component_portType_t portType);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_COMPONENTINFO_H
