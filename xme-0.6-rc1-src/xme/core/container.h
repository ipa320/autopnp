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
 * $Id: container.h 4664 2013-08-13 09:06:08Z ruiz $
 */

/**
 * \file
 *         Software container abstraction.
 */

#ifndef XME_CORE_CONTAINER_H
#define XME_CORE_CONTAINER_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_core_binarySize_t
 *
 * \brief The size of a binary in bytes
 */
typedef uint32_t xme_core_binarySize_t;

/**
 * \typedef xme_core_container_t
 *
 * \brief  Locally valid identifier of a container.
 */
typedef enum
{
    XME_CORE_CONTAINER_INVALID_CONTAINER_CONTEXT = 0, ///< Invalid container context.
    XME_CORE_CONTAINER_MAX_CONTAINER_CONTEXT = XME_MAX_SYSTEM_VALUE ///< Largest possible container context.
}
xme_core_container_t;

#endif // #ifndef XME_CORE_CONTAINER_H
