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
 * $Id: random_arch.h 3345 2013-05-17 12:07:58Z geisinger $
 */

/**
 * \file
 *         Random number generator  abstraction (architecture specific part:
 *         generic OS based implementation).
 */

#ifndef XME_HAL_RANDOM_ARCH_H
#define XME_HAL_RANDOM_ARCH_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def    XME_HAL_RANDOM_RAND_MAX
 *
 * \brief  Maximum value returned by xme_hal_random_rand().
 *
 * \note   XME_HAL_RANDOM_RAND_MAX is defined to RAND_MAX, which,
 *         according to documentation, "is granted to be at least 32767".
 */
#define XME_HAL_RANDOM_RAND_MAX ((uint16_t)-1)

#endif // #ifndef XME_HAL_RANDOM_ARCH_H
