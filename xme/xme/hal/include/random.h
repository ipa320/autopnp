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
 * $Id: random.h 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 * \brief Random number generator abstraction.
 */

#ifndef XME_HAL_RANDOM_H
#define XME_HAL_RANDOM_H

/**
 * \defgroup hal_random Random number generator.
 * @{
 *
 * \brief Random number generator abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/random_arch.h"

#include "xme/defines.h"

#include <stdint.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the random number generator.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the random component has been properly initialized.
 * \retval XME_CORE_STATUS_OUT_OF_RESOURCES if the random component initialization failed.
 */
xme_status_t
xme_hal_random_init(void);

/**
 * \brief  Frees resources occupied by the random number generator.
 */
void
xme_hal_random_fini(void);

/**
 * \brief  Registers the calling thread for use of the random number generator
 *         abstraction.
 *
 *         Every thread that will use the random number generator
 *         has to be registered with a call to this function.
 *         This is necessary, because on some platforms (e.g., Windows),
 *         the random number seed is thread-local.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the calling thread has been successfully registered.
 * \retval XME_CORE_STATUS_OUT_OF_RESOURCES if thread registration has failed.
 */
xme_status_t
xme_hal_random_registerThread(void);

/**
 * \brief  Deregisters the calling thread from use of the random number
 *         generator abstraction.
 */
void
xme_hal_random_deregisterThread(void);

/**
 * \brief  Generates a (pseudo-)random number in range 0 to
 *         XME_HAL_RANDOM_RAND_MAX.
 *
 * \return (Pseudo-)random number.
 */
uint16_t
xme_hal_random_rand(void);

/**
 * \brief  Generates a (pseudo-)random number in the given range
 *         (including min and max).
 *
 *         For max < min, the return value is undefined.
 *
 * \param  min Minimum value.
 * \param  max Maximum value.
 * \return (Pseudo-)random number in the given range.
 */
uint16_t
xme_hal_random_randRange
(
    uint16_t min,
    uint16_t max
);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_HAL_RANDOM_H
