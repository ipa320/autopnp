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
 * $Id: energy.h 2693 2013-03-17 01:51:56Z camek $
 */

/**
 * \file
 * \brief Energy management abstraction.
 */

#ifndef XME_HAL_ENERGY_H
#define XME_HAL_ENERGY_H

/**
 * \defgroup hal_energy Energy management
 * @{
 *
 * \brief Energy management abstraction. This component allow the activation
 *        of energy saving operations, such as sleep, hibernate, wakeup and
 *        other operations related with node energy management.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Sends the device to sleep mode. Sleep mode can only be left via
 *         external or internal events (e.g., interrupts).
 */
void
xme_hal_energy_sleep(void);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_HAL_ENERGY_H
