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
 * $Id: sleep.h 5190 2013-09-26 13:10:19Z camek $
 */

/** 
 * \file
 * \brief Sleep abstraction
 */

#ifndef XME_CORE_SLEEP_H
#define XME_CORE_SLEEP_H

/**
 * \defgroup hal_sleep Sleep Abstraction. Public sleep API. 
 * @{
 *
 * \brief This component is used to force tasks to be in a sleep status. 
 */


/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdint.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Halts execution of the current task for the specified amount of
 *         time.
 *
 * \param  sleepTime Time interval to defer execution of the current task.
 */
void
xme_hal_sleep_sleep
(
    xme_hal_time_timeInterval_t sleepTime
);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_CORE_SLEEP_H
