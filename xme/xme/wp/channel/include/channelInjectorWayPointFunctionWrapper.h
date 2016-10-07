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
 * $Id: channelInjectorWayPointFunctionWrapper.h 6684 2014-02-07 14:34:26Z geisinger $
 */

/**
 * \file
 *         Function wrapper - a generic abstraction for one executable function
 *              scheduled by the execution manager.
 */

#ifndef XME_WP_CHANNEL_CHANNELINJECTORWAYPOINTFUNCTIONWRAPPER_H
#define XME_WP_CHANNEL_CHANNELINJECTORWAYPOINTFUNCTIONWRAPPER_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Executes an instance of the Channel Injector waypoint.
 *
 * \param[in] param Parameters that indicate which waypoint instance to
 *            execute.
 */
void
xme_wp_channel_channelInjector_channelInjectorWayPointFunctionWrapper_execute
(
    void* param
);

XME_EXTERN_C_END

#endif // #ifndef XME_WP_CHANNEL_CHANNELINJECTORWAYPOINTFUNCTIONWRAPPER_H
