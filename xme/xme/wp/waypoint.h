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
 * $Id: waypoint.h 3027 2013-04-24 11:30:28Z wiesmueller $
 */

#ifndef XME_WP_WAYPOINT_H
#define XME_WP_WAYPOINT_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_wp_waypoint_instanceId_t
 *
 * \brief  Identifier of a waypoint configuration.
 *
 * \details Used in the <CODE>addConfig()</CODE> and <CODE>run()</CODE> methods
 *          of waypoints to identify a configuration instance.
 *          A instanceId created by a waypoint is only valid for that waypoint.
 */
typedef enum
{
    XME_WP_WAYPOINT_INSTANCEID_INVALID = 0, ///< Invalid configuration instance id.
    XME_WP_WAYPOINT_INSTANCEID_MAX = XME_MAX_SYSTEM_VALUE ///< Largest possible configuration instance id.
}
xme_wp_waypoint_instanceId_t;

#endif /* XME_WP_WAYPOINT_H */
