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
 * $Id: device.h 4664 2013-08-13 09:06:08Z ruiz $
 */

/**
 * \file
 *         Device abstraction.
 */

#ifndef XME_CORE_DEVICE_H
#define XME_CORE_DEVICE_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_core_device_type_t
 *
 * \brief  Device type.
 */
typedef uint64_t xme_core_device_type_t;

/**
 * \typedef xme_core_device_guid_t
 *
 * \brief  Globally unique identifier for a device.
 *         Examples:
 *          - Serial number
 *          - MAC address
 */
typedef uint64_t xme_core_device_guid_t;

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def    XME_CORE_DEVICE_GUID_RANDOM
 *
 * \brief  Define that can be used for defining a device's GUID.
 *         It will cause a GUID to be randomly selected.
 */
#define XME_CORE_DEVICE_GUID_RANDOM ((xme_core_device_guid_t)-1)

#endif // #ifndef XME_CORE_DEVICE_H
