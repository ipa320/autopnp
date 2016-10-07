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
 * $Id: dataChannel.h 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 *         Data channel abstraction.
 *
 */

#ifndef XME_CORE_DATACHANNEL_H
#define XME_CORE_DATACHANNEL_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
// TODO: See ticket #722
/**
 * \enum xme_core_dataChannel_t
 * \brief the data channel enumeration. 
 */
typedef enum
{
    XME_CORE_DATACHANNEL_INVALID_DATACHANNEL = 0, ///< Invalid data channel.

    // TODO: See ticket #1759
    // XME_CORE_DATACHANNEL_RESERVED_PREFIX = 1 << (sizeof(XME_HAL_TABLE_MAX_ROW_HANDLE) * 8 - 2 ), ///< Prefix marking reserved data channels.
    // XME_CORE_DATACHANNEL_NEIGHBORHOOD_ANNOUNCEMENTS = XME_CORE_DATACHANNEL_RESERVED_PREFIX | 1, ///< Data chanel for neighborhood announcements.
    XME_CORE_DATACHANNEL_NEIGHBORHOOD_ANNOUNCEMENTS = 0x2001,    ///< Data chanel for neighborhood announcements.
                                                                ///< This value is 0x2001, because bit no. 14 is set.
                                                                ///< Address space for UDP multicast only supports 14 bits in the current implementation.

    XME_CORE_DATACHANNEL_LOCAL_DATACHANNEL_PREFIX = 1U << (sizeof(XME_MAX_SYSTEM_VALUE) * 8 - 1), ///< Prefix marking locally assigned data channels.

    // TODO: This does not work as expected, see ticket #1759
    XME_CORE_DATACHANNEL_MASK = 1U << (sizeof(XME_MAX_SYSTEM_VALUE) * 8 - 1), ///< Data channel mask supporting local data channels.
    XME_CORE_DATACHANNEL_MAX_DATACHANNEL = XME_MAX_SYSTEM_VALUE ///< Largest possible data channel.
}
xme_core_dataChannel_t;

// TODO: See ticket #722
/**
 * \struct xme_core_dataChannel_properties_t
 * \brief the data channel properties. 
 */
typedef struct
{
    bool isReliable; ///< Whether the data channel is reliable (i.e., reception of data is guaranteed or an error is thrown on the sender side).
    //bool useSequenceNumbers;
    //bool useConsumptionTime;
}
xme_core_dataChannel_properties_t;

#endif // #ifndef XME_CORE_DATACHANNEL_H
