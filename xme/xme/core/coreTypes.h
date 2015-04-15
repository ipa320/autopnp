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
 * $Id: coreTypes.h 4664 2013-08-13 09:06:08Z ruiz $
 */

/**
 * \file
 *         Core type definitions.
 *
 */

#ifndef XME_CORE_TYPES_H
#define XME_CORE_TYPES_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

// TODO: Check if ennumerate types are correct for defining transactionId (See Ticket #2258)

/**
 * \typedef xme_core_transactionId_t
 *
 * \brief  A transaction ID to ensure ACID properties
 */
typedef enum
{
    XME_CORE_INVALID_TRANSACTION_ID = 0, ///< Invalid transaction id.
    XME_CORE_MAX_TRANSACTION_ID = XME_MAX_SYSTEM_VALUE ///< Largest possible transaction id.
}
xme_core_transactionId_t;

// TODO: Check if ennumerate types are correct for defining metrics (See Ticket #2258)

/**
 * \typedef xme_core_metric_t
 *
 * \brief  A metric to rate the quality of a service. The values of the metric are between 1 (lowest quality) and 100 (highest quality).
 */
typedef enum
{
    XME_CORE_INVALID_METRIC = 0, ///< Invalid metric.
    XME_CORE_MAX_METRIC = 100 ///< Largest possible metric.
}
xme_core_metric_t;

// TODO: Check if ennumerate types are correct for defining channelId (See Ticket #2258)

/**
 * \typedef xme_core_channelId_t
 *
 * \brief A channel ID
 */
typedef enum
{
    XME_CORE_INVALID_CHANNEL_ID = 0, ///< Invalid channel id.
    XME_CORE_MAX_CHANNEL_ID = XME_MAX_SYSTEM_VALUE ///< Largest possible channel id.
}
xme_core_channelId_t;
//TODO(KB): is 'xme_core_channelId_t' from EA model equivalent to 'xme_core_dataChannel_t' from dataChannel.h? see issue #2024

#endif // #ifndef XME_CORE_CORE_H
