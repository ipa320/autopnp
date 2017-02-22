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
 * $Id: waypointConfigInfrastructure.h 5644 2013-10-25 15:20:07Z ruiz $
 */

/**
 * \file
 *         Waypoint Support Infrastructure Header
 */

#ifndef XME_WP_WPCONFIGINFRASTRUCTURE_H
#define XME_WP_WPCONFIGINFRASTRUCTURE_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/mem.h"

#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerConfigurator.h"
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/broker/include/broker.h"
#include "xme/core/topic.h"
#include "xme/core/topicData.h"

#include "xme/wp/waypoint.h"
#include "xme/wp/udp/include/udpSend.h"
#include "xme/wp/marshal/include/marshaler.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN

/**
 * \brief Add configuration for waypoints for login/PlugNPlay infrastructures.
 *
 * \details This function adds the waypoint infrastructure for/to newly joined nodes.
 *
 * \param ipAddress the ip Address to be added.
 * \param portAddress the corresponding port to be added.
 * \param channelID The channel identifier. 
 * \param topic topic for which this is to be added.
 * 
 * \retval XME_STATUS_SUCCESS if the configuration exists or has been successfully added.
 * \retval XME_STATUS_INVALID_CONFIGURATION if the configuration failed to add.
 */
xme_status_t
xme_wp_wci_addConfig
(
    uint32_t ipAddress,
    uint16_t portAddress,
    xme_core_channelId_t channelID,
    xme_core_topic_t topic
);

XME_EXTERN_C_END

#endif // #ifndef XME_WP_WPCONFIGINFRASTRUCTURE_H
