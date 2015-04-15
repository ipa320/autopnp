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
 * $Id: channelInjector.h 6684 2014-02-07 14:34:26Z geisinger $
 */

/**
 * \file
 *
 * \brief Waypoint that injects an attribute representing the channel identifier
 *        into a data packet.
 */

#ifndef XME_WP_CHANNEL_CHANNELINJECTOR_H
#define XME_WP_CHANNEL_CHANNELINJECTOR_H

/**
 * \defgroup wp_channel_channelInjector Channel Injector waypoint.
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"
#include "xme/core/coreTypes.h"
#include "xme/core/dataManagerTypes.h"

#include "xme/wp/waypoint.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initialize this waypoint class.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INTERNAL_ERROR if an error occurred.
 */
xme_status_t
xme_wp_channel_channelInjector_init(void);

/**
 * \brief Execute the given instance of this waypoint class.
 *
 * \details This function reads a data packet from the input port listed in
 *          the waypoint configuration specified by instanceId, injects a
 *          channel attribute and writes the result to the respective
 *          output port.
 *
 * \note Before calling this function, a configuration for the given
 *       instanceId needs to be added by calling
 *       xme_wp_channel_channelInjector_addConfig().
 *
 * \param[in] instanceId Identifier of the configuration for which to execute
 *            the waypoint, as inidcated by the respective call to
 *            xme_wp_channel_channelInjector_addConfig().
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if no configuration has been added for
 *         the given instanceId.
 * \retval XME_STATUS_INTERNAL_ERROR if there was an error reading from the
 *         input port or writing to the output port. When this value is
 *         returned, the output port value must be considered invalid.
 */
xme_status_t
xme_wp_channel_channelInjector_run
(
    xme_wp_waypoint_instanceId_t instanceId
);

/**
 * \brief Add a new configuration to this waypoint class.
 *
 * \param[in] instanceId Address of a variable where the identifier for the
 *            newly added configuration is written to. Only valid if the
 *            function returns XME_CORE_STATUS_SUCCESS.
 * \param[in] inputPort Input data port.
 * \param[in] outputPort Output data port.
 * \param[in] topic Topic associated to this configuration.
 * \param[in] sizeOfTopic Size of the topic data which will appear at the data
 *            port (attributes not considered).
 * \param[in] buffer Buffer space to be used to read data from the input port.
 * \param[in] sizeOfBuffer Size of the buffer space (should include space for
 *            attributes).
 * \param[in] injectedChannelID Channel identifier which is injected into the
 *            attributes when writing to the output port.
 *
 * \retval XME_STATUS_SUCCESS if the configuration has been added successfully.
 * \retval XME_STATUS_INVALID_PARAMETER if the size of the buffer is incorrect.
 * \retval XME_STATUS_OUT_OF_RESOURCES if the configuration could not be added
 *         due to resource constraints (e.g., not enough memory to store entry).
 */
xme_status_t
xme_wp_channel_channelInjector_addConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_dataManager_dataPacketId_t inputPort,
    xme_core_dataManager_dataPacketId_t outputPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    void* buffer,
    uint16_t sizeOfBuffer,
    xme_core_channelId_t injectedChannelID
);

/**
 * \brief Removes a configuration of this waypoint.
 *
 * \param[in] instanceId InstanceID of the configuration which has to be removed.
 *
 * \retval XME_STATUS_SUCCESS if configuration was successfully removed.
 * \retval XME_STATUS_INVALID_HANDLE if the given instanceID was invalid.
 */
xme_status_t
xme_wp_channel_channelInjector_removeConfig
(
    xme_wp_waypoint_instanceId_t instanceId
);

/**
 * \brief Frees all resources occupied by this waypoint class.
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_wp_channel_channelInjector_fini(void);

XME_EXTERN_C_END

/** @} */

#endif // #ifndef XME_WP_CHANNEL_CHANNELINJECTOR_H
