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
 * $Id: channelSelector.h 6684 2014-02-07 14:34:26Z geisinger $
 */

/**
 * \file
 *
 * \brief Waypoint that selects one of a set of output ports to copy the
 *        received data to depending on the channel identifier.
 *
 */

#ifndef XME_WP_CHANNEL_CHANNELSELECTOR_H
#define XME_WP_CHANNEL_CHANNELSELECTOR_H

/**
 * \defgroup wp_channel_channelSelector Channel Selector waypoint.
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"
#include "xme/core/coreTypes.h"
#include "xme/core/dataManagerTypes.h"

#include "xme/wp/waypoint.h"
#include <stdbool.h>

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
xme_wp_channel_channelSelector_init(void);

/**
 * \brief Execute the given instance of this waypoint class.
 *
 * \details This function reads a data packet from the input port listed in
 *          the waypoint configuration specified by instanceId, interprets
 *          the associated channel attribute and writes the result to the
 *          output port matching the channel identifier.
 *
 * \note Before calling this function, a configuration for the given
 *       instanceId needs to be added by calling
 *       xme_wp_channel_channelSelector_addConfig().
 *
 * \param[in] instanceId Identifier of the configuration for which to execute
 *            the waypoint, as inidcated by the respective call to
 *            xme_wp_channel_channelSelector_addConfig().
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_HANDLE if no configuration has been added for
 *         the given instanceId.
 * \retval XME_STATUS_INTERNAL_ERROR if there was an error reading from the
 *         input port or writing to the output port. When this value is
 *         returned, the output port value must be considered invalid.
 */
xme_status_t
xme_wp_channel_channelSelector_run
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
 *            NOTE: This can be set to XME_CORE_DATAMANAGER_DATAPACKETID_INVALID
 *                  if the caller knows that configuration for the given topic
 *                  already exists. Then this function does not return 
 *                  XME_STATUS_INVALID_PARAMETER.
 * \param[in] outputPort Output data port.
 * \param[in] topic Topic associated to this configuration.
 * \param[in] sizeOfTopic Size of the topic data which will appear at the data
 *            port (attributes not considered).
 * \param[in] buffer Buffer space to be used to read data from the input port.
 * \param[in] sizeOfBuffer Size of the buffer space (should include space for
 *            attributes).
 * \param[in] sourceChannelID The source channel identifier to handle.
 * \param[in] destinationChannelID The destination channel identifier to use.
 *
 * \retval XME_STATUS_SUCCESS if the configuration has been added successfully.
 * \retval XME_STATUS_INVALID_PARAMETER if the size of the buffer is incorrect
 *                                   or if input or the output port are invalid
 *                                   or if the source or destination ChannelID
 *                                      are invalid.
 * \retval XME_STATUS_OUT_OF_RESOURCES if the configuration could not be added
 *         due to resource constraints (e.g., not enough memory to store entry).
 */
xme_status_t
xme_wp_channel_channelSelector_addConfig
(
    xme_wp_waypoint_instanceId_t* instanceId,
    xme_core_dataManager_dataPacketId_t inputPort,
    xme_core_dataManager_dataPacketId_t outputPort,
    xme_core_topic_t topic,
    uint16_t sizeOfTopic,
    void* buffer,
    uint16_t sizeOfBuffer,
    xme_core_channelId_t sourceChannelID,
    xme_core_channelId_t destinationChannelID
);

/**
 * \brief Removes a configuration of this waypoint.
 * \details Since it is possible to add more than one sourceChannelID to 
 *          destinationChannelID mapping for any one instanceID so this function removes 
 *          only the mapping. The entries for the instanceID are removed once there is
 *          no mapping that uses that instanceID.
 *
 * \param[in] instanceId InstanceID of the configuration which has to be removed.
 * \param[in] sourceChannelID The source channel identifier.
 * \param[in] destinationChannelID The destination channel identifier to use.
 *
 * \retval XME_STATUS_SUCCESS if the configuration has been removed successfully.
 * \retval XME_STATUS_INVALID_PARAMETER if the passed parameters are not correct.
 * \retval XME_STATUS_INVALID_CONFIGURATION if there is no configuration with the given parameters.
 */
xme_status_t
xme_wp_channel_channelSelector_removeConfig
(
    xme_wp_waypoint_instanceId_t instanceId,
    xme_core_channelId_t sourceChannelID,
    xme_core_channelId_t destinationChannelID
);

/**
 * \brief Frees all resources occupied by this waypoint class.
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_wp_channel_channelSelector_fini(void);

/**
 * \brief Finds if Channel Selctor waypoint already has a configuration with the given topic.
 *
 * \param[in] topic Topic for which configuration has to be queried.
 * \param[in,out] inputPort This is set to input data port for the given topic if the 
 *                          configuration exists otherwise it is set to
 *                          XME_CORE_DATAMANAGER_DATAPACKETID_INVALID.
 *                          
 * \retval true if the configuration for the given topic exists.
 * \retval false if the configuration for the given topic does not exist.
 */
bool
xme_wp_channel_channelSelector_hasConfig
(
    xme_core_topic_t topic,
    xme_core_dataManager_dataPacketId_t* inputPort
);

XME_EXTERN_C_END

/** @} */

#endif // #ifndef XME_WP_CHANNEL_CHANNELSELECTOR_H
