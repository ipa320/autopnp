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
 * $Id: plugAndPlayClientInternalTypes.h 5894 2013-11-22 15:26:10Z gulati $
 */

/**
 * \file
 *         Plug and Play Client.
 */

#ifndef XME_CORE_PNP_PLUGANDPLAYCLIENTINTERNALTYPES_H
#define XME_CORE_PNP_PLUGANDPLAYCLIENTINTERNALTYPES_H

/**
 * \addtogroup core_pnp_pnpClient
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \brief  Table to store configuration for components.
 */
extern XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_CPV_config_t, xme_core_pnp_pnpClientConfiguration_CPV_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for masrhaler waypoint.
 */
extern XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_marshaler_config_t, xme_core_pnp_pnpClientConfiguration_marshaler_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for demarshaler waypoint.
 */
extern XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_demarshaler_config_t, xme_core_pnp_pnpClientConfiguration_demarshaler_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for udpSend waypoint.
 */
extern XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_udpSend_config_t, xme_core_pnp_pnpClientConfiguration_udpSend_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for udpReceive waypoint.
 */
extern XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_udpRecv_config_t, xme_core_pnp_pnpClientConfiguration_udpRecv_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store port mappings.
 */
extern XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_portMapping_t, xme_core_pnp_pnpClientConfiguration_portMapping_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for channel injector waypoint.
 */
extern XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_channelInjector_config_t, xme_core_pnp_pnpClientConfiguration_channelInjector_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * \brief  Table to store configuration for channel selector waypoint.
 */
extern XME_HAL_TABLE(xme_core_pnp_pnpClientConfiguration_channelSelector_config_t, xme_core_pnp_pnpClientConfiguration_channelSelector_config_table, XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX);

/**
 * @}
 */

#endif // #XME_CORE_PNP_PLUGANDPLAYCLIENTINTERNALTYPES_H
