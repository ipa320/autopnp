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
 * $Id: readSensorValueCommon.h 6157 2013-12-18 16:17:39Z geisinger $
 */

/**
 * \file
 *         Header file for common functionality in readSensorValue functions.
 */

#ifndef SENSORMONITOR_ADV_COMMON_READSENSORVALUECOMMON_H
#define SENSORMONITOR_ADV_COMMON_READSENSORVALUECOMMON_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Fills the sensorType member of the given sensorData structure
 *        according to the type of sensor.
 *
 * \param[in,out] componentConfig Address of the component configuration
 *                structure to fill with information. Must not be NULL.
 */
void
sensorMonitor_adv_common_fillSensorType
(
    xme_core_component_config_t* const componentConfig
);

/**
 * \brief Fills the hostName member of the given sensorData structure according
 *        to the host this application is running on.
 *
 * \param[in,out] componentConfig Address of the component configuration
 *                structure to fill with information. Must not be NULL.
 */
void
sensorMonitor_adv_common_fillSensorHostName
(
    xme_core_component_config_t* const componentConfig
);

/**
 * \brief Fills the given sensorData structure with information about the
 *        exact sensor data being acquired.
 *
 * \param[in,out] componentConfig Address of the component configuration
 *                structure to fill with information. Must not be NULL.
 */
void
sensorMonitor_adv_common_fillSensorTopic
(
    xme_core_component_config_t* const componentConfig
);

/**
 * \brief Reads and returns the sensor value corresponding to the given
 *        component configuration.
 *
 * \param[in] componentConfig Address of the component configuration
 *            structure for which to return the sensor value.
 *            Must not be NULL.
 *
 * \return Sensor reading corresponding to the given component configuration.
 */
uint64_t
sensorMonitor_adv_common_fillSensorValue
(
    const xme_core_component_config_t* const componentConfig
);

XME_EXTERN_C_END

#endif // #ifndef SENSORMONITOR_ADV_COMMON_READSENSORVALUECOMMON_H
