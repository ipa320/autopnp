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
 * $Id: digitalOutput.c 3172 2013-05-06 12:43:28Z ruiz $
 */

/**
 * \file
 *         Digital output pin.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/prim/digitalOutput.h"

#include "xme/hal/include/dio.h"

/******************************************************************************/
/***   Forward declarations                                                 ***/
/******************************************************************************/

/**
 * \brief sets digital output data in boolean.
 * \param data the data. 
 * \param userData user data. 
 * \return the subscription. 
 */
XME_SUBSCRIPTION
xme_prim_digitalOutput_onBoolean(void* data, void* userData);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
xme_prim_digitalOutput_create(xme_prim_digitalOutput_configStruct* config)
{
	xme_core_read("Boolean", xme_prim_digitalOutput_onBoolean, config);
}

void
xme_prim_digitalOutput_activate(xme_prim_digitalOutput_configStruct* config)
{
	xme_hal_dio_configurePin(config->pin, XME_HAL_DIO_PIN_OUPUT);
}

void
xme_prim_digitalOutput_deactivate(xme_prim_digitalOutput_configStruct* config)
{
	// Switch back to default configuration.
	// Note: If input is not always the default configuration, introduce an
	// XME_HAL_DIO_PIN_RESET flag or a separate HAL function for this.
	xme_hal_dio_setDir(config->pin, XME_HAL_DIO_PIN_INPUT);
}

void
xme_prim_digitalOutput_destroy(xme_prim_digitalOutput_configStruct* config)
{
}

XME_SUBSCRIPTION
xme_prim_digitalOutput_onBoolean(void* data, void* userData)
{
	bool* value = (bool*)data;
	xme_hal_dio_setPin(config->..., *value);
}
