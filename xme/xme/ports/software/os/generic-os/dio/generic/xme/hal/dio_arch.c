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
 * $Id: dio_arch.c 3458 2013-05-23 09:37:04Z ruiz $
 */

/**
 * \file
 *         Digital I/O abstraction (platform specific part: generic OS based
 *         implementation).
 */

/**
 * \addtogroup hal_dio 
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/dio_arch.h"

#include "xme/defines.h"
#include "xme/hal/include/io.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_hal_dio_arch_portState_t
 *
 * \brief  Virtual port state.
 */
typedef struct
{
	uint8_t ddr; ///< Virtual data direction register.
	uint8_t port; ///< Virtual output register (identical to input register).
	uint8_t pullup; ///< Virtual pull-up resistor register.
	uint8_t pulldown; ///< Virtual pull-down resistor register.
}
xme_hal_dio_arch_portState_t;

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
static xme_hal_dio_arch_portState_t xme_hal_dio_arch_ports[1] =
{
	0x00, ///< ddr (all pins as input)
	0x00, ///< port (all pins low)
	0xFF, ///< pullup (all pull-up resistors set)
	0x00, ///< pulldown (no pull-down resistors set)
}; ///< the port state. 

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_dio_getPortConfiguration(xme_hal_io_port_t port, xme_hal_dio_portConfiguration_t* configuration)
{
	XME_ASSERT(NULL != configuration);
	XME_CHECK(port < XME_HAL_DIO_NUM_PORTS, XME_STATUS_INVALID_PARAMETER);
	*configuration = xme_hal_dio_arch_ports[port].ddr;
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_dio_setPortConfiguration(xme_hal_io_port_t port, xme_hal_dio_portConfiguration_t configuration)
{
	XME_CHECK(port < XME_HAL_DIO_NUM_PORTS, XME_STATUS_INVALID_PARAMETER);
	xme_hal_dio_arch_ports[port].ddr = configuration;
	XME_LOG(XME_LOG_VERBOSE, "Setting DIO port #%u direction register to 0x%02X\n", port, configuration);
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_dio_getPinConfiguration(xme_hal_io_portPin_t portPin, xme_hal_dio_pinConfiguration_t* configuration)
{
	XME_ASSERT(NULL != configuration);
	XME_CHECK(portPin.port < XME_HAL_DIO_NUM_PORTS, XME_STATUS_INVALID_PARAMETER);
	*configuration = (xme_hal_dio_arch_ports[portPin.port].ddr & _BV(portPin.pin)) ? XME_HAL_DIO_PIN_OUTPUT :
		(
			(xme_hal_dio_arch_ports[portPin.port].pullup & _BV(portPin.pin)) ? XME_HAL_DIO_PIN_INPUT_PULLUP :
			(
				(xme_hal_dio_arch_ports[portPin.port].pulldown & _BV(portPin.pin)) ? XME_HAL_DIO_PIN_INPUT_PULLDOWN :
					XME_HAL_DIO_PIN_INPUT
			)
		);
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_dio_setPinConfiguration(xme_hal_io_portPin_t portPin, xme_hal_dio_pinConfiguration_t configuration)
{
	// This implementation never returns XME_CORE_INVALID_CONFIGURATION

	xme_hal_dio_arch_portState_t* portState;
	uint8_t mask = _BV(portPin.pin);

	XME_CHECK(portPin.port < XME_HAL_DIO_NUM_PORTS, XME_STATUS_INVALID_PARAMETER);
	portState = &xme_hal_dio_arch_ports[portPin.port];
	switch (configuration)
	{
		case XME_HAL_DIO_PIN_OUTPUT:
			portState->ddr |= mask;
			XME_LOG(XME_LOG_VERBOSE, "Configuring DIO pin #%u.%u for output\n", portPin.port, portPin.pin);
			break;

		case XME_HAL_DIO_PIN_INPUT_PULLUP:
			portState->pullup |= mask;
			portState->pulldown &= ~mask;
			portState->ddr &= ~mask;
			XME_LOG(XME_LOG_VERBOSE, "Configuring DIO pin #%u.%u for input with pull-up\n", portPin.port, portPin.pin);
			break;

		case XME_HAL_DIO_PIN_INPUT_PULLDOWN:
			portState->pullup &= ~mask;
			portState->pulldown |= mask;
			portState->ddr &= ~mask;
			XME_LOG(XME_LOG_VERBOSE, "Configuring DIO pin #%u.%u for input with pull-down\n", portPin.port, portPin.pin);
			break;

		case XME_HAL_DIO_PIN_INPUT:
			portState->ddr &= ~mask;
			XME_LOG(XME_LOG_VERBOSE, "Configuring DIO pin #%u.%u for input without pull\n", portPin.port, portPin.pin);
			break;
	}
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_dio_getPort(xme_hal_io_port_t port, xme_hal_dio_portState_t* state)
{
	XME_ASSERT(NULL != state);
	XME_CHECK(port < XME_HAL_DIO_NUM_PORTS, XME_STATUS_INVALID_PARAMETER);
	*state = xme_hal_dio_arch_ports[port].port;
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_dio_setPort(xme_hal_io_port_t port, xme_hal_dio_portState_t state)
{
	XME_CHECK(port < XME_HAL_DIO_NUM_PORTS, XME_STATUS_INVALID_PARAMETER);
	xme_hal_dio_arch_ports[port].port = state;
	XME_LOG(XME_LOG_VERBOSE, "Setting DIO port #%u value to 0x%02X\n", port, state);
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_dio_getPin(xme_hal_io_portPin_t portPin, bool* state)
{
	XME_ASSERT(NULL != state);
	XME_CHECK(portPin.port < XME_HAL_DIO_NUM_PORTS, XME_STATUS_INVALID_PARAMETER);
	*state = xme_hal_dio_arch_ports[portPin.port].port & _BV(portPin.pin);
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_dio_setPin(xme_hal_io_portPin_t portPin, bool state)
{
	XME_CHECK(portPin.port < XME_HAL_DIO_NUM_PORTS, XME_STATUS_INVALID_PARAMETER);
	if (state)
	{
		xme_hal_dio_arch_ports[portPin.port].port |= _BV(portPin.pin);
	}
	else
	{
		xme_hal_dio_arch_ports[portPin.port].port &= ~_BV(portPin.pin);
	}
	XME_LOG(XME_LOG_VERBOSE, "Setting DIO pin #%u.%u value to %s\n", portPin.port, portPin.pin, state ? "high" : "low");
	return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
