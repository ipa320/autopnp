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
 * $Id: dio.h 2855 2013-04-05 23:44:00Z tum.martin.perzl $
 */

/** 
 * \file 
 * \brief Digital I/O abstraction.
 *
 */

#ifndef XME_HAL_DIO_H
#define XME_HAL_DIO_H

/** 
 * \defgroup hal_dio Digital I/O.
 * @{ 
 *
 * \brief Digital I/O abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/hal/include/io.h"
#include "xme/hal/dio_arch.h"

#include <stdbool.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/** 
 * \typedef xme_hal_dio_portConfiguration_t
 *
 * \brief  Digital I/O port configuration. A zero bit indicates that the
 *         respective pin is configured for input. A one bit indicates that
 *         the respective pin is configured for output.
 */
typedef uint8_t xme_hal_dio_portConfiguration_t;

/** 
 * \enum xme_hal_dio_pinConfiguration_t
 * \brief Digital I/O pin configuration.
 */
typedef enum 
{
	XME_HAL_DIO_PIN_RESET = 0, ///< Pin is configured according to the reset (i.e., default) configuration.
	XME_HAL_DIO_PIN_INPUT = 1, ///< Pin is configured as free-running input.
	XME_HAL_DIO_PIN_INPUT_PULLUP = 0, ///< Pin is configured as input with pull-up resistor enabled.
	XME_HAL_DIO_PIN_INPUT_PULLDOWN = 2, ///< Pin is configured as input with pull-down resistor enabled.
	XME_HAL_DIO_PIN_OUTPUT = 4 ///< Pin is configured as output.
} xme_hal_dio_pinConfiguration_t;

/** 
 * \typedef xme_hal_dio_portState_t
 *
 * \brief  Digital I/O port state. A zero bit indicates that the respective
 *         pin is currently low. A one bit indicates that the respective pin
 *         is high.
 */
typedef uint8_t xme_hal_dio_portState_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Retrieves the current pin configuration of a digital I/O port.
 *
 * \param  port Port to retrieve configuration for.
 * \param  configuration Address of a variable where to put the configuration
 *         of the port. A zero bit indicates that the respective pin is
 *         configured for input. A one bit indicates that the respective pin is
 *         configured for output.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if the given port or output variable address is invalid.
 */
xme_status_t
xme_hal_dio_getPortConfiguration
(
	xme_hal_io_port_t port, 
	xme_hal_dio_portConfiguration_t* configuration
);

/**
 * \brief  Configures the pins of a digital I/O port for input or output.
 *         The function does not affect the pull-up or pull-down setup of
 *         the port.
 *
 * \param  port Port to configure.
 * \param  configuration New configuration of the port.
 *         A zero bit indicates that the respective pin is configured for
 *         input. A one bit indicates that the respective pin is configured
 *         for output.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the operation completed successfully
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if the given port is invalid
 */
xme_status_t
xme_hal_dio_setPortConfiguration(xme_hal_io_port_t port, xme_hal_dio_portConfiguration_t configuration);

/**
 * \brief  Retrieves the current configuration of a digital I/O pin.
 *
 * \param  portPin Pin to retrieve configuration for.
 * \param  configuration Address of a variable where to put the configuration
 *         of the pin.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if the given pin or output variable address is invalid.
 */
xme_status_t
xme_hal_dio_getPinConfiguration(xme_hal_io_portPin_t portPin, xme_hal_dio_pinConfiguration_t* configuration);

/**
 * \brief  Configures a digital I/O pin for input or output.
 *
 * \param  portPin Pin to configure.
 * \param  configuration New configuration of the pin.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if the given pin is invalid.
 * \retval XME_CORE_STATUS_INVALID_CONFIGURATION if the given configuration is
 *         not feasible (e.g., XME_HAL_DIO_PIN_INPUT_PULLDOWN is requested, but
 *         the hardware does not offer this functionality).
 */
xme_status_t
xme_hal_dio_setPinConfiguration(xme_hal_io_portPin_t portPin, xme_hal_dio_pinConfiguration_t configuration);

/**
 * \brief  Retrieves the port's current state.
 *         If the port is configured for input, the result depends on the
 *         high/low state of the electrical lines attached to the port pins.
 *         If the port is configured for output, the result is the most
 *         recent value set by calls to xme_hal_dio_setPort() (or, for
 *         individual pins, xme_hal_dio_setPin()) or the port's default
 *         value in case no call to this function was performed before.
 *
 * \param  port Port to retrieve state for.
 * \param  state Address of a variable where to put the state for the port
 *         pins. A zero bit indicates that the respective pin is currently
 *         low. A one bit indicates that the respective pin is high.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the operation completed successfully
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if the given port or output variable address is invalid
 */
xme_status_t
xme_hal_dio_getPort(xme_hal_io_port_t port, xme_hal_dio_portState_t* state);

/**
 * \brief  Sets a port to the given state.
 *
 * \param  port Port to set state for.
 * \param  state New state for all pins of the port. A value of true indicates
 *         that the pins should go high, a value of false indicates that they
 *         should become low.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if the given port is invalid.
 */
xme_status_t
xme_hal_dio_setPort(xme_hal_io_port_t port, xme_hal_dio_portState_t state);

/**
 * \brief  Retrieves the pin's current state.
 *         If the pin is configured for input, the result depends on the
 *         high/low state of the electrical line attached to the pin.
 *         If the pin is configured for output, the result is the most
 *         recent value set by a call to xme_hal_dio_setPin() (or, for
 *         the whole port, xme_hal_dio_setPort()) or the pin's default
 *         value in case no call to this function was performed before.
 *
 * \param  portPin Pin to retrieve state for.
 * \param  state Address of a variable where to put the state for the pin.
 *         A value of true indicates that the pin is currently high, a
 *         value of false indicates that is is low.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if the given pin or output variable address is invalid.
 */
xme_status_t
xme_hal_dio_getPin(xme_hal_io_portPin_t portPin, bool* state);

/**
 * \brief  Sets a pin to the given state.
 *
 * \param  portPin Pin to set state for.
 * \param  state New state for the pin. A value of true indicates that the pin
 *         should go high, a value of false indicates that it should become low.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if the given pin is invalid.
 */
xme_status_t
xme_hal_dio_setPin(xme_hal_io_portPin_t portPin, bool state);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_HAL_DIO_H
