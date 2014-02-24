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
 * $Id: uart.h 3158 2013-05-06 07:38:25Z ruiz $
 */

/**
 * \file
 * \brief  Generic header for UART abstraction.
 */

#ifndef XME_HAL_UART_H
#define XME_HAL_UART_H

/**
 * \defgroup hal_uart U(S)ART
 *
 * \brief  Universal (Synchronous/)Asynchronous Receiver Transmitter abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/**
 * \brief  Baud rates (transmission speeds) of the UART device.
 */
// More supported baud rates may be defined in the platform specific headers.
#define XME_HAL_UART_BAUD_RATE_2400    ((xme_hal_uart_baudRate_t)2400)    ///< 2.4 kBd/s.
#define XME_HAL_UART_BAUD_RATE_4800    ((xme_hal_uart_baudRate_t)4800)    ///< 4.8 kBd/s.
#define XME_HAL_UART_BAUD_RATE_9600    ((xme_hal_uart_baudRate_t)9600)    ///< 9.6 kBd/s.
#define XME_HAL_UART_BAUD_RATE_14400   ((xme_hal_uart_baudRate_t)14400)   ///< 14.4 kBd/s.
#define XME_HAL_UART_BAUD_RATE_19200   ((xme_hal_uart_baudRate_t)19200)   ///< 19.2 kBd/s.
#define XME_HAL_UART_BAUD_RATE_28800   ((xme_hal_uart_baudRate_t)28800)   ///< 28.8 kBd/s.
#define XME_HAL_UART_BAUD_RATE_38400   ((xme_hal_uart_baudRate_t)38400)   ///< 38.4 kBd/s.
#define XME_HAL_UART_BAUD_RATE_57600   ((xme_hal_uart_baudRate_t)57600)   ///< 57.6 kBd/s.
#define XME_HAL_UART_BAUD_RATE_76800   ((xme_hal_uart_baudRate_t)76800)   ///< 76.8 kBd/s.
#define XME_HAL_UART_BAUD_RATE_115200  ((xme_hal_uart_baudRate_t)115200)  ///< 115.2 kBd/s.

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \brief  UART descriptor.
 */
typedef struct
{
	//xme_hal_uart_archDescr_t arch;
}
xme_hal_uart_descr_t;

/**
 * \brief  Baud rate (transmission speed) of the UART device.
 */
typedef uint32_t
xme_hal_uart_baudRate_t;

/**
 * \brief  Mode of the UART device (e.g., ansynchronous or synchronous).
 */
typedef enum
{
	XME_HAL_UART_MODE_ASYNC,   ///< Asynchronous normal mode.
	XME_HAL_UART_MODE_ASYNC2X, ///< Asynchronous double speed mode.
	XME_HAL_UART_MODE_SYNC     ///< Synchronous master mode.
}
xme_hal_uart_mode_t;

/**
 * \brief  Type of parity check.
 */
typedef enum
{
	XME_HAL_UART_PARITY_NONE, ///< No parity bit.
	XME_HAL_UART_PARITY_EVEN, ///< Even parity.
	XME_HAL_UART_PARITY_ODD   ///< Odd parity.
}
xme_hal_uart_parity_t;

/**
 * \brief  Number of stop bits.
 */
typedef enum
{
	XME_HAL_UART_STOP_BITS_1 = 1, ///< 1 stop bit.
	XME_HAL_UART_STOP_BITS_2 = 2  ///< 2 stop bits.
}
xme_hal_uart_stopBits_t;

/**
 * \brief  Number of data bits.
 */
typedef enum
{
	XME_HAL_UART_DATA_BITS_5 = 5, ///< 5 data bits.
	XME_HAL_UART_DATA_BITS_6 = 6, ///< 6 data bits.
	XME_HAL_UART_DATA_BITS_7 = 7, ///< 7 data bits.
	XME_HAL_UART_DATA_BITS_8 = 8, ///< 8 data bits.
	XME_HAL_UART_DATA_BITS_9 = 9  ///< 9 data bits.
}
xme_hal_uart_dataBits_t;

/**
 * \brief  Type of clock polarity.
 */
typedef enum
{
	XME_HAL_UART_CLOCK_POLARITY_RISING, ///< Rising clock polarity.
	XME_HAL_UART_CLOCK_POLARITY_FALLING ///< Falling clock polarity.
}
xme_hal_uart_clockPolarity_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the given UART, but does not enable transmitter or
 *         receiver.
 *
 * \param  intf UART interface to initialize.
 * \param  baudRate Desired baud rate (transmission speed).
 * \param  mode Asynchronous or synchronous mode.
 * \param  parity Type of parity check.
 * \param  stopBits Number of stop bits.
 * \param  dataBits Number of data bits.
 * \param  clockPolarity Type of clock polarity (rising or falling).
 * \return Returns on of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the UART device has been properly
 *            initialized.
 *          - XME_CORE_STATUS_INVALID_INTERFACE if the given interface was
 *            invalid.
 *          - XME_CORE_STATUS_OUT_OF_RESOURCES if not enough resources are
 *            available to initialize the UART device.
 */
xme_status_t
xme_hal_uart_init
(
	xme_hal_uart_descr_t* intf,
	xme_hal_uart_baudRate_t baudRate           = XME_HAL_UART_BAUD_RATE_19200,
	xme_hal_uart_mode_t mode                   = XME_HAL_UART_MODE_ASYNC,
	xme_hal_uart_parity_t parity               = XME_HAL_UART_PARITY_NONE,
	xme_hal_uart_stopBits_t stopBits           = XME_HAL_UART_STOP_BITS_1,
	xme_hal_uart_dataBits_t dataBits           = XME_HAL_UART_DATA_BITS_8,
	xme_hal_uart_clockPolarity_t clockPolarity = XME_HAL_UART_CLOCK_POLARITY_RISING
);

/**
 * \brief  Finalizes the given UART.
 *
 * \param  intf UART interface to finalize.
 */
void
xme_hal_uart_fini
(
	xme_hal_uart_descr_t* intf
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_HAL_UART_H
