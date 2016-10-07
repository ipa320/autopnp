/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: byteOrder.h 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 * \brief Generic header for byte order (endianness) conversion abstraction
 */

#ifndef XME_HAL_BYTEORDER_H
#define XME_HAL_BYTEORDER_H

/**
 * \defgroup hal_byteOrder Conversion Abstraction
 * @{
 *
 * \brief  ByteOrder(Endianness) Conversion Abstraction
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum xme_hal_byteOrder_byteOrder_t
 *
 * \brief  Host byte order endianess types.
 */
typedef enum
{
    XME_HAL_BYTEORDER_LITTLE_ENDIAN = 0x78563412UL, ///< Little endian representation.
    XME_HAL_BYTEORDER_BIG_ENDIAN = 0x12345678UL,    ///< Big endian represenatation.
    XME_HAL_BYTEORDER_PDP_ENDIAN = 0x34127856UL     ///< PDP-based endian representation.
} xme_hal_byteOrder_byteOrder_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \union xme_hal_byteOrder_hostByteOrderTest
 *
 * \brief A constant union definition for testing host byte order.
 *
 * \details Do not access this variable firectly, but use
 *          xme_hal_byteOrder_getHostByteOrder() instead.
 */
static const union
{
    unsigned char bytes[4]; ///< Bytes to be tested.
    uint32_t value; ///< 32-bit unsigned integer representation of bytes to be tested. The exact value will vary depending on the platform's byte order.
} xme_hal_byteOrder_hostByteOrderTest = { { 0x12, 0x34, 0x56, 0x78 } };

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \def    xme_hal_byteOrder_getHostByteOrder
 *
 * \brief  Returns the byte order used on this host system.
 *
 * \retval XME_HAL_BYTEORDER_LITTLE_ENDIAN if the host
 *         system uses little endian byte order.
 * \retval XME_HAL_BYTEORDER_BIG_ENDIAN if the host
 *         system uses big endian byte order.
 * \retval XME_HAL_BYTEORDER_PDP_ENDIAN if the host
 *         system uses PDP-endian byte order.
 */
#define xme_hal_byteOrder_getHostByteOrder() ((unsigned long) xme_hal_byteOrder_hostByteOrderTest.value)

/**
 * \brief  Convert 16 bit value from host to network byte order.
 *         This function essentially does nothing on big endian systems.
 *
 * \param  hostshort 16 bit value to convert.
 *
 * \return Converted 16 bit value.
 */
uint16_t
xme_hal_byteOrder_htons
(
    uint16_t hostshort
);

/**
 * \brief  Convert 32 bit value from host to network byte order.
 *         This function essentially does nothing on big endian systems.
 *
 * \param  hostlong 32 bit value to convert.
 *
 * \return Converted 32 bit value.
 */
uint32_t
xme_hal_byteOrder_htonl
(
    uint32_t hostlong
);

/**
 * \brief  Convert 64 bit value from host to network byte order.
 *         This function essentially does nothing on big endian systems.
 *
 * \param  hostlonglong 64 bit value to convert.
 *
 * \return Converted 64 bit value.
 */
uint64_t
xme_hal_byteOrder_htonll
(
    uint64_t hostlonglong
);

/**
 * \brief  Convert 16 bit value from network to host byte order.
 *         This function essentially does nothing on big endian systems.
 *
 * \param  netshort 16 bit value to convert.
 *
 * \return Converted 16 bit value.
 */
uint16_t
xme_hal_byteOrder_ntohs
(
    uint16_t netshort
);

/**
 * \brief  Convert 32 bit value from network to host byte order.
 *         This function essentially does nothing on big endian systems.
 *
 * \param  netlong 32 bit value to convert.
 *
 * \return Converted 32 bit value.
 */
uint32_t
xme_hal_byteOrder_ntohl
(
    uint32_t netlong
);

/**
 * \brief  Convert 64 bit value from network to host byte order.
 *         This function essentially does nothing on big endian systems.
 *
 * \param  netlonglong 64 bit value to convert.
 *
 * \return Converted 64 bit value.
 */
uint64_t
xme_hal_byteOrder_ntohll
(
    uint64_t netlonglong
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/byteOrder_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_BYTEORDER_H
