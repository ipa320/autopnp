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
 * $Id: byteOrder_arch.c 4675 2013-08-13 13:57:57Z kukreja $
 */

/**
 * \file
 *         Byte Order (Endianness) conversion functions
 */

/**
 * \addtogroup hal_byteOrder 
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/hal/include/byteOrder.h"

#if defined(_WIN32) && !defined(CYGWIN)
#include <winsock2.h>
#else // #if defined(_WIN32) && !defined(CYGWIN)
#include <arpa/inet.h>
#endif // #if defined(_WIN32) && !defined(CYGWIN)

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

uint16_t
xme_hal_byteOrder_htons(uint16_t hostshort)
{
    return htons(hostshort);
}

uint32_t
xme_hal_byteOrder_htonl(uint32_t hostlong)
{
    return htonl(hostlong);
}

uint64_t
xme_hal_byteOrder_htonll(uint64_t hostlonglong)
{
    static const int num = 42;

    // Check the endianness
    if (*(const char*)(&num) == num)
    {
        // Little endian, conversion required
        const uint32_t high_part = htonl((uint32_t)(hostlonglong >> 32));
        const uint32_t low_part = htonl((uint32_t)(hostlonglong & 0xFFFFFFFFLL));

        return ((uint64_t)(low_part) << 32) | high_part;
    }
    else
    {
        // Big endian, no conversion required
        return hostlonglong;
    }
}

uint16_t
xme_hal_byteOrder_ntohs(uint16_t netshort)
{
    return ntohs(netshort);
}

uint32_t
xme_hal_byteOrder_ntohl(uint32_t netlong)
{
    return ntohl(netlong);
}

uint64_t
xme_hal_byteOrder_ntohll(uint64_t netlonglong)
{
    return xme_hal_byteOrder_htonll(netlonglong);
}

/**
 * @}
 */
