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
 * $Id: interface.c 5613 2013-10-25 09:04:14Z ruiz $
 */

/**
 * \file
 *         Communication interface abstraction.
 */

/**
 * \addtogroup xme_com_interface
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/com/interface.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

#ifdef WIN32
#define sscanf sscanf_s
#endif

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief function to convert from host to network byte order for short values
 * \param hostshort short value in host byte order
 * \return short value in network byte order
 */
extern uint16_t
xme_hal_net_htons
(
        uint16_t hostshort
);

/**
 * \brief function to convert from network to host byte order for short values
 * \param netshort short value in network byte order
 * \return short value in host byte order
 */
extern uint16_t
xme_hal_net_ntohs
(
        uint16_t netshort
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_com_interfaceDescr_t*
xme_com_interface_getInterface
(
    xme_com_interface_interfaceId_t interfaceId
)
{
    // TODO: How to manage interfaces? See ticket #746
    static xme_com_interfaceDescr_t intf =
    {
        XME_COM_INTERFACEMANAGER_INVALID_INTERFACE_ID, // interfaceID

        NULL, // init
        NULL, // fini

        NULL, // read_non_blocking
        NULL, // read_blocking

        NULL, // write_non_blocking
        NULL, // write_blocking

        NULL, // join_channel
        NULL, // leave_channel

        NULL, // get_available_data_size
        NULL, // wait_for_state_change

        NULL, // register_callback
        NULL, // clear_callback

        NULL, // provide_channel
        NULL, // unprovide_channel

        NULL,
        NULL
    };

    if(XME_COM_INTERFACEMANAGER_INVALID_INTERFACE_ID != interfaceId)
    {

    }
    return &intf;
}

// TODO: Rename to xme_com_interface_ipv4StringToGenericAddress()! (Issue #3121)
// TODO: Mismatch between this function and xme_com_interface_genericAddressToIPv4String(): Port is here specified as a separate parameter! (Issue #3121)
xme_status_t
xme_com_interface_ipv4StringToGenericAddress
(
    const char* ip,
    xme_com_interface_address_t* const interfaceAddress
)
{
    uint8_t i;
    unsigned int port;

    XME_CHECK(NULL != interfaceAddress && NULL != ip, XME_STATUS_INVALID_PARAMETER);

    interfaceAddress->addressType = XME_COM_INTERFACE_ADDRESS_TYPE_IPV4;

    for (i = 0; i < 10; i++)
    {
        interfaceAddress->data.ipv4.reserved0[i] = 0;
    }
    for (i = 0; i < 2; i++)
    {
        interfaceAddress->data.ipv4.reserved1[i] = (uint8_t) 0xFF;
    }

    {
        unsigned int ipHost[4] = { 0, 0, 0, 0 };
        sscanf(ip, "%u.%u.%u.%u:%u", &ipHost[0], &ipHost[1], &ipHost[2], &ipHost[3], &port);

        // aa.bb.cc.dd is stored as { aa, bb, cc, dd }
        // This helps in converting to IPv6 easily.
        for (i = 0; i < 4; i++)
        {
            interfaceAddress->data.ipv4.ip[i] = (uint8_t) ipHost[i];
        }
    }

    {
        // Port is input in host byte order, but needs to go in network byte order
        uint16_t portN = xme_hal_net_htons((uint16_t)port);
        interfaceAddress->data.ipv4.port[1] = (uint8_t) (portN >> 8);
        interfaceAddress->data.ipv4.port[0] = (uint8_t) portN;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_com_interface_ipv4ToGenericAddress
(
    uint8_t a,
    uint8_t b,
    uint8_t c,
    uint8_t d,
    uint16_t port,
    xme_com_interface_address_t* const interfaceAddress
)
{
    uint8_t i;

    XME_CHECK(NULL != interfaceAddress, XME_STATUS_INVALID_PARAMETER);

    interfaceAddress->addressType = XME_COM_INTERFACE_ADDRESS_TYPE_IPV4;

    for (i = 0; i < 10; i++)
    {
        interfaceAddress->data.ipv4.reserved0[i] = 0;
    }
    for (i = 0; i < 2; i++)
    {
        interfaceAddress->data.ipv4.reserved1[i] = (uint8_t) 0xFF;
    }

    interfaceAddress->data.ipv4.ip[0] = a;
    interfaceAddress->data.ipv4.ip[1] = b;
    interfaceAddress->data.ipv4.ip[2] = c;
    interfaceAddress->data.ipv4.ip[3] = d;

    {
        // Port is input in host byte order, but needs to go in network byte order
        uint16_t portN = xme_hal_net_htons(port);
        interfaceAddress->data.ipv4.port[1] = (uint8_t) (portN >> 8);
        interfaceAddress->data.ipv4.port[0] = (uint8_t) portN;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_com_interface_genericAddressToIPv4String
(
    const xme_com_interface_address_t* interfaceAddress,
    char* const buffer,
    size_t size
)
{
    unsigned short int port;

    XME_CHECK(NULL != interfaceAddress && NULL != buffer, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4 == interfaceAddress->addressType, XME_STATUS_INVALID_CONFIGURATION);

    // Port is stored in network byte order
    port = xme_hal_net_ntohs((((uint16_t) interfaceAddress->data.ipv4.port[1]) << 8) | interfaceAddress->data.ipv4.port[0]);

    // IP address is stored in order { aa, bb, cc, dd }
    (void)xme_hal_safeString_snprintf
    (
        buffer, size, "%u.%u.%u.%u:%hu",
        interfaceAddress->data.ipv4.ip[0],
        interfaceAddress->data.ipv4.ip[1],
        interfaceAddress->data.ipv4.ip[2],
        interfaceAddress->data.ipv4.ip[3],
        port
    );

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_com_interface_genericAddressToIPv4
(
    const xme_com_interface_address_t* const interfaceAddress,
    uint32_t* const ip,
    uint16_t* const port
)
{
    XME_CHECK(NULL != interfaceAddress, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_COM_INTERFACE_ADDRESS_TYPE_IPV4 == interfaceAddress->addressType, XME_STATUS_INVALID_CONFIGURATION);

    if (NULL != ip)
    {
        // IP address is stored in order { aa, bb, cc, dd }
        // and network byte order of aa.bb.cc.dd is ddccbbaa.
        *ip =
            interfaceAddress->data.ipv4.ip[3] << 24 |
            interfaceAddress->data.ipv4.ip[2] << 16 |
            interfaceAddress->data.ipv4.ip[1] << 8 |
            interfaceAddress->data.ipv4.ip[0];
    }

    if (NULL != port)
    {
        // Port is stored in network byte order
        *port = (((uint16_t) interfaceAddress->data.ipv4.port[1]) << 8) | interfaceAddress->data.ipv4.port[0];
    }

    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
