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
 * $Id: interface.h 5613 2013-10-25 09:04:14Z ruiz $
 */

/**
 * \file
 *         Communication interface abstraction.
 */

#ifndef XME_COM_INTERFACE_H
#define XME_COM_INTERFACE_H

/**
 * \addtogroup xme_com_interface
 * @{
 */


/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/dataChannel.h"
#include "xme/hal/include/sharedPtr.h"
#include "xme/hal/include/safeString.h"

#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \brief Transforms a generic address to a host tcp address.
 * \param addr the generic address.
 */
#define GENERIC_ADDRESS_TO_HOST_TCP(addr) \
    (*((char **)((addr).data)))

/**
 * \brief Transforms a generic address to a host tcp address with memory allocation. 
 * \param addr the generic address.
 * \param charPtr the output char pointer.
 */
#define GENERIC_ADDRESS_TO_HOST_TCP_MALLOC_TCP(addr, charPtr) \
    do { \
        if ( GENERIC_ADDRESS_TO_HOST_TCP(addr) != NULL ) \
        { \
            size_t size; \
            size = strlen( GENERIC_ADDRESS_TO_HOST_TCP(addr) ) + 1; \
            charPtr = (char *)xme_hal_mem_alloc( (uint16_t)size ); \
            xme_hal_mem_copy( charPtr, GENERIC_ADDRESS_TO_HOST_TCP(addr), size ); \
        } else { \
            charPtr = NULL; \
        } \
    } while(0);

/**
 * \brief Transforms a generic address to a port tcp address.
 * \param addr the generic address.
 * \param port the output port.
 */
#define GENERIC_ADDRESS_TO_PORT_TCP(addr, port) \
    do { \
        port = *(uint16_t *)(&((addr).data[4])); \
    } while(0);

/**
 * \brief Transforms a port tcp address into a generic address.
 * \param addr the output generic address.
 * \param port the input port.
 */
#define PORT_TO_GENERIC_ADDRESS_TCP(addr, port) \
    do { \
        (*(uint16_t *)(&((addr).data[4]))) = port; \
    } while(0);

/**
 * \brief Transforms a host tcp address into a generic address.
 * \param addr the output generic address.
 * \param host the input host address.
 */
#define HOST_TO_GENERIC_ADDRESS_TCP(addr, host) \
    do { \
        *((char **)(addr).data) = host; \
    } while(0);

/**
 * \brief  Optimal buffer size for ::xme_com_interface_genericAddressToIPv4String()
 *         function.
 *
 * \see    ::xme_com_interface_genericAddressToIPv4String()
 */
#define XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE (sizeof("255.255.255.255:65535"))

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum xme_com_interface_interfaceId_t
 *
 * \brief  Locally unique interface identifier.
 */
typedef enum
{
    XME_COM_INTERFACEMANAGER_INVALID_INTERFACE_ID = 0, ///< Invalid interface identifier.
    XME_COM_INTERFACEMANAGER_MAX_INTERFACE_ID = XME_MAX_SYSTEM_VALUE ///< Largest possible interface identifier.
} xme_com_interface_interfaceId_t;

/**
 * \struct xme_com_interface_addressDataIPv4_t
 * \brief structure to store the IPv4 address.
 */
typedef struct
{
    uint8_t reserved0[10]; ///< 80 bytes reserved as 0 for compatibility to IPv6
    uint8_t reserved1[2]; ///< 16 bytes reserved as 1 for compatibility to IPv6
    uint8_t ip[4]; ///< IPv4 address in byte format
    uint8_t port[2]; ///< port in byte format
} xme_com_interface_addressDataIPv4_t;

/**
 * \struct xme_com_interface_addressDataIPv6_t
 * \brief structure to store the IPv6 address.
 */
typedef struct
{
    uint8_t ip[16]; ///< IPv6 address in byte format
    uint8_t port[2]; ///< port in byte format
} xme_com_interface_addressDataIPv6_t;

/**
 * \enum xme_com_interface_addressType_t
 * \brief enumerate all the possible interface address types.
 */
typedef enum
{
    XME_COM_INTERFACE_ADDRESS_TYPE_INVALID = 0, ///< Invalid address type

    // IP-based
    XME_COM_INTERFACE_ADDRESS_TYPE_IPV4, ///< IPv4 address
    XME_COM_INTERFACE_ADDRESS_TYPE_IPV6, ///< IPv6 address

    XME_COM_INTERFACE_ADDRESS_TYPE_MAX = 0xFFFF
} xme_com_interface_addressType_t;

/**
 * \struct xme_com_interface_address_t
 *
 * \brief  Interface-specific address data.
 */
typedef struct
{
    xme_com_interface_addressType_t addressType; ///< variable to tell the kind of address it holds, enum is casted to uint16_t (because it is platform dependent)
    union
    {
        xme_com_interface_addressDataIPv4_t ipv4; ///< IPv4 address type
        xme_com_interface_addressDataIPv6_t ipv6; ///< IPv6 address type
    }data; ///< union of all the possible address types
} xme_com_interface_address_t;

/**
 * \enum xme_com_interface_state_t
 *
 * \brief The state of an interface. Used for callbacks.
 */
typedef enum
{
    XME_COM_INTERFACE_STATE_UNDEFINED = 0, ///< Interface in uninitialized.
    XME_COM_INTERFACE_STATE_IDLE, ///< Interface ready to receive/send data.
    XME_COM_INTERFACE_STATE_DATA_AVAILABLE, ///< New data is available.
    XME_COM_INTERFACE_STATE_ERROR   ///< Error occured. Interface is not reliable anymore.
} xme_com_interface_state_t;

/**
 * \typedef xme_com_interface_callback_t
 *
 * \brief  Callback function to notify about a status change within an interface.
 *
 * \param  status New state of interface.
 */
typedef void (*xme_com_interface_callback_t)
(
    xme_com_interface_state_t status,
    xme_com_interface_interfaceId_t interfaceID
);

/**
 * \struct xme_com_interfaceDescr_t
 *
 * \brief  Generic interface for communication device access.
 *         Every device has to implement the full set of these functions.
 */
typedef struct
{
    // public
    xme_com_interface_interfaceId_t interfaceID;  ///< the interface identifier. 

    xme_status_t (*init) (xme_com_interface_address_t *localAddress); ///< the callback to init function. 
    xme_status_t (*fini) (void); ///< the callback to fini function. 

    uint16_t (*read_non_blocking) (void *buffer, uint16_t count, xme_core_dataChannel_t *channel); ///< the callback to read non blocking function. 
    uint16_t (*read_blocking) (void *buffer, uint16_t count, xme_core_dataChannel_t *channel); ///< the callback to read with blocking function. 

    uint16_t (*write_non_blocking) (const void *buffer, uint16_t count, xme_core_dataChannel_t channel); ///< the callback to write non blocking function.
    uint16_t (*write_blocking) (const void *buffer, uint16_t count, xme_core_dataChannel_t channel); ///< the callback to write non blocking function.

    xme_status_t (*join_channel) (xme_core_dataChannel_t channel); ///< the callback to join communication channel function.
    xme_status_t (*leave_channel) (xme_core_dataChannel_t channel); ///< the callback to leave communication channel function.

    uint16_t (*get_available_data_size) (void); ///< the callback to get available data size function.

    xme_com_interface_state_t (*wait_for_state_change) (void); ///< the callback to wait for state change function.

    void (*register_callback) (xme_com_interface_callback_t cb); ///< the callback to register a callback.
    void (*clear_callback) (void); ///< the callback to clear a callback.

    xme_status_t (*provide_channel) (xme_core_dataChannel_t channel); ///< the callback to provide a channel function.
    xme_status_t (*unprovide_channel) (xme_core_dataChannel_t channel); ///< the callback to remove a provided channel function.

    xme_status_t (*add_channel_to_phy_address_mapping) (xme_core_dataChannel_t channel, xme_com_interface_address_t *address); ///< the callback to add the channel to a mapping with a physical address function.
    xme_status_t (*remove_channel_to_phy_address_mapping) (xme_core_dataChannel_t channel, xme_com_interface_address_t *address); ///< the callback to remove a channel to a mapping with a physical address function.

    // private
} xme_com_interfaceDescr_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief this funcion is not required
 * \param interfaceId interfaceId
 * \return xme_com_interfaceDescr_t* pointer to xme_com_interfaceDescr_t
 */
xme_com_interfaceDescr_t*
xme_com_interface_getInterface
(
    xme_com_interface_interfaceId_t interfaceId
);

/**
 * \brief  Converts the given IPv4 address specified as a string and the given port into a generic address.
 *
 * \param[in] ip Pointer to a string containing an IPv4/port address in the fomat "a.b.c.d:xx" (e.g., "192.168.0.1:1234").
 * \param[out] interfaceAddress Address of a variable where to store the generic address.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_PARAMETER if interfaceAddress was NULL.
 */
xme_status_t
xme_com_interface_ipv4StringToGenericAddress
(
    const char* ip,
    xme_com_interface_address_t* const interfaceAddress
);

/**
 * \brief  Converts the given IPv4 address and port into a generic address.
 *
 * For example, to convert address 192.168.0.1:1234 to a generic address,
 * call this function as follows:
 * ::xme_com_interface_ipv4ToGenericAddress(192U, 168U, 0U, 1U, 1234U, &addr)
 *
 * \param[in] a First block in IPv4 address.
 * \param[in] b Second block in IPv4 address.
 * \param[in] c Third block in IPv4 address.
 * \param[in] d Fourth block in IPv4 address.
 * \param[in] port Port number (in host byte order). May be zero (e.g., if the
 *            port is irrelevant).
 * \param[out] interfaceAddress Address of a variable where to store the
 *             generic address.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_PARAMETER if interfaceAddress was NULL.
 */
xme_status_t
xme_com_interface_ipv4ToGenericAddress
(
    uint8_t a,
    uint8_t b,
    uint8_t c,
    uint8_t d,
    uint16_t port,
    xme_com_interface_address_t* const interfaceAddress
);

/**
 * \brief  Converts the given generic address which is suppossed to represent
 *         an IPv4 address into a string of form "a.b.c.d:port".
 *
 * \param[in] interfaceAddress Address of a variable containing the generic
 *            address to convert.
 * \param[in,out] buffer Address of a buffer where to store the string
 *                representation of the IPv4 address in the format "a.b.c.d:port".
 * \param[in] bufferSize Size of the buffer. The optimal buffer size is indicated
 *            by ::XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE.
 *            If the buffer is too small, the resulting string will be truncated.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_CONFIGURATION if the generic address did not
 *         represent an IPv4 address.
 * \retval XME_STATUS_INVALID_PARAMETER if interfaceAddress or buffer were NULL.
 *
 * \see    ::XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE
 */
xme_status_t
xme_com_interface_genericAddressToIPv4String
(
    const xme_com_interface_address_t* interfaceAddress,
    char* const buffer,
    size_t bufferSize
);

/**
 * \brief  Converts the given generic address to an IPv4 address and port.
 *
 * \param[in] interfaceAddress Address of a variable containing the generic
 *            address to convert.
 * \param[in,out] ip Address of a variable where to store the unsigned 32 bit
 *                representation of the IPv4 address in network byte order.
 *                May be NULL to indicate that the caller is not interested
 *                in retrieving the IPv4 address.
 * \param[in,out] port Address of a variable where to store the unsigned 16 bit
 *                representation of the port number in network byte order.
 *                May be NULL to indicate that the caller is not interested in
 *                retrieving the port number.
 *
 * \retval XME_STATUS_SUCCESS on success.
 * \retval XME_STATUS_INVALID_CONFIGURATION if the generic address did not
 *         represent an IPv4 address.
 * \retval XME_STATUS_INVALID_PARAMETER if interfaceAddress was NULL.
 */
xme_status_t
xme_com_interface_genericAddressToIPv4
(
    const xme_com_interface_address_t* const interfaceAddress,
    uint32_t* const ip,
    uint16_t* const port
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_COM_INTERFACE_H
