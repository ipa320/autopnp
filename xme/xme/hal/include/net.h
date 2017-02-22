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
 * $Id: net.h 7809 2014-03-13 11:47:47Z gulati $
 */

/**
 * \file
 * \brief Generic header for network communication abstraction.
 */

#ifndef XME_HAL_NET_H
#define XME_HAL_NET_H

/**
 * \defgroup hal_net Network communication
 * @{
 *
 * \brief  Network communication abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/com/interface.h"

#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def XME_HAL_NET_IP_PORT_NUMBER_DATA_CENTRIC_COMMUNICATION
 * \brief IP port number used for data centric communication. This port number is part of a port number range that is unassigned by IANA and is hence unlikely to be assigned in the future.
 */
#define XME_HAL_NET_IP_PORT_NUMBER_DATA_CENTRIC_COMMUNICATION ((uint16_t) 0x804Fu)
/**
 * \def XME_HAL_NET_IP_PORT_NUMBER_LOGIN
 * \ brief IP port number used for login. This port number is part of a port number range that is unassigned by IANA and is hence unlikely to be assigned in the future.
 */
#define XME_HAL_NET_IP_PORT_NUMBER_LOGIN                      ((uint16_t) 0x804Eu)

/**
 * \def XME_HAL_NET_SOCKET_UDP
 * \brief Socket will be datagram-based (UDP).
 */
#define XME_HAL_NET_SOCKET_UDP             ((uint16_t)0x0001)
/**
 * \def XME_HAL_NET_SOCKET_TCP
 * \brief Socket will be connection-oriented (TCP).
 */
#define XME_HAL_NET_SOCKET_TCP             ((uint16_t)0x0002)
/**
 * \def XME_HAL_NET_SOCKET_BROADCAST
 * \brief Socket will allow sending broadcasts. Will only work in combination with XME_HAL_NET_SOCKET_UDP.
 */
#define XME_HAL_NET_SOCKET_BROADCAST       ((uint16_t)0x0004)
/**
 * \def XME_HAL_NET_SOCKET_NONBLOCKING
 * \brief Socket operations will be non-blocking.
 */
#define XME_HAL_NET_SOCKET_NONBLOCKING     ((uint16_t)0x0008)
/**
 * \def XME_HAL_NET_SOCKET_MULTICAST
 * \brief Socket will allow sending multicast. Will only work in combination with XME_HAL_NET_SOCKET_BROADCAST
 */
#define XME_HAL_NET_SOCKET_MULTICAST            ((uint16_t)0x0010)
/**
 * \def XME_HAL_NET_SOCKET_NUMERIC_ADDRESS
 * \brief Address specified is numeric (optional optimization).
 */
#define XME_HAL_NET_SOCKET_NUMERIC_ADDRESS ((uint16_t)0x0040)
/**
 * \def XME_HAL_NET_SOCKET_IPV6
 * \brief Address specified is an IPv6 address.
 */
#define XME_HAL_NET_SOCKET_IPV6            ((uint16_t)0x0080)

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef *xme_hal_net_callback_t
 * \brief a pointer to a function having one parameter: status of the communication interface.
 */
typedef void (*xme_hal_net_callback_t)
(
    xme_com_interface_state_t status
);

/**
 * \enum xme_hal_net_socketHandle_t
 * \brief Socket handle.
 */
typedef enum
{
    XME_HAL_NET_INVALID_SOCKET_HANDLE = 0, ///< Invalid socket handle.
    XME_HAL_NET_MAX_SOCKET_HANDLE = XME_MAX_SYSTEM_VALUE ///< Largest possible socket handle.
}
xme_hal_net_socketHandle_t;

/**
 * \enum xme_hal_net_byteOrder_t
 *
 * \brief  Host byte order endianess types.
 */
typedef enum
{
    XME_HAL_NET_BYTEORDER_LITTLE_ENDIAN = 0x78563412UL, ///< little endian representation.
    XME_HAL_NET_BYTEORDER_BIG_ENDIAN = 0x12345678UL,    ///< big endian represenatation.
    XME_HAL_NET_BYTEORDER_PDP_ENDIAN = 0x34127856UL     ///< PDP-based endian representation.
}
xme_hal_net_byteOrder_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static const union
{
    unsigned char bytes[4];
    uint32_t value;
}
xme_hal_net_hostByteOrderTest = { { 0x12, 0x34, 0x56, 0x78 } }; ///< a constant union definition for testing host byte order.

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the network abstraction.
 *
 * \retval XME_STATUS_SUCCESS if the network abstraction has been properly initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if network abstraction initialization failed.
 * \retval XME_STATUS_UNSUPPORTED if the network abstraction is unsupported on this platform.
 */
xme_status_t
xme_hal_net_init(void);

/**
 * \brief  Frees resources occupied by the network abstraction.
 */
void
xme_hal_net_fini(void);

/**
 * \brief  Initializes the given network communication interface.
 *
 * \param  localAddress The local address of the interface.
 *
 * \retval XME_STATUS_SUCCESS if the network communication interface
 *                                 has been properly initialized.
 * \retval XME_HAL_STATUS_INVALID_INTERFACE if the given interface was
 *                                          invalid.
 * \retval XME_STATUS_OUT_OF_RESOURCES if not enough resources are
 *                                          available to initialize the interface.
 */
xme_status_t
xme_hal_net_initInterface
(
    xme_com_interface_address_t* localAddress
);

/**
 * \brief  Deinitializes the given network communication interface and frees
 *         all resources associated with it.
 *
 * \param  intf Network communication interface to deinitialize.
 */
void
xme_hal_net_finiInterface
(
    xme_com_interfaceDescr_t* intf
);

/**
 * \brief  Creates a new communication socket for the given network
 *         communication interface.
 *
 * \param  intf Network communication interface for which to create the
 *         communication socket.
 * \param  flags A combination of the following flags:
 *          - xme_HAL_NET_SOCKET_UDP: Socket will be datagram-based (UDP).
 *            Not compatible with XME_HAL_NET_SOCKET_TCP.
 *          - xme_HAL_NET_SOCKET_TCP: Socket will be connection-oriented
 *            (TCP). Not compatible with XME_HAL_NET_SOCKET_UDP.
 *          - xme_HAL_NET_SOCKET_NUMERIC_ADDRESS: Address specified is numeric
 *            (optional optimization).
 *          - xme_HAL_NET_SOCKET_IPV6: Address specified is an IPv6 address
 *            (must be specified in case of IPv6 addressing).
 * \param  hostname Destination host name for the socket.
 * \param  port Port number under which to bind to the destination address.
 * \return Returns a non-zero socket handle on success and
 *         XME_HAL_NET_INVALID_SOCKET on error.
 */
xme_hal_net_socketHandle_t
xme_hal_net_createSocket
(
    xme_com_interfaceDescr_t* intf,
    uint16_t flags,
    const char* hostname,
    uint16_t port
);

/**
 * \brief  Closes and destroys the given socket.
 *
 * \param  socketHandle Handle of the socket to close and destroy.
 *
 * \retval XME_STATUS_SUCCESS if the socket has been successfully closed and
 *                            destroyed.
 * \retval XME_STATUS_INVALID_HANDLE if the given socket handle was invalid.
 */
xme_status_t
xme_hal_net_destroySocket
(
    xme_hal_net_socketHandle_t socketHandle
);

/**
 * \brief  Tries to open the communication channel.
 *
 * If the given socket is to be bound to a remote peer (hostname
 * parameter was set to non-zero in xme_hal_net_createSocket()), the
 * function will try to resolve the address of the host and, upon
 * success, open the communication channel under the specified port.
 * Depending on the type of transport protocol used, handshaking may
 * occur.
 *
 * If the given socket is to be used for sending (hostname parameter
 * was set to zero in xme_hal_net_createSocket()), the function will
 * bind the socket to the local host under the specified port.
 *
 * If the given socket has already been opened, the function silently
 * reports success.
 *
 * \param  socketHandle Handle of the socket to open.
 *
 * \retval XME_STATUS_SUCCESS if the socket has been successfully
 *                            opened or was already opened before.
 * \retval XME_STATUS_OPERATION_FAILED if the socket could not be
 *                                     opened.
 * \retval XME_STATUS_INVALID_HANDLE if the given socket handle was invalid.
 */
xme_status_t
xme_hal_net_openSocket
(
    xme_hal_net_socketHandle_t socketHandle
);

/**
 * \ingroup hal_net
 *
 * \brief  Closes the communication channel.
 *
 * In case the socket is already closed, the function silently reports
 * success.
 *
 * \param  socketHandle Handle of the socket to close.
 *
 * \retval XME_STATUS_SUCCESS if the socket has been successfully
 *                                closed or was already closed before.
 * \retval XME_STATUS_INVALID_HANDLE if the given socket handle was invalid.
 */
xme_status_t
xme_hal_net_closeSocket
(
    xme_hal_net_socketHandle_t socketHandle
);

/**
 * \brief  Returns whether the communication channel is currently open.
 *
 * \param  socketHandle Handle of the socket to open.
 *
 * \return Returns whether the communication channel is currently open.
 */
bool
xme_hal_net_isSocketConnected(xme_hal_net_socketHandle_t socketHandle);

// TODO: See ticket #722
/**
 * \brief  Sets a blocking behaviour to the socket.
 *
 * \note   Some functionality may not be supported on all platforms!
 *
 * \param  socketHandle Handle of the socket to open.
 * \param  blocking determines whether the behaviour of the shocked should be
 *          established in blocking mode (T) or in non-blocking mode (F).
 *
 * \retval XME_STATUS_SUCCESS if blocking behaviour was correctly established.
 * \retval XME_STATUS_INVALID_HANDLE if the given socket handle was invalid.
 * \retval XME_STATUS_INTERNAL_ERROR if the operation failed
 *                                        internally.
 */
xme_status_t
xme_hal_net_setBlockingBehavior
(
    xme_hal_net_socketHandle_t socketHandle,
    bool blocking
);

/**
 * \brief  Waits until read or write operations are available, or until
 *         the given timeout interval expires.
 *
 * \note   Some functionality may not be supported on all platforms!
 *
 * \param  socketHandle Handle of the socket to open.
 * \param  readSocket If true, the function returns as soon as at least one byte is
 *         available to read from the socket.
 * \param  writeSocket If true, the function returns as soon as at least one byte
 *         may be written to the socket.
 * \param  timeoutMs Timeout interval in milliseconds after which the function
 *         should return even if no read or write operations are available.
 *         Specifying zero for this parameter will cause the function to never
 *         return before the requested operations are available.
 * \retval XME_STATUS_SUCCESS if read or write operations are
 *                                 available.
 * \retval XME_STATUS_TIMEOUT if the given timeout interval elapsed
 *                                 before read or write operations were available.
 * \retval XME_STATUS_INVALID_HANDLE if the given socket handle was invalid.
 * \retval XME_STATUS_INTERNAL_ERROR if the operation failed
 *                                        internally.
 */
xme_status_t
xme_hal_net_selectSocket
(
    xme_hal_net_socketHandle_t socketHandle,
    bool readSocket,
    bool writeSocket,
    uint16_t timeoutMs
);


// TODO: See ticket #722
/**
 * \brief Selects multiple sockets.
 *
 * \param max_socket maximum number of sockets to be opened.
 * \param socketSetRead the socket handle set to read.
 * \param socketSetWrite the socket handle set to write.
 * \param socketSetError the socket handle set to errors.
 * \param timeoutMs the timeout in miliseconds.
 *
 * \retval XME_STATUS_SUCCESS if read or write operations are available.
 * \retval XME_STATUS_TIMEOUT if the given timeout interval elapsed
 *                                 before read or write operations were available.
 * \retval XME_STATUS_INVALID_HANDLE if the given socket handle was invalid.
 * \retval XME_STATUS_INTERNAL_ERROR if the operation failed internally.
 */
xme_status_t
xme_hal_net_selectMultipleSockets
(
    uint16_t max_socket,
    xme_hal_net_socketHandle_t* socketSetRead,
    xme_hal_net_socketHandle_t* socketSetWrite,
    xme_hal_net_socketHandle_t* socketSetError,
    uint16_t timeoutMs
);

// TODO: See ticket #722
/**
 * \brief Waits for a connection from the socket handle.
 *
 * \param socketHandle the socket handle.
 *
 * \return the connected socket handle.
 */
xme_hal_net_socketHandle_t
xme_hal_net_waitForConnection
(
    xme_hal_net_socketHandle_t socketHandle
);

/**
 * \brief  Receives up to count bytes from the given socket, places them in
 *         into the given buffer and returns the number of bytes actually read.
 *
 * \note   If the XME_HAL_NET_SOCKET_NONBLOCKING flag is set on the socket,
 *         the function will return immediatly with a return value of 0 if
 *         there is no data available. Otherwise, the function will block
 *         until data is available.
 *
 * \param  socketHandle the socket handle.
 * \param  buffer Buffer to place received data in. Must be at least of the
 *         size specified in count.
 * \param  count Size of the given buffer or number of bytes to read at
 *         maximum, whichever is smaller.
 * \return Number of bytes actually read.
 */
uint16_t
xme_hal_net_readSocket
(
    xme_hal_net_socketHandle_t socketHandle,
    void* buffer,
    uint16_t count
);

/**
 * \brief  Sends up to count bytes from the given buffer over the given socket
 *         and returns the number of bytes actually sent.
 *
 * \note   If the XME_HAL_NET_SOCKET_NONBLOCKING flag is set on the socket,
 *         the function will return immediatly with a return value of 0 if
 *         no data can be sent. Otherwise, the function will block until
 *         data can be sent.
 *
 * \param  socketHandle the socket handle.
 * \param  buffer Buffer with data to send. Must be at least of size count.
 * \param  count Number of bytes to send.
 * \return Number of bytes actually sent.
 */
uint16_t
xme_hal_net_writeSocket
(
    xme_hal_net_socketHandle_t socketHandle,
    const void* buffer,
    uint16_t count
);

/**
 * \brief  Uses given socket to send a datagram.
 *         For IP-Networks, this function can be used for broadcast as
 *         well as multicast.
 *
 * \param socketHandle Socket to use for sending.
 * \param remoteAddress Address of destination.
 * \param buffer Buffer with data to send. Must be at least of size count.
 * \param count Number of bytes to send.
 * \return Number of bytes actually sent.
 */
uint16_t
xme_hal_net_writeDatagram
(
    xme_hal_net_socketHandle_t socketHandle,
    xme_com_interface_address_t* remoteAddress,
    const void* buffer,
    uint16_t count
);

/**
 * \brief Get the IP address of the networking interface.
 *        If no interface is provided it returns the first valid IP address.
 *        Currently it is limited to IPv4 addresses.
 *
 * \param[in] interfaceName Name of the interface of which IP is returned.
 *            It can be passed in couple of ways:
 *            1. For *nix like Operating systems, it can be eth0, net0
 *            2. Windows can have two options:
 *            Friendly Name: seen in Adapter connections like 
 *            LAN-Verbindung
 *            or
 *            Adapater Name: the name the network is registered with
 *            {ED18C351-038A-4A21-93FC-6ABC85ECB25F}
 *            3. The hostname, for which IP is obtained from DNS
 *            hello.world.com
 *            This is currently not supported.
 *            4. It can be NULL. In that case the function will
 *            the first valid working IP. This can be loopback too.
 *            For Options 1 and 2 the string should be passed in []
 *            eg
 *              [eth0]
 *             or
 *              [LAN-Verbindung]
 *             or
 *              [{ED18C351-038A-4A21-93FC-6ABC85ECB25F}]
 * \param[in] addressType Type of address IPv4 or IPv6 
 *            XME_COM_INTERFACE_ADDRESS_TYPE_IPV4 or
 *            XME_COM_INTERFACE_ADDRESS_TYPE_IPV6
 *            Currently we are only supporting IPv4
 * \param[out] ipAddress char pointer to the memory location where IP address is stored.
 *             For IPv4 it should be of size XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE.
 *             For IPv6 it should be of size XME_COM_INTERFACE_IPV6_STRING_BUFFER_SIZE.
 *             Resulting string is null-terminated.
 *
 * \retval XME_STATUS_SUCCESS On success.
 * \retval XME_STATUS_INVALID_PARAMETER if the input parameter is not correct.
 * \retval XME_STATUS_BUFFER_TOO_SMALL if the allocated buffer used during the operation is too small.
 * \retval XME_STATUS_INTERNAL_ERROR if the operation failed for any other reason. Reason can be
 *                                   The interface supplied did not have an IP address
 *                                   or The network cable was plugged out so the network card was not UP
 *                                   In such cases it is advisable to recheck network settings.
 */
xme_status_t
xme_hal_net_getInterfaceAddr
(
    char* interfaceName,
    xme_com_interface_addressType_t addressType,
    char* ipAddress
);

/**
 * \brief Joins a multicast group.
 *
 * \param socketHandle Socket to use for joining.
 * \param remoteAddress Address of multicast group.
 * \retval XME_STATUS_SUCCESS On success.
 * \retval XME_STATUS_INTERNAL_ERROR if the operation failed internally.
 */
xme_status_t
xme_hal_net_joinMulticastGroup
(
    xme_hal_net_socketHandle_t socketHandle,
    xme_com_interface_address_t* remoteAddress
);

/**
 * \brief Leaves a multicast group.
 *
 * \param socketHandle Socket to use for sending.
 * \param remoteAddress Address of destination.
 * \return Number of bytes actually sent.
 */
xme_status_t
xme_hal_net_leaveMulticastGroup
(
    xme_hal_net_socketHandle_t socketHandle,
    xme_com_interface_address_t* remoteAddress
);

// TODO: This is probably not thread-safe! See ticket #810
/**
 * \brief Get the address of the peer host.
 *
 * \param socketHandle Socket that received something before.
 * \param remoteAddress Pointer to memory to store address.
 */
void
xme_hal_net_getAddressOfLastReception
(
    xme_hal_net_socketHandle_t socketHandle,
    xme_com_interface_address_t* remoteAddress
);

/**
 * \brief Gets the available data to read of a specific socket.
 *
 * \param socketHandle Socket to be checked.
 * \return Number of bytes available. Zero on error or if zero bytes are available.
 */
uint16_t
xme_hal_net_get_available_data_size
(
    xme_hal_net_socketHandle_t socketHandle
);

/**
 * \brief Callback for interface change.
 *
 * \param cb Callback function.
 */
void
xme_hal_net_registerCallback
(
    xme_hal_net_callback_t cb
);

/**
 * \brief Clear callback function.
 */
void
xme_hal_net_clearCallback(void);

/**
 * \def    xme_hal_net_getHostByteOrder()
 *
 * \brief  Returns the byte order used on this host system.
 *
 * \retval XME_HAL_NET_BYTEORDER_LITTLE_ENDIAN if the host
 *         system uses little endian byte order.
 * \retval XME_HAL_NET_BYTEORDER_BIG_ENDIAN if the host
 *         system uses big endian byte order.
 * \retval XME_HAL_NET_BYTEORDER_PDP_ENDIAN if the host
 *         system uses PDP-endian byte order.
 */
#define xme_hal_net_getHostByteOrder() (xme_hal_net_hostByteOrderTest.value)

/**
 * \brief  Convert 16 bit value from host to network byte order.
 *         This function essentially does nothing on big endian systems.
 *
 * \param  hostshort 16 bit value to convert.
 *
 * \return Converted 16 bit value.
 */
uint16_t
xme_hal_net_htons
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
xme_hal_net_htonl
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
xme_hal_net_htonll
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
xme_hal_net_ntohs
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
xme_hal_net_ntohl
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
xme_hal_net_ntohll
(
    uint64_t netlonglong
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/net_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_NET_H
