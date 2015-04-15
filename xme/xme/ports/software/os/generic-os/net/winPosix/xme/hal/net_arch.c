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
 * $Id: net_arch.c 7831 2014-03-14 10:32:44Z geisinger $
 */

/**
 * \file
 *         Network communication abstraction (architecture specific part:
 *         generic OS based implementation).
 */

/**
 * \addtogroup hal_net
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
// This header is included *before* CHROMOSOME-specific headers, because
// it uses inline functions which call functions that are marked as deprecated
// in CHROMOSOME.
#if defined(_WIN32) && !defined(CYGWIN)
#    include <ws2tcpip.h>
#endif // #if defined(_WIN32) && !defined(CYGWIN)

#include "xme/hal/include/net.h"

#include "xme/core/log.h"

#include "xme/hal/include/math.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/table.h"

#if defined(_WIN32) && !defined(CYGWIN)
#    include <winsock2.h>
#    include <mswsock.h>
#    include <iphlpapi.h>
#else // #if defined(_WIN32) && !defined(CYGWIN)
#    include <errno.h>
#    include <netdb.h>
#    include <arpa/inet.h>
#    include <sys/types.h>
#    include <fcntl.h>
#    include <sys/ioctl.h>
#   include <unistd.h>
#   include <ifaddrs.h>
#   include <net/if.h>
#endif // #if defined(_WIN32) && !defined(CYGWIN)

/**
 * \def SOCKET
 *
 * \details Define socket as integer
 *
 * \def INVALID_SOCKET
 *
 * \details Define Invalid socket as -1
 *
 * \def closesocket
 *
 * \details Define function call close as closesocket
 *
 */
#if defined(_WIN32) || defined(CYGWIN)
#    ifdef _MSC_VER
#        // Do not change anything if microsoft compiler is running
#    elif defined(__MINGW32__) || defined(__MINGW64__)
#        ifndef ADDRESS_FAMILY
#            define ADDRESS_FAMILY short
#        endif
#        ifndef _TRUNCATE
#            define _TRUNCATE ((size_t)-1)
#        endif
#    else // _MSC_VER
#        ifndef SOCKET
#            define SOCKET int
#        endif
#        ifndef INVALID_SOCKET
#            define INVALID_SOCKET -1
#        endif
#        ifndef SOCKET_ERROR
#            define SOCKET_ERROR (-1)
#        endif
#        ifndef closesocket
#            define closesocket(s) close(s)
#        endif
#    endif // _MSC_VER
#else
#    ifndef SOCKET
#        define SOCKET int
#    endif
#    ifndef INVALID_SOCKET
#        define INVALID_SOCKET -1
#    endif
#    ifndef SOCKET_ERROR
#        define SOCKET_ERROR (-1)
#    endif
#    ifndef closesocket
#        define closesocket(s) close(s)
#    endif
#endif // #if defined(_WIN32) || defined(CYGWIN)

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \union xme_hal_net_address_t
 *
 * \brief  Union for strict aliasing conformant translation between different
 *         socket address types.
 */
typedef union
{
    struct sockaddr sa; ///< Generic socket address.
    struct sockaddr_in sa_in; ///< IPv4 internet address.
    struct sockaddr_in6 sa_in6; ///< IPv6 internet address.
    struct sockaddr_storage sa_stor; ///< Structure big enough to store IPv4 or IPv6 internet address.
} xme_hal_net_address_t;

/**
 * \struct xme_hal_net_socketItem
 *
 * \brief  Socket descriptor used internally to store properties of a socket.
 */
typedef struct
{
    xme_com_interfaceDescr_t* intf; ///< Corresponding network interface identifier.
    uint16_t flags; ///< Socket flags.
    const char* hostname; ///< Associated hostname (remote for outgoing, local for incoming).
    uint16_t port; ///< IPv4 or IPv6 port.
    SOCKET socket; ///< OS socket identifier/descriptor.
    xme_hal_net_address_t peerAddr; ///< Address of communication peer, if available.
    bool connected; ///< Whether this socket is currently connected. Notice that some socket types are never connected.
} xme_hal_net_socketItem;

/**
 * \struct xme_hal_net_configStruct_t
 *
 * \brief  Socket abstraction configuration structure.
 */
typedef struct
{
    //private
    XME_HAL_TABLE(xme_hal_net_socketItem, sockets, XME_HAL_DEFINES_MAX_SOCKETS); // socketHandle is an index into this table
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle; ///< Critical section handle for protecting critical regions.
} xme_hal_net_configStruct_t;

#define GENERIC_ADDRESS_TO_IP4(generic_address) (*((uint32_t *)generic_address)) ///< the IPv4 address.
#define GENERIC_ADDRESS_TO_PORT(generic_address) (*(((uint16_t *)generic_address)+2)) ///< the address port.

#if defined(USE_IP6)
#define GENERIC_ADDRESS_TO_IP6(generic_address) (*((char *)generic_address))
#endif
#define GENERIC_ADDRESS_TO_PORT_IP6(generic_address) (*(((uint16_t *)generic_address)+8)) ///< the IPv6 address.

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static xme_hal_net_configStruct_t xme_hal_net_config; ///< the hal net configuration.

/**
 * \brief write a datagram in an IP address.
 *
 * \param socketItem the socket item.
 * \param remoteAddress the remote IP address.
 * \param remotePort the remote port in host byte order.
 * \param buffer the buffer.
 * \param count the number of bytes to write.
 *
 * \return the result of the operation.
 */
uint16_t
xme_hal_net_writeDatagram_ipaddress
(
    xme_hal_net_socketItem* socketItem,
    void* remoteAddress,
    uint16_t remotePort,
    const void* buffer,
    uint16_t count
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_net_init(void)
{
#if defined(_WIN32) && !defined(CYGWIN)
    WSADATA wsaData;
    int status;
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    XME_ASSERT(XME_ASSERT_NO_SIDE_EFFECTS(0 == XME_HAL_TABLE_ITEM_COUNT(xme_hal_net_config.sockets)));
    XME_ASSERT(XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE == xme_hal_net_config.criticalSectionHandle);

    // Ensure that xme_hal_sync is initialized
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_sync_init(), XME_STATUS_OUT_OF_RESOURCES);

    XME_HAL_TABLE_INIT(xme_hal_net_config.sockets);

    XME_CHECK
    (
        XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE != (xme_hal_net_config.criticalSectionHandle = xme_hal_sync_createCriticalSection()),
        XME_STATUS_OUT_OF_RESOURCES
    );

#if defined(_WIN32) && !defined(CYGWIN)
    status = WSAStartup(MAKEWORD(2, 2), &wsaData);

    XME_CHECK_MSG(NO_ERROR == status, XME_STATUS_UNSUPPORTED, XME_LOG_ERROR, "WSAStartup() failed with code %d!\n", status);

    XME_CHECK_MSG_REC
    (
        0x02 == HIBYTE(wsaData.wVersion),
        XME_STATUS_UNSUPPORTED,
        {
            WSACleanup();
        },
        XME_LOG_ERROR, "WSA version is not 2.2, but %u.%u!\n", HIBYTE(wsaData.wVersion), LOBYTE(wsaData.wVersion)
    );

    XME_LOG_IF
    (
        0x02 != LOBYTE(wsaData.wVersion),
        XME_LOG_WARNING, "Requested WSA version 2.2, but only %u.%u is supported!\n", HIBYTE(wsaData.wVersion), LOBYTE(wsaData.wVersion)
    );
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    return XME_STATUS_SUCCESS;
}

void
xme_hal_net_fini(void)
{
#if defined(_WIN32) && !defined(CYGWIN)
    WSACleanup();
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    (void) xme_hal_sync_destroyCriticalSection(xme_hal_net_config.criticalSectionHandle);
    xme_hal_net_config.criticalSectionHandle = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;

    XME_HAL_TABLE_FINI(xme_hal_net_config.sockets);

    // Decrement reference count to xme_hal_sync
    xme_hal_sync_fini();
}

xme_status_t
xme_hal_net_initInterface
(
    xme_com_interface_address_t* localAddress
)
{
    // Returns one of the following status codes:
    //  - XME_STATUS_SUCCESS if the network communication interface
    //    has been properly initialized.
    //  - XME_HAL_STATUS_INVALID_INTERFACE if the given interface was
    //    invalid.
    //  - XME_STATUS_OUT_OF_RESOURCES if not enough resources are
    //    available to initialize the interface.

    XME_UNUSED_PARAMETER(localAddress);

    // TODO: See ticket #808
    return XME_STATUS_SUCCESS;
}

void
xme_hal_net_finiInterface
(
    xme_com_interfaceDescr_t* intf
)
{
    // TODO: See ticket #809
    if(NULL != intf)
    {

    }
}

xme_hal_net_socketHandle_t
xme_hal_net_createSocket
(
    xme_com_interfaceDescr_t* intf,
    uint16_t flags,
    const char* hostname,
    uint16_t port
)
{
    xme_hal_net_socketHandle_t newSocketHandle;
    xme_hal_net_socketItem* socketItem;

    // Either XME_HAL_NET_SOCKET_TCP or XME_HAL_NET_SOCKET_UDP must be set
    if (!((flags & XME_HAL_NET_SOCKET_TCP) ^ (flags & XME_HAL_NET_SOCKET_UDP)))
    {
        return XME_HAL_NET_INVALID_SOCKET_HANDLE;
    }
    // If XME_HAL_NET_SOCKET_MULTICAST is set then BROADCAST and UDP must be set
    if (flags & XME_HAL_NET_SOCKET_MULTICAST)
    {
        if (!(flags & XME_HAL_NET_SOCKET_BROADCAST) || !(flags & XME_HAL_NET_SOCKET_UDP))
        {
            return XME_HAL_NET_INVALID_SOCKET_HANDLE;
        }
    }

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        newSocketHandle = (xme_hal_net_socketHandle_t)XME_HAL_TABLE_ADD_ITEM(xme_hal_net_config.sockets);
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, newSocketHandle);
    XME_CHECK(NULL != socketItem, XME_HAL_NET_INVALID_SOCKET_HANDLE);

    // Initialize the socket descriptor
    socketItem->intf = intf;
    socketItem->flags = flags;
    socketItem->hostname = hostname;
    socketItem->port = port;
    socketItem->socket = INVALID_SOCKET;
    (void) xme_hal_mem_set(&socketItem->peerAddr, 0, sizeof(socketItem->peerAddr));
    socketItem->connected = false;

    return newSocketHandle;
}

xme_status_t
xme_hal_net_destroySocket(xme_hal_net_socketHandle_t socketHandle)
{
    xme_status_t status;

    // Try to close the socket to shut down gracefully.
    // This will succeed in any case if the socket handle is valid.
    // If the socket handle is invalid, we exit prematurely,
    // because then removal from the list of sockets also won't succeed.
    status = xme_hal_net_closeSocket(socketHandle);
    XME_ASSERT(XME_STATUS_SUCCESS == status || XME_STATUS_INVALID_HANDLE == status);
    XME_CHECK(XME_STATUS_INVALID_HANDLE != status, status);

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Invalidate the socket handle and free associated memory
        status = XME_HAL_TABLE_REMOVE_ITEM(xme_hal_net_config.sockets, (xme_hal_table_rowHandle_t)socketHandle);
        XME_ASSERT(XME_STATUS_SUCCESS == status);
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    return status;
}

xme_status_t
xme_hal_net_openSocket
(
    xme_hal_net_socketHandle_t socketHandle
)
{
    xme_hal_net_socketItem* socketItem;
    struct addrinfo ai;
    struct addrinfo* aiResult = 0;
    struct addrinfo* aiPtr = 0;
    char portName[6];
    int status;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            XME_STATUS_INVALID_HANDLE,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    (void) xme_hal_mem_set(&ai, 0, sizeof(ai));
    ai.ai_flags = (socketItem->flags & XME_HAL_NET_SOCKET_NUMERIC_ADDRESS) ? AI_NUMERICHOST : AF_UNSPEC;
    ai.ai_family = (socketItem->flags & XME_HAL_NET_SOCKET_IPV6) ? AF_INET6 : AF_INET;
    ai.ai_socktype = (socketItem->flags & XME_HAL_NET_SOCKET_TCP) ? SOCK_STREAM : SOCK_DGRAM;
    ai.ai_protocol = (socketItem->flags & XME_HAL_NET_SOCKET_TCP) ? IPPROTO_TCP : IPPROTO_UDP;

    (void) xme_hal_safeString_snprintf(portName, sizeof(portName), "%u", (0u != socketItem->port) ? socketItem->port : XME_HAL_NET_IP_PORT_NUMBER_DATA_CENTRIC_COMMUNICATION);

    if (socketItem->hostname == 0 || (socketItem->flags & XME_HAL_NET_SOCKET_BROADCAST))
    {
        // If no address was specified or the socket is for broadcast, use AI_PASSIVE flag to accept data from any address
        ai.ai_flags |= AI_PASSIVE;
        status = getaddrinfo(NULL, portName, &ai, &aiResult);
    }
    else
    {
        status = getaddrinfo(socketItem->hostname, portName, &ai, &aiResult);
    }

    XME_CHECK_REC
    (
        EAI_AGAIN != status,
        XME_STATUS_TEMPORARY_FAILURE,
        {
            freeaddrinfo(aiResult);
        }
    );

    XME_CHECK_REC
    (
        EAI_NONAME != status,
        XME_STATUS_NOT_FOUND,
        {
            freeaddrinfo(aiResult);
        }
    );

    XME_CHECK_REC
    (
        0 == status,
        XME_STATUS_INVALID_CONFIGURATION,
        {
            freeaddrinfo(aiResult);
        }
    );

    XME_ASSERT(0 != aiResult);

    // Iterate over the result list and obtain at least one usable address
    status = 0;
    for (aiPtr = aiResult; aiPtr != 0; aiPtr = aiPtr->ai_next)
    {
        // The following assertions are on the style of (a => b), i.e., (!a || b)
        XME_ASSERT(!(socketItem->flags & XME_HAL_NET_SOCKET_IPV6) || AF_INET6 == aiPtr->ai_family);
        XME_ASSERT( (socketItem->flags & XME_HAL_NET_SOCKET_IPV6) || AF_INET == aiPtr->ai_family);
        XME_ASSERT(!(socketItem->flags & XME_HAL_NET_SOCKET_TCP)  || SOCK_STREAM == aiPtr->ai_socktype);
        XME_ASSERT(!(socketItem->flags & XME_HAL_NET_SOCKET_UDP)  || SOCK_DGRAM == aiPtr->ai_socktype);
        XME_ASSERT(!(socketItem->flags & XME_HAL_NET_SOCKET_TCP)  || IPPROTO_TCP == aiPtr->ai_protocol);
        XME_ASSERT(!(socketItem->flags & XME_HAL_NET_SOCKET_UDP)  || IPPROTO_UDP == aiPtr->ai_protocol);

        // We just consider the first working item returned.
        // In case multiple items are returned, issue a warning here.
        if (socketItem->connected)
        {
            XME_LOG(XME_LOG_WARNING,
                    "Host name lookup for '%s' returned multiple results, connected to first address!\n", socketItem->hostname);
            break;
        }

        socketItem->socket = socket(ai.ai_family, ai.ai_socktype, ai.ai_protocol);
        if (INVALID_SOCKET == socketItem->socket)
        {
            continue;
        }


        if (socketItem->flags & XME_HAL_NET_SOCKET_BROADCAST)
        {
            // Configure socket for broadcasts
#if defined(_WIN32) && !defined(CYGWIN)
            DWORD yes = 1;
#else // #if defined(_WIN32) && !defined(CYGWIN)
            int yes = 1;
#endif // #if defined(_WIN32) && !defined(CYGWIN)
            if(!(XME_HAL_NET_SOCKET_MULTICAST & socketItem->flags))
            {
                XME_CHECK_REC
                (
                    setsockopt(socketItem->socket, SOL_SOCKET, SO_BROADCAST, (const char*)&yes, sizeof(yes)) == 0,
                    XME_STATUS_INVALID_CONFIGURATION,
                    {
                        closesocket(socketItem->socket);
                        socketItem->socket = INVALID_SOCKET;
                        freeaddrinfo(aiResult);
                    }
                );
            }
            XME_CHECK_REC
            (
                setsockopt(socketItem->socket, SOL_SOCKET, SO_REUSEADDR, (const char*)&yes, sizeof(yes)) == 0,
                XME_STATUS_INVALID_CONFIGURATION,
                {
                    closesocket(socketItem->socket);
                    socketItem->socket = INVALID_SOCKET;
                    freeaddrinfo(aiResult);
                }
            );

            XME_CHECK_REC
            (
                setsockopt(socketItem->socket, IPPROTO_IP, IP_PKTINFO, (const char*)&yes, sizeof(yes)) == 0,
                XME_STATUS_INVALID_CONFIGURATION,
                {
                    closesocket(socketItem->socket);
                    socketItem->socket = INVALID_SOCKET;
                    freeaddrinfo(aiResult);
                }
            );
        }
        if (0 == socketItem->hostname || (socketItem->flags & XME_HAL_NET_SOCKET_MULTICAST))
        {
            // If null is specified as hostname and the socket is not configured
            // for broadcast, the socket will be a local socket that can be used
            // to receive data.
            if (0 == socketItem->port && (socketItem->flags & XME_HAL_NET_SOCKET_UDP))
            {
                //But this is the case of UDP send socket we don't need to bind as well
                status = XME_STATUS_SUCCESS;
            }
            else
            {
                status = bind(socketItem->socket, (const struct sockaddr*) aiResult->ai_addr, aiResult->ai_addrlen);
            }
        }
        else if (!(socketItem->flags & XME_HAL_NET_SOCKET_BROADCAST))
        {
            // Hostname was specified. Connect to the host
            status = connect(socketItem->socket, (const struct sockaddr*) aiResult->ai_addr, aiResult->ai_addrlen);
        }
        if (0 != status)
        {
            // Remember actual error code in status before closing
            // the socket, otherwise it will be reset to zero
#if defined(_WIN32) && !defined(CYGWIN)
            status = WSAGetLastError();
#else // #if defined(_WIN32) && !defined(CYGWIN)
            status = errno;
#endif // #if defined(_WIN32) && !defined(CYGWIN)

            XME_LOG(XME_LOG_WARNING, "Binding socket failed: %d\n",    status);
            XME_ASSERT(0 != status);

            closesocket(socketItem->socket);
            socketItem->socket = INVALID_SOCKET;
        }
        else
        {
            socketItem->connected = true;
        }


        if (NULL == socketItem->hostname &&
            !(socketItem->flags & XME_HAL_NET_SOCKET_BROADCAST) &&
            (XME_HAL_NET_SOCKET_TCP & socketItem->flags))
        {
            // TCP connection without hostname.
            // Make the socket listen for incoming connection requests
            #if defined(_WIN32) && !defined(CYGWIN)
            XME_CHECK_REC
            (
                listen(socketItem->socket, SOMAXCONN) != SOCKET_ERROR,
                (WSAEOPNOTSUPP == WSAGetLastError()) ? XME_STATUS_INVALID_CONFIGURATION : XME_STATUS_OUT_OF_RESOURCES,
            {
                    closesocket(socketItem->socket);
                    socketItem->socket = INVALID_SOCKET;
                }
            );
#else // #if defined(_WIN32) && !defined(CYGWIN)
            XME_CHECK_REC
            (
                listen(socketItem->socket, SOMAXCONN) == 0,
                (ENOTSUP == errno) ? XME_STATUS_INVALID_CONFIGURATION : XME_STATUS_OUT_OF_RESOURCES,
            {
                    closesocket(socketItem->socket);
                    socketItem->socket = INVALID_SOCKET;
                }
            );
#endif // #if defined(_WIN32) && !defined(CYGWIN)
        }
    }
    freeaddrinfo(aiResult);

    if (0 != status)
    {
#if defined(_WIN32) && !defined(CYGWIN)
        XME_CHECK(WSAETIMEDOUT != status, XME_STATUS_TIMEOUT);
        XME_CHECK(WSAECONNREFUSED != status, XME_STATUS_CONNECTION_REFUSED);
        XME_CHECK(WSAEHOSTUNREACH != status, XME_STATUS_NOT_FOUND);
#else // #if defined(_WIN32) && !defined(CYGWIN)
        XME_CHECK(ETIMEDOUT != status, XME_STATUS_TIMEOUT);
        XME_CHECK(ECONNREFUSED != status, XME_STATUS_CONNECTION_REFUSED);
        XME_CHECK(EHOSTUNREACH != status, XME_STATUS_NOT_FOUND);
#endif // #if defined(_WIN32) && !defined(CYGWIN)
        return XME_STATUS_INVALID_CONFIGURATION;
    }

#if defined(_WIN32) && !defined(CYGWIN)
    XME_CHECK(INVALID_SOCKET != socketItem->socket, XME_STATUS_INVALID_CONFIGURATION);
#else // #if defined(_WIN32) && !defined(CYGWIN)
    XME_CHECK(-1 != socketItem->socket, XME_STATUS_INVALID_CONFIGURATION);
#endif // #if defined(_WIN32) && !defined(CYGWIN)
    XME_CHECK(socketItem->connected, XME_STATUS_NOT_FOUND);

    // Set socket blocking behavior
    XME_CHECK
    (
        XME_STATUS_SUCCESS == xme_hal_net_setBlockingBehavior(socketHandle, (bool) !(socketItem->flags & XME_HAL_NET_SOCKET_NONBLOCKING)),
        XME_STATUS_INVALID_CONFIGURATION
    );

    if (0 == socketItem->hostname || (socketItem->flags & XME_HAL_NET_SOCKET_BROADCAST))
    {
        // Broadcast sockets and sockets without a hostname are not really connected
        socketItem->connected = false;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_net_closeSocket
(
    xme_hal_net_socketHandle_t socketHandle
)
{
    xme_hal_net_socketItem* socketItem;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            XME_STATUS_INVALID_HANDLE,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

#if defined(_WIN32) && !defined(CYGWIN)
    {
        // Wait a maximum of 100ms for data from the remote host
        DWORD timeoutMs = 100;

        // <http://msdn.microsoft.com/en-us/library/ms740481%28v=VS.85%29.aspx>
        // says we should do the following for a graceful shutdown:

        // 1. Call shutdown with how=SD_SEND.
        shutdown(socketItem->socket, SD_SEND);

        // Make sure the following recv() call does not block too long
        setsockopt(socketItem->socket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeoutMs, sizeof(timeoutMs));

        // 2. Call recv or WSARecv until the function completes with success and
        //    indicates zero bytes were received. If SOCKET_ERROR is returned,
        //    then the graceful disconnect is not possible.
        while (1)
        {
            char buffer[64];
            int numBytes = recv(socketItem->socket, (char*)&buffer, 64, 0);
            if (0 == numBytes || SOCKET_ERROR == numBytes)
            {
                break;
            }
        }

        // 3. Call closesocket.
        closesocket(socketItem->socket);

        socketItem->socket = INVALID_SOCKET;
    }

#else // #if defined(_WIN32) && !defined(CYGWIN)

    // Calling shutdown here won't hurt...
    (void) shutdown(socketItem->socket, SHUT_RDWR);

    close(socketItem->socket);

    socketItem->socket = -1;
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    socketItem->connected = false;

    return XME_STATUS_SUCCESS;
}

bool
xme_hal_net_isSocketConnected
(
    xme_hal_net_socketHandle_t socketHandle
)
{
    xme_hal_net_socketItem* socketItem;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            false,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    return socketItem->connected;
}

xme_status_t
xme_hal_net_setBlockingBehavior
(
    xme_hal_net_socketHandle_t socketHandle,
    bool blocking
)
{
    xme_hal_net_socketItem* socketItem;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            XME_STATUS_INVALID_HANDLE,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    // Change blocking behavior
#if defined(_WIN32) && !defined(CYGWIN)
    {
        u_long nonBlocking = blocking ? 0 : 1;
        int status = ioctlsocket(socketItem->socket, FIONBIO, &nonBlocking);
        XME_CHECK(0 == status, XME_STATUS_INVALID_CONFIGURATION);
    }
#else // #if defined(_WIN32) && !defined(CYGWIN)
    {
        int flags = fcntl(socketItem->socket, F_GETFL);
        XME_ASSERT(-1 != flags);

        if (blocking)
        {
            flags &= ~O_NONBLOCK;
        }
        else
        {
            flags |= O_NONBLOCK;
        }

        flags = fcntl(socketItem->socket, F_SETFL, flags);
        XME_CHECK(-1 != flags, XME_STATUS_INVALID_CONFIGURATION);
    }
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    // Remember new blocking behavior
    if (blocking)
    {
        socketItem->flags = (uint16_t) (socketItem->flags & ~XME_HAL_NET_SOCKET_NONBLOCKING);
    }
    else
    {
        socketItem->flags = (uint16_t) (socketItem->flags | XME_HAL_NET_SOCKET_NONBLOCKING);
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_net_selectSocket
(
    xme_hal_net_socketHandle_t socketHandle,
    bool readSocket,
    bool writeSocket,
    uint16_t timeoutMs
)
{
    xme_hal_net_socketItem* socketItem;
    float tmp = (float)timeoutMs / 1000;
#if defined(_WIN32) && !defined(CYGWIN)
    struct fd_set readfds;
    struct fd_set writefds;
#else // #if defined(_WIN32) && !defined(CYGWIN)
    fd_set readfds;
    fd_set writefds;
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    struct timeval tv;

#if defined(_WIN32) && !defined(CYGWIN)
    int numDescriptors;
#else // #if defined(_WIN32) && !defined(CYGWIN)
    ssize_t numDescriptors;
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            XME_STATUS_INVALID_HANDLE,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    tv.tv_sec = (long int)tmp;
    tmp -= (float)tv.tv_sec;
    tv.tv_usec = (long int)(tmp * 1000000.0f);

    FD_ZERO(&readfds);
    FD_SET(socketItem->socket, &readfds);

    FD_ZERO(&writefds);
    FD_SET(socketItem->socket, &writefds);

    numDescriptors = select(socketItem->socket+1, readSocket ? &readfds : NULL, writeSocket ? &writefds : NULL, NULL, (0 == timeoutMs) ? 0 : &tv);

    XME_CHECK(0 != numDescriptors, XME_STATUS_TIMEOUT);
    XME_CHECK(SOCKET_ERROR != numDescriptors, XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_net_selectMultipleSockets
(
    uint16_t max_socket,
    xme_hal_net_socketHandle_t* socketSetRead,
    xme_hal_net_socketHandle_t* socketSetWrite,
    xme_hal_net_socketHandle_t* socketSetError,
    uint16_t timeoutMs
)
{
    xme_hal_net_socketItem* socketItem = NULL;
    xme_hal_net_socketHandle_t socketHandle;
    //float tmp = (float)timeoutMs / 1000;
#if defined(_WIN32) && !defined(CYGWIN)
    struct fd_set readfds;
    struct fd_set writefds;
    struct fd_set exceptfds;
#else // #if defined(_WIN32) && !defined(CYGWIN)
    fd_set readfds;
    fd_set writefds;
    fd_set exceptfds;
#endif // #if defined(_WIN32) && !defined(CYGWIN)
    struct timeval tv;
    uint16_t i;
    uint16_t maxInternalSocketHandle;


#if defined(_WIN32) && !defined(CYGWIN)
    int numDescriptors;
#else // #if defined(_WIN32) && !defined(CYGWIN)
    ssize_t numDescriptors;
#endif // #if defined(_WIN32) && !defined(CYGWIN)
    tv.tv_sec = timeoutMs / 1000;
    tv.tv_usec = timeoutMs % 1000;
    maxInternalSocketHandle = 0;

    FD_ZERO(&readfds);
    //TODO: REFACTORING => internal of if could be a function because it is similar! (Issue #1700)
    if (socketSetRead != NULL)
    {
        for (i = 0; i < max_socket; i++)
        {
            socketHandle = socketSetRead[i];
            if (socketHandle == XME_HAL_NET_INVALID_SOCKET_HANDLE)
            {
                continue;
            }

            xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
            {
                // Verify socket handle
                XME_CHECK_REC
                (
                    NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
                    XME_STATUS_INVALID_HANDLE,
                    {
                        xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
                    }
                );
            }
            xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

            FD_SET(socketItem->socket, &readfds);
            if (((uint16_t) socketItem->socket) > maxInternalSocketHandle )
            {
                maxInternalSocketHandle = (uint16_t)socketItem->socket;
            }
        }
    }

    FD_ZERO(&writefds);
    if (socketSetWrite != NULL)
    {
        for (i = 0; i < max_socket; i++)
        {
            socketHandle = socketSetWrite[i];
            if (socketHandle == XME_HAL_NET_INVALID_SOCKET_HANDLE)
            {
                continue;
            }

            xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
            {
                // Verify socket handle
                XME_CHECK_REC
                (
                    NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
                    XME_STATUS_INVALID_HANDLE,
                    {
                        xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
                    }
                );
            }
            xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

            FD_SET(socketItem->socket, &writefds);
            if (((uint16_t)socketItem->socket) > maxInternalSocketHandle)
            {
                maxInternalSocketHandle = (uint16_t)socketItem->socket;
            }
        }
    }

    FD_ZERO(&exceptfds);
    if (socketSetError != NULL)
    {
        for (i = 0; i < max_socket; i++)
        {
            socketHandle = socketSetError[i];
            if (socketHandle == XME_HAL_NET_INVALID_SOCKET_HANDLE)
            {
                continue;
            }

            xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
            {
                // Verify socket handle
                XME_CHECK_REC
                (
                    NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
                    XME_STATUS_INVALID_HANDLE,
                    {
                        xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
                    }
                );
            }
            xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

            FD_SET(socketItem->socket, &exceptfds);
            if (((uint16_t)socketItem->socket) > maxInternalSocketHandle)
            {
                maxInternalSocketHandle = (uint16_t)socketItem->socket;
            }
        }
    }

    numDescriptors = select(
        max_socket,
        (socketSetRead != NULL )? &readfds : NULL,
        (socketSetWrite != NULL ) ? &writefds : NULL,
        (socketSetError != NULL ) ? &exceptfds : NULL,
        (0 == timeoutMs) ? 0 : &tv );

    XME_CHECK(0 != numDescriptors, XME_STATUS_TIMEOUT);
    XME_CHECK(SOCKET_ERROR != numDescriptors, XME_STATUS_INTERNAL_ERROR);

    if (socketSetRead != NULL)
    {
        for (i = 0; i < max_socket; i++)
        {
            socketHandle = socketSetRead[i];
            if (socketHandle == XME_HAL_NET_INVALID_SOCKET_HANDLE) continue;
            socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle);
            XME_CHECK(NULL != socketItem, XME_STATUS_INTERNAL_ERROR);
            if (FD_ISSET(socketItem->socket, &readfds)) continue;
            socketSetRead[i] = XME_HAL_NET_INVALID_SOCKET_HANDLE;
        }
    }

    if (socketSetWrite != NULL)
    {
        for (i = 0; i < max_socket; i++)
        {
            socketHandle = socketSetWrite[i];
            if (socketHandle == XME_HAL_NET_INVALID_SOCKET_HANDLE) continue;
            socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle);
            XME_CHECK(NULL != socketItem, XME_STATUS_INTERNAL_ERROR);
            if (FD_ISSET(socketItem->socket, &writefds)) continue;
            socketSetWrite[i] = XME_HAL_NET_INVALID_SOCKET_HANDLE;
        }
    }

    if (socketSetError != NULL)
    {
        for (i = 0; i < max_socket; i++)
        {
            socketHandle = socketSetError[i];
            if (socketHandle == XME_HAL_NET_INVALID_SOCKET_HANDLE) continue;
            socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle);
            XME_CHECK(NULL != socketItem, XME_STATUS_INTERNAL_ERROR);
            if (FD_ISSET(socketItem->socket, &exceptfds)) continue;
            socketSetError[i] = XME_HAL_NET_INVALID_SOCKET_HANDLE;
        }
    }

    return XME_STATUS_SUCCESS;
}

xme_hal_net_socketHandle_t
xme_hal_net_waitForConnection
(
    xme_hal_net_socketHandle_t socketHandle
)
{
    xme_hal_net_socketItem* socketItem;
    bool wasNonBlocking;
    xme_hal_net_address_t peerAddr;
    int addrLength;
    SOCKET connectedSocket;
    xme_hal_net_socketHandle_t connectedSocketHandle;
    xme_hal_net_socketItem* connectedSocketItem = NULL;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            XME_HAL_NET_INVALID_SOCKET_HANDLE,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    // This only works on the server side
    XME_CHECK(0 == socketItem->hostname, XME_HAL_NET_INVALID_SOCKET_HANDLE);

    // This only works for connection-oriented sockets
    XME_CHECK(XME_HAL_NET_SOCKET_TCP & socketItem->flags, XME_HAL_NET_INVALID_SOCKET_HANDLE);

    // Set the socket to blocking regardless of the XME_HAL_NET_SOCKET_NONBLOCKING flag
    wasNonBlocking = (socketItem->flags & XME_HAL_NET_SOCKET_NONBLOCKING);
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_net_setBlockingBehavior(socketHandle, (bool) true),
              XME_HAL_NET_INVALID_SOCKET_HANDLE);

    xme_hal_mem_set(&peerAddr, 0, sizeof(peerAddr));

    addrLength = sizeof(peerAddr);
    connectedSocket = accept(socketItem->socket, (struct sockaddr*) &peerAddr.sa, (socklen_t *)&addrLength);

    // Restore intended blocking behavior
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_net_setBlockingBehavior(socketHandle, (bool) !wasNonBlocking),
              XME_HAL_NET_INVALID_SOCKET_HANDLE);

    // Check whether a client has connected
    XME_CHECK(INVALID_SOCKET != connectedSocket, XME_HAL_NET_INVALID_SOCKET_HANDLE);

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Allocate a unique socket handle for the client connection
        connectedSocketHandle = (xme_hal_net_socketHandle_t)XME_HAL_TABLE_ADD_ITEM(xme_hal_net_config.sockets);
        XME_CHECK_REC
        (
            XME_HAL_NET_INVALID_SOCKET_HANDLE != connectedSocketHandle,
            XME_HAL_NET_INVALID_SOCKET_HANDLE,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    // Initialize the socket descriptor
    connectedSocketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, connectedSocketHandle);
    XME_CHECK(NULL != connectedSocketItem, XME_HAL_NET_INVALID_SOCKET_HANDLE);
    connectedSocketItem->intf = socketItem->intf;
    connectedSocketItem->flags = socketItem->flags;
    connectedSocketItem->hostname = socketItem->hostname;
    connectedSocketItem->port = socketItem->port;
    connectedSocketItem->socket = connectedSocket;
    xme_hal_mem_set(&connectedSocketItem->peerAddr, 0, sizeof(connectedSocketItem->peerAddr));
    xme_hal_mem_copy(&connectedSocketItem->peerAddr, &peerAddr, (size_t) addrLength);
    connectedSocketItem->connected = true;

    // Also apply the intended blocking behavior to the client socket
    XME_CHECK_REC
    (
        XME_STATUS_SUCCESS == xme_hal_net_setBlockingBehavior(connectedSocketHandle, (bool) !wasNonBlocking),
        XME_HAL_NET_INVALID_SOCKET_HANDLE,
        {
            closesocket(socketItem->socket);
            XME_HAL_TABLE_REMOVE_ITEM(xme_hal_net_config.sockets, (xme_hal_table_rowHandle_t)connectedSocketHandle);
        }
    );

    // Return the connected socket handle
    return connectedSocketHandle;
}

uint16_t
xme_hal_net_readSocket
(
    xme_hal_net_socketHandle_t socketHandle,
    void* buffer,
    uint16_t count
)
{
    xme_hal_net_socketItem* socketItem;

#if defined(_WIN32) && !defined(CYGWIN)
    int numBytes;
#else // #if defined(_WIN32) && !defined(CYGWIN)
    ssize_t numBytes;
    // the control data is dumped here
    char controlBuffer[5000];//sizeof(struct in6_pktinfo) + sizeof(struct cmsghdr)];
    // the remote/source sockaddr is put here - not necessary in this case
    xme_hal_net_address_t peeraddr;
    struct iovec  iov[1];
    struct msghdr mh;
    struct cmsghdr *cmsg;
    struct in_pktinfo *pi;


#endif // #if defined(_WIN32) && !defined(CYGWIN)

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            0,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    xme_hal_mem_set(buffer, 0, count);

#if defined(_WIN32) && !defined(CYGWIN)
{
    GUID WSARecvMsg_GUID = WSAID_WSARECVMSG;
    LPFN_WSARECVMSG WSARecvMsg;
    DWORD NumberOfBytes;
    WSAMSG WSAMsg;
    unsigned long bNr;
    WSABUF WSABuf;
    WSACMSGHDR *pCMsgHdr;

    DWORD flags;

    // Memory for packet headers. The only activated header should be the
    // PKT_INFO header. This means, the neccessary size is exactly the size
    // of the PKT_INFO header plus the generic header.
    char controlBuffer[sizeof(struct in6_pktinfo) + sizeof(WSACMSGHDR)];

    // Get function pointer for WSARecvMsg call.
    // Only with this function, the destination address
    // can be retrieved.
    if (WSAIoctl(socketItem->socket, SIO_GET_EXTENSION_FUNCTION_POINTER,
        &WSARecvMsg_GUID, sizeof WSARecvMsg_GUID,
        &WSARecvMsg, sizeof WSARecvMsg,
        &NumberOfBytes, NULL, NULL) == SOCKET_ERROR)
    {
        return 0;
    }

    // Initialize buffer for incoming data
    WSABuf.buf = (char *)buffer;
    WSABuf.len = count;

    // Initialize Parameters for call
    xme_hal_mem_set(&WSAMsg, 0, sizeof(WSAMsg));
    WSAMsg.name = 0; // Not interested in remote name
    WSAMsg.namelen = 0; // Not interested in remote name
    WSAMsg.lpBuffers = &WSABuf; // Buffer array for incoming data
    WSAMsg.dwBufferCount = 1; // Size of buffer array for incoming data
    WSAMsg.Control.buf = controlBuffer; // Buffer for packet header data
    WSAMsg.Control.len = sizeof(controlBuffer); // Available space for packet header data

    // TODO: Same function should be used here. But WSARecvMsg gives error for TCP sockets. See ticket #811
    flags = 0;
    if (socketItem->flags & XME_HAL_NET_SOCKET_UDP)
    {
        if (WSARecvMsg(socketItem->socket, &WSAMsg, &bNr, NULL, NULL ) == SOCKET_ERROR)
        {
            return 0;
        }
    } else {
        if (WSARecv(socketItem->socket, &WSABuf, 1, &bNr, &flags, NULL, NULL ) == SOCKET_ERROR)
        {
            return 0;
        }
    }
    numBytes = bNr;

    // Iterate through the packet headers and
    // extract destination IP address
    pCMsgHdr = WSA_CMSG_FIRSTHDR(&WSAMsg);
    while (pCMsgHdr)
    {
        if (pCMsgHdr->cmsg_level == IPPROTO_IP && pCMsgHdr->cmsg_type == IP_PKTINFO)
        {
            struct in_pktinfo *info;
            info = (struct in_pktinfo *)WSA_CMSG_DATA(pCMsgHdr);
            socketItem->peerAddr.sa_in.sin_addr.S_un.S_addr = info->ipi_addr.S_un.S_addr;
            break;
        }
        else if (pCMsgHdr->cmsg_level == IPPROTO_IPV6 && pCMsgHdr->cmsg_type == IPV6_PKTINFO)
        {
            struct in6_pktinfo *info;
            struct sockaddr_in6 *addr = &socketItem->peerAddr.sa_in6;
            info = (struct in6_pktinfo *)WSA_CMSG_DATA(pCMsgHdr);
            xme_hal_mem_copy((char *)&(addr->sin6_addr.u), &(info->ipi6_addr.u), sizeof(struct in6_addr));
            break;
        }
        pCMsgHdr = WSA_CMSG_NXTHDR(&WSAMsg, pCMsgHdr);
    }

}
#else // #if defined(_WIN32) && !defined(CYGWIN)

    /* Initialize the message header for receiving */
    (void) xme_hal_mem_set(&mh, 0, sizeof (mh));
    // the remote/source sockaddr is put here - not necessary in this case
    mh.msg_name = &peeraddr;
    mh.msg_namelen = sizeof(peeraddr);
    mh.msg_control = controlBuffer;
    mh.msg_controllen = 5000;//sizeof(struct in6_pktinfo) + sizeof(struct cmsghdr);
    iov->iov_base=buffer;
    iov->iov_len=count;
    mh.msg_iov = iov;
    mh.msg_iovlen = 1;

    numBytes = recvmsg(socketItem->socket, &mh, 0);

    for ( // iterate all the control headers
        cmsg = CMSG_FIRSTHDR(&mh);
        cmsg != NULL;
        cmsg = CMSG_NXTHDR(&mh, cmsg))
    {
        // ignore the control headers that don't match what we want
        if (!(cmsg->cmsg_level == IPPROTO_IP || cmsg->cmsg_level == IPPROTO_IPV6) || !(cmsg->cmsg_type == IP_PKTINFO || cmsg->cmsg_type == IPV6_PKTINFO))
        {
            continue;
        }

        if (cmsg->cmsg_level == IPPROTO_IP && cmsg->cmsg_type == IP_PKTINFO)
        {
            pi = (struct in_pktinfo *)CMSG_DATA(cmsg);
            socketItem->peerAddr.sa_in.sin_addr = pi->ipi_addr;
        }
        else if (cmsg->cmsg_level == IPPROTO_IPV6 && cmsg->cmsg_type == IPV6_PKTINFO)
        {
            struct in6_pktinfo *info;
            struct sockaddr_in6 *addr = &socketItem->peerAddr.sa_in6;
            info = (struct in6_pktinfo *)CMSG_DATA(cmsg);
            xme_hal_mem_copy(addr->sin6_addr.s6_addr, info->ipi6_addr.s6_addr, sizeof(struct in6_addr));
            break;
        }

        // at this point, peeraddr is the source sockaddr
        // pi->ipi_spec_dst is the destination in_addr
        // pi->ipi_addr is the receiving interface in_addr
    }

    if (socketItem->flags & XME_HAL_NET_SOCKET_IPV6)
    {
        struct sockaddr_in6 *addr6 = &peeraddr.sa_in6;
        xme_hal_mem_copy(socketItem->peerAddr.sa_in6.sin6_addr.s6_addr, addr6->sin6_addr.s6_addr, sizeof(struct in6_addr));
    }
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    //printf("\nReceived from peer: %d.%d.%d.%d\n", (uint8_t)socketItem->peerAddr.sa_data[2], (uint8_t) socketItem->peerAddr.sa_data[3], (uint8_t) socketItem->peerAddr.sa_data[4], (uint8_t) socketItem->peerAddr.sa_data[5]);

    XME_CHECK(SOCKET_ERROR != numBytes, 0);

    return (uint16_t) numBytes;
}

uint16_t
xme_hal_net_writeSocket
(
    xme_hal_net_socketHandle_t socketHandle,
    const void* buffer,
    uint16_t count
)
{
    xme_hal_net_socketItem* socketItem;

#if defined(_WIN32) && !defined(CYGWIN)
    int numBytes = 0;
#else // #if defined(_WIN32) && !defined(CYGWIN)
    ssize_t numBytes = 0;
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            0U,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    if (socketItem->connected)
    {
        // Dedicated connection
        numBytes = send(socketItem->socket, (const char*) buffer, count, 0);
    }
    else if (socketItem->flags & XME_HAL_NET_SOCKET_BROADCAST)
    {
        uint32_t addr = INADDR_BROADCAST;
        // Broadcast
        numBytes = xme_hal_net_writeDatagram_ipaddress
        (
            socketItem,
            (void *)&addr,
            socketItem->port,
            buffer,
            count
        );
    }
    else
    {
        // Length of address struct depends on IP version
        size_t addrlen;

        // Reply
        if (socketItem->flags & XME_HAL_NET_SOCKET_IPV6)
        {
            addrlen = sizeof(struct sockaddr_in6);
        }
        else
        {
            addrlen = sizeof(struct sockaddr_in);
        }

        numBytes = sendto(socketItem->socket, (const char*) buffer, count, 0, (const struct sockaddr*) &socketItem->peerAddr.sa, addrlen);
    }

#if defined(_WIN32) && !defined(CYGWIN)
    XME_CHECK_MSG
    (
        SOCKET_ERROR != numBytes,
        0U,
        XME_LOG_DEBUG,
        "[xme_hal_net_writeSocket] returned error code %d\n",
        (int) WSAGetLastError()
    );
#else // #if defined(_WIN32) && !defined(CYGWIN)
    XME_CHECK_MSG
    (
        -1 != numBytes,
        0U,
        XME_LOG_DEBUG,
        "[xme_hal_net_writeSocket] returned error code %d\n",
        errno
    );
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    XME_ASSERT_RVAL(numBytes > 0, 0U);
    return (uint16_t) numBytes;
}

uint16_t
xme_hal_net_writeDatagram
(
    xme_hal_net_socketHandle_t socketHandle,
    xme_com_interface_address_t* remoteAddress,
    const void* buffer,
    uint16_t count
)
{
    uint32_t ip;
    uint16_t port;
    xme_hal_net_socketItem* socketItem;
    port = 0;
    ip = 0;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            0,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    // Translate xme_com_interface_address_t to generic address and port depending on the socket's IP version
    if (socketItem->flags & XME_HAL_NET_SOCKET_IPV6)
    {
        return xme_hal_net_writeDatagram_ipaddress(
            socketItem,
            (void *)remoteAddress,
            GENERIC_ADDRESS_TO_PORT_IP6(remoteAddress),
            buffer,
            count);
    }

    //So we should be dealing with IPv4 address.

    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4(remoteAddress, &ip, &port), XME_STATUS_INVALID_PARAMETER);

    // Convert port to host byte order
    port = ntohs(port);

    return xme_hal_net_writeDatagram_ipaddress(
            socketItem,
            (void*)&ip,
            port,
            buffer,
            count);
}

uint16_t
xme_hal_net_writeDatagram_ipaddress
(
    xme_hal_net_socketItem* socketItem,
    void* remoteAddress,
    uint16_t remotePort,
    const void* buffer,
    uint16_t count
)
{
    xme_hal_net_address_t broadcastAddr;
    size_t addrlen;

#if defined(_WIN32) && !defined(CYGWIN)
    int numBytes;
#else // #if defined(_WIN32) && !defined(CYGWIN)
    ssize_t numBytes;
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    if (socketItem->connected)
    {
        return 0U;
    }

    if (socketItem->flags & XME_HAL_NET_SOCKET_IPV6)
    {
        // Create struct sockaddr from IPv6 address
        struct sockaddr_in6 *Addr = &broadcastAddr.sa_in6;
        xme_hal_mem_set(&broadcastAddr, 0, sizeof(struct sockaddr_in6));
        Addr->sin6_family = AF_INET6;
        Addr->sin6_port = htons(remotePort);
        xme_hal_mem_copy((char *)Addr->sin6_addr.s6_addr, (char *)remoteAddress, sizeof(struct sockaddr_in6) );
        addrlen = sizeof(struct sockaddr_in6);
    }
    else
    {
        // Create struct sockaddr from IPv4 address
        struct sockaddr_in *Addr = &broadcastAddr.sa_in;
        xme_hal_mem_set(&broadcastAddr, 0, sizeof(struct sockaddr_in));
        Addr->sin_family = AF_INET;
        Addr->sin_port = htons(remotePort);
        Addr->sin_addr.s_addr = *((uint32_t *)remoteAddress);
        addrlen = sizeof(struct sockaddr_in);
    }

    numBytes = sendto(socketItem->socket, (char*)buffer, count, 0, (const struct sockaddr*) &broadcastAddr.sa, addrlen);

#if defined(_WIN32) && !defined(CYGWIN)
    XME_CHECK_MSG
    (
        SOCKET_ERROR != numBytes,
        0U,
        XME_LOG_DEBUG,
        "[xme_hal_net_writeDatagram_ipaddress] returned error code %d\n",
        (int) WSAGetLastError()
    );
#else // #if defined(_WIN32) && !defined(CYGWIN)
    XME_CHECK_MSG
    (
        -1 != numBytes,
        0U,
        XME_LOG_DEBUG,
        "[xme_hal_net_writeDatagram_ipaddress] returned error code %d\n",
        errno
    );
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    XME_ASSERT_RVAL(numBytes > 0, 0U);
    return (uint16_t) numBytes;
}

xme_status_t
xme_hal_net_getInterfaceAddr
(
    char* interfaceName,
    xme_com_interface_addressType_t addressType,
    char* ipAddress
)
{
    int family;
   
    // The following condition helps us to expand the code for IPv6 
    if (XME_COM_INTERFACE_ADDRESS_TYPE_IPV4 == addressType)
    {
        family = AF_INET;
    }
    else
    {
        return XME_STATUS_INVALID_PARAMETER;
    }

    XME_CHECK(NULL != ipAddress, XME_STATUS_INVALID_PARAMETER);

#if defined(_WIN32) && !defined(CYGWIN)
    {
        /* The following code is adapted from MSDN page which is released under Microsoft Limited Public License.
         * http://msdn.microsoft.com/en-us/cc300389.aspx
         */
        ULONG buffferLen = 15000;
        DWORD returnVal = 0;
        ULONG flags = GAA_FLAG_INCLUDE_PREFIX;
        PIP_ADAPTER_ADDRESSES pAddresses = NULL;

        if (NULL != interfaceName && '[' != interfaceName[0])
        {
            XME_LOG(XME_LOG_ERROR, "[xme_hal_net_getInterfaceAddr]: The format given is hostname which is currently not supported\n");
            return XME_STATUS_INTERNAL_ERROR;
        }
        pAddresses = (IP_ADAPTER_ADDRESSES *) xme_hal_mem_alloc(buffferLen);
        XME_ASSERT(NULL != pAddresses);

        returnVal = GetAdaptersAddresses(family, flags, NULL, pAddresses, &buffferLen);

        if (returnVal == ERROR_BUFFER_OVERFLOW)
        {
            xme_hal_mem_free(pAddresses);
            pAddresses = NULL;
            XME_LOG(XME_LOG_ERROR, "[xme_hal_net_getInterfaceAddr] The buffer used to read the network address is too small.\n");
            return XME_STATUS_BUFFER_TOO_SMALL;
        }

        if (NO_ERROR == returnVal)
        {
            PIP_ADAPTER_ADDRESSES pCurrAddresses = pAddresses;
            while (pCurrAddresses)
            {
                // This gets the length of buffer needed for the converted string.
                // This also includes the space for terminating null charachter.
                int friendlyNameLength = WideCharToMultiByte(CP_UTF8, 0, pCurrAddresses->FriendlyName, -1, NULL, 0, NULL, NULL);
                char *bufferFriendlyName = (char *) xme_hal_mem_alloc(friendlyNameLength);
                // The conversion
                int status = WideCharToMultiByte(CP_OEMCP, 0, pCurrAddresses->FriendlyName, -1, bufferFriendlyName, friendlyNameLength, NULL, NULL);
                XME_CHECK(0 != status, XME_STATUS_INTERNAL_ERROR);

                if (IfOperStatusUp == pCurrAddresses->OperStatus &&
                    (NULL == interfaceName || //interface == NULL => first working IP address => can be loopback too
                     0 == strncmp((&interfaceName[1]), pCurrAddresses->AdapterName, (strlen(pCurrAddresses->AdapterName))) || 
                     0 == strncmp((&interfaceName[1]), bufferFriendlyName, (strlen(interfaceName)-2))))
                {
                    PIP_ADAPTER_UNICAST_ADDRESS pUnicast = pCurrAddresses->FirstUnicastAddress;
                    if (pUnicast != NULL)
                    {
                        uint32_t tempIP = (uint32_t)((struct sockaddr_in *)(pUnicast->Address.lpSockaddr))->sin_addr.S_un.S_addr;
                        xme_hal_mem_free(pAddresses);
                        XME_CHECK
                        (
                            0 < xme_hal_safeString_snprintf(ipAddress, XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE, "%u.%u.%u.%u",
                                                                      (uint8_t)(0xFF & tempIP), (uint8_t)(0xFF & (tempIP>>8) ),
                                                                      (uint8_t)(0xFF & (tempIP>>16) ), (uint8_t)(0xFF & (tempIP>>24) ) ), 
                            XME_STATUS_INTERNAL_ERROR
                        );

                        return XME_STATUS_SUCCESS;
                    }
                }
                xme_hal_mem_free(bufferFriendlyName);
                pCurrAddresses = pCurrAddresses->Next;
            }
        } 
        xme_hal_mem_free(pAddresses);
    }
#else // #if defined(_WIN32) && !defined(CYGWIN)
    {
        /* The following code is adapted from getifaddrs man page which is released under Verbatim license.
         * http://man7.org/linux/man-pages/man3/getifaddrs.3.license.html
         */

        struct ifaddrs *intfaddr = NULL, *ifa = NULL;

        if (NULL != interfaceName && '[' != interfaceName[0])
        {
            XME_LOG(XME_LOG_ERROR, "[xme_hal_net_getInterfaceAddr]: The format given is hostname which is currently not supported\n");
            return XME_STATUS_INTERNAL_ERROR;
        }
    
        if (-1 == getifaddrs(&intfaddr))
        {
            XME_LOG(XME_LOG_DEBUG,"[getifaddrs] failed because of errno %d\n", errno);
            return XME_STATUS_INTERNAL_ERROR;
        }
        for (ifa = intfaddr; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr->sa_family == family)
            {
                // Find the adapter which is up and running and has the given interface name
                 if (ifa->ifa_flags & IFF_UP &&  ifa->ifa_flags & IFF_RUNNING &&
                     (NULL == interfaceName || //interface == NULL => first working IP address => can be loopback too
                      0 == strncmp((&interfaceName[1]), ifa->ifa_name, strlen(ifa->ifa_name))))
                {
                    if (AF_INET == family)
                    {
                        uint32_t tempIP = ((struct sockaddr_in*)(ifa->ifa_addr))->sin_addr.s_addr;
                        XME_CHECK
                        (
                            0 < xme_hal_safeString_snprintf(ipAddress, XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE, "%u.%u.%u.%u",
                                                                      (uint8_t)(0xFF & tempIP), (uint8_t)(0xFF & (tempIP>>8) ),
                                                                      (uint8_t)(0xFF & (tempIP>>16) ), (uint8_t)(0xFF & (tempIP>>24) ) ), 
                            XME_STATUS_INTERNAL_ERROR
                        );
                    }
                // Else can be added to get the IPv6
                // But remember IPv6 address is stored as "unsigned char s6_addr[16]" in struct in6_addr
                   freeifaddrs(intfaddr);
                   return XME_STATUS_SUCCESS;
                }
            }
        }
        freeifaddrs(intfaddr);
    }
#endif // #if defined(_WIN32) && !defined(CYGWIN)
    return XME_STATUS_INTERNAL_ERROR;
}
    
xme_status_t
xme_hal_net_joinMulticastGroup
(
    xme_hal_net_socketHandle_t socketHandle,
    xme_com_interface_address_t* remoteAddress
)
{
    xme_hal_net_socketItem* socketItem;
    struct ip_mreq imr;
    int ret;
    uint32_t ip = 0;
    uint16_t port = 0;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            XME_STATUS_INVALID_HANDLE,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    // TODO: Find a way to handle IPv4 and IPv6
    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4(remoteAddress, &ip, &port), XME_STATUS_INVALID_PARAMETER);
    imr.imr_multiaddr.s_addr = ip;
    imr.imr_interface.s_addr = INADDR_ANY;

    ret = setsockopt(socketItem->socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &imr, sizeof(imr));

    XME_CHECK(0 == ret, XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_net_leaveMulticastGroup(xme_hal_net_socketHandle_t socketHandle, xme_com_interface_address_t* remoteAddress)
{
    xme_hal_net_socketItem* socketItem;
    struct ip_mreq imr;
    int ret;
    uint32_t ip = 0;
    uint16_t port = 0;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            XME_STATUS_INVALID_HANDLE,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    // TODO: Find a way to handle IPv4 and IPv6
    XME_CHECK(XME_STATUS_SUCCESS == xme_com_interface_genericAddressToIPv4(remoteAddress, &ip, &port), XME_STATUS_INVALID_PARAMETER);
    imr.imr_multiaddr.s_addr = ip;
    imr.imr_interface.s_addr = INADDR_ANY;

    ret = setsockopt(socketItem->socket, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char *) &imr, sizeof(imr));

    XME_CHECK(0 == ret, XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

void
xme_hal_net_getAddressOfLastReception
(
    xme_hal_net_socketHandle_t socketHandle, xme_com_interface_address_t* remoteAddress
)
{
    xme_hal_net_socketItem* socketItem;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            ,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    if (socketItem->flags & XME_HAL_NET_SOCKET_IPV6 )
    {
        struct sockaddr_in6* sAdd6 = &socketItem->peerAddr.sa_in6;

#if defined(_WIN32) && !defined(CYGWIN)
        xme_hal_mem_copy((char *)remoteAddress, &(sAdd6->sin6_addr.u), sizeof(struct in6_addr));
#else // #if defined(_WIN32) && !defined(CYGWIN)
        xme_hal_mem_copy((char *)remoteAddress, sAdd6->sin6_addr.s6_addr, sizeof(struct in6_addr));
#endif // #if defined(_WIN32) && !defined(CYGWIN)
        GENERIC_ADDRESS_TO_PORT(remoteAddress) = xme_hal_net_ntohs(sAdd6->sin6_port);
    }
    else
    {
        struct sockaddr_in* sAdd = &socketItem->peerAddr.sa_in;
#if defined(_WIN32) && !defined(CYGWIN)
        GENERIC_ADDRESS_TO_IP4(remoteAddress) = xme_hal_net_ntohl(sAdd->sin_addr.S_un.S_addr);
#else // #if defined(_WIN32) && !defined(CYGWIN)
        GENERIC_ADDRESS_TO_IP4(remoteAddress) = xme_hal_net_ntohl(sAdd->sin_addr.s_addr);
#endif // #if defined(_WIN32) && !defined(CYGWIN)
        GENERIC_ADDRESS_TO_PORT(remoteAddress) = xme_hal_net_ntohs(sAdd->sin_port);
    }
}

uint16_t
xme_hal_net_get_available_data_size
(
    xme_hal_net_socketHandle_t socketHandle
)
{
    xme_hal_net_socketItem* socketItem;
    unsigned long dataCount = 0;

    xme_hal_sync_enterCriticalSection(xme_hal_net_config.criticalSectionHandle);
    {
        // Verify socket handle
        XME_CHECK_REC
        (
            NULL != (socketItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(xme_hal_net_config.sockets, socketHandle)),
            0,
            {
                xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);
            }
        );
    }
    xme_hal_sync_leaveCriticalSection(xme_hal_net_config.criticalSectionHandle);

    // Determine number of bytes ready to be read
#if defined(_WIN32) && !defined(CYGWIN)
    {
        DWORD ret;
        XME_CHECK
        (
            0 == WSAIoctl(socketItem->socket, FIONREAD, NULL, 0, &dataCount, sizeof(dataCount), &ret, NULL, NULL ),
            0
        );
    }
#else // #if defined(_WIN32) && !defined(CYGWIN)
    XME_CHECK(0 == ioctl(socketItem->socket, FIONREAD, &dataCount),    0);
#endif // #if defined(_WIN32) && !defined(CYGWIN)
    return (uint16_t)XME_HAL_MATH_MIN(dataCount, 0xFFFF);
}

uint16_t
xme_hal_net_htons(uint16_t hostshort)
{
    return htons(hostshort);
}

uint32_t
xme_hal_net_htonl(uint32_t hostlong)
{
    return htonl(hostlong);
}

uint64_t
xme_hal_net_htonll(uint64_t hostlonglong)
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
xme_hal_net_ntohs(uint16_t netshort)
{
    return ntohs(netshort);
}

uint32_t
xme_hal_net_ntohl(uint32_t netlong)
{
    return ntohl(netlong);
}

uint64_t
xme_hal_net_ntohll(uint64_t netlonglong)
{
    return xme_hal_net_htonll(netlonglong);
}

/**
 * @}
 */
