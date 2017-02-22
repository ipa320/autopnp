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
 * $Id: neighborhoodDetection.c 7829 2014-03-14 10:29:33Z ruiz $
 */

/**
 * \file
 *         Neighborhood Detection.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/nodeManager/include/neighborhoodDetection.h"

#include "xme/core/log.h"
#include "xme/hal/include/net.h"
#include "xme/hal/include/mem.h"
#include "xme/com/packet.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief The neighbor list.
 */
static xme_core_nodeManager_neighborhoodDetection_neighborsList_t neighborList;

/**
 * \brief The current node ID.
 */
static const xme_core_node_nodeId_t* currentNodeId;

/**
 * \brief The overflow flag.
 */
static bool overflow;

/**
 * \brief The socket list.
 */
static xme_core_nodeManager_neighborhoodDetection_socketList_t socketsList;

/**
 * \brief The received packet.
 */
//xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t* receivedPacket;

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/*
 * 239.192.x.x is chosen as the address range for XME multicast communication.
 *
 * While the first two bytes of the IP address are fixed, the third and fourth byte
 * represent the channel number. The port is fixed to the value XME_HAL_NET_IP_PORT_NUMBER_DATA_CENTRIC_COMMUNICATION.
 *
 * RFC:
 * "239.192.0.0/14 is defined to be the IPv4 Organization Local Scope,
 * and is the space from which an organization should allocate sub-
 * ranges when defining scopes for private use."
 *
 * http://tools.ietf.org/html/rfc2365
 *
 */
//TODO: Find a way to manage IPv4 and IPv6
/**
 * \brief Converts the addresses to multicast addresses.
 */
#define CHANNEL_TO_MULTICAST_ADDRESS(channel, address) \
    do { \
        (void) xme_com_interface_ipv4ToGenericAddress \
        ( \
            239U, 192U, \
            (uint8_t) (channel >> 8), \
            (uint8_t) (channel & 0xFF), \
            XME_HAL_NET_IP_PORT_NUMBER_DATA_CENTRIC_COMMUNICATION, \
            (address) \
        ); \
    } while (0)

/**
 * \brief Value of the announcement channel
 */
#define XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_ANNOUNCEMENT_CHANNEL 302


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief Writing data to the network.
 *
 * \param[in] channel The channel the data should be sent to .
 * \param[in] shmHandle Handle to the shared memory area.
 * \param[in] address The address of the sending node.
 *
 * \return Returns the number of bytes that have actually been processed. If and only
 *         if an error occoured, the number of written bytes does not match the size
 *         of the shared memory area.
 */
static uint16_t
xme_core_nodeManager_neighborhoodDetection_write_blocking
(
    uint32_t channel,
    xme_hal_sharedPtr_t shmHandle,
    xme_com_interface_address_t address
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
static uint16_t
xme_core_nodeManager_neighborhoodDetection_write_blocking
(
    uint32_t channel,
    xme_hal_sharedPtr_t shmHandle,
    xme_com_interface_address_t address
)
{
    xme_status_t status;
    uint16_t payloadSize = xme_hal_sharedPtr_getSize(shmHandle);
    uint16_t alreadySent = 0;
    uint16_t justSent = 0;
    uint16_t sampleSize;

    xme_com_packet_sample_t sample;
    xme_com_packet_sample_header_t* sampleHeader;
    void* samplePayload;

    sample = xme_com_packet_sample_create_payloadSize(payloadSize);
    if (sample == XME_HAL_SHAREDPTR_INVALID_POINTER)
    {
        return (int)XME_STATUS_OUT_OF_RESOURCES;
    }

    sampleHeader = xme_com_packet_sample_header(sample);
    samplePayload = xme_com_packet_sample_payload(sample);
    sampleSize = xme_hal_sharedPtr_getSize(sample);

    sampleHeader->senderGuid = (int)*currentNodeId;

    (void) xme_hal_mem_copy(samplePayload, xme_hal_sharedPtr_getPointer(shmHandle), (size_t)xme_hal_sharedPtr_getSize(shmHandle));

    //Send the packet out on all interfaces
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(socketsList, xme_hal_net_socketHandle_t, currentItem);
    {
        while(alreadySent != sampleSize)
        {
            CHANNEL_TO_MULTICAST_ADDRESS(channel, &address);
            //TODO: Check how the address has to be configured correctly
            status = xme_hal_net_setBlockingBehavior(*currentItem, false);
            if(status != XME_STATUS_SUCCESS)
            {
                return (int)status;
            }
            justSent = xme_hal_net_writeDatagram(*currentItem, &address, xme_hal_sharedPtr_getPointer(sample), xme_hal_sharedPtr_getSize(sample));

            // If nothing is sent in blocking mode, something went wrong.
            if(justSent == 0)
            {
                break;
            }

            alreadySent += justSent;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    xme_com_packet_sample_destroy(sample);
    
    return alreadySent;
}

//This function is used to test if something was received over the network
/*void
receiveFromNetwork(void)
{
    int count;
    char ipString[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];

    xme_com_packet_sample_t sample;
    void* samplePayload;
    sample = xme_hal_sharedPtr_create(50);
    if (sample == XME_HAL_SHAREDPTR_INVALID_POINTER)
    {
        return;
    }

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(socketsList, xme_hal_net_socketHandle_t, currentItem);
    {
        //xme_hal_net_setBlockingBehavior(*currentItem, false );
        count = xme_hal_net_readSocket(*currentItem, xme_hal_sharedPtr_getPointer(sample), 50);
        samplePayload = xme_com_packet_sample_payload(sample);
        receivedPacket = (xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t*) samplePayload;
        XME_LOG(XME_LOG_ALWAYS, "Received %d bytes\n", count);
        XME_LOG(XME_LOG_ALWAYS, "Received node ID: %d\n", receivedPacket->sendingNodeID);
        xme_com_interface_genericAddressToIPv4String(&(receivedPacket->sendingInterfaceID), ipString, sizeof(ipString));
        XME_LOG(XME_LOG_ALWAYS, "Received IP: %s\n", ipString);

    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    xme_hal_sharedPtr_destroy(sample);
}*/

xme_status_t
xme_core_nodeManager_neighborhoodDetection_init
(
    void* initStruct
)
{
    xme_status_t status;
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t* iterator;
    xme_com_interface_address_t *currentInterface;
    xme_com_interface_address_t address;
    xme_hal_net_socketHandle_t* socket;

    currentNodeId = (xme_core_node_nodeId_t*) initStruct;
    if(currentNodeId == NULL)
    {
        return XME_STATUS_INTERNAL_ERROR;
    }

    //Initialize the list of neighbors
    XME_HAL_SINGLYLINKEDLIST_INIT(neighborList);

    //Initialize the list of sockets
    XME_HAL_SINGLYLINKEDLIST_INIT(socketsList);

    //Assumption: The network abstraction is already initialized

    status = xme_core_directory_nodeRegistryController_initNodeInterfaceIterator(*currentNodeId, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &iterator);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    while(xme_core_directory_nodeRegistryController_hasNextInterface(iterator))
    {
        currentInterface = xme_core_directory_nodeRegistryController_nextInterface(iterator);

        socket = (xme_hal_net_socketHandle_t*) xme_hal_mem_alloc(sizeof(xme_hal_net_socketHandle_t*));
        *socket = xme_hal_net_createSocket(NULL, XME_HAL_NET_SOCKET_UDP | XME_HAL_NET_SOCKET_BROADCAST | XME_HAL_NET_SOCKET_MULTICAST, NULL, XME_HAL_NET_IP_PORT_NUMBER_DATA_CENTRIC_COMMUNICATION);
        status = xme_hal_net_openSocket(*socket);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        status = xme_hal_singlyLinkedList_addItem(&socketsList, socket);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        // Incoming traffic
        address = *currentInterface;
        CHANNEL_TO_MULTICAST_ADDRESS(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_ANNOUNCEMENT_CHANNEL, &address);
        status = xme_hal_net_joinMulticastGroup(*socket, &address);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);

        // Nothing to do for outgoing traffic
    }

    status = xme_core_directory_nodeRegistryController_finiNodeInterfaceIterator(iterator);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    return XME_STATUS_SUCCESS;
}

void
xme_core_nodeManager_neighborhoodDetection_fini(void)
{
    xme_com_interface_address_t address;
    if(currentNodeId == NULL)
    {
        return;
    }
    address.addressType = XME_COM_INTERFACE_ADDRESS_TYPE_IPV4;

    xme_hal_singlyLinkedList_fini(&neighborList);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(socketsList, xme_hal_net_socketHandle_t, currentItem);
    {
        xme_status_t status;

        // Incoming traffic
        CHANNEL_TO_MULTICAST_ADDRESS(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_ANNOUNCEMENT_CHANNEL, &address);

        status = xme_hal_net_leaveMulticastGroup(*currentItem, &address);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        status = xme_hal_net_closeSocket(*currentItem);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        status = xme_hal_net_destroySocket(*currentItem);
        XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);

        // Nothing to do for outgoing traffic
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    xme_hal_singlyLinkedList_fini(&socketsList);
}

void
xme_core_nodeManager_neighborhoodDetection_updateNeighborhoodInformation(void)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighborList, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentItem);
    {
        if(--(currentItem->lastSeenBeforeTicks) <= 0)
        {
            // TODO: Issue 3362: Both senderInterfaceID and receivingInterfaceID are structs. We should place a %d value there. 
            XME_LOG(
                    XME_LOG_VERBOSE,
                    "neighborhoodDetection: Removing entry from sender node %d sender interface %d receiving interface %d due to timeout\n",
                    currentItem->senderNodeID,
                    currentItem->senderInterfaceID,
                    currentItem->receivingInterfaceID
                );

                (void) xme_hal_singlyLinkedList_removeItem(&neighborList, currentItem, (bool) false);
                overflow = false;
                break;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
}

xme_status_t
xme_core_nodeManager_neighborhoodDetection_sendAnnouncement(void)
{
    xme_status_t status;
    xme_core_directory_nodeRegistryController_nodeInterfaceIterator_t* iterator;
    xme_com_interface_address_t *currentInterface;
    xme_hal_sharedPtr_t packetToSend;
    xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t* data;

    XME_CHECK(currentNodeId != NULL, XME_STATUS_INTERNAL_ERROR);

    packetToSend = xme_hal_sharedPtr_create((uint16_t)sizeof(xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t));
    data = (xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t*)xme_hal_sharedPtr_getPointer(packetToSend);
    data->sendingNodeID = *currentNodeId;
    XME_CHECK(XME_HAL_SHAREDPTR_INVALID_POINTER != packetToSend, XME_STATUS_INTERNAL_ERROR);

    status = xme_core_directory_nodeRegistryController_initNodeInterfaceIterator(*currentNodeId, XME_COM_INTERFACE_ADDRESS_TYPE_INVALID, &iterator);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    while(xme_core_directory_nodeRegistryController_hasNextInterface(iterator))
    {
        currentInterface = xme_core_directory_nodeRegistryController_nextInterface(iterator);
        data->sendingInterfaceID = *currentInterface;
        (void) xme_core_nodeManager_neighborhoodDetection_write_blocking(XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_ANNOUNCEMENT_CHANNEL, packetToSend, *currentInterface);
    }

    status = xme_core_directory_nodeRegistryController_finiNodeInterfaceIterator(iterator);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    xme_hal_sharedPtr_destroy(packetToSend);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_neighborhoodDetection_processReceivedAnnouncements
(
    const xme_core_nodeManager_neighborhoodDetection_nodeAnnouncements_t* receivedPacketAnnouncement,
    const xme_com_interface_address_t* receivedInterfaceId
)
{
    xme_status_t status;
    bool exists = false;
    char ip[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
    char receivingIP[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
    xme_core_nodeManager_neighborhoodDetection_neighborInformation_t* item = (xme_core_nodeManager_neighborhoodDetection_neighborInformation_t*) xme_hal_mem_alloc(sizeof(xme_core_nodeManager_neighborhoodDetection_neighborInformation_t));

    //Check whether we received valid packets
    XME_CHECK(NULL != currentNodeId, XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(NULL != receivedPacketAnnouncement, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != receivedInterfaceId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != receivedPacketAnnouncement->sendingNodeID, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_COM_INTERFACE_ADDRESS_TYPE_INVALID != receivedPacketAnnouncement->sendingInterfaceID.addressType, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_COM_INTERFACE_ADDRESS_TYPE_INVALID != receivedInterfaceId->addressType, XME_STATUS_INVALID_PARAMETER);
    //Assumption: The address in the received packet and the received interface are correct

    status = xme_com_interface_genericAddressToIPv4String(&receivedPacketAnnouncement->sendingInterfaceID, ip, sizeof(ip));
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_PARAMETER);

    status = xme_com_interface_genericAddressToIPv4String(receivedInterfaceId, receivingIP, sizeof(receivingIP));
    XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_PARAMETER);

    //Check whether we received this announcement before
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighborList, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentItem);
    {
        char senderIP[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];
        char receiverIP[XME_COM_INTERFACE_IPV4_STRING_BUFFER_SIZE];

        status = xme_com_interface_genericAddressToIPv4String(&currentItem->senderInterfaceID, senderIP, sizeof(senderIP));
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_PARAMETER);

        status = xme_com_interface_genericAddressToIPv4String(&currentItem->receivingInterfaceID, receiverIP, sizeof(receiverIP));
        XME_CHECK(XME_STATUS_SUCCESS == status, XME_STATUS_INVALID_PARAMETER);
        
        if(currentItem->senderNodeID == receivedPacketAnnouncement->sendingNodeID
            && currentItem->senderInterfaceID.addressType == receivedPacketAnnouncement->sendingInterfaceID.addressType
            && (0 == strcmp(senderIP, ip))
            && currentItem->receivingInterfaceID.addressType== receivedInterfaceId->addressType
            && (0 == strcmp(receiverIP, receivingIP)))
        {
            exists = true;
            currentItem->lastSeenBeforeTicks = XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL;
            break;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    if(!exists)
    {
        //Add the item to the list
        item->lastSeenBeforeTicks = XME_CORE_NODEMANAGER_NEIGHBORHOODDETECTION_STANDARD_VALUE_TICKS_BEFORE_REMOVAL;
        item->receivingInterfaceID = *receivedInterfaceId;
        item->senderInterfaceID = receivedPacketAnnouncement->sendingInterfaceID;
        item->senderNodeID = receivedPacketAnnouncement->sendingNodeID;

        status = xme_hal_singlyLinkedList_addItem(&neighborList, item);

        if(status == XME_STATUS_OUT_OF_RESOURCES)
        {
            XME_LOG(
                XME_LOG_WARNING,
                "neighborhoodDetection: Overflow. Not able to keep complete list of neighborhood.\n"
            );
            overflow = true;
            return XME_STATUS_INTERNAL_ERROR;
        }
    }
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_neighborhoodDetection_forwardNeighborhoodInformation(xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t* neighborhoodInfo)
{
    unsigned int i = 0;
    bool copyOverflow = false;

    XME_CHECK(currentNodeId != NULL, XME_STATUS_INTERNAL_ERROR);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(neighborList, xme_core_nodeManager_neighborhoodDetection_neighborInformation_t, currentItem);
    {
        if(i < NEIGHBOR_COUNT)
        {
            neighborhoodInfo->neighbors[i].receiverInterfaceId = currentItem->receivingInterfaceID;
            neighborhoodInfo->neighbors[i].receiverNodeId = *currentNodeId;
            neighborhoodInfo->neighbors[i].senderInterfaceId = currentItem->senderInterfaceID;
            neighborhoodInfo->neighbors[i].senderNodeId = currentItem->senderNodeID;
            i++;
        }
        else
        {
            copyOverflow = true;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    neighborhoodInfo->nodeId = *currentNodeId;
    neighborhoodInfo->overflow = overflow || copyOverflow;
    neighborhoodInfo->neighborsCount = xme_hal_singlyLinkedList_getItemCount(&neighborList);

    return XME_STATUS_SUCCESS;
}

xme_core_nodeManager_neighborhoodDetection_neighborsList_t
xme_core_nodeManager_neighborhoodDetection_getNeighborList(void)
{
    return neighborList;
}
