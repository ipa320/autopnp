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
 * $Id: packet.c 4945 2013-09-04 07:37:46Z ruiz $
 */

/**
 * \file
 *         Packet encapsulation.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/com/packet.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_com_packet_sample_t
xme_com_packet_sample_create_payloadSize
(
    uint16_t dataSize
)
{
    xme_com_packet_sample_t samplePtr;
    xme_com_packet_sample_header_t* sampleHeader;

    // Create sample packet of size dataSize
    samplePtr = xme_hal_sharedPtr_create(dataSize + (uint16_t)sizeof(xme_com_packet_sample_header_t));
    XME_CHECK(XME_HAL_SHAREDPTR_INVALID_POINTER != samplePtr, XME_HAL_SHAREDPTR_INVALID_POINTER);

    // Retrieve pointer from shared pointer handle
    sampleHeader = (xme_com_packet_sample_header_t*)xme_hal_sharedPtr_getPointer(samplePtr);
    XME_ASSERT_RVAL(NULL != sampleHeader, XME_HAL_SHAREDPTR_INVALID_POINTER);

    // Initialize common XME packet header and packet type field
    XME_COM_PACKET_INIT(*sampleHeader, XME_COM_PACKET_HEADER_TYPE_SAMPLE);

    return samplePtr;
} 

xme_com_packet_sample_t xme_com_packet_sample_create_totalSize(uint16_t totalSize)
{
    return xme_hal_sharedPtr_create( totalSize );
}

uint16_t xme_com_packet_sample_getPayloadSize(xme_com_packet_sample_t sample)
{
    return xme_hal_sharedPtr_getSize(sample) - (uint16_t)sizeof(xme_com_packet_sample_header_t);
}

uint16_t xme_com_packet_sample_getCompleteSize(xme_com_packet_sample_t sample)
{
    return xme_hal_sharedPtr_getSize(sample);
}

xme_com_packet_sample_header_t* xme_com_packet_sample_header(xme_com_packet_sample_t sample)
{
    xme_com_packet_sample_header_t* hdr =
        (xme_com_packet_sample_header_t *)xme_hal_sharedPtr_getPointer(sample);
    return hdr;
}


void* xme_com_packet_sample_payload(xme_com_packet_sample_t sample)
{
    void* payload = 
        ((xme_com_packet_sample_header_t *)xme_hal_sharedPtr_getPointer(sample))+1;
    return payload;
}

void xme_com_packet_sample_destroy(xme_com_packet_sample_t sample)
{
    xme_hal_sharedPtr_destroy(sample);
}
