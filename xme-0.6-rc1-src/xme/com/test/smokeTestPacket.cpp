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
 * $Id: smokeTestPacket.cpp 5038 2013-09-11 08:32:42Z gulati $
 */

/**
 * \file
 *         Packet smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/com/packet.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class PacketSmokeTest: public ::testing::Test
{
protected:
    PacketSmokeTest() :
    str("Hello World")
    {
        xme_hal_sync_init();
        xme_hal_sharedPtr_init();
    }

    virtual ~PacketSmokeTest()
    {
        xme_hal_sync_fini();
        xme_hal_sharedPtr_fini();
    }
    const char* str;
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     PacketSmokeTest                                                     //
//----------------------------------------------------------------------------//

TEST_F(PacketSmokeTest, sunnyDayTests)
{
    xme_com_packet_sample_t pktSample1;
    xme_com_packet_sample_t pktSample2;
    uint16_t pktSample1Size = 0;
    uint16_t pktSample2Size = 0;
    
    pktSample1 = xme_com_packet_sample_create_payloadSize(sizeof(str));
    pktSample1Size = xme_com_packet_sample_getCompleteSize(pktSample1);
    pktSample2 = xme_com_packet_sample_create_totalSize(pktSample1Size); 
    pktSample2Size = xme_com_packet_sample_getCompleteSize(pktSample2);

    XME_COM_PACKET_INIT(*xme_com_packet_sample_header(pktSample2), XME_COM_PACKET_HEADER_TYPE_SAMPLE); 

    EXPECT_TRUE(XME_COM_PACKET_VALID(*(xme_com_packet_sample_header(pktSample1))));
    EXPECT_EQ(XME_COM_PACKET_HEADER_TYPE_SAMPLE, XME_COM_PACKET_TYPE(*(xme_com_packet_sample_header(pktSample2))));

    EXPECT_EQ(pktSample1Size, pktSample2Size);

    EXPECT_EQ(sizeof(str), xme_com_packet_sample_getPayloadSize(pktSample2));

    xme_hal_sharedPtr_destroy(pktSample1);
    xme_hal_sharedPtr_destroy(pktSample2);
}
/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
