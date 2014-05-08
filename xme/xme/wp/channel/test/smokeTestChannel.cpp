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
 * $Id: smokeTestChannel.cpp 6684 2014-02-07 14:34:26Z geisinger $
 */

/**
 * \file
 *         UDP Waypoint interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include <gtest/gtest.h>

#include "xme/wp/channel/include/channelInjector.h"
#include "xme/wp/channel/include/channelSelector.h"

#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/log.h"
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class ChannelWaypointSmokeTest : public ::testing::Test
{
protected:
    ChannelWaypointSmokeTest()
    {
        xme_wp_channel_channelInjector_init();
        xme_wp_channel_channelSelector_init();
    }

    virtual ~ChannelWaypointSmokeTest()
    {
        xme_wp_channel_channelSelector_fini();
        xme_wp_channel_channelInjector_fini();
    }
    xme_wp_waypoint_instanceId_t ciw_instanceId, csw_instanceId, csw_instanceId1;
    xme_core_dataManager_dataPacketId_t inputPort;
    xme_core_dataManager_dataPacketId_t outputPort;

};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     UDPWaypointInterfaceTest                                               //
//----------------------------------------------------------------------------//

TEST_F(ChannelWaypointSmokeTest, sunnyDayTests)
{
    inputPort = (xme_core_dataManager_dataPacketId_t) 1;
    outputPort = (xme_core_dataManager_dataPacketId_t) 2;
    uint32_t bytesRead=0, chanelId=231;
    uint64_t tempIn = 321 , tempOut = 0;
    char data[] = "HelloWorld";
    char bufferIn[100];
    char bufferOut[100];

    xme_core_dataHandler_writeData(inputPort, data, strlen(data));
    xme_core_dataHandler_writeAttribute(inputPort, (xme_core_attribute_key_t) XME_CORE_ATTRIBUTE_KEY_CHANNELID, &chanelId, sizeof(uint32_t)); 
    xme_core_dataHandler_writeAttribute(inputPort, (xme_core_attribute_key_t) (XME_CORE_ATTRIBUTE_KEY_CHANNELID+1), &tempIn, sizeof(uint64_t)); 

    xme_wp_channel_channelInjector_addConfig(&ciw_instanceId, inputPort, outputPort, (xme_core_topic_t) 1, strlen(data), bufferIn, sizeof(bufferIn), (xme_core_channelId_t)10);
    xme_wp_channel_channelInjector_run(ciw_instanceId);

    xme_core_dataHandler_readData(outputPort, bufferOut, strlen(data), &bytesRead);
    bufferOut[bytesRead] = 0;
    EXPECT_STREQ("HelloWorld", bufferOut);
    xme_core_dataHandler_readAttribute(outputPort, (xme_core_attribute_key_t) XME_CORE_ATTRIBUTE_KEY_CHANNELID, &chanelId, sizeof(uint32_t), &bytesRead); 
    EXPECT_EQ(10u,chanelId);
    xme_core_dataHandler_readAttribute(outputPort, (xme_core_attribute_key_t) (XME_CORE_ATTRIBUTE_KEY_CHANNELID+1), &tempOut, sizeof(uint64_t), &bytesRead); 
    EXPECT_EQ(tempIn, tempOut);

    inputPort = (xme_core_dataManager_dataPacketId_t) 2;
    outputPort = (xme_core_dataManager_dataPacketId_t) 3;
    tempOut = 0;
    xme_hal_mem_set(bufferIn, 0x0, sizeof(bufferIn));
    xme_hal_mem_set(bufferOut, 0x0, sizeof(bufferOut));

    xme_wp_channel_channelSelector_addConfig(&csw_instanceId, inputPort, outputPort, (xme_core_topic_t) 1, strlen(data), bufferIn, sizeof(bufferIn), (xme_core_channelId_t)10, (xme_core_channelId_t)30);
    xme_wp_channel_channelSelector_addConfig(&csw_instanceId1, inputPort, (xme_core_dataManager_dataPacketId_t) 4, (xme_core_topic_t) 1, strlen(data), bufferIn, sizeof(bufferIn), (xme_core_channelId_t)20, (xme_core_channelId_t)40);
    xme_wp_channel_channelSelector_run(csw_instanceId);

    xme_core_dataHandler_readData(outputPort, bufferOut, strlen(data), &bytesRead);
    bufferOut[bytesRead] = 0;
    EXPECT_STREQ("HelloWorld", bufferOut);
    xme_core_dataHandler_readAttribute(outputPort, (xme_core_attribute_key_t) XME_CORE_ATTRIBUTE_KEY_CHANNELID, &chanelId, sizeof(uint32_t), &bytesRead); 
    EXPECT_EQ(30u, chanelId);
    xme_core_dataHandler_readAttribute(outputPort, (xme_core_attribute_key_t) (XME_CORE_ATTRIBUTE_KEY_CHANNELID+1), &tempOut, sizeof(uint64_t), &bytesRead); 
    EXPECT_EQ(tempIn, tempOut);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_channel_channelInjector_removeConfig(ciw_instanceId));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_channel_channelSelector_removeConfig(csw_instanceId, (xme_core_channelId_t)10, (xme_core_channelId_t)30));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_channel_channelSelector_removeConfig(csw_instanceId1, (xme_core_channelId_t)20, (xme_core_channelId_t)40));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
