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
 * $Id: smokeTestWCI.cpp 7483 2014-02-18 16:14:01Z wiesmueller $
 */

/**
 * \file
 *         Waypoint Configuration Infrastructure smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include "xme/com/interface.h"
#include "xme/wp/waypointConfigInfrastructure.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class WCISmokeTest: public ::testing::Test
{
    public :
    WCISmokeTest()
    {
        ipAddress = 0x201a8c0u; //192.168.1.2
        portAddress = 0xC681u; //33222
        channelID = (xme_core_channelId_t)5000u;
    }
    ~WCISmokeTest()
    {
    }
    uint32_t ipAddress;
    uint16_t portAddress;
    xme_core_channelId_t channelID;
    xme_core_topic_t topic;
};

TEST_F(WCISmokeTest, sunnyDayTests)
{
    topic = XME_CORE_TOPIC_PNPMANAGER_RUNTIME_GRAPH_MODEL;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

    topic = XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_wci_addConfig(0, portAddress, channelID, topic));

    topic = XME_CORE_TOPIC_LOGIN_LOGINRESPONSE;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));

    topic = (xme_core_topic_t) XME_CORE_TOPIC_INVALID_TOPIC;
    EXPECT_EQ(XME_STATUS_INVALID_CONFIGURATION, xme_wp_wci_addConfig(ipAddress, portAddress, channelID, topic));
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
