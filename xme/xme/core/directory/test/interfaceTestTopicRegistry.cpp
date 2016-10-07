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
 * $Id: interfaceTestTopicRegistry.cpp 5588 2013-10-23 14:38:17Z wiesmueller $
 */

/**
 * \file
 *         Attribute abstraction interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/directory/include/topicRegistry.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class TopicInterfaceTest: public ::testing::Test
{
protected:
    TopicInterfaceTest()
    {
    }

    virtual ~TopicInterfaceTest()
    {
    }

    virtual void SetUp()
    {
        xme_core_directory_topicRegistry_init();
    }

    virtual void TearDown()
    {
        xme_core_directory_topicRegistry_fini();
    }

};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/
TEST_F(TopicInterfaceTest, getTopicSize)
{
    xme_core_topic_t topics[] =
    {
        (xme_core_topic_t)1,
        (xme_core_topic_t)2,
        (xme_core_topic_t)3
    };
    const xme_core_topic_t expectedSizes[] =
    {
        12,
        42,
        0
    };
    xme_core_topic_t sizes[] =
    {
        0xFFFF,
        0xFFFF,
        0xFFFF
    };

    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_topicRegistry_getTopicSize(XME_CORE_TOPIC_INVALID_TOPIC, &sizes[0])
    );

    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_topicRegistry_getTopicSize(topics[0], NULL)
    );

    // Invalid parameters
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_topicRegistry_getTopicSize(XME_CORE_TOPIC_INVALID_TOPIC, NULL)
    );

    // Valid topic, but not registered yet
    EXPECT_EQ
    (
        XME_STATUS_NOT_FOUND,
        xme_core_directory_topicRegistry_getTopicSize(topics[0], &sizes[0])
    );

    // Valid topic, but not registered yet
    EXPECT_EQ
    (
        XME_STATUS_NOT_FOUND,
        xme_core_directory_topicRegistry_getTopicSize(topics[1], &sizes[1])
    );

    // Valid topic, but not registered yet
    EXPECT_EQ
    (
        XME_STATUS_NOT_FOUND,
        xme_core_directory_topicRegistry_getTopicSize(topics[2], &sizes[2])
    );

    // Register some topics
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_registerTopicSize(topics[1], expectedSizes[1], false)
    );
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_registerTopicSize(topics[2], expectedSizes[2], false)
    );

    // topics[0] is still not registered
    EXPECT_EQ
    (
        XME_STATUS_NOT_FOUND,
        xme_core_directory_topicRegistry_getTopicSize(topics[0], &sizes[0])
    );

    // topics[1] is registered
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_getTopicSize(topics[1], &sizes[1])
    );

    // Check if returned size matches the expeced size
    EXPECT_EQ(expectedSizes[1], sizes[1]);

    // topics[2] is registered
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_getTopicSize(topics[2], &sizes[2])
    );

    // Check if returned size matches the expeced size
    EXPECT_EQ(expectedSizes[2], sizes[2]);
}

TEST_F(TopicInterfaceTest, registerTopicSize)
{
    xme_core_topic_t topics[] =
    {
        (xme_core_topic_t)1,
        (xme_core_topic_t)2,
        (xme_core_topic_t)3
    };
    uint16_t size = 42;

    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_topicRegistry_registerTopicSize(XME_CORE_TOPIC_INVALID_TOPIC, size, false)
    );

    // Invalid parameter
    EXPECT_EQ
    (
        XME_STATUS_INVALID_PARAMETER,
        xme_core_directory_topicRegistry_registerTopicSize(XME_CORE_TOPIC_INVALID_TOPIC, size, true)
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_registerTopicSize(topics[0], size, false)
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_registerTopicSize(topics[1], size, false)
    );

    // Trying to registering second attribute descriptor list, when overwrite is false
    EXPECT_EQ
    (
        XME_STATUS_ALREADY_EXIST,
        xme_core_directory_topicRegistry_registerTopicSize(topics[0], size, false)
    );

    // Trying to registering second attribute descriptor list, when overwrite is true
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_registerTopicSize(topics[0], size, true)
    );

    // Trying to registering second attribute descriptor list, when overwrite is false
    EXPECT_EQ
    (
        XME_STATUS_ALREADY_EXIST,
        xme_core_directory_topicRegistry_registerTopicSize(topics[1], size, false)
    );

    // Trying to registering second attribute descriptor list, when overwrite is true
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_registerTopicSize(topics[1], size, true)
    );

    // Registering list with overwrite set to true and no pre-existing entry
    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_registerTopicSize(topics[2], size, true)
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_registerTopicSize(topics[2], size, true)
    );

    EXPECT_EQ
    (
        XME_STATUS_SUCCESS,
        xme_core_directory_topicRegistry_registerTopicSize(topics[2], size, true)
    );
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
