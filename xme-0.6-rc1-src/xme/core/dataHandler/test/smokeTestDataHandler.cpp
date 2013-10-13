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
 * $Id: smokeTestDataHandler.cpp 5123 2013-09-19 14:30:10Z camek $
 */

/**
 * \file
 *         Data Handler test.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include <cstring>

#include "xme/core/component.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define BUFFER ((int) 16)
#define DATAHANDLER_TEST_MEMORYVALUE 10

/******************************************************************************/
/***   Static variables                                                     ***/
/******************************************************************************/

xme_core_dataManager_dataPacketId_t xme_core_exec_CycleCounter;

class DataHandlerSmokeTest : public ::testing::Test {
    protected:
        DataHandlerSmokeTest() :
                        component(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT), type(
                                        XME_CORE_COMPONENT_PORTTYPE_INVALID), topic(
                                        XME_CORE_TOPIC_INVALID_TOPIC), inport(
                                        XME_CORE_DATAMANAGER_DATAPACKETID_INVALID), outport(
                                        XME_CORE_DATAMANAGER_DATAPACKETID_INVALID), key(
                                        XME_CORE_ATTRIBUTE_KEY_UNDEFINED), got(0) {

        }

        virtual
        ~DataHandlerSmokeTest() {
        }

        xme_core_component_t component;
        xme_core_component_portType_t type;
        xme_core_topic_t topic;
        xme_core_dataManager_dataPacketId_t inport;
        xme_core_dataManager_dataPacketId_t outport;
        xme_core_attribute_key_t key;
        unsigned int got;
        char buffer[BUFFER ];
};

typedef DataHandlerSmokeTest DataHandlerSmokeTestDeathTest;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
TEST_F(DataHandlerSmokeTest, shutdownWithoutInit) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(xme_core_dataHandler_fini(), "");
#else
    ASSERT_DEATH_IF_SUPPORTED(xme_core_dataHandler_fini(), "");
#endif
}

TEST_F(DataHandlerSmokeTestDeathTest, startupAndShutdownWithZeroMemory) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(xme_core_dataHandler_init(0u), "");
    ASSERT_DEBUG_DEATH(xme_core_dataHandler_fini(), "");
#else
    ASSERT_DEATH_IF_SUPPORTED(xme_core_dataHandler_init(0u), "");
    ASSERT_DEATH_IF_SUPPORTED(xme_core_dataHandler_fini(), "");
#endif
}

TEST_F(DataHandlerSmokeTest, startupAndShutdownWithSomeMemory) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE));
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_fini());
}

TEST_F(DataHandlerSmokeTestDeathTest, startupTwiceAndShutdownWithSomeMemory) {
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE));
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE), "");
#else
    ASSERT_DEATH_IF_SUPPORTED(xme_core_dataHandler_init(DATAHANDLER_TEST_MEMORYVALUE), "");
#endif
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_fini());
}

// FIXME: test missing for initalizing system with less memory than needed!

TEST_F(DataHandlerSmokeTest, createPortUnitialized) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER,
              xme_core_dataHandler_createPort(component, type, topic, BUFFER * sizeof(char), XME_CORE_NO_ATTRIBUTE, 0, false, false, 0, &inport));
}

TEST_F(DataHandlerSmokeTestDeathTest, readingUnitialized) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readData(inport, buffer, BUFFER * sizeof(char), &got), "");
}

TEST_F(DataHandlerSmokeTestDeathTest, writingUnitialized) {
    ASSERT_DEATH_IF_SUPPORTED(xme_core_dataHandler_writeData(inport, buffer, BUFFER * sizeof(char)),
                              "");
}

TEST_F(DataHandlerSmokeTestDeathTest, transferingUnitialized) {
#ifdef NDEBUG
    ASSERT_DEBUG_DEATH(xme_core_dataHandler_transferData(inport, outport), "");
#else
    ASSERT_DEATH_IF_SUPPORTED(xme_core_dataHandler_transferData(inport, outport), "");
#endif
}

TEST_F(DataHandlerSmokeTestDeathTest, readAttributeUnitialized) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_readAttribute(inport, key, buffer, BUFFER * sizeof(char), &got),
                    "");
}

TEST_F(DataHandlerSmokeTestDeathTest, writeAttributeUnitialized) {
    ASSERT_DEATH_IF_SUPPORTED(
                    xme_core_dataHandler_writeAttribute(inport, key, buffer, BUFFER * sizeof(char)),
                    "");
}

TEST_F(DataHandlerSmokeTest, completeReadOperationPortUnitialized) {
    ASSERT_EQ(XME_STATUS_INVALID_PARAMETER, xme_core_dataHandler_completeReadOperation(inport));
}

TEST_F(DataHandlerSmokeTest, completeWriteOperationPortUnitialized) {
    ASSERT_EQ(XME_STATUS_INTERNAL_ERROR, xme_core_dataHandler_completeWriteOperation(inport));
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
