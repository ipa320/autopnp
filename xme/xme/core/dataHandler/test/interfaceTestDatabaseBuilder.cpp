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
 * $Id: interfaceTestDatabaseBuilder.cpp 7594 2014-02-25 16:58:30Z ruiz $
 */

/**
 * \file
 *         Database Builder Interface test.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/dataHandler/internal/databaseBuilder.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

class DatabaseBuilderInterfaceTest : public ::testing::Test {
    protected:
        DatabaseBuilderInterfaceTest() 
        {
            builder = xme_core_dataHandler_databaseBuilder_create();
            EXPECT_TRUE(NULL != builder); 
        }

        virtual
        ~DatabaseBuilderInterfaceTest() {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_databaseBuilder_destroy(builder));
        }

    xme_core_dataHandler_databaseBuilder_t* builder;
};

/******************************************************************************/
/***   Initial test for database builder                                    ***/
/******************************************************************************/
TEST_F(DatabaseBuilderInterfaceTest, CreateGenericMemoryRegion) 
{
    xme_core_dataManager_memoryRegionID_t memoryRegionID;

    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
}

TEST_F(DatabaseBuilderInterfaceTest, CreateSharedMemoryRegion) 
{
    xme_core_dataManager_memoryRegionID_t memoryRegionID;

    builder = builder->createSharedMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
}

TEST_F(DatabaseBuilderInterfaceTest, CreatePrivateMemoryRegion) 
{
    xme_core_dataManager_memoryRegionID_t memoryRegionID;

    builder = builder->createPrivateMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetNumberOfMemoryRegionCopiesWithInvalidNumber) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setNumberOfMemoryRegionCopies(memoryRegionID, 0U);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetNumberOfMemoryRegionCopiesWithInvalidMemoryRegion) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setNumberOfMemoryRegionCopies(XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID, 1U);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetNumberOfMemoryRegionCopiesWithNonExistingMemoryRegion) 
{
    xme_core_dataManager_memoryRegionID_t memoryRegionID1, memoryRegionID2;
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    builder = builder->createGenericMemoryRegion(&memoryRegionID1);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createGenericMemoryRegion(&memoryRegionID2);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setNumberOfMemoryRegionCopies(XME_CORE_DATAMANAGER_MEMORYREGIONID_MAX, 1U);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetNumberOfMemoryRegionCopiesWithValidMemoryRegion) 
{
    xme_core_dataManager_memoryRegionID_t memoryRegionID1, memoryRegionID2;

    builder = builder->createGenericMemoryRegion(&memoryRegionID1);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createGenericMemoryRegion(&memoryRegionID2);
    ASSERT_TRUE(NULL != builder);

    builder = builder->setNumberOfMemoryRegionCopies(memoryRegionID2, 2U);
    ASSERT_TRUE(NULL != builder);
}

TEST_F(DatabaseBuilderInterfaceTest, CreateDataStoreInMemoryRegionWithInvalidSize) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->createDataStoreInMemoryRegion(0U, memoryRegionID, &dataStoreID);
    ASSERT_EQ(NULL, returnBuilder);
    returnBuilder = builder->createDataStoreInMemoryRegion(0U, XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID, &dataStoreID);
    ASSERT_EQ(NULL, returnBuilder);
    returnBuilder = builder->createDataStoreInMemoryRegion(0U, XME_CORE_DATAMANAGER_MEMORYREGIONID_MAX, &dataStoreID);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, CreateDataStoreInMemoryRegionWithInvalidMemoryRegion) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->createDataStoreInMemoryRegion(40U, XME_CORE_DATAMANAGER_MEMORYREGIONID_INVALID, &dataStoreID);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, CreateDataStoreInMemoryRegionWithNonExistingMemoryRegion) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->createDataStoreInMemoryRegion(40U, XME_CORE_DATAMANAGER_MEMORYREGIONID_MAX, &dataStoreID);
    ASSERT_TRUE(NULL != returnBuilder); // In this case, we do not have any information about the already existing memory regions in the database. 
}

TEST_F(DatabaseBuilderInterfaceTest, CreateDataStoreInMemoryRegionWithValidMemoryRegion) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->createDataStoreInMemoryRegion(40U, memoryRegionID, &dataStoreID);
    ASSERT_TRUE(NULL != returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetQueueWithInvalidSize) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setQueueSize(dataStoreID, 0U);
    ASSERT_EQ(NULL, returnBuilder);
    returnBuilder = builder->setQueueSize(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 0U);
    ASSERT_EQ(NULL, returnBuilder);
    returnBuilder = builder->setQueueSize(XME_CORE_DATAMANAGER_DATAPACKETID_MAX, 0U);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetQueueWithInvalidDataStoreID) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setQueueSize(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, 4U);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetQueueWithNonExistingDataStoreID) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setQueueSize(XME_CORE_DATAMANAGER_DATAPACKETID_MAX, 4U);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetQueueWithValidDataPacketID) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setQueueSize(dataStoreID, 4U);
    ASSERT_TRUE(NULL != returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetPersistencyWithInvalidDataStoreID) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setPersistency(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, true);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetPersistencyWithNonExistingDataStoreID) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setPersistency(XME_CORE_DATAMANAGER_DATAPACKETID_MAX, false);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, SetPersistencyWithValidDataPacketID) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->setPersistency(dataStoreID, true);
    ASSERT_TRUE(NULL != returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, CreateAttributeWithInvalidSize) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    uint32_t attributeKey = 1U;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->createAttribute(dataStoreID, attributeKey, 0U);
    ASSERT_EQ(NULL, returnBuilder);
    returnBuilder = builder->createAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey, 0U);
    ASSERT_EQ(NULL, returnBuilder);
    returnBuilder = builder->createAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_MAX, attributeKey, 0U);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, CreateAttributeWithInvalidDataPacketID) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    uint32_t attributeKey = 1U;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->createAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID, attributeKey, 4U);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, CreateAttributeWithNonExistingDataStore) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    uint32_t attributeKey = 1U;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->createAttribute(XME_CORE_DATAMANAGER_DATAPACKETID_MAX, attributeKey, 4U);
    ASSERT_EQ(NULL, returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, CreateAttributeWithValidData) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;
    uint32_t attributeKey = 1U;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID);
    ASSERT_TRUE(NULL != builder);

    returnBuilder = builder->createAttribute(dataStoreID, attributeKey, 4U);
    ASSERT_TRUE(NULL != returnBuilder);
}

TEST_F(DatabaseBuilderInterfaceTest, CompleteValidBuild) 
{
    xme_core_dataHandler_database_t* returnDatabase;
    uint32_t attributeKey1 = 1U;
    uint32_t attributeKey2 = 2U;
    xme_core_dataManager_dataStoreID_t dataStoreID;
    xme_core_dataManager_memoryRegionID_t memoryRegionID;

    // First Memory Region
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID); // Data Store 1
    ASSERT_TRUE(NULL != builder);
    builder = builder->createAttribute(dataStoreID, attributeKey1, 4U);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createAttribute(dataStoreID, attributeKey2, 4U);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID); // Data Store 2
    ASSERT_TRUE(NULL != builder);
    builder = builder->createAttribute(dataStoreID, attributeKey1, 4U);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStore(4U, &dataStoreID); // Data Store 3
    ASSERT_TRUE(NULL != builder);

    // Second Memory Region
    builder = builder->createGenericMemoryRegion(&memoryRegionID);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStoreInMemoryRegion(4U, memoryRegionID, &dataStoreID); // Data Store 4
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStoreInMemoryRegion(4U, memoryRegionID, &dataStoreID); // Data Store 5
    ASSERT_TRUE(NULL != builder);
    builder = builder->createAttribute(dataStoreID, attributeKey1, 4U);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createDataStoreInMemoryRegion(4U, memoryRegionID, &dataStoreID); // Data Store 6
    ASSERT_TRUE(NULL != builder);
    builder = builder->createAttribute(dataStoreID, attributeKey1, 4U);
    ASSERT_TRUE(NULL != builder);
    builder = builder->createAttribute(dataStoreID, attributeKey2, 4U);
    ASSERT_TRUE(NULL != builder);

    // Generate the database.
    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

