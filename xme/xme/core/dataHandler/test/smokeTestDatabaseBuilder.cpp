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
 * $Id: smokeTestDatabaseBuilder.cpp 7684 2014-03-05 15:00:06Z ruiz $
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

#include "xme/core/dataHandler/internal/databaseBuilder.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

class DatabaseBuilderInitSmokeTest : public ::testing::Test {
    protected:
        DatabaseBuilderInitSmokeTest() 
        {
            // do nothing. 
        }

        virtual
        ~DatabaseBuilderInitSmokeTest() {
            // do nothing. 
        }

    xme_core_dataHandler_databaseBuilder_t* builder;
};


class DatabaseBuilderSmokeTest : public ::testing::Test {
    protected:
        DatabaseBuilderSmokeTest() :
             attributeKey1(1U),
             attributeKey2(2U)
        {
            builder = xme_core_dataHandler_databaseBuilder_create();
            EXPECT_TRUE(NULL != builder); 
        }

        virtual
        ~DatabaseBuilderSmokeTest() {
            EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_databaseBuilder_destroy(builder));
            // Clean up the database. 
            xme_core_dataHandler_databaseInternal_fini();
        }

    xme_core_dataHandler_databaseBuilder_t* builder;

    xme_core_dataManager_dataStoreID_t outDataStoreID1;
    xme_core_dataManager_dataStoreID_t outDataStoreID2;
    xme_core_dataManager_dataStoreID_t outDataStoreID3;

    uint32_t attributeKey1;
    uint32_t attributeKey2;

    xme_core_dataManager_memoryRegionID_t outMemoryRegionID1;
    xme_core_dataManager_memoryRegionID_t outMemoryRegionID2;

    xme_core_dataHandler_database_t* returnDatabase;
};

/******************************************************************************/
/***   Initial test for database builder                                    ***/
/******************************************************************************/
TEST_F(DatabaseBuilderInitSmokeTest, CreateAndDestroyBuilder) 
{
    builder = xme_core_dataHandler_databaseBuilder_create();
    ASSERT_TRUE(NULL != builder);
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_databaseBuilder_destroy(builder));
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildDatabaseWithZeroSizedMemoryGenericRegionWithoutDataStores) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildDatabaseWithMemoryGenericRegionWithSizeAndWithoutDataStores) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setMemoryRegionSize(outMemoryRegionID1, 1024U);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildDatabaseWithMemoryGenericRegionWithSizeAndCopiesAndWithoutDataStores) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setMemoryRegionSize(outMemoryRegionID1, 1024U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setNumberOfMemoryRegionCopies(outMemoryRegionID1, 2U);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoresWithoutMemoryRegions) 
{
    // NOTE: We cannot declare data stores without having at least one memory region. 
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL == returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoresAndWithOneMemoryRegionWithoutSizeSpecification) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoresAndWithOneMemoryRegionWithSizeSpecification) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setMemoryRegionSize(outMemoryRegionID1, 1024U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoresAndWithOneMemoryRegionWithSizeSpecificationAndCopies) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setMemoryRegionSize(outMemoryRegionID1, 1024U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setNumberOfMemoryRegionCopies(outMemoryRegionID1, 2U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoresAndWithOnePreconfiguredMemoryRegionWithoutSizeSpecification) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    // Configuration of the memory region. 
    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    // Note: If we do configure the database with a memory region of zero size and without data packets,
    // we will have an out of resources error from database. That is why we ASSERT that database is NULL-valued. 
    returnDatabase = builder->build();
    ASSERT_TRUE(NULL == returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoresAndWithOnePreconfiguredMemoryRegionWithSizeSpecification) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setMemoryRegionSize(outMemoryRegionID1, 1024U);
    ASSERT_EQ(builder, returnBuilder);

    // Configuration of the memory region. 
    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoresAndWithOnePreconfiguredMemoryRegionWithSizeSpecificationAndCopies) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setMemoryRegionSize(outMemoryRegionID1, 1024U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setNumberOfMemoryRegionCopies(outMemoryRegionID1, 2U);
    ASSERT_EQ(builder, returnBuilder);

    // Configuration of the memory region. 
    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoreAndPersistencyToTrue) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setPersistency(outDataStoreID1, true);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoreAndPersistencyToFalse) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setPersistency(outDataStoreID1, true);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoreAndQueueSizeToOne) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setQueueSize(outDataStoreID1, 1U);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoreAndQueueSizeToMoreThanOne) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->setQueueSize(outDataStoreID1, 5U);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithOneDataStoreWithAttributes) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID1, attributeKey1, 32U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID1, attributeKey2, 16U);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}


TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildWithSeveralDataStoresWithAttributes) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID1, attributeKey1, 32U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID1, attributeKey2, 16U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(128U, &outDataStoreID2);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID2, attributeKey1, 16U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(64U, &outDataStoreID3);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildTwoDifferentMemoryRegionsInTheSameBuild) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID2);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildTwoDifferentMemoryRegionsInDifferentBuild) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID2);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildTwoDifferentMemoryRegionsAndDataPacketsWithoutSpecifyingMemoryRegion) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID2);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(256U, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID1, attributeKey1, 32U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID1, attributeKey2, 16U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(128U, &outDataStoreID2);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID2, attributeKey1, 16U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStore(64U, &outDataStoreID3);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

TEST_F(DatabaseBuilderSmokeTest, CreateAndBuildTwoDifferentMemoryRegionsAndDataPacketsSpecifyingConcreteMemoryRegion) 
{
    xme_core_dataHandler_databaseBuilder_t* returnBuilder;

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createGenericMemoryRegion(&outMemoryRegionID2);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStoreInMemoryRegion(256U, outMemoryRegionID2, &outDataStoreID1);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID1, attributeKey1, 32U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID1, attributeKey2, 16U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStoreInMemoryRegion(128U, outMemoryRegionID1, &outDataStoreID2);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createAttribute(outDataStoreID2, attributeKey1, 16U);
    ASSERT_EQ(builder, returnBuilder);

    returnBuilder = builder->createDataStoreInMemoryRegion(64U, outMemoryRegionID2, &outDataStoreID3);
    ASSERT_EQ(builder, returnBuilder);

    returnDatabase = builder->build();
    ASSERT_TRUE(NULL != returnDatabase);
}

#if 0
TEST_F(DatabaseBuilderSmokeTest, initialTest) 
{
    bool isNullValued;

    // Create the database builder
    builder = xme_core_dataHandler_databaseBuilder_create();

    // Create one data packet with two associated attributes. 
    builder = builder->createDataPacket(24U, &outDataPacket1);
    isNullValued = (builder == NULL);
    ASSERT_FALSE(isNullValued);
    builder = builder->createAttribute(outDataPacket1, 1U, 4U);
    isNullValued = (builder == NULL);
    ASSERT_FALSE(isNullValued);
    builder = builder->createAttribute(outDataPacket1, 2U, 16U);
    isNullValued = (builder == NULL);
    ASSERT_FALSE(isNullValued);

    // Create a second data packet with three associated attributes. 
    builder = builder->createDataPacket(128U, &outDataPacket2);
    isNullValued = (builder == NULL);
    ASSERT_FALSE(isNullValued);
    builder = builder->createAttribute(outDataPacket2, 1U, 10U);
    isNullValued = (builder == NULL);
    ASSERT_FALSE(isNullValued);
    builder = builder->createAttribute(outDataPacket2, 2U, 8U);
    isNullValued = (builder == NULL);
    ASSERT_FALSE(isNullValued);
    builder = builder->createAttribute(outDataPacket2, 3U, 16U);
    isNullValued = (builder == NULL);
    ASSERT_FALSE(isNullValued);

    // Create a third data packet with +out attributes. 
    builder = builder->createDataPacket(128U, &outDataPacket3);
    isNullValued = (builder == NULL);
    ASSERT_FALSE(isNullValued);

    // Build the database. 
    returnDatabase = builder->build();
    isNullValued = (returnDatabase == NULL);
    ASSERT_FALSE(isNullValued);
}
#endif

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

