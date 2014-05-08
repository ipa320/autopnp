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
 * $Id: smokeTestDatabase.cpp 7684 2014-03-05 15:00:06Z ruiz $
 */

/**
 * \file
 *         Database test.
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/dataHandler/internal/database.h"
#include "xme/core/dataHandler/internal/databaseConfigurator.h"
#include "xme/hal/include/mem.h"
#include "xme/core/testUtils.h"
#include <stdbool.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

class DatabaseVoidSmokeTest : public ::testing::Test
{
protected:
    DatabaseVoidSmokeTest()
    {
        // Do nothing. This class is for testing init, fini, create, destroy. 
    }
    
    virtual
    ~DatabaseVoidSmokeTest() 
    {
        // do nothing. 
    }
    xme_core_dataHandler_database_t* database;
};

/******************************************************************************/
/***   Initial test for database                                            ***/
/******************************************************************************/
TEST_F(DatabaseVoidSmokeTest, InitializeDatabase)
{
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_database_init());
}

TEST_F(DatabaseVoidSmokeTest, FinalizeANonInitializedDatabase)
{
    xme_core_dataHandler_database_fini();
}

TEST_F(DatabaseVoidSmokeTest, CreateAndDestroyDatabase)
{
    database = xme_core_dataHandler_databaseConfigurator_create();
    ASSERT_EQ(XME_STATUS_SUCCESS, xme_core_dataHandler_databaseConfigurator_destroy());
}

TEST_F(DatabaseVoidSmokeTest, ConfigureDatabaseWithOutInit)
{
#ifdef DEBUG
    ASSERT_XME_ASSERTION_FAILURE(xme_core_dataHandler_databaseConfigurator_configure());
#endif
}

int
main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

