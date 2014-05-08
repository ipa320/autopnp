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
 * $Id: smokeTestManifestRepository.cpp 4498 2013-08-01 14:43:19Z geisinger $
 */

/**
 * \file
 *         Manifest Repository tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/manifestRepository/include/manifestRepository.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class ManifestRepositorySmokeTest: public ::testing::Test
{
protected:
    ManifestRepositorySmokeTest()
    {
    }

    virtual ~ManifestRepositorySmokeTest()
    {
    }
};

TEST_F(ManifestRepositorySmokeTest, init)
{
    ASSERT_EQ(xme_core_manifestRepository_init(), XME_STATUS_SUCCESS);
}

TEST_F(ManifestRepositorySmokeTest, initAndFini)
{
    ASSERT_EQ(xme_core_manifestRepository_init(), XME_STATUS_SUCCESS);
    xme_core_manifestRepository_fini();
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
