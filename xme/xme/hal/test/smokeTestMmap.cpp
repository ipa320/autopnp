/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: smokeTestMmap.cpp 6626 2014-02-05 14:36:43Z geisinger $
 */

/**
 * \file
 *         Memory mapping smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/mmap.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class MmapSmokeTest: public ::testing::Test
{
protected:
    MmapSmokeTest()
    {
    }

    virtual ~MmapSmokeTest()
    {
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     MmapSmokeTest                                                          //
//----------------------------------------------------------------------------//

TEST_F(MmapSmokeTest, todo)
{
    // TODO!
    EXPECT_EQ(1, 1);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
