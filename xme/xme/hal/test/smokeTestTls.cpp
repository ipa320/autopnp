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
 * $Id: smokeTestTls.cpp 3464 2013-05-23 12:21:49Z geisinger $
 */

/**
 * \file
 *         Thread-local storage (TLS) smoke tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/tls.h"

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class TlsSmokeTest: public ::testing::Test
{
protected:
    TlsSmokeTest()
    {
    }

    virtual ~TlsSmokeTest()
    {
        xme_hal_tls_fini();
    }
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     TlsSmokeTest                                                           //
//----------------------------------------------------------------------------//

TEST_F(TlsSmokeTest, init)
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_tls_init());
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
