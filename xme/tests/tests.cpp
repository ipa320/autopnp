/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: tests.cpp 3348 2013-05-17 12:28:31Z geisinger $
 */

/**
 * \file
 *         Testsuite invoker.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/core/componentList.h"
#include "xme/core/directory.h"
#include "xme/core/nodeManager/loginClient.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define XME_COMPONENT_CONTEXT_TEST_DUMMY ((xme_core_component_t)0x0003)
#define XME_CORE_NODEMANAGER_DEVICE_TYPE ((xme_core_device_type_t)0x1234567890ABCDEFULL)
#define XME_CORE_NODEMANAGER_DEVICE_GUID ((xme_core_device_guid_t)0x0000000000001E51)

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
XME_COMPONENT_CONFIG_STRUCT
(
	xme_test_dummy,
);

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
xme_status_t
xme_test_dummy_create
(
	xme_core_componentConfigStruct_t* config
);

xme_status_t
xme_test_dummy_activate
(
	xme_core_componentConfigStruct_t* config
);

void
xme_test_dummy_deactivate
(
	xme_core_componentConfigStruct_t* config
);

void
xme_test_dummy_destroy
(
	xme_core_componentConfigStruct_t* config
);

/******************************************************************************/
/***   Component configurations                                             ***/
/******************************************************************************/
XME_COMPONENT_CONFIG_INSTANCE(xme_test_dummy, 1);

/******************************************************************************/
/***   Component descriptor                                                 ***/
/******************************************************************************/
XME_COMPONENT_LIST_BEGIN
	XME_COMPONENT_LIST_ITEM(xme_test_dummy, 0) // Dummy component for testsuite. Must be at position number XME_COMPONENT_CONTEXT_TEST_DUMMY
XME_COMPONENT_LIST_END;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_test_dummy_create
(
	xme_core_componentConfigStruct_t* config
)
{
	xme_status_t result = XME_STATUS_UNEXPECTED;
	if (NULL != config)
	{
		result = XME_STATUS_SUCCESS;
	}
	return result;
}

xme_status_t
xme_test_dummy_activate
(
	xme_core_componentConfigStruct_t* config
)
{
	xme_status_t result = XME_STATUS_UNEXPECTED;
	if (NULL != config)
	{
		result = XME_STATUS_SUCCESS;
	}
	return result;
}

void
xme_test_dummy_deactivate
(
	xme_core_componentConfigStruct_t* config
)
{
	XME_ASSERT_NORVAL(NULL != config);
}

void
xme_test_dummy_destroy
(
	xme_core_componentConfigStruct_t* config
)
{
	XME_ASSERT_NORVAL(NULL != config);
}

int main (int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
