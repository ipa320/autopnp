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
 * $Id: binaryManager.c 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         Binary Manager.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/nodeManager/include/binaryManager.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_nodeManager_binaryManager_binaryFeasible
(
	xme_core_binarySize_t binarySize,
	xme_core_nodeManager_binaryId_t binaryId,
	xme_core_transactionId_t cmTransactionId,
	xme_core_metric_t* outMetric,
	bool* outBinaryPresent
)
{
	XME_UNUSED_PARAMETER(binarySize);
	XME_UNUSED_PARAMETER(binaryId);
	XME_UNUSED_PARAMETER(cmTransactionId);

	//TODO: implement functionality, see issue #2030

	*outMetric = (xme_core_metric_t) 100;
	*outBinaryPresent = true;

	return XME_STATUS_SUCCESS;
}


xme_status_t
xme_core_nodeManager_binaryManager_storeBinary
(
	xme_core_transactionId_t transactionId
)
{
	XME_UNUSED_PARAMETER(transactionId);
	//TODO: implement functionality, see issue #2030
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_binaryManager_getBinaryInformation
(
	xme_core_transactionId_t transactionId,
	void* outBinaryAddress,
	xme_core_binarySize_t* outBinarySize
)
{
	XME_UNUSED_PARAMETER(transactionId);
	XME_UNUSED_PARAMETER(outBinaryAddress);
	//TODO: implement functionality, see issue #2030

	/* * */outBinaryAddress = 0;
	*outBinarySize = 666;
	return XME_STATUS_SUCCESS;
}
