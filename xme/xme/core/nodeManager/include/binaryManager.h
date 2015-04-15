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
 * $Id: binaryManager.h 3458 2013-05-23 09:37:04Z ruiz $
 */

/**
 * \file
 *         Binary Manager.
 */

#ifndef XME_CORE_NODEMANAGER_BINARYMANAGER_H
#define XME_CORE_NODEMANAGER_BINARYMANAGER_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/coreTypes.h"
#include "xme/core/container.h"
#include "xme/core/nodeManager/include/componentManager.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \enum xme_core_nodeManager_binaryId_t
 *
 * \brief A binary ID
 */
typedef enum
{
	XME_CORE_INVALID_BINARY_ID = 0, ///< Invalid binary id.
	XME_CORE_MAX_BINARY_ID = XME_MAX_SYSTEM_VALUE ///< Largest possible binary id.
}
xme_core_nodeManager_binaryId_t;


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Checks whether the binary can be stored on the node and allocates the according memory.
 *
 * \param binarySize the binary size.
 * \param binaryId the binary id. 
 * \param cmTransactionId the transaction id. 
 * \param outMetric the output metric. 
 * \param outBinaryPresent output parameter indicating if the binary is present. 
 *
 * \return status code
 */
xme_status_t
xme_core_nodeManager_binaryManager_binaryFeasible
(
	xme_core_binarySize_t binarySize,
	xme_core_nodeManager_binaryId_t binaryId,
	xme_core_transactionId_t cmTransactionId,
	xme_core_metric_t* outMetric,
	bool* outBinaryPresent
);

/**
 * \brief Stores a binary on this node.
 *
 * \param transactionId the transaction id. 
 *
 * \return status code
 */
xme_status_t
xme_core_nodeManager_binaryManager_storeBinary
(
	xme_core_transactionId_t transactionId
);

/**
 * \brief Provides information about a binary
 *
 * \param transactionId the transaction id. 
 * \param outBinaryAddress the output parameter indicating the binary address. 
 * \param outBinarySize the output parameter indicating the binary size. 
 *
 * \return status code
 */
xme_status_t
xme_core_nodeManager_binaryManager_getBinaryInformation
(
	xme_core_transactionId_t transactionId,
	void* outBinaryAddress,
	xme_core_binarySize_t* outBinarySize
);



XME_EXTERN_C_END

#endif // #ifndef XME_CORE_NODEMANAGER_BINARYMANAGER_H
