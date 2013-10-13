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
 * $Id: broker.h 4945 2013-09-04 07:37:46Z ruiz $
 */

/**
 * \file
 *         Broker.
 */

/**
 * \defgroup core_broker Broker group
 * @{
 *
 * \brief The objective of the broker group is to act as a mediatior between
 *        the Data Handler and the Execution Manager.
 *
 * \details The broker receives calls from the Data Handler and stores the port
 *         information depending on the data (port) availability and the requirements
 *         of the given functions. 
 */

#ifndef XME_CORE_BROKER_BROKER_H
#define XME_CORE_BROKER_BROKER_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/defines.h"
#include <stdbool.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_core_transferDataCallback_t
 * \brief the callback to the transfer function
 */
typedef xme_status_t (*xme_core_transferDataCallback_t) 
(
    xme_core_dataManager_dataPacketId_t src, 
    xme_core_dataManager_dataPacketId_t dst
);

/**
 * \struct xme_core_broker_dataFunction_t
 * \note this structure is here to get function list. 
 * 
 * \brief information about a function associated to a data packet.
 */
typedef struct
{
    xme_core_component_t componentId; ///< the component identifier.
    xme_core_component_functionId_t functionId; ///< the function identifier.
    xme_core_component_functionVariantId_t functionVariantId; ///< the function variant.
}
xme_core_broker_dataFunction_t;

/**
 * \struct xme_core_broker_initStruct_t
 * \brief Initialization structure for the broker.
 */
typedef struct
{
    xme_core_component_t componentId; ///< the component identifier.
} 
xme_core_broker_initStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the broker component.
 *         Exactly one component of this type must be present on every node.
 * 
 * \param params Initialization parameters for component broker (TBD)
 * 
 * \retval XME_SUCCESS if the broker component has been properly initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if initialization failed.
 */ 
extern xme_status_t
xme_core_broker_init 
(
    void* params
);


/**
 * \brief  Frees all resources occupied by the broker component.
 *         Exactly one component of this type must be present on every node.
 */
extern void
xme_core_broker_fini (void);

/**
 * \brief This function registers an entry from which data packet to which data packet shall be
 *        transfered.
 *
 * \param srcDataPacketId is the source data packet, which contains the data for the transfer
 * \param dstDataPacketId is the destination data packet
 *
 * \retval XME_STATUS_SUCCESS if the data packet relationship has been successfully added.
 * \retval XME_STATUS_ALREADY_EXIST if the data packed already existed in the broker. 
 * \retval XME_STATUS_INVALID_CONFIGURATION if a component of this
 *            type has already been initialized. Exactly one component of this
 *            type must be present on every node.
 */
extern xme_status_t
xme_core_broker_addDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
);

/**
 * \brief This function removes a registry entry for transfers.
 *
 * \param srcDataPacketId is the source data packet to be removed.
 * \param dstDataPacketId is the destination data packet to be removed. 
 *
 * \retval XME_STATUS_SUCCESS if the data packet entry has been successfully removed.
 * \retval XME_STATUS_NOT_FOUND if the data packet entry to remove cannot be found
 *         in the broker.
 * \retval XME_STATUS_INVALID_CONFIGURATION if this component cannot remove
 *         data packet entry due to an internal configuration error.
 */
extern xme_status_t
xme_core_broker_removeDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
);

/**
 * \brief This function establishes the callback for transfer function. This function
 *        is called dynamically from the broker to perform the effective transfer
 *        between two dataPackets. 
 *
 * \param transferCallback the callback to transfer function.
 *
 * \retval XME_STATUS_SUCCESS if the transfer callback is correctly established. 
 * \retval XME_STATUS_UNEXPECTED if the transfer callback cannot be asigned in the broker. 
 */
xme_status_t
xme_core_broker_setTransferCallback
(
    xme_core_transferDataCallback_t transferCallback
);


XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_BROKER_BROKER_H
