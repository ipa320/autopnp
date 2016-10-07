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
 * $Id: brokerDataManagerInterface.h 5839 2013-11-18 16:51:54Z wiesmueller $
 */

/**
 * \file
 *         Broker interface for Data Manager.
 */

/**
 * \addtogroup core_broker
 * @{
 *
 */

#ifndef XME_CORE_BROKER_DATAMANAGERINTERFACE_H
#define XME_CORE_BROKER_DATAMANAGERINTERFACE_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//

#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/defines.h"
#include "xme/core/broker/include/broker.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Signaling from Data Manager to notify that a data availability
 *        of the given data packet has been modified.
 *
 * \details This function will update data availability for data packet
 *          identified as parameter. 
 * 
 * \param dataPacketId the data packet identifier.
 * \param size the absolute value of data availability in the data manager.
 *
 * \retval XME_STATUS_SUCCESS if the data packet has been successfully
 *         updated.
 *\ retval XME_STATUS_NOT_FOUND when there is no data transfer entry for the given packet ID.
 * \retval XME_STATUS_NO_SUCH_VALUE if the data packet is not registered in the broker.
 * \retval XME_STATUS_INVALID_PARAMETER if provided parameters are not valid parameters. 
 * \retval XME_STATUS_INVALID_CONFIGURATION if the broker has not been initialized.
 * \retval XME_STATUS_UNEXPECTED if cannot update data availability in the broker. 
 */
xme_status_t
xme_core_broker_dataAvailabilityChange
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint8_t size
);

/**
 * \brief Determines if a function is ready for execution. 
 * \details A function is ready for execution when all mandatory 
 *          input data packets are ready. If the function has additional
 *          optional data packets, the function will remain ready even
 *          if these optional data packets are not ready. 
 *
 * \param componentId the component identifier
 * \param functionId the function identifier
 * \param functionVariantId the function variant identifier
 *
 * \retval XME_STATUS_SUCCESS if the function is ready for execution.
 * \retval XME_STATUS_NOT_FOUND if the function is not ready for execution.
 * \retval XME_STATUS_INVALID_PARAMETER if provided parameters are not valid parameters. 
 * \retval XME_STATUS_UNEXPECTED if cannot check function availability. 
 */
xme_status_t
xme_core_broker_isFunctionReady
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_BROKER_DATAMANAGERINTERFACE_H
