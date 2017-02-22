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
 * $Id: brokerInternalMethods.h 6513 2014-01-27 22:20:31Z ruiz $
 */

/**
 * \file
 *         Broker internal methods interface.
 */

/**
 * \addtogroup core_broker
 * @{
 */

#ifndef XME_CORE_BROKER_BROKERINTERNALMETHODS_H
#define XME_CORE_BROKER_BROKERINTERNALMETHODS_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//

#include "xme/core/dataManagerTypes.h"
#include "xme/core/broker/include/brokerInternalTypes.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Given an internal pointer to a function entry from function table
 *        adds a the information associated to a given data packet.
 * \details The data packet id and the function item should be valid values.
 *         The function checks if the provided data packet already exists in
 *         the data packet list. If so, the return status indicate it.
 *
 * \param dataPacketId The data packet identifier to register.
 * \param functionItem The pointer to the function entry from functionDescriptions table.
 * \param mandatory a boolean value indicating if the data packet is mandatory.
 *
 * \retval XME_STATUS_SUCCESS if the data packet is successfully added to the function entry.
 * \retval XME_STATUS_ALREADY_EXIST if the data packet already exist in the function entry.
 * \retval XME_STATUS_INVALID_PARAMETER if the parameters provide invalid values
 *         for corresponding data fields.
 * \retval XME_STATUS_UNEXPECTED if the data packet cannot be added to the function entry.
 */
xme_status_t
xme_core_broker_addDataPacketToFunctionItem
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_broker_functionDescriptionItem_t* functionItem,
    bool mandatory
);

/**
 * \brief   Updates the readiness of the data packet. 
 * \details This function will update the readiness flag for data packet
 *          identified as parameter. If size is zero, then the data packet
 *          is labeled as non-ready. If size is more than zero, the
 *          data packet is labeled as ready.
 * 
 * \param dataPacketId the data packet identifier.
 * \param size the absolute value of data availability in the data manager.
 *
 * \retval XME_STATUS_SUCCESS if the data packet has been successfully
 *            updated.
 * \retval XME_STATUS_NO_SUCH_VALUE if the data packet is no registered in the broker.
 * \retval XME_STATUS_INVALID_PARAMETER if provided parameters are not valid parameters. 
 * \retval XME_STATUS_INVALID_CONFIGURATION if the broker has not been initialized.
 * \retval XME_STATUS_UNEXPECTED if cannot update data availability in the broker. 
 */
xme_status_t
xme_core_broker_updateDataReadiness
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint8_t size
);

/**
 * \brief This function checks if a data packet is an output data packet.
 *
 * \details If and only if data packet is included in the transfer list as source
 *          data packet, this means that the data packet is an output data packet.
 *
 * \param dataPacketId the data packet identifier
 *
 * \retval true if the data packet is an output data packet.
 * \retval false if the data packet is not an output data packet.
 */
bool
xme_core_broker_isOutputDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
);

/**
 * \brief This function checks if a data packet is an input data packet.
 * \details If data packet is included in the transfer list as destination
 *          data packet, this means that the data packet is an input data packet. 
 *
 * \param dataPacketId the data packet identifier
 *
 * \retval true if the data packet is an input data packet.
 * \retval false if the data packet is not an input data packet.
 */
bool
xme_core_broker_isInputDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
);

/**
 * \brief Determines if a source data packed is locked.
 * \details A source data packet is locked during the complete write operation. 
 *          The lock is used as a signal to discard deactivation of data availability
 *          during the complete read operation, who send a signal of dataAvailability
 *          change reducing the quantity of data availability.
 *
 * \param dataPacketId the data packet identifier
 *
 * \retval true if the data packet is locked.
 * \retval false if the data packet is not locked.
 */
bool 
xme_core_broker_isSourceDataPacketLocked
(
    xme_core_dataManager_dataPacketId_t dataPacketId
);

/**
 * \brief Unlocks a source data packed.
 * \details A source data packet is locked during the complete write operation. 
 *          The lock is used as a signal to discard deactivation of data availability
 *          during the complete read operation, who send a signal of dataAvailability
 *          change reducing the quantity of data availability.
 *
 * \param dataPacketId the data packet identifier
 *
 * \retval XME_STATUS_SUCCESS if the data packet is successfully unlocked.
 * \retval XME_STATUS_INTERNAL_ERROR if the data packet cannot be unlocked.
 */
xme_status_t 
xme_core_broker_unlockSourceDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
);


XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_BROKER_BROKERINTERNALMETHODS_H
