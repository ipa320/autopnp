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
 * $Id: brokerPnpManagerInterface.h 7483 2014-02-18 16:14:01Z wiesmueller $
 */

/**
 * \file
 *         Broker interface for Plug and Play Manager.
 */

/**
 * \addtogroup core_broker
 * @{
 *
 */

#ifndef XME_CORE_BROKER_PNPMANAGERINTERFACE_H
#define XME_CORE_BROKER_PNPMANAGERINTERFACE_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/component.h"

#include "xme/defines.h"
#include "xme/hal/include/linkedList.h"
#include "xme/core/dataManagerTypes.h"

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Registers a function in the broker.
 *
 * \details If the function provided as parameter does not exist in the broker,
 *          the function is registered. Null values are assumed as error in parameters
 *          and no operations are performed in the current status of the broker. 
 *
 * \param componentId the component identifier
 * \param functionId The function identifier
 * \param functionVariantId the function variant
 *
 * \retval XME_STATUS_SUCCESS if the function is successfully registered.
 * \retval XME_STATUS_ALREADY_EXIST if the function is already registered 
 *         in the broker.
 * \retval XME_STATUS_INVALID_PARAMETER if the parameters provide an invalid value
 *         (e.g. null-valued). 
 * \retval XME_STATUS_UNEXPECTED if the function cannot be registered in the broker. 
 */
xme_status_t
xme_core_broker_registerFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
);

/**
 * \brief Associates a data packet to a function.
 *
 * \details The data packet id and the function should be valid values. 
 *          If the data packet or the function is not previously registered,
 *          the registration takes place. 
 *
 * \param dataPacketId The data packet identifier to register.
 * \param componentId The component identifier.
 * \param functionId The function identifier.
 * \param functionVariantId The function variant identifier.
 * \param mandatory declares the data packet as mandatory if true. 
 *
 * \retval XME_STATUS_SUCCESS if the data packet is successfully registered.
 * \retval XME_STATUS_ALREADY_EXIST if the data packet is already registered.
 * \retval XME_STATUS_INVALID_PARAMETER if the parameters provide invalid values
 *         for corresponding data fields. 
 * \retval XME_STATUS_UNEXPECTED if the data packet cannot be registered in the broker. 
 */
xme_status_t
xme_core_broker_addDataPacketToFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    bool mandatory
);

/**
 * \brief Removes the association of a data packet to a given function.
 *
 * \details The data packet id and function should be valid values.
 *          If the data packet or the function were not previously registered,
 *          the return status value will indicate it.
 *
 * \param dataPacketId The data packet identifier to deregister.
 * \param componentId The component identifier.
 * \param functionId The function identifier.
 * \param functionVariantId The function variant identifier.
 *
 * \retval XME_STATUS_SUCCESS if the data packet is successfully unlinked from the function.
 * \retval XME_STATUS_NO_SUCH_VALUE if the data packet is not associated to the function.
 * \retval XME_STATUS_INVALID_PARAMETER if the data packet or function are
 *         not valid values for corresponding data fields. 
 * \retval XME_STATUS_UNEXPECTED if the data packet cannot be registered in the broker. 
 */
xme_status_t
xme_core_broker_removeDataPacketFromFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
);

/**
 * \brief Removes a data packet from the broker. 
 * \details The given data packet is removed from the broker. 
 *          The data packet is deregistered from all associated functions. 
 *
 * \param dataPacketId The data packet identifier to deregister
 *
 * \retval XME_STATUS_SUCCESS if the data packet is successfully deregistered.
 * \retval XME_STATUS_NO_SUCH_VALUE if the data packet does not exist in the broker.
 * \retval XME_STATUS_INVALID_PARAMETER if the data packet value is not a valid data packet identifier. 
 * \retval XME_STATUS_UNEXPECTED if the data packet cannot be deregistered from the broker. 
 */
xme_status_t
xme_core_broker_removeDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
);

/**
 * \brief Removes a function variant from the broker. 
 * \details The given function is removed from the broker. 
 *          The function is also removed from associated data packets. 
 *          If the data packets was only associated to that given function,
 *          the data packet data is removed. 
 *
 * \param componentId The component identifier.
 * \param functionId The function identifier.
 * \param functionVariantId The function variant identifier.
 *
 * \retval XME_STATUS_SUCCESS if the function variant is successfully deregistered.
 * \retval XME_STATUS_NO_SUCH_VALUE if the function variant is not registered in the broker.
 * \retval XME_STATUS_INVALID_PARAMETER if the function variant is not a valid function. 
 * \retval XME_STATUS_UNEXPECTED if the function cannot be deregistered from the broker. 
 */
xme_status_t
xme_core_broker_removeFunctionVariant
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
);

/**
 * \brief Removes a function from the broker. 
 * \details The given function is removed from the broker. 
 *          The function is also removed from associated data packets. 
 *          If the data packets was only associated to that given function,
 *          the data packet data is removed. All function variants are removed
 *          as well
 *
 * \param componentId The component identifier.
 * \param functionId The function identifier.
 *
 * \retval XME_STATUS_SUCCESS if the function variant is successfully deregistered.
 * \retval XME_STATUS_NO_SUCH_VALUE if the function variant is not registered in the broker.
 * \retval XME_STATUS_INVALID_PARAMETER if the function variant is not a valid function. 
 * \retval XME_STATUS_UNEXPECTED if the function cannot be deregistered from the broker. 
 */
xme_status_t
xme_core_broker_removeFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

/**
 * \brief This function checks if a function is registered in the broker.
 * \details If exists one or more variants implementing the function, the
 *          returned value is true. 
 *
 * \param componentId The component identifier.
 * \param functionId the function identifier to check. 
 *
 * \retval true if at least one variant of the function is registered in the broker.
 * \retval false if the function is not registered in the broker.
 */
bool
xme_core_broker_isFunctionRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
);

/**
 * \brief This function checks if a function variant is registered in the broker.
 * \details If the function variant is registered in the broker, returns true. 
 *
 * \param componentId The component identifier.
 * \param functionId the function identifier to check. 
 * \param functionVariantId the function variant identifier to check. 
 *
 * \retval true if the function variant is registered in the broker.
 * \retval false if the function variant is not registered in the broker.
 */
bool
xme_core_broker_isFunctionVariantRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
);

/**
 * \brief This function obtains the list of data packets associated
 *        to a function variant. This data is obtained from registered
 *        functions in broker.
 *
 * \param componentId the component identifier
 * \param functionId the function identifier
 * \param functionVariantId the function variant identifier
 * \param dataPacketsList the output parameter containing all data packets
 *        that match the input criteria.
 * 
 * \retval XME_STATUS_SUCCESS if the data packet table is successfully populated.
 * \retval XME_STATUS_NO_SUCH_VALUE if there are no matches for function input parameters.
 * \retval XME_STATUS_INVALID_PARAMETER if provided parameters are not valid parameters. 
 * \retval XME_STATUS_UNEXPECTED if cannot get matches from the broker. 
 */
xme_status_t
xme_core_broker_getFunctionDataPackets
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    xme_hal_linkedList_descriptor_t* dataPacketsList
);

/**
 * \brief This function obtains the list of functions associated
 *        to a data packet. This data is obtained from registered
 *        data packets in broker.
 *
 * \param dataPacketId the data packet identifier
 * \param functionsList the output parameter containing all functions
 *        that match the input criteria.
 *
 * \retval XME_STATUS_SUCCESS if the function table is successfully populated.
 * \retval XME_STATUS_NO_SUCH_VALUE if there are no matches for data packet input parameter.
 * \retval XME_STATUS_INVALID_PARAMETER if provided parameters are not valid parameters. 
 * \retval XME_STATUS_UNEXPECTED if cannot get matches from the broker. 
 */
xme_status_t
xme_core_broker_getDataPacketFunctions
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_hal_linkedList_descriptor_t* functionsList
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_BROKER_PNPMANAGERINTERFACE_H
