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
 * $Id: executionManagerConfigurator.h 5157 2013-09-24 16:29:43Z rupanov $
 */

/**
 * \file
 *         Execution Manager configurator.
 */
#ifndef XME_CORE_EXEC_CONFIGURATOR_H
#define XME_CORE_EXEC_CONFIGURATOR_H

/**
 * \ingroup core_em Execution Manager
 * @{
 *
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include <stdint.h>

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//
typedef enum
{
    XME_CORE_EXEC_TRANSACTION_ID_INVALID = 0xffffffff
} xme_core_exec_transactionId_t;

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN
/*-------------------------------------------------------------------------*/
/** \brief Add a component to a schedule set in a transactional manner
 *
 * \param   transactionId   ID of transaction
 * \param   componentId     component ID
 * \param   functionId      function ID
 * \param   functionArgs    function argument set pointer
 * \param   afterTime_ns    beginning of time interval where insertion is allowed
 * \param   beforeTime_ns   end of interval where insertion is allowed, set to 0 for end of cycle
 * \param   schedules       a linked list of schedule handles to be modified. NULL for just the active schedule
 *
 * \note When using with schedules = NULL, please ensure to execute activateSchedule() in advance
 *
 * \return  XME_STATUS_SUCCESS in case of success,
 *          XME_STATUS_INVALID_CONFIGURATION if failed to add function (could not fit),
 *          different error code if some error has occured
 */
extern xme_status_t
xme_core_exec_configurator_addComponentToSchedule
(
        xme_core_exec_transactionId_t transactionId,
        xme_core_component_t componentId,
        xme_core_component_functionId_t functionId,
        void* functionArgs,
        xme_hal_time_timeInterval_t afterTime_ns,
        xme_hal_time_timeInterval_t beforeTime_ns,
        uint32_t periodDivider,
    	uint32_t periodDividerOffset,
        xme_hal_linkedList_descriptor_t* schedules
);

/*-------------------------------------------------------------------------*/
/** \brief Commit all changes within a transaction
 *
 * \param   transactionId   ID of transaction
 *
 *
 * \return  XME_STATUS_SUCCESS in case of success,
 *          XME_STATUS_UNEXPECTED if operating on an uninitialized component,
 *          XME_STATUS_INVALID_PARAMETER if transaction ID is not the active transaction identifier,
 *          XME_STATUS_INTERNAL_ERROR if some error has occured while applying changes
 */
extern xme_status_t
xme_core_exec_configurator_commit
(
    xme_core_exec_transactionId_t transactionId
);

/*-------------------------------------------------------------------------*/
/** \brief Roll back all changes within a transaction / ignore configurator changes since last commit
 *
 * \param   transactionId   ID of transaction
 *
 * \return  XME_STATUS_SUCCESS in case of success,
 *          XME_STATUS_UNEXPECTED if operating on an uninitialized component,
 *          XME_STATUS_INVALID_PARAMETER if transaction ID is not the active transaction identifier
 */
extern xme_status_t
xme_core_exec_configurator_rollback
(
    xme_core_exec_transactionId_t transactionId
);

/*-------------------------------------------------------------------------*/

XME_EXTERN_C_END
/**
 * @}
 */

#endif /* #ifdef XME_CORE_EXEC_CONFIGURATION_INTERFACE_H */
