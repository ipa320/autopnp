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
 * $Id: executionManager.h 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Execution manager.
 */

#ifndef XME_CORE_EXEC_EXECUTIONMANAGER_H
#define XME_CORE_EXEC_EXECUTIONMANAGER_H
/**
 * \defgroup core_em Execution Manager
 * @{
 *
 *
 */
//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//

#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/core/component.h"

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * TODO document
 */
xme_core_dataManager_dataPacketId_t
xme_core_exec_initDbCycleCounter(void);


/*-------------------------------------------------------------------------*/
/**
 * \brief  Initialize the whole execution manager, including 
 *            dispatcher and scheduler.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if initialization was successful.
 *          - XME_STATUS_UNEXPECTED if operation is performed on an initialized component
 *          - XME_CORE_STATUS_INTERNAL_ERROR if it is not possible to perform
 *            full initialization.
 */
extern xme_status_t
xme_core_exec_init ( xme_core_exec_configStruct_t* initConfig );

/*-------------------------------------------------------------------------*/
/**
 * \brief  Runs the scheduler loop. Requires that at least one schedule has been defined and activated.
 *
 * \param  numCycles        defines the number of full schedule cycles to execute; 0 = infinite
 * \param  separateThread   if *false* then will run in main thread, if not - starts in a separate thread
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if initialization was successful.
 *          - XME_STATUS_UNEXPECTED if operation is performed on an uninitialized component
 *          - XME_STATUS_INTERNAL_ERROR if anything goes wrong
 */
extern xme_status_t
xme_core_exec_run (
    uint32_t numCycles,
    bool separateThread
);

/*-------------------------------------------------------------------------*/
/**
 * \brief Stops the execution (break the loop)
 * \param atEndOfCycle  setting this to 'true' will cause pause at the end of schedule cycle
 *
  * \return Returns one of the following status codes:
 *          - XME_STATUS_UNEXPECTED if operation is performed on an uninitialized component
 *          - XME_STATUS_SUCCESS if all operations were successful.
 */
extern xme_status_t
xme_core_exec_stop
(
    bool atEndOfCycle
);

/*-------------------------------------------------------------------------*/
/**
 * \brief  Stops the whole execution manager,
 *          including dispatcher and scheduler, and clear all structures.
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if all operations were successful.
 *          - XME_STATUS_UNEXPECTED if operation is performed on an uninitialized component
 *          - XME_STATUS_INTERNAL_ERROR if it is not possible to perform
 *            full finalization.
 */
extern xme_status_t
xme_core_exec_fini( void );

/*-------------------------------------------------------------------------*/
/**
 * \brief Creates a dispatchable instance of a function.
 *
 * \param   function        descriptor of the function to start
 * \param   eventTriggered  a flag determining if the function will get started only
 *                          when all the necessary inputs are ready
 *
 * \return Returns one of the following status codes:
 *          - XME_STATUS_UNEXPECTED if operation is performed on an uninitialized component
 *          - XME_STATUS_SUCCESS if all operations were successful.
 *          - XME_STATUS_INTERNAL_ERROR if it is not possible to perform
 *            full finalization.
 */
extern xme_status_t
xme_core_exec_dispatcher_createFunctionExecutionUnit
(
    xme_core_exec_functionDescriptor_t* function,
    bool eventTriggered
);

XME_EXTERN_C_END

/**
 * @}
 */
#endif /* EXECUTIONMANAGER_H_ */


