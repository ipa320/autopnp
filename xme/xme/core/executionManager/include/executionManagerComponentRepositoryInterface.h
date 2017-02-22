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
 * $Id: executionManagerComponentRepositoryInterface.h 7483 2014-02-18 16:14:01Z wiesmueller $
 */

/**
 * \file
 *         Execution manager component repository
 */

#ifndef XME_CORE_EXEC_COMPONENT_REPOSITORY_INTERFACE_H
#define XME_CORE_EXEC_COMPONENT_REPOSITORY_INTERFACE_H

/**
 * \ingroup core_em Execution Manager
 * @{
 *
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/defines.h"
#include "xme/core/component.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/*-------------------------------------------------------------------------*/
/**
 * \brief  Initializes the component repository.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the transfer was done correct.
 *          - XME_CORE_STATUS_INVALID_CONFIGURATION if a component of this
 *            type has already been initialized. Exactly one component of this
 *            type must be present on every node.
 */
extern xme_status_t
xme_core_exec_componentRepository_init( void );

/*-------------------------------------------------------------------------*/
/**
 * \brief  Finalize the component repository
 *     odo This function shall also clean up all the memory allocations.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the transfer was done correct.
 */
extern xme_status_t
xme_core_exec_componentRepository_fini( void );

/*-------------------------------------------------------------------------*/
/**
 * \brief  Adds a component to repository. All components that wish to be l
 * \param[in] componentDescriptor   the descriptor for component to register
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the component has been added
 *          - XME_CORE_STATUS_ALREADY_EXIST if the repository contains
 *              the component already
 */
extern xme_status_t
xme_core_exec_componentRepository_registerComponent
(
    const xme_core_exec_componentDescriptor_t* const componentDescriptor
);

/*-------------------------------------------------------------------------*/
/**
 * \brief  Gets the component descriptor by ID
 * \param[in] componentId Locally unique ID of the component
 * \param[out] component  component descriptor or NULL if not found
 * \return FIXME
 */
extern xme_status_t
xme_core_exec_componentRepository_getComponent
(
    xme_core_component_t componentId,
    xme_core_exec_componentDescriptor_t** component
);

/*-------------------------------------------------------------------------*/
/**
 * \brief Get a function descriptor by a (cid, fic) pair
 * \param[in] componentId   ID of the component
 * \param[in] functionId    ID of the function
 * \param[out]function      function descriptor or NULL if not found
 * \return FIXME
 */
extern xme_status_t
xme_core_exec_componentRepository_getFunction(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_exec_functionDescriptor_t** function
);

/*-------------------------------------------------------------------------*/
/**
 * \brief  Find a function descriptor when component is known
 * \param[in] component  descriptor of the component containing the function
 * \param[in] functionId ID of the function within the component
 * \param[out] function  descriptor of the function within the component
 * \return FIXME
 */
extern xme_status_t
xme_core_exec_componentRepository_getComponentFunction
(
        xme_core_exec_componentDescriptor_t* component,
        xme_core_component_functionId_t functionId,
        xme_core_exec_functionDescriptor_t** function
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif /* #ifdef XME_CORE_EXEC_COMPONENT_REPOSITORY_INTERFACE_H */
