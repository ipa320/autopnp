/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: componentRepositoryPnpManagerInterface.h 7828 2014-03-14 09:32:09Z ruiz $
 */

/**
 * \file
 *         Component Repository Plug and Play Manager Interface.
 *
 *         All functions defined here may only be called by the Plug and Play Manager or
 *         by XMT-generated code.
 */

/**
 * \addtogroup core_compRepository
 * @{
 *
 */

#ifndef XME_CORE_NODEMGR_COMPONENTREPOSITORYPNPMANAGERINTERFACE_H
#define XME_CORE_NODEMGR_COMPONENTREPOSITORYPNPMANAGERINTERFACE_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/core/nodeManager/include/componentRepositoryBuilder.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Set component ID (default is XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT).
 *
 * \param[in] componentBuilder Given component builder. Must not be NULL.
 * \param[in] componentID The new component ID.
 */
void
xme_core_nodeMgr_compRep_builderSetComponentID
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    xme_core_component_t componentID
);

/**
 * \brief Set foreign component handle.
 *
 * \param[in] componentBuilder Given component builder. Must not be NULL.
 * \param[in] foreignComponentHandle The new foreign component handle.
 */
void
xme_core_nodeMgr_compRep_builderSetForeignComponentHandle
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    xme_core_nodeMgr_compRep_componentHandle_t foreignComponentHandle
);

/**
 * \brief Assign a new component ID to the given component.
 *
 * \details May not be called when the component is already in state
 *          XME_CORE_NODEMGR_COMPREP_STATE_CREATED.
 *
 * \param[in] componentHandle Given component handle.
 * \param[in] componentID The new component ID.
 */
void
xme_core_nodeMgr_compRep_setComponentID
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle,
    xme_core_component_t componentID
);

/**
 * \brief Assign a new queue size to a given port.
 *
 * \details Only has effect when component of port is in state
 *          XME_CORE_NODEMGR_COMPREP_STATE_PREPARED or
 *          XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED.
 *
 * \param[in] portHandle Given port handle.
 * \param[in] queueSize The new queue size.
 */
void
xme_core_nodeMgr_compRep_setQueueSize
(
    xme_core_nodeMgr_compRep_portHandle_t portHandle,
    uint8_t queueSize
);

/**
 * \brief Get foreign component handle.
 *
 * \param[in] componentHandle Given component handle.
 *
 * \return The foreign component handle of the given component.
 */
xme_core_nodeMgr_compRep_componentHandle_t
xme_core_nodeMgr_compRep_getForeignComponentHandle
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
);

/**
 * \brief Set the state of the component to XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED.
 *
 * \details This state transition is only allowed if the current state is
 *          XME_CORE_NODEMGR_COMPREP_STATE_PREPARED and if the component
 *          has a valid ID assigned.
 *
 * \param[in] componentHandle Given component handle.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successful.
 * \retval XME_STATUS_INVALID_CONFIGURATION if one of the above mentioned
 *         conditions for the transition is violated.
 */
xme_status_t
xme_core_nodeMgr_compRep_setStateToAnnounced
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
);

/**
 * \brief Destroys the given component.
 *
 * \details This will deregister and destroy all ports and functions of the
 *          component (if available).
 *
 * \param[in] componentHandle Handle of the component.
 */
void
xme_core_nodeMgr_compRep_destroyComponentInstance
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_NODEMGR_COMPONENTREPOSITORYPNPMANAGERINTERFACE_H

