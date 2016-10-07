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
 * $Id: componentRepositoryPnpClientInterface.h 7814 2014-03-13 13:57:57Z geisinger $
 */

/**
 * \file
 *         Component Repository Plug and Play Manager Interface.
 *
 *         All functions defined here may only be called by the Plug and Play Client or
 *         by XMT-generated code.
 */

/**
 * \addtogroup core_compRepository
 * @{
 *
 */

#ifndef XME_CORE_NODEMGR_COMPONENTREPOSITORYPNPCLIENTINTERFACE_H
#define XME_CORE_NODEMGR_COMPONENTREPOSITORYPNPCLIENTINTERFACE_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/nodeManager/include/componentRepository.h"

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
 * \brief Creates data packets for the given component in the data handler
 *        and registers all the components functions in the broker.
 *        Also calls the init function of the component and configures the
 *        associated component wrapper.
 *        Sets the component state to XME_CORE_NODEMGR_COMPREP_STATE_CREATED.
 *
 * \note After this call xme_core_nodeMgr_compRep_getDataPacketID() can be
 *       used on the component.
 *
 * \param[in] componentHandle Given component handle.
 *
 * \retval XME_STATUS_SUCCESS if operation was successful.
 * \retval XME_STATUS_NOT_FOUND if given component handle is unknown, or
 *         necessary type information, like the manifests, topics and
 *         attributes could not be found.
 * \retval XME_STATUS_INVALID_CONFIGURATION if previous state was not
 *         XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED.
 * \retval XME_STATUS_ALREADY_EXIST if a component with the same component
 *         ID and node ID already exists in the repository.
 * \retval XME_STATUS_INTERNAL_ERROR when any call from the component repository
 *         to other components fail (e.g. broker registration, etc.).
 */
xme_status_t
xme_core_nodeMgr_compRep_createAndRegisterComponent
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
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
 * \brief Sets the component state to XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED.
 *        Will be called after the component instance manifest has been sent.
 *
 * \param[in] componentHandle Given component handle.
 *
 * \retval XME_STATUS_SUCCESS if operation was successful.
 * \retval XME_STATUS_NOT_FOUND If no component with the given handle could be found.
 * \retval XME_STATUS_INVALID_CONFIGURATION if the previous state of the component
 *         was not XME_CORE_NODEMGR_COMPREP_STATE_PREPARED.
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

#endif // #ifndef XME_CORE_NODEMGR_COMPONENTREPOSITORYPNPCLIENTINTERFACE_H
