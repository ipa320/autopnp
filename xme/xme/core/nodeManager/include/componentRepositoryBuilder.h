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
 * $Id: componentRepositoryBuilder.h 7844 2014-03-14 14:11:49Z ruiz $
 */

/**
 * \file
 *         Component Repository Builder.
 *
 *         Contains functions for building a new component instance.
 */

/**
 * \addtogroup core_compRepository
 * @{
 *
 */

#ifndef XME_CORE_NODEMGR_COMPONENTREPOSITORYBUILDER_H
#define XME_CORE_NODEMGR_COMPONENTREPOSITORYBUILDER_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/nodeManager/include/componentRepository.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_core_nodeMgr_compRep_componentBuilder_t
 *
 * \brief Data structure for the component builder.
 */
typedef struct xme_core_nodeMgr_compRep_componentBuilder_s
xme_core_nodeMgr_compRep_componentBuilder_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Creates a new component builder.
 *        Initializes a component instance with default options, according
 *        to its manifest.
 *
 * \details To override default settings of the new component instance call the
 *          respective builderSet*() functions.
 *          To finalize the component and build it, call xme_core_nodeMgr_compRep_build().
 *          If the return value is non-NULL then xme_core_nodeMgr_compRep_build()
 *          must be called to free again the resources used by the created
 *          builder.
 *
 * \param[in] nodeID ID of the node in which the component should reside.
 * \param[in] componentType Type of the new component instance.
 *
 * \return Pointer to the builder.
 *         The pointer will be NULL when:
 *         - No manifest for the given type is found (will be logged as error).
 *         - Not enough memory to create builder (will be logged as error).
 */
xme_core_nodeMgr_compRep_componentBuilder_t*
xme_core_nodeMgr_compRep_createBuilder
(
    xme_core_node_nodeId_t nodeID,
    xme_core_componentType_t componentType
);

/**
 * \brief Set the (null-terminated) initialization string (default is NULL).
 *
 * \details The given string will be copied.
 *          If there is not enough memory to copy the string, or the
 *          string is longer than the maximum, an error will be logged and
 *          the xme_core_nodeMgr_compRep_build() call will return an error.
 *
 * \param[in] componentBuilder Given component builder. Must not be NULL.
 * \param[in] initializationString The new initialization string.
 */
void
xme_core_nodeMgr_compRep_builderSetInitializationString
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    const char* const initializationString
);

/**
 * \brief Set the queue size for a given port (default depends on manifest).
 *
 * \details If the given port type ID, or queue size is not valid an error
 *          will be logged and the xme_core_nodeMgr_compRep_build() call will
 *          return an error.
 *
 * \param[in] componentBuilder Given component builder. Must not be NULL.
 * \param[in] portTypeID The port type ID as defined in the manifest.
 * \param[in] queueSize The new queue size. Must not be 0.
 */
void
xme_core_nodeMgr_compRep_builderSetQueueSize
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    uint16_t portTypeID,
    uint8_t queueSize
);

/**
 * \brief Set the execution period for a given function (default is 0).
 *
 * \details If the given function type ID is not valid an error will be
 *          logged and the xme_core_nodeMgr_compRep_build() call will
 *          return an error.
 *
 * \param[in] componentBuilder Given component builder. Must not be NULL.
 * \param[in] functionTypeID The function type ID as defined in the manifest.
 * \param[in] executionPeriod The new execution period.
 */
void
xme_core_nodeMgr_compRep_builderSetExecutionPeriod
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    uint16_t functionTypeID,
    xme_hal_time_timeInterval_t executionPeriod
);

/**
 * \brief Finalizes the build and creates the new component instance.
 *
 * \details Also frees all resources used by the builder. The builder
 *          pointer will be invalid when this function returns.
 *          The resulting component instance will have a state of
 *          XME_CORE_NODEMGR_COMPREP_STATE_PREPARED.
 *
 * \note This will not yet plug in the component. To do so call
 *       the respective plug and play client or manager functions
 *       with the resulting component handle.
 *
 * \param[in] componentBuilder Given component builder. Must not be NULL.
 * \param[out] outComponentHandle Handle of the new component instance. May be NULL.
 *
 * \retval XME_STATUS_SUCCESS if component creation was successful.
 * \retval XME_STATUS_ALREADY_EXIST if a component with the given component ID and
 *         node ID already exist in the repository.
 * \retval XME_STATUS_INVALID_PARAMETER if componentBuilder is NULL.
 * \retval XME_STATUS_ALREADY_EXIST if component ID is not invalid and a
 *         component with the given component ID and node ID already exists.
 *         The parameter outComponentHandle will then be assigned to the handle
 *         of the existing component.
 * \retval XME_STATUS_OUT_OF_RESOURCES if there is not enough memory to
 *         add the new component to the component repository.
 * \retval XME_STATUS_INTERNAL_ERROR if any of the previous setter calls
 *         on the builder failed. See the documentation of the setters
 *         for details.
 */
xme_status_t
xme_core_nodeMgr_compRep_build
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder,
    xme_core_nodeMgr_compRep_componentHandle_t* outComponentHandle
);

/**
 * \brief Cancel the build and free all resources.
 *
 * \param[in] componentBuilder Given component builder.
 */
void
xme_core_nodeMgr_compRep_cancelBuild
(
    xme_core_nodeMgr_compRep_componentBuilder_t* const componentBuilder
);

XME_EXTERN_C_END


/**
 * @}
 */

#endif // #ifndef XME_CORE_NODEMGR_COMPONENTREPOSITORYBUILDER_H
