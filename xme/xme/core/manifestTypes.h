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
 * $Id: manifestTypes.h 7844 2014-03-14 14:11:49Z ruiz $
 */

/**
 * \file
 *         Definition of manifest data types.
 */

#ifndef XME_CORE_MANIFEST_TYPES_H
#define XME_CORE_MANIFEST_TYPES_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include "xme/core/container.h"
#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"
#include "xme/core/directory/include/attribute.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/xme_opt.h"

#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum xme_core_componentManifest_e
 *
 * \brief The different component manifest types that are available
 *        for XME.
 */
enum xme_core_componentManifest_e
{
    XME_CORE_COMPONENT_TYPE_INVALID = 0x00000000, ///< Invalid component type.

    // Core component types are defined in generated *Manifest.h files

    // FIXME: Remove these items as soon as pnpManager and LRM tests do not depend on this anymore (Issue #3178, #3458):
    XME_CORE_COMPONENT_TYPE_SENSOR = 0x00000F00,
    XME_CORE_COMPONENT_TYPE_SENSOR_B,
    XME_CORE_COMPONENT_TYPE_SENSOR_KB,
    XME_CORE_COMPONENT_TYPE_SENSOR_MB,
    XME_CORE_COMPONENT_TYPE_MONITOR,
    XME_CORE_COMPONENT_TYPE_MONITOR_B,
    XME_CORE_COMPONENT_TYPE_MONITOR_KB,
    XME_CORE_COMPONENT_TYPE_MONITOR_MB,
    
    XME_CORE_COMPONENT_TYPE_USER = 0x00001000, ///< Values up to here are reserved for XME component types.
};

/**
 * \typedef xme_core_componentType_t
 * \brief Defines the component type. 
 */
typedef uint32_t xme_core_componentType_t;

/**
 * \struct xme_core_componentSchedulingManifest_t
 *
 * \brief The alpha curve of a component.
 */
typedef struct
{
    //TODO: this typedef is a dummy => substitute by final datatype, see ticket #2166
    int alphaCurve; ///< The alpha curve. 
}
xme_core_componentSchedulingManifest_t;

/**
 * \struct xme_core_functionManifest_t
 *
 * \brief  The manifest of a container.
 */
typedef struct
{
    xme_core_component_functionId_t functionId; ///< Locally unique identifier of function within component.
    char name[32]; ///< The function manifest name. 
    xme_hal_time_timeInterval_t wcet; ///< The worst case execution time.
    xme_core_componentSchedulingManifest_t alphaCurve; ///< The alpha curve. 
    char completion; ///< Defines if the function should be scheduled preemptive (nonzero value) or not (zero).
    uint8_t requiredPortIndices[XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_FUNCTION]; ///< Indices of ports (in xme_core_componentManifest_t.portManifests) that are required ports of this function.
    uint8_t requiredPortIndicesLength; ///< Number of valid entries in requiredPortIndices.
    uint8_t optionalPortIndices[XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_FUNCTION]; ///< Indices of ports (in xme_core_componentManifest_t.portManifests) that are optional ports of this function.
    uint8_t optionalPortIndicesLength; ///< Number of valid entries in optionalPortIndices.
    xme_core_exec_initCallback_t functionInit; ///< Pointer to init function of this function's implementation.
    xme_core_exec_finiCallback_t functionFini; ///< Pointer to fini function of this function's implementation.
    xme_hal_sched_taskCallback_t functionWrapperExecute; ///< Pointer to execute function of this function's wrapper.
}
xme_core_functionManifest_t;

/**
 * \struct xme_core_componentPortManifest_t
 *
 * \brief  The manifest of a component port. 
 */
typedef struct
{
    xme_core_component_portType_t portType; ///< Type of the port.
    xme_core_topic_t topic; ///< Topic associated to the port.
    xme_core_directory_attributeSetHandle_t attrSet; ///< Attribute set of the port's topic.
    xme_core_component_connectionBound_t lowerConnectionBound; ///< Lower connection bound. \see xme_core_component_connectionBound_t
    xme_core_component_connectionBound_t upperConnectionBound; ///< Upper connection bound. \see xme_core_component_connectionBound_t
    uint8_t queueSize; ///< Size (max number of elements) of queue of this port. Must be greater than zero.
    char persistent; ///< Whether this port is persistent.
}
xme_core_componentPortManifest_t;

/**
 * \brief Function pointer to component init function, used in xme_core_componentManifest_t.
 */
typedef xme_status_t (*xme_core_componentManifest_componentInit_t) (void* const config, const char* initializationString);

/**
 * \struct xme_core_componentManifest_t
 *
 * \brief  The manifest of a component.
 */
typedef struct
{
    xme_core_componentType_t componentType; ///< The global component type. 
    char name[32]; ///< The component name. 
    xme_core_functionManifest_t functionManifests[XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT]; ///< The associated function manifests.
    xme_core_componentPortManifest_t portManifests[XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT]; ///< The associated port manifests.
    xme_status_t (*componentWrapperInit) (void); ///< Pointer to init function of this component's wrapper.
    xme_status_t (*componentWrapperReceivePort) (xme_core_dataManager_dataPacketId_t dataPacketId, uint8_t componentInternalPortId); ///< Pointer to receivePort function of this component's wrapper.
    void (*componentWrapperFini) (void); ///< Pointer to fini function of this component's wrapper.
    xme_core_componentManifest_componentInit_t componentInit; ///< Pointer to init function of this component's implementation.
    uint16_t configStructSize; ///< Size of configuration structure as passed in componentInit.
    char constraint[512]; ///< Constraint string for this component (null-terminated).
}
xme_core_componentManifest_t;

/**
 * \struct xme_core_containerManifest_t
 *
 * \brief  The manifest of a container
 */
typedef struct
{
    xme_core_container_t containerId; ///< The container id. 
    char* sharedLibraryPath; ///< The shared library path. 
    xme_core_binarySize_t binarySize; ///< The binary size. 
    xme_core_componentManifest_t componentManifests[XME_CORE_MANIFEST_TYPES_MAX_COMPONENTS_PER_CONTAINER]; ///< The component manifests associated to the container. 
}
xme_core_containerManifest_t;

#endif // #ifndef XME_CORE_MANIFEST_TYPES_H
