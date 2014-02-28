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
 * $Id: manifestTypes.h 5233 2013-09-30 15:18:55Z geisinger $
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

#include "xme/xme_opt.h"

#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum   xme_core_componentType_t
 *
 * \brief  The different component manifest types that are available
 *         for XME.
 */
typedef enum
{
    XME_CORE_COMPONENT_TYPE_INVALID = 0x00000000, ///< Invalid component type.

    XME_CORE_COMPONENT_TYPE_PNPMANAGER = 0x00000001, ///< Plug and play manager.
    XME_CORE_COMPONENT_TYPE_PNPCLIENT = 0x00000002, ///< Plug and play client.

    XME_CORE_COMPONENT_TYPE_LOGINMANAGER = 0x0000000A, ///< Login manager.
    XME_CORE_COMPONENT_TYPE_LOGINCLIENT = 0x0000000B, ///< Login client.
    
    XME_CORE_COMPONENT_TYPE_USER = 0x00001000, ///< Values up to here are reserved for XME component types.

    XME_CORE_COMPONENT_TYPE_SENSOR = 0x00001101, ///< FIXME: Remove me as soon as pnpClient does not depend on this anymore (Issue #3178, #3458).
    XME_CORE_COMPONENT_TYPE_SENSOR_B = 0x00001102, ///< FIXME: Remove me as soon as pnpClient does not depend on this anymore (Issue #3178, #3458).
    XME_CORE_COMPONENT_TYPE_SENSOR_KB = 0x00001103, ///< FIXME: Remove me as soon as pnpClient does not depend on this anymore (Issue #3178, #3458).
    XME_CORE_COMPONENT_TYPE_SENSOR_MB = 0x00001104, ///< FIXME: Remove me as soon as pnpClient does not depend on this anymore (Issue #3178, #3458).

    XME_CORE_COMPONENT_TYPE_MONITOR = 0x00001108, ///< FIXME: Remove me as soon as pnpClient does not depend on this anymore (Issue #3178, #3458).
    XME_CORE_COMPONENT_TYPE_MONITOR_B = 0x00001109, ///< FIXME: Remove me as soon as pnpClient does not depend on this anymore (Issue #3178, #3458).
    XME_CORE_COMPONENT_TYPE_MONITOR_KB = 0x0000110A, ///< FIXME: Remove me as soon as pnpClient does not depend on this anymore (Issue #3178, #3458).
    XME_CORE_COMPONENT_TYPE_MONITOR_MB = 0x0000110B, ///< FIXME: Remove me as soon as pnpClient does not depend on this anymore (Issue #3178, #3458).
}
xme_core_componentType_t;

/**
 * \struct xme_core_componentSchedulingManifest_t
 *
 * \brief  The alpha curve of a component.
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
    bool completion; ///< Defines if the function should be scheduled preemptive or not.
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
}
xme_core_componentPortManifest_t;

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
