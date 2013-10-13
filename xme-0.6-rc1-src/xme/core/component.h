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
 * $Id: component.h 5108 2013-09-18 17:50:19Z geisinger $
 */

/**
 * \file
 *         Software component abstraction.
 *
 */

#ifndef XME_CORE_COMPONENT_H
#define XME_CORE_COMPONENT_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
//FIXME: this should be removed, we only need that due to manifest declaration
#include "xme/core/topic.h"

#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
// TODO (See ticket #803): Remove these defines and force correct order in component list (i.e., 1. directory, 2. resource manager)
#define XME_CORE_COMPONENT_DIRECTORY_COMPONENT_ID ((xme_core_component_t)0x0001) ///< Component ID of directory.
#define XME_CORE_COMPONENT_RESOURCEMANAGER_COMPONENT_ID ((xme_core_component_t)0x0002) ///< Component ID of resource manager.

#define XME_CORE_ATTRIBUTES(attribute) ((xme_core_attribute_key_t)(attribute)) ///< transformation of attributes to core attribute keys datatype.

/**
 * \def    XME_COMPONENT_CONFIG_STRUCT
 *
 * \brief  Defines a configuration structure type for a component type with
 *         the given name.
 *
 *         The configuration structure type will be named
 *         (componentName)_configStruct_t.
 *         Using this macro ensures that naming conventions are enforced.
 *
 * \param  componentTypeName Full name of the component type the configuration
 *         structure is to be defined for (e.g., xme_adv_myComponent).
 * \param  members List of members (with data type) in the configuration
 *         structure, separated by semicola.
 */
#define XME_COMPONENT_CONFIG_STRUCT(componentTypeName, members) \
    typedef struct \
    { \
        members \
    } \
    componentTypeName##_configStruct_t

/**
 * \def    XME_COMPONENT_CONFIG
 *
 * \brief  Generates the name of the component configuration array for the
 *         given component type.
 *
 * \param  componentTypeName Full name of the component the configuration structure
 *         is defined for (e.g., xme_adv_myComponent).
 */
#define XME_COMPONENT_CONFIG(componentTypeName) \
    componentTypeName##_config

/**
 * \def    XME_COMPONENT_CONFIG_INSTANCE
 *
 * \brief  Defines an array of component configurations for the given
 *         component type.
 *
 *         The component configuration array type will be named
 *         (componentName)_config and will be of type
 *         (componentName)_configStruct_t.
 *
 *         Usage examples:
 *         \code
 *         // In case you want to initialize at least one member of the
 *         // configuration structure, use the following syntax
 *         // (assuming xme_adv_myComponent has at least two public
 *         // configuration variables, the first one being a string and
 *         // the second one being a number):
 *         XME_COMPONENT_CONFIG_INSTANCE(xme_adv_myComponent) =
 *         {
 *             // Initial configuration of first instance of xme_adv_myComponent
 *             "myComponent1",
 *             42
 *         },
 *         {
 *             // Initial configuration of second instance of xme_adv_myComponent
 *             "myComponent2",
 *             43
 *         };
 *         \endcode
 *
 *         \code
 *         // In case the component does not contain any members that should
 *         // be initialized (assuming two instance of xme_adv_myComponent
 *         // should exist):
 *         XME_COMPONENT_CONFIG_INSTANCE(xme_adv_myComponent, 2);
 *         \endcode
 *
 * \param  componentTypeName Full name of the component type the configuration
 *         array is to be defined for (e.g., xme_adv_myComponent).
 * \param  ... Optional parameter that specifies the number of configuration
 *         instances to create in case the configuration instances do not
 *         contain any fields that should be initialized. In this case, the
 *         compiler can not statically determine the number of elements,
 *         which is why their number has to be explicitly specified here.
 *         This value can also be specified when the configuration instances
 *         are initialized explicitly, in which case it must match the
 *         number of explicit configuration instances initializations that
 *         follow this macro. See examples section.
 */
#define XME_COMPONENT_CONFIG_INSTANCE(componentTypeName, ...) \
    static \
    componentTypeName##_configStruct_t XME_COMPONENT_CONFIG(componentTypeName)[__VA_ARGS__]

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \enum xme_core_component_t
 *
 * \brief  Locally valid identifier of a component.
 */
typedef enum
{
    XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT = 0, ///< Invalid component context.
    XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT = XME_MAX_SYSTEM_VALUE ///< Largest possible component context.
}
xme_core_component_t;

/**
 * \enum xme_core_component_functionId_t
 *
 * \brief  Locally valid identifier of a function.
 */
typedef enum
{
    XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT = 0, ///< Invalid component context.
    XME_CORE_COMPONENT_MAX_FUNCTION_CONTEXT = XME_MAX_SYSTEM_VALUE ///< Largest possible component context.
}
xme_core_component_functionId_t;

/**
 * \typedef xme_core_component_functionVariantId_t
 *
 * \brief  Function variant type: currently void* internally.
 */
typedef void* xme_core_component_functionVariantId_t;

/**
 * \typedef xme_core_component_portStatus_t
 *
 * \brief  Status of a component port.
 */
typedef enum
{
    XME_CORE_COMPONENT_PORTSTATUS_INVALID = 0, ///< Port does not hold valid data.
    XME_CORE_COMPONENT_PORTSTATUS_VALID = 1, ///< Port holds valid data.
}
xme_core_component_portStatus_t;

/**
 * \typedef xme_core_component_portType_t
 *
 * \brief  Type of a component or waypoint port.
 */
typedef enum
{
    XME_CORE_COMPONENT_PORTTYPE_INVALID = 0, ///< Invalid port type.
    XME_CORE_COMPONENT_PORTTYPE_INTERNAL = 1, ///< Internal port used in waypoints.
    XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION = 8, ///< Publication port (output port).
    XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION, ///< Subscription port (input port).
    XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_SENDER = 16, ///< Request sender port (output port).
    XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER, ///< Request handler port (input port).
    XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_SENDER, ///< Response sender port (output port).
    XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER ///< Response handler port (input port).
}
xme_core_component_portType_t;

/**
 * \typedef xme_core_componentConfigStruct_t
 *
 * \brief  Component configuration structure type (dereferenced).
 */
typedef void xme_core_componentConfigStruct_t;

/**
 * \typedef xme_core_componentCreationCallback_t
 *
 * \brief  Callback function for software component creation.
 */
typedef xme_status_t (*xme_core_componentCreationCallback_t) (xme_core_componentConfigStruct_t* config);

/**
 * \typedef xme_core_componentActivationCallback_t
 *
 * \brief  Callback function for software component activation and
 *         deactivation.
 */
typedef xme_status_t (*xme_core_componentActivationCallback_t) (xme_core_componentConfigStruct_t* config);

/**
 * \typedef xme_core_componentDeactivationCallback_t
 *
 * \brief  Callback function for software component activation and
 *         deactivation.
 */
typedef xme_status_t (*xme_core_componentDeactivationCallback_t) (xme_core_componentConfigStruct_t* config);

/**
 * \typedef xme_core_componentDestructionCallback_t
 *
 * \brief  Callback function for software component destruction.
 */
typedef xme_status_t (*xme_core_componentDestructionCallback_t) (xme_core_componentConfigStruct_t* config);

/**
 * \struct xme_core_componentDescriptor_t
 *
 * \brief  Software component descriptor.
 */
typedef struct
{
    xme_core_componentCreationCallback_t create; ///< the component creation callback. 
    xme_core_componentActivationCallback_t activate; ///< the component activation callback. 
    xme_core_componentDeactivationCallback_t deactivate; ///< the component deactivation callback. 
    xme_core_componentDestructionCallback_t destroy; ///< the component destruction callback. 
    xme_core_componentConfigStruct_t* config; ///< the configuration structure. 
    xme_core_component_t componentId; ///< the component identifier. 
    const char* componentName; ///< the component name. 
}
xme_core_componentDescriptor_t;

/**
 * \typedef xme_core_attr_listHandle_t
 *
 * \brief  Handle to the attribute list.
 *
 * \deprecated This type definition is deprecated.
 *             Use xme_core_directory_attributeSetHandle_t instead!
 */
typedef int xme_core_attr_listHandle_t;

/**
 * \typedef xme_core_attribute_key_t
 *
 * \brief  Key to a given attribute value.
 */
typedef enum
{
    XME_CORE_ATTRIBUTE_KEY_UNDEFINED = 0,
    XME_CORE_ATTRIBUTE_KEY_USER = 0x00001000, ///< Minimum attribute identifier for user-defined (not reserved for XME itself) attributes.
    XME_CORE_ATTRIBUTE_KEY_MAX = 0xFFFFFFFF ///< Maximum valid attribute identifier.
}
xme_core_attribute_key_t;

/**
 * \struct xme_core_attribute_descriptor_t
 *
 * \brief The attribute descriptor.
 */
typedef struct
{
    xme_core_attribute_key_t key; ///< the attribute key. 
    size_t size; ///< the attribute size. 
}
xme_core_attribute_descriptor_t;

/**
 * \struct xme_core_attribute_descriptor_list_t
 *
 * \brief  The attribute descriptor list. 
 */
typedef struct
{
    uint8_t length; ///< number of elements.
    xme_core_attribute_descriptor_t *element; ///< array of elements.
}
xme_core_attribute_descriptor_list_t;

#endif // #ifndef XME_CORE_COMPONENT_H
