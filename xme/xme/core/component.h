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
 * $Id: component.h 7483 2014-02-18 16:14:01Z wiesmueller $
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
#include "xme/core/coreTypes.h"
#include "xme/core/topic.h"

#include <limits.h>
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


// Component ID boundary values. 
#define XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT ((uint32_t) 0U) ///< Defines the invalid component context for any given component identifier.
#define XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT ((uint32_t) XME_MAX_SYSTEM_VALUE) ///< Defines the maximum component context for any given component identifier. 

// NOTE:
// Following static component IDs are also present in XMT
// org.fortiss.chromosome.xmt/src/org/fortiss/chromosome/xmt/model/BuiltInModels.java
// If you change any value please update the above file too.

// Static core components.
#define XME_CORE_COMPONENT_ID_LOGINMANAGER ((uint32_t) 1U) ///< Defines the predefined component identifier for login manager component at startup.
#define XME_CORE_COMPONENT_ID_LOGINCLIENT ((uint32_t) 2U) ///< Defines the predefined component identifier for login client component at startup.
#define XME_CORE_COMPONENT_ID_PNPMANAGER ((uint32_t) 3U) ///< Defines the predefined component identifier for plug and play manager component at startup.
#define XME_CORE_COMPONENT_ID_PNPCLIENT ((uint32_t) 4U) ///< Defines the predefined component identifier for plug and play client component at startup.

// Fixed Waypoint ComponentIDs
#define XME_CORE_COMPONENT_ID_WAYPOINT_MARSHALER ((uint32_t) 5U) ///< Defines the predefined component identifier for marshaler waypoint at startup.
#define XME_CORE_COMPONENT_ID_WAYPOINT_DEMARSHALER ((uint32_t) 6U) ///< Defines the predefined component identifier for demarshaler waypoint at startup.
#define XME_CORE_COMPONENT_ID_WAYPOINT_UDPSEND ((uint32_t) 7U) ///< Defines the predefined component identifier for UDP send waypoint at startup.
#define XME_CORE_COMPONENT_ID_WAYPOINT_UDPRECEIVE ((uint32_t) 8U) ///< Defines the predefined component identifier for UDP receive waypoint at startup.
#define XME_CORE_COMPONENT_ID_WAYPOINT_CHANNELINJECTOR ((uint32_t) 9U) ///< Defines the predefined component identifier for channel injector waypoint at startup.
#define XME_CORE_COMPONENT_ID_WAYPOINT_CHANNELSELECTOR ((uint32_t) 10U) ///< Defines the predefined component identifier for channel selector waypoint at startup.

// Initial code generated components. 
#define XME_CORE_COMPONENT_CODEGEN_INITIAL_ID ((uint32_t) 128U) ///< Defines the lower component identifier for code generated components.
#define XME_CORE_COMPONENT_CODEGEN_MAX_ID ((uint32_t) 4095U) ///< Defines the maximum component identifier for code generated components.

// Plug and Play Manager runtime assigned components. 
#define XME_CORE_COMPONENT_PNPMANAGER_INITIAL_ID ((uint32_t) 4096U) ///< Defines the lower component identifier for runtime plug and play manager assigned components.
#define XME_CORE_COMPONENT_PNPMANAGER_MAX_ID XME_CORE_COMPONENT_MAX_COMPONENT_CONTEXT ///< Defines the maximum component identifier for runtime plug and play manager assigned components.

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_core_component_t
 *
 * \brief Locally valid identifier of a component.
 */
typedef uint32_t xme_core_component_t;

/**
 * \typedef xme_core_component_config_t
 *
 * \brief Generic component configuration type: currently void internally.
 */
typedef void xme_core_component_config_t;

/**
 * \enum xme_core_component_functionId_t
 *
 * \brief Locally valid identifier of a function.
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
 *
 * \deprecated This definition is deprecated, new component wrappers
 *             use xme_core_component_portState_t instead!
 */
// TODO: Remove after all component wrappers have been updated (Issue #4078)
typedef enum
{
    XME_CORE_COMPONENT_PORTSTATUS_INVALID = 0, ///< Port does not hold valid data.
    XME_CORE_COMPONENT_PORTSTATUS_VALID = 1, ///< Port holds valid data.
}
xme_core_component_portStatus_t;

/**
 * \typedef xme_core_component_portState_t
 *
 * \brief  Represents the state of a component port in the component wrapper.
 */
typedef struct
{
    uint8_t dataValid : 1; ///< For input ports, whether data has been successfully read at least once. For output ports, whether data have been successfully written at least once.
    uint8_t attributesValid : 1; ///< For input ports, whether at least one attribute has been successfully read. For output ports, whether at least one attribute has been successfully written.
    uint8_t locked : 1; /* TODO: REMOVE! */ ///< Whether the port is currently locked.
    uint8_t error : 1; ///< For input ports, whether at least one error occurred while reading data or attributes. For output ports, whether at least one error occurred while writing data or attributes.
}
xme_core_component_portState_t;

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
 * \enum xme_core_component_connectionBound_e
 *
 * \brief Reserved special values for xme_core_component_connectionBound_t.
 */
enum xme_core_component_connectionBound_e
{
    XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED = USHRT_MAX - 1, ///< Unbounded means no restriction on the allowed number of connections.
    XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID = USHRT_MAX ///< Invalid connection bound.
};

/**
 * \typedef xme_core_component_connectionBound_t
 *
 * \brief Connection bounds for ports.
 *        See xme_core_component_connectionBound_e for reserved special values.
 *
 * \details Meaning depends on the type of the port and whether it is a lower
 *          or upper connection bound.
 *           - Lower connection bound:
 *             For input ports the lower connection bound is the minimum number
 *             of connections (incoming routes) to the port that are required.
 *             For a request sender this is the minimum number of request
 *             handlers that should answer the request.
 *           - Upper connection bound:
 *             For input ports this is the maximum number of connections
 *             (incoming routes) to the port that are allowed.
 *             For a request sender this is the maximum number of request
 *             handlers that will answer the request.
 *          For any other port type the connection bounds must be set to
 *          XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID.
 */
typedef uint16_t xme_core_component_connectionBound_t;

/**
 * \typedef xme_core_componentConfigStruct_t
 *
 * \brief Component configuration structure type (dereferenced).
 */
typedef void xme_core_componentConfigStruct_t;

/**
 * \typedef xme_core_componentCreationCallback_t
 *
 * \brief Callback function for software component creation.
 */
typedef xme_status_t (*xme_core_componentCreationCallback_t) (xme_core_componentConfigStruct_t* config);

/**
 * \typedef xme_core_componentActivationCallback_t
 *
 * \brief Callback function for software component activation and
 *        deactivation.
 */
typedef xme_status_t (*xme_core_componentActivationCallback_t) (xme_core_componentConfigStruct_t* config);

/**
 * \typedef xme_core_componentDeactivationCallback_t
 *
 * \brief Callback function for software component activation and
 *        deactivation.
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
    XME_CORE_ATTRIBUTE_KEY_CHANNELID = 1, ///< Channel id attribute, denotes channel where data should be sent to (data type: uint32_t).
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
    uint32_t size; ///< the attribute size. 
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

/**
 * \brief Predefined values for xme_core_component_requestDataHandle_t.
 */
enum xme_core_component_requestDataHandle_e
{
    XME_CORE_COMPONENT_INVALID_REQUEST_DATA_HANDLE = XME_CORE_INVALID_CHANNEL_ID ///< Indicates invalid handle.
};

/**
 * \typedef xme_core_component_requestDataHandle_t
 *
 * \brief Handle for requests that are used by the readRequestPort and
 *        writeResponsePort functions of component wrappers.
 *
 * \note This should not be modified/interpreted by the user.
 */
typedef xme_core_channelId_t xme_core_component_requestDataHandle_t;

#endif // #ifndef XME_CORE_COMPONENT_H
