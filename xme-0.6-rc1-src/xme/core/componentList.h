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
 * $Id: componentList.h 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 *         Software component abstraction.
 */

#ifndef XME_CORE_COMPONENTLIST_H
#define XME_CORE_COMPONENTLIST_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

// TODO (See ticket #745): Implicitly add directory to component list as follows
//       (currently, this produces an inclusion order problem):
#ifndef XME_CORE_NO_IMPLICIT_COMPONENTS
    #ifdef XME_CORE_DIRECTORY_TYPE_LOCAL
        #define XME_COMPONENT_LIST_BEGIN \
            XME_COMPONENT_CONFIG_INSTANCE(xme_core_directory) = \
            { \
                /* public */ \
                {XME_CORE_DATACHANNEL_MASK}, /* baseDataChannel */ \
                {XME_CORE_DATACHANNEL_MASK} /* dataChannelMask */ \
            }; \
            \
            xme_core_componentDescriptor_t xme_core_resourceManager_componentDescriptorArray[] = \
            { \
                XME_COMPONENT_LIST_ITEM(xme_core_directory, 0) \
                XME_COMPONENT_LIST_ITEM_MANUAL \
                ( \
                    xme_core_resourceManager, \
                    &xme_core_resourceManager_create, \
                    &xme_core_resourceManager_activate, \
                    &xme_core_resourceManager_deactivate, \
                    &xme_core_resourceManager_destroy, \
                    NULL\
                )
                // TODO: Add resource manager as well. See ticket #745
    #else // #ifdef XME_CORE_DIRECTORY_TYPE_LOCAL
        #define XME_COMPONENT_LIST_BEGIN \
            XME_COMPONENT_CONFIG_INSTANCE(xme_core_directory) = \
            { \
                /* public */ \
                {(xme_core_dataChannel_t)0}, /* baseDataChannel */ \
                {XME_CORE_DATACHANNEL_MASK} /* dataChannelMask */ \
            }; \
            \
            xme_core_componentDescriptor_t xme_core_resourceManager_componentDescriptorArray[] = \
            { \
                XME_COMPONENT_LIST_ITEM(xme_core_directory, 0) \
                XME_COMPONENT_LIST_ITEM_MANUAL \
                ( \
                    xme_core_resourceManager, \
                    &xme_core_resourceManager_create, \
                    &xme_core_resourceManager_activate, \
                    &xme_core_resourceManager_deactivate, \
                    &xme_core_resourceManager_destroy, \
                    NULL\
                )
                // TODO: Add resource manager as well. See ticket #745
    #endif // #ifdef XME_CORE_DIRECTORY_TYPE_LOCAL
#else // #ifdef XME_CORE_NO_IMPLICIT_COMPONENTS
    #define XME_COMPONENT_LIST_BEGIN \
        xme_core_componentDescriptor_t xme_core_resourceManager_componentDescriptorArray[] = \
        {
#endif // #ifdef XME_CORE_NO_IMPLICIT_COMPONENTS

/**
 * \def XME_COMPONENT_LIST_ITEM
 *
 * \brief  List a component item callbacks.
 *
 * \param  componentName Component name.
 * \param  configIndex Configuration Index.
 */
#define XME_COMPONENT_LIST_ITEM(componentName, configIndex) \
        { \
            (xme_core_componentCreationCallback_t)&componentName##_create, \
            (xme_core_componentActivationCallback_t)&componentName##_activate, \
            (xme_core_componentDeactivationCallback_t)&componentName##_deactivate, \
            (xme_core_componentDestructionCallback_t)&componentName##_destroy, \
            (xme_core_componentConfigStruct_t*)&componentName##_config[configIndex], \
            XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, \
            #componentName \
        },

/**
 * \def XME_COMPONENT_LIST_ITEM_MANUAL
 *
 * \brief  List a component item establishing callbacks.
 *
 * \param  componentName Component name.
 * \param  createCallback Component creation callback.
 * \param  activateCallback Component activation callback.
 * \param  deactivateCallback Component deactivation callback.
 * \param  destroyCallback Component destroy callback.
 * \param  config Configuration structure.
 */
#define XME_COMPONENT_LIST_ITEM_MANUAL(componentName, createCallback, activateCallback, deactivateCallback, destroyCallback, config) \
        { \
            (xme_core_componentCreationCallback_t)createCallback, \
            (xme_core_componentActivationCallback_t)activateCallback, \
            (xme_core_componentDeactivationCallback_t)deactivateCallback, \
            (xme_core_componentDestructionCallback_t)destroyCallback, \
            (xme_core_componentConfigStruct_t*)config, \
            XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, \
            #componentName \
        },

/**
 * \def XME_COMPONENT_LIST_EMPTY
 * \brief  Creates an empty component list.
 */
#define XME_COMPONENT_LIST_EMPTY /* nothing */

/**
 * \def XME_COMPONENT_LIST_END
 * \brief  Finalizes a component list.
 */
#define XME_COMPONENT_LIST_END \
    }; \
XME_EXTERN_C_BEGIN \
xme_core_componentDescriptor_t* xme_core_nodeManager_componentManager_componentDescriptorsPtr = (xme_core_componentDescriptor_t*) xme_core_resourceManager_componentDescriptorArray; \
xme_core_component_t xme_core_nodeManager_componentManager_initialNumberOfComponents = (xme_core_component_t)(sizeof(xme_core_resourceManager_componentDescriptorArray)/sizeof(xme_core_resourceManager_componentDescriptorArray[0])); \
XME_EXTERN_C_END \

#endif // #ifndef XME_CORE_COMPONENTLIST_H
