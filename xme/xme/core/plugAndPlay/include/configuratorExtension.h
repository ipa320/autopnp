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
 * $Id: configuratorExtension.h 7748 2014-03-10 16:02:11Z wiesmueller $
 */

/**
 * \file
 *         Configurator Extension.
 */

/*
 * TODO: (issue #3626)
 * Open problems/questions:
 * - Providing dedicated data structure + interface for user instead of data link graph?
 *   - Data link graph requires knowledge about internal structure (e.g. casting to correct vertex/edge data)
 *   - User more interested in components and routes (data link graph contains only ports and routes)
 *   - Info about network structure (available nodes and how are they connected physically, what components are running on a node...)
 * - Note that we do not detect infinite cycles in configurators
 * - Make this component optional (via global function pointer(s), should a new build system component be created for this?)
 */

#ifndef XME_CORE_PNP_CONFIGURATOREXTENSION_H
#define XME_CORE_PNP_CONFIGURATOREXTENSION_H

/**
 * \defgroup core_pnp_configExt Configurator Extension group. 
 * @{
 *
 * \brief The Configurator Extension is an optional core component that allows
 *        users to register their own configurator that can modify the data link
 *        graph calculated by the Logical Route Manager and the Network Configuration
 *        Calculator.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/manifestTypes.h"
#include "xme/core/plugAndPlay/include/logicalRouteManager.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \brief Callback function for configurators.
 *
 * \see xme_core_pnp_configExt_addConfigurator
 */
typedef void (*xme_core_pnp_configExt_configuratorCallback_t)(xme_hal_graph_graph_t* const configGraph);

/**
 * \brief Handle for identifying a cofnigurator.
 */
typedef uint8_t xme_core_pnp_configExt_configuratorHandle_t;

/**
 * \brief Values for xme_core_pnp_configExt_configuratorType_t.
 *
 * \note Do not use this type directly, but use xme_core_pnp_configExt_configuratorType_t instead.
 */
enum xme_core_pnp_configExt_configuratorType_e
{
    XME_CORE_PNP_CONFIG_EXT_CONFIGURATORTYPE_LOGICAL_ROUTES ///< Configurator will be triggered after the Logical Route Manager. Given graph in callback will be xme_core_pnp_lrm_logicalRoutes_t.
};

/**
 * \brief Indicates type of configurator, which determines when during the configuration
 *        process it will be called.
 *
 * \note See xme_core_pnp_configExt_configuratorType_e for possible values.
 */
typedef uint8_t xme_core_pnp_configExt_configuratorType_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the configurator extension.
 *
 * \retval XME_STATUS_SUCCESS when initialization was successful.
 */ 
xme_status_t
xme_core_pnp_configExt_init(void);

/**
 * \brief Frees all resources occupied by the configurator extension.
 */
void
xme_core_pnp_configExt_fini(void);

/**
 * \brief Adds a new configurator.
 *
 * \details All registered configurators will be called during the configuration
 *          process (e.g. on a plugin-event). The exact when they are called
 *          is determined by the type parameter.
 *          For modifying the configuration the callback can use the following
 *          functions:
 *          - xme_core_pnp_configExt_addComponent()
 *          - xme_core_pnp_configExt_removeComponent()
 *          - xme_core_pnp_configExt_removeLink()
 *
 *          When addComponent or removeComponent has been called, the configuration
 *          will be restarted (so that the logical route manager will compute the
 *          new graph with the new/removed components).
 *
 * \warning Reruns of the configuration process are triggered as long as any
 *          configurator calls addComponent or removeComponent.
 *          So the configurators must be designed in a way to prevent infinite cycles.
 *
 * \param[in] type Type of configurator.
 * \param[in] callback Callback that will be called during the configuration process.
 *            May not be NULL.
 * \param[out] handle Handle of the added configurator. Can be used later in 
 *             xme_core_pnp_configExt_removeConfigurator to remove the configurator.
 *             May be NULL.
 *
 * \retval XME_STATUS_SUCCESS if addition was successful.
 * \retval XME_STATUS_INVALID_PARAMETER when callback is NULL.
 * \retval XME_STATUS_OUT_OF_RESOURCES when there are not enough resources left to
 *         add a new configurator.
 */
xme_status_t
xme_core_pnp_configExt_addConfigurator
(
    xme_core_pnp_configExt_configuratorType_t type,
    xme_core_pnp_configExt_configuratorCallback_t callback,
    xme_core_pnp_configExt_configuratorHandle_t* handle
);

/**
 * \brief Remove the configurator with the given handle.
 *
 * \param handle Handle, as returned by xme_core_pnp_configExt_addConfigurator,
 *        of the configurator to remove.
 *
 * \retval XME_STATUS_SUCCESS if the configurator has been successfully removed.
 * \retval XME_STATUS_INVALID_HANDLE if the given handle was invalid.
 */
xme_status_t
xme_core_pnp_configExt_removeConfigurator
(
    xme_core_pnp_configExt_configuratorHandle_t handle
);

/**
 * \brief Executes all registered configurators.
 *
 * \param type Only configurators added with the given type
 *        will be executed.
 * \param logicalRouteGraph The logical route graph that will be passed to the
 *        configurators. May not be NULL.
 *
 * \return Returns true when a rerun of the configuration is necessary, otherwise
 *         false.
 */
bool
xme_core_pnp_configExt_executeConfigurators
(
    xme_core_pnp_configExt_configuratorType_t type,
    xme_core_pnp_lrm_logicalRoutes_t* logicalRouteGraph
);

/**
 * \brief Add a component on the given node.
 *
 * \details Will trigger a rerun of the configuration, beginning with the logical route
 *          manager.
 *
 * \note To be called from a configurator callback.
 *
 * \param[in] componentType Type of the new component.
 * \param[in] initializationString Component type specific initialization
 *            string to pass to the new component instance.
 * \param[in] nodeId Node ID of the node where the new component should be added.
 */
void
xme_core_pnp_configExt_addComponent
(
    xme_core_componentType_t componentType,
    const char* const initializationString,
    xme_core_node_nodeId_t nodeId
);

/**
 * \brief Remove a component from a specific node.
 *
 * \details Will trigger a rerun of the configuration, beginning with the logical route
 *          manager.
 *
 * \note To be called from a configurator callback.
 *
 * \param componentID Component ID of the component to remove.
 * \param nodeID Node ID of the node from which to remove the component.
 */
void
xme_core_pnp_configExt_removeComponent
(
    xme_core_component_t componentID,
    xme_core_node_nodeId_t nodeID
);

/**
 * \brief Removes a link from the configuration graph.
 *
 * \note To be called from a configurator callback.
 *
 * \warning Currently only supported for non-established routes.
 *
 * \param edgeId Edge ID of the link in the configuration graph that should
 *        be removed.
 */
xme_status_t
xme_core_pnp_configExt_removeLink
(
    xme_hal_graph_edgeId_t edgeId
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_CONFIGURATOREXTENSION_H