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
 * $Id: plugAndPlayManager.h 5067 2013-09-16 13:00:23Z ruiz $
 */

/**
 * \file
 *         Plug and Play Manager.
 */

/*
### Tasks:
- Receiving new install requests including manifest
- Coordinating the deployment calculation (first version: fixed calculation included in manifest): 
    - check which nodes are able to run the component, 
    - check whether enough bandwidth is available, 
    - check if all required input ports can be wired
- In case deployment fails, rollback of relevant transactions
- In case of success, final commit of transactions
*/

#ifndef XME_CORE_PNP_PLUGANDPLAYMANAGER_H
#define XME_CORE_PNP_PLUGANDPLAYMANAGER_H

/**
 * \defgroup core_pnp_pnpManager Plug and Play Manager group. 
 * @{
 *
 * \brief Plug and Play Manager group is related with controlling each new component
 *        attached to the system, and automatically configure it. 
 *
 * \details The plug and play manager gets new requests from plugged components and
 *          requests to connect new components in local or remote nodes.
 *          this component uses two additional components: logical route manager and
 *          network configuration calculator. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/manifestRepository/include/manifestRepository.h"

#include "xme/core/node.h"
#include "xme/core/topicData.h"


/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes the plug and play manager component.
 *         Exactly one component of this type must be present on every node.
 * 
 * \param params Initialization parameters for plug and play manager component (TBD)
 * 
 * \retval XME_SUCCESS if the plug and play manager component has been properly initialized.
 * \retval XME_STATUS_OUT_OF_RESOURCES if plug and play manager component initialization failed.
 */ 
xme_status_t
xme_core_pnp_pnpManager_init
(
    void *params
);

/**
 * \brief  Frees all resources occupied by the plug and play manager component.
 *         Exactly one component of this type must be present on every node.
 */
void
xme_core_pnp_pnpManager_fini (void);

/**
 * \brief  Initiates a plug and play event with the goal to install a new
 *         instance of the given component on the given node.
 *
 * \note Currently, no parameterization of the new component is possible.
 *       This will be added in future versions.
 *
 * \param componentType Type of component to instantiate.
 * \param nodeId Unique node identifier of the node where to place the
 *            new component instance.
 * \param existingComponentId If the component id is know by the local node,
 *        this fact indicates that the component is already running. On the 
 *        contrary, XME_CORE_COMPONENT_INVALID_CONTEXT should be provided
 *        for newly instantiated components at runtime (plug-phase). 
 *
 * \retval XME_STATUS_SUCCESS if a new component of the given type has
 *         been successfully installed.
 * \retval XME_STATUS_UNSUPPORTED if the provided component type is
 *         unknown.
 * \retval XME_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_pnp_pnpManager_instantiateComponentOnNode
(
    xme_core_componentType_t componentType,
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t existingComponentId
);

/**
 * \brief Checks if there are new runtime graphs to be sended.
 *
 * \param nodeId the target node id.
 *
 * \retval true if there are more runtime graphs not sended to the target node. 
 * \retval false if there are no runtime graph that should be sent to target node. 
 */
bool
xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode
(
    xme_core_node_nodeId_t nodeId
);

/**
 * \brief Gets the first runtime graph that has not been processed.
 *
 * \param[in] nodeId the target node id.
 * \param[out] outGraph the first runtime graph not processed.
 *
 * \retval XME_STATUS_SUCCESS if the graph is successfully stored in output parameter outGraph. 
 * \retval XME_STATUS_INVALID_PARAMETER if the input parameter is an invalid parameter. 
 * \retval XME_STATUS_INTERNAL_ERROR if the request cannot be processed. 
 */
xme_status_t
xme_core_pnp_pnpManager_getNextRuntimeGraphForNode
(
    xme_core_node_nodeId_t nodeId,
    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph
);

/**
 * \brief Checks if there are new runtime graphs to be sended.
 *
 * \retval true if there are more runtime graphs not sended. 
 * \retval false if there are no runtime graph that should be sent. 
 */
bool
xme_core_pnp_pnpManager_hasNewRuntimeGraphs(void);

/**
 * \brief Gets the first runtime graph that has not been processed.
 *
 * \param[out] outGraph the first runtime graph not processed.
 *
 * \retval XME_STATUS_SUCCESS if the graph is successfully stored in output parameter outGraph. 
 * \retval XME_STATUS_INTERNAL_ERROR if the request cannot be processed. 
 */
xme_status_t
xme_core_pnp_pnpManager_getNextRuntimeGraph
(
    xme_core_topic_pnpManager_runtime_graph_model_t* outGraph
);

/**
 * \brief Updates the instance status in the runtime graph. 
 * 
 * \details The status should be success or non-success. In this
 *          case, the instance is updated to RUNNING or FAILURE. 
 *
 * \param rtGraph The sent runtime graph. 
 * \param status The status returned from the node. 
 *
 * \retval XME_STATUS_SUCCESS if the corresponding instance was
 *         successfully updated in the plug and play manager.
 * \retval XME_STATUS_NOT_FOUND if there are not instances matching
 *         that graph instance. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot update the corresponding
 *         instance in the plug and play manager. 
 */
xme_status_t
xme_core_pnp_pnpManager_updateInstanceFromRuntimeGraph
(
    xme_core_topic_pnpManager_runtime_graph_model_t* rtGraph,
    xme_status_t status
);

/**
 * \brief Registers a node in the PnPManager. 
 * \details These requests are received from the Login Manager.
 *          Only registered nodes are allowed to install components
 *          in the Plug and Play Manager. The registration of the
 *          node enables the announcement of components in the Plug
 *          and Play Manager. 
 *
 * \param nodeId the node id to register. 
 *
 * \retval XME_STATUS_SUCCESS if the node is successfully registered 
 *         in the plug and play manager. 
 * \retval XME_STATUS_INVALID_PARAMETER if the node is an invalid node id. 
 * \retval XME_STATUS_ALREADY_EXISTS if the node is already registered in the node. 
 * \retval XME_STATUS_INTERNAL_ERROR if the data structures for storing the 
 *         node to be registered is not available. 
 */
xme_status_t
xme_core_pnp_pnpManager_registerNode
(
    xme_core_node_nodeId_t nodeId
);

/**
 * \brief Deregisters a node from the PnPManager. 
 * \details The deregistration of the node disables the node to announce
 *          components to be installed in the Plug and Play Manager. 
 *
 * \param nodeId the node id to deregister. 
 *
 * \retval XME_STATUS_SUCCESS if the node is successfully deregistered 
 *         from the plug and play manager. 
 * \retval XME_STATUS_INVALID_PARAMETER if the node is an invalid node id. 
 * \retval XME_STATUS_NOT_FOUND if the node is not registered in the 
 *         Plug and Play Manager. 
 * \retval XME_STATUS_INTERNAL_ERROR if the data structures for storing the 
 *         node to be registered is not available. 
 */
xme_status_t
xme_core_pnp_pnpManager_deregisterNode
(
    xme_core_node_nodeId_t nodeId
);

/**
 * \brief Registers in the Plug and Play Manager the network interface corresponding to a node. 
 * \details This information is needed to configure the corresponding waypoints 
 *          during the generation of physical routes. 
 *
 * \param nodeId the node id. 
 * \param interfaceAddress the public interface address associated to the node id. 
 *
 * \retval XME_STATUS_SUCCESS if the interface address is successfully registered. 
 * \retval XME_STATUS_INVALID_PARAMETER if the some parameter does not contain a valid
 *         value. 
 * \retval XME_STATUS_INTERNAL_ERROR if an error arised during the registration process. 
 */
xme_status_t
xme_core_pnp_pnpManager_registerInterfaceAddress
(
    xme_core_node_nodeId_t nodeId,
    xme_com_interface_address_t interfaceAddress
);

/**
 * \brief Loads a components instance manifest in the plug and play manager. 
 * \details The component instance manifest contains all components already
 *          running in the requesting node. 
 *
 * \param manifest the component instance manifest. 
 *
 * \retval XME_STATUS_SUCCESS if all the component instances are successfully
 *         installed in the PnPManager. 
 * \retval XME_STATUS_INVALID_PARAMETER if the manifest content does not store
 *         correct data. This should be either due to incorrect component types.
 * \retval XME_STATUS_NOT_FOUND if the node associated to the component instance
 *         manifest is not registered in the plug and play manager. 
 * \retval XME_STATUS_INTERNAL_ERROR if an error arised during the announcement
 *         of the component instances. 
 */
xme_status_t
xme_core_pnp_pnpManager_announceComponentInstanceManifest
(
    xme_core_topic_pnp_componentInstanceManifest_t* manifest
);

/**
 * \brief Sets as established in the logical route manager all unestablished logical routes. 
 *
 * \details Sets all logical routes stored as unestablished in the unestablishedLogicalRoutes
 *          table as established in the logical route manager.
 * \note The logical route table can be emptied only when all runtime graphs are delivered
 *       to respective nodes. This function will act as a mechanism to empty this table
 *       once after all runtime graphs are consumed. This method will be called at every cycle,
 *       but only it will set the established flag for the first call after runtime graph
 *       consumption. 
 *
 * \retval XME_STATUS_SUCCESS if all logical routes are set as established. 
 * \retval XME_STATUS_INTERNAL_ERROR if operation could not be completed.
 */
xme_status_t
xme_core_pnp_pnpManager_setAllLogicalRoutesAsEstablished(void);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_PLUGANDPLAYMANAGER_H
