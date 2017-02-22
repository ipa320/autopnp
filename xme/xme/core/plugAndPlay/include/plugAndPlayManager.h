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
 * $Id: plugAndPlayManager.h 7703 2014-03-06 23:58:40Z ruiz $
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
#include "xme/core/nodeManager/include/componentRepository.h"
#include "xme/core/node.h"
#include "xme/core/topicData.h"

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
 * \brief Frees all resources occupied by the plug and play manager component.
 *        Exactly one component of this type must be present on every node.
 */
void
xme_core_pnp_pnpManager_fini(void);

/**
 * \brief Trigger update of configuration.
 *
 * \details This will trigger recalculation of logical route graphs and will call
 *          all configuration extensions (see xme_core_pnp_configuratorExtension).
 *          For any new unestablished logical routes, updates for the affected
 *          nodes will be scheduled and processed later by the plug and play manager
 *          generate runtime graphs function.
 *
 * \retval XME_STATUS_SUCCESS if the operation was successful.
 */
xme_status_t
xme_core_pnp_pnpManager_updateConfiguration(void);

/**
 * \brief Initiates a plug and play event with the goal to install a new
 *        instance of the given component on the given node.
 *
 * \deprecated Use xme_core_pnp_pnpManager_plugInNewComponent() instead.
 *
 * \param[in] componentType Type of component to instantiate.
 * \param[in] initializationString Component-specific initialization data
 *            to pass to the new component instance. May be NULL.
 * \param[in] nodeID Unique node identifier of the node where to place the
 *            new component instance.
 * \param[in] existingComponentID Specifies the unique component identifier
 *            of the instantgiated component. If this parameter is not set to
 *            XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT, the the component
 *            is assumed to be already existing. If an invalid component
 *            context is passed, the new component is actually instantiated
 *            and a unique identifier assigned.
 *
 * \retval XME_STATUS_SUCCESS if a new component of the given type has
 *         been successfully installed or the fact that the component is
 *         already running has been accepted.
 * \retval XME_STATUS_UNSUPPORTED if the provided component type is
 *         unknown.
 * \retval XME_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_pnp_pnpManager_instantiateComponentOnNode
(
    xme_core_componentType_t componentType,
    const char* const initializationString,
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t existingComponentID
);

/**
 * \brief Plug in a new component.
 *
 * \details To create the component use a component builder, see
 *          xme_core_nodeMgr_compRep_createBuilder().
 *          This function will change the component state to
 *          XME_CORE_NODEMGR_COMPREP_STATE_ANNOUNCED and the component will be
 *          created as soon as its connection constraints are met.
 *
 * \param[in] componentHandle Handle of a component in state
 *            XME_CORE_NODEMGR_COMPREP_STATE_PREPARED.
 *
 * \retval XME_STATUS_SUCCESS if operation was successful.
 * \retval XME_STATUS_INVALID_PARAMETER if given component is in an invalid state.
 * \retval XME_STATUS_INTERNAL_ERROR on any other error.
 */
xme_status_t
xme_core_pnp_pnpManager_plugInNewComponent
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
);

/**
 * \brief Announce a new component instance.
 *
 * \details If the given component has no component ID yet, the
 *          manager will assign a new one.
 *          Announces the given component to the logical route manager.
 *          Does not trigger a configuration update.
 *          Sets the state of the given component to ANNOUNCED.
 *
 * \param[in] componentHandle Given component handle. The component must be in state
 *            PREPARED.
 *
 * \retval XME_STATUS_INVALID_CONFIGURATION if the component type cannot be
 *         found in the manifest repository, or when component is in invalid
 *         state.
 * \retval XME_STATUS_INVALID_PARAMETER if componentType is
 *         XME_CORE_COMPONENT_TYPE_INVALID or nodeId is
 *         XME_CORE_NODE_INVALID_NODE_ID.
 * \retval XME_STATUS_INTERNAL_ERROR on any other error.
 */
xme_status_t
xme_core_pnp_pnpManager_announceNewComponentOnNode
(
    xme_core_nodeMgr_compRep_componentHandle_t componentHandle
);

/**
 * \brief Removes a given component from announcements.
 *
 * \details This function will remove all announcements associated to a given
 *          component and calculates routes to remove. This will be stored in a temporary
 *          variable that will be later checked using ::xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(),
 *          and ::xme_core_pnp_pnpManager_getNextRuntimeGraphForNode().
 *
 * \param[in] nodeID The Node ID of the node where the component should be unannounced.
 * \param[in] componentID The component identifier to remove.
 *
 * \retval XME_STATUS_SUCCESS if the function successfully unannounced the component from
 *         Logical Route Manager, and obtained the logical routes to be removed. 
 * \retval XME_STATUS_INVALID_PARAMETER If the provided input parameters have invalid values. 
 * \retval XME_STATUS_NOT_FOUND if there are no such component stated as running in instance list. 
 * \retval XME_STATUS_INTERNAL_ERROR on any other error.
 */
xme_status_t
xme_core_pnp_pnpManager_unannounceComponentOnNode
(
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID
);

/**
 * \brief Removes the provided node from logical routes.
 *
 * \details This function will remove all announcements associated to a given
 *          node and calculates routes to remove. This will be stored in a temporary
 *          variable that will be later checked using ::xme_core_pnp_pnpManager_hasNewRuntimeGraphsForNode(),
 *          and ::xme_core_pnp_pnpManager_getNextRuntimeGraphForNode().
 *
 * \param[in] nodeID The Node ID to be unannounced.
 *
 * \retval XME_STATUS_SUCCESS if the function successfully unannounced all components associated to 
 *         the provided node ID from Logical Route Manager, and obtained the logical routes to be removed. 
 * \retval XME_STATUS_INVALID_PARAMETER If the provided input parameter has an invalid value. 
 * \retval XME_STATUS_NOT_FOUND if there are no such node in instance list. 
 * \retval XME_STATUS_INTERNAL_ERROR on any other error.
 */
xme_status_t
xme_core_pnp_pnpManager_unannounceNode
(
    xme_core_node_nodeId_t nodeID
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

/**
 * \brief Removes all the instances associated to the node. 
 *
 * \param[in] nodeID The node identifier. 
 *
 * \retval XME_STATUS_SUCCESS if all the instances associate to the node were removed. 
 * \retval XME_STATUS_NOT_FOUND if cannot find any instance associated to that node. 
 * \retval XME_STATUS_INTERNAL_ERROR if operation could not be completed.
 */
xme_status_t
xme_core_pnp_pnpManager_removeNodeInstances
(
    xme_core_node_nodeId_t nodeID
);

/**
 * \brief Removes a single component instance from the component repository. 
 *
 * \param[in] nodeID The node identifier. 
 * \param[in] componentID The component identifier. 
 *
 * \retval XME_STATUS_SUCCESS if the component instance was successfully removed from
 *         component repository. 
 * \retval XME_STATUS_NOT_FOUND If there is not matching component instance for the provided parameters.
 */
xme_status_t
xme_core_pnp_pnpManager_removeComponentInstance
(
    xme_core_node_nodeId_t nodeID,
    xme_core_component_t componentID
);

/**
 * \brief Generates a new channel identifier for the given node. 
 * \details The newly generated channel is globally unique for the communication
 *          between provided node and the plug and play manager (to receive the 
 *          component instance manifest). 
 * \note As there is only one single plug and play client in every node, the
 *       channel identifier is unique for that communication. The communication refers
 *       to the communication lifecycle, so every time a node request to login,
 *       this is interpreted internally as a new communication channel. 
 *
 * \param[in] nodeID the requesting node identifier. 
 * \param[out] channelID the generated channel identifier. 
 *
 * \return XME_STATUS_SUCCESS if the channel identifier is successfully generated. 
 * \return XME_STATUS_INVALID_PARAMETER if the provided input parameters are incorrect. 
 * \return XME_STATUS_INTERNAL_ERROR if cannot generated a channel identifier for the provided node. 
 */
xme_status_t
xme_core_pnp_pnpManager_generateChannelIDForManifestReception
(
    xme_core_node_nodeId_t nodeID,
    xme_core_channelId_t* channelID
);

/**
 * \brief This function announces the channel IDs for the given set of publisher
 *        and subscriber ports.
 *
 * \details This does not create a channel.
 *          Whenever a channel is created the announced channel IDs will be checked.
 *          When a match for the source and destination ports of the new channel is
 *          found the previously announced channelID will be used.
 *
 * \param[in] publicationNodeID Node ID of the publisher port.
 * \param[in] publicationComponentID component ID for the publisher port.
 * \param[in] publicationPortIndex Portindex of the publisher port for that component.
 * \param[in] subscriptionNodeID Node ID of the subscriber port.
 * \param[in] subscriptionComponentID component ID for the subscriber port.
 * \param[in] subscriptionPortIndex Portindex of the subscriber port for that component.
 * \param[in] channelId The channel identifier.
 *
 * \retval XME_STATUS_SUCCESS if the channelID is successfully announced.
 * \retval XME_STATUS_OUT_OF_RESOURCES if not enough resources were available
 *         to announce the channelID.
 */
xme_status_t
xme_core_pnp_pnpManager_announceChannelId
(
    xme_core_node_nodeId_t publicationNodeID,
    xme_core_component_t publicationComponentID,
    uint16_t publicationPortIndex,
    xme_core_node_nodeId_t subscriptionNodeID,
    xme_core_component_t subscriptionComponentID,
    uint16_t subscriptionPortIndex,
    xme_core_channelId_t channelId
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_PLUGANDPLAYMANAGER_H
