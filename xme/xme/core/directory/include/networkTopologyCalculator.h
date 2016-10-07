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
 * $Id: networkTopologyCalculator.h 5643 2013-10-25 14:58:10Z ruiz $
 */

/**
 * \file
 *         Network Topology Calculator.
 */

#ifndef XME_CORE_DIRECTORY_NETWORKTOPOLOGYCALCULATOR_H
#define XME_CORE_DIRECTORY_NETWORKTOPOLOGYCALCULATOR_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/node.h"
#include "xme/core/directory/include/nodeRegistryController.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/linkedList.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_core_directory_networkTopologyCalculator_neighbors_t
 *
 * \brief This structure contains the neighborhood relation between two nodes.
          It contains the node ID of the sending node, the interface ID of the
          sending node, the node ID of the receiving node and the interfaceID
          of the receiving node.
 */
typedef struct
{
    xme_core_node_nodeId_t senderNodeId; ///< the node identifier of the sending node.
    xme_com_interface_address_t senderInterfaceId; ///< the interface identifier of the sending node.
    xme_core_node_nodeId_t receiverNodeId; ///< the node identifier of the receiving node.
    xme_com_interface_address_t receiverInterfaceId; ///< the interface identifier of the receiving node.
}
xme_core_directory_networkTopologyCalculator_neighbors_t;

/**
 * \struct xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t
 *
 * \brief This structure contains all the information aquired by each node for building
          up a network topology graph.
 */
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< the node identifier.
    xme_core_directory_networkTopologyCalculator_neighbors_t* neighbors; ///< the list of the neighbors of the node.
    char overflow; ///< Overflow indicator. Is true if problems occured during the storage of neighborhood information.
    uint16_t neighborsCount; ///< The amount of neighbors contained in neighborhood update message.
}
xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t;

/**
 * \struct xme_core_directory_networkTopologyCalculator_vertexData_t
 * \brief the data associated to a vertex in the network graph.
 */
typedef struct
{
    xme_core_node_nodeId_t nodeId; ///< Node ID of the node represented by the current vertex.
    int8_t lastSeenBeforeTicks; ///< Entry will be removed if lastSeenBeforeTicks becomes zero.
}
xme_core_directory_networkTopologyCalculator_vertexData_t;

/**
 * \struct xme_core_directory_networkTopologyCalculator_edgeData_t
 * \brief the data associated to an edge in the network graph.
 */
typedef struct
{
    xme_com_interface_address_t senderInterfaceId; ///< the interface identifier of the sending node.
    xme_com_interface_address_t receiverInterfaceId; ///< the interface identifier of the receiving node.
}
xme_core_directory_networkTopologyCalculator_edgeData_t;

/**
 * \brief Defines the number of steps we wait before we delete nodes.
 */
//TODO: What is a good number?
#define NUMBER_OF_TICKS 10

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initialize the network topology calculator.
 *
 * \param initStruct the initialization structure. 
 *
 * \retval XME_CORE_STATUS_SUCCESS on success.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_directory_networkTopologyCalculator_init
(
    void* initStruct
);

/**
 * \brief  Stop the network topoplogy calculator.
 *
 * \retval XME_CORE_STATUS_SUCCESS on success.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR on error.
 */
void
xme_core_directory_networkTopologyCalculator_fini(void);

/**
 * \brief  Evaluate the neighborhood information and build up the network graph.
 *
 * \param update the updated information to update the network graph. 
 *
 * \retval XME_CORE_STATUS_SUCCESS on success.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR on error.
 */
xme_status_t
xme_core_directory_networkTopologyCalculator_updateNetwork
(
    const xme_core_directory_networkTopologyCalculator_neighborhoodInformation_t* update
);

/**
 * \brief  Get the network graph.
 *
 * \return Returns the current network graph
 */
xme_hal_graph_graph_t
xme_core_directory_networkTopologyCalculator_getNetworkGraph(void);


XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DIRECTORY_NETWORKTOPOLOGYCALCULATOR_H
