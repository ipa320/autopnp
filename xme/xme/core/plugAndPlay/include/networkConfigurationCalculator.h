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
 * $Id: networkConfigurationCalculator.h 4664 2013-08-13 09:06:08Z ruiz $
 */

/**
 * \file
 *         Network Configuration Calculator.
 */

#ifndef XME_CORE_PNP_NETWORKCONFIGURATIONCALCULATOR_H
#define XME_CORE_PNP_NETWORKCONFIGURATIONCALCULATOR_H

/**
 * \defgroup core_pnp_ncc Network Configurator Calculator group. 
 * @{
 *
 * \brief Network Configuration Calculator calculates the physical route based
*         on provided logical routes. 
 *
 * \details Network Configuration Calculator provides the calculus to establish the
 *          physical route for a given logical route. This component will include
 *          configured waypoints to connect remote components, or corresponding
 *          memory copies to components placed in the same node.  
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include "xme/hal/include/graph.h"
#include "xme/hal/include/table.h"
#include "xme/core/node.h"
#include "xme/core/topic.h"
#include "xme/core/plugAndPlay/include/logicalRouteManager.h"
#include "xme/core/directory/include/nodeRegistryController.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_core_pnp_ncc_physicalRoutes_t
 * \brief  Directed graph containing physical route candidates output by the
 *         Network Configuration Calculator.
 */
typedef xme_hal_graph_graph_t xme_core_pnp_ncc_physicalRoutes_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Calculates a logical representation of the required waypoints for
 *         realizing all the non-established logical routes in the given
 *         data link graph.
 *
 * \details This function also adds the required configurations for the
 *          waypoints. It is meant to be called by the Plug & Play Manager.
 *
 * \param  logicalRoutesGraph Pointer to the data link graph, typically obtained
 *         from Logical Route Manager.
 * \param  physicalRouteGraph Pointer to the deployment graph that includes the
 *         waypoints to physically instantiate a data route.
 *
 * \retval XME_STATUS_SUCCESS if waypoint transformation completed successfully.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the parameters was NULL or
 *         inappropriate.
 */
xme_status_t
xme_core_pnp_ncc_getPhysicalRoutes
(
    xme_hal_graph_graph_t *logicalRoutesGraph,
    xme_core_pnp_ncc_physicalRoutes_t *physicalRouteGraph
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_NETWORKCONFIGURATIONCALCULATOR_H
