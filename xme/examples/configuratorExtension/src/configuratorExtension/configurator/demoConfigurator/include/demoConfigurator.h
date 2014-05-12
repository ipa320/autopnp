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
 * $Id: demoConfigurator.h 6367 2014-01-16 15:50:31Z geisinger $
 */

/**
 * \file
 *         Sample logical route configurator.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/configuratorExtension.h"

/******************************************************************************/
/***   Prototype                                                            ***/
/******************************************************************************/
/**
 * \brief Implementation of an xme_core_pnp_configExt_configuratorCallback_t.
 *
 * \details This configurator performs the following modifications:
 *          1) When the lower connection bound of an input port is not met,
 *             it will remove all logical routes (which will prevent
 *             component initialization.
 *          2) When the upper connection bound of an input port is not met:
 *             2.1) When the port topic has the 'quality' attribute (key: CONFIGURATOREXTENSION_ATTRIBUTE_QUALITY):
 *                  Sort routes by quality. Remove routes until upper connection
 *                  bound is met (beginning with lowest quality).
 *             2.2) When the port topic does NOT have the 'quality' attribute (key: CONFIGURATOREXTENSION_ATTRIBUTE_QUALITY):
 *                  Randomly remove routes until upper connection bound is met.
 *
 * \warning Currently configurator extensions are not able to remove already
 *          established routes (Issue #3952).
 */
void
configuratorExtension_configurator_demoConfigurator_callback
(
    xme_hal_graph_graph_t* const logicalRouteGraph
);
