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
 * $Id: randomNumberGenerator.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         Random number generator.
 *
 */

#ifndef XME_PRIM_RANDOMNUMBERGENERATOR_H
#define XME_PRIM_RANDOMNUMBERGENERATOR_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/resourceManager.h"

#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_prim_randomNumberGenerator_configStruct_t
 *
 * \brief  Random number generator configuration structure.
 */
typedef struct
{
	// public
	int minValue; ///< the minimum value. 
	int maxValue; ///< the maximum value. 
	xme_hal_time_timeInterval_t interval; ///< the time interval. 
	// private
	xme_core_dcc_publicationHandle_t publicationHandle; ///< the publication handle. 
	xme_core_resourceManager_taskHandle_t taskHandle; ///< the task handle. 
}
xme_prim_randomNumberGenerator_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Creates a random number generator component.
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully created.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be created.
 */
xme_status_t
xme_prim_randomNumberGenerator_create(xme_prim_randomNumberGenerator_configStruct_t* config);

/**
 * \brief  Activates a random number generator component.
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be activated.
 */
xme_status_t
xme_prim_randomNumberGenerator_activate(xme_prim_randomNumberGenerator_configStruct_t* config);

/**
 * \brief  Deactivates a random number generator component.
 * \param  config the configuration parameters.
 */
void
xme_prim_randomNumberGenerator_deactivate(xme_prim_randomNumberGenerator_configStruct_t* config);

/**
 * \brief  Destroys a random number generator component.
 * \param  config the configuration parameters.
 */
void
xme_prim_randomNumberGenerator_destroy(xme_prim_randomNumberGenerator_configStruct_t* config);

#endif // #ifndef XME_PRIM_RANDOMNUMBERGENERATOR_H
