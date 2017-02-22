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
 * $Id: randomStringGenerator.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         Random string generator.
 *
 */

#ifndef XME_PRIM_RANDOMSTRINGGENERATOR_H
#define XME_PRIM_RANDOMSTRINGGENERATOR_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/resourceManager.h"

#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_prim_randomStringGenerator_configStruct_t
 *
 * \brief  Random string generator configuration structure.
 */
typedef struct
{
	// public
	xme_hal_time_timeInterval_t interval; ///< the time interval. 
	// private
	xme_core_dcc_publicationHandle_t publicationHandle; ///< the publication handle. 
	xme_core_resourceManager_taskHandle_t taskHandle; ///< the task handle. 
}
xme_prim_randomStringGenerator_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Creates a random string generator component.
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully created.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be created.
 */
xme_status_t
xme_prim_randomStringGenerator_create(xme_prim_randomStringGenerator_configStruct_t* config);

/**
 * \brief  Activates a random string generator component.
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be activated.
 */
xme_status_t
xme_prim_randomStringGenerator_activate(xme_prim_randomStringGenerator_configStruct_t* config);

/**
 * \brief  Deactivates a random string generator component.
 * \param  config the configuration parameters.
 */
void
xme_prim_randomStringGenerator_deactivate(xme_prim_randomStringGenerator_configStruct_t* config);

/**
 * \brief  Destroys a random string generator component.
 * \param  config the configuration parameters.
 */
void
xme_prim_randomStringGenerator_destroy(xme_prim_randomStringGenerator_configStruct_t* config);

#endif // #ifndef XME_PRIM_RANDOMSTRINGGENERATOR_H
