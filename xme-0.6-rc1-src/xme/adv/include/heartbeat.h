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
 * $Id: heartbeat.h 2693 2013-03-17 01:51:56Z camek $
 */

/**
 * \file 
 * \brief Heartbeat sender component.
 */

#ifndef XME_ADV_HEARTBEAT_H
#define XME_ADV_HEARTBEAT_H

/**
 * \defgroup adv_heartbeat Hearthbeat Component.
 * @{
 *
 * \brief This component provides activates the subscription to heartbeat messages
 *        and receives the node status. This component is used by the health monitor
 *        component to check the system health. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/adv/include/healthmonitor.h"

#include "xme/hal/include/sched.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/** 
 * \struct xme_adv_hbeat_configStruct_t
 * \brief Configuration structure for heartbeat.
 */
typedef struct
{
	// public
	xme_hal_time_timeInterval_t interval; ///< Frequency of heartbeating.
	// private
	xme_core_dcc_publicationHandle_t pubHandle; ///< Subscription handle.
	xme_hal_sched_taskHandle_t taskHandle;      ///< Task handle.
}
xme_adv_hbeat_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates a heartbeat component.
 *
 * \param config the heartbeat configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully created.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be created.
 */
xme_status_t
xme_adv_hbeat_create
(
	xme_adv_hbeat_configStruct_t* config
);


/**
 * \brief  Activates a heartbeat component.
 * 
 * \param config the heartbeat configuration structure.
 * 
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be activated due to an error during activation.
 */
xme_status_t
xme_adv_hbeat_activate
(
	xme_adv_hbeat_configStruct_t* config
);


/**
 * \brief  Deactivates a heartbeat component.
 * 
 * \param config the heartbeat configuration structure.
 */
void
xme_adv_hbeat_deactivate
(
	xme_adv_hbeat_configStruct_t* config
);

/**
 * \brief  Destroys a heartbeat component.
 * 
 * \param config the heartbeat configuration structure.
 */
void
xme_adv_hbeat_destroy
(
	xme_adv_hbeat_configStruct_t* config
);

/**
 * \brief  Callback function that sends the local node id.
 * 
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_hbeat_componentCallback
(
	void* userData
);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_ADV_HEARTBEAT_H
