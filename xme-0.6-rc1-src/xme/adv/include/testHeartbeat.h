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
 * $Id: testHeartbeat.h 3546 2013-05-28 18:56:57Z geisinger $
 */

/**
 * \file
 * \brief Health monitor component.
 */

#ifndef xme_adv_testHbeat_H
#define xme_adv_testHbeat_H

/**
 * \defgroup adv_testHeartbeat Heartbeat Component Testing.
 * @{
 *
 * \brief This component test heartbeat component functions and structures. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/healthmonitor.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/** 
 * \struct xme_adv_testHbeat_configStruct_t
 * \brief Configuration structure for testing heartbeat component.
 */
typedef struct
{
	// public
	xme_hal_time_handle_t maxTimestampAge; ///< Maximum timestamp for testing heartbeat.
	bool catchAllHeartBeats; ///< A boolean value for receiving all heartbeats (T) or none (F).
	xme_status_t (*callback)(); ///< instant error reaction function.
	// private
	xme_core_dcc_subscriptionHandle_t subHandle; ///< internal heartbeat subscription handle.
	xme_core_dcc_publicationHandle_t pubHandle; ///< internal publication handle for error messages.
}
xme_adv_testHbeat_configStruct_t;

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/** 
 * \def XME_ADV_TESTHBEAT_HEARTBEATS_MAX 
 * \brief Maximum number of heartbeats to be produced .
 */
#define XME_ADV_TESTHBEAT_HEARTBEATS_MAX 50

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates a heartbeat testing component.
 *
 * \param config the health monitor testing configuration structure.
 * 
 * \retval XME_CORE_STATUS_SUCCESS if the heartbeat testing component has been successfully created.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the heartbeat testing component cannot be created.
 */
xme_status_t
xme_adv_testHbeat_create
(
	xme_adv_testHbeat_configStruct_t* config
);


/**
 * \brief  Activates a heartbeat testing component.
 * 
 * \param config the health monitor testing configuration structure.
 * 
 * \retval XME_CORE_STATUS_SUCCESS if the heartbeat testing component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the heartbeat testing component cannot be activated due to an error during activation.
 */
xme_status_t
xme_adv_testHbeat_activate
(
	xme_adv_testHbeat_configStruct_t* config
);


/**
 * \brief  Deactivates a heartbeat testing component.
 *
 * \param config the healthbeat testing configuration structure.
 */
void
xme_adv_testHbeat_deactivate
(
	xme_adv_testHbeat_configStruct_t* config
);


/**
 * \brief  Destroys a heartbeat testing component.
 * 
 * \param config the heartbeat testing configuration structure.
 */
void
xme_adv_testHbeat_destroy(xme_adv_testHbeat_configStruct_t* config);

/**
 * \brief  Callback function that sends the sends the local node id.
 *
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_testHbeat_callback
(
	void* userData
);

/**
 * \brief  Callback function that received a single heartbeat node.
 *
 * \param dataHandle the data handler subscription to heartbeat.
 *
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_testHbeat_receiveHeartbeatSingleNode
(
	xme_hal_sharedPtr_t dataHandle, 
	void* userData
);

/**
 * \brief  Callback function that received multiple heartbeats from connected node.
 *
 * \param dataHandle the data handler subscription to heartbeat.
 *
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_testHbeat_receiveHeartbeatMultiNode
(
	xme_hal_sharedPtr_t dataHandle, 
	void* userData
);


/**
 * \brief  Function that sets the node ids, which should be monitored. Project specific!
 */
void 
xme_adv_testHbeat_initHeartbeatTable(void);

/**
 * \brief Debug function, printing the timestamp table.
 */
void 
xme_adv_testHbeat_debugHeartbeatTable(void);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef xme_adv_testHbeat_H
