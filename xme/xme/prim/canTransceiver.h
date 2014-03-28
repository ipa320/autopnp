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
 * $Id: canTransceiver.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         CAN transceiver.
 *
 */

#ifndef XME_PRIM_CANTRANSCEIVER_H
#define XME_PRIM_CANTRANSCEIVER_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"
#include "xme/hal/include/can.h"
#include "xme/hal/include/sched.h"
#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_prim_canTransceiver_configStruct_t
 *
 * \brief  Configuration of a canTransceiver.
 */
typedef struct
{
	xme_hal_can_port_t port; ///< the can port.
	xme_hal_can_speed_t speed; ///< the can speed. 
	xme_core_topic_t topicCANSendRaw; ///< the topic can send raw. 
	xme_core_topic_t topicCANReceiveRaw; ///< the topic can receive raw. 
	xme_core_topic_t topicCANError; ///< the topic can error
	xme_hal_time_timeInterval_t periodOfWorkingTask; ///< the period of working task. 

	// Private
	xme_core_dcc_subscriptionHandle_t handleSubCANSendRaw; ///< the subscription handle. 
	xme_core_dcc_publicationHandle_t handlePubCANReceiveRaw; ///< the publication handle. 
	xme_core_dcc_publicationHandle_t handlePubCANError; ///< the publication handle for errors. 
	xme_hal_sched_taskHandle_t handleWorkingTask; ///< the task handle for working task. 

	bool isActivated; ///< boolean value determining is can is activated. 

}
xme_prim_canTransceiver_configStruct_t;

/**
 * \struct xme_prim_canTransceiver_can_message_t
 *
 * \brief  The CAN message.
 */
typedef struct
{
	uint32_t msgID; ///< Identifier of message. Might be 11 or 29 bit.
	uint8_t data[8]; ///< Payload.
	uint8_t msgLen; ///< Number of valid bytes in payload.
} xme_prim_canTransceiver_can_message_t;

/**
 * \enum xme_prim_canTransceiver_can_error_ids_t
 *
 * \brief  The CAN message errors.
 */
typedef enum
{

	XME_CAN_TRANSCEIVER_ERROR_RX_BUFFER_OVERRUN, ///> Message could not be received.
	XME_CAN_TRANSCEIVER_ERROR_TX_BUFFER_OVERRUN ///> Message could not be transmitted.
}
xme_prim_canTransceiver_can_error_ids_t;

/**
 * \struct xme_prim_canTransceiver_can_error_t
 *
 * \brief  The storage of CAN errors.
 */
typedef struct
{
	xme_prim_canTransceiver_can_error_ids_t id; ///< the error id. 
} xme_prim_canTransceiver_can_error_t;


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Create the CAN transceiver.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully created.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be created.
 */
xme_status_t
xme_prim_canTransceiver_create
(
	xme_prim_canTransceiver_configStruct_t* config
);

/**
 * \brief  Activate the CAN transceiver.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be activated.
 */
xme_status_t
xme_prim_canTransceiver_activate
(
	xme_prim_canTransceiver_configStruct_t* config
);

/**
 * \brief  Deactivate the CAN transceiver.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully deactivated.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be deactivated.
 */

xme_status_t
xme_prim_canTransceiver_deactivate
(
	xme_prim_canTransceiver_configStruct_t* config
);

/**
 * \brief  Destroy the CAN transceiver.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully destroyed.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be destroyed.
 */

xme_status_t
xme_prim_canTransceiver_destroy
(
	xme_prim_canTransceiver_configStruct_t* config
);

#endif // #ifndef XME_PRIM_CANTRANSCEIVER_H
