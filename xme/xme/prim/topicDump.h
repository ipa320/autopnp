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
 * $Id: topicDump.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         Topic dump component.
 *         Subscribes to a (configurable) topic, creates a dump of the received
 *         topic data and creates a log for the dump.
 *
 */

#ifndef XME_PRIM_TOPICDUMP_H
#define XME_PRIM_TOPICDUMP_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_prim_topicDump_configStruct_t
 *
 * \brief  Console logger configuration structure.
 */
typedef struct
{
	// public
	xme_core_topic_t subscribedTopic; ///< the subscribed topic. 
	// private
	xme_core_dcc_subscriptionHandle_t subscriptionHandle; ///< the subscription handle. 
}
xme_prim_topicDump_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Creates a topic dump component.
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully created.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be created.
 */
xme_status_t
xme_prim_topicDump_create(xme_prim_topicDump_configStruct_t* config);

/**
 * \brief  Activates a topic dump component.
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be activated.
 */
xme_status_t
xme_prim_topicDump_activate(xme_prim_topicDump_configStruct_t* config);

/**
 * \brief  Deactivates a topic dump component.
 * \param  config the configuration parameters.
 */
void
xme_prim_topicDump_deactivate(xme_prim_topicDump_configStruct_t* config);

/**
 * \brief  Destroys a topic dump component.
 * \param  config the configuration parameters.
 */
void
xme_prim_topicDump_destroy(xme_prim_topicDump_configStruct_t* config);

#endif // #ifndef XME_PRIM_TOPICDUMP_H
