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
 * $Id: voter.h 3546 2013-05-28 18:56:57Z geisinger $
 */

/**
 * \file
 * \brief  Voter component. 
 *
 * \details (Proof of concept - voters very application specific
 *         and should be generated. The following aspects have to be taken into
 *         account: voting algorithm, data type, timeout, trigger-event, input
 *         data structure, mapping of input data to node ids, ...)
 */

#ifndef XME_ADV_VOTER_H
#define XME_ADV_VOTER_H

/**
 * \defgroup adv_voter Voter Component
 * @{
 *
 * \brief This component implements the voting component data fusion needs to 
 *        take place. This should include voting algorithms, data types, timeouts, 
 *        trigger-event, input data structure, mapping of input data to node ids, ... 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/healthmonitor.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/** 
 * \enum xme_adv_voter_algorithm_t
 * \brief The enumeration structure for defining algorithms to be used in test 
 */
typedef enum
{
	XME_ADV_VOTER_ALGORITHM_MEDIAN = 0, ///< the voter will use the median as voting algorithm.
	XME_ADV_VOTER_ALGORITHM_MEAN ///< the voter will use the mean as voting algorithm.
}
xme_adv_voter_algorithm_t;

/** 
 * \enum xme_adv_voter_dataType_t
 * \brief The enumeration structure for defining datatype to be used for the voting 
 */
typedef enum
{
	XME_ADV_VOTER_DATATYPE_INT = 0, ///< the voter datatype is int.
	XME_ADV_VOTER_DATATYPE_BINARY  ///< the voter datatype is binary.
}
xme_adv_voter_dataType_t;

/** 
 * \struct xme_adv_voter_votableData_t
 * \brief Data structure to store votable data
 */
typedef struct
{
	xme_adv_voter_dataType_t type; ///< voter data type.
	xme_core_node_nodeId_t sourceNode; ///< source note.
	int sizeOfBinaryData; ///< size of binary data.
	void *data; ///< pointer to the data.
}
xme_adv_voter_votableData_t;

/** 
 * \struct xme_adv_voter_configStruct_t
 * \brief Configuration structure for voting
 */
typedef struct
{
	xme_core_topic_t inputTopic; ///< input topic.
	xme_core_topic_t outputTopic; ///< output topic.
	xme_adv_voter_algorithm_t algorithm; ///< voting algorithm.
	xme_adv_voter_dataType_t dataType; ///< voting datatype.
	xme_hal_time_timeInterval_t startTime; ///< voting start time.
	xme_hal_time_timeInterval_t period; ///< voting period.
	xme_core_dcc_subscriptionHandle_t subHandle; ///< collect data
	xme_core_dcc_publicationHandle_t pubHandle; ///< voted results
	xme_core_dcc_publicationHandle_t errorHandle; ///< error messages
}
xme_adv_voter_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates a voter component.
 * 
 * \param config the voting configuration structure
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully initialized.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be initialized. 
 */
xme_status_t
xme_adv_voter_create
(
	xme_adv_voter_configStruct_t* config
);


/**
 * \brief  Activates a voter component.
 * 
 * \param config the voting configuration structure
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be activated. 
 */
xme_status_t
xme_adv_voter_activate
(
	xme_adv_voter_configStruct_t* config
);


/**
 * \brief  Deactivates a voter component.
 * 
 * \param config the voting configuration structure
 */
void
xme_adv_voter_deactivate
(
	xme_adv_voter_configStruct_t* config
);

/**
 * \brief  Destroys a voter component.
 * 
 * \param config the memory testing configuration structure
 */
void
xme_adv_voter_destroy
(
	xme_adv_voter_configStruct_t* config
);

/**
 * \brief calculates the median of the received values and publishes it.
 *
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 *
 * \return the median value of voting data included as incoming parameter
 */
int
xme_adv_voter_medianInt
(
	void* userData
);

/**
 * \brief  calculates the mean of the received values.
 *
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 *
 * \return the mean value of voting data included as incoming parameter
 */
int
xme_adv_voter_meanInt
(
	void* userData
);

/**
 * \brief  votes the received values and publishes the result.
 *
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_voter_vote
(
	void* userData
);

/**
 * \brief  stores received data.
 *
 * \param  dataHandle the data handle
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_voter_receiveValue
(
	xme_hal_sharedPtr_t dataHandle, 
	void* userData
);

/**
 * \brief // TODO: delete
 */
void 
xme_adv_voter_debugTableSort(void); // TODO: delete

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_ADV_VOTER_H
