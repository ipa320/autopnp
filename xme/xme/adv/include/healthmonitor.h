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
 * $Id: healthmonitor.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *
 * \brief Health monitor component.
 */

#ifndef XME_ADV_HEALTHMONITOR_H
#define XME_ADV_HEALTHMONITOR_H

/**
 * \defgroup adv_healthmonitor Health Monitoring Component
 * @{
 *
 * \brief This component creates and tracks monitoring observer of components,
 *        nodes, and checks at runtime.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/broker/include/broker.h"
#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_adv_hmon_configStruct_t
 * \brief Configuration structure for health monitoring
 */
typedef struct {
	// public
	xme_core_node_nodeId_t nodeId; ///< the nodeId.
	xme_core_component_t componentId; ///< the componentId. set to -1 to monitor complete node.
	xme_core_dcc_subscriptionHandle_t subHandleErrors; ///< handler for error messages in subscriptions.
	xme_core_dcc_publicationHandle_t pubHandleErrors; ///< handler for error messages in publications.
	int monitorComponent; ///< monitor component. // TODO: instead of node?
}
xme_adv_hmon_configStruct_t;

/**
 * \enum xme_adv_hmon_defines_t
 * \brief The enumeration structure for defining health monitoring errors.
 */
typedef enum {
	XME_ADV_HMON_ERRORS_MAX = 5 ///< max number of errors.
}
xme_adv_hmon_defines_t;

/**
 * \enum xme_adv_hmon_status_t
 * \brief Enumeration type definition for health monitoring status.
 */
typedef enum
{
	XME_ADV_HMON_STATUS_COMPONENT_OK = 0, ///< the component is ok.
	XME_ADV_HMON_STATUS_COMPONENT_EXCEPTION, ///< the component throw an exception.
	XME_ADV_HMON_STATUS_COMPONENT_UNKNOWN, ///< the component is unknown.
	XME_ADV_HMON_STATUS_TEST_OK, ///< the test is success.
	XME_ADV_HMON_STATUS_TEST_UNKNOWN, ///< the test is unknown.
	XME_ADV_HMON_STATUS_TEST_FAILED, ///< the test failed.
	XME_ADV_HMON_STATUS_NODE_OK, ///< the node is healthy.
	XME_ADV_HMON_STATUS_NODE_UNKNOWN, ///< the node is non existent.
	XME_ADV_HMON_STATUS_NODE_EXCEPTION, ///< the target node throwed an exception.
	XME_ADV_HMON_STATUS_UNKNOWN, ///< the health status is unknown.
}
xme_adv_hmon_status_t;

/**
 * \enum xme_adv_hmon_testType_t
 * \brief Enumeration type definition for test types.
 */
typedef enum
{
	XME_ADV_HMON_TYPE_HEARTBEAT = 0, ///< the test type is heartbeat (element is alive).
	XME_ADV_HMON_TYPE_CPU, ///< the test is targeted to test the CPU.
	XME_ADV_HMON_TYPE_CONSISTENCY, ///< the test is targeted to test the consistency.
	XME_ADV_HMON_TYPE_MEMORY, ///< the test is targeted to test the memory.
	XME_ADV_HMON_TYPE_NOTEST, ///< there is no test type definition.
}
xme_adv_hmon_testType_t;

/**
 * \struct xme_adv_hmon_errorMessage_t
 * \brief Structure containing error message description.
 */
typedef struct
{
	xme_core_component_t componentId; ///< the componentId.
	xme_core_device_guid_t nodeId; ///< the nodeId.
	xme_adv_test_type_t identifier; ///< the test type performed.
	xme_adv_hmon_status_t status; ///< the status of the health monitoring operation.
}
xme_adv_hmon_errorMessage_t;

/**
 * \struct xme_adv_hmon_heartbeat_t
 * \brief Structure containing heart beat (periodic check) of components.
 */
typedef struct
{
	xme_core_device_guid_t nodeId; ///< id of the heartbeat's sender.
	xme_hal_time_timeHandCheckdle_t maxTimestampAge; ///< maximum period for health monitoring.
}
xme_adv_hmon_heartbeat_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates a health monitor component.
 *
 * \param config the health monitor configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the health monitor component has been successfully created.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the health monitor component cannot be created.
 */
xme_status_t
xme_adv_hmon_create
(
	xme_adv_hmon_configStruct_t* config
);

/**
 * \brief  Activates a health monitor component.
 *
 * \param config the health monitor configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the health monitor component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the health monitor component cannot be activated due to an error during activation.
 */
xme_status_t
xme_adv_hmon_activate
(
	xme_adv_hmon_configStruct_t* config
);

/**
 * \brief  Deactivates a health monitor component.
 *
 * \param config the health monitor configuration structure.
 */
void
xme_adv_hmon_deactivate
(
	xme_adv_hmon_configStruct_t* config
);

/**
 * \brief  Destroys a health monitor component.
 *
 * \param config the health monitor configuration structure.
 */
void
xme_adv_hmon_destroy
(
	xme_adv_hmon_configStruct_t* config
);

/**
 * \brief  Callback function that is executed when an error message is received.
 *
 * \param  dataHandle Handle to buffer with topic data to send.
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_hmon_receiveErrorMessage
(
	xme_hal_sharedPtr_t dataHandle,
	void* userData
);

// TODO: Prototype defined but not implemented
///**
// * \brief  Callback function that performs consistency checks on the locally
// *         collected data, sends their results and sends meta-heartbeats.
// */
//void
//xme_adv_hmon_callback
//(
//	void* userData
//);

/**
 * \brief  prints the textual expression of a status code.
 *
 * \param error the error status.
 *
 * \return a pointer to the xme_adv_hmon_status_t.
 */
char*
xme_adv_hmon_printHealthmonitorErrNo
(
	xme_adv_hmon_status_t error
);

/**
 * \brief  create error message publication, subscription and routes.
 *
 * \param config the health monitor configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component successfully subscribes to error messages.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot subscribe to system error messages.
 */
xme_status_t
xme_adv_hmon_createHealthMonitorErrorMessages
(
	xme_adv_hmon_configStruct_t* config
);

/**
 * \brief checks the health of the monitored entity, based on received error messages.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component successfully checks health.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot check system health.
 */
xme_status_t
xme_adv_hmon_checkHealth(void);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_ADV_HEALTHMONITOR_H
