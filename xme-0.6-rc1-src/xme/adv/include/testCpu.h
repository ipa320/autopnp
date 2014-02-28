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
 * $Id: testCpu.h 3546 2013-05-28 18:56:57Z geisinger $
 */

/**
 * \file
 * \brief CPU test component.
 */

#ifndef XME_ADV_TESTCPU_H
#define XME_ADV_TESTCPU_H

/**
 * \defgroup adv_testCPU CPU Testing Component
 * @{
 *
 * \brief This component test CPU capabilities for health monitor component. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/healthmonitor.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/** 
 * \enum xme_adv_testcpu_algorithm_t
 * \brief The enumeration structure for defining algorithms to be used in test.
 */
typedef enum
{
	XME_ADV_TEST_MEMORY_ALGORITHM_LIMITED_NR_PATTERNS = 0, ///< number of patterns to apply to the algorithm */
	XME_ADV_TEST_CPU_ALGORITHM_WALKING_BIT, ///< algorithm walking bit */
}
xme_adv_testcpu_algorithm_t;

/** 
 * \struct xme_adv_testCpu_configStruct_t
 * \brief Configuration structure for CPU testing
 */
typedef struct
{
	// public
	xme_hal_time_timeInterval_t interval;       ///< activation interval.
	xme_adv_testcpu_algorithm_t algorithm;      ///< test algorithm.
	xme_status_t (*callback)();            ///< instant error reaction function.
	int runlength;                              ///< width of data path.
	xme_core_dcc_publicationHandle_t pubHandle; ///< publication handle.
}
xme_adv_testCpu_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates a cpu test component.
 * 
 * \param config the CPU testing configuration structure
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully initialized.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be initialized. 
 */
xme_status_t
xme_adv_testCpu_create
(
	xme_adv_testCpu_configStruct_t* config
);


/**
 * \brief  Activates a cpu test component.
 * 
 * \param config the CPU testing configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be activated. 
 */
xme_status_t
xme_adv_testCpu_activate
(
	xme_adv_testCpu_configStruct_t* config
);

/**
 * \brief  Deactivates a cpu test component.
 * 
 * \param config the CPU testing configuration structure.
 */
void
xme_adv_testCpu_deactivate
(
	xme_adv_testCpu_configStruct_t* config
);


/**
 * \brief  Destroys a cpu test component.
 * 
 * \param config the CPU testing configuration structure.
 */
void
xme_adv_testCpu_destroy
(
	xme_adv_testCpu_configStruct_t* config
);

/**
 * \brief  Callback function that executes CPU checks
 *
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_testCpu_callback 
(
	void* userData
);

/**
 * \brief  limited number of pattern test function.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the check provided positive results.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the check failed. 
 */
xme_status_t
xme_adv_testCpu_limitedNrPatterns(void);


/**
 * \brief  walking bit test function.
 *
 * \param runlength duration of CPU check test.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the check provided positive results.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the check failed. 
 */
xme_status_t
xme_adv_testCpu_walkingBit
(
	unsigned int runlength
);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_ADV_TESTCPU_H
