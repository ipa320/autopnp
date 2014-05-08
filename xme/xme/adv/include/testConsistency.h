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
 * $Id: testConsistency.h 3546 2013-05-28 18:56:57Z geisinger $
 */

/**
 * \file
 * \brief Consistency test component.
 */

#ifndef XME_ADV_TESTCONSISTENCY_H
#define XME_ADV_TESTCONSISTENCY_H

/**
 * \defgroup adv_testConsistency Consistency Testing Component.
 * @{
 *
 * \brief This component test consistency for health monitor component. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/healthmonitor.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/** 
 * \struct xme_adv_testConsistency_configStruct_t
 * \brief Configuration structure for testing consistency in health monitor component.
 */
typedef struct
{
	// public
	xme_hal_time_timeInterval_t startTime; ///< the starting time.
	xme_hal_time_timeInterval_t period;    ///< the period.
}
xme_adv_testConsistency_configStruct_t;

/** 
 * \typedef (*checkFunctionPointer_t)()
 * \brief defines a new data type based on bool.
 */
typedef bool (*checkFunctionPointer_t)();

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/** 
 * \def XME_ADV_TESTCONSISTENCY_FUNCTIONS_MAX
 * \brief Maximum number of consistency functions to be tested.
 */
#define XME_ADV_TESTCONSISTENCY_FUNCTIONS_MAX 10

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates a consistency test component.
 * 
 * \param config the consistency testing configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the consistency test component has been successfully initialized.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the consistency test component cannot be created.
 */
xme_status_t
xme_adv_testConsistency_create
(
	xme_adv_testConsistency_configStruct_t* config
);


/**
 * \brief  Activates a consistency component.
 * 
 * \param config the consistency testing configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the consistency test component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the consistency test component cannot be activated.
 */
xme_status_t
xme_adv_testConsistency_activate
(
	xme_adv_testConsistency_configStruct_t* config
);


/**
 * \brief  Deactivates a consistency component.
 * 
 * \param config the consistency testing configuration structure.
 */
void
xme_adv_testConsistency_deactivate
(
	xme_adv_testConsistency_configStruct_t* config
);


/**
 * \brief  Destroys a consistency component.
 * 
 * \param config the consistency testing configuration structure.
 */
void
xme_adv_testConsistency_destroy
(
	xme_adv_testConsistency_configStruct_t* config
);

/**
 * \brief  Callback function that executes consistency checks.
 *
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_testConsistency_callback 
(
	void* userData
);

/**
 * \brief  Function that adds the consistency checks, which should be executed. Project specific!
 */
void 
xme_adv_testConsistency_initFunctionsTable(void);

/**
 * \brief  example function 1
 *
 * \return a boolean value. // TODO: Define this value
 */
bool 
xme_adv_testConsistency_func1();

/**
 * \brief  example function 2
 *
 * \return a boolean value. // TODO: Define this value
 */
bool 
xme_adv_testConsistency_func2();

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_ADV_TESTCONSISTENCY_H
