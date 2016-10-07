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
 * $Id: testStub.h 3546 2013-05-28 18:56:57Z geisinger $
 */

/**
 * \file
 * \brief  This is the stub of a test component
 *
 * \detail This stub is based on the SAFE design. It
 *         can be used to create new test components very easily by making a
 *         copy of it and changing the sections, marked with 'TODO'. In addition,
 *         it might be necessary to extend xme_adv_testStub_configStruct_t, if
 *         additional information is required to execute the test. Please have
 *         a look at the heartbeatComponent and the heartbeatTestComponent for
 *         an example how tests with external evidence generators can be imple-
 *         mented.
 */

#ifndef XME_ADV_TESTSTUB_H
#define XME_ADV_TESTSTUB_H

/**
 * \defgroup adv_testStub Stub Test Component
 * @{
 *
 * \brief This component is the stub of a test component. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/healthmonitor.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/** 
 * \enum xme_adv_teststub_algorithm_t
 * \brief The enumeration structure for defining algorithms to be used in test.
 */
typedef enum

/** 
 * \struct xme_adv_testStub_configStruct_t
 * \brief Configuration structure for <<stub>> testing.
 */
typedef struct
{
	// public
	xme_hal_time_timeInterval_t interval; ///< activation interval.
	xme_status_t (*callback)(); ///< instant error reaction function.
	xme_core_dcc_publicationHandle_t pubHandle; ///< publication handle.
}
xme_adv_testStub_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates a stub test component.
 * 
 * \param config the <<stub>> testing configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully initialized.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be initialized. 
 */
xme_status_t
xme_adv_testStub_create
(
	xme_adv_testStub_configStruct_t* config
);

/**
 * \brief  Activates a stub test component.
 * 
 * \param config the <<stub>> testing configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be activated. 
 */
xme_status_t
xme_adv_testStub_activate(xme_adv_testStub_configStruct_t* config);

/**
 * \brief  Deactivates a stub test component.
 * 
 * \param config the <<stub>> testing configuration structure.
 */
void
xme_adv_testStub_deactivate(xme_adv_testStub_configStruct_t* config);

/**
 * \brief  Destroys a stub test component.
 * 
 * \param config the <<stub>> testing configuration structure.
 */
void
xme_adv_testStub_destroy(xme_adv_testStub_configStruct_t* config);

/**
 * \brief  Callback function that executes <<stub>> checks.
 */
void
xme_adv_testStub_callback 
(
	void* userData
);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_ADV_TESTSTUB_H
