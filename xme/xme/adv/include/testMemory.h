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
 * $Id: testMemory.h 2693 2013-03-17 01:51:56Z camek $
 */

/**
 * \file
 * \brief Memory test component.
 */

#ifndef XME_ADV_TESTMEMORY_H
#define XME_ADV_TESTMEMORY_H

/**
 * \defgroup adv_testMemory Memory Testing Component.
 * @{
 *
 * \brief This component test memory capabilities for health monitor component. 
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/adv/include/healthmonitor.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/** 
 * \enum xme_adv_testmemory_algorithm_t
 * \brief The enumeration structure for defining algorithms to be used in test.
 */
typedef enum
{
	XME_ADV_TEST_MEMORY_ALGORITHM_WALKPATH = 0, ///< use of walkpath algorithm.
	XME_ADV_TEST_MEMORY_ALGORITHM_CHECKERBOARD, ///< use of board checker algorithm.
	XME_ADV_TEST_MEMORY_ALGORITHM_GALPAT, ///< use of galpat algorithm.
	XME_ADV_TEST_MEMORY_ALGORITHM_TRANSPARENT_GALPAT, ///< use of transparent galpat algorithm.
	XME_ADV_TEST_MEMORY_ALGORITHM_INVERT ///< use of invert algorithm.
}
xme_adv_testmemory_algorithm_t;

/** 
 * \struct xme_adv_testMemory_configStruct_t
 * \brief Configuration structure for Memory testing.
 */
typedef struct
{
	// public
	xme_hal_time_timeInterval_t interval; ///< activation interval.
	int startAddress; ///< memory address where test should start.
	int stopAddress; ///< memory address where test should stop.
	xme_adv_testmemory_algorithm_t algorithm; ///< test algorithm.
	xme_status_t (*callback)(); ///< instant error reaction function.
	xme_core_dcc_publicationHandle_t pubHandle; ///< publication handle.
}
xme_adv_testMemory_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates a memory test component.
 * 
 * \param config the memory testing configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully initialized.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be initialized. 
 */
xme_status_t
xme_adv_testMemory_create
(
	xme_adv_testMemory_configStruct_t* config
);


/**
 * \brief  Activates a memory test component.
 * 
 * \param config the memory testing configuration structure.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the component cannot be activated. 
 */
xme_status_t
xme_adv_testMemory_activate
(
	xme_adv_testMemory_configStruct_t* config
);

/**
 * \brief  Deactivates a memory test component.
 * 
 * \param config the memory testing configuration structure.
 */
void
xme_adv_testMemory_deactivate
(
	xme_adv_testMemory_configStruct_t* config
);

/**
 * \brief  Destroys a memory test component.
 * 
 * \param config the memory testing configuration structure.
 */
void
xme_adv_testMemory_destroy
(
	xme_adv_testMemory_configStruct_t* config
);

/**
 * \brief  Callback function that executes memory checks.
 *
 * \param  userData User-defined data passed to the callback function.
 *         The value of this parameter is specified in the call to
 *         xme_core_dcc_subscribeTopic() where this callback function has
 *         been registered.
 */
void
xme_adv_testMemory_callback
(
	void* userData
);

/**
 * \brief  walkpath test function.
 *
 * \param start start time of the walkpath test function.
 * \param stop end time of the walkpath test function.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the check provided positive results.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the check failed. 
 */
xme_status_t
xme_adv_testMemory_walkpath
(
	int start, 
	int stop
);

/**
 * \brief  invert test function.
 *
 * \param start start time of the invert test function.
 * \param stop end time of the invert test function.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the check provided positive results.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if the check failed. 
 */
xme_status_t
xme_adv_testMemory_invert
(
	int start, 
	int stop
);

XME_EXTERN_C_END

/**
 * @}
 */


#endif // #ifndef XME_ADV_TESTMEMORY_H
