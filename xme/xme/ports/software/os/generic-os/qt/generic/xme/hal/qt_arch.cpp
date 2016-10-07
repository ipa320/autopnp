/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: qt_arch.cpp 6616 2014-02-04 13:33:19Z geisinger $
 */

/**
 * \file
 *         Qt abstraction (platform specific part: generic OS based
 *         implementation).
 */

/**
 * \addtogroup hal_qt
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/qt.h"

#include <stdint.h>

#include "xme/hal/include/cmdLine.h"
#include "xme/hal/include/sched.h"
#include "xme/hal/include/table.h"
#include "xme/hal/include/time.h"

#include "xme/hal/QtApplication.h"


/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static uint8_t xme_hal_qt_initializationCount = 0;
static xme_hal_sched_taskHandle_t xme_hal_qt_taskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;
static QtApplication* xme_hal_qt_application = NULL;


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief Task function to execute qt main loop.
 *
 * \param[in] userData User data of task function (not used).
 */
static
void
xme_hal_qt_task
(
    void* userData
);

#include <iostream>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_qt_init(void)
{
	if (0 == xme_hal_qt_initializationCount)
	{
		//Init Qt;
		xme_hal_qt_taskHandle = xme_hal_sched_addTask(xme_hal_time_timeIntervalFromMilliseconds(0), xme_hal_time_timeIntervalFromMilliseconds(0), 0, xme_hal_qt_task, NULL);
		// Check for errors
		XME_CHECK(XME_HAL_SCHED_INVALID_TASK_HANDLE != xme_hal_qt_taskHandle, XME_STATUS_INTERNAL_ERROR);

		xme_hal_qt_initializationCount++;

		return XME_STATUS_SUCCESS;
	}
	else
	{
		xme_hal_qt_initializationCount++;

		return XME_STATUS_SUCCESS;
	}
}

static
void
xme_hal_qt_task
(
    void* userData
)
{
std::cout<<"xme_hal_qt_task"<<std::endl;
	char** argv;
	int argc = xme_hal_cmdLine_getArgs(&argv);

	xme_hal_qt_application = new QtApplication(argc, argv);
	xme_hal_qt_application->exec();

	delete xme_hal_qt_application;
}

void
xme_hal_qt_triggerExecution
(
    xme_hal_qt_callback_t callback,
    void* userData
)
{
	xme_hal_qt_application->triggerExecution(callback, userData);
}

xme_hal_qt_applicationHandle_t
xme_hal_qt_getApplication()
{
	return xme_hal_qt_application;
}

void
xme_hal_qt_fini(void)
{
	if (0 < xme_hal_qt_initializationCount)
	{
		xme_hal_qt_initializationCount--;

		if (0 == xme_hal_qt_initializationCount)
		{
			//Finalize Qt
			xme_hal_sched_removeTask(xme_hal_qt_taskHandle);
			xme_hal_qt_taskHandle = XME_HAL_SCHED_INVALID_TASK_HANDLE;

			xme_hal_qt_application->exit();
		}
	}
}

/**
 * @}
 */
