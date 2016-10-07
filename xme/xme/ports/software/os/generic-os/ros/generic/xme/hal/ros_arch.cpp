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
 * $Id: ros_arch.cpp 6587 2014-02-02 12:27:34Z kainz $
 */

/**
 * \file
 *         ROS abstraction (platform specific part: generic OS based
 *         implementation).
 */

/**
 * \addtogroup hal_ros 
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/ros.h"

#include <stdint.h>

#pragma GCC diagnostic ignored "-Wshadow" //TODO find better solution
#include "ros/ros.h"

#include "xme/core/node.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static uint8_t xme_hal_ros_initializationCount = 0;
static int xme_hal_ros_argc = 0;
static char** xme_hal_ros_argv = NULL;


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_ros_setCommandLineParameters(int argc, char* argv[])
{
	if (0 == xme_hal_ros_initializationCount)
	{
		xme_hal_ros_argc = argc;
		xme_hal_ros_argv = argv;

		return XME_STATUS_SUCCESS;
	}
	else
	{
		return XME_STATUS_INTERNAL_ERROR;
	}
}

xme_status_t
xme_hal_ros_init()
{
	if (0 == xme_hal_ros_initializationCount)
	{
		char nodeName[256];

		if (XME_STATUS_SUCCESS == xme_core_node_getNodeName(nodeName, (uint16_t)sizeof(nodeName)))
		{
			ros::init(xme_hal_ros_argc, xme_hal_ros_argv, nodeName, ros::init_options::AnonymousName);
			xme_hal_ros_initializationCount++;

			return XME_STATUS_SUCCESS;
		}
		else
		{
			return XME_STATUS_INTERNAL_ERROR;
		}
	}
	else
	{
		xme_hal_ros_initializationCount++;

		return XME_STATUS_SUCCESS;
	}
}

bool
xme_hal_ros_ok()
{
	return ros::ok();
}

void
xme_hal_ros_spinOnce()
{
	ros::spinOnce();
}

void
xme_hal_ros_spin()
{
	ros::spin();
}

void
xme_hal_ros_fini()
{
	if (0 < xme_hal_ros_initializationCount)
	{
		xme_hal_ros_initializationCount--;

		if (0 == xme_hal_ros_initializationCount)
		{
			//Finalize code of ROS
		}
	}
}

/**
 * @}
 */

