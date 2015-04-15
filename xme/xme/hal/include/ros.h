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
 * $Id: ros.h 6616 2014-02-04 13:33:19Z geisinger $
 */

/** 
 * \file 
 * \brief ROS abstraction.
 *
 */

#ifndef XME_HAL_ROS_H
#define XME_HAL_ROS_H

/** 
 * \defgroup hal_ROS ROS.
 * @{ 
 *
 * \brief ROS abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Sets the command line parameters, which shall be handed over to ROS
 *        during the initialization.
 *
 * \param[in] argc Number of command line parameters.
 * \param[in] argv Array containing the command line parameters.
 *
 * \retval XME_STATUS_SUCCESS if the operation completed successfully
 * \retval XME_STATUS_INTERNAL_ERROR if the function is called after ROS has
 *         been initialized.
 */
xme_status_t
xme_hal_ros_setCommandLineParameters(int argc, char* argv[]);

/**
 * \brief  Initializes the ROS system for this node, if not already done.
 *
 * \retval XME_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_STATUS_INTERNAL_ERROR if something went wrong.
 */
xme_status_t
xme_hal_ros_init(void);

/**
 * \brief  Checks whether ROS is still running.
 *
 * \return Whether ROS is still running or not.
 */
bool
xme_hal_ros_ok(void);

/**
 * \brief  Kicks on ROS system for internal processing once.
 */
void
xme_hal_ros_spinOnce(void);

/**
 * \brief  Kicks on ROS system for internal processing.
 */
void
xme_hal_ros_spin(void);

/**
 * \brief  Finalizes the ROS system for this node, if no one else is using ROS
 *         anymore.
 */
void
xme_hal_ros_fini(void);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_HAL_ROS_H
