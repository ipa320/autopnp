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
 * $Id: rOSComponentWrapper.h 7769 2014-03-11 15:27:13Z geisinger $
 */

/**
 * \file
 *         Component wrapper - implements interface of a component
 *              to the data handler
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

#ifndef ROSGATEWAY_ADV_ROS_ROSCOMPONENTWRAPPER_H
#define ROSGATEWAY_ADV_ROS_ROSCOMPONENTWRAPPER_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/executionManager/include/executionManagerDataStructures.h"


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes this component wrapper.
 *
 * \retval XME_STATUS_SUCCESS on success.
 */
xme_status_t
rOSGateway_adv_rOS_rOSComponentWrapper_init(void);

/**
 * \brief Finalizes this component wrapper.
 */
void
rOSGateway_adv_rOS_rOSComponentWrapper_fini(void);


XME_EXTERN_C_END

#endif // #ifndef ROSGATEWAY_ADV_ROS_ROSCOMPONENTWRAPPER_H