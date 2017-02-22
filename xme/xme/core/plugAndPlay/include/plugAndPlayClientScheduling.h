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
 * $Id: plugAndPlayClientScheduling.h 6250 2013-12-20 18:16:10Z ruiz $
 */

/**
 * \file
 *         Plug and Play Client Scheduling Header.
 *
 */
#ifndef XME_CORE_PNP_PLUGANDPLAYCLIENTSCHEDULING_H
#define XME_CORE_PNP_PLUGANDPLAYCLIENTSCHEDULING_H

/**
 * \addtogroup core_pnp_pnpClient
 * @{
 */
/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/mem.h"

#include "xme/core/broker/include/broker.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/hal/include/table.h"

#include <inttypes.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

XME_EXTERN_C_BEGIN

/**
 * \brief This function initializes all the newly added components.
 *
 * \details It iterates over all the configuration tables, initializes the corresponding.
 *          parameters of the components and addes them to the scheduler to be ready
 *          for execution in the next cycle.
 *
 * \retval XME_STATUS_SUCCESS if the components were initialized successfully
 * \retval other status value returned by the failing component
 */
xme_status_t 
xme_core_pnp_pnpClientScheduling_initComponents(void);

/**
 * \brief This function initializes the static data structures of the file.
 *
 * \details It is called from  xme_core_pnp_pnpClient_init durin the initializing
 *          of the PnP Client module.
 */
void
xme_core_pnp_pnpClientScheduling_init(void);

/**
 * \brief Finalize the components in scheduling. 
 *
 * \retval XME_STATUS_SUCCESS If components in schedule were finished. 
 */
xme_status_t
xme_core_pnp_pnpClientScheduling_finiComponents(void);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_PNP_PLUGANDPLAYCLIENTSCHEDULING_H
