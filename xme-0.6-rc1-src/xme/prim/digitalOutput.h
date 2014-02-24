/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: digitalOutput.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         Digital output pin.
 */

#ifndef XME_PRIM_DIGITALOUTPUT_H
#define XME_PRIM_DIGITALOUTPUT_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/io.h"

/******************************************************************************/
/***   Model constraints                                                    ***/
/******************************************************************************/
XME_MDL_RESOURCE_DEPENDENCY()

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_prim_digitalOutput_configStruct
 *
 * \brief  Digital output configuration structure.
 */
typedef struct
{
	xme_hal_io_portPin pin; ///< the port pin. 
}
xme_prim_digitalOutput_configStruct;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief  Creates the Digital output component.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully created.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be created.
 */
void
xme_prim_digitalOutput_create
(
	xme_prim_digitalOutput_configStruct* config
);

/**
 * \brief  Activate the Digital output component.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully activated.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be activated.
 */
void
xme_prim_digitalOutput_activate
(
	xme_prim_digitalOutput_configStruct* config
);

/**
 * \brief  Deactivates the Digital Output component.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully deactivated.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be deactivated.
 */
void
xme_prim_digitalOutput_deactivate
(
	xme_prim_digitalOutput_configStruct* config
);

/**
 * \brief  Destroys the Digital output component.
 *
 * \param  config the configuration parameters.
 * \retval XME_CORE_STATUS_SUCCESS if the component has been successfully destroyed.
 * \retval XME_CORE_STATUS_INTERNAL ERROR if the component cannot be destroyed.
 */
void
xme_prim_digitalOutput_destroy
(
	xme_prim_digitalOutput_configStruct* config
);

#endif // #ifndef XME_PRIM_DIGITALOUTPUT_H
