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
 * $Id: ipLoginClientProxy.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         IP login client proxy.
 *
 */

#ifndef XME_PRIM_IPLOGINCLIENTPROXY_H
#define XME_PRIM_IPLOGINCLIENTPROXY_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/rr.h"

#include "xme/hal/include/net.h"
#include "xme/hal/include/sched.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_prim_ipLoginClientProxy_configStruct_t
 *
 * \brief  IP login proxy configuration structure.
 */
typedef struct
{
	// public
	xme_com_interface_interfaceId_t interfaceId; ///< the interface identifier.
	// private
	xme_core_rr_requestHandle_t loginRequestHandle; ///< the login request handle. 
	xme_core_dcc_publicationHandle_t managementChannelsToNewNode; ///< Publication handle to trigger construction of management channel to new node.
	xme_hal_sched_taskHandle_t loginTaskHandle; ///< the login task handle. 
	xme_hal_net_socketHandle_t loginServerSocket; ///< the login server socket. 
}
xme_prim_ipLoginClientProxy_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates an IP login client proxy component.
 * \param config the configuration structure. 
 * \retval XME_STATUS_SUCCESS if the IP login client proxy component is successfully created. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the component. 
 */
xme_status_t
xme_prim_ipLoginClientProxy_create(xme_prim_ipLoginClientProxy_configStruct_t* config);

/**
 * \brief  Activates an IP login client proxy component.
 * \param config the configuration structure. 
 * \retval XME_STATUS_SUCCESS if the IP login client proxy component is successfully activated. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot activate the component. 
 */
xme_status_t
xme_prim_ipLoginClientProxy_activate(xme_prim_ipLoginClientProxy_configStruct_t* config);

/**
 * \brief  Deactivates an IP login client proxy component.
 * \param config the configuration structure. 
 */
void
xme_prim_ipLoginClientProxy_deactivate(xme_prim_ipLoginClientProxy_configStruct_t* config);

/**
 * \brief  Destroys an IP login client proxy component.
 * \param config the configuration structure. 
 */
void
xme_prim_ipLoginClientProxy_destroy(xme_prim_ipLoginClientProxy_configStruct_t* config);

XME_EXTERN_C_END

#endif // #ifndef XME_PRIM_IPLOGINCLIENTPROXY_H
