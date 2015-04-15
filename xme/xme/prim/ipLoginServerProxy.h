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
 * $Id: ipLoginServerProxy.h 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         IP login server proxy.
 *
 */

#ifndef XME_PRIM_IPLOGINSERVERPROXY_H
#define XME_PRIM_IPLOGINSERVERPROXY_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/device.h"
#include "xme/hal/include/net.h"
#include "xme/hal/include/sched.h"
#include "xme/hal/include/table.h"
#include "xme/hal/include/time.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_prim_ipLoginServerProxy_pendingResponseItem_t
 *
 * \brief  The IP login server proxy pending response item.
 */
typedef struct
{
	xme_core_rr_responseInstanceHandle_t responseInstanceHandle; ///< the response instance handle. 
	xme_core_device_type_t deviceType; ///< the device type.
	xme_core_device_guid_t deviceGuid; ///< the device identification. 
	xme_hal_time_timeInterval_t responseTimeoutMs; ///< the response timeout (in miliseconds)
	xme_hal_time_timeHandle_t lastUpdate; ///< last update handle. 
}
xme_prim_ipLoginServerProxy_pendingResponseItem_t;

/**
 * \struct xme_prim_ipLoginServerProxy_configStruct_t
 *
 * \brief  IP login server proxy configuration structure.
 */
typedef struct
{
	// public
	// private
	xme_core_rr_requestHandlerHandle_t loginRequestHandlerHandle; ///< Login request handler handle.
	xme_hal_net_socketHandle_t loginClientSocket; ///< Socket over which login requests are sent.
	xme_hal_sched_taskHandle_t loginResponseTaskHandle; ///< Login response task.
	XME_HAL_TABLE(xme_prim_ipLoginServerProxy_pendingResponseItem_t, pendingResponses, XME_HAL_DEFINES_MAX_IPLOGINSERVER_PENDINGRESPONSE_ITEMS); ///< Pending login responses.
}
xme_prim_ipLoginServerProxy_configStruct_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Creates an IP login server proxy component.
 * \param config the configuration structure. 
 * \retval XME_STATUS_SUCCESS if the IP login server proxy component is successfully created. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the component. 
 */
xme_status_t
xme_prim_ipLoginServerProxy_create(xme_prim_ipLoginServerProxy_configStruct_t* config);

/**
 * \brief  Activates an IP login server proxy component.
 * \param config the configuration structure. 
 * \retval XME_STATUS_SUCCESS if the IP login server proxy component is successfully created. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot create the component. 
 */
xme_status_t
xme_prim_ipLoginServerProxy_activate(xme_prim_ipLoginServerProxy_configStruct_t* config);

/**
 * \brief  Deactivates an IP login server proxy component.
 * \param config the configuration structure. 
 */
void
xme_prim_ipLoginServerProxy_deactivate(xme_prim_ipLoginServerProxy_configStruct_t* config);

/**
 * \brief  Destroys an IP login server proxy component.
 * \param config the configuration structure. 
 */
void
xme_prim_ipLoginServerProxy_destroy(xme_prim_ipLoginServerProxy_configStruct_t* config);

XME_EXTERN_C_END

#endif // #ifndef XME_PRIM_IPLOGINSERVERPROXY_H
