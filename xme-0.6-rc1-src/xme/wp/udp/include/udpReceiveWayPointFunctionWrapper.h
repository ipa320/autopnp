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
 * $Id: udpReceiveWayPointFunctionWrapper.h 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 *         Function wrapper - a generic abstraction for one executable function
 *              scheduled by the execution manager.
 */

#ifndef XME_WP_UDP_UDPRECEIVEWAYPOINTFUNCTIONWRAPPER_H
#define XME_WP_UDP_UDPRECEIVEWAYPOINTFUNCTIONWRAPPER_H

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
/**
 * \brief Executes the current operation. 
 * \param param the associated parameters. 
 */
void
xme_wp_udp_udpReceive_udpReceiveWayPointFunctionWrapper_execute
(
    void* param
);

#endif // #ifndef XME_WP_UDP_UDPRECEIVEWAYPOINTFUNCTIONWRAPPER_H
