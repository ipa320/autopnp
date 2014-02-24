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
 * $Id: mockLayerForLoginClient.h 4989 2013-09-05 12:52:18Z gulati $
 */

/**
 * \file
 *         Header for mock layer for login client tests.
 */

XME_EXTERN_C_BEGIN

extern void
xme_wp_marshal_marshaler_getConfigSetStatus(xme_status_t status);

extern void
xme_wp_udp_udpSend_getConfigSetStatus(xme_status_t status);

extern void
udpSendWaypointAddConfigSetStatus(xme_status_t status);

extern void
xme_core_exec_configurator_addComponentToScheduleSetStatus(xme_status_t status);

XME_EXTERN_C_END
