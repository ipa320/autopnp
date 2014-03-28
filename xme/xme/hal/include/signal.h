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
 * $Id: signal.h 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Signal abstraction for communication with other processes/threads.
 */

#ifndef SIGNAL_H_
#define SIGNAL_H_

#include "xme/defines.h"
XME_EXTERN_C_BEGIN

xme_status_t
xme_hal_signal_create(bool blocking, uint32_t processSrc, uint32_t processTarget, uint32_t signalId, uint32_t* signalHandle);

xme_status_t
xme_hal_signal_set(uint32_t signalHandle);

xme_status_t
xme_hal_signal_wait(uint32_t signalHandle);

xme_status_t
xme_hal_signal_test(uint32_t signalHandle);

xme_status_t
xme_hal_signal_destroy(uint32_t signalHandle);

XME_EXTERN_C_END

#endif /* SIGNAL_H_ */
