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
 * $Id: log.c 3022 2013-04-24 10:54:45Z ruiz $
 */

/**
 * \file
 *         Logging system abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/log.h"

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/
xme_core_log_logCallback_t xme_core_log_logCallback;
