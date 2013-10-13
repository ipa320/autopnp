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
 * $Id: testHelper_executionManager.h 4696 2013-08-14 14:13:08Z rupanov $
 */

#ifndef TESTHELPER_EXECUTIONMANAGER_H_
#define TESTHELPER_EXECUTIONMANAGER_H_

#include "mHelperApplication/include/mHelperApplicationFunction.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/core/component.h"
#include "xme/hal/include/time.h"
#include "xme/defines.h"



XME_EXTERN_C_BEGIN

extern xme_core_exec_initCallback_t functionPointers[N_FUNCTIONS_SIMULATED];
extern xme_core_exec_configStruct_t testConfig;

void
preinitXme(void);

/* Helper functions */

xme_core_exec_componentDescriptor_t*
createHelper
(
    xme_core_component_t                cid,
    xme_core_component_functionId_t     fid,
    xme_hal_time_timeInterval_t         wcet
);

xme_status_t
createHelperInstance
(
    xme_core_component_t                cid,
    xme_core_component_functionId_t     fid,
    xme_hal_time_timeInterval_t         wcet
);

uint32_t
getExecutionCount
(
    xme_core_component_t cid,
    xme_core_component_functionId_t fid
);

void clearExecutionCounters( void );

XME_EXTERN_C_END

#endif /* TESTHELPERS_EXECUTIONMANAGER_H_ */
