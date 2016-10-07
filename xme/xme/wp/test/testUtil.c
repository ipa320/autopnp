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
 * $Id: testUtil.c 5538 2013-10-17 09:12:41Z wiesmueller $
 */

/**
 * \file
 *         Waypoint Test Utility Functions.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/wp/test/testUtil.h"

#include "xme/core/log.h"

#include "xme/hal/include/time.h"
#include "xme/hal/include/mem.h"

#include <inttypes.h>
#include <limits.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
bool xme_wp_testUtil_measureExecutionTime
(
    uint32_t execCount,
    xme_status_t (*run)(xme_wp_waypoint_instanceId_t instanceId),
    void (*beforeRun)(xme_wp_waypoint_instanceId_t instanceId, uint64_t i),
    void (*afterRun)(xme_wp_waypoint_instanceId_t instanceId, uint64_t i),
    const char* const runName,
    xme_wp_waypoint_instanceId_t* instanceIds,
    uint32_t instanceIdsLength,
    xme_hal_time_timeInterval_t expectedWcet
)
{
    xme_hal_time_timeHandle_t startTimeHandle;
    xme_hal_time_timeInterval_t timeInterval;
    xme_hal_time_timeInterval_t maxTime = 0;
    uint32_t i;
    uint32_t c;
    xme_hal_time_timeInterval_t totalSum = 0;
    xme_hal_time_timeInterval_t averageTime = 0;

    c = 0;
    for (i = 0; i < execCount; i++)
    {
        if (NULL != beforeRun)
        {
            beforeRun(instanceIds[c], i);
        }
    
        startTimeHandle = xme_hal_time_getCurrentTime();

        run(instanceIds[c]);

        timeInterval = xme_hal_time_getTimeInterval(&startTimeHandle, false);
        
        if (NULL != afterRun)
        {
            afterRun(instanceIds[c], i);
        }

#if 0 // Activate this to print each measured execution time individually
        XME_LOG(XME_LOG_NOTE, "%" PRIu64 " ns\n", timeInterval);
#endif

#if 0 // Activate this to print a message every time when the assumed wcet is exceeded
        if (timeInterval > expectedWcet)
        {
            XME_LOG(XME_LOG_NOTE, "Exceeding wcet: %d %" PRIu64 " ns\n", i, timeInterval);
        }
#endif

        if (ULLONG_MAX - timeInterval < totalSum)
        {
            totalSum = ULLONG_MAX;
            XME_LOG(XME_LOG_WARNING, "Total sum of execution time exceeds uint64_t. Computed average time will be wrong (maximum execution time is unaffected by this error).\n", i, timeInterval);
        }
        else
        {
            totalSum += timeInterval;
        }

        if (timeInterval > maxTime)
        {
            maxTime = timeInterval;
        }

        c++;
        if (c >= instanceIdsLength - 1)
        {
            c = 0;
        }
    }

    averageTime = totalSum / (xme_hal_time_timeInterval_t)execCount;

    // Print measurements
    XME_LOG
    (
        XME_LOG_NOTE,
        "Measurements for function %s() with %" PRIu32 " configurations:\n"
        "Average execution time: %12" PRIu64 " ns\n"
        "Maximum execution time: %12" PRIu64 " ns\n"
        "Assumed WCET:           %12" PRIu64 " ns\n",
        runName, instanceIdsLength, averageTime, maxTime, expectedWcet
    );

    // Check if assumed WCET is exceeded by maximum measured execution time
    return expectedWcet < maxTime;
}
