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
 * $Id: testUtil.h 5538 2013-10-17 09:12:41Z wiesmueller $
 */

#ifndef XME_WP_TESTUTIL_H
#define XME_WP_TESTUTIL_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/wp/waypoint.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Measure execution time for given waypoint run method.
 *
 * \details Executes the given waypoint run method execCount times.
 *          While executing it will iterate over the given instanceIds
 *          (starting again from index 0 when end of instanceIds array
 *          is reached, which will happen when execCount > instanceIdsLength).
 *
 * \note Depending on the implementation of xme_hal_time the measurement may
 *       be very inaccurate.
 *
 * \param execCount How often to execute and measure the function.
 * \param run Function to measure.
 * \param beforeRun This function will be executed before each call to the
 *        run method (not included in time measurement). The instanceId
 *        and current execution count will be passed to the function.
 *        May be NULL.
 * \param afterRun This function will be executed after each call to the
 *        run method (not included in time measurement). The instanceId
 *        and current execution count will be passed to the function.
 *        May be NULL.
 * \param runName Name of function to measure (used for log output).
 * \param instanceIds Array of waypoint instance ids.
 * \param instanceIdsLength Number of elements in instanceIds array.
 * \param expectedWcet The expected wcet time.
 *
 * \return Returns true when maximum measured execution time exceeds
 *         expectedWcet.
 */
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
);

XME_EXTERN_C_END

#endif /* XME_WP_TESTUTIL_H */
