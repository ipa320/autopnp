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
 * $Id: coreLoopScheduler.h 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         RTE Scheduler.
 */

/**
 * \defgroup core_loop RTE Scheduler
 * @{
 *
 * \brief The objective of the RTE Scheduler is to allow compact low-overhead execution of core components.
 *
 * \details RTE Scheduler is taking care of scheduling the RTE components.
 *        In some cases it is just a simple sequence of components, but
 *        in general it should define the strategy to execute core
 *        components to reach seamless time+event-triggered behavior.
 *
 */
#ifndef RTE_SCHEDULER_H
#define RTE_SCHEDULER_H

#include "xme/core/rteScheduler/include/coreLoopChunk.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"

/** ID of the RTE component - this componentId is used by all RTE chunks
 *      while functionId is set to chunk ID.
 */
#define  RTE_BASE_COMPONENT_ID ((xme_core_component_t)73)

/** Wait slack in a hybrid waiting algorithm. If the RTEScheduler needs to wait more than this number of nanoseconds,
 * it issues sleep(), otherwise using busy-wait. Wakeup from sleep is also offset back in time with this value.*/
#define XME_CORE_RTESCHED_HYBRID_WAIT_SLACK ((uint64_t)100)

typedef struct xme_core_loop_configStruct_
{
    xme_core_exec_functionSpecificCallback_t onFunctionActivate;
    xme_core_exec_functionSpecificCallback_t onFunctionReturned;
}
xme_core_loop_configStruct_t;

extern xme_core_loop_configStruct_t xme_core_loop_intConfig;

/**
 * \brief Initialize the RTE scheduler
 *
 * \returns Operation status: XME_STATUS_SUCCESS in case of success, a relevant error code in case of error.
 */
extern
xme_status_t
xme_core_loop_init( xme_core_loop_configStruct_t* );

/**
 * \brief Free all internal structures and finalize the RTE scheduler
 *
 * \returns Operation status: XME_STATUS_SUCCESS in case of success, a relevant error code in case of error.
 */
extern xme_status_t
xme_core_loop_fini( void );

/**
 * \brief Allocate a chunk in the heap and initialize it
 * \param[in]   startTime_ns start of chunk relative to start of cyclic schedule
 * \param[in]   endTime_ns end of chunk relative to start of cyclic schedule
 * \param[out]  ID of the newly created chunk
 * \returns Operation status: XME_STATUS_SUCCESS in case of success, a relevant error code in case of error.
 */
extern xme_status_t 
xme_core_loop_allocateRteChunk
(
	xme_hal_time_timeInterval_t startTime_ns,
	xme_hal_time_timeInterval_t endTime_ns,
    xme_core_loop_Chunk_handle_t* const chunkId
);

/**
 * \brief Adds a chunk to node schedule
 * \param[in] scheduleId    ID of the node schedule to append to
 * \param[in] chunkId       ID of the chunk
 *
 * \returns Operation status: XME_STATUS_SUCCESS in case of success, a relevant error code in case of error.
 */
extern xme_status_t
xme_core_loop_addChunkToGlobalSchedule
(
    xme_core_exec_schedule_handle_t scheduleId,
    xme_core_loop_Chunk_handle_t chunkId
);

/**
 * \brief Returns a descriptor of an RTE chunk with specified ID
 *
 * \param[in] id ID of the chunk
 *
 * \returns Chunk descriptor in case of success, NULL if not found or an error occured.
 */
extern xme_core_loop_ChunkDescriptor_t*
xme_core_loop_getChunkDescriptor(xme_core_loop_Chunk_handle_t id);

#endif

/**
 * @}
 */
