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
 * $Id: coreLoopChunk.h 5188 2013-09-26 12:39:27Z camek $
 */

/**
 * \file
 *         Declaration of RTE chunks, slots and related functions.
 */

/**
 * \ingroup core_loop RTE Scheduler
 * @{
 *
 */
#ifndef RTE_CHUNK_H
#define RTE_CHUNK_H

#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/defines.h"

/**
 * \typedef xme_core_loop_Chunk_handle_t
 *
 * \brief Handle / ID of the RTE chunk
 */
typedef uint32_t xme_core_loop_Chunk_handle_t;

/**
 * \typedef xme_core_loop_SlotDescriptor_t
 *
 * \brief Slot descriptor for RTE scheduler.
 *
 */
/* todo think if core_loop needs both component descriptor and function descriptor in slot */
typedef struct xme_core_loop_SlotDescriptor_s
{
    xme_core_exec_componentDescriptor_t* componentDesc; ///< Pointer to the locally executed RTE component
    xme_core_exec_functionDescriptor_t* functionDesc; ///< Exact function to start within component
    xme_hal_time_timeInterval_t startTime_ns;  ///< Microsecond precise slot location within the frame

    xme_hal_time_timeInterval_t expAvgExecTime;
    uint64_t nExecutions;
} xme_core_loop_SlotDescriptor_t;

/**
 * \typedef xme_core_loop_ChunkDescriptor_t
 *
 * \brief Chunk descriptor for RTE scheduler.
 *
 * A chunk is a set of functions to be executed in the specified sequence in the same task.
 *
 */
typedef struct xme_core_loop_ChunkDescriptor_s
{
    xme_hal_singlyLinkedList_t(XME_CORE_EXEC_MAX_FUNCTIONS_PER_CHUNK) slots; ///< Slot list
    xme_core_loop_Chunk_handle_t id;   ///< Id of the chunk - number in the chunk table
    xme_hal_time_timeInterval_t wcet_ns; ///< Worst case execution time for an RTE chunk as a sum of all slots
    xme_hal_time_timeInterval_t startTime_ns; ///< Start of chunk
    xme_hal_time_timeInterval_t endTime_ns; ///< End of chunk
    xme_core_exec_functionDescriptor_t* executionUnit; ///< Function descriptor for the execution manager.

} xme_core_loop_ChunkDescriptor_t;

XME_EXTERN_C_BEGIN

/** \brief Execute the contests of the chunk with the given descriptor
 *
 *  Acts as a callback function / wrapper function for Execution Manager.
 *
 * \param[in] chunk descriptor of the chunk to be executed
 */
extern
void
xme_core_loop_execChunk
(
        void* chunk
);

/** \brief Add a function to a specific chunk with given start time
 *
 * \param[in] chunkId ID of the chunk
 * \param[in] componentId ID of the component
 * \param[in] functionId ID of a function
 * \param[in] startTime_ns cycle-global time, at which the function is to be inserted
 * \returns Operation status: XME_STATUS_SUCCESS in case of success, a relevant error code in case of error.
 */
/* todo relative start time option is desirable */
extern
xme_status_t
xme_core_loop_addFunctionSlotToRteChunk
(
    xme_core_loop_Chunk_handle_t chunkId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_hal_time_timeInterval_t startTime_ns
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif


