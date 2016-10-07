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
 * $Id: chunk.c 7459 2014-02-18 10:25:58Z geisinger $
 */


/**
 * \file
 *         Implementation of RTE chunk-related functionality.
 */

#define MODULE_ACRONYM "RteSchedCh: "

/*****************************************************************************/
/* XME core*/
#include "xme/defines.h"
#include "xme/core/executionManager/include/executionManagerWrapperInterface.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/rteScheduler/include/coreLoopChunk.h"
#include "xme/core/rteScheduler/include/coreLoopScheduler.h"
#include "xme/core/log.h"

/* HAL */
#include "xme/hal/include/linkedList.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/sleep.h"
#include "xme/hal/include/time.h"
#include <inttypes.h>

/******************************************************************************/
/*  Prototypes  */
/******************************************************************************/
/**
 * \brief Execute all slots of the chunk sequentially
 */
static uint32_t
xme_core_loop_executeAllSlots
(
    xme_core_loop_ChunkDescriptor_t* chunkDesc
);
//******************************************************************************//
/** \brief Checks the schedule for a chunk and computes its wcet */
xme_status_t
xme_core_loop_checkRteChunkSchedule
(
        xme_core_loop_ChunkDescriptor_t* scheduleChunk
);
//******************************************************************************//
extern
xme_core_loop_ChunkDescriptor_t*
xme_core_loop_initChunk( void );
//******************************************************************************//
extern
xme_status_t
xme_core_loop_finiChunk
(
        xme_core_loop_ChunkDescriptor_t* chunk
);
//******************************************************************************//
static void
dumpSchedule(xme_core_loop_ChunkDescriptor_t* chunkDesc, const char* message);

#ifdef XME_CORE_RTESCHED_USE_SLEEP
static void
delayTillStartOfNextSlot_usingSleep(const xme_hal_time_timeHandle_t timeToStartSlot,
                                          xme_hal_time_timeHandle_t* currentTime);
#else
static void
delayTillStartOfNextSlot_usingBusyWait(const xme_hal_time_timeHandle_t timeToStartSlot,
                                       xme_hal_time_timeHandle_t* currentTime);
#endif
//******************************************************************************//
/* Chunk API for definition of RTE chunks */
static void
dumpSchedule(xme_core_loop_ChunkDescriptor_t* chunkDesc, const char* message)
{
    XME_UNUSED_PARAMETER(message);
    XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM
"***************************************************************************************\n"
            );//, message);

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "ID   %16d Start%16"PRIu64" End  %16"PRIu64" wcet %16"PRIu64"\n",
        chunkDesc->id,
        chunkDesc->startTime_ns,
        chunkDesc->endTime_ns,
        chunkDesc->wcet_ns);

XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM
"---------------------------------------------------------------------------------------\n");

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "%-21s %-21s %-21s %-21s\n",
        "SlotStart",
        "SlotWcet",
        "Component",
        "Function");

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(chunkDesc->slots, xme_core_loop_SlotDescriptor_t, slotDesc);

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "     %16"PRIu64"      %16"PRIu64"      %16u      %16u\n",
        slotDesc->startTime_ns,
        slotDesc->functionDesc->wcet_ns,
        slotDesc->functionDesc->componentId,
        slotDesc->functionDesc->functionId);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
}

//***************************************************************************//
extern xme_status_t
xme_core_loop_checkRteChunkSchedule
(
    xme_core_loop_ChunkDescriptor_t* scheduleChunk
)
{
    xme_hal_time_timeInterval_t endOfPreviousSlot_ns = 0;
    xme_hal_time_timeInterval_t totalChunkRuntime_ns = 0;

    // Iterate through the whole rte schedule chunk
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(scheduleChunk->slots,
            xme_core_loop_SlotDescriptor_t,    // list type
            slot);     // iterated variable
    {
        // Check if the slots are well aligned and do not intersect
        XME_CHECK_MSG((endOfPreviousSlot_ns <= slot->startTime_ns),
                XME_STATUS_INVALID_CONFIGURATION,
                XME_LOG_FATAL,
                "Checking schedule (chunk %d slot %d|%d): endOfPreviousSlot[=%"PRIu64"] > slot->startTime[=%"PRIu64"]\n",
                scheduleChunk->id, slot->functionDesc->componentId, slot->functionDesc->functionId, endOfPreviousSlot_ns, slot->startTime_ns);

        // Get next slot end through function wcet
        endOfPreviousSlot_ns += slot->functionDesc->wcet_ns;
        totalChunkRuntime_ns += slot->functionDesc->wcet_ns;
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // Update the chunk's wcet
    scheduleChunk->wcet_ns = totalChunkRuntime_ns;

    XME_CHECK_MSG(scheduleChunk->wcet_ns < (scheduleChunk->endTime_ns - scheduleChunk->startTime_ns),
        XME_STATUS_INVALID_CONFIGURATION,
        XME_LOG_FATAL,
        MODULE_ACRONYM "Checking schedule (chunk %d): total wcet exceeds chunk boundary\n",
        scheduleChunk->id);

    dumpSchedule(scheduleChunk, "passed the check");

    return XME_STATUS_SUCCESS;
}

//***************************************************************************//
extern
xme_core_loop_ChunkDescriptor_t*
xme_core_loop_initChunk( void )
{
    xme_core_loop_ChunkDescriptor_t* chunk = NULL;
    chunk = (xme_core_loop_ChunkDescriptor_t*)
            xme_hal_mem_alloc(sizeof(xme_core_loop_ChunkDescriptor_t));
    XME_HAL_SINGLYLINKEDLIST_INIT(chunk->slots);

    return chunk;
}

//***************************************************************************//
extern
xme_status_t
xme_core_loop_finiChunk
(
        xme_core_loop_ChunkDescriptor_t* chunk
)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(chunk->slots, xme_core_loop_SlotDescriptor_t, slot);

    XME_LOG(XME_LOG_ERROR, "|[%3d|%3d] %6"PRIu64" %10"PRIu64" us %10"PRIu64" us|\n",
            slot->functionDesc->componentId, slot->functionDesc->functionId,
            slot->nExecutions, slot->expAvgExecTime/((uint64_t)1000),
            slot->functionDesc->wcet_ns/((uint64_t)1000));

    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_HAL_SINGLYLINKEDLIST_FINI(chunk->slots);
    xme_hal_mem_free(chunk);
    return XME_STATUS_SUCCESS;
}

#ifdef XME_CORE_RTESCHED_USE_SLEEP
//***************************************************************************//
/* Sleep-based implementation will run good under SCHED_FIFO conditions */
static void
delayTillStartOfNextSlot_usingSleep(const xme_hal_time_timeHandle_t timeToStartSlot,
                                    xme_hal_time_timeHandle_t* currentTime)
{
    xme_hal_time_timeHandle_t currentMoment;
    currentMoment = xme_hal_time_getCurrentTime();

    if (xme_hal_time_compareTime(currentMoment, timeToStartSlot) < 0)
    {
        xme_hal_time_timeInterval_t startDelay;
        startDelay = xme_hal_time_getTimeIntervalBetween(currentMoment, timeToStartSlot);
        XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "sleepsched: startDelay %" PRIu64 "\n", startDelay);
        if (startDelay > (uint64_t) XME_CORE_RTESCHED_HYBRID_WAIT_SLACK )
        {
            startDelay -= (uint64_t) XME_CORE_RTESCHED_HYBRID_WAIT_SLACK;
            xme_hal_sleep_sleep(startDelay);
        }
    }
    *currentTime = xme_hal_time_getCurrentTime();
}
#else

//***************************************************************************//
static void
delayTillStartOfNextSlot_usingBusyWait(const xme_hal_time_timeHandle_t timeToStartSlot,
                                       xme_hal_time_timeHandle_t* currentTime)
{
    do
    {
        *currentTime = xme_hal_time_getCurrentTime();
    }
    while (xme_hal_time_compareTime(*currentTime, timeToStartSlot) < 0);
}
#endif

//***************************************************************************//
uint32_t
xme_core_loop_executeAllSlots
(
    xme_core_loop_ChunkDescriptor_t* chunkDesc
)
{
    /* Timestamps for timing the schedule */
    xme_hal_time_timeHandle_t   chunkStart_ns,
                                timeToStartNext_ns;

    /* Timestamps for checking the execution time */
    xme_hal_time_timeHandle_t   startTime_ns,
                                endTime_ns;

    /* Start time for the chunk */
    chunkStart_ns = xme_hal_time_getCurrentTime();

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "Starting RTE chunk execution number %d at %" PRIu64"\n",
        chunkDesc->executionUnit->functionId,
        xme_hal_time_getTimeInterval(&chunkStart_ns, (bool) false));

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(chunkDesc->slots, xme_core_loop_SlotDescriptor_t, slotDesc);
    {

        XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "slot [%d|%d]@%"PRIu64"\n",
            slotDesc->functionDesc->componentId,
            slotDesc->functionDesc->functionId,
            slotDesc->startTime_ns);

        /* Time till triggering the next item */
        timeToStartNext_ns =
            xme_hal_time_offsetTime(
                    chunkStart_ns,
                    XME_HAL_TIME_OFFSET_OPERATION_ADD,
                    (slotDesc->startTime_ns - chunkDesc->startTime_ns)
        );

        XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "timeToStartNext %" PRIu64 "\n",
            xme_hal_time_getTimeInterval(&chunkStart_ns, (bool) false)/((uint64_t)1000));

#ifdef XME_CORE_RTESCHED_USE_SLEEP
        delayTillStartOfNextSlot_usingSleep(timeToStartNext_ns, &startTime_ns);
#else
        /* Busy-wait for the start of the slot: should be better under normal running mode */

        delayTillStartOfNextSlot_usingBusyWait(timeToStartNext_ns, &startTime_ns);
#endif

        XME_LOG(XME_LOG_DEBUG, MODULE_ACRONYM "TIMING: start time offset (us) for [%3d|%3d]: %10"PRIu64" -- real start time %10"PRIu64" vs expected %10"PRIu64"\n",
                slotDesc->functionDesc->componentId, slotDesc->functionDesc->functionId,
                xme_hal_time_getTimeIntervalBetween(timeToStartNext_ns, startTime_ns)/1000,
                xme_hal_time_getTimeIntervalBetween(chunkStart_ns, startTime_ns)/1000,
                xme_hal_time_getTimeIntervalBetween(chunkStart_ns, timeToStartNext_ns)/1000);

        if(NULL != xme_core_loop_intConfig.onFunctionActivate)
        {
        	xme_core_loop_intConfig.onFunctionActivate(slotDesc->functionDesc->componentId,
        			slotDesc->functionDesc->functionId);
        }

        slotDesc->functionDesc->task(slotDesc->functionDesc->taskArgs);

        if(NULL != xme_core_loop_intConfig.onFunctionReturned)
		{
        	xme_core_loop_intConfig.onFunctionReturned(slotDesc->functionDesc->componentId,
					slotDesc->functionDesc->functionId);
		}

        endTime_ns = xme_hal_time_getCurrentTime();

        XME_LOG(XME_LOG_VERBOSE,
            MODULE_ACRONYM "[%3d|%3d]: startTime: %" PRIu64 "us, endTime %" PRIu64 "us\n",
                slotDesc->functionDesc->componentId,
                slotDesc->functionDesc->functionId,
                xme_hal_time_getTimeIntervalBetween(chunkStart_ns, startTime_ns)/((uint64_t)1000),
                xme_hal_time_getTimeIntervalBetween(chunkStart_ns, endTime_ns)/((uint64_t)1000));

            slotDesc->nExecutions++;

            slotDesc->expAvgExecTime =
            (
                            (slotDesc->nExecutions-1)*(slotDesc->expAvgExecTime) + xme_hal_time_getTimeIntervalBetween(startTime_ns, endTime_ns)
            )
            /
            (slotDesc->nExecutions);

            XME_LOG(XME_LOG_DEBUG,
                            MODULE_ACRONYM "TIMING: RTE schedule: module function [%3d|%3d] execTime = %10"PRIu64" us, avg %10"PRIu64" us\n",
                            slotDesc->functionDesc->componentId,
                            slotDesc->functionDesc->functionId,
                            xme_hal_time_getTimeIntervalBetween(startTime_ns, endTime_ns)/(uint64_t)1000,
                            slotDesc->expAvgExecTime/(uint64_t)1000 );



        if (xme_hal_time_getTimeIntervalBetween(startTime_ns, endTime_ns) > slotDesc->functionDesc->wcet_ns)
        {
            XME_LOG(XME_LOG_NOTE,
                MODULE_ACRONYM "RTE schedule: module function %d|%d exceeded execution time with execTime = %"PRIu64"!\n",
                    slotDesc->functionDesc->componentId,
                    slotDesc->functionDesc->functionId,
                    xme_hal_time_getTimeIntervalBetween(startTime_ns, endTime_ns)/(uint64_t)1000);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "RTE chunk number %d: execution completed in %"PRIu64"\n",
            chunkDesc->executionUnit->functionId,
            xme_hal_time_getTimeInterval(&chunkStart_ns, (bool) false)/((uint64_t)1000));

    return 0;
}

//***************************************************************************//
extern
void xme_core_loop_execChunk(void* chunk)
{
	  xme_core_component_t cid;
	  xme_core_component_functionId_t fid;

  xme_core_exec_functionDescriptor_t * funDesc = (xme_core_exec_functionDescriptor_t*) chunk;
  xme_core_component_functionVariantId_t chunkVariant;

  xme_core_loop_ChunkDescriptor_t* chunkDesc =
                  (xme_core_loop_ChunkDescriptor_t*) xme_core_loop_getChunkDescriptor(funDesc->functionId);

  XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_initializeTask(funDesc->componentId, funDesc->functionId),
                ,
                XME_LOG_FATAL, MODULE_ACRONYM "could not initialize the chunk: %d\n",
                chunkDesc->id);

  cid = chunkDesc->executionUnit->componentId;
  fid = chunkDesc->executionUnit->functionId;

  XME_LOG( XME_LOG_DEBUG,
      MODULE_ACRONYM "RTE chunk started: %d|%d\n",
          cid,
          fid );

  while(XME_CORE_EXEC_FUNCTION_STATE_TERMINATED != funDesc->state)
  {
      XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_waitForStart(
              cid,
              fid,
              &chunkVariant),
          ,
          XME_LOG_FATAL,
          MODULE_ACRONYM "waitForStart returned an error in chunk %d\n",
          chunkDesc->id);

      if(XME_CORE_EXEC_FUNCTION_STATE_TERMINATED == funDesc->state)
      {

          XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_executionCompleted(
                          cid,
                          fid ),
                  ,
                  XME_LOG_FATAL,
                  MODULE_ACRONYM "executionCompleted returned an error in chunk %d\n",
                  chunkDesc->id);

          XME_LOG( XME_LOG_DEBUG,
              MODULE_ACRONYM "chunk terminated %d|%d\n",
                 cid, fid);
          return;
      }

      if(XME_CORE_EXEC_FUNCTION_STATE_PAUSED != funDesc->state)
      {
          XME_LOG( XME_LOG_DEBUG,
              MODULE_ACRONYM "execute chunk %d|%d\n",
                cid,
                fid );

          XME_CHECK_MSG (XME_STATUS_SUCCESS == xme_core_loop_executeAllSlots(chunkDesc),
                    ,
                    XME_LOG_ERROR,
                    MODULE_ACRONYM "error executing chunk %d!",
                    chunkDesc->id);
      }
      else
      {
        XME_LOG( XME_LOG_DEBUG,
            MODULE_ACRONYM "chunk paused %d|%d\n",
                cid,
                fid );
      }

      XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_executionCompleted(
                      cid,
                      fid ),
                      ,
                      XME_LOG_FATAL,
                      MODULE_ACRONYM "executionCompleted returned an error in chunk %d\n",
                      chunkDesc->id);
  }
}

//***************************************************************************//
// todo doc
extern
xme_status_t
xme_core_loop_addFunctionSlotToRteChunk
(
    xme_core_loop_Chunk_handle_t chunkId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_hal_time_timeInterval_t startTime_ns
)
{
    xme_core_loop_SlotDescriptor_t* slotDesc;
    xme_core_loop_ChunkDescriptor_t* chunkDesc;
    xme_core_exec_componentDescriptor_t* componentDesc;
    xme_core_exec_functionDescriptor_t* functionDesc;

    slotDesc = (xme_core_loop_SlotDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_loop_SlotDescriptor_t));

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_componentRepository_getComponent(componentId, &componentDesc),
              XME_STATUS_NOT_FOUND);

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_componentRepository_getComponentFunction(componentDesc, functionId, &functionDesc),
              XME_STATUS_NOT_FOUND);

    slotDesc->componentDesc=componentDesc;
    slotDesc->functionDesc=functionDesc;
    slotDesc->startTime_ns=startTime_ns;
    slotDesc->nExecutions=0;
    slotDesc->expAvgExecTime=0;

    chunkDesc = xme_core_loop_getChunkDescriptor(chunkId);

    XME_CHECK( NULL != chunkDesc,
            XME_STATUS_INTERNAL_ERROR );

    XME_CHECK
    (
        XME_STATUS_SUCCESS==XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(
                chunkDesc->slots,
                slotDesc,
                offsetof(xme_core_loop_SlotDescriptor_t,startTime_ns),
                uint64_t),
        XME_STATUS_OUT_OF_RESOURCES
    );

    return XME_STATUS_SUCCESS;
}
