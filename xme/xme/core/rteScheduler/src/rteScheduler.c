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
 * $Id: rteScheduler.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Implementation of the RTE scheduler.
 */

#define MODULE_ACRONYM "RteSched  :"

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/rteScheduler/include/coreLoopConfig.h"
#include "xme/core/rteScheduler/include/coreLoopChunk.h"
#include "xme/core/rteScheduler/include/coreLoopScheduler.h"

#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/core/executionManager/include/internDescriptorTable.h"
#include "xme/core/log.h"

/* HAL */
#include "xme/hal/include/linkedList.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/time.h"

/* XME core*/
#include "xme/defines.h"

#include "xme/xme_opt.h"

/* stdlib */
#include <stddef.h>

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static
struct xme_core_loop_SchedulerData_s
{
    // todo: initialize, deinitialize, etc.
    xme_hal_singlyLinkedList_t(XME_CORE_EXEC_MAX_RTE_FUNCTIONS) initQueue;

    // todo: initialize, deinitialize, etc
    xme_hal_singlyLinkedList_t(XME_CORE_EXEC_MAX_RTE_FUNCTIONS) chunks;
} xme_core_loop_SchedulerData;

/******************************************************************************/
/***   Globals                                                              ***/
/******************************************************************************/
xme_core_loop_configStruct_t xme_core_loop_intConfig;

/****************************************************************************/
/*  Prototypes  */
/****************************************************************************/
static
xme_status_t
xme_core_loop_completeInitialization( void );

static
xme_status_t
xme_core_loop_completeChunkInit
(
       xme_core_loop_ChunkDescriptor_t* const chunk
);

/**
 * \brief Adds a chunk to RTE schedule
 * \param chunk Descriptor of the chunk to be added.
 * \return Returns one of the following status codes:
 *          - XME_STATUS_SUCCESS if the item has been added successfully.
 *          - XME_STATUS_OUT_OF_RESOURCES if the linked list was not
 *               capable to add an element.
 */
static
xme_status_t
xme_core_loop_addChunk(xme_core_loop_ChunkDescriptor_t* chunk);

extern xme_status_t
xme_core_loop_finiChunk
(
        xme_core_loop_ChunkDescriptor_t* chunk
);

extern xme_status_t
xme_core_loop_checkRteChunkSchedule
(
        const xme_core_loop_ChunkDescriptor_t* const scheduleChunk
);

extern xme_core_loop_ChunkDescriptor_t*
xme_core_loop_initChunk( void );

/****************************************************************************/
/***   Implementation                                                     ***/
/****************************************************************************/
extern
xme_status_t
xme_core_loop_init( xme_core_loop_configStruct_t* initConfig)
{
    /* Initialize with default behavior */
    xme_core_loop_intConfig.onFunctionActivate = NULL;
    xme_core_loop_intConfig.onFunctionReturned = NULL;

    if(NULL != initConfig)
    {
        (void) xme_hal_mem_copy(
                    &xme_core_loop_intConfig,
                    initConfig,
                    sizeof(xme_core_loop_configStruct_t));
    }

    XME_HAL_SINGLYLINKEDLIST_INIT(xme_core_loop_SchedulerData.initQueue);
    XME_HAL_SINGLYLINKEDLIST_INIT(xme_core_loop_SchedulerData.chunks);

    /* Registration of the RTE modules */
    XME_CHECK_MSG( true == xme_core_loop_registerModulesCallback(),
            XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_FATAL,
            MODULE_ACRONYM "Could not register RTE modules!\n");

    /* A configuration routine to define RTE "chunks" - the basic blocks of
         RTE module execution on OS level */
    XME_CHECK_MSG( true == xme_core_loop_createChunksCallback(),
            XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_FATAL,
            MODULE_ACRONYM "Could not create RTE chunks!\n");

    /* Check the chunks for plausibility and register in EM */
    XME_CHECK_MSG( XME_STATUS_SUCCESS == xme_core_loop_completeInitialization(),
            XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_FATAL,
            MODULE_ACRONYM "Could not complete RTE initialization!\n");

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(xme_core_loop_SchedulerData.initQueue,
            xme_core_exec_componentDescriptor_t,
            compToInit);

    if(compToInit->autoInit)
    {
        XME_LOG(XME_LOG_NOTE,
            MODULE_ACRONYM "RTE scheduler: initialize component %3d with priority %5d\n",
            compToInit->componentId,
            compToInit->initPriority);

        XME_CHECK_MSG(XME_STATUS_SUCCESS ==
                        compToInit->init(compToInit->initParam),
            XME_STATUS_INVALID_CONFIGURATION,
            XME_LOG_FATAL,
            MODULE_ACRONYM "Failure during initialization of the component %d\n",
            compToInit->componentId);
    }

    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

     XME_CHECK( true == xme_core_loop_activateScheduleCallback(),
         XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

/*****************************************************************************/
extern xme_status_t
xme_core_loop_fini( void )
{
    XME_LOG(XME_LOG_ERROR, "[cid|fid] exCnt  avgRuntime us confWcet us\n");

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(xme_core_loop_SchedulerData.chunks,
        xme_core_loop_ChunkDescriptor_t,
        chunkDesc
    );
    {
        if(XME_STATUS_SUCCESS != xme_core_loop_finiChunk(chunkDesc))
        {
            XME_LOG(XME_LOG_WARNING, "error finalizing chunk %d\n", chunkDesc->id);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_HAL_SINGLYLINKEDLIST_FINI(xme_core_loop_SchedulerData.chunks);
    XME_HAL_SINGLYLINKEDLIST_FINI(xme_core_loop_SchedulerData.initQueue);
    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
extern xme_status_t
xme_core_loop_addChunkToGlobalSchedule
(
    xme_core_exec_schedule_handle_t scheduleId,
    xme_core_loop_Chunk_handle_t chunkId
)
{
    xme_hal_sched_taskHandle_t taskHandle;
    xme_core_exec_schedule_table_t* schedule = NULL;
    xme_core_loop_ChunkDescriptor_t* chunkDesc = xme_core_loop_getChunkDescriptor(chunkId);
    XME_CHECK( NULL != chunkDesc,
        XME_STATUS_NOT_FOUND);

    // If a chunk has already been registered, do not do it
    if(XME_STATUS_NOT_FOUND == xme_core_exec_dispatcher_getRunnable(chunkDesc->executionUnit->componentId, chunkDesc->executionUnit->functionId,&taskHandle))
    {
        XME_LOG(XME_LOG_DEBUG,
            MODULE_ACRONYM "addToGlobalSchedule(%d, %d)\n", scheduleId, chunkDesc->id);
        XME_CHECK_MSG(XME_STATUS_SUCCESS == xme_core_exec_dispatcher_createFunctionExecutionUnit(
                                chunkDesc->executionUnit, (bool) false),
                      XME_STATUS_INTERNAL_ERROR,
                      XME_LOG_ERROR,
                      MODULE_ACRONYM "execution unit could not be started\n");
    }

    XME_CHECK(XME_STATUS_SUCCESS == xme_core_exec_scheduler_getSchedule(scheduleId, &schedule),
        XME_STATUS_NOT_FOUND);

    return xme_core_exec_scheduler_addElementToScheduleTable(
        schedule, RTE_BASE_COMPONENT_ID,
        chunkDesc->id,
        (xme_core_component_functionVariantId_t)(chunkDesc->id),
        chunkDesc->startTime_ns,
        chunkDesc->endTime_ns-chunkDesc->startTime_ns,
        0, 0,
        (bool) true );
}

/****************************************************************************/
extern xme_status_t
xme_core_loop_allocateRteChunk
(
	xme_hal_time_timeInterval_t startTime_ns,
	xme_hal_time_timeInterval_t endTime_ns,
    xme_core_loop_Chunk_handle_t* chunkId
)
{
    xme_core_loop_ChunkDescriptor_t* chunkDesc = NULL;
	chunkDesc = xme_core_loop_initChunk();
    XME_CHECK(NULL != chunkDesc,
            XME_STATUS_OUT_OF_RESOURCES);
    chunkDesc->startTime_ns = startTime_ns;
    chunkDesc->endTime_ns = endTime_ns;
	XME_CHECK(XME_STATUS_SUCCESS == xme_core_loop_addChunk(chunkDesc),
        XME_STATUS_INTERNAL_ERROR);
    *chunkId = chunkDesc->id;
    return XME_STATUS_SUCCESS;
}

/******************************************************************************/
static xme_status_t
xme_core_loop_addChunk(xme_core_loop_ChunkDescriptor_t* chunk)
{

    XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(xme_core_loop_SchedulerData.chunks, chunk),
	    XME_STATUS_OUT_OF_RESOURCES);
    chunk->id = XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(xme_core_loop_SchedulerData.chunks);
    return XME_STATUS_SUCCESS;
}

/******************************************************************************/
extern xme_core_loop_ChunkDescriptor_t*
xme_core_loop_getChunkDescriptor(xme_core_loop_Chunk_handle_t id)
{
    return XME_HAL_SINGLYLINKEDLIST_ITEM_FROM_INDEX(xme_core_loop_SchedulerData.chunks, id-1);

}

/******************************************************************************/
static xme_status_t
xme_core_loop_addToInitQueue
(
        xme_core_exec_componentDescriptor_t* componentDesc
)
{
    if(!componentDesc->autoInit)
        return XME_STATUS_SUCCESS;

    /* look up the component descriptor in the startup queue */
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(xme_core_loop_SchedulerData.initQueue,
            xme_core_exec_componentDescriptor_t,
            listComponent);

    if(listComponent->componentId == componentDesc->componentId)
    {
        return XME_STATUS_ALREADY_EXIST;
    }

    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM_SORTED(
                            xme_core_loop_SchedulerData.initQueue,
                            componentDesc,
                            offsetof(xme_core_exec_componentDescriptor_t, initPriority),
                            uint32_t),
              XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

/******************************************************************************/
static xme_status_t
xme_core_loop_completeInitialization( void )
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(xme_core_loop_SchedulerData.chunks, xme_core_loop_ChunkDescriptor_t, chunk);
    XME_CHECK( XME_STATUS_SUCCESS == xme_core_loop_completeChunkInit(chunk),
            XME_STATUS_INVALID_CONFIGURATION);
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    return XME_STATUS_SUCCESS;
}

/******************************************************************************/
static
xme_status_t
xme_core_loop_completeChunkInit
(
        xme_core_loop_ChunkDescriptor_t* const chunk
)
{
    xme_core_exec_functionDescriptor_t* newFunctionDesc;

    /* Verify if the schedule does not contain intersections and compute wcet*/
    XME_CHECK(XME_STATUS_SUCCESS == xme_core_loop_checkRteChunkSchedule(chunk),
        XME_STATUS_INVALID_CONFIGURATION);

    /* Create a function descriptor for an RTE chunk.
     * This will be the descriptor of our executable unit. */
    newFunctionDesc =
            (xme_core_exec_functionDescriptor_t*) xme_hal_mem_alloc(
                    sizeof(xme_core_exec_functionDescriptor_t)
                    );

    /* Ugly reassignments, thanks MS for C99 support */
    newFunctionDesc->componentId = RTE_BASE_COMPONENT_ID;
    newFunctionDesc->functionId = chunk->id;
            newFunctionDesc->task = &xme_core_loop_execChunk;
            newFunctionDesc->taskArgs = (void*)newFunctionDesc;
            newFunctionDesc->wcet_ns = chunk->wcet_ns;
            newFunctionDesc->state = XME_CORE_EXEC_FUNCTION_STATE_INVALID_STATE;

    chunk->executionUnit = newFunctionDesc;

    /* Add the respective RTE components to the init queue */
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN( chunk->slots, // list to iterate over
            xme_core_loop_SlotDescriptor_t,
            slot ); // iterator variable
        if(XME_STATUS_SUCCESS != xme_core_loop_addToInitQueue(slot->componentDesc))
        {
            XME_LOG(XME_LOG_NOTE, MODULE_ACRONYM "slot [%d|%d] has not been added to startup queue\n",
                    slot->functionDesc->componentId,
                    slot->functionDesc->functionId);
        }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_SUCCESS;
}
