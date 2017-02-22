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
 * $Id: executionManagerScheduleManagementInterface.h 5157 2013-09-24 16:29:43Z rupanov $
 */

/**
 * \file
 *         Execution Manager schedule management interface.
 *
 * \brief  Schedule manipulation functions are responsible for allocating and
 *      freeing all required structures for schedules, adding elements to
 *      schedule, and copying schedules.
 *
 * A time-triggered schedule is a simple list of entries containing information
 *     on *what* and *when* to execute.
 * - What. In Chromosome a runnable is unambigously identified via a (componentId, functionId) pair.
 * - When. Slot start should be explicitely defined, as well as slot length (for sanity checks).
 */

#ifndef XME_CORE_EXEC_SCHEDULE_MANAGEMENT_INTERFACE_H
#define XME_CORE_EXEC_SCHEDULE_MANAGEMENT_INTERFACE_H

/**
 * \ingroup core_em Execution Manager
 * @{
 *
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/executionManager/include/executionManager.h"
#include "xme/core/component.h"
#include "xme/hal/include/linkedList.h"
#include "xme/defines.h"

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//
/**
 * \typedef xme_core_exec_schedule_handle_t
 *
 * \brief Schedule handle in the EM schedule set.
 *
 */
typedef enum xme_core_exec_schedule_handle_
{
    XME_CORE_EXEC_SCHEDULE_HANDLE_DEFAULT = 0x0, ///< Default schedule to be activated if no implicit `xme_core_exec_scheduler_activateSchedule()` call has been issued before dispatcher loop activation.
    XME_CORE_EXEC_SCHEDULE_HANDLE_INVALID = 0xFFFFFFFF ///< Invalid schedule - to be used as a default initializer
}
xme_core_exec_schedule_handle_t;

/*-------------------------------------------------------------------------*/
/**
 * \typedef xme_core_exec_schedule_task_handle_t
 *
 * \brief Task handle in the EM schedule table.
 *
 */
typedef enum xme_core_exec_schedule_taskHandle_
{
    XME_CORE_EXEC_SCHEDULE_TASKHANDLE_DEFAULT = 0x0, ///< Default schedule to be activated if no implicit `xme_core_exec_scheduler_activateSchedule()` call has been issued before dispatcher loop activation.
    XME_CORE_EXEC_SCHEDULE_TASKHANDLE_INVALID = 0xFFFFFFFF ///< Invalid schedule - to be used as a default initializer
}
xme_core_exec_schedule_taskHandle_t;

/*-------------------------------------------------------------------------*/
/**
 * \typedef xme_core_exec_schedule_table_entry_t
 *
 * \brief Schedule entry for time-triggered fixed-slot schedule.
 *
 */
typedef struct xme_core_exec_schedule_table_entry_
{
    xme_core_component_t componentId; ///< Locally unique component Id
    xme_core_component_functionId_t functionId; ///< Identifier of a function within component
    void* functionArgs; ///< Arguments of function

    xme_hal_time_timeInterval_t slotStart_ns; ///< Slot start in nanoseconds / inside the cycle
    xme_hal_time_timeInterval_t slotLength_ns; ///< Slot length in nanoseconds
    uint32_t periodDivider; ///< Number of major cycles to be set as a period for this slot
    uint32_t periodDividerOffset; ///< Offset to the value of cycle counter

    bool interruptable; ///< Indicates if a function could be interrupted if another function becomes ready (? preemptable)
    bool completion;    ///< Indicates if a function should complete in slot time window
} xme_core_exec_schedule_table_entry_t;

/*-------------------------------------------------------------------------*/
/**
 * \struct xme_core_exec_schedule_table_t
 *
 * \brief A scheduling table for a node.
 *
 * Currently a scheduling table defines major cycle length and a list of task
 * entries to be executed in the sequence of their occurence. Therefore a 
 * precondition is that the schedule table entries are sorted by start time, 
 * and that no intersections exist in between the schedule entries.
 *
 */
typedef struct xme_core_exec_schedule_table_
{
    xme_hal_linkedList_descriptor_t entries; ///< The schedule table itself
    xme_hal_time_timeInterval_t majorCycleDuration_ns; ///< Duration of a cycle
} xme_core_exec_schedule_table_t;


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/*-------------------------------------------------------------------------*/
/**
 * \brief     Makes a copy of current schedule table set for configurator purposes
 *
 */
extern xme_status_t
xme_core_exec_scheduler_replicateFullScheduleSet
(

    xme_hal_linkedList_descriptor_t* source,
    xme_hal_linkedList_descriptor_t* target
);

/*-------------------------------------------------------------------------*/


extern xme_status_t
xme_core_exec_scheduler_overwriteFullScheduleSet
(
    xme_hal_linkedList_descriptor_t* source
);

/*-------------------------------------------------------------------------*/

extern xme_status_t
xme_core_exec_scheduler_addFunctionToScheduleSet
(
        xme_hal_linkedList_descriptor_t* scheduleSet,
        xme_hal_linkedList_descriptor_t* scheduleIds,
        xme_core_component_t componentId,
        xme_core_component_functionId_t functionId,
        void* functionArgs,
        xme_hal_time_timeInterval_t fromTime_ns,
        xme_hal_time_timeInterval_t toTime_ns,
        uint32_t periodDivider,
    	uint32_t periodDividerOffset
);

/*-------------------------------------------------------------------------*/

extern xme_status_t
xme_core_exec_scheduler_findWindow
(
    xme_core_exec_schedule_table_t* schedule,
    xme_hal_time_timeInterval_t wcet_ns,
    xme_hal_time_timeInterval_t fromTime_ns,
    xme_hal_time_timeInterval_t toTime_ns,
    xme_hal_time_timeInterval_t* windowStart_ns
);

/*-------------------------------------------------------------------------*/
/**
 * \brief Creates a copy of a schedule.
 */
extern xme_status_t
xme_core_exec_scheduler_replicateSchedule(
    xme_core_exec_schedule_table_t**     target,
    xme_core_exec_schedule_table_t*     source
);


/*-------------------------------------------------------------------------*/
/** \brief  */
extern xme_status_t
xme_core_exec_scheduler_autoAllocateInScheduleTable
(
    xme_core_exec_schedule_table_t*     schedule,
    xme_core_component_t                 componentId,
    xme_core_component_functionId_t     functionId,
    void*                              functionArgs,
    uint32_t periodDivider,
	uint32_t periodDividerOffset
);

/*-------------------------------------------------------------------------*/
/**
 * \brief   Creates a schedule table.
 *
 * Allocates a schedule and initializes it to usable state.
 *
 * \param   schedule A pointer to a schedule pointer variable to be initialized by the newly allocated schedule.
 *
 * \param   majorCycleLength_ns Major period of the schedule in nanoseconds.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if schedule allocation and initialization succeeded.
 *          - XME_CORE_STATUS_OUT_OF_RESOURCES in case a new schedule could not be allocated.
 */
extern xme_status_t
xme_core_exec_scheduler_createScheduleTable
(
    xme_core_exec_schedule_table_t** schedule,
    xme_hal_time_timeInterval_t majorCycleLength_ns
);

/*-------------------------------------------------------------------------*/
/**
 * \brief   Clears a task schedule table.
 *
 * \param   schedule A pointer to a schedule variable to be deleted.
 * \param   destroy Indicates, whether table itself has to be deleted as well.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the destructive action succeeds. No other options.
 */
extern xme_status_t
xme_core_exec_scheduler_clearScheduleTable
(
    xme_core_exec_schedule_table_t** schedule,
    bool destroy
);

/*-------------------------------------------------------------------------*/
/**
 * XXX such comments are not very good. please make this really a piece of documentation
 *
 *
 * \brief   Adds a time slot into a scheduling table.
 *
 * \param   schedule A pointer to a schedule variable to be appended.
 *
 * \param   componentId Locally unique identifier of the component,
 *          to which the function belongs.
 *
 * \param   functionId Identifier of the function within a component.
 *
 * \param   functionArguments Arguments to pass to the function.
 *
 * \param   slotStart_ns Start time for the time slot (relative to cycle start) in *nanoseconds*.
 *
 * \param   slotLength_ns Duration of the time slot in *nanoseconds*.
 *     todo    xxx: slotLength_ns should come from the registered component descriptor
 * \param   periodDivider Given a periodDivider=n, periodDividerOffset=k the function will get executed every n cycles with offset k, periodDivider=0 lets ignore both divider and offset
 * \param	periodDividerOffset See periodDivider
 * \param   completion Defines whether execution has to be completed by the
 *          end of the slot (usually this is true)
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if timeslot structure has been created and added to the schedule.
 *          - XME_CORE_STATUS_INVALID_PARAMETER if one of parameters is definitely wrong
 *          - XME_CORE_STATUS_OUT_OF_RESOURCES in case memory allocation failed.
 */
extern xme_status_t
xme_core_exec_scheduler_addElementToScheduleTable
(
    xme_core_exec_schedule_table_t* schedule,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArguments,
    xme_hal_time_timeInterval_t slotStart_ns,
    xme_hal_time_timeInterval_t slotLength_ns,
    uint32_t periodDivider,
	uint32_t periodDividerOffset,
    bool completion
);
/** 
 * \brief Add the schedule table to the scheduler registered schedules list
 *
 * \param [in]  schedule A pointer to a schedule table to be registered
 * \param [out] scheduleId A handle of the schedule table to uniquely identify 
 *                      the table in future operations
 * 
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the schedule has been successfully 
                    added to the registered list.
 *          - XME_CORE_STATUS_INTERNAL_ERROR if the schedule ID is incorrect, 
                    so something really bad has happened
 *          - XME_CORE_STATUS_OUT_OF_RESOURCES if a schedule could not be 
 *                  added to the registered list because of memory allocation 
                    problems. 
 */
extern xme_status_t
xme_core_exec_scheduler_registerSchedule
(
     xme_core_exec_schedule_table_t* schedule,
     xme_core_exec_schedule_handle_t* scheduleId
);


/** 
 * \brief   Adds a component function to the schedule.
 *
 * \param   schedule A handle of the schedule to be appended.
 *
 * \param   componentId Locally unique identifier of the component,
 *          to which the function belongs.
 *
 * \param   functionId Identifier of the function within a component.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the function has beed successfully
                 added to the schedule.
 *          - XME_CORE_STATUS_NOT_FOUND if a schedule with such ID has not 
                 been found.
 *          - XME_CORE_STATUS_OUT_OF_RESOURCES if no memory could be allocated 
                 for the timeslot structure *or* no free window of size 
                 `wcet_ns` could be found in the target schedule.
 */
extern xme_status_t
xme_core_exec_scheduler_addFunctionToSchedule
(
    xme_core_exec_schedule_handle_t schedule,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArguments,
    uint32_t periodDivider,
	uint32_t periodDividerOffset
);

/** 
 * \brief   Adds a component function to the schedule.
 *
 * \param   schedule A handle of the schedule to be appended.
 *
 * \param   componentId Locally unique identifier of the component,
 *          to which the function belongs.
 *
 * \param   functionId Identifier of the function within a component.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the function has beed successfully
                 added to the schedule.
 *          - XME_CORE_STATUS_NOT_FOUND if a schedule with such ID has not 
                 been found.
 *          - XME_CORE_STATUS_OUT_OF_RESOURCES if no memory could be allocated 
                 for the timeslot structure *or* no free window of size 
                 `wcet_ns` could be found in the target schedule.
 */
extern xme_status_t
xme_core_exec_scheduler_addFunctionToScheduleAt
(
    xme_core_exec_schedule_handle_t schedule,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void* functionArguments,
    xme_hal_time_timeInterval_t startTime_ns
);

/** 
 * \brief Get the registered schedule structure by id
 * \param [in]  scheduleId ID of the schedule to look up
 * \param [out] schedule    A pointer to the schedule structure
 * 
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the return pointer is valid.
 *          - XME_CORE_STATUS_NOT_FOUND if no schedule with such index could 
 *               be found 
 */
extern xme_status_t
xme_core_exec_scheduler_getSchedule
(
     xme_core_exec_schedule_handle_t scheduleId,
     xme_core_exec_schedule_table_t** schedule
);

extern xme_core_exec_schedule_table_entry_t*
xme_core_exec_scheduler_findEntryInTable
(
    xme_core_exec_schedule_table_t* table,
    xme_core_component_t cid,
    xme_core_component_functionId_t fid
);


/** 
 * \brief Activate a previously registered schedule starting from next major
 *          cycle.
 * \param scheduleId A handle to the schedule (ID of the schedule) to be 
 *          activated
 *
 * \note It is expected that the dispatcher loop is IMMEDIATELY started after 
 *         the very first schedule activation call.
 * 
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the schedule ID is correct, and 
                    schedule change should occur on next cycle
            - XME_CORE_STATUS_INVALID_PARAMETER if the schedule ID is wrong. 
 */
extern void
xme_core_exec_scheduler_activateSchedule
(
 xme_core_exec_schedule_handle_t scheduleId
);

/*
 * \brief This function returns the current value of the cycle counter.
 */
uint32_t
xme_core_exec_scheduler_getCycleCounter(void);

XME_EXTERN_C_END

/**
 * @}
 */

#endif /* #ifdef XME_CORE_EXEC_SCHEDULE_MANAGEMENT_INTERFACE_H */
