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
 * $Id: executionManagerUtils.c 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Utility functions for the whole executionManager.
 */

#define MODULE_ACRONYM "ExecMgr   : "
#define LOCKLOGGING 1
/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/core/log.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
bool
xme_core_exec_isValidFunctionDescriptor(xme_core_exec_functionDescriptor_t* desc)
{
    XME_CHECK_MSG( NULL != desc,
           false,
           XME_LOG_ERROR,
           MODULE_ACRONYM "isValidFunctionDescriptor: function descriptor is NULL!\n");

    XME_CHECK_MSG( XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != desc->functionId,
            false,
            XME_LOG_ERROR,
            MODULE_ACRONYM "isValidFunctionDescriptor: function descriptor [%d|X] has an invalid functionId!\n",
            desc->componentId);

    if(!(0 < desc->wcet_ns))
        XME_LOG(XME_LOG_WARNING,
            MODULE_ACRONYM "isValidFunctionDescriptor: function descriptor [%d|%d] has a zero wcet!\n",
            desc->componentId, desc->functionId);

    XME_CHECK_MSG( NULL != desc->task,
            false,
            XME_LOG_ERROR,
            MODULE_ACRONYM "isValidFunctionDescriptor: function descriptor [%d|%d] has no task pointer!\n",
            desc->componentId, desc->functionId);

    return true;
}

/***************************************************************************/
xme_core_exec_schedule_table_entry_t*
xme_core_exec_scheduler_findEntryInTable
(
    xme_core_exec_schedule_table_t* table,
    xme_core_component_t cid,
    xme_core_component_functionId_t fid
)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(table->entries,
        xme_core_exec_schedule_table_entry_t,
        loopItem);

        if((cid==loopItem->componentId) && (fid==loopItem->functionId))
            return loopItem;

    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return NULL;
}

#undef MODULE_ACRONYM
#define MODULE_ACRONYM "XME-EM-Sig: "
/****************************************************************************/
void
xme_core_exec_lockMutex
(
    const char* ID,
    xme_hal_sync_criticalSectionHandle_t handle,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
#ifdef LOCKLOGGING
    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "%3d %s waiting [%d|%d]\n",
        handle,
        ID, componentId, functionId);
#else
    XME_UNUSED_PARAMETER(ID);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
#endif
    xme_hal_sync_enterCriticalSection(handle);
#ifdef LOCKLOGGING
    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "%3d %s entered [%d|%d]\n",
        handle,
        ID, componentId, functionId);
#endif
}

/****************************************************************************/
void
xme_core_exec_unlockMutex
(
    const char* ID,
    xme_hal_sync_criticalSectionHandle_t handle,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    xme_hal_sync_leaveCriticalSection(handle);
#ifdef LOCKLOGGING
    XME_LOG(XME_LOG_DEBUG,
        MODULE_ACRONYM "%3d %s [%d|%d] rel\n",
        handle,
        ID, componentId, functionId);
#else
    XME_UNUSED_PARAMETER(ID);
    XME_UNUSED_PARAMETER(componentId);
    XME_UNUSED_PARAMETER(functionId);
#endif
}
