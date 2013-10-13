/* 
 *
 * Copyright (c) 2012-2013fortiss GmbH
 *
 *
 * $Id: executionManagerCallback.h 5131 2013-09-20 08:45:18Z camek $
 */

#ifndef _XME_CORE_EXECUTIONMANAGER_CALLBACK_H_
#define _XME_CORE_EXECUTIONMANAGER_CALLBACK_H_

/*-------------------------------------------------------------------------*/
/**
 * \typedef xme_core_exec_initCallback_t
 * \brief Callback function type for component-global initialization.
 */
typedef xme_status_t (*xme_core_exec_initCallback_t) ( void* );

/*-------------------------------------------------------------------------*/
/**
 * \typedef xme_core_exec_finiCallback_t
 * \brief Callback function type for component-global finalization.
 */
typedef void (*xme_core_exec_finiCallback_t) (void);

/**
 * \typedef xme_hal_sched_taskCallback_t
 *
 * \brief  Task callback function.
 */
typedef void (*xme_hal_sched_taskCallback_t) (void* userData);


#endif /* _XME_CORE_EXECUTIONMANAGER_CALLBACK_H_ */
