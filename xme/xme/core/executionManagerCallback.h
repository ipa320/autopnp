/* 
 *
 * Copyright (c) 2012-2013fortiss GmbH
 *
 *
 * $Id: executionManagerCallback.h 6464 2014-01-24 11:59:34Z kainz $
 */

#ifndef _XME_CORE_EXECUTIONMANAGER_CALLBACK_H_
#define _XME_CORE_EXECUTIONMANAGER_CALLBACK_H_

/*-------------------------------------------------------------------------*/
/**
 * \typedef xme_core_exec_initCallback_t
 *
 * \brief Callback function type for component-global initialization.
 *
 * \param[in,out] componentConfig Pointer to the component-specific configuration
 *                structure of this function's parent component instance.
 *                The configuration is first passed to the component instance
 *                initialization function and then subsequently to the
 *                initialization functions of all active functions of the
 *                component. It can be freely modified by this function.
 */
typedef xme_status_t (*xme_core_exec_initCallback_t) (void* const componentConfig);

/*-------------------------------------------------------------------------*/
/**
 * \typedef xme_core_exec_finiCallback_t
 *
 * \brief Callback function type for component-global finalization.
 *
 * \param[in,out] componentConfig Pointer to the component-specific configuration
 *                structure of this function's parent component instance.
 *                The finalization function should free all function-specific
 *                resources in the configuration structure and update it accordingly.
 */
typedef void (*xme_core_exec_finiCallback_t) (void* const componentConfig);

/**
 * \typedef xme_hal_sched_taskCallback_t
 *
 * \brief  Task callback function.
 */
typedef void (*xme_hal_sched_taskCallback_t) (void* userData);

#endif /* _XME_CORE_EXECUTIONMANAGER_CALLBACK_H_ */
