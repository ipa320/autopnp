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
 * $Id: sharedPtr_arch.h 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Shared memory abstraction (architecture specific part: generic OS
 *         based implementation).
 */

#ifndef XME_HAL_SHAREDPTR_ARCH_H
#define XME_HAL_SHAREDPTR_ARCH_H

#ifndef XME_HAL_SHAREDPTR_H
    #error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_HAL_SHAREDPTR_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"
#include "xme/hal/include/sync.h"

#include <stdint.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \def    XME_HAL_SHAREDPTR_MAX_BUFFER_ROWS
 *
 * \brief  Maximum number of supported shared pointer buffer rows.
 */
#define XME_HAL_SHAREDPTR_MAX_BUFFER_ROWS 50

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_hal_sharedPtr_bufferRow_t
 *
 * Stores data associated with a row in the shared memory buffer table.
 */
typedef struct
{
    void* slot_data; ///< Pointer to memory area.
    uint16_t size; ///< Size of allocated memory area.
    xme_hal_sharedPtr_referenceCount_t referenceCount; ///< Reference count.
}
xme_hal_sharedPtr_bufferRow_t;

/**
 * \struct xme_hal_sharedPtr_configStruct_t
 *
 * \brief  Shared memory configuration structure.
 */
XME_COMPONENT_CONFIG_STRUCT
(
    xme_hal_sharedPtr,
    // private
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle; ///< Critical section handle for protecting critical regions.
);

// TODO: change this to a efficient implementation!! See ticket #823
// TODO: Move to configuration structure. See ticket #823
extern xme_hal_sharedPtr_bufferRow_t xme_hal_sharedPtr_buffer[XME_HAL_SHAREDPTR_MAX_BUFFER_ROWS]; ///< the shared ptr buffer. 

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
extern xme_hal_sharedPtr_configStruct_t xme_hal_sharedPtr_config; ///< the configuration struct for shared ptr. 

#endif // #ifndef XME_HAL_SHAREDPTR_ARCH_H
