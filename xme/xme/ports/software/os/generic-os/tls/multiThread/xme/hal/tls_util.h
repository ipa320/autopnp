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
 * $Id: tls_util.h 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Thread-local storage abstraction.
 *
 */

/**
 * \addtogroup hal_tls
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/tls.h"

#include "xme/hal/include/table.h"
#include "xme/hal/include/sync.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \struct xme_hal_tls_item_t
 *
 * \brief  Thread-local storage descriptor.
 */
typedef struct
{
    xme_hal_tls_index_t index; ///< Platform dependent thread-local storage index.
    uint16_t size; ///< Requested size of the memory at the given thread-local storage index.
}
xme_hal_tls_item_t;

/**
 * \struct xme_hal_tls_configStruct_t
 *
 * \brief  Thread-local storage configuration structure.
 */
typedef struct
{
    unsigned int initializationCount; ///< Number of times this component has been initialized.
    xme_hal_sync_criticalSectionHandle_t criticalSectionHandle; ///< the critical section handle. 
    XME_HAL_TABLE(xme_hal_tls_item_t, items, XME_HAL_DEFINES_MAX_TLS_ITEMS); ///< the max tls items table. 
}
xme_hal_tls_configStruct_t;

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
extern xme_hal_tls_configStruct_t xme_hal_tls_config;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief inits the thread local storage.
 *
 * \retval XME_STATUS_SUCCESS if the TLS is successfully initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the TLS cannot be initialized.
 */
xme_status_t
_xme_hal_tls_init(void);

/**
 * \brief inits the thread local storage.
 *
 * \retval XME_STATUS_SUCCESS if the TLS is successfully initialized. 
 * \retval XME_STATUS_INTERNAL_ERROR if the TLS cannot be initialized.
 */
void
_xme_hal_tls_fini(void);

/**
 * \brief allocates memory for adding an item to the thread local storage.
 *
 * \param tlsHandle the thread local storage handle. 
 * 
 * \return the pointer to the created memory location for the new item.
 */
xme_hal_tls_item_t*
_xme_hal_tls_addItem
(
    xme_hal_tls_handle_t* tlsHandle
);

/**
 * \brief gets the current item from the thread local storage.
 *
 * \param tlsHandle the thread local storage handle. 
 * 
 * \return the current item obtained from the TLS.
 */
xme_hal_tls_item_t*
_xme_hal_tls_getItem
(
    xme_hal_tls_handle_t tlsHandle
);

/**
 * \brief Removes an item from the thread local storage.
 *
 * \param tlsHandle the thread local storage handle. 
 * 
 * \retval XME_STATUS_SUCCESS if the current TLS item is successfully removed. 
 * \retval XME_STATUS_INTERNAL_ERROR if the current TLS item cannot be removed.
 */
xme_status_t
_xme_hal_tls_removeItem
(
    xme_hal_tls_handle_t tlsHandle
);

/**
 * \brief Gets the size of the thread local storage.
 *
 * \param tlsHandle the thread local storage handle. 
 *
 * \return the size of the TLS. 
 */
uint16_t
_xme_hal_tls_getSize
(
    xme_hal_tls_handle_t tlsHandle
);

/**
 * @}
 */

