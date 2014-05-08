/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: env.h 7278 2014-02-12 09:59:23Z geisinger $
 */

/**
 * \file
 * \brief Generic environment abstraction.
 */

#ifndef XME_HAL_ENV_H
#define XME_HAL_ENV_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the environment abstraction.
 *
 * \retval XME_CORE_STATUS_SUCCESS always.
 */
xme_status_t
xme_hal_env_init(void);

/**
 * \brief Frees resources occupied by the environment abstraction.
 */
void
xme_hal_env_fini(void);

/**
 * \brief Retrieves the fully qualified path of the executable file for the
 *        current process.
 *
 * \
 *
 * \param[in,out] filePath Memory address where to store the file path.
 * \param[in] size Size in bytes of the filePath memory buffer.
 *
 * \retval XME_STATUS_SUCCESS if the executable filename has been successfully
 *         retrieved.
 * \retval XME_STATUS_INVALID_PARAMETER if filePath was NULL.
 * \retval XME_STATUS_NOT_FOUND if the operation failed.
 */
xme_status_t
xme_hal_env_getCurrentExecutablePath
(
    char* const filePath, 
    size_t size
); 

/**
 * \brief Sets the title of the console window associated to the current
 *        process.
 *
 * \note This function has no effect on platforms where no console window
 *       concept is present.
 *
 * \param[in]
 */
void
xme_hal_env_setConsoleTitle
(
    const char* title
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif /* XME_HAL_ENV_H */
