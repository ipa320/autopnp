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
 * $Id: env.h 3158 2013-05-06 07:38:25Z ruiz $
 */

/**
 * \file
 * \brief Generic Environment Module.
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
 * \brief This function gets current executable path.
 *
 * \param filePath the file path.
 * \param size the size.
 *
 * \retval XME_STATUS_SUCCESS if the operation is completed.
 * \retval XME_STATUS_INTERNAL_ERROR if cannot complete operation. 
 */
extern xme_status_t 
xme_hal_env_getCurrentExecutablePath
(
	char * const filePath, 
	size_t size
); 

XME_EXTERN_C_END

/**
 * @}
 */

#endif /* XME_HAL_ENV_H */
