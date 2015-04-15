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
 * $Id: graphviz.h 6616 2014-02-04 13:33:19Z geisinger $
 */

/** 
 * \file
 * \brief Graphviz abstraction.
 *
 */

#ifndef XME_HAL_GRAPHVIZ_H
#define XME_HAL_GRAPHVIZ_H

/** 
 * \defgroup hal_graphviz Graphviz.
 * @{ 
 *
 * \brief Graphviz abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Initializes graphviz for this node, if not already done.
 *
 * \retval XME_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_STATUS_INTERNAL_ERROR if something went wrong.
 */
xme_status_t
xme_hal_graphviz_init(void);

/**
 * \brief Generates an image out of the given dot file.
 *
 * \param[in] dotFilePath Path to dot file which shall be transformed into an image.
 * \param[in] pngFilePath Path of the image to generate.
 *
 * \retval XME_STATUS_SUCCESS if the operation completed successfully.
 * \retval XME_STATUS_INTERNAL_ERROR if something went wrong.
 */
xme_status_t
xme_hal_graphviz_generateImage
(
    const char* dotFilePath,
    const char* pngFilePath
);

/**
 * \brief  Finalizes graphviz for this node, if no one else is using graphviz
 *         anymore.
 */
void
xme_hal_graphviz_fini(void);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_HAL_GRAPHVIZ_H
