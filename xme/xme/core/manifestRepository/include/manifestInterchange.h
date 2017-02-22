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
 * $Id: manifestInterchange.h 7828 2014-03-14 09:32:09Z ruiz $
 */

/**
 * \file
 *         Manifest interchange.
 */

#ifndef XME_CORE_MANIFESTINTERCHANGE_H
#define XME_CORE_MANIFESTINTERCHANGE_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include "xme/core/manifestTypes.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Parses an input manifest.
 * 
 * \param[in] filename The file name in which is located the file to parse. 
 * \param[out] outComponentManifest The obtained component manifest after parsing. 
 *
 * \retval XME_STATUS_SUCCESS If parsing was success. 
 * \retval XME_STATUS_INVALID_PARAMETER If parameters are not valid parameters. 
 * \retval XME_STATUS_OUT_OF_RESOURCES If cannot parse the provided input content. 
 * \retval XME_STATUS_INTERNAL_ERROR If the provided file is not well-formed. 
 */
xme_status_t
xme_core_manifestInterchange_parseManifest
(
    const char* filename, 
    xme_core_componentManifest_t* outComponentManifest
);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_MANIFESTINTERCHANGE_H
