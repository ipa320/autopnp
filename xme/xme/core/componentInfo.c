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
 * $Id: componentInfo.c 6308 2014-01-14 13:14:56Z geisinger $
 */

/**
 * \file
 *         Software component information.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/componentInfo.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

const char*
xme_core_component_getPortTypeString(xme_core_component_portType_t portType)
{
    // Must correspond to xme_core_component_portType_t enum
    static const char* const portTypeString[] =
    {
        "INVALID",
        "INTERNAL",
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        "DCC_PUB",
        "DCC_SUB",
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        "RR_RQS",
        "RR_RQH",
        "RR_RSS",
        "RR_RSH"
    };

    XME_ASSERT_RVAL(portType < sizeof(portTypeString)/sizeof(portTypeString[0]), NULL);

    return portTypeString[portType];
}
