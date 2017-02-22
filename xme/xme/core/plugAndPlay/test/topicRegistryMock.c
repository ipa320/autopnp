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
 * $Id: topicRegistryMock.c 5588 2013-10-23 14:38:17Z wiesmueller $
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/directory/include/topicRegistry.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_directory_topicRegistry_getTopicSize
(
    xme_core_topic_t topic,
    uint16_t* const size
)
{
    XME_UNUSED_PARAMETER(topic);
    
    *size = 5;

    return XME_STATUS_SUCCESS;
}