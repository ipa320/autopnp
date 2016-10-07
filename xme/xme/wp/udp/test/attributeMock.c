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
 * $Id: attributeMock.c 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Attribute mock.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/directory/include/attribute.h"

#include "xme/core/topic.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief List of attribute descriptors for attrDescList.
 *
 * \details Described attributes:
 *          key | size in bytes | data type
 *          1 | 4 | uint32_t
 *          2 | 8 | uint64_t
 */
xme_core_attribute_descriptor_t attrDescs[2] =
{
    {
        SFINIT(key, (xme_core_attribute_key_t)1),
        SFINIT(size, (size_t)4),
    },
    {
        SFINIT(key, (xme_core_attribute_key_t)2),
        SFINIT(size, (size_t)8),
    },
};

/**
 * \brief The single known attribute descriptor list of this mock.
 */
xme_core_attribute_descriptor_list_t attrDescList =
{
    SFINIT(length, 2),
    SFINIT(element, attrDescs)
};

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
xme_status_t
xme_core_directory_attribute_getAttributeDescriptorList
(
    xme_core_topic_t topic,
    xme_core_attribute_descriptor_list_t* attributeDescriptorList
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
/**
 * \brief Always returns attrDescList.
 */
xme_status_t
xme_core_directory_attribute_getAttributeDescriptorList
(
    xme_core_topic_t topic,
    xme_core_attribute_descriptor_list_t* attributeDescriptorList
)
{
    XME_UNUSED_PARAMETER(topic);
    XME_UNUSED_PARAMETER(attributeDescriptorList);

    if ((xme_core_topic_t)2 == topic)
    {
        *attributeDescriptorList = attrDescList;

        return XME_STATUS_SUCCESS;
    }

    return XME_STATUS_NOT_FOUND;
}