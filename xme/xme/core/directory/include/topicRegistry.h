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
 * $Id: topicRegistry.h 5588 2013-10-23 14:38:17Z wiesmueller $
 */

/**
 * \file
 *         Topic registry.
 */

#ifndef XME_CORE_DIRECTORY_TOPICREGISTRY_H
#define XME_CORE_DIRECTORY_TOPICREGISTRY_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/topic.h"

#include "xme/defines.h"

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Initializes the topic component.
 *
 * \retval XME_STATUS_SUCCESS always.
 */
xme_status_t
xme_core_directory_topicRegistry_init(void);

/**
 * \brief Get the maximum data size in bytes for the given topic (not including
 *        the size of attributes).
 *
 * \details Before the data size of a topic can be queried with this function,
 *          it must be registered via xme_core_topic_registerTopicSize.
 *
 * \param[in] topic Topic identifier for which to return the size.
 * \param[out] size Buffer where data size will be written to.
 *
 * \retval XME_STATUS_SUCCESS if size has been retrieved successfully.
 * \retval XME_STATUS_NOT_FOUND when no data size is registered for the
 *         givent topic.
 * \retval XME_STATUS_INVALID_PARAMETER when size is NULL or topic
 *         is XME_CORE_TOPIC_INVALID_TOPIC.
 */
xme_status_t
xme_core_directory_topicRegistry_getTopicSize
(
    xme_core_topic_t topic,
    uint16_t* const size
);

/**
 * \brief Register the maximum data size in bytes for a topic (not including
 *        the size of attributes).
 *
 * \details After registration the size can be queried with xme_core_topic_getTopicSize.
 *
 * \param topic The topic for which to register the size.
 * \param size The data size in bytes.
 * \param overwrite When this is true an already existing entry will be
 *        overwritten. When false and an entry already exists, then 
 *        XME_STATUS_ALREADY_EXIST is returned and the operation is canceled.
 *
 * \retval XME_STATUS_SUCCESS if size has been registered successfully.
 * \retval XME_STATUS_OUT_OF_RESOURCES when there is not enough memory to register
 *         the size.
 * \retval XME_STATUS_INVALID_PARAMETER when given topic is XME_CORE_TOPIC_INVALID_TOPIC.
 * \retval XME_STATUS_ALREADY_EXIST when overwrite is false and an entry for the given
 *         topic already exists.
 */
xme_status_t
xme_core_directory_topicRegistry_registerTopicSize
(
    xme_core_topic_t topic,
    uint16_t size,
    bool overwrite
);

/**
 * \brief Frees all resources occupied by the topic component.
 */
void
xme_core_directory_topicRegistry_fini(void);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DIRECTORY_TOPICREGISTRY_H
