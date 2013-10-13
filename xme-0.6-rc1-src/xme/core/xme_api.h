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
 * $Id: xme_api.h 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 *         XME-API as struct of pointers to API functions.
 *         Intended to be used inside dynamically loadable libraries to access XME functions.
 */

#ifndef XME_CORE_API_H
#define XME_CORE_API_H


/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#ifdef PRE_IL1_SUPPORT
#include "xme/core/dcc.h"
#include "xme/core/resourceManager.h"
#include "xme/core/componentList.h"
#endif // #ifdef PRE_IL1_SUPPORT
#include "xme/core/log.h"

// For dataHandler functions
#include "xme/core/dataHandler/include/dataHandler.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

#define XME_CORE_API_VERSION 0x0101 ///< the API version.

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

#ifdef PRE_IL1_SUPPORT

//TODO(KB): move these typedefs to the places where the functions are defined?, see ticket #2031
typedef xme_core_dcc_subscriptionHandle_t (*xme_core_dcc_subscribeTopic_t)
(
    xme_core_topic_t topic,
    xme_core_md_topicMetaDataHandle_t filter,
    bool onlyLocal,
    xme_core_dcc_receiveTopicCallback_t callback,
    void* userData
);

typedef xme_core_dcc_publicationHandle_t (*xme_core_dcc_publishTopic_t)
(
    xme_core_topic_t,
    xme_core_md_topicMetaDataHandle_t,
    bool,
    xme_core_dcc_demandCallback_t
);

typedef xme_core_resourceManager_taskHandle_t (*xme_core_resourceManager_scheduleTask_t)
(
    xme_hal_time_timeInterval_t startMs,
    xme_hal_time_timeInterval_t periodMs,
    uint8_t priority,
    xme_hal_sched_taskCallback_t callback,
    void* userData
);

typedef xme_status_t (*xme_core_resourceManager_killTask_t)
(
    xme_core_resourceManager_taskHandle_t taskHandle
);

typedef xme_status_t (*xme_core_dcc_unsubscribeTopic_t)
(
    xme_core_dcc_subscriptionHandle_t subscriptionHandle
);


typedef xme_status_t (*xme_core_dcc_unpublishTopic_t)
(
    xme_core_dcc_publicationHandle_t publicationHandle
);

typedef xme_status_t (*xme_core_dcc_sendTopicData_t)
(
    xme_core_dcc_publicationHandle_t publicationHandle,
    void* data,
    uint16_t size
);

typedef void* (*xme_hal_sharedPtr_getPointer_t)
(
    xme_hal_sharedPtr_t sharedPtr
);

typedef const struct
{
    const xme_core_dcc_subscribeTopic_t xme_core_dcc_subscribeTopic;
    const xme_core_dcc_publishTopic_t xme_core_dcc_publishTopic;
    const xme_core_resourceManager_scheduleTask_t xme_core_resourceManager_scheduleTask;
    const xme_hal_sharedPtr_getPointer_t xme_hal_sharedPtr_getPointer;
    const xme_core_dcc_sendTopicData_t xme_core_dcc_sendTopicData;
    const xme_core_resourceManager_killTask_t xme_core_resourceManager_killTask;
    const xme_core_dcc_unsubscribeTopic_t xme_core_dcc_unsubscribeTopic;
    const xme_core_dcc_unpublishTopic_t xme_core_dcc_unpublishTopic;
    const xme_core_log_logHandler_t xme_log;
} xme_api_t;

//Macro to create an instance of the API struct and initialize the pointers
#define XME_API_INIT() \
    static xme_api_t xme_api = \
    { \
        (xme_core_dcc_subscribeTopic_t)    xme_core_dcc_subscribeTopic, \
        (xme_core_dcc_publishTopic_t) xme_core_dcc_publishTopic, \
        (xme_core_resourceManager_scheduleTask_t) xme_core_resourceManager_scheduleTask, \
        (xme_hal_sharedPtr_getPointer_t) xme_hal_sharedPtr_getPointer, \
        (xme_core_dcc_sendTopicData_t) xme_core_dcc_sendTopicData, \
        (xme_core_resourceManager_killTask_t) xme_core_resourceManager_killTask, \
        (xme_core_dcc_unsubscribeTopic_t) xme_core_dcc_unsubscribeTopic, \
        (xme_core_dcc_unpublishTopic_t) xme_core_dcc_unpublishTopic, \
        (xme_core_log_logHandler_t) xme_log_handler, \
    }; \

#endif // #ifdef PRE_IL1_SUPPORT

// XME_API_IL1
typedef xme_status_t (*xme_core_dataHandler_completeReadOperation_t)
(
    xme_core_dataManager_dataPacketId_t port
); ///< the function callback definition for complete read operation. 

typedef xme_status_t (*xme_core_dataHandler_completeWriteOperation_t)
(
    xme_core_dataManager_dataPacketId_t port
); ///< the function callback definition for complete write operation. 

typedef xme_status_t (*xme_core_dataHandler_readData_t)
(
    xme_core_dataManager_dataPacketId_t port,
    void * const buffer,
    unsigned int bufferSize,
    unsigned int * const bytesRead
); ///< the function callback definition for reading data. 

typedef xme_status_t (*xme_core_dataHandler_readAttribute_t)
(
 xme_core_dataManager_dataPacketId_t port,
 xme_core_attribute_key_t attributeKey,
 void * const buffer,
 unsigned int bufferSize,
 unsigned int * const bytesRead
); ///< the function callback definition for reading attribute. 

typedef xme_status_t (*xme_core_dataHandler_writeData_t)
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    unsigned int bufferSize
); ///< the function callback definition for writing data. 

typedef xme_status_t (*xme_core_dataHandler_writeAttribute_t)
(
 xme_core_dataManager_dataPacketId_t port,
 xme_core_attribute_key_t attributeKey,
 void const * const buffer,
 unsigned int bufferSize
); ///< the function callback definition for writing attribute. 

typedef xme_status_t (*xme_core_exec_dispatcher_initializeTask_t)
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
); ///< the function callback definition for initialize task. 

typedef xme_status_t (*xme_core_exec_dispatcher_waitForStart_t)
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    void ** functionArgs
); ///< the function callback definition for waiting for start. 

typedef xme_status_t (*xme_core_exec_dispatcher_executionCompleted_t)
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
); ///< the function callback definition for notify execution completion. 

typedef const struct
{
    // Add the required API for IL1
    const xme_core_dataHandler_completeReadOperation_t xme_core_dataHandler_completeReadOperation; ///< the complete read operation callback. 
    const xme_core_dataHandler_completeWriteOperation_t xme_core_dataHandler_completeWriteOperation; ///< the complete write operation callback. 
    const xme_core_dataHandler_writeData_t xme_core_dataHandler_writeData; ///< the write data callback. 
    const xme_core_dataHandler_readData_t xme_core_dataHandler_readData; ///< the read data callback. 
    const xme_core_dataHandler_writeAttribute_t xme_core_dataHandler_writeAttribute; ///< the write attribute callback. 
    const xme_core_dataHandler_readAttribute_t xme_core_dataHandler_readAttribute; ///< the read attribute callback. 
    const xme_core_exec_dispatcher_initializeTask_t xme_core_exec_dispatcher_initializeTask; ///< the initialize task callback. 
    const xme_core_exec_dispatcher_waitForStart_t xme_core_exec_dispatcher_waitForStart; ///< the wait for start callback. 
    const xme_core_exec_dispatcher_executionCompleted_t xme_core_exec_dispatcher_executionCompleted; ///< the execution completed callback. 
    const xme_core_log_logHandler_t xme_log; ///< the log handler.
} xme_api_il1_t;

//Macro to create an instance of the API struct and initialize the pointers
#define XME_API_IL1_INIT() \
    static xme_api_il1_t xme_api_il1 = \
    { \
        (xme_core_dataHandler_completeReadOperation_t) xme_core_dataHandler_completeReadOperation, \
        (xme_core_dataHandler_completeWriteOperation_t) xme_core_dataHandler_completeWriteOperation, \
        (xme_core_dataHandler_writeData_t) xme_core_dataHandler_writeData, \
        (xme_core_dataHandler_readData_t) xme_core_dataHandler_readData, \
        (xme_core_dataHandler_writeAttribute_t) xme_core_dataHandler_writeAttribute, \
        (xme_core_dataHandler_readAttribute_t) xme_core_dataHandler_readAttribute, \
        (xme_core_exec_dispatcher_initializeTask_t) xme_core_exec_dispatcher_initializeTask, \
        (xme_core_exec_dispatcher_waitForStart_t) xme_core_exec_dispatcher_waitForStart, \
        (xme_core_exec_dispatcher_executionCompleted_t) xme_core_exec_dispatcher_executionCompleted,\
        (xme_core_log_logHandler_t) xme_log_handler, \
    }; \

#endif //#ifndef XME_CORE_API_H
