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
 * $Id: dataHandler.c 5200 2013-09-27 09:27:57Z ruiz $
 */

/**
 * \file
 *         Data Handler.
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/log.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/math.h"
#include "xme/core/topicData.h"

#include "xme/core/dataHandler/include/dataHandlerInternalTypes.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerRTEInterface.h"
#include "xme/core/dataHandler/include/dataHandlerInternalMethods.h"
#include "xme/core/dataHandler/include/auditHandler.h"

#include "xme/core/broker/include/brokerDataManagerInterface.h"

#include <assert.h>

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//
/**
 * \brief the definition of no attribute descriptor list. 
 */
xme_core_attribute_descriptor_list_t XME_CORE_NO_ATTRIBUTE = {0, NULL};

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//

static void
xme_core_dataHandler_copyDataPacket(xme_core_dataHandler_dataPacket_t * const source,
                                         xme_core_dataHandler_dataPacket_t * const dest);

static void
xme_core_dataHandler_copyAllAttributes(xme_core_dataHandler_dataPacket_t * const source,
                                       xme_core_dataHandler_dataPacket_t * const dest, bool useShadow);

static INLINE xme_status_t
xme_core_dataHandler_checkValidTransfer(
                xme_core_dataHandler_dataStructure_t const * const source,
                xme_core_dataHandler_dataStructure_t const * const destination);


extern xme_core_dataManager_dataPacketId_t xme_core_exec_CycleCounter;

//******************************************************************************//
//***   Implementation                                                       ***//
//******************************************************************************//
xme_status_t xme_core_dataHandler_init(size_t amountOfMemory)
{
    xme_status_t initStatus = XME_STATUS_INTERNAL_ERROR;
    xme_status_t imageAvailable;
    xme_core_dataHandler_imageDataStructure_t imageInformation = NULL;
//    if(0 == amountOfMemory)
//    {
//        amountOfMemory = 15;
//    }
    // FIXME: test will stop when the following line is called, please solve me!
    //XME_ASSERT_RVAL(0u != amountOfMemory, XME_STATUS_INTERNAL_ERROR);
    assert(0u != amountOfMemory);
    //XME_ASSERT_RVAL(xme_core_dataHandler_dataHandlerInternals.initialized == false, XME_STATUS_INTERNAL_ERROR);
    assert(xme_core_dataHandler_dataHandlerInternals.initialized == false);

    xme_core_dataHandler_initializeInternalDataStructure(amountOfMemory);

    //     load from datastorage => if image is there
    imageAvailable = xme_core_dataHandler_checkDataStorageForImage();
    if (imageAvailable == XME_STATUS_SUCCESS)
    {
        imageInformation = xme_core_dataHandler_loadImage();
        initStatus = xme_core_dataHandler_checkImageForConsistancy(imageInformation);
        XME_CHECK(XME_STATUS_SUCCESS == initStatus, XME_STATUS_INTERNAL_ERROR);
        xme_core_dataHandler_dataHandlerInternals.initFromFileStorage = true;
    }

    //     use this to initialize the data structure
    initStatus = xme_core_dataHandler_initializeDataStructure(imageInformation);
    //XME_ASSERT_RVAL(XME_STATUS_SUCCESS == initStatus, XME_STATUS_INTERNAL_ERROR);
    assert(XME_STATUS_SUCCESS == initStatus);
    initStatus = xme_core_dataHandler_checkInternalConistency();
    //XME_ASSERT_RVAL(XME_STATUS_SUCCESS == initStatus, XME_STATUS_INTERNAL_ERROR);
    assert(XME_STATUS_SUCCESS == initStatus);

    //
    xme_core_dataHandler_dataHandlerInternals.initialized = true;
    xme_core_dataHandler_dataHandlerInternals.dataPacketId = (xme_core_dataManager_dataPacketId_t) 1;

    // create our cycle counter port
    {
        xme_status_t status = xme_core_dataHandler_createRTEData(
                        XME_CORE_TOPIC_EXEC_CYCLE_COUNTER,
                        sizeof(xme_core_topic_exec_cycleCounter_t), XME_CORE_NO_ATTRIBUTE, 1, /* queue size */
                        (bool) true, /* override */
                        (bool) false, /* persistent */
                        0, /* history depth */
                        &xme_core_exec_CycleCounter);
        /* do some additional stuff */
        if (status != XME_STATUS_SUCCESS) {
            xme_core_exec_CycleCounter = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;
        }
    }

    /*
    XME_LOG(XME_LOG_WARNING, "__builtin_return_address(0) = %p\n", __builtin_return_address(0));
    XME_LOG(XME_LOG_WARNING, "__builtin_frame_address(0) = %p\n", __builtin_frame_address(0));
    XME_LOG(XME_LOG_WARNING, "__builtin_return_address(1) = %p\n", __builtin_return_address(1));
    XME_LOG(XME_LOG_WARNING, "__builtin_frame_address(1) = %p\n", __builtin_frame_address(1));
    */
    return initStatus;
}

//******************************************************************************//
xme_status_t xme_core_dataHandler_fini(void)
{
    //XME_ASSERT(xme_core_dataHandler_dataHandlerInternals.initialized == true, XME_STATUS_INTERNAL_ERROR);
    assert(xme_core_dataHandler_dataHandlerInternals.initialized == true);

    // dump to fileStorage

    // free our used memory
    xme_core_dataHandler_uninitializeInternalDataStructure();
    xme_core_dataHandler_dataHandlerInternals.initialized = false;
    xme_core_dataHandler_dataHandlerInternals.dataPacketId = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_createRTEData(xme_core_topic_t topic, uint32_t bufferSize,
                                       xme_core_attribute_descriptor_list_t metadata,
                                       uint32_t queueSize, bool overwrite, bool persistent,
                                       uint32_t historyDepth,
                                       xme_core_dataManager_dataPacketId_t * const dataPacketId) {
    return xme_core_dataHandler_createPort(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT,
                                           XME_CORE_COMPONENT_PORTTYPE_INTERNAL, topic,
                                           bufferSize, metadata, queueSize, overwrite,
                                           persistent, historyDepth, dataPacketId);
}

xme_status_t
xme_core_dataHandler_createPort(xme_core_component_t componentID,
                                    xme_core_component_portType_t type, xme_core_topic_t topic,
                                    uint32_t bufferSize, xme_core_attribute_descriptor_list_t metadata,
                                    uint32_t queueSize, bool overwrite, bool persistent,
                                    uint32_t historyDepth,
                                    xme_core_dataManager_dataPacketId_t * const dataPacketId) {
    xme_status_t status;
    xme_core_dataManager_dataPacketId_t port;

    xme_core_dataHandler_setup_t setupValue =
    {
        SFINIT(componentID, componentID),
        SFINIT(type, type),
        SFINIT(topic, topic),
        SFINIT(bufferSize, bufferSize),
        SFINIT(metadata, {metadata.length, metadata.element}),
        SFINIT(queueSize, queueSize),
        SFINIT(overwrite, overwrite),
        SFINIT(persistent, persistent),
        SFINIT(historyDepth, historyDepth)
    };

    *dataPacketId = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

    XME_CHECK(((type == XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION) ||
               (type == XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION) ||
               (type == XME_CORE_COMPONENT_PORTTYPE_INTERNAL)) &&
              (topic != XME_CORE_TOPIC_INVALID_TOPIC) && (bufferSize > 0),
              XME_STATUS_INVALID_PARAMETER);

    status = xme_core_dataHandler_createDataEntry(&setupValue, &port);
    /* do some additional stuff */
    if (status == XME_STATUS_SUCCESS) {
        *dataPacketId = port;
    }
    return status;
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_transferData(xme_core_dataManager_dataPacketId_t portSource,
                                      xme_core_dataManager_dataPacketId_t portSink) {
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    xme_core_dataHandler_dataStructure_t *internalSource, *internalDestination,
                                         *shadowSource, *shadowDestination;
    xme_core_dataHandler_dataPacket_t *elementSRC, *shadowSRC, *elementDST, *shadowDST;

    //XME_ASSERT_RVAL(xme_core_dataHandler_dataHandlerInternals.initialized == true, XME_STATUS_INTERNAL_ERROR);
    assert(xme_core_dataHandler_dataHandlerInternals.initialized == (bool) true);

    XME_CHECK((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID < portSource ) &&
              (XME_CORE_DATAMANAGER_DATAPACKETID_MAX > portSource) &&
              (XME_CORE_DATAMANAGER_DATAPACKETID_INVALID < portSink) &&
              (XME_CORE_DATAMANAGER_DATAPACKETID_MAX > portSink) &&
              (portSource != portSink), XME_STATUS_INVALID_PARAMETER);

    internalSource = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[portSource];
    shadowSource = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[portSource];
    internalDestination = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[portSink];
    shadowDestination = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[portSink];

//    XME_ASSERT_RVAL((NULL != internalSource) && (NULL != internalDestination) &&
//              (NULL != shadowSource) && (NULL != shadowDestination),
//              XME_STATUS_INTERNAL_ERROR);
    assert((NULL != internalSource) && (NULL != internalDestination) &&
           (NULL != shadowSource) && (NULL != shadowDestination));

    status = xme_core_dataHandler_checkValidTransfer(internalSource, internalDestination);
    XME_CHECK_MSG(XME_STATUS_SUCCESS == status, status, XME_LOG_FATAL,
                  "DATAHANDLER: xme_core_dataHandler_transferData problem with port types!\n"
                  "DATAHANDLER: portSource type = %i, but should be = %i\n"
                  "DATAHANDLER: portSink type = %i, but should be = %i\n"
                  "DATAHANDLER: topic %i != topic %i\n",
                  internalSource->dataEntry.type, XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION,
                  internalDestination->dataEntry.type, XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION,
                  internalSource->dataEntry.topic, internalDestination->dataEntry.topic);

#if defined(USE_QUEUE)
    elementSRC = (xme_core_dataHandler_dataPacket_t *) internalSource->dataEntry.current;
    elementDST = (xme_core_dataHandler_dataPacket_t *) internalDestination->dataEntry.current;
    shadowSRC = (xme_core_dataHandler_dataPacket_t *) shadowSource->dataEntry.current;
    shadowDST = (xme_core_dataHandler_dataPacket_t *) shadowDestination->dataEntry.current;
#else
    elementSRC = &internalSource->dataEntry;
    elementDST = &internalDestination->dataEntry;
    shadowSRC = &shadowSource->dataEntry;
    shadowDST = &shadowDestination->dataEntry;
#endif

    XME_ASSERT_RVAL((NULL != elementSRC) && (NULL != elementDST) &&
                    (NULL != shadowSRC) && (NULL != shadowDST),
                    XME_STATUS_INTERNAL_ERROR);

    //Transfer data
    if (internalSource->shadowEntry.useShadowEntry) {
        xme_core_dataHandler_copyDataPacket(shadowSRC, elementDST);
        xme_core_dataHandler_copyAllAttributes(shadowSRC, elementDST, (bool) true);
    } else {
        xme_core_dataHandler_copyDataPacket(elementSRC, elementDST);
        xme_core_dataHandler_copyAllAttributes(elementSRC, elementDST, (bool) false);

        xme_core_dataHandler_copyDataPacket(shadowSRC, shadowDST);
        xme_core_dataHandler_copyAllAttributes(shadowSRC, shadowDST, (bool)true);
    }

    // todo: check if this is valid!!!
    if (internalSource->shadowEntry.useShadowEntry) {
        shadowSource->shadowEntry.lockedByTestSystem = false;
    }

    // todo: check if data availability change should be called in transfer operation, or in completeReadOperation
    //status = xme_core_broker_dataAvailabilityChange(portSink, 1);
    //TODO: check here if we should't add portSoure instead of portSink!
    (void) xme_core_broker_dataAvailabilityChange(portSink, 1);
    status = XME_STATUS_SUCCESS;
    return status;
}

static void
xme_core_dataHandler_copyDataPacket(xme_core_dataHandler_dataPacket_t * const source,
                                         xme_core_dataHandler_dataPacket_t * const dest) {
    size_t size = 0U;

    //XME_ASSERT_NORVAL((NULL != source) && (NULL != dest));
    assert((NULL != source) && (NULL != dest));
    size = XME_HAL_MATH_MIN(source->dataElement.size, dest->dataElement.size);
    (void) xme_hal_mem_set(dest->dataElement.data, 0u, dest->dataElement.size);
    //XME_LOG(XME_LOG_DEBUG, "Will copy an overall size of %lu\n", (unsigned long) size);

    (void) xme_hal_mem_set(dest->dataElement.data, 0u, dest->dataElement.size);
    (void) xme_hal_mem_copy(dest->dataElement.data, source->dataElement.data, size);

    dest->dataElement.size = size;
    dest->dataElement.checksum = xme_core_dataHandler_calculateHash(dest->dataElement.data, size);
}

static void
xme_core_dataHandler_copyAllAttributes(xme_core_dataHandler_dataPacket_t * const source,
                                            xme_core_dataHandler_dataPacket_t * const dest,
                                            bool useShadow) {
    register uint32_t i;
    size_t size = 0U;
    //XME_ASSERT_NORVAL((NULL != source) && (NULL != dest));
    assert((NULL != source) && (NULL != dest));

    for (i = 0U; i < source->attributeElements; ++i) {
        xme_core_dataHandler_attributeElement_t * const destAttribute =
           xme_core_dataHandler_findAttribute(dest->port, source->attributeElement[i].key, useShadow);
        if(NULL == destAttribute) continue;
        size = XME_HAL_MATH_MIN(source->attributeElement[i].size, destAttribute->size);

        (void) xme_hal_mem_set(destAttribute->data, 0u, destAttribute->size);
        (void) xme_hal_mem_copy(destAttribute->data, source->attributeElement[i].data, size);

        destAttribute->size = size;
        destAttribute->checksum = xme_core_dataHandler_calculateHash(destAttribute->data, size);
    }
}

//******************************************************************************//
xme_status_t xme_core_dataHandler_completeReadOperation(xme_core_dataManager_dataPacketId_t port)
{
    register uint32_t i;
    xme_core_dataHandler_dataStructure_t *internalSource, *shadowSource;
    xme_core_dataHandler_dataPacket_t *element, *shadow;

    XME_CHECK((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port), XME_STATUS_INVALID_PARAMETER);

    // 1. find matching entity in db
    internalSource = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
    shadowSource = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
    //XME_ASSERT_RVAL((NULL != internalSource) && (NULL != shadowSource), XME_STATUS_INTERNAL_ERROR);
    assert((NULL != internalSource) && (NULL != shadowSource));

    // 2. check for persistence, if not then clean every data value
#if defined(USE_QUEUE)
    element = (xme_core_dataHandler_dataPacket_t *) internalSource->dataEntry.current;
    shadow = (xme_core_dataHandler_dataPacket_t *) shadowSource->dataEntry.current;
#else
    element = &internalSource->dataEntry;
    shadow = &shadowSource->dataEntry;
#endif
    //XME_ASSERT_RVAL((NULL != element) && (NULL != shadow), XME_STATUS_INTERNAL_ERROR);
    assert((NULL != element) && (NULL != shadow));

    if (!internalSource->info.persistent) {
        (void) xme_hal_mem_set(element->dataElement.data, 0U, element->dataElement.size);
        for (i = 0U; i < element->attributeElements; ++i) {
            (void) xme_hal_mem_set(element->attributeElement[i].data, 0U, element->attributeElement[i].size);
        }
    }

    if (!shadowSource->info.persistent) {
        (void) xme_hal_mem_set(shadow->dataElement.data, 0U, shadow->dataElement.size);
        for (i = 0U; i < shadow->attributeElements; ++i) {
            (void) xme_hal_mem_set(shadow->attributeElement[i].data, 0U, shadow->attributeElement[i].size);
        }
    }

    // 3. unlock the entity
    internalSource->info.writeLock = false;
    shadowSource->info.writeLock = false;
#if defined(USE_QUEUE)
    if(internalSource->dataEntry.last == internalSource->dataEntry.current){
        // here: we are the last element, thus we have to wrap around
        internalSource->dataEntry.current = internalSource->dataEntry.start;
        shadowSource->dataEntry.current = shadowSource->dataEntry.start;
    }else{
        // here: we have to find the next element
        for(i = 0U; i < internalSource->dataEntry.elements; ++i){
            if(internalSource->dataEntry.current == &internalSource->dataEntry.dataElement[i]){
                break;
            }
        }
        internalSource->dataEntry.current = &internalSource->dataEntry.dataElement[i+1];
        shadowSource->dataEntry.current = &shadowSource->dataEntry.dataElement[i+1];
    }
#endif

    // 4. call broker function to signal that data is ready
    // TODO: adapt the count, currently we only provide one element, but we should provide as much as some component has written!
    return xme_core_broker_dataAvailabilityChange(port, 0);
}

//******************************************************************************//
xme_status_t xme_core_dataHandler_completeWriteOperation(xme_core_dataManager_dataPacketId_t port)
{
    xme_core_dataHandler_dataStructure_t *internalSource, *shadowSource;

    XME_CHECK(xme_core_dataHandler_dataHandlerInternals.initialized == true, XME_STATUS_INTERNAL_ERROR);
    XME_CHECK((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port), XME_STATUS_INVALID_PARAMETER);

    // 1. find matching entity in db
    internalSource = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
    shadowSource = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
    //XME_ASSERT_RVAL((NULL != internalSource) && (NULL != shadowSource), XME_STATUS_INTERNAL_ERROR);
    assert((NULL != internalSource) && (NULL != shadowSource));

    // 2. lock the entity
    internalSource->info.writeLock = true;
    shadowSource->info.writeLock = true;
#if defined(USE_QUEUE)
    if(internalSource->dataEntry.last == internalSource->dataEntry.current){
        // here: we are the last element, thus we have to wrap around
        internalSource->dataEntry.current = internalSource->dataEntry.start;
        shadowSource->dataEntry.current = shadowSource->dataEntry.start;
    }else{
        // here: we have to find the next element
        for(i = 0U; i < internalSource->dataEntry.elements; ++i){
            if(internalSource->dataEntry.current == &internalSource->dataEntry.dataElement[i]){
                break;
            }
        }
        internalSource->dataEntry.current = &internalSource->dataEntry.dataElement[i+1];
        shadowSource->dataEntry.current = &shadowSource->dataEntry.dataElement[i+1];
    }
#endif

    // 3. call broker function to signal that data is ready
    return xme_core_broker_dataAvailabilityChange(port, 1);
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_readData
(
    xme_core_dataManager_dataPacketId_t port,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead
)
{
    return xme_core_dataHandler_readDataWithDirective(port, 0u, buffer, bufferSize, bytesRead,
                                                      DATAHANDLER_STATE_READ_TOPICELEMENT);
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_readAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead
)
{
    return xme_core_dataHandler_readAttributeWithDirective(port, attributeKey, 0U, buffer,
                                                           bufferSize, bytesRead,
                                                           DATAHANDLER_STATE_READ_TOPICATTRIBUTE);
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_writeData
(
    xme_core_dataManager_dataPacketId_t port,
    void const * const buffer,
    uint32_t bufferSize
)
{
    return xme_core_dataHandler_writeDataWithDirective(port, 0U, buffer, bufferSize,
                                                       DATAHANDLER_STATE_WRITE_TOPICELEMENT);
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_writeAttribute
(
    xme_core_dataManager_dataPacketId_t port,
    xme_core_attribute_key_t attributeKey,
    void const * const buffer,
    uint32_t bufferSize
)
{
    return xme_core_dataHandler_writeAttributeWithDirective(port, attributeKey, 0U, buffer,
                                                            bufferSize,
                                                            DATAHANDLER_STATE_WRITE_TOPICATTRIBUTE);
}

uint32_t xme_core_dataHandler_readCycle(void){
#if defined(USE_QUEUE)
    return *(uint32_t *) (((dataHandler_dataPacket_t *)(xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[1])->dataEntry.current)->dataElement.data;
#else
    return *(uint32_t *) ((xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[1])->dataEntry.dataElement.data;
#endif
}

void xme_core_dataHandler_writeCycle(uint32_t counter){

#if defined(USE_QUEUE)
            (void) xme_hal_mem_copy(
                                ((dataHandler_dataPacket_t *)((xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals
                                    .dataHandlerPorts[1])->dataEntry.current)->dataElement.data,
                                &counter, sizeof(counter));
#else
            (void) xme_hal_mem_copy(
                                ((xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals
                                    .dataHandlerPorts[1])->dataEntry.dataElement.data,
                                &counter, sizeof(counter));
#endif
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_dataHandlerDump(void) {
#if defined(USE_QUEUE)
    register int32_t i;
    register uint32_t j,k,l;
#else
    register uint32_t i, j, l;
#endif
    xme_core_dataHandler_dataStructure_t *node;

    XME_LOG(XME_LOG_ALWAYS, "Will DUMP datahandler table to output\n");
    for (l = 1U; l < (uint32_t) xme_core_dataHandler_dataHandlerInternals.dataPacketId; ++l) {
        node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals
            .dataHandlerPorts[l];
        //XME_ASSERT_RVAL(NULL != node, XME_STATUS_INTERNAL_ERROR);
        assert(NULL != node);
        XME_LOG(XME_LOG_ALWAYS,
                "\n//******************************************************************************//\n");

        XME_LOG(XME_LOG_ALWAYS, "Port=%d with type=%d and topic=%d\n", node->dataEntry.port,
                node->dataEntry.type, node->dataEntry.topic);

        XME_LOG(XME_LOG_ALWAYS,
                "Port is '%s' and port will be '%s' by test system.",
                node->shadowEntry.lockedByTestSystem ? "locked" : "not locked",
                node->shadowEntry.useShadowEntry ? "used" : "not used");

#if defined(USE_QUEUE)
        XME_LOG(XME_LOG_ALWAYS, "Will now print internal queue:\n");
        for(i = 0; i < node->dataEntry.elements; ++i) {
            XME_LOG(XME_LOG_ALWAYS, "Data element has got a size of %d with value [", node->dataEntry.dataElement[i].dataElement.size);
            for(j = 0u; j < node->dataEntry.dataElement[i].dataElement.size-1; ++j)
            {
                XME_LOG(XME_LOG_ALWAYS, "0x%03x,", ((uint8_t*)node->dataEntry.dataElement[i].dataElement.data)[j]);
            }
            XME_LOG(XME_LOG_ALWAYS, "0x%03x]\n", ((uint8_t*)node->dataEntry.dataElement[i].dataElement.data)[j]);

            XME_LOG(XME_LOG_ALWAYS, "Existing attributes = %5d\n", node->dataEntry.dataElement[i].attributeElements);
            for(j = 0u; j < node->dataEntry.dataElement[i].attributeElements; ++j)
            {
                XME_LOG(XME_LOG_ALWAYS, "With key=%5d and size=%5d [", node->dataEntry.dataElement[i].attributeElement[j].key, node->dataEntry.dataElement[i].attributeElement[j].size);
                for(k = 0u; k < node->dataEntry.dataElement[i].attributeElement[j].size-1; ++k)
                {
                    XME_LOG(XME_LOG_ALWAYS, "0x%03x,", ((uint8_t*)node->dataEntry.dataElement[i].attributeElement[j].data)[k]);
                }
                XME_LOG(XME_LOG_ALWAYS, "0x%03x]\n", ((uint8_t*)node->dataEntry.dataElement[i].attributeElement[j].data)[k]);
            }
            XME_LOG(XME_LOG_ALWAYS, "\n");
        }
#else
        XME_LOG(XME_LOG_ALWAYS, "Data element has got a size of %5d, checksum=%u with value [",
                node->dataEntry.dataElement.size, node->dataEntry.dataElement.checksum);

        for (i = 0u; i < node->dataEntry.dataElement.size - 1; ++i) {
            XME_LOG(XME_LOG_ALWAYS, "0x%03x,", ((uint8_t* )node->dataEntry.dataElement.data)[i]);
        }

        XME_LOG(XME_LOG_ALWAYS, "0x%03x]\n", ((uint8_t* )node->dataEntry.dataElement.data)[i]);

        XME_LOG(XME_LOG_ALWAYS, "Existing attributes = %5d\n", node->dataEntry.attributeElements);

        for (i = 0u; i < node->dataEntry.attributeElements; ++i) {
            XME_LOG(XME_LOG_ALWAYS, "With key=%5d and size=%5d, checksum=%u [",
                    node->dataEntry.attributeElement[i].key,
                    node->dataEntry.attributeElement[i].size,
                    node->dataEntry.attributeElement[i].checksum);
            for (j = 0u; j < node->dataEntry.attributeElement[i].size - 1; ++j) {
                XME_LOG(XME_LOG_ALWAYS, "0x%03x,",
                        ((uint8_t* )node->dataEntry.attributeElement[i].data)[j]);
            }
            XME_LOG(XME_LOG_ALWAYS, "0x%03x]\n",
                    ((uint8_t* )node->dataEntry.attributeElement[i].data)[j]);
        }
        XME_LOG(XME_LOG_ALWAYS, "\n");
#endif
    }

    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
static INLINE xme_status_t
xme_core_dataHandler_checkValidTransfer(
                xme_core_dataHandler_dataStructure_t const * const source,
                xme_core_dataHandler_dataStructure_t const * const destination) {
    //XME_ASSERT((NULL != source) && (NULL != destination));
    assert((NULL != source) && (NULL != destination));
    XME_CHECK((XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION == source->dataEntry.type) &&
              (XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION == destination->dataEntry.type) &&
              (source->dataEntry.topic == destination->dataEntry.topic),
              XME_STATUS_PERMISSION_DENIED);

    return XME_STATUS_SUCCESS;
}
