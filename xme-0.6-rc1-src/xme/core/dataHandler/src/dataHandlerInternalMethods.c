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
 * $Id: dataHandlerInternalMethods.c 5121 2013-09-19 13:51:44Z camek $
 */

/**
 * \file
 *         Data Handler Internal methods.
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/hal/include/mem.h"
#include "xme/hal/include/math.h"

#include "xme/core/log.h"

#include "xme/core/dataManagerTypes.h"
#include "xme/core/dataHandler/include/dataHandlerInternalTypes.h"
#include "xme/core/dataHandler/include/dataHandlerInternalMethods.h"

#if !defined (_STDDEF_H_)
#include <stddef.h>
#endif
#if defined(linux) || defined(__linux) || defined(__linux__)
#include <sys/mman.h>
#endif

#include <assert.h>

//******************************************************************************//
//***   Defines                                                              ***//
//******************************************************************************//
typedef enum xme_core_dataHandler_operation{
    invalidOperation = 0,
    readOperation,
    writeOperation
} xme_core_dataHandler_operation_t;

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//
/* FIXME: Bug #2253*/

/**
 * \brief the data handler internal structure. 
 */
xme_core_dataHandler_internals_t xme_core_dataHandler_dataHandlerInternals;

/**
 * \brief the canary constant. Here we use a terminator canary.
 */
// TODO: maybe a random canray or an random XOR canary would improve the usage here!
static const uint32_t canary = 0x72616365;   // ecar race:0x656361722072616365;

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
static xme_status_t
readDataWithOffset
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint32_t offset,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead,
    bool readFromShadow
);

static xme_status_t
readAttributeWithOffset
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_attribute_key_t attributeKey,
    uint32_t offset,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead,
    bool readFromShadow
);

static xme_status_t
writeDataWithOffset
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint32_t offset,
    void const * const buffer,
    uint32_t bufferSize,
    bool writeToShadow
);

static xme_status_t
writeAttributeWithOffset
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_attribute_key_t attributeKey,
    uint32_t offset,
    void const * const buffer,
    uint32_t bufferSize,
    bool writeToShadow
);

static xme_status_t
writeToDataElement(xme_core_dataHandler_dataElement_t * const element, void const * const buffer,
                   size_t offset, size_t size);

static xme_status_t
writeToAttributeElement(xme_core_dataHandler_attributeElement_t * const element,
                        void const * const buffer, size_t offset, size_t size);

static xme_status_t writeToBlob(void * const data, void const * const buffer, size_t offset, size_t size);

static INLINE bool xme_core_dataHandler_portIsWriteLocked(xme_core_dataManager_dataPacketId_t port);

/**
 * \brief Checks if a buffer contains a canary and violates the.
 *
 */
static xme_status_t
checkCanary(void const * const buffer, size_t size);

static xme_status_t
checkChecksum(uint32_t checksum, void const * const buffer, size_t size);

/**
 * \brief  Allocates some quantity of space in the database.
 * \param value specifies the basis value to calculate the needed amount of memory.
 * \return the pointer to the allocated data structure.
 */
static xme_core_dataHandler_dataStructure_t *
allocateSpace
(
    xme_core_dataHandler_setup_t const * const value, xme_core_dataHandler_buddySystem_t * const memory
);

static size_t calculateNeededMemory(xme_core_dataHandler_setup_t const * const value);

/**
 * \brief  Fills element with values.
 * \param element The element to fill with the corresponding value.
 * \param value The value of filling.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the element has been successfully filled. 
 *          - XME_CORE_STATUS_INTERNAL_ERROR in other case. 
 */
static xme_status_t
fillElementWithValues
(
    xme_core_dataHandler_dataStructure_t * const element,
    xme_core_dataHandler_setup_t const * const value
);

static INLINE xme_status_t
checkDataEntry
(
   xme_core_dataHandler_dataStructure_t const * const dataEntry,
   bool useShadow,
   xme_core_dataHandler_operation_t operation
);

/* Work out the total number of free bytes */
//static availableMemoryPool();
//static releaseMemory();
//static getMemory();

//******************************************************************************//
//***   Implementation                                                       ***//
//******************************************************************************//

//********************** internal functions ************************************//
void
xme_core_dataHandler_initializeInternalDataStructure(size_t amountOfMemory) {
    size_t memory = (size_t) (1U << amountOfMemory);

    (void) xme_hal_mem_set(&xme_core_dataHandler_dataHandlerInternals, 0U, sizeof(xme_core_dataHandler_dataHandlerInternals));

    //xme_core_dataHandler_dataHandlerInternals.selfID = xme_core_resourceManager_getCurrentComponentId();

    xme_core_dataHandler_dataHandlerInternals.memory.amountOfMemory = memory >> 1U;
    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.amountOfMemory = memory >> 1U;

#if defined(linux)
    xme_core_dataHandler_dataHandlerInternals.memory.baseAdress = mmap(NULL, xme_core_dataHandler_dataHandlerInternals.memory.amountOfMemory,
                                                  PROT_READ | PROT_WRITE,
                                                  MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.baseAdress = mmap(NULL, xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.amountOfMemory,
                                                      PROT_READ | PROT_WRITE,
                                                      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
#else
    xme_core_dataHandler_dataHandlerInternals.memory.baseAdress = xme_hal_mem_alloc(xme_core_dataHandler_dataHandlerInternals.memory.amountOfMemory);
    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.baseAdress = xme_hal_mem_alloc(xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.amountOfMemory);
#endif
    //XME_ASSERT_NORVAL(xme_core_dataHandler_dataHandlerInternals.memory.baseAdress != NULL);
    assert((xme_core_dataHandler_dataHandlerInternals.memory.baseAdress != NULL) && (xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.baseAdress != NULL));

    xme_core_dataHandler_dataHandlerInternals.memory.freeBytes = xme_core_dataHandler_dataHandlerInternals.memory.amountOfMemory;
    xme_core_dataHandler_dataHandlerInternals.memory.maxAdress = (void*)((uintptr_t)xme_core_dataHandler_dataHandlerInternals.memory.baseAdress + xme_core_dataHandler_dataHandlerInternals.memory.amountOfMemory);
    xme_core_dataHandler_dataHandlerInternals.memory.offset = 0U;

    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.freeBytes = xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.amountOfMemory;
    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.maxAdress = (void*)((uintptr_t)xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.baseAdress + xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.amountOfMemory);
    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.offset = 0U;
}

void
xme_core_dataHandler_uninitializeInternalDataStructure(void) {
    register uint32_t i;
    (void)xme_hal_mem_set(xme_core_dataHandler_dataHandlerInternals.memory.baseAdress, 0U,
                          xme_core_dataHandler_dataHandlerInternals.memory.amountOfMemory);
     (void)xme_hal_mem_set(xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.baseAdress, 0U,
                          xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.amountOfMemory);

#if defined(linux)
    (void)munmap(xme_core_dataHandler_dataHandlerInternals.memory.baseAdress, xme_core_dataHandler_dataHandlerInternals.memory.amountOfMemory);
    (void)munmap(xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.baseAdress, xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.amountOfMemory);
#else
    xme_hal_mem_free(xme_core_dataHandler_dataHandlerInternals.memory.baseAdress);
    xme_hal_mem_free(xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.baseAdress);
#endif

    for(i = 0U; i < (uint32_t) xme_core_dataHandler_dataHandlerInternals.dataPacketId; ++i){
        xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[i] = 0U;
        xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[i] = 0U;
    }

    xme_core_dataHandler_dataHandlerInternals.memory.baseAdress = NULL;
    xme_core_dataHandler_dataHandlerInternals.memory.maxAdress = NULL;
    xme_core_dataHandler_dataHandlerInternals.memory.freeBytes = 0U;
    xme_core_dataHandler_dataHandlerInternals.memory.offset = 0U;
    xme_core_dataHandler_dataHandlerInternals.memory.amountOfMemory = 0U;

    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.baseAdress = NULL;
    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.maxAdress = NULL;
    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.freeBytes = 0U;
    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.offset = 0U;
    xme_core_dataHandler_dataHandlerInternals.memoryOfShadow.amountOfMemory = 0U;
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_initializeDataStructure(xme_core_dataHandler_imageDataStructure_t image) {
    XME_UNUSED_PARAMETER(image);
    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_checkImageForConsistancy(xme_core_dataHandler_imageDataStructure_t image) {
    XME_UNUSED_PARAMETER(image);
    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_checkInternalConistency(void) {
    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_createDataEntry(xme_core_dataHandler_setup_t const * const value,
                xme_core_dataManager_dataPacketId_t * const port) {
    bool workToDo = true;
    xme_status_t status = XME_STATUS_INTERNAL_ERROR;
    //XME_ASSERT_RVAL((xme_core_dataHandler_dataHandlerInternals.initialized == true), status);
    assert(xme_core_dataHandler_dataHandlerInternals.initialized == (bool) true);

    if (xme_core_dataHandler_dataHandlerInternals.initFromFileStorage) {
        //dataHandler_dataStructure_t *portFound = NULL;

        // first we have to check if port was in the given storage file
        // this will not work correct, because
        //portFound = findPortInInternalDataStructure(*value->);
        //if (portFound != NULL)
        //{
        //  workToDo = false;
        //}
    }

    if (workToDo) {
        xme_core_dataHandler_dataStructure_t *dataElement, *shadowElement;
        status = XME_STATUS_OUT_OF_RESOURCES;

        dataElement = allocateSpace(value, &xme_core_dataHandler_dataHandlerInternals.memory);
        //XME_ASSERT_RVAL((NULL != dataElement), status);
        assert(NULL != dataElement);
        status = fillElementWithValues(dataElement, value);
        //XME_ASSERT_RVAL(status == XME_STATUS_SUCCESS, status);
        assert(status == XME_STATUS_SUCCESS);

        // the same has to be done with our shadow table
        status = XME_STATUS_OUT_OF_RESOURCES;
        shadowElement = allocateSpace(value, &xme_core_dataHandler_dataHandlerInternals.memoryOfShadow);
        //XME_ASSERT_RVAL((NULL != shadowElement), status);
        assert(NULL != shadowElement);
        status = fillElementWithValues(shadowElement, value);
        //XME_ASSERT_RVAL(status == XME_STATUS_SUCCESS, status);
        assert(status == XME_STATUS_SUCCESS);

        // now setup the rest of our internal world.
        xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[xme_core_dataHandler_dataHandlerInternals.dataPacketId] = (uintptr_t) dataElement;
        xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[xme_core_dataHandler_dataHandlerInternals.dataPacketId] = (uintptr_t) shadowElement;
        ++xme_core_dataHandler_dataHandlerInternals.dataPacketId;
        *port = dataElement->dataEntry.port;

        status = XME_STATUS_SUCCESS;
    }
    return status;
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_readDataWithDirective(xme_core_dataManager_dataPacketId_t port, uint32_t offset,
                         void * const buffer, uint32_t bufferSize,
                         uint32_t * const bytesRead, xme_core_dataHandler_states_t directive) {
    switch (directive) {
        case DATAHANDLER_STATE_INVALID:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_AUDITELEMENT:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_WRITE_AUDITELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SECURITYATTRIBUTE:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_WRITE_SECURITYATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SAFETYATTRIBUTE:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_WRITE_SAFETYATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWELEMENT:
            return readDataWithOffset(port, 0U, buffer, bufferSize, bytesRead, (bool)true);
        case DATAHANDLER_STATE_WRITE_SHADOWELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWELEMENT_WITHOFFSET:
            return readDataWithOffset(port, offset, buffer, bufferSize, bytesRead, (bool)true);
        case DATAHANDLER_STATE_WRITE_SHADOWELEMENT_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWATTRIBUTE_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_TOPICELEMENT:
            return readDataWithOffset(port, 0U, buffer, bufferSize, bytesRead, (bool)false);
        case DATAHANDLER_STATE_WRITE_TOPICELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_TOPICATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_TOPICATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        default:
            return XME_STATUS_INTERNAL_ERROR;
    }
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_readAttributeWithDirective(xme_core_dataManager_dataPacketId_t port,
                              xme_core_attribute_key_t attributeKey, uint32_t offset,
                              void * const buffer, uint32_t bufferSize,
                              uint32_t * const bytesRead, xme_core_dataHandler_states_t directive) {
    switch (directive) {
        case DATAHANDLER_STATE_INVALID:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_AUDITELEMENT:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_WRITE_AUDITELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SECURITYATTRIBUTE:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_WRITE_SECURITYATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SAFETYATTRIBUTE:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_WRITE_SAFETYATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWATTRIBUTE:
            return readAttributeWithOffset(port, attributeKey, 0U, buffer, bufferSize, bytesRead, (bool)true);
        case DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWELEMENT_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWELEMENT_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWATTRIBUTE_WITHOFFSET:
            return readAttributeWithOffset(port, attributeKey, offset, buffer, bufferSize, bytesRead, (bool)true);
        case DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_TOPICELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_TOPICELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_TOPICATTRIBUTE:
            return readAttributeWithOffset(port, attributeKey, 0U, buffer, bufferSize, bytesRead, (bool)false);
        case DATAHANDLER_STATE_WRITE_TOPICATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        default:
            return XME_STATUS_INTERNAL_ERROR;
    }
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_writeDataWithDirective(xme_core_dataManager_dataPacketId_t port, uint32_t offset,
                          void const * const buffer, uint32_t bufferSize,
                          xme_core_dataHandler_states_t directive) {
    switch (directive) {
        case DATAHANDLER_STATE_INVALID:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_AUDITELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_AUDITELEMENT:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_READ_SECURITYATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SECURITYATTRIBUTE:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_READ_SAFETYATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SAFETYATTRIBUTE:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_READ_SHADOWELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWELEMENT:
            return writeDataWithOffset(port, 0U, buffer, bufferSize, (bool)true);
        case DATAHANDLER_STATE_READ_SHADOWATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWELEMENT_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWELEMENT_WITHOFFSET:
            return writeDataWithOffset(port, offset, buffer, bufferSize, (bool)true);
        case DATAHANDLER_STATE_READ_SHADOWATTRIBUTE_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_TOPICELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_TOPICELEMENT:
            return writeDataWithOffset(port, 0U, buffer, bufferSize, (bool)false);
        case DATAHANDLER_STATE_READ_TOPICATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_TOPICATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        default:
            return XME_STATUS_INTERNAL_ERROR;
    }
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_writeAttributeWithDirective(xme_core_dataManager_dataPacketId_t port,
                            xme_core_attribute_key_t attributeKey, uint32_t offset, void const * const buffer,
                            uint32_t bufferSize, xme_core_dataHandler_states_t directive) {
    switch (directive) {
        case DATAHANDLER_STATE_INVALID:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_AUDITELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_AUDITELEMENT:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_READ_SECURITYATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SECURITYATTRIBUTE:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_READ_SAFETYATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SAFETYATTRIBUTE:
            return XME_STATUS_UNSUPPORTED;
        case DATAHANDLER_STATE_READ_SHADOWELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE:
            return writeAttributeWithOffset(port, attributeKey, 0U, buffer, bufferSize, (bool)true);
        case DATAHANDLER_STATE_READ_SHADOWELEMENT_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWELEMENT_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_SHADOWATTRIBUTE_WITHOFFSET:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE_WITHOFFSET:
            return writeAttributeWithOffset(port, attributeKey, offset, buffer, bufferSize, (bool)true);
        case DATAHANDLER_STATE_READ_TOPICELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_TOPICELEMENT:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_READ_TOPICATTRIBUTE:
            return XME_STATUS_INTERNAL_ERROR;
        case DATAHANDLER_STATE_WRITE_TOPICATTRIBUTE:
            return writeAttributeWithOffset(port, attributeKey, 0U, buffer, bufferSize, (bool)false);
        default:
            return XME_STATUS_INTERNAL_ERROR;
    }
}

//******************************************************************************//
xme_core_dataHandler_attributeElement_t *
xme_core_dataHandler_findAttribute(xme_core_dataManager_dataPacketId_t port,
                                        xme_core_attribute_key_t attributeKey, bool useShadow) {
    xme_core_dataHandler_attributeElement_t *found = NULL;
    xme_core_dataHandler_dataStructure_t * node;
    register uint32_t i;

    if (useShadow) {
        node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
    } else {
        node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
    }
//    XME_ASSERT_RVAL(NULL != node, found);
    assert(NULL != node);

    for (i = 0U; i < node->dataEntry.attributeElements; ++i) {
        if (attributeKey == node->dataEntry.attributeElement[i].key) {
            found = &node->dataEntry.attributeElement[i];
        }
    }
    return found;
}

//******************************************************************************//
xme_status_t
xme_core_dataHandler_checkDataStorageForImage(void) {
    return XME_STATUS_INTERNAL_ERROR;
}

//******************************************************************************//
xme_core_dataHandler_imageDataStructure_t
xme_core_dataHandler_loadImage(void) {
    return NULL;
}

//******************************************************************************//
uint32_t
xme_core_dataHandler_calculateHash(char const * const key, size_t length) {
    register uint32_t i;
    uint32_t hash = 0U;

    XME_CHECK(NULL != key && 0U < length, hash);

    for (i = 0U; i < length; ++i) {
        hash += key[i];
        hash += (hash << 10);
        hash ^= (hash >> 6);
    }
    hash += (hash << 3);
    hash ^= (hash >> 11);
    hash += (hash << 15);
    return hash;
}

//******************************************************************************//
static xme_status_t
readDataWithOffset(xme_core_dataManager_dataPacketId_t port, uint32_t offset,
                     void * const buffer, uint32_t bufferSize,
                     uint32_t * const bytesRead, bool readFromShadow) {
    xme_status_t status = XME_STATUS_INVALID_PARAMETER;
    xme_core_dataHandler_dataStructure_t *node;
    xme_core_dataHandler_dataPacket_t *element;
    size_t size;

/*
    XME_ASSERT_RVAL((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port) &&
                    (NULL != buffer) && (NULL != bytesRead), status);
*/
    assert((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port) &&
           (NULL != buffer) && (NULL != bytesRead));


    status = xme_core_dataHandler_checkPort(port);
    //XME_ASSERT_RVAL(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    assert(XME_STATUS_SUCCESS == status);

    // this is a fix for a problem in shadow element, putting this code into find element won't fix it. todo: investigate it
    if (readFromShadow && 1U != port) {
        node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
    } else {
        node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
        if(node->shadowEntry.useShadowEntry && 1U != port){
            node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
        } else {
            node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
        }
    }
    //XME_ASSERT_RVAL(NULL != node, XME_STATUS_INVALID_PARAMETER);
    assert(NULL != node);

    status = checkDataEntry(node, readFromShadow, readOperation);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

#if defined(USE_QUEUE)
    element = (dataHandler_dataPacket_t *) node->dataEntry.current;
#else
    element = &node->dataEntry;
#endif
    XME_CHECK(element->dataElement.size > 0u, XME_STATUS_NO_SUCH_VALUE);

    status = checkChecksum(element->dataElement.checksum, element->dataElement.data, element->dataElement.size);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    size = XME_HAL_MATH_MIN(element->dataElement.size-offset, (size_t) bufferSize);
    status = checkCanary((void*)(((uintptr_t)element->dataElement.data) + offset), size);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    (void) xme_hal_mem_set(buffer, 0u, (size_t) bufferSize);
    (void) xme_hal_mem_copy(buffer, (const void*)(((uintptr_t)element->dataElement.data) + offset), size);
    *bytesRead = (uint32_t) size;

    return status;
}

//******************************************************************************//
static xme_status_t
readAttributeWithOffset(xme_core_dataManager_dataPacketId_t port, xme_core_attribute_key_t attributeKey,
                           uint32_t offset, void * const buffer, uint32_t bufferSize,
                           uint32_t * const bytesRead, bool readFromShadow) {
    xme_status_t status = XME_STATUS_INVALID_PARAMETER;
    xme_core_dataHandler_dataStructure_t * node;
    xme_core_dataHandler_attributeElement_t * element;
    size_t size;

    /*XME_ASSERT_RVAL((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port) && (NULL != buffer) && (NULL != bytesRead) &&
                    (0u < bufferSize) && (XME_CORE_ATTRIBUTE_KEY_UNDEFINED != attributeKey), status);
*/
    assert((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port) &&
           (NULL != buffer) && (NULL != bytesRead) && (0u < bufferSize) &&
           (XME_CORE_ATTRIBUTE_KEY_UNDEFINED != attributeKey));

    status = xme_core_dataHandler_checkPort(port);
    //XME_ASSERT_RVAL(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    assert(XME_STATUS_SUCCESS == status);

    // this is a fix for a problem in shadow element, putting this code into find element won't fix it. todo: investigate it
    if (readFromShadow && 1U != port) {
        node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
    } else {
        node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
        if (node->shadowEntry.useShadowEntry && 1U != port) {
            node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
        } else {
            node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
        }
    }
    //XME_ASSERT_RVAL(NULL != node, XME_STATUS_INVALID_PARAMETER);
    assert(NULL != node);

    status = checkDataEntry(node, readFromShadow, readOperation);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    element = xme_core_dataHandler_findAttribute(port, attributeKey, readFromShadow);
    XME_CHECK(NULL != element, XME_STATUS_NO_SUCH_VALUE);

    size = XME_HAL_MATH_MIN(element->size-offset, (size_t) bufferSize);
    status = checkCanary((void*)(((uintptr_t)element->data) + offset), size);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    (void) xme_hal_mem_set(buffer, 0u, (size_t) bufferSize);
    (void) xme_hal_mem_copy(buffer, (const void*)(((uintptr_t)element->data)+offset), size);
    *bytesRead = (uint32_t) size;

    return status;
}

//******************************************************************************//
static xme_status_t
writeDataWithOffset(xme_core_dataManager_dataPacketId_t port, uint32_t offset,
                      void const * const buffer, uint32_t bufferSize, bool writeToShadow) {
    xme_status_t status = XME_STATUS_INVALID_PARAMETER;
    xme_core_dataHandler_dataStructure_t *node;
    xme_core_dataHandler_dataPacket_t *element, *shadowelement;
    size_t size;

    /*XME_ASSERT_RVAL((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port) &&
                    (NULL != buffer) && (0u < bufferSize), status);
*/
    assert((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port) && (NULL != buffer) && (0u < bufferSize));

    status = xme_core_dataHandler_checkPort(port);
    //XME_ASSERT_RVAL(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    assert(XME_STATUS_SUCCESS == status);

    XME_CHECK_MSG(false == xme_core_dataHandler_portIsWriteLocked(port), XME_STATUS_SUCCESS,
                  XME_LOG_WARNING, "Try to write data to a port, which was locked by component wrapper!\n");

    node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
    //XME_ASSERT_RVAL(NULL != node, XME_STATUS_INVALID_PARAMETER);
    assert(NULL != node);

    status = checkDataEntry(node, writeToShadow, writeOperation);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

#if defined(USE_QUEUE)
    element = (dataHandler_dataPacket_t *) node->dataEntry.current;
#else
    element = &node->dataEntry;
#endif
    //XME_ASSERT_RVAL(NULL != element, XME_STATUS_INTERNAL_ERROR);
    XME_CHECK(NULL != element, XME_STATUS_NO_SUCH_VALUE);

    if(!writeToShadow || 1U == port){
        // no call of testserver here
        size = XME_HAL_MATH_MIN(element->dataElement.size-offset, (size_t) bufferSize);
        status = writeToDataElement(&element->dataElement, buffer, offset, size);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }

    if((1U != port) && (writeToShadow || !node->shadowEntry.useShadowEntry)){
        // here we have got either a simple testserver call or a normal write without a lock.
        xme_core_dataHandler_dataStructure_t * const shadow =
                                (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
        //XME_ASSERT_RVAL(NULL != shadow, XME_STATUS_INVALID_PARAMETER);
        XME_CHECK(NULL != shadow, XME_STATUS_NO_SUCH_VALUE);

        status = checkDataEntry(shadow, writeToShadow, writeOperation);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
#if defined(USE_QUEUE)
        shadowelement = (dataHandler_dataPacket_t *) shadow->dataEntry.current;
#else
        shadowelement = &shadow->dataEntry;
#endif
        //XME_ASSERT_RVAL(NULL != shadowelement, XME_STATUS_INTERNAL_ERROR);
        XME_CHECK(NULL != shadow, XME_STATUS_NO_SUCH_VALUE);
        size = XME_HAL_MATH_MIN(shadowelement->dataElement.size-offset, (size_t) bufferSize);
        status = writeToDataElement(&shadowelement->dataElement, buffer, offset, size);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        if (!writeToShadow) {
            if(element->dataElement.checksum != shadowelement->dataElement.checksum){
                return XME_STATUS_INTERNAL_ERROR;
            }
        }
    }
    return status;
}

//******************************************************************************//
static xme_status_t
writeAttributeWithOffset(xme_core_dataManager_dataPacketId_t port,
                         xme_core_attribute_key_t attributeKey, uint32_t offset,
                         void const * const buffer, uint32_t bufferSize, bool writeToShadow) {
    xme_status_t status = XME_STATUS_INVALID_PARAMETER;
    xme_core_dataHandler_dataStructure_t * node;
    xme_core_dataHandler_attributeElement_t * element = NULL, * shadowElement;
    size_t size;

    /*XME_ASSERT_RVAL((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port) && (NULL != buffer) &&
                    (0u < bufferSize) && (XME_CORE_ATTRIBUTE_KEY_UNDEFINED != attributeKey), status);*/
    assert((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != port) && (NULL != buffer) &&
           (0u < bufferSize) && (XME_CORE_ATTRIBUTE_KEY_UNDEFINED != attributeKey));

    status = xme_core_dataHandler_checkPort(port);
    //XME_ASSERT_RVAL(XME_STATUS_SUCCESS == status, XME_STATUS_INTERNAL_ERROR);
    assert(XME_STATUS_SUCCESS == status);

    //node = xme_core_dataHandler_findElement(port, (bool)false);
    node = (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
    //XME_ASSERT_RVAL(NULL != node, XME_STATUS_INVALID_PARAMETER);
    assert(NULL != node);

    status = checkDataEntry(node, writeToShadow, writeOperation);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    XME_CHECK_MSG(false == xme_core_dataHandler_portIsWriteLocked(port), XME_STATUS_SUCCESS,
                  XME_LOG_WARNING, "Try to write an attribute to a port, which was locked by component wrapper!\n");

    element = xme_core_dataHandler_findAttribute(port, attributeKey, (bool)false);
    XME_CHECK(NULL != element, XME_STATUS_NO_SUCH_VALUE);
    if(!writeToShadow || 1U == port){
        // no call of testserver here
        size = XME_HAL_MATH_MIN(element->size-offset, (size_t) bufferSize);

        status = writeToAttributeElement(element, buffer, offset, size);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }

    if((1U != port) && (writeToShadow || !node->shadowEntry.useShadowEntry)){
        // testserver call or normal write
        shadowElement = xme_core_dataHandler_findAttribute(port, attributeKey, (bool)true);
        XME_CHECK(NULL != shadowElement, XME_STATUS_NO_SUCH_VALUE);
        size = XME_HAL_MATH_MIN(shadowElement->size-offset, (size_t) bufferSize);

        status = writeToAttributeElement(shadowElement, buffer, offset, size);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
        if (!writeToShadow) {
            if (shadowElement->checksum != element->checksum) {
                return XME_STATUS_INTERNAL_ERROR;
            }
        }
    }
    return status;
}

//******************************************************************************//
static xme_status_t
writeToDataElement(xme_core_dataHandler_dataElement_t * const element,
                     void const * const buffer, size_t offset, size_t size) {
    xme_status_t status = writeToBlob(element->data, buffer, offset, size);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);
    element->checksum = xme_core_dataHandler_calculateHash(element->data, element->size);
    return status;
}

static xme_status_t writeToAttributeElement(xme_core_dataHandler_attributeElement_t * const element,
                                                void const * const buffer, size_t offset, size_t size){
    xme_status_t status = writeToBlob(element->data, buffer, offset, size);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);
    element->checksum = xme_core_dataHandler_calculateHash(element->data, element->size);
    return status;
}

static xme_status_t writeToBlob(void * const data, void const * const buffer, size_t offset, size_t size){
    xme_status_t status;
    status = checkCanary((void*)(((uintptr_t)data) + offset), size);
    XME_CHECK(XME_STATUS_SUCCESS == status, status);

    (void) xme_hal_mem_copy((void*)(((uintptr_t)data)+offset), buffer, size);
    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
static xme_status_t
checkCanary(void const * const buffer, size_t size){
    register uint32_t i;
    // check here if we do a memory violation with the copy operation
    for (i = 0U; i < size/sizeof(uintptr_t); ++i) {
        XME_CHECK(*((uint32_t* )((uintptr_t)buffer + i)) != canary, XME_STATUS_INVALID_PARAMETER);
    }
    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
static xme_status_t
checkChecksum(uint32_t checksum, void const * const buffer, size_t size) {
    uint32_t calculatedHash = xme_core_dataHandler_calculateHash(buffer, size);
    if (calculatedHash == checksum) {
        return XME_STATUS_SUCCESS;
    } else {
        return XME_STATUS_INTERNAL_ERROR;
    }
}

//******************************************************************************//
static xme_core_dataHandler_dataStructure_t *
allocateSpace(xme_core_dataHandler_setup_t const * const value, xme_core_dataHandler_buddySystem_t * const memory) {
#if defined(USE_QUEUE)
    register int32_t i;
#endif
    register uint32_t j;

    // TODO: REFACTOR: 3 parts, one for the memory calculation, one for the filling, one for the rest => FKT.
    size_t __neededMemory;
    volatile uintptr_t __start;
    volatile uintptr_t __end;
    volatile uintptr_t __walk;
    xme_core_dataHandler_dataStructure_t * __element;

    assert(memory->baseAdress != NULL);
    __start = (uintptr_t) (((uintptr_t) memory->baseAdress) + memory->offset);

    // first calculated the needed amount of memory
    __neededMemory = calculateNeededMemory(value);

#if defined(LOG_MEMORY_FOOTPRINT)
    XME_LOG(XME_LOG_DEBUG,
            "neededMemory = %u; dataHandler_dataStructure_t = %u; dataHandler_infoStructure_t = %u; "
#if defined(USE_QUEUE)
            "dataHandler_dataQueue_t = %u; "
#endif
            "dataHandler_dataPacket_t = %u; dataHandler_dataElement_t = %u; dataHandler_attributeElement_t=%u\n",
            __neededMemory,
            sizeof(dataHandler_dataStructure_t), sizeof(dataHandler_infoStructure_t),
#if defined(USE_QUEUE)
            sizeof(dataHandler_dataQueue_t),
#endif
            sizeof(dataHandler_dataPacket_t),
            sizeof(dataHandler_dataElement_t), sizeof(dataHandler_attributeElement_t));
#endif

    // now check everything on consistency
    /*XME_ASSERT_RVAL((xme_core_dataHandler_dataHandlerInternals.memory.freeBytes >= __neededMemory) &&
                    ((uintptr_t) NULL != __start) && ((uintptr_t) NULL != (__start + __neededMemory)) &&
                    (((ptrdiff_t) 0) <= (ptrdiff_t)((uintptr_t)xme_core_dataHandler_dataHandlerInternals.memory.maxAdress) - (ptrdiff_t)(__start + __neededMemory)),
                    NULL);*/
    assert((memory->freeBytes >= __neededMemory) &&
           ((uintptr_t) NULL != __start) && ((uintptr_t) NULL != (__start + __neededMemory)) &&
           (((ptrdiff_t) 0) <= (ptrdiff_t)((uintptr_t)memory->maxAdress) - (ptrdiff_t)(__start + __neededMemory)));

#if defined(LOG_MEMORY_FOOTPRINT)
    XME_LOG(XME_LOG_DEBUG, "\nGenerate structure layout for port=%u with %u:\n", xme_core_dataHandler_dataHandlerInternals.dataPacketId, __neededMemory);
#endif

    // give each element its memory area
    __element = (xme_core_dataHandler_dataStructure_t *) __start;
    __end = __start + sizeof(xme_core_dataHandler_dataStructure_t);

#if defined(LOG_MEMORY_FOOTPRINT)
    XME_LOG(XME_LOG_DEBUG, "dataElement: start=%p end=%p\n", __start, __end);
#endif

#if defined(USE_QUEUE)
    if(0U != value->queueSize){
#endif
        __start = ++__end;
#if defined(USE_QUEUE)
        __element->dataEntry.dataElement = (dataHandler_dataPacket_t *) __start;
        __end = __start + sizeof(dataHandler_dataPacket_t) * value->queueSize;
#else
        __element->dataEntry.dataElement.data = (void*) __start;
        __end = __start + (size_t) value->bufferSize;
#endif

#if defined(LOG_MEMORY_FOOTPRINT)
        XME_LOG(XME_LOG_DEBUG, "New data element: start=%p end=%p\n", __start, __end);
#endif

#if defined(USE_QUEUE)
        for(i = 0; i < value->queueSize; ++i)
        {
            __start = ++__end;
            __element->dataEntry.dataElement[i].dataElement.data = __start;
            __end = __start + value->bufferSize;

#if defined(LOG_MEMORY_FOOTPRINT)
            XME_LOG(XME_LOG_DEBUG, "dataElement->dateEntry.dataElement[%u].dataElement.data: start=%p end=%p\n", i, __start, __end);
#endif
#endif
            if (0U != value->metadata.length) {
                __start = ++__end;
#if defined(USE_QUEUE)
                __element->dataEntry.dataElement[i].attributeElement = (dataHandler_attributeElement_t*) __start;
                __end = __start + sizeof(dataHandler_attributeElement_t) * value->metadata.length;
#if defined(LOG_MEMORY_FOOTPRINT)
                XME_LOG(XME_LOG_DEBUG, "dataElement->dataEntry.dataElement[%u].attributeElement: start=%p end=%p\n", i, __start, __end);
#endif
#else
                __element->dataEntry.attributeElement = (xme_core_dataHandler_attributeElement_t*) __start;
                __end = __start + sizeof(xme_core_dataHandler_attributeElement_t) * value->metadata.length;
#if defined(LOG_MEMORY_FOOTPRINT)
                XME_LOG(XME_LOG_DEBUG, "dataElement->attributeElement: start=%p end=%p\n", __start, __end);
#endif
#endif

                for (j = 0U; j < value->metadata.length; ++j) {
                    __start = ++__end;
#if defined(USE_QUEUE)
                    __element->dataEntry.dataElement[i].attributeElement[j].data = __start;
                    __end = __start + value->metadata.element[j].size;
#if defined(LOG_MEMORY_FOOTPRINT)
                    XME_LOG(XME_LOG_DEBUG, "dataElement->dataEntry.dataElement[%u].attributeElement[%u].data: start=%p end=%p\n", i, j, __start, __end);
#endif
#else
                    __element->dataEntry.attributeElement[j].data = (void*) __start;
                    __end = __start + value->metadata.element[j].size;
#if defined(LOG_MEMORY_FOOTPRINT)
                    XME_LOG(XME_LOG_DEBUG, "dataElement->attributeElement[%u].data: start=%p end=%p\n", j, __start, __end);
#endif
#endif
                    }
                }
#if defined(USE_QUEUE)
        }
    }
#endif

    // fill the gap between the needed amount of our data structure and the aligned memory
    __walk = (uintptr_t) __end;
    __end = ((uintptr_t) memory->baseAdress + memory->offset) + __neededMemory;
    while (__walk <= (uintptr_t) __end) {
        *(uintptr_t*)__walk = canary;
        __walk++;
    }

    //we add a gap of 8 bytes as an array boarder between two slices of memory!
    memory->offset += (__neededMemory + 8);
    memory->freeBytes -= (__neededMemory + 8);

    // fill the gap with our canary
    __walk = (uintptr_t) __end;
    __end = (uintptr_t) memory->baseAdress + memory->offset;
    while (__walk != (uintptr_t) __end) {
        *(uintptr_t*)__walk = canary;
        __walk++;
    }

#if defined(LOG_MEMORY_FOOTPRINT)
    XME_LOG(XME_LOG_DEBUG, "next dataElement will be at=%p\n", (uintptr_t)memory->baseAdress + memory->offset);
    XME_LOG(XME_LOG_DEBUG, "used memory of=%u ; remaining=%u\n", __neededMemory, memory->freeBytes);
#endif

    return __element;
}

static size_t calculateNeededMemory(xme_core_dataHandler_setup_t const * const value){
#if defined(USE_QUEUE)
    register size_t i;
#endif
    register size_t j;

    // FIXME: recheck this calculation, especially in RACE we have got a port 15
    //         this port is calculated with __neededMemory = 128 without ceil 126
    //         but __end - __start = 130, it seems that with all the __start = ++__end
    //         we increase to much. so this has to be rechecked!
    size_t __neededMemory = sizeof(xme_core_dataHandler_dataStructure_t);
    ++__neededMemory;

#if defined(USE_QUEUE)
    if (0U != value->queueSize) {
        __neededMemory += sizeof(xme_core_dataHandler_dataPacket_t) * value->queueSize;
#else
        __neededMemory += sizeof(xme_core_dataHandler_dataElement_t);
#endif
        ++__neededMemory;
#if defined(USE_QUEUE)
        for(i = 0; i < value->queueSize; ++i)
        {
#endif
            __neededMemory += value->bufferSize;
            ++__neededMemory;
            if (value->metadata.length != 0u) {
                __neededMemory += sizeof(xme_core_dataHandler_attributeElement_t) * value->metadata.length;
                ++__neededMemory;
                for (j = 0u; j < value->metadata.length; ++j) {
                    __neededMemory += value->metadata.element[j].size;
                    ++__neededMemory;
                }
            }
#if defined(USE_QUEUE)
        }
    }
#endif
    __neededMemory = xme_hal_math_ceilPowerOfTwo((uint32_t) __neededMemory);
    return __neededMemory;
}

//******************************************************************************//
static xme_status_t
fillElementWithValues(xme_core_dataHandler_dataStructure_t * const element,
                      xme_core_dataHandler_setup_t const * const value) {
#if defined(USE_QUEUE)
    register int32_t i;
#endif
    register uint32_t j;

    // fill the info part of our data structure
    element->info.writeLock = false;
    element->info.persistent = value->persistent;
    element->info.overwrite = value->overwrite;

    // fill the shadow part of our data structure
    element->shadowEntry.lockedByTestSystem = false;
    element->shadowEntry.useShadowEntry = false;

    // fill the data specific entries for our data structure
    element->dataEntry.type = value->type;
    element->dataEntry.port = xme_core_dataHandler_dataHandlerInternals.dataPacketId;
    element->dataEntry.topic = value->topic;

#if defined(USE_QUEUE)
    element->dataEntry.elements = value->queueSize;
    element->dataEntry.start = &element->dataEntry.dataElement[0];
    element->dataEntry.current = &element->dataEntry.dataElement[0];
    element->dataEntry.last = &element->dataEntry.dataElement[0];
    element->dataEntry.end = &element->dataEntry.dataElement[element->dataEntry.elements - 1];

    for (i = 0; i < value->queueSize; ++i) {
        element->dataEntry.dataElement[i].dataElement.checksum = 0U;
        element->dataEntry.dataElement[i].dataElement.size = (size_t) value->bufferSize;
#else
        element->dataEntry.dataElement.checksum = 0U;
        element->dataEntry.dataElement.size = (size_t) value->bufferSize;
#endif
        // fill the metadata
        if (value->metadata.length != 0U) {
            for (j = 0u; j < value->metadata.length; ++j) {
#if defined(USE_QUEUE)
                element->dataEntry.dataElement[i].attributeElement[j].checksum = 0U;
                element->dataEntry.dataElement[i].attributeElement[j].key = value->metadata.element[j].key;
                element->dataEntry.dataElement[i].attributeElement[j].size = value->metadata.element[j].size;
#else
                element->dataEntry.attributeElement[j].checksum = 0U;
                element->dataEntry.attributeElement[j].key = value->metadata.element[j].key;
                element->dataEntry.attributeElement[j].size = value->metadata.element[j].size;
#endif
            }
#if defined(USE_QUEUE)
            element->dataEntry.dataElement[i].attributeElements = value->metadata.length;
#else
            element->dataEntry.attributeElements = value->metadata.length;
#endif
        }
#if defined(USE_QUEUE)
    }
#endif
    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
//static availableMemoryPool()
//    {
//
//    }

//******************************************************************************//
//static releaseMemory();

//******************************************************************************//
//static getMemory();

//******************************************************************************//
static INLINE xme_status_t
checkDataEntry(xme_core_dataHandler_dataStructure_t const * const dataEntry, bool useShadow, xme_core_dataHandler_operation_t operation){
    xme_status_t status;
    switch(operation)
    {
        case readOperation:
            if ((useShadow && (dataEntry->dataEntry.type == XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION))
                || (dataEntry->dataEntry.type == XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION)
                || (dataEntry->dataEntry.type == XME_CORE_COMPONENT_PORTTYPE_INTERNAL)) {
                status = XME_STATUS_SUCCESS;
            } else {
                status = XME_STATUS_PERMISSION_DENIED;
            }
            break;
        case writeOperation:
            if((useShadow && (dataEntry->dataEntry.type == XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION))
                || (dataEntry->dataEntry.type == XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION)
                || (dataEntry->dataEntry.type == XME_CORE_COMPONENT_PORTTYPE_INTERNAL)) {
                status = XME_STATUS_SUCCESS;
            }
            else
            {
                status = XME_STATUS_PERMISSION_DENIED;
            }
            break;
        case invalidOperation:
            // fall through
        default:
            status = XME_STATUS_PERMISSION_DENIED;
            break;
    }
    return status;
}

//******************************************************************************//
static INLINE bool xme_core_dataHandler_portIsWriteLocked(xme_core_dataManager_dataPacketId_t port){
    XME_CHECK(NULL != (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port], false);
    return ((xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port])->info.writeLock;
}
